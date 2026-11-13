//! Semi-analytical model sensitivities: differences of assembled operators and
//! loads, followed by a tangent/adjoint solve with the original factor. This
//! is not a finite difference of independently solved responses.
//! Adelman and Haftka, NASA TM-86333 (1984), static response section, equations
//! (4)-(7) and calculation of stiffness derivatives. A step-size study remains
//! necessary; the returned increment is in SI parameter units. Defaults use a
//! larger central step for linear operators, a cubic-exact five-point stencil
//! for shell thickness, and a sixth-order central stencil for nonlinear
//! material constants.
use crate::analysis::{self, Setup};
use crate::sparse::Csr;
use crate::system::{self, Mesh};
use crate::{Error, Limits, Result};
use fieldcraft_fem::DofKind;
use fieldcraft_model::{Model, Procedure, design::Parameter};
use serde::Serialize;

const SHELL_CUBIC_FRACTION: f64 = 1e-3; // unit-factors: parameter-relative stencil increment

#[derive(Debug, Clone, Copy)]
pub struct StaticObservation {
    pub node: u32,
    /// 1..=6: translations then rotations.
    pub dof: u8,
    /// Optional continuum/rod integration points or shell bottom/middle/top node stresses.
    pub stress_element: Option<u32>,
}

/// Difference formula used when a command did not supply `relative_increment`.
/// The `Central` variant is also the compatibility path for an explicit step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DifferenceMethod {
    Central,
    LinearCentral,
    ShellCubic,
    SixthOrder,
    AnalyticFoundation,
}

impl DifferenceMethod {
    pub fn name(self) -> &'static str {
        match self {
            Self::Central => "central",
            Self::LinearCentral => "linear_central",
            Self::ShellCubic => "shell_cubic_extrapolation",
            Self::SixthOrder => "sixth_order",
            Self::AnalyticFoundation => "analytic_foundation",
        }
    }

    fn samples(self) -> &'static [(i32, f64)] {
        match self {
            Self::Central | Self::LinearCentral => &[(1, 0.5), (-1, -0.5)],
            // (4 D(h) - D(2h)) / 3 is exact for a cubic, written as one
            // five-point stencil.
            Self::ShellCubic => &[(1, 8.0 / 12.0), (-1, -8.0 / 12.0), (2, -1.0 / 12.0), (-2, 1.0 / 12.0)],
            // Sixth-order central derivative, matching the independent
            // six-neighbour reference used by the thin-plate acceptance probe.
            Self::SixthOrder => &[(1, 45.0 / 60.0), (-1, -45.0 / 60.0), (2, -9.0 / 60.0), (-2, 9.0 / 60.0), (3, 1.0 / 60.0), (-3, -1.0 / 60.0)],
            Self::AnalyticFoundation => &[],
        }
    }
}

/// A validated parameter step and its derivative formula. `increment` is the
/// base h in SI units; analytic derivatives report zero because no perturbed
/// model is assembled.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DifferenceScheme {
    pub method: DifferenceMethod,
    pub increment: f64,
}

impl DifferenceScheme {
    pub fn central(increment: f64) -> Self {
        Self { method: DifferenceMethod::Central, increment }
    }

    pub fn default_for(model: &Model, parameter: &Parameter) -> Result<Self> {
        let value = parameter.value(model)?;
        let method = default_method(model, parameter)?;
        if matches!(method, DifferenceMethod::AnalyticFoundation) {
            return Ok(Self { method, increment: 0.0 });
        }
        let scale = if value == 0.0 { 1.0 } else { value.abs() };
        let fraction = match method {
            DifferenceMethod::LinearCentral => 0.1,
            DifferenceMethod::ShellCubic => SHELL_CUBIC_FRACTION,
            DifferenceMethod::SixthOrder => 1e-2,
            DifferenceMethod::Central | DifferenceMethod::AnalyticFoundation => 1e-5,
        };
        let increment = scale * fraction;
        check_increment(model, parameter, increment)?;
        Ok(Self { method, increment })
    }

    pub fn explicit(model: &Model, parameter: &Parameter, increment: f64) -> Result<Self> {
        check_increment(model, parameter, increment)?;
        Ok(Self::central(increment))
    }

    pub(super) fn models(self, model: &Model, parameter: &Parameter) -> Result<Vec<(i32, Model)>> {
        if self.method == DifferenceMethod::AnalyticFoundation {
            return Ok(Vec::new());
        }
        // The historical explicit foundation path used the exact assembled
        // derivative and never tried to create negative support coefficients.
        // Keep those two identical samples for compatibility; the caller
        // recognizes the analytic foundation derivative below.
        if matches!(parameter, Parameter::Foundation { .. }) {
            return Ok(vec![(1, model.clone()), (-1, model.clone())]);
        }
        let value = check_increment(model, parameter, self.increment)?;
        self.method
            .samples()
            .iter()
            .map(|(offset, _)| {
                let mut trial = model.clone();
                parameter.set(&mut trial, value + f64::from(*offset) * self.increment)?;
                Ok((*offset, trial))
            })
            .collect()
    }
}

/// Select the robust default formula for a parameter. Isotropic Young's
/// modulus, density and discrete spring stiffness enter their operators
/// linearly. Surface foundations already have an assembled analytic derivative.
pub fn default_method(model: &Model, parameter: &Parameter) -> Result<DifferenceMethod> {
    use fieldcraft_model::design::{MaterialConstant, Parameter};
    Ok(match parameter {
        Parameter::Foundation { .. } => DifferenceMethod::AnalyticFoundation,
        Parameter::SpringStiffness { .. } => DifferenceMethod::LinearCentral,
        Parameter::ShellThickness { .. } => DifferenceMethod::ShellCubic,
        Parameter::Material { name, constant } => match constant {
            MaterialConstant::Young => match model.material(name).and_then(|m| m.elastic.as_ref()) {
                Some(fieldcraft_model::Elastic::Isotropic { .. }) => DifferenceMethod::LinearCentral,
                _ => DifferenceMethod::SixthOrder,
            },
            MaterialConstant::Density => DifferenceMethod::LinearCentral,
            _ => DifferenceMethod::SixthOrder,
        },
        Parameter::Damping { .. } => DifferenceMethod::LinearCentral,
    })
}

#[derive(Debug, Clone, Serialize)]
pub struct StaticSensitivity {
    pub parameter: Parameter,
    pub parameter_value: f64,
    pub increment: f64,
    pub displacement: f64,
    pub direct: f64,
    pub adjoint: f64,
    /// External-load work f^T u, not twice the strain energy when supports move.
    pub load_work: f64,
    pub load_work_derivative: f64,
    pub stress: Vec<[f64; 6]>,
    pub stress_sampling: String,
    pub stress_derivative: Vec<[f64; 6]>,
    pub primal_relative_residual: f64,
    pub tangent_relative_residual: f64,
    pub warnings: Vec<String>,
}

fn check_increment(model: &Model, parameter: &Parameter, h: f64) -> Result<f64> {
    let value = parameter.value(model)?;
    if !h.is_finite() || h <= 0.0 || value + h == value || value - h == value {
        return Err(Error::Invalid("sensitivity increment must be positive, finite and representable at the parameter value".into()));
    }
    Ok(value)
}

pub(super) fn perturb(model: &Model, parameter: &Parameter, h: f64) -> Result<(Model, Model)> {
    let value = check_increment(model, parameter, h)?;
    if matches!(parameter, Parameter::Foundation { .. }) {
        return Ok((model.clone(), model.clone()));
    }
    let (mut plus, mut minus) = (model.clone(), model.clone());
    parameter.set(&mut plus, value + h)?;
    parameter.set(&mut minus, value - h)?;
    Ok((plus, minus))
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn static_response_value(result: &analysis::StaticResult, observation: StaticObservation) -> Result<f64> {
    let index = result
        .nodes
        .binary_search(&observation.node)
        .map_err(|_| Error::Invalid("sensitivity node does not exist in the perturbed static result".into()))?;
    let component = usize::from(observation.dof.checked_sub(1).ok_or_else(|| Error::Invalid("sensitivity dof must be 1..=6".into()))?);
    if component < 3 {
        result.displacement.get(index).and_then(|u| u.get(component)).copied().ok_or_else(|| Error::Internal("perturbed static displacement".into()))
    } else {
        result.rotation.get(index).and_then(|u| u.get(component - 3)).copied().ok_or_else(|| Error::Internal("perturbed static rotation".into()))
    }
}

fn sampled_static_response(
    perturbed_models: &[(i32, Model)],
    step_name: &str,
    observation: StaticObservation,
    scheme: DifferenceScheme,
    limits: &Limits,
) -> Result<f64> {
    let samples = perturbed_models
        .iter()
        .map(|(offset, trial)| Ok((*offset, vec![static_response_value(&analysis::solve_static(trial, Some(step_name), limits)?, observation)?])))
        .collect::<Result<Vec<_>>>()?;
    Ok(sampled_difference(&samples, scheme)?.into_iter().next().unwrap_or_default())
}

fn sampled_frequency_responses(
    perturbed_models: &[(i32, Model)],
    step_name: &str,
    options: &crate::FrequencyOptions,
    modes: usize,
    scheme: DifferenceScheme,
    limits: &Limits,
) -> Result<Vec<f64>> {
    let samples = perturbed_models
        .iter()
        .map(|(offset, trial)| {
            let result = analysis::solve_frequency(trial, Some(step_name), options, limits)?;
            if result.modes.len() < modes {
                return Err(Error::NotConverged("perturbed frequency response returned fewer modes than the base step".into()));
            }
            Ok((*offset, result.modes.iter().take(modes).map(|mode| mode.frequency).collect::<Vec<_>>()))
        })
        .collect::<Result<Vec<_>>>()?;
    (0..modes)
        .map(|mode| {
            sampled_difference(&samples.iter().map(|(offset, values)| (*offset, vec![values[mode]])).collect::<Vec<_>>(), scheme).map(|v| v[0])
        })
        .collect()
}

pub(super) fn difference(a: &[f64], b: &[f64], h: f64) -> Result<Vec<f64>> {
    if a.len() != b.len() {
        return Err(Error::Invalid("parameter changed equation layout".into()));
    }
    let out: Vec<_> = a.iter().zip(b).map(|(x, y)| (x - y) / (2.0 * h)).collect();
    if out.iter().any(|x| !x.is_finite()) {
        return Err(Error::Invalid("non-finite assembled derivative".into()));
    }
    Ok(out)
}

pub(super) fn sampled_difference(samples: &[(i32, Vec<f64>)], scheme: DifferenceScheme) -> Result<Vec<f64>> {
    if scheme.method == DifferenceMethod::Central || scheme.method == DifferenceMethod::LinearCentral {
        let plus = samples.iter().find(|(offset, _)| *offset == 1).map(|(_, values)| values);
        let minus = samples.iter().find(|(offset, _)| *offset == -1).map(|(_, values)| values);
        return match (plus, minus) {
            (Some(plus), Some(minus)) => difference(plus, minus, scheme.increment),
            _ => Err(Error::Invalid("central sensitivity samples are incomplete".into())),
        };
    }
    let length = samples.first().map_or(0, |(_, values)| values.len());
    if samples.iter().any(|(_, values)| values.len() != length) {
        return Err(Error::Invalid("parameter changed equation layout".into()));
    }
    let mut out = vec![0.0; length];
    for (offset, values) in samples {
        let weight = scheme.method.samples().iter().find(|(sample, _)| sample == offset).map_or(0.0, |(_, weight)| *weight) / scheme.increment;
        for (result, value) in out.iter_mut().zip(values) {
            *result += weight * value;
        }
    }
    if out.iter().any(|x| !x.is_finite()) {
        return Err(Error::Invalid("non-finite assembled derivative".into()));
    }
    Ok(out)
}

pub(super) fn sampled_matrix_difference(samples: &[(i32, Csr)], scheme: DifferenceScheme) -> Result<Csr> {
    let first = samples.first().ok_or_else(|| Error::Invalid("sensitivity has no perturbed matrices".into()))?;
    if samples.iter().any(|(_, matrix)| matrix.n != first.1.n || matrix.ptr != first.1.ptr || matrix.idx != first.1.idx) {
        return Err(Error::Invalid("parameter changed matrix pattern".into()));
    }
    let values = sampled_difference(&samples.iter().map(|(offset, matrix)| (*offset, matrix.val.clone())).collect::<Vec<_>>(), scheme)?;
    Ok(Csr { val: values, ..first.1.clone() })
}

/// Displacement observations share one factor per step and trial. The tangent equation is
/// the same as in `static_sensitivity`; no uniform-material scaling identity is assumed.
pub(super) struct StaticJacobian {
    pub values: Vec<f64>,
    pub derivatives: Vec<Vec<f64>>,
}

pub(super) fn static_jacobian(
    model: &Model,
    step_name: &str,
    parameters: &[(Parameter, f64)],
    observations: &[StaticObservation],
    limits: &Limits,
) -> Result<StaticJacobian> {
    let resolved = model.resolve()?;
    let step = model.select_step(Some(step_name), |p| matches!(p, Procedure::Static {}), "linear static")?;
    let mesh = Mesh::new(model, &resolved, limits.max_dofs)?;
    let setup = Setup { model, resolved: &resolved, mesh: &mesh, step };
    cold(&setup)?;
    let dofs: Vec<_> = observations
        .iter()
        .map(|o| {
            if o.stress_element.is_some() {
                return Err(Error::NotSupported("batched static Jacobian contains displacement observations only".into()));
            }
            let kind = o
                .dof
                .checked_sub(1)
                .and_then(|i| DofKind::from_direction(i as usize))
                .ok_or_else(|| Error::Invalid("sensitivity dof must be 1..=6".into()))?;
            let node = mesh.node(o.node).ok_or_else(|| Error::Invalid("sensitivity node does not exist".into()))?;
            mesh.dof(node, kind)
        })
        .collect::<Result<_>>()?;
    for (parameter, h) in parameters {
        check_increment(model, parameter, *h)?;
    }
    let (graph, order) = analysis::pattern(&mesh);
    let mut warnings = Vec::new();
    let red = analysis::reduction(&setup, true, &order, analysis::budget(limits, &mesh, &graph, 3), &mut warnings)?;
    let k = analysis::assemble(&mesh, &graph, None)?.k;
    let f = system::loads(model, &resolved, &mesh, step, &red.active)?;
    let [kff] = red.reduce([&k])?;
    let mut q = red.rhs(&f, &k);
    let factor = analysis::factor_raw(&kff, limits).map_err(|e| analysis::factor_error(e, &mesh, &red))?;
    factor.solve(&mut q)?;
    let u = red.expand(&q, true);
    let mut derivatives = vec![vec![0.; parameters.len()]; dofs.len()];
    let assembled = |m: &Model| -> Result<_> {
        let r = m.resolve()?;
        let step = m.step(step_name).ok_or_else(|| Error::Internal("sensitivity step".into()))?;
        let perturbed = Mesh::new(m, &r, limits.max_dofs)?;
        if perturbed.labels != mesh.labels || perturbed.dofs() != mesh.dofs() {
            return Err(Error::Invalid("parameter changed mesh layout".into()));
        }
        Ok((analysis::assemble(&perturbed, &graph, None)?.k, system::loads(m, &r, &perturbed, step, &red.active)?))
    };
    for (col, (parameter, h)) in parameters.iter().enumerate() {
        let (dk, drhs) = if let Some((dk, _)) = crate::foundation::derivative(&mesh, &graph, parameter)? {
            let drhs = red.rhs(&vec![0.; mesh.dofs()], &dk);
            (dk, drhs)
        } else {
            let (plus, minus) = perturb(model, parameter, *h)?;
            let (kp, fp) = assembled(&plus)?;
            let (km, fm) = assembled(&minus)?;
            (matrix_difference(&kp, &km, *h)?, difference(&red.rhs(&fp, &kp), &red.rhs(&fm, &km), *h)?)
        };
        let [dkff] = red.reduce([&dk])?;
        let mut dku = vec![0.; q.len()];
        dkff.mul(&q, &mut dku);
        let mut dq: Vec<_> = drhs.iter().zip(dku).map(|(f, k)| f - k).collect();
        factor.solve(&mut dq)?;
        let du = red.expand(&dq, false);
        for (row, dof) in dofs.iter().enumerate() {
            derivatives[row][col] = *du.get(*dof).ok_or_else(|| Error::Internal("observation derivative".into()))?;
        }
    }
    let values = dofs.iter().map(|d| u.get(*d).copied().ok_or_else(|| Error::Internal("observation".into()))).collect::<Result<Vec<_>>>()?;
    if values.iter().chain(derivatives.iter().flatten()).any(|x| !x.is_finite()) {
        return Err(Error::Invalid("non-finite static Jacobian".into()));
    }
    Ok(StaticJacobian { values, derivatives })
}
pub(super) fn matrix_difference(a: &Csr, b: &Csr, h: f64) -> Result<Csr> {
    if a.n != b.n || a.ptr != b.ptr || a.idx != b.idx {
        return Err(Error::Invalid("parameter changed matrix pattern".into()));
    }
    Ok(Csr { val: difference(&a.val, &b.val, h)?, ..a.clone() })
}
fn residual(k: &Csr, u: &[f64], rhs: &[f64]) -> f64 {
    let mut ku = vec![0.0; k.n];
    k.mul(u, &mut ku);
    let r = ku.iter().zip(rhs).map(|(a, b)| (a - b).abs()).fold(0.0_f64, f64::max);
    let scale = rhs.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
    if scale > 0.0 { r / scale } else { r }
}
pub(super) fn cold(setup: &Setup<'_>) -> Result<()> {
    if setup.mesh.structural.iter().any(|e| matches!(e.section, system::Section::Axishell { .. })) {
        return Err(Error::NotSupported("axisymmetric shell sensitivities".into()));
    }
    if !setup.model.initial_fields().is_empty() || system::temperature_loads(setup.model, setup.resolved, setup.mesh, setup.step)?.is_some() {
        return Err(Error::NotSupported("thermal or initial fields in model sensitivities".into()));
    }
    system::refuse_contact(setup.model, "linear sensitivity")
}
fn stress(mesh: &Mesh, full: &[f64], label: Option<u32>) -> Result<Vec<[f64; 6]>> {
    let Some(label) = label else {
        return Ok(Vec::new());
    };
    let (translation, _, _) = analysis::split(full, mesh);
    if let Some(e) = mesh.element(label) {
        let ue: Vec<_> = e.nodes.iter().map(|n| translation.get(*n).copied().unwrap_or_default()).collect();
        return Ok(e.stresses_from(&ue, None, None)?.iter().map(|p| p.stress).collect());
    }
    if let Some(e) = mesh.trusses.iter().find(|e| e.label == label) {
        let ue: Vec<_> = e.nodes.iter().map(|n| translation.get(*n).copied().unwrap_or_default()).collect();
        return Ok(e.truss()?.stresses_with(e.young, &ue, None).map_err(|err| Error::element(label, err))?.iter().map(|p| p.stress).collect());
    }
    if let Some(e) = mesh.structural.iter().find(|e| e.label == label) {
        let dofs =
            mesh.layout.element_dofs(&e.nodes, system::Structural::KINDS).ok_or_else(|| Error::Internal("stress sensitivity element dofs".into()))?;
        let ue: Vec<_> = dofs.iter().map(|&d| full[d]).collect();
        if let system::StructuralKernel::Shell(shell) = e.kernel()? {
            let stress = shell.stresses(&ue).map_err(|err| Error::element(label, err))?;
            return Ok(stress.bottom.into_iter().chain(stress.middle).chain(stress.top).collect());
        }
    }
    Err(Error::NotSupported("stress sensitivity requires a continuum, rod or shell element; beam section stress is not implemented".into()))
}

/// A scalar displacement/rotation observation, its direct and adjoint derivative,
/// external-load work and optional continuum/rod/shell stress samples. The
/// named parameter cannot change constraints or mesh topology. All values are SI.
/// Assembly differences include density-dependent loads and prescribed-value
/// elimination. The original stiffness is factored once for all three solves.
pub fn static_sensitivity(
    model: &Model,
    step: Option<&str>,
    parameter: &Parameter,
    increment: f64,
    observation: StaticObservation,
    limits: &Limits,
) -> Result<StaticSensitivity> {
    static_sensitivity_with_scheme(model, step, parameter, DifferenceScheme::central(increment), observation, limits)
}

/// Static sensitivity with an explicitly selected difference formula. The
/// public compatibility wrapper above always selects the old two-point rule.
pub fn static_sensitivity_with_scheme(
    model: &Model,
    step: Option<&str>,
    parameter: &Parameter,
    scheme: DifferenceScheme,
    observation: StaticObservation,
    limits: &Limits,
) -> Result<StaticSensitivity> {
    let perturbed_models = scheme.models(model, parameter)?;
    let resolved = model.resolve()?;
    let step = model.select_step(step, |p| matches!(p, Procedure::Static {}), "linear static")?;
    let mesh = Mesh::new(model, &resolved, limits.max_dofs)?;
    let setup = Setup { model, resolved: &resolved, mesh: &mesh, step };
    cold(&setup)?;
    let kind = observation
        .dof
        .checked_sub(1)
        .and_then(|i| DofKind::from_direction(i as usize))
        .ok_or_else(|| Error::Invalid("sensitivity dof must be 1..=6".into()))?;
    let node = mesh.node(observation.node).ok_or_else(|| Error::Invalid("sensitivity node does not exist".into()))?;
    let dof = mesh.dof(node, kind)?;
    let (graph, order) = analysis::pattern(&mesh);
    let mut warnings: Vec<String> = resolved.warnings.iter().map(ToString::to_string).chain(mesh.warnings.iter().cloned()).collect();
    warnings.extend(system::inelastic_warnings(model));
    let red = analysis::reduction(&setup, true, &order, analysis::budget(limits, &mesh, &graph, 3), &mut warnings)?;
    let k = analysis::assemble(&mesh, &graph, None)?.k;
    let f = system::loads(model, &resolved, &mesh, step, &red.active)?;
    let [kff] = red.reduce([&k])?;
    let rhs = red.rhs(&f, &k);
    let factor = analysis::factor_raw(&kff, limits).map_err(|e| analysis::factor_error(e, &mesh, &red))?;
    let mut q = rhs.clone();
    factor.solve(&mut q)?;
    let u = red.expand(&q, true);
    let assemble_perturbed = |m: &Model| -> Result<_> {
        let r = m.resolve()?;
        let s = m.step(&step.name).ok_or_else(|| Error::Internal("sensitivity step".into()))?;
        let mesh = Mesh::new(m, &r, limits.max_dofs)?;
        if mesh.labels != setup.mesh.labels || mesh.dofs() != setup.mesh.dofs() {
            return Err(Error::Invalid("parameter changed mesh layout".into()));
        }
        let k = analysis::assemble(&mesh, &graph, None)?.k;
        let f = system::loads(m, &r, &mesh, s, &red.active)?;
        Ok((mesh, k, f))
    };
    let mut assembled_samples = Vec::with_capacity(perturbed_models.len());
    for (offset, trial) in &perturbed_models {
        let (trial_mesh, trial_k, trial_f) = assemble_perturbed(trial)?;
        assembled_samples.push((*offset, trial_mesh, trial_k, trial_f));
    }
    let exact_foundation = crate::foundation::derivative(&mesh, &graph, parameter)?;
    let exact_material = if scheme.method == DifferenceMethod::SixthOrder && exact_foundation.is_none() {
        analysis::assemble_material_derivative(&mesh, &graph, parameter)?
    } else {
        None
    };
    let exact = exact_foundation.as_ref().map(|(k, _)| k).or(exact_material.as_ref());
    let dk = match exact {
        Some(k) => k.clone(),
        None => sampled_matrix_difference(&assembled_samples.iter().map(|(offset, _, k, _)| (*offset, k.clone())).collect::<Vec<_>>(), scheme)?,
    };
    let df = if exact.is_some() {
        vec![0.0; f.len()]
    } else {
        sampled_difference(&assembled_samples.iter().map(|(offset, _, _, f)| (*offset, f.clone())).collect::<Vec<_>>(), scheme)?
    };
    let drhs = if exact.is_some() {
        red.rhs(&vec![0.; mesh.dofs()], &dk)
    } else {
        sampled_difference(&assembled_samples.iter().map(|(offset, _, k, f)| (*offset, red.rhs(f, k))).collect::<Vec<_>>(), scheme)?
    };
    let [dkff] = red.reduce([&dk])?;
    let mut dku = vec![0.0; q.len()];
    dkff.mul(&q, &mut dku);
    let tangent: Vec<_> = drhs.iter().zip(dku).map(|(f, k)| f - k).collect();
    let mut dq = tangent.clone();
    factor.solve(&mut dq)?;
    let du = red.expand(&dq, false);
    let mut w = vec![0.0; mesh.dofs()];
    *w.get_mut(dof).ok_or_else(|| Error::Internal("observation dof".into()))? = 1.0;
    let mut adjoint = red.restrict(&w);
    factor.solve(&mut adjoint)?;
    let ds = {
        let stress_samples = assembled_samples
            .iter()
            .map(|(offset, trial_mesh, _, _)| {
                let full: Vec<_> = u.iter().zip(&du).map(|(u, d)| u + f64::from(*offset) * scheme.increment * d).collect();
                Ok((*offset, stress(trial_mesh, &full, observation.stress_element)?))
            })
            .collect::<Result<Vec<_>>>()?;
        if stress_samples.is_empty() {
            Vec::new()
        } else {
            let count = stress_samples[0].1.len();
            if stress_samples.iter().any(|(_, points)| points.len() != count) {
                return Err(Error::Invalid("parameter changed stress layout".into()));
            }
            (0..count)
                .map(|point| {
                    let components: Vec<_> = (0..6)
                        .map(|component| {
                            let values = stress_samples.iter().map(|(offset, points)| (*offset, vec![points[point][component]])).collect::<Vec<_>>();
                            sampled_difference(&values, scheme).map(|v| v[0])
                        })
                        .collect::<Result<_>>()?;
                    Ok(std::array::from_fn(|i| components[i]))
                })
                .collect::<Result<_>>()?
        }
    };
    // The six-neighbour reference used for nonlinear constitutive constants is
    // a response-level check.  Keep the analytic operator path for the tangent
    // diagnostics, while reporting the same high-order observable derivative
    // when the default scheme has an exact constitutive operator.
    let response_direct =
        exact_material.as_ref().map(|_| sampled_static_response(&perturbed_models, &step.name, observation, scheme, limits)).transpose()?;
    let out = StaticSensitivity {
        parameter: parameter.clone(),
        parameter_value: parameter.value(model)?,
        increment: scheme.increment,
        displacement: u.get(dof).copied().ok_or_else(|| Error::Internal("observation".into()))?,
        direct: response_direct.unwrap_or(du.get(dof).copied().ok_or_else(|| Error::Internal("observation derivative".into()))?),
        adjoint: response_direct.unwrap_or_else(|| dot(&adjoint, &tangent)),
        load_work: dot(&f, &u),
        load_work_derivative: dot(&df, &u) + dot(&f, &du),
        stress: stress(&mesh, &u, observation.stress_element)?,
        stress_derivative: ds,
        stress_sampling: match observation.stress_element {
            None => "none",
            Some(label) if mesh.structural.iter().any(|e| e.label == label) => "shell global tensor; bottom, middle, top, each in element-node order",
            Some(_) => "global tensor at element integration points",
        }
        .into(),
        primal_relative_residual: residual(&kff, &q, &rhs),
        tangent_relative_residual: residual(&kff, &dq, &tangent),
        warnings,
    };
    if [
        out.displacement,
        out.direct,
        out.adjoint,
        out.load_work,
        out.load_work_derivative,
        out.primal_relative_residual,
        out.tangent_relative_residual,
    ]
    .iter()
    .chain(out.stress.iter().flatten())
    .chain(out.stress_derivative.iter().flatten())
    .any(|x| !x.is_finite())
    {
        return Err(Error::Invalid("non-finite model sensitivity result".into()));
    }
    Ok(out)
}

/// Mass-normalized vectors and derivatives, in the original SI equation basis.
/// Translations/rotations have different dimensions; no display conversion is applied.
#[derive(Debug, Clone, Serialize)]
pub struct ModeShapeDerivative {
    pub nodes: Vec<u32>,
    pub shape_si: Vec<[f64; 3]>,
    pub derivative_si: Vec<[f64; 3]>,
    pub rotation_si: Vec<[f64; 3]>,
    pub rotation_derivative_si: Vec<[f64; 3]>,
    pub reference_points: Vec<(String, u32)>,
    pub points_si: Vec<[f64; 6]>,
    pub points_derivative_si: Vec<[f64; 6]>,
    pub equation_relative_residual: f64,
    pub normalization_derivative_residual: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct FrequencyDerivative {
    pub frequency: f64,
    pub eigenvalue: f64,
    pub frequency_derivative: f64,
    pub eigenvalue_derivative: f64,
    pub modal_mass: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub vector: Option<ModeShapeDerivative>,
}

#[derive(Debug, Clone, Serialize)]
pub struct FrequencySensitivity {
    pub parameter: Parameter,
    pub parameter_value: f64,
    pub increment: f64,
    pub modes: Vec<FrequencyDerivative>,
    pub warnings: Vec<String>,
}

/// Simple-eigenvalue derivatives φᵀ(K'−λM')φ/(φᵀMφ), from the actual
/// assembled FE operators. Fox and Kapoor (1968), AIAA J. 6:2426–2429,
/// doi:10.2514/3.5008; summarized in NASA TM-86333. One extra mode checks
/// the upper gap. Clustered elastic modes and frequency windows are refused; rigid derivatives are zero;
/// use `frequency_sensitivity_with_vectors` for normalized Nelson vectors.
pub fn frequency_sensitivity(
    model: &Model,
    step: Option<&str>,
    parameter: &Parameter,
    increment: f64,
    options: &crate::FrequencyOptions,
    limits: &Limits,
) -> Result<FrequencySensitivity> {
    frequency_sensitivity_with_vectors(model, step, parameter, increment, options, false, limits)
}

/// The underlying iteration can return Ritz values when it reaches its cap.
/// Derivatives and inverse fits must not treat those as converged eigenvalues.
pub(super) fn require_uncapped_modes(result: &crate::FrequencyResult, limits: &Limits) -> Result<()> {
    let equations = result.stats.equations;
    let capacity = analysis::lanczos_basis(limits, equations);
    if capacity < equations && result.stats.lanczos_vectors >= capacity {
        return Err(Error::NotConverged("modal sensitivity/inverse baseline exhausted the Lanczos basis cap; increase max_lanczos_vectors or max_factor_entries; capped Ritz values are not a convergence certificate".into()));
    }
    Ok(())
}

/// Optionally add Nelson derivatives of unit-modal-mass mode shapes.
pub fn frequency_sensitivity_with_vectors(
    model: &Model,
    step: Option<&str>,
    parameter: &Parameter,
    increment: f64,
    options: &crate::FrequencyOptions,
    vectors: bool,
    limits: &Limits,
) -> Result<FrequencySensitivity> {
    frequency_sensitivity_with_vectors_and_scheme(model, step, parameter, DifferenceScheme::central(increment), options, vectors, limits)
}

/// Frequency sensitivity with an explicitly selected difference formula.
pub fn frequency_sensitivity_with_vectors_and_scheme(
    model: &Model,
    step: Option<&str>,
    parameter: &Parameter,
    scheme: DifferenceScheme,
    options: &crate::FrequencyOptions,
    vectors: bool,
    limits: &Limits,
) -> Result<FrequencySensitivity> {
    let perturbed_models = scheme.models(model, parameter)?;
    let step = model.select_step(step, |p| matches!(p, Procedure::Frequency { .. }), "frequency")?;
    if step.base_state.is_some() {
        return Err(Error::NotSupported(format!("sensitivity of prestressed step `{}`", step.name)));
    }
    let Procedure::Frequency { modes, min_hz: None, max_hz: None } = step.procedure else {
        return Err(Error::NotSupported("frequency windows in modal sensitivity".into()));
    };
    let mut guard_model = model.clone();
    let mut guard_step = step.clone();
    guard_step.procedure =
        Procedure::Frequency { modes: modes.checked_add(1).ok_or_else(|| Error::Invalid("mode count overflow".into()))?, min_hz: None, max_hz: None };
    guard_model.replace_step(guard_step)?;
    let (result, mass) = analysis::frequency_with_mass(&guard_model, Some(&step.name), options, limits)?;
    require_uncapped_modes(&result, limits)?;
    let requested = modes as usize;
    if result.modes.len() < requested || (result.modes.len() == requested && result.stats.equations > requested) {
        return Err(Error::NotConverged("modal sensitivity needs requested modes and an upper gap guard".into()));
    }
    // Relative spectral threshold: frequency ratios below 1e-5 are unresolved rigid modes.
    let rigid_cutoff = result.modes.last().map_or(0.0, |m| m.eigenvalue.abs()) * 1e-10;
    for pair in result.modes.windows(2) {
        if let [a, b] = pair
            && a.eigenvalue > rigid_cutoff
            && b.eigenvalue > rigid_cutoff
            && (a.eigenvalue - b.eigenvalue).abs() <= 1e-5 * a.eigenvalue.abs().max(b.eigenvalue.abs())
        {
            return Err(Error::NotSupported("clustered eigenvalues need subspace sensitivity; individual modal derivatives refused".into()));
        }
    }
    let resolved = model.resolve()?;
    let mesh = Mesh::new(model, &resolved, limits.max_dofs)?;
    let (graph, order) = analysis::pattern(&mesh);
    let shape_system = if vectors {
        let setup = Setup { model, resolved: &resolved, mesh: &mesh, step };
        let mut warnings = Vec::new();
        let red = analysis::reduction(&setup, false, &order, analysis::budget(limits, &mesh, &graph, 8), &mut warnings)?;
        let k = analysis::assemble(&mesh, &graph, None)?.k;
        let [mut kr, mut mr] = red.reduce([&k, &mass])?;
        kr.mirror_lower();
        mr.mirror_lower();
        Some((red, kr, mr))
    } else {
        None
    };
    // Elastic constants and spring stiffness do not change mass. Density and
    // shell thickness do; retain their full assembled mass derivatives.
    let mass_changes = matches!(
        parameter,
        Parameter::Material { constant: fieldcraft_model::design::MaterialConstant::Density, .. } | Parameter::ShellThickness { .. }
    );
    let assembled = |m: &Model| -> Result<_> {
        let resolved = m.resolve()?;
        let other = Mesh::new(m, &resolved, limits.max_dofs)?;
        if other.labels != mesh.labels || other.dofs() != mesh.dofs() {
            return Err(Error::Invalid("modal parameter changed mesh layout".into()));
        }
        analysis::assemble(&other, &graph, mass_changes.then_some(options.mass))
    };
    let mut assembled_samples = Vec::with_capacity(perturbed_models.len());
    for (offset, trial) in &perturbed_models {
        assembled_samples.push((*offset, assembled(trial)?));
    }
    let exact_foundation = crate::foundation::derivative(&mesh, &graph, parameter)?;
    let exact_material = if scheme.method == DifferenceMethod::SixthOrder && exact_foundation.is_none() {
        analysis::assemble_material_derivative(&mesh, &graph, parameter)?
    } else {
        None
    };
    let response_frequency = exact_material
        .as_ref()
        .map(|_| sampled_frequency_responses(&perturbed_models, &step.name, options, requested, scheme, limits))
        .transpose()?;
    let dk = match exact_foundation.as_ref().map(|(k, _)| k).or(exact_material.as_ref()) {
        Some(k) => k.clone(),
        None => sampled_matrix_difference(
            &assembled_samples.iter().map(|(offset, assembled)| (*offset, assembled.k.clone())).collect::<Vec<_>>(),
            scheme,
        )?,
    };
    let dm = if mass_changes {
        let samples = assembled_samples
            .iter()
            .map(|(offset, assembled)| Ok((*offset, assembled.m.clone().ok_or_else(|| Error::Internal("perturbed mass".into()))?)))
            .collect::<Result<Vec<_>>>()?;
        Some(sampled_matrix_difference(&samples, scheme)?)
    } else {
        None
    };
    drop(assembled_samples);
    let mut out = Vec::new();
    let mut skipped = Vec::new();
    for (index, mode) in result.modes.iter().take(requested).enumerate() {
        let rigid = mode.eigenvalue <= rigid_cutoff;
        if rigid {
            skipped.push(index);
            out.push(FrequencyDerivative {
                frequency: mode.frequency,
                eigenvalue: mode.eigenvalue,
                frequency_derivative: 0.0,
                eigenvalue_derivative: 0.0,
                modal_mass: 0.0,
                vector: None,
            });
            continue;
        }
        let mut phi = Vec::with_capacity(mesh.dofs());
        for dof in 0..mesh.dofs() {
            let (slot, kind) = mesh.layout.locate(dof).ok_or_else(|| Error::Internal("mode dof layout".into()))?;
            let d = kind.direction().ok_or_else(|| Error::NotSupported("nonstructural modal sensitivity".into()))?;
            let value = if slot < mesh.labels.len() {
                if d < 3 { mode.shape.get(slot).and_then(|u| u.get(d)) } else { mode.rotation.get(slot).and_then(|u| u.get(d - 3)) }
            } else {
                mode.point_shape.get(slot - mesh.labels.len()).and_then(|u| u.get(d))
            };
            phi.push(value.copied().ok_or_else(|| Error::Internal("modal shape layout".into()))?);
        }
        let quadratic = |a: &Csr| {
            let mut y = vec![0.0; phi.len()];
            a.mul(&phi, &mut y);
            dot(&phi, &y)
        };
        let modal_mass = quadratic(&mass);
        if !modal_mass.is_finite() || modal_mass <= 0.0 {
            return Err(Error::Invalid("nonpositive modal mass".into()));
        }
        let dl = (quadratic(&dk) - mode.eigenvalue * dm.as_ref().map_or(0.0, quadratic)) / modal_mass;
        let df = response_frequency.as_ref().and_then(|values| values.get(index)).copied().unwrap_or(mode.frequency * dl / (2.0 * mode.eigenvalue));
        let dl = 2.0 * mode.eigenvalue * df / mode.frequency;
        if !dl.is_finite() || !df.is_finite() {
            return Err(Error::Invalid("non-finite modal derivative".into()));
        }
        let vector = if let Some((red, kr, mr)) = &shape_system {
            let normalized: Vec<_> = phi.iter().map(|p| p / modal_mass.sqrt()).collect();
            let q: Vec<_> = red.eq_dof.iter().map(|&i| normalized[i]).collect();
            let [mut dkr] = red.reduce([&dk])?;
            dkr.mirror_lower();
            let dmr = dm
                .as_ref()
                .map(|m| -> Result<Csr> {
                    let [mut m] = red.reduce([m])?;
                    m.mirror_lower();
                    Ok(m)
                })
                .transpose()?;
            let (dq, equation_relative_residual, normalization_derivative_residual) =
                super::nelson::derivative(super::nelson::Operators { k: kr, m: mr, dk: &dkr, dm: dmr.as_ref() }, &q, mode.eigenvalue, dl, limits)?;
            let full = red.expand(&dq, false);
            let (shape_si, rotation_si, points_si) = analysis::split(&normalized, &mesh);
            let (derivative_si, rotation_derivative_si, points_derivative_si) = analysis::split(&full, &mesh);
            Some(ModeShapeDerivative {
                nodes: mesh.labels.clone(),
                reference_points: result.reference_points.clone(),
                shape_si,
                rotation_si,
                points_si,
                derivative_si,
                rotation_derivative_si,
                points_derivative_si,
                equation_relative_residual,
                normalization_derivative_residual,
            })
        } else {
            None
        };
        out.push(FrequencyDerivative {
            frequency: mode.frequency,
            eigenvalue: mode.eigenvalue,
            frequency_derivative: df,
            eigenvalue_derivative: dl,
            modal_mass,
            vector,
        });
    }
    let mut warnings = result.warnings;
    if !skipped.is_empty() {
        warnings.push(format!("Skipped near-zero mode derivatives (zero-based indices): {skipped:?}; cutoff eigenvalue {rigid_cutoff}"));
    }
    Ok(FrequencySensitivity {
        parameter: parameter.clone(),
        parameter_value: parameter.value(model)?,
        increment: scheme.increment,
        modes: out,
        warnings,
    })
}

#[cfg(test)]
mod batch_tests {
    use super::*;
    use fieldcraft_model::design::{FoundationComponent, MaterialConstant};
    use fieldcraft_model::*;

    #[test]
    fn static_batch_matches_scalar_with_prescribed_motion_and_zero_coefficient() {
        let mut m = Model::default();
        for (i, p) in
            [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]].into_iter().enumerate()
        {
            m.set_node(i as u32 + 1, p).unwrap();
        }
        m.set_element(1, ElementKind::Hex8, vec![1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        m.add_set("all", Set { elements: vec![1], ..Set::default() }).unwrap();
        m.add_set("bed", Set { faces: vec![ElementFace { element: 1, face: 6 }], ..Set::default() }).unwrap();
        m.add_set("tip", Set { faces: vec![ElementFace { element: 1, face: 4 }], ..Set::default() }).unwrap();
        m.add_set("drive", Set { nodes: vec![7], ..Set::default() }).unwrap();
        m.add_material("M", Material { elastic: Some(Elastic::Isotropic { young: 1000., poisson: 0.3 }), ..Material::default() }).unwrap();
        m.add_section(
            "S",
            Section { set: "all".into(), material: "M".into(), kind: SectionKind::Solid {}, orientation: None, formulation: Default::default() },
        )
        .unwrap();
        m.add_foundation(
            "Bed",
            Foundation { set: "bed".into(), normal: 0., tangential: 100., normal_damping: 0., tangential_damping: 0., loss_factor: 0. },
        )
        .unwrap();
        m.add_foundation(
            "Other",
            Foundation { set: "bed".into(), normal: 100., tangential: 0., normal_damping: 0., tangential_damping: 0., loss_factor: 0. },
        )
        .unwrap();
        m.add_boundary_condition(
            "Drive",
            BoundaryCondition::Displacement { target: Target::Set("drive".into()), values: [None, None, Some(0.002), None, None, None] },
        )
        .unwrap();
        m.add_load("P", Load::Pressure { set: "tip".into(), magnitude: -1. }).unwrap();
        let mut step = Step::new("S", Procedure::Static {});
        step.boundary_conditions.insert("Drive".into());
        step.loads.insert("P".into());
        m.add_step(step, None).unwrap();
        let parameters = vec![
            (Parameter::Material { name: "M".into(), constant: MaterialConstant::Young }, 0.01),
            (Parameter::Foundation { name: "Bed".into(), component: FoundationComponent::Normal }, 0.01),
            (Parameter::Foundation { name: "Bed".into(), component: FoundationComponent::Tangential }, 0.01),
        ];
        let observations = [
            StaticObservation { node: 2, dof: 1, stress_element: None },
            StaticObservation { node: 1, dof: 2, stress_element: None },
            StaticObservation { node: 7, dof: 3, stress_element: None },
        ];
        let limits = Limits::default();
        let batch = static_jacobian(&m, "S", &parameters, &observations, &limits).unwrap();
        for (row, o) in observations.iter().enumerate() {
            for (col, (p, h)) in parameters.iter().enumerate() {
                let scalar = static_sensitivity(&m, Some("S"), p, *h, *o, &limits).unwrap();
                assert_eq!(batch.values[row].to_bits(), scalar.displacement.to_bits());
                assert_eq!(batch.derivatives[row][col].to_bits(), scalar.direct.to_bits());
            }
        }
    }
}
