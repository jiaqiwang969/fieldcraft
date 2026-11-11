//! Re-evaluate a stateless elastic terminal using the existing constraint expansion.
use super::*;
use std::borrow::Cow;

pub(crate) fn terminal(model: &Model, result: &StaticResult, limits: &Limits) -> Result<crate::prestress::Terminal> {
    let step = model.step(&result.step).ok_or_else(|| Error::Invalid("stored_equilibrium.base_missing".into()))?;
    let fixed = if step.pretension.values().any(|c| matches!(c, fieldcraft_model::PretensionControl::Fixed)) {
        let mut out = model.clone();
        let mut step = step.clone();
        for (name, control) in &mut step.pretension {
            if matches!(control, fieldcraft_model::PretensionControl::Fixed) {
                let section = model.pretensions().get(name).ok_or_else(|| Error::Invalid("stored_equilibrium.section_missing".into()))?;
                let q = result
                    .pretension
                    .iter()
                    .find(|p| p.name == *name)
                    .map(|p| p.adjustment)
                    .filter(|q| q.is_finite())
                    .ok_or_else(|| Error::Invalid("stored_equilibrium.adjustment_missing".into()))?;
                let rp = result
                    .reference_points
                    .iter()
                    .find(|p| p.name == section.control)
                    .ok_or_else(|| Error::Invalid("stored_equilibrium.control_missing".into()))?;
                if rp.displacement[0].to_bits() != q.to_bits() {
                    return Err(Error::Invalid("stored_equilibrium.adjustment_mismatch".into()));
                }
                *control = fieldcraft_model::PretensionControl::Adjustment { value: q };
            }
        }
        out.replace_step(step)?;
        Cow::Owned(out)
    } else {
        Cow::Borrowed(model)
    };
    let policies = std::collections::BTreeMap::new();
    let mut terminal = None;
    solve_on(&fixed, &result.step, limits, Vec::new(), Some((&policies, &mut terminal)), Some(result))?;
    terminal.ok_or_else(|| Error::Internal("stored equilibrium terminal tangent".into()))
}

fn component(mesh: &Mesh, result: &StaticResult, dof: usize) -> Result<Option<f64>> {
    let (slot, kind) = mesh
        .layout
        .locate(dof)
        .ok_or_else(|| Error::Invalid(format!("stored_equilibrium.displacement_layout: unknown DOF {dof} of {}", mesh.dofs())))?;
    let direction = kind.direction().ok_or_else(|| Error::NotSupported("stored_equilibrium.nonstructural_dof".into()))?;
    if let Some(label) = mesh.labels.get(slot) {
        let Ok(i) = result.nodes.binary_search(label) else { return Ok(None) };
        return Ok(if direction < 3 {
            result.displacement.get(i).and_then(|p| p.get(direction)).copied()
        } else {
            result.rotation.get(i).and_then(|p| p.get(direction - 3)).copied()
        });
    }
    let point = slot
        .checked_sub(mesh.labels.len())
        .and_then(|i| mesh.points.get(i))
        .ok_or_else(|| Error::Invalid("stored_equilibrium.reference_layout".into()))?;
    let value = result
        .reference_points
        .iter()
        .find(|p| p.label == point.label && p.name == point.name)
        .and_then(|p| p.displacement.get(direction))
        .copied()
        .ok_or_else(|| Error::Invalid("stored_equilibrium.reference_layout: missing point".into()))?;
    Ok(Some(value))
}

pub(super) fn evaluate(
    model: &Model,
    resolved: &fieldcraft_model::Resolved,
    run: &mut Run<'_>,
    result: &StaticResult,
) -> Result<crate::prestress::Terminal> {
    let mesh = run.problem.mesh;
    if !mesh.structural.is_empty() || !mesh.trusses.is_empty() || mesh.theory.is_some() {
        return Err(Error::NotSupported("stored_equilibrium.requires_elastic_3d_solids".into()));
    }
    // The eliminated private cut copies are reconstructed by the same T and
    // summation order as the solve, rather than by adding q to nodal output.
    let q = run
        .problem
        .red
        .eq_dof
        .iter()
        .map(|&d| component(mesh, result, d)?.ok_or_else(|| Error::Invalid("stored_equilibrium.independent_displacement_missing".into())))
        .collect::<Result<Vec<_>>>()?;
    let u = run.problem.red.expand_scaled(&q, 1.0);
    for (d, actual) in u.iter().enumerate() {
        if let Some(expected) = component(mesh, result, d)?
            && actual.to_bits() != expected.to_bits()
        {
            return Err(Error::Invalid(format!("stored_equilibrium.constraint_mismatch: DOF {d}")));
        }
    }
    for label in &result.nodes {
        if mesh.node(*label).is_none() {
            return Err(Error::Invalid(format!("stored_equilibrium.displacement_layout: node {label} absent from terminal mesh")));
        }
    }
    let mut evaluated = run.problem.evaluate_with_contact(&u, 1.0, true, &run.committed, &run.contact, &run.reference, 0.0, false)?;
    let stiffness = evaluated.tangent.take().ok_or_else(|| Error::Internal("stored equilibrium stiffness".into()))?;
    // A second equilibrium check uses the existing force criterion, with its
    // round-off floor evaluated at this terminal. It never corrects the state.
    let [reduced] = run.problem.red.reduce([&stiffness])?;
    run.seed_force_floor(&reduced);
    let residual = run.residual(&evaluated);
    if !run.force_within(&residual, run.settings.tolerances.force) {
        return Err(Error::Invalid(format!(
            "stored_equilibrium.not_in_balance: force measure {} exceeds {}",
            run.force_measure(&residual),
            run.settings.tolerances.force
        )));
    }
    let constraints = crate::tie::rows(model, resolved, mesh, &mut Vec::new())?.into_iter().map(crate::prestress::FrozenRow::from_row).collect();
    Ok(crate::prestress::Terminal { stiffness, constraints })
}
