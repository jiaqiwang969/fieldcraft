//! Opt-in reuse of a current, converged elastic equilibrium.
//!
//! This is not a material/contact restart format. The caller binds a result while
//! its input identity is current (the engine checks its stored definition hash).
//! The immutable binding then rejects changed inputs. The nonlinear assembler
//! recovers independent displacements and evaluates its existing terminal tangent;
//! no load increments or Newton corrections are performed.

use crate::{Error, Result, StaticResult};
use fieldcraft_model::{Model, Procedure};

pub struct StoredEquilibrium<'a> {
    pub(crate) result: &'a StaticResult,
    inputs: Vec<u8>,
}

fn inputs(model: &Model, step: &str) -> Result<Vec<u8>> {
    let input = model.step_inputs(step).ok_or_else(|| Error::Invalid(format!("stored_equilibrium.base_missing: `{step}`")))?;
    serde_json::to_vec(&input).map_err(|e| Error::Invalid(format!("stored_equilibrium.input_identity: {e}")))
}

impl<'a> StoredEquilibrium<'a> {
    /// Bind only a result whose producer input identity has been checked by the
    /// caller. Displacement fields alone cannot prove their provenance.
    pub fn bind(model: &Model, result: &'a StaticResult) -> Result<Self> {
        let step = model.step(&result.step).ok_or_else(|| Error::Invalid("stored_equilibrium.base_missing".into()))?;
        let Procedure::NonlinearStatic(settings) = &step.procedure else {
            return Err(Error::NotSupported("stored_equilibrium.requires_nlgeom_static".into()));
        };
        settings.validate()?;
        if !settings.nonlinear_geometry
            || settings.imperfection.is_some()
            || settings.arc_length.is_some()
            || settings.load_path.is_some()
            || !settings.load_paths.is_empty()
        {
            return Err(Error::NotSupported("stored_equilibrium.requires_monotone_nlgeom_without_imperfection".into()));
        }
        if !model.contacts().is_empty() || !step.contact_perturbation.is_empty() {
            return Err(Error::NotSupported("stored_equilibrium.contact_history_unavailable".into()));
        }
        if model.materials().values().any(|m| m.plastic.is_some()) || !model.initial_fields().is_empty() {
            return Err(Error::NotSupported("stored_equilibrium.material_history_unavailable".into()));
        }
        if model.sections().values().any(|s| s.formulation.hybrid || s.formulation.incompatible_modes) {
            return Err(Error::NotSupported("stored_equilibrium.element_history_unavailable".into()));
        }
        if result.nonlinear.as_ref().and_then(|h| h.increments.last()).is_none_or(|i| !i.converged || i.load_factor != 1.0) {
            return Err(Error::Invalid("stored_equilibrium.not_converged_at_full_load".into()));
        }
        let nodes: std::collections::BTreeSet<_> = model.elements().values().flat_map(|e| e.nodes.iter().copied()).collect();
        if !result.nodes.iter().copied().eq(nodes) {
            return Err(Error::Invalid("stored_equilibrium.displacement_layout: source element nodes differ".into()));
        }
        if result.nodes.windows(2).any(|n| n[0] >= n[1])
            || result.displacement.len() != result.nodes.len()
            || result.rotation.len() != result.nodes.len()
        {
            return Err(Error::Invalid("stored_equilibrium.displacement_layout".into()));
        }
        if result.displacement.iter().chain(&result.rotation).flatten().any(|v| !v.is_finite())
            || result.reference_points.iter().flat_map(|p| p.displacement).any(|v| !v.is_finite())
        {
            return Err(Error::Invalid("stored_equilibrium.nonfinite_displacement".into()));
        }
        Ok(Self { result, inputs: inputs(model, &result.step)? })
    }

    pub(crate) fn validate(&self, model: &Model, step: &fieldcraft_model::Step) -> Result<()> {
        if step.base_state.as_deref() != Some(self.result.step.as_str()) {
            return Err(Error::Invalid("stored_equilibrium.base_mismatch".into()));
        }
        if !step.contact_perturbation.is_empty() {
            return Err(Error::NotSupported("stored_equilibrium.contact_history_unavailable".into()));
        }
        if step.perturbation_follower_pressure {
            return Err(Error::NotSupported(
                "perturbation_follower_pressure requires a linear static shell base; nonlinear shell equilibrium is unavailable".into(),
            ));
        }
        if inputs(model, &self.result.step)? != self.inputs {
            return Err(Error::Invalid("stored_equilibrium.input_changed".into()));
        }
        Ok(())
    }
}
