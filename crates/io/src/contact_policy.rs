//! Contact-policy request inspection, separate from native application and runtime qualification.
//! Only the native `Step.contact_perturbation` spelling is used; this is not a solver control.
use crate::{Geometry, Result, Run, identity, invalid};
use fieldcraft_model::{ContactSurfaces, Model, Procedure, Step};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::BTreeMap;

pub const REQUEST_SCHEME: &str = "fieldcraft-contact-policy-request/1";

fn map(value: &Value) -> Result<BTreeMap<String, String>> {
    let policies: BTreeMap<String, String> =
        serde_json::from_value(value.clone()).map_err(|e| crate::Error::Invalid(format!("InvalidContactPolicyMap: {e}")))?;
    for (name, policy) in &policies {
        if policy != "stick" {
            return invalid(format!("UnsupportedContactPolicy: {name}={policy}"));
        }
    }
    Ok(policies)
}
fn stored(step: &Step) -> Result<BTreeMap<String, String>> {
    let value = serde_json::to_value(step).map_err(|e| crate::Error::Invalid(e.to_string()))?;
    match value.get("contact_perturbation") {
        Some(v) => map(v),
        None => Ok(BTreeMap::new()),
    }
}
/// The current compiled model type, not a claim inferred from another branch's documentation.
pub fn native_field_available() -> bool {
    let mut probe = json!({"name":"probe","procedure":{"type":"static"},"output":[],"contact_perturbation":{"probe":"stick"}});
    let Ok(step) = serde_json::from_value::<Step>(probe.take()) else {
        return false;
    };
    stored(&step).is_ok_and(|v| v.get("probe").is_some_and(|p| p == "stick"))
}

fn check_request(model: &Model, step_name: &str, policies: &BTreeMap<String, String>) -> Result<()> {
    let Some(step) = model.step(step_name) else {
        return invalid(format!("UnknownPolicyStep: {step_name}"));
    };
    if policies.is_empty() {
        return Ok(());
    }
    for (name, value) in policies {
        if value != "stick" {
            return invalid(format!("UnsupportedContactPolicy: {name}={value}"));
        }
        let Some(contact) = model.contact(name) else {
            return invalid(format!("UnknownPolicyContact: {name}"));
        };
        if !matches!(contact.surfaces, ContactSurfaces::Pair { .. }) {
            return invalid(format!("ContactPolicyPairRequired: {name}"));
        }
    }
    for name in model.contacts().keys() {
        if !policies.contains_key(name) {
            return invalid(format!("MissingContactPolicy: {name}"));
        }
    }
    if !matches!(step.procedure, Procedure::Frequency { .. }) {
        return invalid("ContactPolicyFrequencyRequired");
    }
    if step.linear_base {
        return invalid("ContactPolicyNonlinearBaseRequired");
    }
    let Some(base_name) = step.base_state.as_deref() else {
        return invalid("ContactPolicyBaseRequired");
    };
    let Some(base) = model.step(base_name) else {
        return invalid("ContactPolicyBaseRequired");
    };
    if !matches!(&base.procedure,Procedure::NonlinearStatic(n) if n.nonlinear_geometry) || base.base_state.is_some() {
        return invalid("ContactPolicyNonlinearBaseRequired");
    }
    let base_index = model.steps().iter().position(|s| s.name == base_name);
    let step_index = model.steps().iter().position(|s| s.name == step_name);
    if !matches!((base_index,step_index),(Some(b),Some(s)) if b<s) {
        return invalid("ContactPolicyEarlierBaseRequired");
    }
    if base.boundary_conditions != step.boundary_conditions {
        return invalid("ContactPolicyUnchangedSupportsRequired");
    }
    Ok(())
}

/// A frozen declaration, never a certificate of closed/sticking state or solver success.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyRequest {
    pub scheme: String,
    pub snapshot: identity::Snapshot,
    pub step: String,
    pub contact_perturbation: BTreeMap<String, String>,
}
impl PolicyRequest {
    pub fn freeze(model: &Model, geometry: &Geometry, run: &Run, step: &str, declaration: &Value) -> Result<Self> {
        let policies = map(declaration)?;
        check_request(model, step, &policies)?;
        Ok(Self {
            scheme: REQUEST_SCHEME.into(),
            snapshot: identity::snapshot(model, geometry, run)?,
            step: step.into(),
            contact_perturbation: policies,
        })
    }
    pub fn validate(&self, model: &Model, geometry: &Geometry, run: &Run) -> Result<()> {
        if self.scheme != REQUEST_SCHEME {
            return invalid("UnsupportedPolicyRequestScheme");
        }
        self.snapshot.validate(model, geometry, run)?;
        check_request(model, &self.step, &self.contact_perturbation)
    }
    /// Checks native application only. Runtime qualification still needs original solver evidence.
    pub fn require_applied(&self, model: &Model, geometry: &Geometry, run: &Run) -> Result<()> {
        self.validate(model, geometry, run)?;
        let Some(step) = model.step(&self.step) else {
            return invalid("UnknownPolicyStep");
        };
        if stored(step)? != self.contact_perturbation {
            return invalid("NativeContactPolicyNotApplied");
        }
        Ok(())
    }
    /// Independently freeze this digest in the run lease alongside the native model snapshot.
    pub fn digest(&self) -> Result<String> {
        let bytes = serde_json::to_vec(self).map_err(|e| crate::Error::Invalid(e.to_string()))?;
        Ok(identity::artifact_digest(&bytes))
    }
    /// Validate an archived request against both its frozen digest and current model.
    pub fn decode(bytes: &[u8], digest: &str, model: &Model, geometry: &Geometry, run: &Run) -> Result<Self> {
        if bytes.len() > 1024 * 1024 {
            return invalid("PolicyRequestTooLarge");
        }
        let request: Self = serde_json::from_slice(bytes).map_err(|e| crate::Error::Invalid(e.to_string()))?;
        if request.digest()? != digest {
            return invalid("ContactPolicyRequestMismatch");
        }
        request.validate(model, geometry, run)?;
        Ok(request)
    }
}

/// Availability and selected inputs only. No parsing of free-text solver warnings.
pub fn report(model: &Model) -> Result<Value> {
    let mut steps = BTreeMap::new();
    for step in model.steps() {
        let selected = stored(step)?;
        steps.insert(step.name.clone(),json!({
            "contact_perturbation":selected,
            "selection":if !selected.is_empty(){"explicit_stick"}else if step.base_state.is_some() && !model.contacts().is_empty() && matches!(step.procedure,Procedure::Frequency {..}) {"legacy_bonded_when_closed"}else{"not_requested"},
            "runtime_qualification":"unverified",
        }));
    }
    Ok(json!({
        "native_field":"Step.contact_perturbation",
        "native_field_available":native_field_available(),
        "declaration_values":["stick"],
        "steps":steps,
        "qualification_requires":["frequency_from_earlier_nlgeom_base","unchanged_support_names","all_contacts_are_explicit_pairs","closed_planar_linear_solid_faces","fixed_main_translations","full_reference_overlap","positive_pressure_above_guard","gap_over_thickness_within_solver_tolerance","positive_static_friction_and_strict_cone_margin","not_slipping","interface_motion_eliminated"],
        "state_evidence":"pending_original_runtime_receipt; do not parse diagnostic warnings as a stable protocol",
        "commercial_keyword_mapping":"pending; no verified mapping",
        "other_policies":"slide/open unsupported",
    }))
}
