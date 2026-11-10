//! Receiver-facing parameter capabilities. Extraction is not constitutive equivalence.
use crate::{Result, invalid};
use fieldcraft_model::{Elastic, Model};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::collections::BTreeMap;

pub const SCHEME: &str = "fieldcraft-parameter-capabilities/1";
pub const CONTACT_SCHEME: &str = "fieldcraft-contact-controls/1";

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IsotropicParameters {
    pub young_pa: f64,
    pub poisson: f64,
    pub density_kg_m3: f64,
}

/// Only pure constant isotropic elastic/density inputs. Never silently discard extra physics.
pub fn isotropic_parameters(model: &Model, name: &str) -> Result<IsotropicParameters> {
    let Some(m) = model.material(name) else {
        return invalid(format!("UnknownMaterial: {name}"));
    };
    let raw = serde_json::to_value(m).map_err(|e| crate::Error::Invalid(e.to_string()))?;
    if let Some(extra) = raw.as_object().and_then(|o| o.keys().find(|k| !matches!(k.as_str(), "elastic" | "density"))) {
        return invalid(format!("UnsupportedMaterialParameter: {name}.{extra}"));
    }
    let Some(Elastic::Isotropic { young, poisson }) = m.elastic else {
        return invalid(format!("UnsupportedElasticMapping: {name}"));
    };
    let Some(density) = m.density else {
        return invalid(format!("MissingDensity: {name}"));
    };
    Ok(IsotropicParameters { young_pa: young, poisson, density_kg_m3: density })
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case", deny_unknown_fields)]
pub enum Extraction {
    Supported { parameters: IsotropicParameters },
    Refused { reason: String },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaterialParameters {
    /// All original model material fields, in SI; nothing is replaced by the extraction.
    pub physical: Value,
    pub isotropic_extraction: Extraction,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContactParameters {
    pub physical: Value,
    pub numerical: Value,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Report {
    pub scheme: String,
    pub materials: BTreeMap<String, MaterialParameters>,
    pub contacts: BTreeMap<String, ContactParameters>,
    pub dependencies: BTreeMap<String, String>,
}
/// Recomputed from authoritative model definitions, not a static backend support claim.
pub fn report(model: &Model) -> Result<Report> {
    let materials = model
        .materials()
        .iter()
        .map(|(name, m)| {
            let physical = serde_json::to_value(m).map_err(|e| crate::Error::Invalid(e.to_string()))?;
            let isotropic_extraction = match isotropic_parameters(model, name) {
                Ok(parameters) => Extraction::Supported { parameters },
                Err(e) => Extraction::Refused { reason: e.to_string() },
            };
            Ok((name.clone(), MaterialParameters { physical, isotropic_extraction }))
        })
        .collect::<Result<_>>()?;
    let contacts=model.contacts().iter().map(|(name,c)| (name.clone(),ContactParameters {
        physical:json!({"friction_static":c.friction.static_coefficient,"friction_kinetic":c.friction.kinetic_coefficient(),"friction_decay_s_per_m":c.friction.decay}),
        numerical:json!({"penalty_scale":c.penalty_scale,"initial_penetration":c.initial_penetration}),
    })).collect();
    Ok(Report {
        scheme: SCHEME.into(),
        materials,
        contacts,
        dependencies: BTreeMap::from([
            ("kappa_effective_pa".into(), "Unknown: backend observation required; native penalty_scale is not kappa".into()),
            ("kappa_clamped".into(), "Unknown: backend observation required; absence of a warning is not false".into()),
            ("contact_perturbation_policy".into(), "Pending policy contract; no new keyword mapping claimed".into()),
            (
                "constitutive_conversion".into(),
                "Separate versioned material conversion contract required; equal E/nu does not establish equivalence".into(),
            ),
        ]),
    })
}
impl Report {
    pub fn validate(&self, model: &Model) -> Result<()> {
        if self.scheme != SCHEME {
            return invalid("UnsupportedCapabilityScheme");
        }
        if self != &report(model)? {
            return invalid("CapabilityModelMismatch");
        }
        Ok(())
    }
}

/// Explicit provenance is required for a known fact; unknown values have no numeric substitute.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case", deny_unknown_fields)]
pub enum Observation<T> {
    Known { value: T, source_sha256: String, locator: String },
    Unknown { reason: String },
}
impl<T> Observation<T> {
    fn check_source(&self) -> Result<()> {
        match self {
            Self::Known { source_sha256, locator, .. } => {
                if !crate::identity::digest_valid(source_sha256) || locator.trim().is_empty() {
                    return invalid("MissingObservationSource");
                }
            }
            Self::Unknown { reason } if reason.trim().is_empty() => return invalid("MissingUnknownReason"),
            _ => {}
        }
        Ok(())
    }
}
/// SI numerical controls, separate from physical E/nu/rho/friction. No serde defaults.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ContactControls {
    pub scheme: String,
    pub d_hat_m: f64,
    pub kappa_requested_pa: f64,
    pub kappa_effective_pa: Observation<f64>,
    pub kappa_clamped: Observation<bool>,
}
impl ContactControls {
    /// Diagnostic profiles may retain unknown observations; quantitative use must require them.
    /// Source byte verification and run/epoch binding are obligations of the receiving host.
    pub fn validate(&self, require_effective: bool) -> Result<()> {
        if self.scheme != CONTACT_SCHEME {
            return invalid("UnsupportedContactControlsScheme");
        }
        if !self.d_hat_m.is_finite() || self.d_hat_m <= 0.0 {
            return invalid("InvalidContactControl: d_hat_m");
        }
        if !self.kappa_requested_pa.is_finite() || self.kappa_requested_pa <= 0.0 {
            return invalid("InvalidContactControl: kappa_requested_pa");
        }
        self.kappa_effective_pa.check_source()?;
        self.kappa_clamped.check_source()?;
        if let Observation::Known { value, .. } = &self.kappa_effective_pa
            && (!value.is_finite() || *value <= 0.0)
        {
            return invalid("InvalidContactControl: kappa_effective_pa");
        }
        if require_effective {
            if matches!(self.kappa_effective_pa, Observation::Unknown { .. }) {
                return invalid("MissingEffectiveKappa");
            }
            if matches!(self.kappa_clamped, Observation::Unknown { .. }) {
                return invalid("MissingKappaClamped");
            }
        }
        Ok(())
    }
    pub fn encode(&self, require_effective: bool) -> Result<Vec<u8>> {
        self.validate(require_effective)?;
        let bytes = serde_json::to_vec(self).map_err(|e| crate::Error::Invalid(e.to_string()))?;
        if bytes.len() > 65536 {
            return invalid("ContactControlsTooLarge");
        }
        Ok(bytes)
    }
    pub fn decode(bytes: &[u8], require_effective: bool) -> Result<Self> {
        if bytes.len() > 65536 {
            return invalid("ContactControlsTooLarge");
        }
        let controls: Self = serde_json::from_slice(bytes).map_err(|e| crate::Error::Invalid(e.to_string()))?;
        controls.validate(require_effective)?;
        Ok(controls)
    }
}
