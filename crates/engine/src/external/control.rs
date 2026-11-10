//! Route-20 contact-control evidence.
//!
//! A control receipt is an observation envelope, not a solver result.  In particular,
//! an initialization value is kept at the observation point and never promoted to a
//! per-frame value.  The receipt digest and every source reference stay attached so a
//! receiver can reject a copied value from another run.

use serde::{Deserialize, Serialize};

use super::receive::EvidenceCheck;
use super::{ContractError, EvidenceKind, Observed, RunKey, SourceRef, digest_valid, token_valid};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NativeIpcStatus {
    Enabled,
    Disabled,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlEvidenceScope {
    Initialization,
    AdvanceFrames,
    NativeIpcDisabled,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlAvailability {
    InitializationKnown,
    InitializationUnknown,
    AdvanceKnown,
    AdvanceUnknown,
    NativeIpcDisabled,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlEvidenceReceipt {
    /// The producer run label in bindings.json, deliberately separate from RunKey::run_id.
    pub run_label: String,
    pub epoch: Option<u64>,
    pub observed_compute_pids: Vec<u32>,
    pub scope: ControlEvidenceScope,
    pub native_ipc: NativeIpcStatus,
    /// A frame and time are both present or both absent.  Initialization receipts from
    /// route 20 have neither: they prove only the initialization observation point.
    pub observed_frame: Option<u64>,
    pub observed_time_s: Option<f64>,
    /// SHA-256 of the complete, independently audited bindings.json record set.
    pub record_sha256: String,
    pub record_locator: String,
    /// SHA-256 of the raw fieldcraft-contact-controls/1 control file.  IPC-disabled
    /// records intentionally have no control file or fabricated digest.
    pub control_sha256: Option<String>,
    pub control_locator: String,
    pub kappa_effective_pa: Observed<f64>,
    pub kappa_clamped: Observed<bool>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ControlAdmission {
    pub availability: ControlAvailability,
    pub native_ipc: NativeIpcStatus,
    pub run_label: String,
    pub epoch: Option<u64>,
    pub observed_frame: Option<u64>,
    pub observed_time_s: Option<f64>,
    pub record_sha256: String,
    pub record_locator: String,
    pub control_sha256: Option<String>,
    pub control_locator: String,
    pub kappa_effective_pa: Observed<f64>,
    pub kappa_clamped: Observed<bool>,
    pub effective_check: EvidenceCheck,
    pub clamped_check: EvidenceCheck,
}

impl ControlEvidenceReceipt {
    pub fn validate(&self) -> Result<(), ContractError> {
        if !token_valid(&self.run_label)
            || !digest_valid(&self.record_sha256)
            || !token_valid(&self.record_locator)
            || !token_valid(&self.control_locator)
        {
            return Err(ContractError::Invalid("contact control evidence identity"));
        }
        if self.observed_compute_pids.windows(2).any(|pair| pair[0] >= pair[1]) || self.observed_compute_pids.contains(&0) {
            return Err(ContractError::Invalid("contact control compute pid list"));
        }
        match (self.observed_frame, self.observed_time_s) {
            (Some(_), Some(time)) if time.is_finite() && time >= 0.0 => {}
            (None, None) => {}
            _ => return Err(ContractError::Invalid("contact control observation time")),
        }
        match self.control_sha256 {
            Some(ref digest) if digest_valid(digest) => {}
            Some(_) => return Err(ContractError::Invalid("contact control file digest")),
            None => {
                if !matches!(self.scope, ControlEvidenceScope::NativeIpcDisabled) {
                    return Err(ContractError::Invalid("missing contact control file digest"));
                }
            }
        }
        self.kappa_effective_pa.validate(EvidenceKind::Solver)?;
        self.kappa_clamped.validate(EvidenceKind::Solver)?;
        if let Observed::Known { value, .. } = &self.kappa_effective_pa
            && (!value.is_finite() || *value <= 0.0)
        {
            return Err(ContractError::Invalid("effective kappa must be positive and finite"));
        }
        match self.scope {
            ControlEvidenceScope::NativeIpcDisabled => {
                if self.native_ipc != NativeIpcStatus::Disabled
                    || self.epoch.is_some()
                    || !self.observed_compute_pids.is_empty()
                    || self.observed_frame.is_some()
                    || self.observed_time_s.is_some()
                    || !matches!(self.kappa_effective_pa, Observed::Unknown { .. })
                    || !matches!(self.kappa_clamped, Observed::Unknown { .. })
                {
                    return Err(ContractError::Invalid("native IPC disabled evidence shape"));
                }
            }
            ControlEvidenceScope::Initialization | ControlEvidenceScope::AdvanceFrames => {
                if self.native_ipc == NativeIpcStatus::Disabled || self.epoch.is_none() || self.observed_compute_pids.is_empty() {
                    return Err(ContractError::Invalid("contact control run evidence shape"));
                }
            }
        }
        Ok(())
    }

    pub fn availability(&self) -> ControlAvailability {
        if matches!(self.scope, ControlEvidenceScope::NativeIpcDisabled) {
            return ControlAvailability::NativeIpcDisabled;
        }
        let known = matches!(self.kappa_effective_pa, Observed::Known { .. }) && matches!(self.kappa_clamped, Observed::Known { .. });
        match (self.scope, known) {
            (ControlEvidenceScope::Initialization, true) => ControlAvailability::InitializationKnown,
            (ControlEvidenceScope::Initialization, false) => ControlAvailability::InitializationUnknown,
            (ControlEvidenceScope::AdvanceFrames, true) => ControlAvailability::AdvanceKnown,
            (ControlEvidenceScope::AdvanceFrames, false) => ControlAvailability::AdvanceUnknown,
            (ControlEvidenceScope::NativeIpcDisabled, _) => ControlAvailability::NativeIpcDisabled,
        }
    }

    pub fn validate_for_run(&self, run: &RunKey, require_effective: bool) -> Result<(), ContractError> {
        self.validate()?;
        if self.run_label != run.run_id || self.epoch.is_some_and(|epoch| epoch != run.epoch) {
            return Err(ContractError::StaleRun);
        }
        if !require_effective {
            return Ok(());
        }
        match self.availability() {
            ControlAvailability::InitializationKnown | ControlAvailability::InitializationUnknown => {
                Err(ContractError::Invalid("MissingPerFrameEffectiveKappa: initialization-only observation"))
            }
            ControlAvailability::AdvanceKnown => Ok(()),
            ControlAvailability::AdvanceUnknown => Err(ContractError::Invalid("MissingEffectiveKappa: per-frame observation is unknown")),
            ControlAvailability::NativeIpcDisabled => Err(ContractError::Unsupported("native IPC disabled; effective kappa unavailable")),
        }
    }

    pub fn admission(&self) -> ControlAdmission {
        let effective_check =
            if matches!(self.kappa_effective_pa, Observed::Known { .. }) { EvidenceCheck::Verified } else { EvidenceCheck::Unverified };
        let clamped_check = if matches!(self.kappa_clamped, Observed::Known { .. }) { EvidenceCheck::Verified } else { EvidenceCheck::Unverified };
        ControlAdmission {
            availability: self.availability(),
            native_ipc: self.native_ipc,
            run_label: self.run_label.clone(),
            epoch: self.epoch,
            observed_frame: self.observed_frame,
            observed_time_s: self.observed_time_s,
            record_sha256: self.record_sha256.clone(),
            record_locator: self.record_locator.clone(),
            control_sha256: self.control_sha256.clone(),
            control_locator: self.control_locator.clone(),
            kappa_effective_pa: self.kappa_effective_pa.clone(),
            kappa_clamped: self.kappa_clamped.clone(),
            effective_check,
            clamped_check,
        }
    }

    pub fn source_refs(&self) -> impl Iterator<Item = &SourceRef> {
        self.kappa_effective_pa.source().into_iter().chain(self.kappa_clamped.source())
    }
}

impl<T> Observed<T> {
    fn source(&self) -> Option<&SourceRef> {
        match self {
            Self::Known { source, .. } => Some(source),
            Self::Unknown { .. } => None,
        }
    }
}
