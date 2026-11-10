//! External backend contracts. No native solver or CUDA is loaded by this module.
//!
//! Frame facts are source claims, distinct from receipt/transport success. The default
//! numerical policy rejects unknown diagnostics and either iteration limit. The receiver
//! independently checks a contiguous prefix. Persistent model identity and durable result
//! registration are integration seams, not inferred from temporary run identifiers.

pub mod control;
pub mod receive;
pub mod run;
pub mod transaction;
#[cfg(not(target_arch = "wasm32"))]
pub mod worker;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceKind {
    Solver,
    ProtocolFixture,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SourceRef {
    pub sha256: String,
    pub locator: String,
    pub observer: String,
    pub kind: EvidenceKind,
}

impl SourceRef {
    pub fn validate(&self) -> Result<(), ContractError> {
        if !digest_valid(&self.sha256) || !token_valid(&self.locator) || !token_valid(&self.observer) {
            return Err(ContractError::Invalid("source reference"));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case", deny_unknown_fields)]
pub enum Observed<T> {
    Known { value: T, source: SourceRef },
    Unknown { reason: String },
}

impl<T> Observed<T> {
    fn validate(&self, kind: EvidenceKind) -> Result<(), ContractError> {
        match self {
            Self::Known { source, .. } => {
                source.validate()?;
                if source.kind != kind {
                    return Err(ContractError::Invalid("mixed evidence kinds"));
                }
            }
            Self::Unknown { reason } if !token_valid(reason) => return Err(ContractError::Invalid("unknown reason")),
            Self::Unknown { .. } => {}
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunKey {
    pub run_id: String,
    pub epoch: u64,
    pub worker_generation: String,
}

impl RunKey {
    pub fn validate(&self) -> Result<(), ContractError> {
        if !token_valid(&self.run_id) || !token_valid(&self.worker_generation) {
            return Err(ContractError::Invalid("run key"));
        }
        Ok(())
    }
}

pub(super) fn token_valid(s: &str) -> bool {
    !s.trim().is_empty() && s.len() <= 1024 && !s.chars().any(char::is_control)
}

pub(super) fn digest_valid(s: &str) -> bool {
    s.len() == 64 && s.bytes().all(|c| c.is_ascii_digit() || (b'a'..=b'f').contains(&c))
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ContractError {
    #[error("invalid external result: {0}")]
    Invalid(&'static str),
    #[error("run, epoch or worker generation does not match")]
    StaleRun,
    #[error("operation is not allowed in the current run state")]
    InvalidState,
    #[error("frame rejected at the first failure or unknown barrier: {0:?}")]
    FrameRejected(run::FrameIssue),
    #[error("persistent identity verifier is not configured")]
    IdentityUnavailable,
    #[error("model binding mismatch: {0}")]
    IdentityMismatch(String),
    #[error("not supported yet: durable external result registration")]
    StorageUnavailable,
    #[error("NativeContactPolicyNotApplied: declaration validation is not native application or runtime qualification")]
    NativeContactPolicyNotApplied,
    #[error("binary field checksum mismatch")]
    ChecksumMismatch,
    #[error("not supported yet: {0}")]
    Unsupported(&'static str),
    #[error("worker operation failed: {0}")]
    Worker(String),
    #[error("worker has not exited within the observation budget")]
    WorkerTimeout,
}
