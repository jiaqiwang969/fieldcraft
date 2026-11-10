//! Frozen receiver context. Archive integrity is not runtime qualification.
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Component, Path};

use fieldcraft_io::identity::digest_valid;
use serde::{Deserialize, Deserializer, Serialize};

use super::production::{IoModelContext, IoSnapshotProvider, PublishedResultBytes};
use super::receive::{Admission, Coverage, FieldData, ResultEnvelope};
use super::run::SolveOutcome;
use super::transaction::{ReceiveProfile, ReceiverHost};
use super::{ContractError, EvidenceKind, RunKey, SourceRef};

pub const SCHEMA: &str = "fieldcraft-external-receive-context/1";
pub const MAX_BYTES: u64 = 256 * 1024 * 1024;
pub type FrozenReceiveProfile = ReceiveProfile;

/// Option fields must be present; null is explicit, omission is a version error.
pub(super) fn required_option<'de, D: Deserializer<'de>, T: Deserialize<'de>>(d: D) -> Result<Option<T>, D::Error> {
    Option::deserialize(d)
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommittedOnly {
    Committed,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UnverifiedOnly {
    Unverified,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PublicationReceipt {
    pub state: CommittedOnly,
    pub result_id: String,
    pub payload_sha256: String,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UnverifiedQualification {
    pub state: UnverifiedOnly,
    pub reason: String,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RetainedAvailability {
    Embedded,
    IdentityOnly,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RetainedSource {
    pub locator: String,
    pub role: String,
    pub sha256: String,
    pub bytes: u64,
    pub availability: RetainedAvailability,
    #[serde(deserialize_with = "required_option")]
    pub retained_bytes: Option<Vec<u8>>,
    pub verification_scope: String,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReceiverIdentity {
    #[serde(deserialize_with = "required_option")]
    pub source_commit: Option<String>,
    #[serde(deserialize_with = "required_option")]
    pub program: Option<SourceRef>,
    #[serde(deserialize_with = "required_option")]
    pub unknown_reason: Option<String>,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReceiveContextV1 {
    pub schema: String,
    pub run: RunKey,
    pub profile: FrozenReceiveProfile,
    pub publication: PublicationReceipt,
    pub runtime_qualification: UnverifiedQualification,
    pub source_inventory: Vec<RetainedSource>,
    pub receiver_identity: ReceiverIdentity,
}

pub fn relative_locator(name: &str) -> bool {
    !name.is_empty()
        && !name.contains(['\\', '#', '\0'])
        && name.len() <= 4096
        && Path::new(name).components().all(|c| matches!(c, Component::Normal(_)))
}
impl RetainedSource {
    pub fn validate(&self) -> Result<(), ContractError> {
        if !relative_locator(&self.locator)
            || !digest_valid(&self.sha256)
            || !super::token_valid(&self.role)
            || !super::token_valid(&self.verification_scope)
            || self.bytes > MAX_BYTES
        {
            return Err(ContractError::Invalid("ExternalArchiveContext: source shape"));
        }
        match (&self.availability, &self.retained_bytes) {
            (RetainedAvailability::Embedded, Some(bytes)) if bytes.len() as u64 == self.bytes && fieldcraft_io::sha256_hex(bytes) == self.sha256 => {
                Ok(())
            }
            (RetainedAvailability::IdentityOnly, None) => Ok(()),
            _ => Err(ContractError::Invalid("ExternalArchiveContext: source bytes")),
        }
    }
}
impl ReceiveContextV1 {
    pub fn decode(bytes: &[u8]) -> Result<Self, ContractError> {
        if bytes.len() as u64 > MAX_BYTES {
            return Err(ContractError::Invalid("ExternalArchiveContext: size"));
        }
        serde_json::from_slice(bytes).map_err(|e| ContractError::Provider(format!("ExternalArchiveContext: {e}")))
    }

    pub fn validate(&self) -> Result<(), ContractError> {
        if self.schema != SCHEMA || !super::token_valid(&self.runtime_qualification.reason) || self.source_inventory.len() > 100_000 {
            return Err(ContractError::Invalid("ExternalArchiveContext: schema or reason"));
        }
        self.run.validate()?;
        self.profile.goal.validate()?;
        if !digest_valid(&self.publication.payload_sha256)
            || self.publication.result_id != format!("fieldcraft-result/{}", self.publication.payload_sha256)
        {
            return Err(ContractError::Invalid("ExternalArchiveContext: publication"));
        }
        let mut seen = BTreeSet::new();
        let mut total = 0_u64;
        for s in &self.source_inventory {
            s.validate()?;
            if !seen.insert(&s.locator) {
                return Err(ContractError::Invalid("ExternalArchiveContext: duplicate source"));
            }
            if s.retained_bytes.is_some() {
                total = total.checked_add(s.bytes).ok_or(ContractError::Invalid("source size overflow"))?;
            }
        }
        if total > MAX_BYTES {
            return Err(ContractError::Invalid("ExternalArchiveContext: retained size"));
        }
        self.receiver_identity.validate()?;
        self.check_controls()?;
        Ok(())
    }

    fn check_controls(&self) -> Result<(), ContractError> {
        let check = |locator: &str, sha: &str| -> Result<(), ContractError> {
            let name = locator.split('#').next().unwrap_or_default();
            if !self.source_inventory.iter().any(|s| s.locator == name && s.sha256 == sha && s.retained_bytes.is_some()) {
                return Err(ContractError::Invalid("ExternalArchiveSource: control origin not retained"));
            }
            Ok(())
        };
        if let Some(c) = &self.profile.control_evidence {
            check(&c.record_locator, &c.record_sha256)?;
            if let Some(sha) = &c.control_sha256 {
                check(&c.control_locator, sha)?;
            }
            if let super::Observed::Known { source, .. } = &c.kappa_effective_pa {
                check(&source.locator, &source.sha256)?;
            }
            if let super::Observed::Known { source, .. } = &c.kappa_clamped {
                check(&source.locator, &source.sha256)?;
            }
        }
        Ok(())
    }

    /// Re-run original admission using the current document and retained original identities.
    /// No publication, launch or observation of the current machine occurs here.
    pub fn revalidate(&self, model: IoModelContext, raw: &PublishedResultBytes) -> Result<(ResultEnvelope, Admission), ContractError> {
        self.validate()?;
        if fieldcraft_io::sha256_hex(&raw.envelope_json) != raw.payload_sha256 {
            return Err(ContractError::ArchivePayloadMismatch);
        }
        if self.publication.result_id != raw.result_id || self.publication.payload_sha256 != raw.payload_sha256 {
            return Err(ContractError::ArchivePublicationMismatch);
        }
        let envelope: ResultEnvelope = serde_json::from_slice(&raw.envelope_json).map_err(|e| ContractError::Provider(e.to_string()))?;
        let prior: Admission = serde_json::from_slice(&raw.admission_json).map_err(|e| ContractError::Provider(e.to_string()))?;
        self.verify_sources(&envelope)?;
        super::package::validate_sources(&self.source_inventory, &envelope, &model)?;
        let host = ReceiverHost::new(model, self.run.clone(), IoSnapshotProvider)?;
        let tx = host.begin(&self.run, envelope.entities.clone(), self.profile.clone())?;
        let actual = tx.validate(&envelope)?;
        if actual != prior {
            return Err(ContractError::Invalid("ExternalArchiveAdmission: changed on replay"));
        }
        if actual.evidence_kind != EvidenceKind::Solver
            || actual.solve != SolveOutcome::Succeeded
            || !matches!(actual.coverage, Coverage::Complete { .. })
        {
            return Err(ContractError::Invalid("ExternalArchiveAdmission: complete solver success required"));
        }
        Ok((envelope, actual))
    }

    pub fn verify_sources(&self, e: &ResultEnvelope) -> Result<(), ContractError> {
        Self::check_source_inventory(&self.source_inventory, e)
    }

    pub(super) fn check_source_inventory(inventory: &[RetainedSource], e: &ResultEnvelope) -> Result<(), ContractError> {
        let sources: BTreeMap<_, _> = inventory.iter().map(|s| (s.locator.as_str(), s)).collect();
        let check = |r: &SourceRef| -> Result<(), ContractError> {
            r.validate()?;
            let name = r.locator.split('#').next().unwrap_or_default();
            let s = sources.get(name).ok_or(ContractError::Invalid("ExternalArchiveSource: original absent"))?;
            if s.sha256 != r.sha256 || s.retained_bytes.is_none() {
                return Err(ContractError::Invalid("ExternalArchiveSource: original not retained or mismatched"));
            }
            Ok(())
        };
        check(&e.source_scene)?;
        for f in &e.frames {
            let x = &f.facts;
            for v in [&x.completed, &x.converged, &x.hit_newton_limit, &x.hit_line_search_limit, &x.world_valid, &x.snapshot_complete] {
                if let super::Observed::Known { source, .. } = v {
                    check(source)?;
                }
            }
            for block in &f.fields {
                if let FieldData::Present { source, .. } = &block.data {
                    check(source)?;
                }
            }
        }
        if let Some(t) = &e.terminal {
            check(&t.source)?;
        }
        // Machine executables can remain identity-only: a project is not an executable bundle.
        for (role, sha) in &e.binding.artifacts {
            if !inventory.iter().any(|s| &s.role == role && &s.sha256 == sha) {
                return Err(ContractError::Invalid("ExternalArchiveSource: producer identity absent"));
            }
        }
        if !inventory.iter().any(|s| s.role == "entity_map" && s.retained_bytes.is_some())
            || !inventory.iter().any(|s| s.role == "original_input" && s.retained_bytes.is_some())
        {
            return Err(ContractError::Invalid("ExternalArchiveSource: input/map not retained"));
        }
        Ok(())
    }
}

impl ReceiverIdentity {
    pub fn validate(&self) -> Result<(), ContractError> {
        let r = self;
        if r.source_commit.as_ref().is_some_and(|s| s.len() != 40 || !s.bytes().all(|c| c.is_ascii_hexdigit() && !c.is_ascii_uppercase())) {
            return Err(ContractError::Invalid("ExternalArchiveContext: source commit"));
        }
        if (r.source_commit.is_none() || r.program.is_none()) && r.unknown_reason.as_ref().is_none_or(|s| !super::token_valid(s)) {
            return Err(ContractError::Invalid("ExternalArchiveContext: receiver identity reason"));
        }
        if let Some(program) = &r.program {
            program.validate()?;
        }
        Ok(())
    }
}
