//! Production seams for the merged FieldCraft IO identity and a receiver-owned result store.
//!
//! The provider delegates identity, contact-control decoding and native policy application to
//! `fieldcraft_io`; this module does not duplicate canonicalization.  The file store writes a
//! self-checking envelope atomically and allocates an identifier only after the write succeeds.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use serde::{Deserialize, Serialize};

use super::receive::{Admission, EntityMap, ModelBinding, ReceiverSnapshot, ResultEnvelope};
use super::transaction::SnapshotProvider;
use super::{ContractError, RunKey};

/// The current FieldCraft document and its authoritative IO run settings.
#[derive(Debug, Clone)]
pub struct IoModelContext {
    pub model: fieldcraft_model::Model,
    pub geometry: fieldcraft_io::Geometry,
    pub run: fieldcraft_io::Run,
    pub artifacts: BTreeMap<String, String>,
    pub backend_version: String,
}

impl IoModelContext {
    pub fn new(
        model: fieldcraft_model::Model,
        geometry: fieldcraft_io::Geometry,
        run: fieldcraft_io::Run,
        artifacts: BTreeMap<String, String>,
    ) -> Self {
        Self { model, geometry, run, artifacts, backend_version: fieldcraft_io::identity::BACKEND_VERSION.into() }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct IoSnapshotProvider;

fn io_map(entities: &EntityMap) -> fieldcraft_io::identity::EntityMap {
    fieldcraft_io::identity::EntityMap {
        nodes: entities.nodes.clone(),
        elements: entities
            .elements
            .iter()
            .map(|element| fieldcraft_io::identity::MappedElement {
                target: element.target,
                kind: element.kind,
                source_nodes: element.source_nodes.clone(),
            })
            .collect(),
    }
}

fn binding(binding: fieldcraft_io::identity::Binding) -> ModelBinding {
    ModelBinding {
        snapshot: ReceiverSnapshot {
            scheme: binding.snapshot.scheme,
            model_id: binding.snapshot.model_id,
            model_revision: binding.snapshot.model_revision,
            reference_mesh_digest: binding.snapshot.reference_mesh_digest,
        },
        entity_map_digest: binding.entity_map_digest,
        artifacts: binding.artifacts,
        backend_version: binding.backend_version,
    }
}

fn io_error(e: fieldcraft_io::Error) -> ContractError {
    ContractError::Provider(e.to_string())
}

fn policy_error(e: fieldcraft_io::Error) -> ContractError {
    if e.to_string().contains("NativeContactPolicyNotApplied") {
        ContractError::NativeContactPolicyNotApplied
    } else {
        ContractError::Provider(format!("PolicyRequest: {e}"))
    }
}

impl SnapshotProvider for IoSnapshotProvider {
    type Model = IoModelContext;

    fn freeze(&self, current: &Self::Model, entities: &EntityMap) -> Result<ModelBinding, ContractError> {
        fieldcraft_io::identity::Binding::freeze(
            &current.model,
            &current.geometry,
            &current.run,
            &io_map(entities),
            current.artifacts.clone(),
            &current.backend_version,
        )
        .map(binding)
        .map_err(io_error)
    }

    fn validate(&self, current: &Self::Model, frozen: &ModelBinding, entities: &EntityMap) -> Result<(), ContractError> {
        let expected = fieldcraft_io::identity::Binding {
            snapshot: fieldcraft_io::identity::Snapshot {
                scheme: frozen.snapshot.scheme.clone(),
                model_id: frozen.snapshot.model_id.clone(),
                model_revision: frozen.snapshot.model_revision.clone(),
                reference_mesh_digest: frozen.snapshot.reference_mesh_digest.clone(),
            },
            entity_map_digest: frozen.entity_map_digest.clone(),
            artifacts: frozen.artifacts.clone(),
            backend_version: frozen.backend_version.clone(),
        };
        expected
            .validate(&current.model, &current.geometry, &current.run, &io_map(entities), current.artifacts.clone(), &current.backend_version)
            .map_err(|e| ContractError::IdentityMismatch(e.to_string()))
    }

    fn validate_controls(&self, _current: &Self::Model, _run: &RunKey, bytes: &[u8], require_effective: bool) -> Result<(), ContractError> {
        fieldcraft_io::capabilities::ContactControls::decode(bytes, require_effective)
            .map(|_| ())
            .map_err(|e| ContractError::Provider(format!("ContactControls: {e}")))
    }

    fn validate_policy_declaration(&self, current: &Self::Model, _run: &RunKey, bytes: &[u8], request_digest: &str) -> Result<(), ContractError> {
        fieldcraft_io::contact_policy::PolicyRequest::decode(bytes, request_digest, &current.model, &current.geometry, &current.run)
            .map(|_| ())
            .map_err(policy_error)
    }

    fn validate_policy_application(&self, current: &Self::Model, _run: &RunKey, bytes: &[u8], request_digest: &str) -> Result<(), ContractError> {
        let request = fieldcraft_io::contact_policy::PolicyRequest::decode(bytes, request_digest, &current.model, &current.geometry, &current.run)
            .map_err(policy_error)?;
        request.require_applied(&current.model, &current.geometry, &current.run).map_err(policy_error)
    }
}

/// Receiver-owned persistence. The identifier is returned only after an atomic write succeeds.
pub trait ResultStore: Send + Sync {
    fn publish(&self, envelope: &ResultEnvelope, admission: &Admission) -> Result<String, ContractError>;
    fn reopen(&self, result_id: &str) -> Result<ResultEnvelope, ContractError>;
}

#[derive(Debug, Clone)]
pub struct FileResultStore {
    root: PathBuf,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct StoredResult {
    schema: String,
    result_id: String,
    payload_sha256: String,
    envelope: ResultEnvelope,
    admission: Admission,
}

impl FileResultStore {
    pub fn new(root: impl Into<PathBuf>) -> Result<Self, ContractError> {
        let store = Self { root: root.into() };
        fieldcraft_io::fs::create_dir_all(&store.root).map_err(io_error)?;
        Ok(store)
    }

    fn path_for(&self, result_id: &str) -> Result<PathBuf, ContractError> {
        let digest = result_id.strip_prefix("fieldcraft-result/").ok_or(ContractError::Invalid("result identifier scheme"))?;
        if digest.len() != 64 || !digest.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)) {
            return Err(ContractError::Invalid("result identifier digest"));
        }
        Ok(self.root.join(format!("{digest}.json")))
    }
}

impl ResultStore for FileResultStore {
    fn publish(&self, envelope: &ResultEnvelope, admission: &Admission) -> Result<String, ContractError> {
        if admission.evidence_kind != super::EvidenceKind::Solver
            || admission.solve != super::run::SolveOutcome::Succeeded
            || !matches!(admission.coverage, super::receive::Coverage::Complete { .. })
        {
            return Err(ContractError::Invalid("result store requires a complete solver success"));
        }
        let payload = serde_json::to_vec(envelope).map_err(|e| ContractError::Provider(format!("serialize result: {e}")))?;
        let payload_sha256 = fieldcraft_io::sha256_hex(&payload);
        let result_id = format!("fieldcraft-result/{payload_sha256}");
        let stored = StoredResult {
            schema: "fieldcraft-external-result/1".into(),
            result_id: result_id.clone(),
            payload_sha256,
            envelope: envelope.clone(),
            admission: admission.clone(),
        };
        let bytes = serde_json::to_vec(&stored).map_err(|e| ContractError::Provider(format!("serialize stored result: {e}")))?;
        let path = self.path_for(&result_id)?;
        if fieldcraft_io::fs::is_file(&path) {
            let existing = fieldcraft_io::fs::read_capped(&path, fieldcraft_io::MAX_FILE_BYTES).map_err(io_error)?;
            if existing != bytes {
                return Err(ContractError::Invalid("result identifier collision"));
            }
            return Ok(result_id);
        }
        fieldcraft_io::fs::write_atomic(&path, &bytes, false).map_err(io_error)?;
        Ok(result_id)
    }

    fn reopen(&self, result_id: &str) -> Result<ResultEnvelope, ContractError> {
        let path = self.path_for(result_id)?;
        let bytes = fieldcraft_io::fs::read_capped(&path, fieldcraft_io::MAX_FILE_BYTES).map_err(io_error)?;
        let stored: StoredResult = serde_json::from_slice(&bytes).map_err(|e| ContractError::Provider(format!("decode stored result: {e}")))?;
        if stored.schema != "fieldcraft-external-result/1" || stored.result_id != result_id {
            return Err(ContractError::Invalid("stored result identity"));
        }
        let payload = serde_json::to_vec(&stored.envelope).map_err(|e| ContractError::Provider(format!("serialize reopened result: {e}")))?;
        let digest = fieldcraft_io::sha256_hex(&payload);
        if digest != stored.payload_sha256 || result_id.strip_prefix("fieldcraft-result/") != Some(digest.as_str()) {
            return Err(ContractError::ChecksumMismatch);
        }
        Ok(stored.envelope)
    }
}

pub type SharedResultStore = Arc<dyn ResultStore>;
