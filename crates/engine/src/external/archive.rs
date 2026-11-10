//! Project archive bridge. IO owns kind 13 and its canonical record digest.
use super::ContractError;
use super::context::ReceiveContextV1;
use super::package::PublishedReceive;
use super::production::{IoModelContext, PublishedResultBytes};
use super::receive::{Admission, EntityMap, ModelBinding, ResultEnvelope};
use fieldcraft_io::external_results::{ArchivedTrajectory, ArchivedTrajectoryParts};

pub fn io_binding(b: &ModelBinding) -> fieldcraft_io::identity::Binding {
    fieldcraft_io::identity::Binding {
        snapshot: fieldcraft_io::identity::Snapshot {
            scheme: b.snapshot.scheme.clone(),
            model_id: b.snapshot.model_id.clone(),
            model_revision: b.snapshot.model_revision.clone(),
            reference_mesh_digest: b.snapshot.reference_mesh_digest.clone(),
        },
        entity_map_digest: b.entity_map_digest.clone(),
        artifacts: b.artifacts.clone(),
        backend_version: b.backend_version.clone(),
    }
}
pub fn io_map(e: &EntityMap) -> fieldcraft_io::identity::EntityMap {
    fieldcraft_io::identity::EntityMap {
        nodes: e.nodes.clone(),
        elements: e
            .elements
            .iter()
            .map(|e| fieldcraft_io::identity::MappedElement { target: e.target, kind: e.kind, source_nodes: e.source_nodes.clone() })
            .collect(),
    }
}
pub fn from_published(name: String, published: PublishedReceive) -> Result<ArchivedTrajectory, ContractError> {
    let raw = published.raw;
    let context_json = serde_json::to_vec(&published.context).map_err(|e| ContractError::Provider(e.to_string()))?;
    ArchivedTrajectory::new(ArchivedTrajectoryParts {
        archive_key: name,
        binding: io_binding(&published.envelope.binding),
        entity_map: io_map(&published.envelope.entities),
        result_id: raw.result_id,
        payload_sha256: raw.payload_sha256,
        envelope_json: raw.envelope_json,
        admission_json: raw.admission_json,
        context_json,
    })
    .map_err(|e| ContractError::Provider(format!("ExternalArchive: {e}")))
}
pub fn reopen(record: &ArchivedTrajectory, model: IoModelContext) -> Result<(ResultEnvelope, Admission, ReceiveContextV1), ContractError> {
    record.validate_binding(&model.model, &model.geometry, &model.run).map_err(|e| ContractError::IdentityMismatch(e.to_string()))?;
    let context = ReceiveContextV1::decode(&record.context_json)?;
    let raw = PublishedResultBytes {
        result_id: record.result_id.clone(),
        payload_sha256: record.payload_sha256.clone(),
        envelope_json: record.envelope_json.clone(),
        admission_json: record.admission_json.clone(),
    };
    let (envelope, admission) = context.revalidate(model, &raw)?;
    if io_binding(&envelope.binding) != record.binding || io_map(&envelope.entities) != record.entity_map {
        return Err(ContractError::IdentityMismatch("ExternalArchive: envelope/header binding".into()));
    }
    Ok((envelope, admission, context))
}
