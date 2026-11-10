//! Bounded external trajectory storage. Frame semantics remain owned by the receiver.
use crate::{Error, Geometry, Result, Run, codec, identity, invalid};
use fieldcraft_model::Model;
use serde::{Deserialize, Serialize};
pub const SCHEMA: &str = "fieldcraft-external-trajectory/1";
/// Per JSON blob cap; the combined record is also bounded by the results-part limit.
pub const MAX_JSON_BYTES: usize = 256 * 1024 * 1024;
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArchivedTrajectoryParts {
    pub archive_key: String,
    pub binding: crate::identity::Binding,
    pub entity_map: crate::identity::EntityMap,
    pub result_id: String,
    pub payload_sha256: String,
    pub envelope_json: Vec<u8>,
    pub admission_json: Vec<u8>,
    pub context_json: Vec<u8>,
}
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ArchivedTrajectory {
    pub schema: String,
    pub archive_key: String,
    pub binding: crate::identity::Binding,
    pub entity_map: crate::identity::EntityMap,
    pub result_id: String,
    pub payload_sha256: String,
    pub envelope_json: Vec<u8>,
    pub admission_json: Vec<u8>,
    pub context_json: Vec<u8>,
    pub record_sha256: String,
}
impl ArchivedTrajectory {
    pub fn new(parts: ArchivedTrajectoryParts) -> Result<Self> {
        let mut value = Self {
            schema: SCHEMA.into(),
            archive_key: parts.archive_key,
            binding: parts.binding,
            entity_map: parts.entity_map,
            result_id: parts.result_id,
            payload_sha256: parts.payload_sha256,
            envelope_json: parts.envelope_json,
            admission_json: parts.admission_json,
            context_json: parts.context_json,
            record_sha256: String::new(),
        };
        value.check_parts()?;
        value.record_sha256 = value.record_digest()?;
        value.validate()?;
        Ok(value)
    }
    /// Digest of codec fields in schema/Parts declaration order, excluding this digest.
    pub fn record_digest(&self) -> Result<String> {
        self.check_parts()?;
        let bytes = codec::encode(&(
            &self.schema,
            &self.archive_key,
            &self.binding,
            &self.entity_map,
            &self.result_id,
            &self.payload_sha256,
            &self.envelope_json,
            &self.admission_json,
            &self.context_json,
        ))
        .map_err(|e| Error::Invalid(e.to_string()))?;
        Ok(crate::sha256_hex(&bytes))
    }
    fn check_parts(&self) -> Result<()> {
        if self.schema != SCHEMA {
            return invalid("ExternalArchiveSchemaUnsupported");
        }
        if self.archive_key.trim().is_empty() || self.archive_key.len() > 1024 {
            return invalid("ExternalArchiveLimit: archive key");
        }
        for bytes in [&self.envelope_json, &self.admission_json, &self.context_json] {
            if bytes.len() > MAX_JSON_BYTES {
                return invalid("ExternalArchiveLimit: JSON bytes");
            }
            let _: serde_json::Value = serde_json::from_slice(bytes).map_err(|e| Error::Invalid(format!("ExternalArchiveJson: {e}")))?;
        }
        let snap = &self.binding.snapshot;
        if snap.scheme != identity::SCHEME {
            return invalid("UnsupportedIdentityScheme");
        }
        if self.binding.backend_version != identity::BACKEND_VERSION {
            return invalid("UnsupportedBackendVersion");
        }
        if !identity::digest_valid(&snap.model_revision)
            || !identity::digest_valid(&snap.reference_mesh_digest)
            || !identity::digest_valid(&self.binding.entity_map_digest)
            || !identity::digest_valid(&self.payload_sha256)
        {
            return invalid("ExternalArchiveDigestMismatch: digest shape");
        }
        if snap.model_id != identity::model_id(&snap.reference_mesh_digest) {
            return invalid("ModelIdentityMismatch");
        }
        if identity::REQUIRED_ARTIFACT_ROLES.iter().any(|k| !self.binding.artifacts.contains_key(*k))
            || self.binding.artifacts.values().any(|v| !identity::digest_valid(v))
        {
            return invalid("MissingOrInvalidArtifactDigest");
        }
        if crate::sha256_hex(&self.envelope_json) != self.payload_sha256 {
            return invalid("ExternalArchiveDigestMismatch: payload");
        }
        if self.result_id != format!("fieldcraft-result/{}", self.payload_sha256) {
            return invalid("ExternalArchiveResultIdMismatch");
        }
        Ok(())
    }
    pub fn validate(&self) -> Result<()> {
        if !identity::digest_valid(&self.record_sha256) || self.record_digest()? != self.record_sha256 {
            return invalid("ExternalArchiveDigestMismatch: record");
        }
        Ok(())
    }
    pub fn definition(&self) -> Result<[u8; 32]> {
        self.validate()?;
        let mut out = [0; 32];
        for (dst, pair) in out.iter_mut().zip(self.binding.snapshot.model_revision.as_bytes().as_chunks::<2>().0) {
            let text = std::str::from_utf8(pair).map_err(|e| Error::Invalid(e.to_string()))?;
            *dst = u8::from_str_radix(text, 16).map_err(|e| Error::Invalid(e.to_string()))?;
        }
        Ok(out)
    }
    pub fn validate_binding(&self, model: &Model, geometry: &Geometry, run: &Run) -> Result<()> {
        self.validate()?;
        self.binding.validate(model, geometry, run, &self.entity_map, self.binding.artifacts.clone(), &self.binding.backend_version)
    }
}

pub use crate::external_chunked::{
    ArchivedChunkedTrajectory, ArchivedChunkedTrajectoryParts, BlobDescriptor, BlobSource, CHUNKED_SCHEMA, COPY_BUFFER_BYTES,
};
