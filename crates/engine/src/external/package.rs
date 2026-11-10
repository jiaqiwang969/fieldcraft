//! Public receiving package preparation, independent of the project archive codec.
//! Expected policy/artifacts are read separately, before opening the producer envelope.
use super::context::*;
use super::production::{FileResultStore, IoModelContext, IoSnapshotProvider, PublishedResultBytes};
use super::receive::{Admission, EntityMap, ResultEnvelope};
use super::transaction::{ReceiveProfile, ReceiverHost, SnapshotProvider};
use super::{ContractError, RunKey};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;

pub const PACKAGE_SCHEMA: &str = "fieldcraft-external-package/1";
pub const EXPECTED_SCHEMA: &str = "fieldcraft-external-receive-request/1";
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FileSpec {
    pub locator: String,
    pub role: String,
    pub sha256: String,
    pub bytes: u64,
    pub availability: RetainedAvailability,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PackageManifest {
    pub schema: String,
    pub envelope: FileSpec,
    pub sources: Vec<FileSpec>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ExpectedReceive {
    pub schema: String,
    pub run: RunKey,
    pub profile: ReceiveProfile,
    pub artifacts: BTreeMap<String, String>,
}

/// A checked package has no authority to execute commands or silently lower its profile.
pub struct PreparedReceive {
    model: IoModelContext,
    expected: ExpectedReceive,
    envelope: ResultEnvelope,
    sources: Vec<RetainedSource>,
    receiver: ReceiverIdentity,
}
pub struct PublishedReceive {
    pub raw: PublishedResultBytes,
    pub context: ReceiveContextV1,
    pub envelope: ResultEnvelope,
    pub admission: Admission,
}
fn decode<T: for<'de> Deserialize<'de>>(bytes: &[u8]) -> Result<T, ContractError> {
    serde_json::from_slice(bytes).map_err(|e| ContractError::Provider(format!("ExternalPackage: {e}")))
}
impl ExpectedReceive {
    pub fn decode(bytes: &[u8]) -> Result<Self, ContractError> {
        if bytes.len() as u64 > MAX_BYTES {
            return Err(ContractError::Invalid("expected profile size"));
        }
        let v: Self = decode(bytes)?;
        if v.schema != EXPECTED_SCHEMA {
            return Err(ContractError::Invalid("expected profile schema"));
        }
        v.run.validate()?;
        v.profile.goal.validate()?;
        v.profile.source_scene.validate()?;
        Ok(v)
    }
}
impl PreparedReceive {
    /// `read` must enforce the host's file roots and the supplied size cap. All source
    /// bytes, including identity-only executable files, are checked at this boundary.
    pub fn read(
        mut model: IoModelContext,
        expected: ExpectedReceive,
        manifest: PackageManifest,
        receiver: ReceiverIdentity,
        mut read: impl FnMut(&str, u64) -> Result<Vec<u8>, ContractError>,
    ) -> Result<Self, ContractError> {
        if expected.schema != EXPECTED_SCHEMA || manifest.schema != PACKAGE_SCHEMA || manifest.sources.len() > 100_000 {
            return Err(ContractError::Invalid("ExternalPackage: schema/size"));
        }
        let mut load = |spec: &FileSpec| -> Result<Vec<u8>, ContractError> {
            if !relative_locator(&spec.locator) || spec.bytes > MAX_BYTES || !fieldcraft_io::identity::digest_valid(&spec.sha256) {
                return Err(ContractError::Invalid("ExternalPackage: file shape"));
            }
            let bytes = read(&spec.locator, spec.bytes)?;
            if bytes.len() as u64 != spec.bytes || fieldcraft_io::sha256_hex(&bytes) != spec.sha256 {
                return Err(ContractError::ChecksumMismatch);
            }
            Ok(bytes)
        };
        let envelope: ResultEnvelope = decode(&load(&manifest.envelope)?)?;
        model.artifacts = expected.artifacts.clone();
        let mut sources = Vec::new();
        let mut total = 0u64;
        for spec in &manifest.sources {
            let bytes = load(spec)?;
            let retained = spec.availability == RetainedAvailability::Embedded;
            if retained {
                total = total.checked_add(spec.bytes).ok_or(ContractError::Invalid("source size"))?;
            }
            if total > MAX_BYTES {
                return Err(ContractError::Invalid("retained source size"));
            }
            sources.push(RetainedSource {
                locator: spec.locator.clone(),
                role: spec.role.clone(),
                sha256: spec.sha256.clone(),
                bytes: spec.bytes,
                availability: spec.availability.clone(),
                retained_bytes: retained.then_some(bytes),
                verification_scope:
                    "original bytes checked by receiving command; reopening checks retained identities, not current producer binaries".into(),
            });
        }
        // No result id or publication receipt is allocated at preparation.
        validate_sources(&sources, &envelope, &model)?;
        receiver.validate()?;
        let host = ReceiverHost::new(model.clone(), expected.run.clone(), IoSnapshotProvider)?;
        let tx = host.begin(&expected.run, envelope.entities.clone(), expected.profile.clone())?;
        tx.validate(&envelope)?;
        Ok(Self { model, expected, envelope, sources, receiver })
    }

    pub fn publish(self, store: Arc<FileResultStore>) -> Result<PublishedReceive, ContractError> {
        let host = ReceiverHost::with_store(self.model.clone(), self.expected.run.clone(), IoSnapshotProvider, store.clone())?;
        let tx = host.begin(&self.expected.run, self.envelope.entities.clone(), self.expected.profile.clone())?;
        let id = tx.commit(&self.envelope)?;
        let after = || -> Result<PublishedReceive, ContractError> {
            let raw = store.read_published(&id)?;
            let context = ReceiveContextV1 {
                schema: SCHEMA.into(),
                run: self.expected.run,
                profile: self.expected.profile,
                publication: PublicationReceipt {
                    state: CommittedOnly::Committed,
                    result_id: id.clone(),
                    payload_sha256: raw.payload_sha256.clone(),
                },
                runtime_qualification: UnverifiedQualification {
                    state: UnverifiedOnly::Unverified,
                    reason: "native frame completion/receiving integrity is not runtime qualification; no qualification provider is available".into(),
                },
                source_inventory: self.sources,
                receiver_identity: self.receiver,
            };
            let (envelope, admission) = context.revalidate(self.model, &raw)?;
            Ok(PublishedReceive { raw, context, envelope, admission })
        };
        after().map_err(|e| ContractError::Provider(format!("ExternalPublishedNotInstalled: result_id={id}; project_saved=false; {e}")))
    }
}

pub(super) fn validate_sources(sources: &[RetainedSource], e: &ResultEnvelope, model: &IoModelContext) -> Result<(), ContractError> {
    let mut locators = std::collections::BTreeSet::new();
    for s in sources {
        s.validate()?;
        if !locators.insert(&s.locator) {
            return Err(ContractError::Invalid("duplicate source locator"));
        }
    }
    let original = |role: &str| -> Result<&[u8], ContractError> {
        let found: Vec<_> = sources.iter().filter(|s| s.role == role).collect();
        if found.len() != 1 {
            return Err(ContractError::Invalid("original input/map role must be unique"));
        }
        found[0].retained_bytes.as_deref().ok_or(ContractError::Invalid("original input/map must be retained"))
    };
    let entities: EntityMap = decode(original("entity_map")?)?;
    if entities != e.entities {
        return Err(ContractError::IdentityMismatch("ExternalPackage entity map".into()));
    }
    let input = fieldcraft_io::read(original("original_input")?).map_err(|e| ContractError::Provider(e.to_string()))?;
    let input = IoModelContext::new(input.model, input.geometry, input.run, model.artifacts.clone());
    IoSnapshotProvider.validate(&input, &e.binding, &e.entities)?;
    ReceiveContextV1::check_source_inventory(sources, e)
}
