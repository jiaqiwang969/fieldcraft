//! Explicit v2 package admission and owned type-13 sources. No full trajectory Vec.
use super::archive::{io_binding, io_map};
use super::chunked::{self, ChunkFile, ChunkSpec};
use super::context::{CommittedOnly, PublicationReceipt, ReceiverIdentity, RetainedAvailability, UnverifiedOnly, UnverifiedQualification};
use super::package::{ExpectedReceive, FileSpec};
use super::production::{IoModelContext, IoSnapshotProvider};
use super::receive::*;
use super::run::SolveOutcome;
use super::transaction::{ReceiverHost, SnapshotProvider};
use super::{ContractError, EvidenceKind, Observed, SourceRef};
use fieldcraft_io::external_results::{ArchivedChunkedTrajectory, ArchivedChunkedTrajectoryParts, BlobDescriptor};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    io::Read,
};

pub const PACKAGE_SCHEMA: &str = "fieldcraft-external-package/2";
pub const REQUEST_SCHEMA: &str = "fieldcraft-external-receive-request/2";
pub const CONTEXT_SCHEMA: &str = "fieldcraft-external-receive-context/2";
pub const METADATA_CAP: u64 = 16 * 1024 * 1024;
const SOURCE_CAP: u64 = 256 * 1024 * 1024;

fn fail(s: &'static str) -> ContractError {
    ContractError::Invalid(s)
}
fn err(e: impl std::fmt::Display) -> ContractError {
    ContractError::Provider(e.to_string())
}
fn decode<T: for<'a> Deserialize<'a>>(b: &[u8], cap: u64) -> Result<T, ContractError> {
    if b.len() as u64 > cap {
        return Err(fail("ExternalPackageLimit: metadata"));
    }
    serde_json::from_slice(b).map_err(err)
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Acceptance {
    CompleteOnly,
    PreserveQualifiedPrefix,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Request {
    pub schema: String,
    pub expected: ExpectedReceive,
    pub acceptance: Acceptance,
}
impl Request {
    pub fn decode(b: &[u8]) -> Result<Self, ContractError> {
        let v: Self = decode(b, METADATA_CAP)?;
        if v.schema != REQUEST_SCHEMA {
            return Err(fail("ExternalPackage: request schema"));
        }
        ExpectedReceive::decode(&serde_json::to_vec(&v.expected).map_err(err)?)?;
        Ok(v)
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub schema: String,
    pub header: ChunkFile,
    pub chunks: Vec<ChunkSpec>,
    pub sources: Vec<FileSpec>,
    #[serde(deserialize_with = "super::context::required_option")]
    pub terminal: Option<TerminalClaim>,
}
impl Manifest {
    pub fn decode(b: &[u8]) -> Result<Self, ContractError> {
        let v: Self = decode(b, METADATA_CAP)?;
        v.validate(b.len() as u64)?;
        Ok(v)
    }
    fn validate(&self, initial: u64) -> Result<(), ContractError> {
        if self.schema != PACKAGE_SCHEMA || self.chunks.len() > 8192 || self.sources.len() > 4096 {
            return Err(fail("ExternalPackageLimit: schema or count"));
        }
        let mut locators = BTreeSet::new();
        let mut total = initial;
        let mut add = |file: &ChunkFile, cap: u64| -> Result<(), ContractError> {
            file.validate(cap)?;
            if !locators.insert(file.locator.clone()) {
                return Err(fail("ExternalChunkLayout: duplicate locator"));
            }
            total = total.checked_add(file.bytes).filter(|n| *n <= fieldcraft_io::MAX_FILE_BYTES).ok_or(fail("ExternalPackageLimit: total"))?;
            Ok(())
        };
        add(&self.header, METADATA_CAP)?;
        let mut next = 1;
        for (i, c) in self.chunks.iter().enumerate() {
            if c.index as usize != i
                || c.first_frame != next
                || c.last_frame < c.first_frame
                || c.last_frame - c.first_frame >= 256
                || c.last_frame > 1_000_000
            {
                return Err(fail("ExternalChunkOrder: manifest range"));
            }
            next = c.last_frame + 1;
            add(&c.metadata, chunked::METADATA_CAP)?;
            add(&c.binary, chunked::BINARY_CAP)?;
        }
        let mut prior = None;
        for s in &self.sources {
            if prior.is_some_and(|p: &str| p >= s.locator.as_str()) {
                return Err(fail("ExternalChunkLayout: source order"));
            }
            prior = Some(s.locator.as_str());
            add(&source_file(s), SOURCE_CAP)?;
        }
        Ok(())
    }
    fn embedded(&self) -> Vec<ChunkFile> {
        let mut out = vec![self.header.clone()];
        for c in &self.chunks {
            out.push(c.metadata.clone());
            out.push(c.binary.clone());
        }
        out.extend(self.sources.iter().filter(|s| s.availability == RetainedAvailability::Embedded).map(source_file));
        out
    }
}
fn source_file(s: &FileSpec) -> ChunkFile {
    ChunkFile { locator: s.locator.clone(), role: s.role.clone(), bytes: s.bytes, sha256: s.sha256.clone() }
}

/// Typed metadata keeps hostile array declarations out of an intermediate Value tree.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct WireHeader {
    schema: String,
    schema_version: u32,
    binding: ModelBinding,
    run: super::RunKey,
    entities: EntityMap,
    goal: super::run::RunGoal,
    evidence_kind: EvidenceKind,
    source_scene: SourceRef,
    contact_controls_sha256: Option<String>,
    contact_control_record_sha256: Option<String>,
    policy_request_digest: Option<String>,
    required_fields: Vec<Quantity>,
}
fn header(bytes: &[u8]) -> Result<ResultEnvelope, ContractError> {
    let h: WireHeader = decode(bytes, METADATA_CAP)?;
    if h.schema != "fieldcraft-external-header/2" {
        return Err(fail("ExternalChunkLayout: header schema"));
    }
    Ok(ResultEnvelope {
        schema_version: h.schema_version,
        binding: h.binding,
        run: h.run,
        entities: h.entities,
        goal: h.goal,
        evidence_kind: h.evidence_kind,
        source_scene: h.source_scene,
        contact_controls_sha256: h.contact_controls_sha256,
        contact_control_record_sha256: h.contact_control_record_sha256,
        policy_request_digest: h.policy_request_digest,
        required_fields: h.required_fields,
        frames: vec![],
        terminal: None,
    })
}
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct QuantityCoverage {
    pub present: u64,
    pub partial_mask: u64,
    pub unavailable: u64,
}
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Summary {
    pub observed_frames: u64,
    pub first_failed_facts: Option<super::run::FrameFacts>,
    pub quantities: BTreeMap<Quantity, QuantityCoverage>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Context {
    pub schema: String,
    pub request: Request,
    pub publication: PublicationReceipt,
    pub runtime_qualification: UnverifiedQualification,
    pub receiver_identity: ReceiverIdentity,
    pub summary: Summary,
}

fn allow(a: &Admission, policy: Acceptance) -> Result<(), ContractError> {
    if a.policy.as_ref().is_some_and(|p| p.runtime_qualification != EvidenceCheck::Verified) {
        return Err(fail("RuntimeQualificationUnverified: partial persistence cannot qualify a policy-dependent trajectory"));
    }
    let complete = matches!(a.coverage, Coverage::Complete { .. }) && a.solve == SolveOutcome::Succeeded;
    let partial = matches!(a.coverage,Coverage::QualifiedPrefix{last,..} if last>0)
        && matches!(a.solve, SolveOutcome::Failed | SolveOutcome::Cancelled)
        && a.first_rejected_frame.is_some();
    if a.evidence_kind != EvidenceKind::Solver || !a.sealed || !(complete || policy == Acceptance::PreserveQualifiedPrefix && partial) {
        return Err(fail("ExternalTrajectoryAdmission: frozen policy requires complete success or an explicit sealed failed/cancelled prefix"));
    }
    Ok(())
}
fn source_refs(m: &Manifest, h: &ResultEnvelope, frames: &[ResultFrame]) -> Result<(), ContractError> {
    let sources: BTreeMap<_, _> = m.sources.iter().map(|s| (s.locator.as_str(), s)).collect();
    let check = |r: &SourceRef| -> Result<(), ContractError> {
        r.validate()?;
        let name = r.locator.split('#').next().unwrap_or_default();
        let s = sources.get(name).ok_or(fail("ExternalArchiveSource: original absent"))?;
        if s.sha256 != r.sha256 || s.availability != RetainedAvailability::Embedded {
            return Err(fail("ExternalArchiveSource: original not retained or mismatched"));
        }
        Ok(())
    };
    check(&h.source_scene)?;
    if let Some(t) = &m.terminal {
        check(&t.source)?;
    }
    for f in frames {
        let x = &f.facts;
        for v in [&x.completed, &x.converged, &x.hit_newton_limit, &x.hit_line_search_limit, &x.world_valid, &x.snapshot_complete] {
            if let Observed::Known { source, .. } = v {
                check(source)?;
            }
        }
        for field in &f.fields {
            if let FieldData::Present { source, .. } = &field.data {
                check(source)?;
            }
        }
    }
    for (role, sha) in &h.binding.artifacts {
        if !m.sources.iter().any(|s| &s.role == role && &s.sha256 == sha) {
            return Err(fail("ExternalArchiveSource: producer identity absent"));
        }
    }
    for s in &m.sources {
        if s.availability == RetainedAvailability::IdentityOnly && h.binding.artifacts.get(&s.role) != Some(&s.sha256) {
            return Err(fail("ExternalArchiveSource: identity-only must be a bound program artifact"));
        }
    }
    Ok(())
}
fn check_controls(m: &Manifest, r: &Request) -> Result<(), ContractError> {
    let check = |loc: &str, sha: &str| -> Result<(), ContractError> {
        let name = loc.split('#').next().unwrap_or_default();
        if !m.sources.iter().any(|s| s.locator == name && s.sha256 == sha && s.availability == RetainedAvailability::Embedded) {
            return Err(fail("ExternalArchiveSource: control origin not retained"));
        }
        Ok(())
    };
    if let Some(c) = &r.expected.profile.control_evidence {
        check(&c.record_locator, &c.record_sha256)?;
        if let Some(sha) = &c.control_sha256 {
            check(&c.control_locator, sha)?;
        }
        if let Observed::Known { source, .. } = &c.kappa_effective_pa {
            check(&source.locator, &source.sha256)?;
        }
        if let Observed::Known { source, .. } = &c.kappa_clamped {
            check(&source.locator, &source.sha256)?;
        }
    }
    Ok(())
}
fn unique_source<'a>(m: &'a Manifest, role: &str) -> Result<&'a FileSpec, ContractError> {
    let mut matches = m.sources.iter().filter(|s| s.role == role && s.availability == RetainedAvailability::Embedded);
    let first = matches.next().ok_or(fail("ExternalArchiveSource: input/map not retained"))?;
    if matches.next().is_some() {
        return Err(fail("original input/map role must be unique"));
    }
    Ok(first)
}
fn index(m: &Manifest, descriptors: &[BlobDescriptor]) -> Result<BTreeMap<String, u32>, ContractError> {
    let files = m.embedded();
    if files.len() != descriptors.len() {
        return Err(fail("ExternalChunkLayout: blob count"));
    }
    let mut out = BTreeMap::new();
    for (i, (f, d)) in files.iter().zip(descriptors).enumerate() {
        if d.index as usize != i || d.role != f.role || d.bytes != f.bytes || d.sha256 != f.sha256 {
            return Err(fail("ExternalChunkLayout: blob identity"));
        }
        out.insert(f.locator.clone(), d.index);
    }
    Ok(out)
}
fn read_member<'a>(
    spec: &ChunkFile,
    idx: &BTreeMap<String, u32>,
    open: &mut impl FnMut(u32) -> Result<Box<dyn Read + 'a>, ContractError>,
) -> Result<Vec<u8>, ContractError> {
    let i = idx.get(&spec.locator).ok_or(fail("ExternalArchiveSource: member absent"))?;
    chunked::read_exact_file(spec, &mut *open(*i)?)
}
fn verify<'a>(
    m: &Manifest,
    h: &ResultEnvelope,
    request: &Request,
    model: &IoModelContext,
    idx: &BTreeMap<String, u32>,
    mut open: impl FnMut(u32) -> Result<Box<dyn Read + 'a>, ContractError>,
    publish: impl FnOnce(Admission, Summary) -> Result<ArchivedChunkedTrajectory, ContractError>,
) -> Result<ArchivedChunkedTrajectory, ContractError> {
    // IO opens the small original model from an owned temporary file; even a large
    // source is copied with one fixed buffer, never a Vec of its complete bytes.
    let input = unique_source(m, "original_input")?;
    let input_i = *idx.get(&input.locator).ok_or(fail("ExternalArchiveSource: input absent"))?;
    let original = super::chunked_store::read_original_model(&mut *open(input_i)?, input)?;
    let original = IoModelContext::new(original.model, original.geometry, original.run, request.expected.artifacts.clone());
    IoSnapshotProvider.validate(&original, &h.binding, &h.entities)?;
    let map = unique_source(m, "entity_map")?;
    // Entity descriptions use the header metadata budget, not the raw-source budget.
    source_file(map).validate(METADATA_CAP)?;
    let entities: EntityMap = decode(&read_member(&source_file(map), idx, &mut open)?, METADATA_CAP)?;
    if entities != h.entities {
        return Err(ContractError::IdentityMismatch("ExternalPackage entity map".into()));
    }
    source_refs(m, h, &[])?;
    check_controls(m, request)?;
    let host = ReceiverHost::new(model.clone(), request.expected.run.clone(), IoSnapshotProvider)?;
    let tx = host.begin(&request.expected.run, h.entities.clone(), request.expected.profile.clone())?;
    tx.with_validated_stream(
        h,
        m.terminal.as_ref(),
        |stream| {
            let mut summary = Summary::default();
            for (i, spec) in m.chunks.iter().enumerate() {
                let frames = chunked::read_chunk(i as u32, spec, &h.entities, |f| {
                    open(*idx.get(&f.locator).ok_or(fail("ExternalChunkLayout: chunk absent"))?)
                })?;
                source_refs(m, h, &frames)?;
                stream.push_chunk(&frames)?;
                summary.observed_frames += frames.len() as u64;
                for frame in &frames {
                    if summary.first_failed_facts.is_none() && frame.facts.issue(h.evidence_kind).is_some() {
                        summary.first_failed_facts = Some(frame.facts.clone());
                    }
                    for f in &frame.fields {
                        let c = summary.quantities.entry(f.spec.quantity).or_default();
                        match &f.data {
                            FieldData::Present { available, .. } => {
                                c.present += 1;
                                if available.iter().any(|v| !*v) {
                                    c.partial_mask += 1;
                                }
                            }
                            FieldData::Unavailable { .. } => c.unavailable += 1,
                        }
                    }
                }
            }
            Ok(summary)
        },
        |admission, summary| {
            allow(&admission, request.acceptance)?;
            publish(admission, summary)
        },
    )
}

pub fn reopen(record: &ArchivedChunkedTrajectory, model: IoModelContext) -> Result<Context, ContractError> {
    record.validate_binding(&model.model, &model.geometry, &model.run).map_err(err)?;
    let m = Manifest::decode(record.manifest_json())?;
    let idx = index(&m, record.blobs())?;
    let context: Context = decode(record.context_json(), METADATA_CAP)?;
    if context.schema != CONTEXT_SCHEMA
        || context.publication.result_id != record.result_id()
        || context.publication.payload_sha256 != record.payload_sha256()
    {
        return Err(ContractError::ArchivePublicationMismatch);
    }
    Request::decode(&serde_json::to_vec(&context.request).map_err(err)?)?;
    context.receiver_identity.validate()?;
    if !super::token_valid(&context.runtime_qualification.reason) {
        return Err(fail("ExternalArchiveContext: qualification reason"));
    }
    let prior: Admission = decode(record.admission_json(), 1024 * 1024)?;
    let h = header(&read_member(&m.header, &idx, &mut |i| record.open_blob(i).map_err(err))?)?;
    if io_binding(&h.binding) != *record.binding() || io_map(&h.entities) != *record.entity_map() {
        return Err(ContractError::IdentityMismatch("ExternalArchive: header binding".into()));
    }
    verify(
        &m,
        &h,
        &context.request,
        &model,
        &idx,
        |i| record.open_blob(i).map_err(err),
        |actual, summary| {
            if actual != prior || summary != context.summary {
                return Err(fail("ExternalArchiveAdmission: changed on replay"));
            }
            Ok(record.clone())
        },
    )?;
    Ok(context)
}

/// Native bounded staging. Only the completed transaction is exposed to the Session.
#[cfg(not(target_arch = "wasm32"))]
pub fn receive(
    name: String,
    model: IoModelContext,
    request: Request,
    manifest_json: Vec<u8>,
    receiver: ReceiverIdentity,
    store: &std::path::Path,
    mut open: impl FnMut(&ChunkFile) -> Result<Box<dyn Read>, ContractError>,
) -> Result<ArchivedChunkedTrajectory, ContractError> {
    let m = Manifest::decode(&manifest_json)?;
    receiver.validate()?;
    let mut stage = super::chunked_store::Stage::new(store)?;
    let files = m.embedded();
    for f in &files {
        stage.append(f, &mut *open(f)?)?;
    }
    // Identity-only programs are checked at ingestion; reopening never opens producer paths.
    for s in &m.sources {
        if s.availability == RetainedAvailability::IdentityOnly {
            super::chunked_store::copy_checked(&mut *open(&source_file(s))?, &mut std::io::sink(), &source_file(s))?;
        }
    }
    let (source, blobs) = stage.source()?;
    let idx = index(&m, &blobs)?;
    let h = header(&read_member(&m.header, &idx, &mut |i| source.open(i).map_err(err))?)?;
    let payload = fieldcraft_io::sha256_hex(&manifest_json);
    let id = format!("fieldcraft-result/{payload}");
    verify(
        &m,
        &h,
        &request,
        &model,
        &idx,
        |i| source.open(i).map_err(err),
        |admission, summary| {
            let context = Context {
                schema: CONTEXT_SCHEMA.into(),
                request: request.clone(),
                publication: PublicationReceipt { state: CommittedOnly::Committed, result_id: id.clone(), payload_sha256: payload.clone() },
                runtime_qualification: UnverifiedQualification {
                    state: UnverifiedOnly::Unverified,
                    reason: "receiving and retained frame diagnostics do not establish runtime qualification".into(),
                },
                receiver_identity: receiver,
                summary,
            };
            let admission_json = serde_json::to_vec(&admission).map_err(err)?;
            let context_json = serde_json::to_vec(&context).map_err(err)?;
            let record = ArchivedChunkedTrajectory::new(ArchivedChunkedTrajectoryParts {
                archive_key: name,
                binding: io_binding(&h.binding),
                entity_map: io_map(&h.entities),
                result_id: id,
                payload_sha256: payload.clone(),
                manifest_json,
                admission_json,
                context_json,
                blobs,
                source: source.clone(),
            })
            .map_err(err)?;
            stage.publish(&payload, &record)?;
            Ok(record)
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn failed_prefix_cannot_bypass_policy_runtime_qualification() {
        let mut a = Admission {
            transfer: TransferState::Verified,
            coverage: Coverage::QualifiedPrefix { last: 1, requested_last: 3 },
            solve: SolveOutcome::Failed,
            evidence_kind: EvidenceKind::Solver,
            commit: CommitState::NotAttempted,
            first_rejected_frame: Some(2),
            blockers: vec![],
            sealed: true,
            policy: None,
            control: None,
        };
        assert!(allow(&a, Acceptance::PreserveQualifiedPrefix).is_ok());
        a.policy = Some(PolicyAdmission {
            declaration: EvidenceCheck::Verified,
            application: EvidenceCheck::Verified,
            runtime_qualification: EvidenceCheck::Unverified,
        });
        let err = allow(&a, Acceptance::PreserveQualifiedPrefix).unwrap_err().to_string();
        assert!(err.contains("RuntimeQualificationUnverified"));
    }
}
