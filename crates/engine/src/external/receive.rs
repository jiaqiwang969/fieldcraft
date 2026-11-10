//! Bounded binary result preflight. Does not allocate result kinds or write project files.
//!
//! Identity policy is supplied by the model owner. Checksums establish byte integrity, not
//! numerical success. All arrays use explicit SI units and the FieldCraft world frame (Z up).
//! Values are compact: one row per true mask bit, with no zero/NaN placeholder rows.

use std::collections::{BTreeMap, BTreeSet};

use fieldcraft_io::identity::digest_valid;
use serde::{Deserialize, Serialize};

use super::control::ControlAdmission;
use super::run::{FrameFacts, RunGoal, RunLedger, SolveOutcome};
use super::{ContractError, EvidenceKind, RunKey, SourceRef, token_valid};

const MAX_BYTES: usize = 64 * 1024 * 1024;
const MAX_ENTITIES: usize = 100_000;
const MAX_RECEIVE_FRAMES: usize = 4096;

/// Wire representation of fieldcraft-receiver-snapshot/1; hashing belongs to the IO provider.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReceiverSnapshot {
    pub scheme: String,
    pub model_id: String,
    pub model_revision: String,
    pub reference_mesh_digest: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ModelBinding {
    pub snapshot: ReceiverSnapshot,
    pub entity_map_digest: String,
    pub artifacts: BTreeMap<String, String>,
    pub backend_version: String,
}

impl ModelBinding {
    pub fn validate_shape(&self) -> Result<(), ContractError> {
        if self.snapshot.scheme != fieldcraft_io::identity::SCHEME {
            return Err(ContractError::Unsupported("receiver identity scheme"));
        }
        if self.backend_version != fieldcraft_io::identity::BACKEND_VERSION {
            return Err(ContractError::Unsupported("backend version"));
        }
        if !digest_valid(&self.snapshot.model_revision)
            || !digest_valid(&self.snapshot.reference_mesh_digest)
            || !digest_valid(&self.entity_map_digest)
            || self.snapshot.model_id != fieldcraft_io::identity::model_id(&self.snapshot.reference_mesh_digest)
        {
            return Err(ContractError::Invalid("snapshot identity fields"));
        }
        if self.artifacts.len() > 32
            || fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES.iter().any(|k| !self.artifacts.contains_key(*k))
            || self.artifacts.iter().any(|(k, v)| !token_valid(k) || !digest_valid(v))
        {
            return Err(ContractError::Invalid("artifact roles and digests"));
        }
        Ok(())
    }
}

pub trait IdentityVerifier {
    /// Must check the receiver's current revision, reference mesh and persistent entity map.
    /// Comparing strings alone is not a production identity proof.
    fn verify(&self, expected: &ModelBinding, submitted: &ModelBinding, entities: &EntityMap) -> Result<(), ContractError>;
}

pub struct UnconfiguredIdentity;
impl IdentityVerifier for UnconfiguredIdentity {
    fn verify(&self, _: &ModelBinding, _: &ModelBinding, _: &EntityMap) -> Result<(), ContractError> {
        Err(ContractError::IdentityUnavailable)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EntityPair {
    pub source: String,
    pub target: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EntityMap {
    /// nodes[i] is the target label for dense, zero-based source node i.
    pub nodes: Vec<u32>,
    pub elements: Vec<MappedElement>,
    /// Object flattening is an independent runtime contract, outside the IO map digest.
    pub objects: Vec<EntityPair>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MappedElement {
    pub target: u32,
    pub kind: fieldcraft_model::ElementKind,
    pub source_nodes: Vec<usize>,
}

impl EntityMap {
    pub(super) fn validate(&self) -> Result<(), ContractError> {
        if self.nodes.is_empty() || [self.nodes.len(), self.elements.len(), self.objects.len()].into_iter().any(|n| n > MAX_ENTITIES) {
            return Err(ContractError::Invalid("entity count cap or empty node map"));
        }
        if self.nodes.iter().collect::<BTreeSet<_>>().len() != self.nodes.len() {
            return Err(ContractError::Invalid("duplicate node mapping"));
        }
        let mut elements = BTreeSet::new();
        for element in &self.elements {
            if !elements.insert(element.target)
                || element.source_nodes.is_empty()
                || element.source_nodes.len() > 64
                || element.source_nodes.iter().any(|i| *i >= self.nodes.len())
            {
                return Err(ContractError::Invalid("element mapping"));
            }
        }
        let mut sources = BTreeSet::new();
        let mut targets = BTreeSet::new();
        for pair in &self.objects {
            if !token_valid(&pair.source) || !token_valid(&pair.target) || !sources.insert(&pair.source) || !targets.insert(&pair.target) {
                return Err(ContractError::Invalid("object mapping must be one-to-one"));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Quantity {
    Position,
    Displacement,
    Velocity,
    ObjectForce,
    ObjectMoment,
    ContactEnergy,
    Stress,
}

const FORWARD_FIELDS: [Quantity; 6] =
    [Quantity::Position, Quantity::Displacement, Quantity::Velocity, Quantity::ObjectForce, Quantity::ObjectMoment, Quantity::ContactEnergy];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Location {
    Node,
    Object,
    Global,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FieldSpec {
    pub quantity: Quantity,
    pub location: Location,
    pub unit: String,
    pub coordinate_frame: String,
    pub components: usize,
    /// Required for object moment; positions/displacements use the bound reference mesh.
    pub moment_origin_m: Option<[f64; 3]>,
}

impl FieldSpec {
    fn validate(&self) -> Result<(), ContractError> {
        let (location, unit, components) = match self.quantity {
            Quantity::Position | Quantity::Displacement => (Location::Node, "m", 3),
            Quantity::Velocity => (Location::Node, "m/s", 3),
            Quantity::ObjectForce => (Location::Object, "N", 3),
            Quantity::ObjectMoment => (Location::Object, "N*m", 3),
            Quantity::ContactEnergy => (Location::Global, "J", 1),
            Quantity::Stress => return Err(ContractError::Unsupported("stress measure and location contract")),
        };
        if self.location != location || self.unit != unit || self.components != components || self.coordinate_frame != "world_z_up" {
            return Err(ContractError::Invalid("field units, coordinate frame, location or components"));
        }
        match (self.quantity, self.moment_origin_m) {
            (Quantity::ObjectMoment, Some(origin)) if origin.iter().all(|v| v.is_finite()) => {}
            (Quantity::ObjectMoment, _) => return Err(ContractError::Invalid("moment origin")),
            (_, None) => {}
            _ => return Err(ContractError::Invalid("unexpected moment origin")),
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "availability", rename_all = "snake_case", deny_unknown_fields)]
pub enum FieldData {
    Present {
        /// Little endian f64, compact entity rows (only true availability bits).
        bytes: Vec<u8>,
        sha256: String,
        available: Vec<bool>,
        source: SourceRef,
    },
    Unavailable {
        reason: String,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FieldBlock {
    pub spec: FieldSpec,
    pub data: FieldData,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResultFrame {
    pub facts: FrameFacts,
    pub fields: Vec<FieldBlock>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResultEnvelope {
    pub schema_version: u32,
    pub binding: ModelBinding,
    pub run: RunKey,
    pub entities: EntityMap,
    pub goal: RunGoal,
    pub evidence_kind: EvidenceKind,
    pub source_scene: SourceRef,
    /// Digest of the separately validated fieldcraft-contact-controls/1 receipt, if supplied.
    pub contact_controls_sha256: Option<String>,
    /// Digest of the complete route-20 control binding record set, if supplied.
    pub contact_control_record_sha256: Option<String>,
    /// Canonical PolicyRequest digest frozen by the receiver, distinct from the wire SHA.
    pub policy_request_digest: Option<String>,
    pub required_fields: Vec<Quantity>,
    pub frames: Vec<ResultFrame>,
    /// None means still staging; it cannot authorize a full or prefix commit.
    pub terminal: Option<TerminalClaim>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TerminalClaim {
    pub outcome: SolveOutcome,
    pub source: SourceRef,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransferState {
    Verified,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Coverage {
    Empty,
    QualifiedPrefix { last: u64, requested_last: u64 },
    Complete { last: u64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommitState {
    NotAttempted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceCheck {
    Unverified,
    Verified,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PolicyAdmission {
    pub declaration: EvidenceCheck,
    pub application: EvidenceCheck,
    pub runtime_qualification: EvidenceCheck,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Admission {
    pub transfer: TransferState,
    pub coverage: Coverage,
    pub solve: SolveOutcome,
    pub evidence_kind: EvidenceKind,
    pub commit: CommitState,
    pub first_rejected_frame: Option<u64>,
    pub blockers: Vec<String>,
    pub sealed: bool,
    pub policy: Option<PolicyAdmission>,
    pub control: Option<ControlAdmission>,
}

pub struct ExternalReceiver<I> {
    identity: I,
}

impl<I: IdentityVerifier> ExternalReceiver<I> {
    pub fn new(identity: I) -> Self {
        Self { identity }
    }

    /// Borrowed, typed preflight. Wire framing/decompression must also cap bytes before allocation.
    pub fn validate(&self, expected_model: &ModelBinding, active_run: &RunKey, envelope: &ResultEnvelope) -> Result<Admission, ContractError> {
        if envelope.schema_version != 1 {
            return Err(ContractError::Unsupported("external envelope schema"));
        }
        if envelope.frames.len() > MAX_RECEIVE_FRAMES || envelope.required_fields.len() > FORWARD_FIELDS.len() {
            return Err(ContractError::Invalid("receive size cap"));
        }
        envelope.run.validate()?;
        envelope.goal.validate()?;
        envelope.source_scene.validate()?;
        if envelope.source_scene.kind != envelope.evidence_kind {
            return Err(ContractError::Invalid("scene evidence kind"));
        }
        if envelope.policy_request_digest.as_ref().is_some_and(|d| !digest_valid(d)) {
            return Err(ContractError::Invalid("policy request digest"));
        }
        envelope.binding.validate_shape()?;
        expected_model.validate_shape()?;
        envelope.entities.validate()?;
        if active_run != &envelope.run {
            return Err(ContractError::StaleRun);
        }
        if expected_model != &envelope.binding {
            return Err(ContractError::IdentityMismatch("revision, mesh, program, backend or scheme".into()));
        }
        self.identity.verify(expected_model, &envelope.binding, &envelope.entities)?;
        let required: BTreeSet<_> = envelope.required_fields.iter().copied().collect();
        if !required.contains(&Quantity::Position) || required.len() != envelope.required_fields.len() || required.contains(&Quantity::Stress) {
            return Err(ContractError::Invalid("required field profile"));
        }
        if let Some(terminal) = &envelope.terminal {
            terminal.source.validate()?;
            if terminal.source.kind != envelope.evidence_kind || terminal.outcome == SolveOutcome::Running {
                return Err(ContractError::Invalid("terminal source"));
            }
        }
        let mut ledger = RunLedger::new(envelope.run.clone(), envelope.goal, envelope.evidence_kind)?;
        ledger.initialize(&envelope.run)?;
        ledger.begin_advance(&envelope.run, envelope.goal.last_frame)?;
        let mut bytes_total = 0usize;
        let mut first_rejected_frame = None;
        let mut blockers = Vec::new();
        let mut previous_frame = 0;
        for frame in &envelope.frames {
            if frame.facts.run != envelope.run {
                return Err(ContractError::StaleRun);
            }
            if frame.facts.frame <= previous_frame || frame.facts.frame > envelope.goal.last_frame {
                return Err(ContractError::Invalid("frames must be ordered and unique within the run goal"));
            }
            previous_frame = frame.facts.frame;
            if frame.fields.len() > FORWARD_FIELDS.len() + 1 {
                return Err(ContractError::Invalid("field count"));
            }
            let mut seen = BTreeSet::new();
            let mut missing_required = false;
            for block in &frame.fields {
                block.spec.validate()?;
                if !seen.insert(block.spec.quantity) {
                    return Err(ContractError::Invalid("duplicate field"));
                }
                let entities = match block.spec.location {
                    Location::Node => envelope.entities.nodes.len(),
                    Location::Object => envelope.entities.objects.len(),
                    Location::Global => 1,
                };
                match &block.data {
                    FieldData::Unavailable { reason } => {
                        if !token_valid(reason) {
                            return Err(ContractError::Invalid("unavailable reason"));
                        }
                        missing_required |= required.contains(&block.spec.quantity);
                    }
                    FieldData::Present { bytes, sha256, available, source } => {
                        source.validate()?;
                        if source.kind != envelope.evidence_kind {
                            return Err(ContractError::Invalid("field evidence kind"));
                        }
                        // Include masks in the work budget even when every row is unavailable.
                        bytes_total = bytes_total
                            .checked_add(bytes.len())
                            .and_then(|n| n.checked_add(available.len()))
                            .ok_or(ContractError::Invalid("byte count"))?;
                        if bytes_total > MAX_BYTES || available.len() != entities {
                            return Err(ContractError::Invalid("binary or mask size"));
                        }
                        let values = available
                            .iter()
                            .filter(|v| **v)
                            .count()
                            .checked_mul(block.spec.components)
                            .and_then(|n| n.checked_mul(8))
                            .ok_or(ContractError::Invalid("array shape"))?;
                        if bytes.len() != values {
                            return Err(ContractError::Invalid("compact array length"));
                        }
                        if fieldcraft_io::sha256_hex(bytes) != *sha256 {
                            return Err(ContractError::ChecksumMismatch);
                        }
                        for array in bytes.as_chunks::<8>().0 {
                            if !f64::from_le_bytes(*array).is_finite() {
                                return Err(ContractError::Invalid("nonfinite field value"));
                            }
                        }
                        missing_required |= required.contains(&block.spec.quantity) && (entities == 0 || available.iter().any(|v| !*v));
                    }
                }
            }
            // All six quantities must be explicitly present or unavailable; omission is never zero.
            let missing_declaration = FORWARD_FIELDS.iter().any(|q| !seen.contains(q));
            if first_rejected_frame.is_none() {
                if missing_required || missing_declaration {
                    first_rejected_frame = Some(ledger.last_qualified() + 1);
                    blockers.push("missing field declaration or required data".into());
                } else if let Err(e) = ledger.accept_frame(frame.facts.clone()) {
                    first_rejected_frame = Some(ledger.last_qualified() + 1);
                    blockers.push(e.to_string());
                }
            }
        }
        let last = ledger.last_qualified();
        let complete = last == envelope.goal.last_frame && first_rejected_frame.is_none();
        let mut solve = ledger.solve_outcome();
        if let Some(terminal) = &envelope.terminal {
            if complete && terminal.outcome == SolveOutcome::Succeeded {
                ledger.finish(&envelope.run, terminal.source.clone())?;
                solve = ledger.solve_outcome();
            } else if !matches!(solve, SolveOutcome::Failed | SolveOutcome::Unknown) {
                solve = match terminal.outcome {
                    SolveOutcome::Failed => SolveOutcome::Failed,
                    SolveOutcome::Cancelled => SolveOutcome::Cancelled,
                    _ => SolveOutcome::Unknown,
                };
            }
            if !complete && terminal.outcome == SolveOutcome::Succeeded {
                blockers.push("success claim does not cover the requested trajectory".into());
            }
        }
        if !complete && first_rejected_frame.is_none() {
            first_rejected_frame = Some(last + 1);
            blockers.push("incomplete trajectory".into());
        }
        if envelope.evidence_kind == EvidenceKind::ProtocolFixture {
            blockers.push("protocol fixture: not evidence of a real numerical solve".into());
            if solve == SolveOutcome::Succeeded {
                solve = SolveOutcome::Unknown;
            }
        }
        // Runtime qualification remains unverified even when the provider verifies application.
        let policy = envelope.policy_request_digest.as_ref().map(|_| {
            blockers.push("RuntimeQualificationUnverified: policy-dependent results require independently verified runtime qualification".into());
            if solve == SolveOutcome::Succeeded {
                solve = SolveOutcome::Unknown;
            }
            PolicyAdmission {
                declaration: EvidenceCheck::Unverified,
                application: EvidenceCheck::Unverified,
                runtime_qualification: EvidenceCheck::Unverified,
            }
        });
        Ok(Admission {
            transfer: TransferState::Verified,
            coverage: if last == 0 {
                Coverage::Empty
            } else if complete {
                Coverage::Complete { last }
            } else {
                Coverage::QualifiedPrefix { last, requested_last: envelope.goal.last_frame }
            },
            solve,
            evidence_kind: envelope.evidence_kind,
            commit: CommitState::NotAttempted,
            first_rejected_frame,
            blockers,
            sealed: envelope.terminal.is_some(),
            policy,
            control: None,
        })
    }

    /// No result identifier exists until a future receiver-owned atomic store is integrated.
    pub fn commit(&self) -> Result<(), ContractError> {
        Err(ContractError::StorageUnavailable)
    }
}
