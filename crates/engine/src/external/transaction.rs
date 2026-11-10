//! Begin/commit identity gates with one active receiver lease and a model consistency lock.
//!
//! The provider must delegate to the authoritative IO Binding::freeze/validate implementation.
//! This module supplies no replacement canonicalization or persistent storage. A fixture
//! provider is useful only for checking lock and failure behavior, not production identity.

use std::sync::{Mutex, MutexGuard};

use super::receive::{Admission, EntityMap, EvidenceCheck, ExternalReceiver, IdentityVerifier, ModelBinding, Quantity, ResultEnvelope};
use super::run::RunGoal;
use super::{ContractError, EvidenceKind, RunKey, SourceRef, digest_valid};

/// Receiver-owned expectations, frozen before accepting any producer envelope.
#[derive(Debug, Clone, PartialEq)]
pub struct ReceiveProfile {
    pub goal: RunGoal,
    pub evidence_kind: EvidenceKind,
    pub source_scene: SourceRef,
    pub required_fields: Vec<Quantity>,
    pub contact_controls: Option<ContactControlPayload>,
    /// A receiver-owned quantitative policy; never weakened after a provider refusal.
    pub require_effective: bool,
    pub policy_request: Option<PolicyRequestPayload>,
    /// This foundation refuses application-dependent admission, even with a valid declaration.
    pub require_policy_applied: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ContactControlPayload {
    pub bytes: Vec<u8>,
    pub sha256: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolicyRequestPayload {
    pub bytes: Vec<u8>,
    pub wire_sha256: String,
    /// PolicyRequest::digest at the trusted freeze boundary; pretty JSON has a different SHA.
    pub request_digest: String,
}

pub trait SnapshotProvider {
    type Model;
    /// Freeze the current model/geometry/run settings, full map and actual artifact bytes.
    fn freeze(&self, current: &Self::Model, entities: &EntityMap) -> Result<ModelBinding, ContractError>;
    /// Recompute through the authoritative provider, at both begin and commit.
    fn validate(&self, current: &Self::Model, frozen: &ModelBinding, entities: &EntityMap) -> Result<(), ContractError>;
    /// Delegate to ContactControls::decode and authenticate each Known source against this
    /// run/config/artifact context. The default refuses; shape alone is not an observation.
    fn validate_controls(&self, _current: &Self::Model, _run: &RunKey, _bytes: &[u8], _require_effective: bool) -> Result<(), ContractError> {
        Err(ContractError::Unsupported("contact controls provider not integrated"))
    }
    /// Delegate to IO PolicyRequest::decode(bytes, frozen_digest, current model/geometry/run).
    /// A valid declaration does not prove native application or runtime qualification.
    fn validate_policy_declaration(&self, _current: &Self::Model, _run: &RunKey, _bytes: &[u8], _request_digest: &str) -> Result<(), ContractError> {
        Err(ContractError::Unsupported("policy declaration provider not integrated"))
    }
}

struct HostState<M> {
    model: M,
    run: RunKey,
    next_lease: u64,
    active: Option<u64>,
}

pub struct ReceiverHost<P: SnapshotProvider> {
    state: Mutex<HostState<P::Model>>,
    provider: P,
}

/// Non-cloneable and non-deserializable. Dropping it releases only this lease.
pub struct ReceiveTransaction<'a, P: SnapshotProvider> {
    host: &'a ReceiverHost<P>,
    lease: u64,
    run: RunKey,
    binding: ModelBinding,
    entities: EntityMap,
    profile: ReceiveProfile,
}

impl<P: SnapshotProvider> ReceiverHost<P> {
    pub fn new(model: P::Model, run: RunKey, provider: P) -> Result<Self, ContractError> {
        run.validate()?;
        Ok(Self { state: Mutex::new(HostState { model, run, next_lease: 0, active: None }), provider })
    }

    fn lock(&self) -> Result<MutexGuard<'_, HostState<P::Model>>, ContractError> {
        self.state.lock().map_err(|_| ContractError::Invalid("receiver model lock poisoned"))
    }

    pub fn begin(&self, expected_run: &RunKey, entities: EntityMap, profile: ReceiveProfile) -> Result<ReceiveTransaction<'_, P>, ContractError> {
        entities.validate()?;
        profile.goal.validate()?;
        profile.source_scene.validate()?;
        if profile.source_scene.kind != profile.evidence_kind
            || profile.required_fields.len() > 6
            || !profile.required_fields.contains(&Quantity::Position)
        {
            return Err(ContractError::Invalid("receive profile"));
        }
        let mut state = self.lock()?;
        if &state.run != expected_run {
            return Err(ContractError::StaleRun);
        }
        if state.active.is_some() {
            return Err(ContractError::InvalidState);
        }
        self.validate_controls(&state, &profile)?;
        self.validate_policy(&state, &profile)?;
        let binding = self.provider.freeze(&state.model, &entities)?;
        binding.validate_shape()?;
        self.provider.validate(&state.model, &binding, &entities)?;
        let lease = state.next_lease.checked_add(1).ok_or(ContractError::Invalid("lease overflow"))?;
        state.next_lease = lease;
        state.active = Some(lease);
        Ok(ReceiveTransaction { host: self, lease, run: expected_run.clone(), binding, entities, profile })
    }

    fn validate_controls(&self, state: &HostState<P::Model>, profile: &ReceiveProfile) -> Result<(), ContractError> {
        match &profile.contact_controls {
            Some(control) => {
                if control.bytes.len() > 65536 {
                    return Err(ContractError::Invalid("contact controls byte cap"));
                }
                if fieldcraft_io::sha256_hex(&control.bytes) != control.sha256 {
                    return Err(ContractError::ChecksumMismatch);
                }
                self.provider.validate_controls(&state.model, &state.run, &control.bytes, profile.require_effective)
            }
            None if profile.require_effective => Err(ContractError::Invalid("MissingContactControls: quantitative profile requires observations")),
            None => Ok(()),
        }
    }

    fn validate_policy(&self, state: &HostState<P::Model>, profile: &ReceiveProfile) -> Result<(), ContractError> {
        if let Some(policy) = &profile.policy_request {
            if policy.bytes.len() > 1024 * 1024 || !digest_valid(&policy.request_digest) {
                return Err(ContractError::Invalid("policy receipt size or canonical digest"));
            }
            if fieldcraft_io::sha256_hex(&policy.bytes) != policy.wire_sha256 {
                return Err(ContractError::ChecksumMismatch);
            }
            self.provider.validate_policy_declaration(&state.model, &state.run, &policy.bytes, &policy.request_digest)?;
        }
        if profile.require_policy_applied {
            return Err(ContractError::NativeContactPolicyNotApplied);
        }
        Ok(())
    }

    /// Edits may invalidate an outstanding run. They cannot race the commit check.
    /// The owner must route all changes through this lock, not share a mutable side channel.
    pub fn update_model(&self, update: impl FnOnce(&mut P::Model)) -> Result<(), ContractError> {
        update(&mut self.lock()?.model);
        Ok(())
    }

    /// Reset/replacement invalidates the old lease; its Drop cannot release a newer lease.
    pub fn replace_run(&self, run: RunKey) -> Result<(), ContractError> {
        run.validate()?;
        let mut state = self.lock()?;
        if run == state.run
            || (run.run_id == state.run.run_id && (run.epoch <= state.run.epoch || run.worker_generation == state.run.worker_generation))
        {
            return Err(ContractError::Invalid("replacement run must advance epoch and generation"));
        }
        state.run = run;
        state.active = None;
        Ok(())
    }
}

struct ProviderVerifier<'a, P: SnapshotProvider> {
    provider: &'a P,
    current: &'a P::Model,
}
impl<P: SnapshotProvider> IdentityVerifier for ProviderVerifier<'_, P> {
    fn verify(&self, expected: &ModelBinding, _: &ModelBinding, entities: &EntityMap) -> Result<(), ContractError> {
        self.provider.validate(self.current, expected, entities)
    }
}

impl<P: SnapshotProvider> ReceiveTransaction<'_, P> {
    pub fn binding(&self) -> &ModelBinding {
        &self.binding
    }

    fn validate_locked(&self, state: &HostState<P::Model>, envelope: &ResultEnvelope) -> Result<Admission, ContractError> {
        if state.run != self.run || envelope.run != self.run {
            return Err(ContractError::StaleRun);
        }
        if state.active != Some(self.lease) {
            return Err(ContractError::InvalidState);
        }
        self.host.validate_controls(state, &self.profile)?;
        self.host.validate_policy(state, &self.profile)?;
        if envelope.policy_request_digest.as_deref() != self.profile.policy_request.as_ref().map(|p| p.request_digest.as_str()) {
            return Err(ContractError::Invalid("frozen policy declaration changed"));
        }
        if envelope.contact_controls_sha256.as_deref() != self.profile.contact_controls.as_ref().map(|p| p.sha256.as_str()) {
            return Err(ContractError::Invalid("frozen contact controls receipt changed"));
        }
        if envelope.entities != self.entities {
            return Err(ContractError::IdentityMismatch("EntityMapMismatch: frozen source order or object map changed".into()));
        }
        if envelope.goal != self.profile.goal
            || envelope.source_scene != self.profile.source_scene
            || envelope.evidence_kind != self.profile.evidence_kind
            || envelope.required_fields != self.profile.required_fields
        {
            return Err(ContractError::Invalid("frozen run goal, scene or field profile changed"));
        }
        let verifier = ProviderVerifier { provider: &self.host.provider, current: &state.model };
        let mut admission = ExternalReceiver::new(verifier).validate(&self.binding, &self.run, envelope)?;
        if let Some(policy) = &mut admission.policy {
            policy.declaration = EvidenceCheck::Verified;
        }
        Ok(admission)
    }

    pub fn validate(&self, envelope: &ResultEnvelope) -> Result<Admission, ContractError> {
        let state = self.host.lock()?;
        self.validate_locked(&state, envelope)
    }

    /// Rechecks the live identity and run under the same lock that a future atomic publisher
    /// must hold. No result number, durability claim or success receipt is created here.
    pub fn commit(self, envelope: &ResultEnvelope) -> Result<(), ContractError> {
        let state = self.host.lock()?;
        let _admission = self.validate_locked(&state, envelope)?;
        Err(ContractError::StorageUnavailable)
    }
}

impl<P: SnapshotProvider> Drop for ReceiveTransaction<'_, P> {
    fn drop(&mut self) {
        if let Ok(mut state) = self.host.state.lock()
            && state.active == Some(self.lease)
        {
            state.active = None;
        }
    }
}
