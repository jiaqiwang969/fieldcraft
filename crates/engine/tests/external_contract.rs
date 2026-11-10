//! Protocol fixtures only. These tests do not run or validate any numerical solver.
use fieldcraft_engine::external::receive::*;
use fieldcraft_engine::external::run::*;
use fieldcraft_engine::external::*;

fn key() -> RunKey {
    RunKey { run_id: "fixture-run".into(), epoch: 1, worker_generation: "worker-a".into() }
}
fn source() -> SourceRef {
    SourceRef {
        sha256: fieldcraft_io::sha256_hex(b"protocol fixture"),
        locator: "fixture/frame".into(),
        observer: "test".into(),
        kind: EvidenceKind::ProtocolFixture,
    }
}
fn known(value: bool) -> Observed<bool> {
    Observed::Known { value, source: source() }
}
fn facts(frame: u64) -> FrameFacts {
    FrameFacts {
        run: key(),
        frame,
        time_s: frame as f64,
        completed: known(true),
        converged: known(true),
        hit_newton_limit: known(false),
        hit_line_search_limit: known(false),
        world_valid: known(true),
        snapshot_complete: known(true),
    }
}
fn ledger(last: u64) -> RunLedger {
    let mut run = RunLedger::new(key(), RunGoal { last_frame: last, end_time_s: last as f64 }, EvidenceKind::ProtocolFixture).unwrap();
    run.initialize(&key()).unwrap();
    run
}

#[test]
fn batches_and_frame_completion_do_not_implicitly_finish_a_run() {
    let mut run = ledger(2);
    run.begin_advance(&key(), 1).unwrap();
    run.accept_frame(facts(1)).unwrap();
    assert_eq!(run.state(), RunState::Ready);
    assert_eq!(run.solve_outcome(), SolveOutcome::Running);
    assert_eq!(run.finish(&key(), source()), Err(ContractError::InvalidState));
    run.begin_advance(&key(), 1).unwrap();
    run.accept_frame(facts(2)).unwrap();
    assert_eq!(run.solve_outcome(), SolveOutcome::Running);
    run.finish(&key(), source()).unwrap();
    assert_eq!(run.solve_outcome(), SolveOutcome::Succeeded);
    assert_eq!(run.evidence_kind(), EvidenceKind::ProtocolFixture);
}

#[test]
fn first_newton_failure_retains_thirteen_frames_and_rejects_suffix() {
    let mut run = ledger(20);
    run.begin_advance(&key(), 20).unwrap();
    for n in 1..14 {
        run.accept_frame(facts(n)).unwrap();
    }
    let mut failed = facts(14);
    failed.converged = known(false);
    failed.hit_newton_limit = known(true);
    assert!(run.accept_frame(failed.clone()).is_err());
    assert_eq!(run.last_qualified(), 13);
    assert_eq!(run.barrier().unwrap().facts, Some(failed));
    assert_eq!(run.accept_frame(facts(15)), Err(ContractError::InvalidState));
    assert_eq!(run.begin_advance(&key(), 1), Err(ContractError::InvalidState));
    assert_eq!(run.barrier().unwrap().frame, 14);
}

#[test]
fn each_limit_and_unknown_is_independent_of_completed_and_converged() {
    let mut newton = facts(1);
    newton.hit_newton_limit = known(true);
    let mut line = facts(1);
    line.hit_line_search_limit = known(true);
    let mut unknown = facts(1);
    unknown.converged = Observed::Unknown { reason: "not exported".into() };
    for (frame, expected) in [(newton, RunState::Failed), (line, RunState::Failed), (unknown, RunState::Unknown)] {
        let mut run = ledger(2);
        run.begin_advance(&key(), 2).unwrap();
        assert!(run.accept_frame(frame).is_err());
        assert_eq!(run.state(), expected);
        assert_eq!(run.last_qualified(), 0);
    }
}

#[test]
fn init_zero_gaps_bad_times_and_stale_epoch_are_not_solved_frames() {
    for frame in [facts(0), facts(2), FrameFacts { time_s: f64::NAN, ..facts(1) }] {
        let mut run = ledger(3);
        run.begin_advance(&key(), 3).unwrap();
        assert!(run.accept_frame(frame).is_err());
        assert_eq!(run.last_qualified(), 0);
        assert_eq!(run.state(), RunState::Unknown);
    }
    let mut run = ledger(3);
    run.begin_advance(&key(), 3).unwrap();
    let mut stale = facts(1);
    stale.run.epoch = 0;
    assert_eq!(run.accept_frame(stale), Err(ContractError::StaleRun));
    run.accept_frame(facts(1)).unwrap();
}

#[test]
fn missing_diagnostics_never_deserialize_to_success() {
    let mut value = serde_json::to_value(facts(1)).unwrap();
    value.as_object_mut().unwrap().remove("converged");
    assert!(serde_json::from_value::<FrameFacts>(value).is_err());
    assert!(RunLedger::new(key(), RunGoal { last_frame: u64::MAX, end_time_s: 1.0 }, EvidenceKind::ProtocolFixture).is_err());
}

/// Deliberately test-only identity policy; production uses UnconfiguredIdentity until wired.
struct FixtureIdentity;
impl IdentityVerifier for FixtureIdentity {
    fn verify(&self, expected: &ModelBinding, submitted: &ModelBinding, _: &EntityMap) -> Result<(), ContractError> {
        if expected != submitted {
            return Err(ContractError::IdentityMismatch("fixture".into()));
        }
        Ok(())
    }
}
fn binding() -> ModelBinding {
    let mesh = fieldcraft_io::sha256_hex(b"fixture mesh");
    ModelBinding {
        snapshot: ReceiverSnapshot {
            scheme: "fieldcraft-receiver-snapshot/1".into(),
            model_id: format!("sha256:{mesh}"),
            model_revision: fieldcraft_io::sha256_hex(b"fixture revision"),
            reference_mesh_digest: mesh,
        },
        entity_map_digest: fieldcraft_io::sha256_hex(b"fixture map"),
        artifacts: ["producer", "adapter", "receiver"].into_iter().map(|s| (s.into(), fieldcraft_io::sha256_hex(s.as_bytes()))).collect(),
        backend_version: "0.0.29".into(),
    }
}
fn block(quantity: Quantity) -> FieldBlock {
    let (location, unit, components) = match quantity {
        Quantity::Position | Quantity::Displacement => (Location::Node, "m", 3),
        Quantity::Velocity => (Location::Node, "m/s", 3),
        Quantity::ObjectForce => (Location::Object, "N", 3),
        Quantity::ObjectMoment => (Location::Object, "N*m", 3),
        Quantity::ContactEnergy => (Location::Global, "J", 1),
        Quantity::Stress => (Location::Node, "Pa", 6),
    };
    let spec = FieldSpec {
        quantity,
        location,
        unit: unit.into(),
        coordinate_frame: "world_z_up".into(),
        components,
        moment_origin_m: (quantity == Quantity::ObjectMoment).then_some([0.0; 3]),
    };
    let data = if quantity == Quantity::Position {
        let bytes = [1.0_f64, 2.0, 3.0].into_iter().flat_map(f64::to_le_bytes).collect::<Vec<_>>();
        FieldData::Present { sha256: fieldcraft_io::sha256_hex(&bytes), bytes, available: vec![true], source: source() }
    } else {
        FieldData::Unavailable { reason: "not exported by fixture".into() }
    };
    FieldBlock { spec, data }
}
fn envelope() -> ResultEnvelope {
    ResultEnvelope {
        schema_version: 1,
        binding: binding(),
        run: key(),
        evidence_kind: EvidenceKind::ProtocolFixture,
        source_scene: source(),
        contact_controls_sha256: None,
        policy_request_digest: None,
        goal: RunGoal { last_frame: 3, end_time_s: 3.0 },
        entities: EntityMap { nodes: vec![1], elements: vec![], objects: vec![] },
        required_fields: vec![Quantity::Position],
        frames: (1..=3)
            .map(|frame| ResultFrame {
                facts: facts(frame),
                fields: [
                    Quantity::Position,
                    Quantity::Displacement,
                    Quantity::Velocity,
                    Quantity::ObjectForce,
                    Quantity::ObjectMoment,
                    Quantity::ContactEnergy,
                ]
                .into_iter()
                .map(block)
                .collect(),
            })
            .collect(),
        terminal: Some(TerminalClaim { outcome: SolveOutcome::Succeeded, source: source() }),
    }
}
fn inspect(e: &ResultEnvelope) -> Admission {
    ExternalReceiver::new(FixtureIdentity).validate(&binding(), &key(), e).unwrap()
}

#[test]
fn checksummed_complete_fixture_is_neither_real_solve_nor_committed_result() {
    let receiver = ExternalReceiver::new(FixtureIdentity);
    let report = inspect(&envelope());
    assert_eq!(report.transfer, TransferState::Verified);
    assert_eq!(report.coverage, Coverage::Complete { last: 3 });
    assert_eq!(report.solve, SolveOutcome::Unknown);
    assert_eq!(report.commit, CommitState::NotAttempted);
    assert_eq!(receiver.commit(), Err(ContractError::StorageUnavailable));
    println!("protocol_receipt={}", serde_json::to_string(&report).unwrap());
}

#[test]
fn unconfigured_or_mismatching_identity_is_named_and_never_accepted() {
    let mut e = envelope();
    assert_eq!(ExternalReceiver::new(UnconfiguredIdentity).validate(&binding(), &key(), &e), Err(ContractError::IdentityUnavailable));
    e.binding.snapshot.model_revision = fieldcraft_io::sha256_hex(b"wrong-revision");
    assert!(matches!(ExternalReceiver::new(FixtureIdentity).validate(&binding(), &key(), &e), Err(ContractError::IdentityMismatch(_))));
    e = envelope();
    e.run.epoch = 0;
    assert_eq!(ExternalReceiver::new(FixtureIdentity).validate(&binding(), &key(), &e), Err(ContractError::StaleRun));
    e = envelope();
    e.entities.nodes.push(e.entities.nodes[0]);
    assert!(ExternalReceiver::new(FixtureIdentity).validate(&binding(), &key(), &e).is_err());
}

#[test]
fn missing_frames_and_fields_keep_only_the_contiguous_prefix() {
    for change in [0, 1, 2, 3] {
        let mut e = envelope();
        match change {
            0 => {
                e.frames.remove(1);
            }
            1 => {
                e.frames[1].fields.remove(0);
            }
            2 => {
                e.frames[1].fields[0].data = FieldData::Unavailable { reason: "missing".into() };
            }
            _ => {
                e.frames[1].facts.hit_line_search_limit = known(true);
            }
        }
        let r = inspect(&e);
        assert_eq!(r.coverage, Coverage::QualifiedPrefix { last: 1, requested_last: 3 });
        assert_eq!(r.first_rejected_frame, Some(2));
        assert_ne!(r.solve, SolveOutcome::Succeeded);
        assert_eq!(r.commit, CommitState::NotAttempted);
    }
}

#[test]
fn truncation_cancel_and_unsealed_data_remain_distinct() {
    let mut e = envelope();
    e.frames.truncate(2);
    assert_eq!(inspect(&e).solve, SolveOutcome::Unknown);
    e.terminal = Some(TerminalClaim { outcome: SolveOutcome::Cancelled, source: source() });
    assert_eq!(inspect(&e).solve, SolveOutcome::Cancelled);
    e.terminal = None;
    assert!(!inspect(&e).sealed);
    assert_eq!(inspect(&e).solve, SolveOutcome::Running);
}

#[test]
fn checksums_shapes_nan_units_and_undefined_stress_are_rejected() {
    for mode in 0..5 {
        let mut e = envelope();
        let block = &mut e.frames[0].fields[0];
        match mode {
            0 => {
                if let FieldData::Present { sha256, .. } = &mut block.data {
                    *sha256 = "0".repeat(64);
                }
            }
            1 => {
                if let FieldData::Present { available, .. } = &mut block.data {
                    available.push(false);
                }
            }
            2 => {
                if let FieldData::Present { bytes, sha256, .. } = &mut block.data {
                    bytes[0..8].copy_from_slice(&f64::NAN.to_le_bytes());
                    *sha256 = fieldcraft_io::sha256_hex(bytes);
                }
            }
            3 => block.spec.unit = "mm".into(),
            _ => block.spec.quantity = Quantity::Stress,
        }
        assert!(ExternalReceiver::new(FixtureIdentity).validate(&binding(), &key(), &e).is_err());
    }
}

#[test]
fn mask_uses_compact_rows_and_required_unavailability_cuts_prefix() {
    let mut e = envelope();
    e.entities.nodes.push(3);
    for frame in &mut e.frames {
        if let FieldData::Present { available, .. } = &mut frame.fields[0].data {
            available.push(false);
        }
    }
    let r = inspect(&e);
    assert_eq!(r.transfer, TransferState::Verified);
    assert_eq!(r.coverage, Coverage::Empty);
    assert_eq!(r.first_rejected_frame, Some(1));
}

#[cfg(unix)]
mod processes {
    use super::*;
    use fieldcraft_engine::external::worker::{ExitOutcome, OwnedWorker};
    use std::process::Command;
    use std::time::Duration;

    #[test]
    fn direct_children_exit_zero_two_139_and_signal_have_distinct_real_receipts() {
        for (script, expected) in [
            ("exit 0", ExitOutcome::Exited { code: 0 }),
            ("exit 2", ExitOutcome::Exited { code: 2 }),
            ("exit 139", ExitOutcome::Exited { code: 139 }),
            ("kill -TERM $$", ExitOutcome::Signaled { number: 15, core_dumped: false }),
        ] {
            let mut worker = OwnedWorker::spawn(Command::new("/bin/sh").args(["-c", script]), key()).unwrap();
            assert_ne!(worker.pid(), std::process::id());
            let exit = worker.wait_for_exit(Duration::from_secs(2)).unwrap();
            assert_eq!(exit.outcome(), &expected);
            assert_eq!(worker.try_exit().unwrap(), Some(exit.clone()));
            println!("process_receipt={}", serde_json::to_string(&exit).unwrap());
            let mut run = ledger(3);
            run.begin_advance(&key(), 3).unwrap();
            run.accept_frame(facts(1)).unwrap();
            run.observe_exit(&exit).unwrap();
            assert_eq!(run.last_qualified(), 1);
            assert_eq!(run.solve_outcome(), if exit.success() { SolveOutcome::Unknown } else { SolveOutcome::Failed });
            assert_eq!(run.begin_advance(&key(), 1), Err(ContractError::InvalidState));
        }
    }

    #[test]
    fn cancel_is_pending_until_reaped_then_reset_changes_epoch_and_generation() {
        let mut worker = OwnedWorker::spawn(Command::new("/bin/sleep").arg("30"), key()).unwrap();
        assert!(matches!(worker.terminate_and_reap(Duration::ZERO), Err(ContractError::Invalid(_))));
        assert_eq!(worker.try_exit().unwrap(), None, "invalid request must not kill the child");
        assert_eq!(worker.wait_for_exit(Duration::from_millis(10)), Err(ContractError::WorkerTimeout));
        let mut run = ledger(3);
        run.begin_advance(&key(), 3).unwrap();
        run.accept_frame(facts(1)).unwrap();
        run.request_cancel(&key()).unwrap();
        assert_eq!(run.state(), RunState::CancelPending);
        assert_eq!(run.solve_outcome(), SolveOutcome::Running);
        let exit = worker.terminate_and_reap(Duration::from_secs(2)).unwrap();
        assert!(matches!(exit.outcome(), ExitOutcome::Signaled { number: 9, .. }));
        assert_eq!(run.reset_after_exit(&exit, "worker-b".into()), Err(ContractError::InvalidState));
        run.observe_exit(&exit).unwrap();
        assert_eq!(run.solve_outcome(), SolveOutcome::Cancelled);
        assert_eq!(run.last_qualified(), 1);
        run.reset_after_exit(&exit, "worker-b".into()).unwrap();
        assert_eq!(run.key().epoch, 2);
        assert_eq!(run.key().worker_generation, "worker-b");
        assert_eq!(run.last_qualified(), 0);
        assert!(run.barrier().is_none());
        assert_eq!(run.initialize(&key()), Err(ContractError::StaleRun));
        let next = run.key().clone();
        run.initialize(&next).unwrap();
        assert_eq!(run.observe_exit(&exit), Err(ContractError::StaleRun));
        println!("cancel_exit={}", serde_json::to_string(&exit).unwrap());
    }

    #[test]
    fn exit_zero_cannot_erase_a_numerical_failure_and_spawn_failure_is_named() {
        let mut worker = OwnedWorker::spawn(Command::new("/bin/sh").args(["-c", "exit 0"]), key()).unwrap();
        let exit = worker.wait_for_exit(Duration::from_secs(2)).unwrap();
        let mut run = ledger(2);
        run.begin_advance(&key(), 2).unwrap();
        let mut frame = facts(1);
        frame.converged = known(false);
        assert!(run.accept_frame(frame).is_err());
        run.observe_exit(&exit).unwrap();
        assert_eq!(run.solve_outcome(), SolveOutcome::Failed);
        assert!(matches!(OwnedWorker::spawn(&mut Command::new("/nonexistent/fieldcraft-worker"), key()), Err(ContractError::Worker(_))));
    }
}

#[test]
fn all_forward_fields_accept_real_binary_layout_but_remain_fixture_evidence() {
    let mut e = envelope();
    e.entities.objects.push(EntityPair { source: "body-a".into(), target: "body-b".into() });
    e.required_fields =
        vec![Quantity::Position, Quantity::Displacement, Quantity::Velocity, Quantity::ObjectForce, Quantity::ObjectMoment, Quantity::ContactEnergy];
    for frame in &mut e.frames {
        for block in &mut frame.fields {
            let bytes = vec![1.25_f64; block.spec.components].into_iter().flat_map(f64::to_le_bytes).collect::<Vec<_>>();
            block.data = FieldData::Present { sha256: fieldcraft_io::sha256_hex(&bytes), bytes, available: vec![true], source: source() };
        }
    }
    assert_eq!(inspect(&e).coverage, Coverage::Complete { last: 3 });
    assert_eq!(inspect(&e).solve, SolveOutcome::Unknown);
    e.entities.objects.clear();
    for frame in &mut e.frames {
        for block in &mut frame.fields {
            if block.spec.location == Location::Object {
                block.data = FieldData::Present { bytes: vec![], sha256: fieldcraft_io::sha256_hex(&[]), available: vec![], source: source() };
            }
        }
    }
    assert_eq!(inspect(&e).coverage, Coverage::Empty, "an empty object map cannot satisfy required forces");
}

mod identity_transactions {
    use super::*;
    use fieldcraft_engine::external::transaction::{ContactControlPayload, PolicyRequestPayload, ReceiveProfile, ReceiverHost, SnapshotProvider};
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};

    fn profile(e: &ResultEnvelope) -> ReceiveProfile {
        ReceiveProfile {
            goal: e.goal,
            evidence_kind: e.evidence_kind,
            source_scene: e.source_scene.clone(),
            required_fields: e.required_fields.clone(),
            contact_controls: None,
            require_effective: false,
            policy_request: None,
            require_policy_applied: false,
        }
    }

    struct FixtureModel {
        binding: ModelBinding,
        map: EntityMap,
    }
    struct FixtureProvider {
        validations: Arc<AtomicUsize>,
    }
    impl SnapshotProvider for FixtureProvider {
        type Model = FixtureModel;
        fn freeze(&self, current: &Self::Model, _: &EntityMap) -> Result<ModelBinding, ContractError> {
            Ok(current.binding.clone())
        }
        fn validate(&self, current: &Self::Model, frozen: &ModelBinding, entities: &EntityMap) -> Result<(), ContractError> {
            self.validations.fetch_add(1, Ordering::SeqCst);
            let reason = if current.binding.snapshot.reference_mesh_digest != frozen.snapshot.reference_mesh_digest {
                Some("ReferenceMeshMismatch")
            } else if current.binding.snapshot.model_revision != frozen.snapshot.model_revision {
                Some("ModelRevisionMismatch")
            } else if current.map != *entities {
                Some("EntityMapMismatch")
            } else {
                None
            };
            reason.map_or(Ok(()), |s| Err(ContractError::IdentityMismatch(s.into())))
        }
    }
    fn host() -> (ReceiverHost<FixtureProvider>, Arc<AtomicUsize>) {
        let validations = Arc::new(AtomicUsize::new(0));
        let e = envelope();
        let host =
            ReceiverHost::new(FixtureModel { binding: e.binding, map: e.entities }, key(), FixtureProvider { validations: validations.clone() })
                .unwrap();
        (host, validations)
    }

    #[test]
    fn matching_begin_and_commit_revalidate_and_release_the_exclusive_run_lease() {
        let (host, calls) = host();
        let e = envelope();
        let tx = host.begin(&key(), e.entities.clone(), profile(&e)).unwrap();
        assert_eq!(tx.binding(), &e.binding);
        assert!(matches!(host.begin(&key(), e.entities.clone(), profile(&e)), Err(ContractError::InvalidState)));
        assert_eq!(tx.validate(&e).unwrap().coverage, Coverage::Complete { last: 3 });
        assert_eq!(tx.commit(&e), Err(ContractError::StorageUnavailable));
        assert_eq!(calls.load(Ordering::SeqCst), 3, "begin, preview and commit each invoke the current provider");
        drop(host.begin(&key(), e.entities.clone(), profile(&e)).unwrap());
    }

    #[test]
    fn model_or_mesh_edits_during_a_run_are_rejected_at_commit() {
        for mesh in [false, true] {
            let (host, _) = host();
            let e = envelope();
            let tx = host.begin(&key(), e.entities.clone(), profile(&e)).unwrap();
            host.update_model(|m| {
                if mesh {
                    m.binding.snapshot.reference_mesh_digest = fieldcraft_io::sha256_hex(b"edited mesh");
                } else {
                    m.binding.snapshot.model_revision = fieldcraft_io::sha256_hex(b"edited definition");
                }
            })
            .unwrap();
            let expected = if mesh { "ReferenceMeshMismatch" } else { "ModelRevisionMismatch" };
            assert_eq!(tx.commit(&e), Err(ContractError::IdentityMismatch(expected.into())));
        }
    }

    #[test]
    fn wrong_mapping_at_begin_or_commit_is_not_silently_rebound() {
        let (host, _) = host();
        let mut e = envelope();
        let mut wrong = e.entities.clone();
        wrong.nodes[0] = 2;
        assert!(matches!(host.begin(&key(), wrong.clone(), profile(&e)), Err(ContractError::IdentityMismatch(_))));
        let tx = host.begin(&key(), e.entities.clone(), profile(&e)).unwrap();
        e.entities = wrong;
        assert!(matches!(tx.commit(&e), Err(ContractError::IdentityMismatch(_))));
    }

    #[test]
    fn replacing_epoch_rejects_old_commit_and_old_drop_preserves_the_new_lease() {
        let (host, _) = host();
        let e = envelope();
        let old = host.begin(&key(), e.entities.clone(), profile(&e)).unwrap();
        let next = RunKey { epoch: 2, worker_generation: "new-worker".into(), ..key() };
        host.replace_run(next.clone()).unwrap();
        let new = host.begin(&next, e.entities.clone(), profile(&e)).unwrap();
        assert_eq!(old.commit(&e), Err(ContractError::StaleRun));
        assert!(matches!(host.begin(&next, e.entities.clone(), profile(&e)), Err(ContractError::InvalidState)));
        drop(new);
        assert!(host.begin(&next, e.entities.clone(), profile(&e)).is_ok());
    }

    #[test]
    fn a_producer_cannot_shorten_the_goal_or_change_the_scene_after_begin() {
        for scene in [false, true] {
            let (host, _) = host();
            let mut e = envelope();
            let tx = host.begin(&key(), e.entities.clone(), profile(&e)).unwrap();
            if scene {
                e.source_scene.sha256 = fieldcraft_io::sha256_hex(b"another scene");
            } else {
                e.goal = RunGoal { last_frame: 1, end_time_s: 1.0 };
                e.frames.truncate(1);
            }
            assert!(matches!(tx.commit(&e), Err(ContractError::Invalid(_))));
        }
    }

    #[test]
    fn binding_schema_rejects_unknown_scheme_bad_roles_and_old_backend() {
        for which in 0..3 {
            let mut b = binding();
            match which {
                0 => b.snapshot.scheme = "future-unknown/2".into(),
                1 => {
                    b.artifacts.remove("receiver");
                }
                _ => b.backend_version = "0.0.28".into(),
            }
            assert!(b.validate_shape().is_err());
        }
    }

    #[test]
    fn quantitative_controls_cannot_be_missing_or_use_an_unconfigured_provider() {
        let (host, _) = host();
        let e = envelope();
        let mut p = profile(&e);
        p.require_effective = true;
        assert!(matches!(host.begin(&key(), e.entities.clone(), p.clone()), Err(ContractError::Invalid(_))));
        let bytes = b"fixture controls, not a physical observation".to_vec();
        p.contact_controls = Some(ContactControlPayload { sha256: fieldcraft_io::sha256_hex(&bytes), bytes });
        assert!(matches!(host.begin(&key(), e.entities.clone(), p.clone()), Err(ContractError::Unsupported(_))));
        p.contact_controls.as_mut().unwrap().sha256 = fieldcraft_io::sha256_hex(b"wrong receipt");
        assert!(matches!(host.begin(&key(), e.entities.clone(), p), Err(ContractError::ChecksumMismatch)));
    }

    /// Exercises the host/provider seam only. It deliberately does not implement or replace
    /// ContactControls::decode, source authentication or the authoritative IO identity codec.
    struct ControlFixtureProvider {
        identity: FixtureProvider,
        seen: Arc<std::sync::Mutex<Vec<bool>>>,
    }
    impl SnapshotProvider for ControlFixtureProvider {
        type Model = (FixtureModel, bool);
        fn freeze(&self, current: &Self::Model, entities: &EntityMap) -> Result<ModelBinding, ContractError> {
            self.identity.freeze(&current.0, entities)
        }
        fn validate(&self, current: &Self::Model, frozen: &ModelBinding, entities: &EntityMap) -> Result<(), ContractError> {
            self.identity.validate(&current.0, frozen, entities)
        }
        fn validate_controls(&self, current: &Self::Model, run: &RunKey, bytes: &[u8], require_effective: bool) -> Result<(), ContractError> {
            self.seen.lock().unwrap().push(require_effective);
            if run != &key() || bytes != b"fixture controls" {
                return Err(ContractError::Invalid("fixture source/run mismatch"));
            }
            if require_effective && !current.1 {
                return Err(ContractError::Invalid("MissingEffectiveKappa: fixture unknown"));
            }
            Ok(())
        }
        fn validate_policy_declaration(&self, current: &Self::Model, run: &RunKey, bytes: &[u8], request_digest: &str) -> Result<(), ContractError> {
            if !current.1
                || run != &key()
                || bytes != b"fixture policy wire"
                || request_digest != fieldcraft_io::sha256_hex(b"canonical policy fixture")
            {
                return Err(ContractError::Invalid("fixture policy context or digest changed"));
            }
            Ok(())
        }
    }

    #[test]
    fn controls_are_frozen_and_revalidated_without_downgrading_quantitative_policy() {
        let mut e = envelope();
        let seen = Arc::new(std::sync::Mutex::new(Vec::new()));
        let provider = ControlFixtureProvider { identity: FixtureProvider { validations: Arc::new(AtomicUsize::new(0)) }, seen: seen.clone() };
        let host = ReceiverHost::new((FixtureModel { binding: e.binding.clone(), map: e.entities.clone() }, true), key(), provider).unwrap();
        let bytes = b"fixture controls".to_vec();
        let digest = fieldcraft_io::sha256_hex(&bytes);
        e.contact_controls_sha256 = Some(digest.clone());
        let mut p = profile(&e);
        p.require_effective = true;
        p.contact_controls = Some(ContactControlPayload { bytes, sha256: digest.clone() });
        let tx = host.begin(&key(), e.entities.clone(), p.clone()).unwrap();
        assert_eq!(tx.validate(&e).unwrap().solve, SolveOutcome::Unknown);
        e.contact_controls_sha256 = Some(fieldcraft_io::sha256_hex(b"substituted controls"));
        assert!(matches!(tx.validate(&e), Err(ContractError::Invalid(_))));
        e.contact_controls_sha256 = Some(digest);
        host.update_model(|m| m.1 = false).unwrap();
        assert_eq!(tx.commit(&e), Err(ContractError::Invalid("MissingEffectiveKappa: fixture unknown")));
        assert!(seen.lock().unwrap().iter().all(|v| *v), "a quantitative refusal must never retry with false");
        assert!(host.begin(&key(), e.entities.clone(), p.clone()).is_err());
        // A separately requested diagnostic transaction may retain Unknown, still not commit.
        p.require_effective = false;
        let diagnostic = host.begin(&key(), e.entities.clone(), p).unwrap();
        assert_eq!(diagnostic.commit(&e), Err(ContractError::StorageUnavailable));
        println!(
            "controls_gate_receipt={{\"evidence_kind\":\"protocol_fixture\",\"quantitative_unknown\":\"rejected\",\"diagnostic_unknown\":\"retained\",\"committed\":false}}"
        );
    }

    fn policy_payload() -> PolicyRequestPayload {
        let bytes = b"fixture policy wire".to_vec();
        PolicyRequestPayload {
            wire_sha256: fieldcraft_io::sha256_hex(&bytes),
            bytes,
            request_digest: fieldcraft_io::sha256_hex(b"canonical policy fixture"),
        }
    }

    #[test]
    fn policy_declaration_and_native_application_are_distinct_and_frozen() {
        let mut e = envelope();
        // Forged solver-labelled facts test that a policy declaration cannot upgrade admission.
        // These are still fabricated test data, not a numerical run.
        e.evidence_kind = EvidenceKind::Solver;
        e.source_scene.kind = EvidenceKind::Solver;
        e.terminal.as_mut().unwrap().source.kind = EvidenceKind::Solver;
        for frame in &mut e.frames {
            let f = &mut frame.facts;
            for fact in [
                &mut f.completed,
                &mut f.converged,
                &mut f.hit_newton_limit,
                &mut f.hit_line_search_limit,
                &mut f.world_valid,
                &mut f.snapshot_complete,
            ] {
                if let Observed::Known { source, .. } = fact {
                    source.kind = EvidenceKind::Solver;
                }
            }
            for field in &mut frame.fields {
                if let FieldData::Present { source, .. } = &mut field.data {
                    source.kind = EvidenceKind::Solver;
                }
            }
        }
        let provider = ControlFixtureProvider {
            identity: FixtureProvider { validations: Arc::new(AtomicUsize::new(0)) },
            seen: Arc::new(std::sync::Mutex::new(Vec::new())),
        };
        let host = ReceiverHost::new((FixtureModel { binding: e.binding.clone(), map: e.entities.clone() }, true), key(), provider).unwrap();
        let payload = policy_payload();
        e.policy_request_digest = Some(payload.request_digest.clone());
        let mut p = profile(&e);
        p.policy_request = Some(payload);
        p.require_policy_applied = true;
        assert!(matches!(host.begin(&key(), e.entities.clone(), p.clone()), Err(ContractError::NativeContactPolicyNotApplied)));
        p.require_policy_applied = false;
        let tx = host.begin(&key(), e.entities.clone(), p).unwrap();
        let report = tx.validate(&e).unwrap();
        assert_eq!(report.solve, SolveOutcome::Unknown);
        let policy = report.policy.unwrap();
        assert_eq!(policy.declaration, EvidenceCheck::Verified);
        assert_eq!(policy.application, EvidenceCheck::Unverified);
        assert_eq!(policy.runtime_qualification, EvidenceCheck::Unverified);
        let original = e.policy_request_digest.clone();
        e.policy_request_digest = Some(fieldcraft_io::sha256_hex(b"another declaration"));
        assert!(matches!(tx.validate(&e), Err(ContractError::Invalid(_))));
        e.policy_request_digest = original;
        host.update_model(|m| m.1 = false).unwrap();
        assert!(matches!(tx.commit(&e), Err(ContractError::Invalid(_))));
        println!(
            "policy_gate_receipt={{\"test_origin\":\"protocol_fixture\",\"declaration\":\"verified\",\"application\":\"unverified\",\"runtime_qualification\":\"unverified\",\"require_applied\":\"NativeContactPolicyNotApplied\"}}"
        );
    }

    #[test]
    fn policy_wire_hash_and_provider_are_required_and_wire_cannot_claim_qualification() {
        let (host, _) = host();
        let e = envelope();
        let mut p = profile(&e);
        p.policy_request = Some(policy_payload());
        assert!(matches!(host.begin(&key(), e.entities.clone(), p.clone()), Err(ContractError::Unsupported(_))));
        p.policy_request.as_mut().unwrap().wire_sha256 = fieldcraft_io::sha256_hex(b"changed wire");
        assert!(matches!(host.begin(&key(), e.entities.clone(), p), Err(ContractError::ChecksumMismatch)));
        let mut wire = serde_json::to_value(e).unwrap();
        wire.as_object_mut().unwrap().insert("runtime_qualification".into(), serde_json::json!("closed_stick"));
        assert!(serde_json::from_value::<ResultEnvelope>(wire).is_err());
    }
}
