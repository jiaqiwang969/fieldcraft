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
            scheme: fieldcraft_io::identity::SCHEME.into(),
            model_id: fieldcraft_io::identity::model_id(&mesh),
            model_revision: fieldcraft_io::sha256_hex(b"fixture revision"),
            reference_mesh_digest: mesh,
        },
        entity_map_digest: fieldcraft_io::sha256_hex(b"fixture map"),
        artifacts: fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES
            .into_iter()
            .map(|s| (s.into(), fieldcraft_io::sha256_hex(s.as_bytes())))
            .collect(),
        backend_version: fieldcraft_io::identity::BACKEND_VERSION.into(),
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
        contact_control_record_sha256: None,
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
    use fieldcraft_engine::external::control::{ControlAvailability, ControlEvidenceReceipt, ControlEvidenceScope, NativeIpcStatus};
    use fieldcraft_engine::external::production::{FileResultStore, IoModelContext, IoSnapshotProvider, ResultStore};
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
            control_evidence: None,
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

    const ROUTE20_BINDINGS_SHA: &str = "ec9fb5ee22c95b3462d5989812738dda0f20247cdb372b842579d1ce0bfa8257";
    const ROUTE20_CASE20_CONTROL_SHA: &str = "cca161d81e4b86b91d36240296702b5213165ecaba2a8efab6b94ff4492b20c1";
    const ROUTE20_CASE21_CONTROL_SHA: &str = "d081e0912a253aa9f3390520c66a1193fb70147d5326a70867d0498ea5509a3d";
    const ROUTE20_CASE20_SOURCE_SHA: &str = "a167490fc17d788c3b26792f967c5e4efb66fe1d1032ff0b030ce0eac21bec23";

    fn route20_key(run_id: &str, epoch: u64) -> RunKey {
        RunKey { run_id: run_id.into(), epoch, worker_generation: "route20-worker".into() }
    }

    fn route20_evidence(run: &RunKey, kind: ControlEvidenceScope) -> ControlEvidenceReceipt {
        let known = matches!(kind, ControlEvidenceScope::Initialization) && run.run_id == "fresh-00";
        let (effective, clamped) = if known {
            (
                Observed::Known {
                    value: 152606501887.99008,
                    source: SourceRef {
                        sha256: ROUTE20_CASE20_SOURCE_SHA.into(),
                        locator: "line:5; contact model 1,2; World initialization".into(),
                        observer: "route20-control-receipt".into(),
                        kind: EvidenceKind::Solver,
                    },
                },
                Observed::Known {
                    value: true,
                    source: SourceRef {
                        sha256: ROUTE20_CASE20_SOURCE_SHA.into(),
                        locator: "line:5; contact model 1,2; World initialization".into(),
                        observer: "route20-control-receipt".into(),
                        kind: EvidenceKind::Solver,
                    },
                },
            )
        } else {
            (
                Observed::Unknown {
                    reason: "No direct effective-kappa observation in this run; configured resistance is not an observation.".into(),
                },
                Observed::Unknown { reason: "No positive clipping receipt; absence of a warning does not prove false.".into() },
            )
        };
        let (record, control, locator, pids, native_ipc, epoch) = match kind {
            ControlEvidenceScope::Initialization if known => (
                "bindings.json#/records/0",
                Some(ROUTE20_CASE20_CONTROL_SHA),
                "case20/接触控制收据/case20-fresh-00.json",
                vec![2051436],
                NativeIpcStatus::Unknown,
                Some(run.epoch),
            ),
            ControlEvidenceScope::Initialization => (
                "bindings.json#/records/10",
                Some(ROUTE20_CASE21_CONTROL_SHA),
                "case21/接触控制收据/case21-control-00-resident0.json",
                vec![2053690, 2054729, 2055031],
                NativeIpcStatus::Unknown,
                Some(run.epoch),
            ),
            ControlEvidenceScope::NativeIpcDisabled => {
                ("bindings.json#/records/80", None, "bindings.json#/records/80", vec![], NativeIpcStatus::Disabled, None)
            }
            ControlEvidenceScope::AdvanceFrames => unreachable!("route-20 fixture helper uses initialization or IPC-disabled only"),
        };
        ControlEvidenceReceipt {
            run_label: run.run_id.clone(),
            epoch,
            observed_compute_pids: pids,
            scope: kind,
            native_ipc,
            observed_frame: None,
            observed_time_s: None,
            record_sha256: ROUTE20_BINDINGS_SHA.into(),
            record_locator: record.into(),
            control_sha256: control.map(str::to_owned),
            control_locator: locator.into(),
            kappa_effective_pa: effective,
            kappa_clamped: clamped,
        }
    }

    fn route20_envelope(run: &RunKey) -> ResultEnvelope {
        let mut e = envelope();
        e.run = run.clone();
        for frame in &mut e.frames {
            frame.facts.run = run.clone();
        }
        e
    }

    struct Route20Provider {
        identity: FixtureProvider,
        expected_controls: &'static [u8],
    }
    impl SnapshotProvider for Route20Provider {
        type Model = FixtureModel;
        fn freeze(&self, current: &Self::Model, entities: &EntityMap) -> Result<ModelBinding, ContractError> {
            self.identity.freeze(current, entities)
        }
        fn validate(&self, current: &Self::Model, frozen: &ModelBinding, entities: &EntityMap) -> Result<(), ContractError> {
            self.identity.validate(current, frozen, entities)
        }
        fn validate_controls(&self, _: &Self::Model, run: &RunKey, bytes: &[u8], require_effective: bool) -> Result<(), ContractError> {
            if run.run_id == "fresh-00" {
                if bytes != self.expected_controls || require_effective {
                    return Err(ContractError::Invalid("route20 initialization is not per-frame evidence"));
                }
            } else if bytes != self.expected_controls {
                return Err(ContractError::Invalid("route20 control source mismatch"));
            }
            let value: serde_json::Value = serde_json::from_slice(bytes).map_err(|_| ContractError::Invalid("route20 control JSON"))?;
            if value.get("scheme").and_then(serde_json::Value::as_str) != Some("fieldcraft-contact-controls/1") {
                return Err(ContractError::Invalid("route20 control scheme"));
            }
            Ok(())
        }
    }

    fn route20_host(run: RunKey, controls: &'static [u8]) -> ReceiverHost<Route20Provider> {
        let e = route20_envelope(&run);
        ReceiverHost::new(
            FixtureModel { binding: e.binding, map: e.entities },
            run,
            Route20Provider { identity: FixtureProvider { validations: Arc::new(AtomicUsize::new(0)) }, expected_controls: controls },
        )
        .unwrap()
    }

    fn route20_profile(
        e: &ResultEnvelope,
        evidence: ControlEvidenceReceipt,
        controls: Option<&'static [u8]>,
        require_effective: bool,
    ) -> ReceiveProfile {
        let payload = controls.map(|bytes| ContactControlPayload { bytes: bytes.to_vec(), sha256: fieldcraft_io::sha256_hex(bytes) });
        ReceiveProfile {
            goal: e.goal,
            evidence_kind: e.evidence_kind,
            source_scene: e.source_scene.clone(),
            required_fields: e.required_fields.clone(),
            contact_controls: payload,
            control_evidence: Some(evidence),
            require_effective,
            policy_request: None,
            require_policy_applied: false,
        }
    }

    fn production_entities() -> EntityMap {
        EntityMap {
            nodes: vec![1, 2, 3, 4],
            elements: vec![MappedElement { target: 8, kind: fieldcraft_model::ElementKind::Tet4, source_nodes: vec![0, 1, 2, 3] }],
            objects: vec![],
        }
    }

    fn entities_for_model(model: &fieldcraft_model::Model) -> EntityMap {
        let nodes: Vec<u32> = model.nodes().keys().copied().collect();
        let elements = model
            .elements()
            .iter()
            .map(|(target, element)| MappedElement {
                target: *target,
                kind: element.kind,
                source_nodes: element.nodes.iter().map(|label| nodes.iter().position(|candidate| candidate == label).unwrap()).collect(),
            })
            .collect();
        EntityMap { nodes, elements, objects: vec![] }
    }

    fn production_context() -> (IoModelContext, EntityMap) {
        let mut model = fieldcraft_model::Model::default();
        for (index, position) in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]].into_iter().enumerate() {
            model.set_node(index as u32 + 1, position).unwrap();
        }
        model.set_element(8, fieldcraft_model::ElementKind::Tet4, vec![1, 2, 3, 4]).unwrap();
        let entities = production_entities();
        let artifacts = fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES
            .into_iter()
            .map(|role| (role.into(), fieldcraft_io::identity::artifact_digest(role.as_bytes())))
            .collect();
        (IoModelContext::new(model, fieldcraft_io::Geometry::default(), fieldcraft_io::Run::default(), artifacts), entities)
    }

    fn production_envelope(run: &RunKey, binding: ModelBinding, entities: EntityMap) -> ResultEnvelope {
        let mut e = envelope();
        e.binding = binding;
        let node_count = entities.nodes.len();
        e.entities = entities;
        e.run = run.clone();
        e.evidence_kind = EvidenceKind::Solver;
        e.source_scene.kind = EvidenceKind::Solver;
        if let Some(terminal) = &mut e.terminal {
            terminal.source.kind = EvidenceKind::Solver;
        }
        for frame in &mut e.frames {
            frame.facts.run = run.clone();
            for fact in [
                &mut frame.facts.completed,
                &mut frame.facts.converged,
                &mut frame.facts.hit_newton_limit,
                &mut frame.facts.hit_line_search_limit,
                &mut frame.facts.world_valid,
                &mut frame.facts.snapshot_complete,
            ] {
                if let Observed::Known { source, .. } = fact {
                    source.kind = EvidenceKind::Solver;
                }
            }
            for field in &mut frame.fields {
                if let FieldData::Present { source, .. } = &mut field.data {
                    source.kind = EvidenceKind::Solver;
                }
                if field.spec.location == Location::Node {
                    let values = vec![1.0_f64; field.spec.components * node_count];
                    let bytes = values.into_iter().flat_map(f64::to_le_bytes).collect::<Vec<_>>();
                    field.data = FieldData::Present {
                        sha256: fieldcraft_io::sha256_hex(&bytes),
                        bytes,
                        available: vec![true; node_count],
                        source: SourceRef { kind: EvidenceKind::Solver, ..source() },
                    };
                }
            }
        }
        e
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
    fn production_binding_shape_rejections_do_not_publish_or_hold_the_run_lock() {
        let run = key();
        let (mut context, entities) = production_context();
        context.artifacts.insert("postprocessor".into(), fieldcraft_io::identity::artifact_digest(b"protocol fixture postprocessor"));
        let binding = IoSnapshotProvider.freeze(&context, &entities).unwrap();
        let e = production_envelope(&run, binding, entities.clone());
        let root = std::env::temp_dir().join(format!("fieldcraft-binding-shape-{}", std::process::id()));
        let store = Arc::new(FileResultStore::new(&root).unwrap());
        let host = ReceiverHost::with_store(context, run.clone(), IoSnapshotProvider, store.clone()).unwrap();
        // These are protocol fixtures using the production identity/transaction/store path.
        // Solver labels exercise the store gate, not libuipc or numerical qualification.
        for which in ["scheme", "backend", "model_id", "producer", "adapter", "receiver", "extra_digest"] {
            let mut changed = e.clone();
            match which {
                "scheme" => changed.binding.snapshot.scheme = "future-unknown/2".into(),
                "backend" => changed.binding.backend_version = "0.0.28".into(),
                "model_id" => changed.binding.snapshot.model_id = "unrecognized-model".into(),
                "extra_digest" => {
                    changed.binding.artifacts.insert("postprocessor".into(), "not-a-digest".into());
                }
                role => {
                    changed.binding.artifacts.remove(role);
                }
            }
            let expected = match which {
                "scheme" => ContractError::Unsupported("receiver identity scheme"),
                "backend" => ContractError::Unsupported("backend version"),
                "model_id" => ContractError::Invalid("snapshot identity fields"),
                _ => ContractError::Invalid("artifact roles and digests"),
            };
            let rejected = host.begin(&run, entities.clone(), profile(&e)).unwrap().commit(&changed);
            assert_eq!(rejected, Err(expected));
            assert_eq!(std::fs::read_dir(&root).unwrap().count(), 0);
            // Every rejected commit releases the lease, without publishing a result.
            drop(host.begin(&run, entities.clone(), profile(&e)).unwrap());
            println!(
                "production_binding_shape_rejection={}",
                serde_json::json!({"field": which, "error": format!("{rejected:?}"), "published": false, "lease_released": true})
            );
        }
        let id = host.begin(&run, entities, profile(&e)).unwrap().commit(&e).unwrap();
        assert_eq!(store.reopen(&id).unwrap(), e);
        let raw = store.read_published(&id).unwrap();
        assert_eq!(raw.envelope_json, serde_json::to_vec(&e).unwrap());
        assert_eq!(fieldcraft_io::sha256_hex(&raw.envelope_json), raw.payload_sha256);
        assert_eq!(raw.result_id, id);
        let read_admission: Admission = serde_json::from_slice(&raw.admission_json).unwrap();
        assert_eq!(read_admission.solve, SolveOutcome::Succeeded);
        println!(
            "production_binding_shape_positive={}",
            serde_json::json!({
                "scheme": e.binding.snapshot.scheme, "result_id": id, "reopened_equal": true,
                "artifact_roles": e.binding.artifacts.keys().collect::<Vec<_>>(),
                "evidence_class": "protocol_fixture", "runtime_qualification": "unverified", "libuipc_called": false,
            })
        );
        std::fs::remove_dir_all(root).unwrap();
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
        assert!(report.blockers.iter().any(|b| b.starts_with("RuntimeQualificationUnverified:")));
        assert!(!report.blockers.iter().any(|b| b.starts_with("NativeContactPolicyNotApplied:")));
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

    #[test]
    fn route20_initialization_known_is_consumed_at_its_observation_point_only() {
        const CONTROLS: &[u8] = include_bytes!("fixtures/route20-case20-fresh-00-controls.json");
        let run = route20_key("fresh-00", 0);
        let mut e = route20_envelope(&run);
        e.contact_controls_sha256 = Some(ROUTE20_CASE20_CONTROL_SHA.into());
        e.contact_control_record_sha256 = Some(ROUTE20_BINDINGS_SHA.into());
        let evidence = route20_evidence(&run, ControlEvidenceScope::Initialization);
        let host = route20_host(run.clone(), CONTROLS);
        let p = route20_profile(&e, evidence, Some(CONTROLS), false);
        let tx = host.begin(&run, e.entities.clone(), p).unwrap();
        let admission = tx.validate(&e).unwrap();
        let control = admission.control.unwrap();
        assert_eq!(control.availability, ControlAvailability::InitializationKnown);
        assert_eq!(control.effective_check, EvidenceCheck::Verified);
        assert_eq!(control.clamped_check, EvidenceCheck::Verified);
        assert_eq!(control.observed_frame, None);
        assert_eq!(control.observed_time_s, None);
        match control.kappa_effective_pa {
            Observed::Known { value, .. } => assert!((value - 152606501887.99008).abs() < f64::EPSILON),
            Observed::Unknown { .. } => panic!("known route20 initialization was lost"),
        }
        assert_eq!(tx.commit(&e), Err(ContractError::StorageUnavailable));
    }

    #[test]
    fn route20_initialization_known_is_rejected_when_per_frame_effective_is_required() {
        const CONTROLS: &[u8] = include_bytes!("fixtures/route20-case20-fresh-00-controls.json");
        let run = route20_key("fresh-00", 0);
        let e = route20_envelope(&run);
        let host = route20_host(run.clone(), CONTROLS);
        let p = route20_profile(&e, route20_evidence(&run, ControlEvidenceScope::Initialization), Some(CONTROLS), true);
        assert!(matches!(
            host.begin(&run, e.entities.clone(), p),
            Err(ContractError::Invalid("MissingPerFrameEffectiveKappa: initialization-only observation"))
        ));
    }

    #[test]
    fn route20_unknown_initialization_is_retained_diagnostically_and_rejected_quantitatively() {
        const CONTROLS: &[u8] = include_bytes!("fixtures/route20-case21-control-00-unknown.json");
        let run = route20_key("control-00-resident0", 1);
        let mut e = route20_envelope(&run);
        e.contact_controls_sha256 = Some(ROUTE20_CASE21_CONTROL_SHA.into());
        e.contact_control_record_sha256 = Some(ROUTE20_BINDINGS_SHA.into());
        let host = route20_host(run.clone(), CONTROLS);
        let p = route20_profile(&e, route20_evidence(&run, ControlEvidenceScope::Initialization), Some(CONTROLS), false);
        let tx = host.begin(&run, e.entities.clone(), p).unwrap();
        assert_eq!(tx.validate(&e).unwrap().control.unwrap().availability, ControlAvailability::InitializationUnknown);

        let host = route20_host(run.clone(), CONTROLS);
        let p = route20_profile(&e, route20_evidence(&run, ControlEvidenceScope::Initialization), Some(CONTROLS), true);
        assert!(matches!(
            host.begin(&run, e.entities.clone(), p),
            Err(ContractError::Invalid("MissingPerFrameEffectiveKappa: initialization-only observation"))
        ));
    }

    #[test]
    fn route20_ipc_disabled_is_separate_from_unknown_and_has_no_control_payload() {
        let run = route20_key("repeat-00", 0);
        let mut e = route20_envelope(&run);
        e.contact_control_record_sha256 = Some(ROUTE20_BINDINGS_SHA.into());
        let host = route20_host(run.clone(), b"");
        let p = route20_profile(&e, route20_evidence(&run, ControlEvidenceScope::NativeIpcDisabled), None, false);
        let tx = host.begin(&run, e.entities.clone(), p).unwrap();
        let control = tx.validate(&e).unwrap().control.unwrap();
        assert_eq!(control.availability, ControlAvailability::NativeIpcDisabled);
        assert_eq!(control.native_ipc, NativeIpcStatus::Disabled);
        assert!(control.control_sha256.is_none());
        assert!(matches!(control.kappa_effective_pa, Observed::Unknown { .. }));

        let host = route20_host(run.clone(), b"");
        let p = route20_profile(&e, route20_evidence(&run, ControlEvidenceScope::NativeIpcDisabled), None, true);
        assert!(matches!(
            host.begin(&run, e.entities.clone(), p),
            Err(ContractError::Unsupported("native IPC disabled; effective kappa unavailable"))
        ));
    }

    #[test]
    fn route20_record_and_control_digests_are_frozen_at_the_receiver_boundary() {
        const CONTROLS: &[u8] = include_bytes!("fixtures/route20-case20-fresh-00-controls.json");
        let run = route20_key("fresh-00", 0);
        let mut e = route20_envelope(&run);
        e.contact_controls_sha256 = Some(ROUTE20_CASE20_CONTROL_SHA.into());
        e.contact_control_record_sha256 = Some(ROUTE20_BINDINGS_SHA.into());
        let host = route20_host(run.clone(), CONTROLS);
        let p = route20_profile(&e, route20_evidence(&run, ControlEvidenceScope::Initialization), Some(CONTROLS), false);
        let tx = host.begin(&run, e.entities.clone(), p).unwrap();
        e.contact_control_record_sha256 = Some("0".repeat(64));
        assert_eq!(tx.validate(&e), Err(ContractError::Invalid("frozen contact control record changed")));
    }

    #[test]
    fn production_store_reopen_rejects_malformed_identifiers_before_reading() {
        let root = std::env::temp_dir().join(format!("fieldcraft-result-path-{}", std::process::id()));
        let store = FileResultStore::new(&root).unwrap();
        for digest in [
            String::new(),
            "0".repeat(63),
            "0".repeat(65),
            "A".repeat(64),
            "g".repeat(64),
            format!("{}\n", "0".repeat(63)),
            format!("{}\0", "0".repeat(63)),
            format!("../{}", "0".repeat(61)),
            format!("sha256:{}", "0".repeat(64)),
            "é".repeat(32),
        ] {
            assert_eq!(store.reopen(&format!("fieldcraft-result/{digest}")), Err(ContractError::Invalid("result identifier digest")));
        }
        assert_eq!(store.reopen(&"0".repeat(64)), Err(ContractError::Invalid("result identifier scheme")));
        // Valid shapes reach storage; missing bytes are not reported as malformed identities.
        for digest in ["0".repeat(64), "0123456789abcdef".repeat(4)] {
            assert!(matches!(store.reopen(&format!("fieldcraft-result/{digest}")), Err(ContractError::Provider(_))));
        }
        assert_eq!(std::fs::read_dir(&root).unwrap().count(), 0);
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn production_store_reopen_rejects_payload_rehash_under_original_id() {
        let run = key();
        let (context, entities) = production_context();
        let binding = IoSnapshotProvider.freeze(&context, &entities).unwrap();
        let e = production_envelope(&run, binding, entities.clone());
        let root = std::env::temp_dir().join(format!("fieldcraft-result-rehash-{}", std::process::id()));
        let store = Arc::new(FileResultStore::new(&root).unwrap());
        let host = ReceiverHost::with_store(context, run.clone(), IoSnapshotProvider, store.clone()).unwrap();
        let id = host.begin(&run, entities, profile(&e)).unwrap().commit(&e).unwrap();
        let path = root.join(format!("{}.json", id.strip_prefix("fieldcraft-result/").unwrap()));
        let mut stored: serde_json::Value = serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        let mut changed = e;
        changed.source_scene.locator = "tampered scene locator".into();
        stored["envelope"] = serde_json::to_value(&changed).unwrap();
        stored["payload_sha256"] = serde_json::json!(fieldcraft_io::sha256_hex(&serde_json::to_vec(&changed).unwrap()));
        std::fs::write(path, serde_json::to_vec(&stored).unwrap()).unwrap();
        let observed = store.reopen(&id);
        std::fs::remove_dir_all(root).unwrap();
        assert_eq!(observed, Err(ContractError::ChecksumMismatch));
    }

    #[test]
    fn production_identity_lock_commits_and_reopens_a_real_file_store() {
        let run = RunKey { run_id: "production-run".into(), epoch: 1, worker_generation: "production-worker".into() };
        let (mut context, entities) = production_context();
        // Explicit opt-in export for independent consumption. Never overwrite an older package.
        // Solver labels below exercise the store gate; every value is a protocol fixture.
        let export = std::env::var_os("FIELDCRAFT_FIXTURE_EXPORT").map(std::path::PathBuf::from);
        if let Some(dir) = &export {
            std::fs::create_dir(dir).unwrap();
            std::fs::create_dir(dir.join("programs")).unwrap();
            let program = std::fs::read(std::env::current_exe().unwrap()).unwrap();
            std::fs::write(dir.join("programs/interface-fixture"), &program).unwrap();
            let digest = fieldcraft_io::identity::artifact_digest(&program);
            context.artifacts = fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES.into_iter().map(|role| (role.into(), digest.clone())).collect();
            let model_bytes = fieldcraft_io::write_with(
                &context.model,
                &context.geometry,
                &[],
                &fieldcraft_io::JournalHeader::default(),
                &fieldcraft_io::View::default(),
                &fieldcraft_io::Extras { run: context.run.clone(), ..Default::default() },
            )
            .unwrap();
            std::fs::write(dir.join("input.fieldcraft"), model_bytes).unwrap();
            std::fs::write(dir.join("fixture-source.txt"), b"protocol fixture").unwrap();
        }
        let provider = IoSnapshotProvider;
        let binding = provider.freeze(&context, &entities).unwrap();
        let e = production_envelope(&run, binding, entities.clone());
        let frozen_profile = profile(&e);
        let root = export
            .as_ref()
            .map(|dir| dir.join("store"))
            .unwrap_or_else(|| std::env::temp_dir().join(format!("fieldcraft-engine-result-store-{}", std::process::id())));
        let store = Arc::new(FileResultStore::new(&root).unwrap());
        let host = ReceiverHost::with_store(context, run.clone(), provider, store.clone()).unwrap();
        let tx = host.begin(&run, entities.clone(), frozen_profile.clone()).unwrap();
        assert!(matches!(host.begin(&run, entities.clone(), frozen_profile.clone()), Err(ContractError::InvalidState)));
        let preview = tx.validate(&e).unwrap();
        assert_eq!(preview.coverage, Coverage::Complete { last: 3 });
        let result_id = tx.commit(&e).unwrap();
        assert!(result_id.starts_with("fieldcraft-result/"));
        let reopened = store.reopen(&result_id).unwrap();
        assert_eq!(reopened, e);
        if let Some(dir) = &export {
            let values = [
                ("input-envelope.json", serde_json::to_value(&e).unwrap()),
                ("entities.json", serde_json::to_value(&entities).unwrap()),
                ("binding.json", serde_json::to_value(&e.binding).unwrap()),
                ("admission.json", serde_json::to_value(&preview).unwrap()),
                (
                    "request-profile.json",
                    serde_json::json!({
                        "run": run, "goal": e.goal, "evidence_kind": e.evidence_kind,
                        "source_scene": e.source_scene, "required_fields": e.required_fields,
                        "contact_controls": null, "control_evidence": null, "require_effective": false,
                        "policy_request": null, "require_policy_applied": false,
                    }),
                ),
            ];
            for (name, value) in values {
                std::fs::write(dir.join(name), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
            }
            let stored_path = format!("store/{}.json", result_id.strip_prefix("fieldcraft-result/").unwrap());
            let files: std::collections::BTreeMap<_, _> = [
                "input.fieldcraft",
                "fixture-source.txt",
                "programs/interface-fixture",
                "input-envelope.json",
                "entities.json",
                "binding.json",
                "admission.json",
                "request-profile.json",
                &stored_path,
            ]
            .into_iter()
            .map(|name| (name, fieldcraft_io::sha256_hex(&std::fs::read(dir.join(name)).unwrap())))
            .collect();
            let manifest = serde_json::json!({
                "schema": "fieldcraft-external-fixture-package/1", "result_id": result_id, "files": files,
                "evidence_class": "protocol_fixture", "runtime_qualification": "unverified",
                "numerical_solve_verified": false, "libuipc_called": false,
                "wire_labels": "Solver/Succeeded are injected fixture claims used only to exercise the storage gate",
                "artifact_roles": "producer, adapter and receiver are the same retained CPU test executable",
                "source_locator": "fixture/frame is the synthetic factory in external_contract.rs, not a numerical observation",
            });
            std::fs::write(dir.join("package.json"), serde_json::to_vec_pretty(&manifest).unwrap()).unwrap();
        }
        let tx = host.begin(&run, entities.clone(), frozen_profile).unwrap();
        assert_eq!(tx.validate(&reopened).unwrap().coverage, Coverage::Complete { last: 3 });
        drop(tx);
        println!(
            "production_store_receipt={}",
            serde_json::json!({
                "run": run,
                "result_id": result_id,
                "reopened_equal": reopened == e,
                "coverage": preview.coverage,
                "model_revision": e.binding.snapshot.model_revision,
                "reference_mesh_digest": e.binding.snapshot.reference_mesh_digest,
                "entity_map_digest": e.binding.entity_map_digest,
                "backend_version": e.binding.backend_version,
            })
        );
        let mut forged = reopened.clone();
        forged.binding.snapshot.model_revision = fieldcraft_io::sha256_hex(b"forged production revision");
        let tx = host.begin(&run, entities, profile(&reopened)).unwrap();
        let mismatch = tx.commit(&forged);
        println!("production_identity_rejection={}", serde_json::json!({"error": format!("{mismatch:?}")}));
        assert!(matches!(mismatch, Err(ContractError::IdentityMismatch(_))));
        if export.is_none() {
            std::fs::remove_dir_all(root).unwrap();
        }
    }

    #[test]
    fn production_identity_rejects_model_and_mapping_changes_at_the_running_commit_boundary() {
        let run = RunKey { run_id: "production-mismatch".into(), epoch: 1, worker_generation: "production-worker".into() };
        for change_mesh in [true, false] {
            let (context, entities) = production_context();
            let provider = IoSnapshotProvider;
            let binding = provider.freeze(&context, &entities).unwrap();
            let e = production_envelope(&run, binding, entities.clone());
            let host = ReceiverHost::new(context, run.clone(), provider).unwrap();
            let tx = host.begin(&run, entities.clone(), profile(&e)).unwrap();
            host.update_model(|current| {
                if change_mesh {
                    current.model.set_node(2, [2.0, 0.0, 0.0]).unwrap();
                } else {
                    current.model.set_element(8, fieldcraft_model::ElementKind::Tet4, vec![1, 3, 2, 4]).unwrap();
                }
            })
            .unwrap();
            let mismatch = tx.commit(&e);
            println!("production_running_identity_rejection={}", serde_json::json!({"changed_mesh": change_mesh, "error": format!("{mismatch:?}")}));
            assert!(matches!(mismatch, Err(ContractError::IdentityMismatch(_))));
        }
        let (context, entities) = production_context();
        let provider = IoSnapshotProvider;
        let binding = provider.freeze(&context, &entities).unwrap();
        let e = production_envelope(&run, binding, entities.clone());
        let host = ReceiverHost::new(context, run.clone(), provider).unwrap();
        let mut wrong = entities;
        wrong.nodes.swap(0, 1);
        let mismatch = host.begin(&run, wrong, profile(&e));
        let error = mismatch.as_ref().err().map(ToString::to_string).unwrap_or_else(|| "unexpected success".into());
        println!("production_mapping_rejection={}", serde_json::json!({"error": error}));
        assert!(matches!(mismatch, Err(ContractError::Provider(_))));
    }

    #[test]
    fn production_policy_application_is_verified_only_when_the_native_field_is_stored() {
        let mut session = fieldcraft_engine::Session::default();
        session.run_script(&serde_json::from_str(include_str!("fixtures/contact-stick.json")).unwrap()).unwrap();
        let policy_value = serde_json::json!({"Interface":"stick"});
        let geometry = fieldcraft_io::Geometry::default();
        let io_run = fieldcraft_io::Run::default();
        let request = fieldcraft_io::contact_policy::PolicyRequest::freeze(&session.model, &geometry, &io_run, "Modes", &policy_value).unwrap();
        let policy_bytes = serde_json::to_vec(&request).unwrap();
        let payload = PolicyRequestPayload {
            wire_sha256: fieldcraft_io::sha256_hex(&policy_bytes),
            bytes: policy_bytes,
            request_digest: request.digest().unwrap(),
        };
        let run = RunKey { run_id: "policy-production".into(), epoch: 1, worker_generation: "policy-worker".into() };
        let (context, entities) = {
            let artifacts = fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES
                .into_iter()
                .map(|role| (role.into(), fieldcraft_io::identity::artifact_digest(role.as_bytes())))
                .collect();
            (IoModelContext::new(session.model.clone(), geometry.clone(), io_run.clone(), artifacts), entities_for_model(&session.model))
        };
        let provider = IoSnapshotProvider;
        let binding = provider.freeze(&context, &entities).unwrap();
        let mut e = production_envelope(&run, binding, entities.clone());
        e.policy_request_digest = Some(payload.request_digest.clone());
        let mut p = profile(&e);
        p.policy_request = Some(payload.clone());
        p.require_policy_applied = true;
        let root = std::env::temp_dir().join(format!("applied-policy-unqualified-{}", std::process::id()));
        let store = Arc::new(FileResultStore::new(&root).unwrap());
        let host = ReceiverHost::with_store(context, run.clone(), provider, store).unwrap();
        let tx = host.begin(&run, entities.clone(), p).unwrap();
        let admission = tx.validate(&e).unwrap();
        assert_eq!(admission.solve, SolveOutcome::Unknown);
        assert!(admission.blockers.iter().any(|b| b.starts_with("RuntimeQualificationUnverified:")));
        assert!(!admission.blockers.iter().any(|b| b.starts_with("NativeContactPolicyNotApplied:")));
        let policy = admission.policy.unwrap();
        assert_eq!(policy.declaration, EvidenceCheck::Verified);
        assert_eq!(policy.application, EvidenceCheck::Verified);
        assert_eq!(policy.runtime_qualification, EvidenceCheck::Unverified);
        println!(
            "production_policy_applied={}",
            serde_json::json!({
                "declaration": policy.declaration,
                "application": policy.application,
                "runtime_qualification": policy.runtime_qualification,
                "blockers": admission.blockers,
            })
        );
        assert!(matches!(tx.commit(&e), Err(ContractError::Invalid("result store requires a complete solver success"))));
        assert_eq!(std::fs::read_dir(&root).unwrap().count(), 0);
        std::fs::remove_dir(&root).unwrap();

        let mut no_policy = fieldcraft_engine::Session::default();
        no_policy.run_script(&serde_json::from_str(include_str!("fixtures/contact-stick.json")).unwrap()).unwrap();
        no_policy.execute("model.step.edit", &serde_json::json!({"name":"Modes","contact_perturbation":{}})).unwrap();
        let (context, entities) = {
            let artifacts = fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES
                .into_iter()
                .map(|role| (role.into(), fieldcraft_io::identity::artifact_digest(role.as_bytes())))
                .collect();
            let entities = entities_for_model(&no_policy.model);
            (IoModelContext::new(no_policy.model, geometry, io_run, artifacts), entities)
        };
        let request =
            fieldcraft_io::contact_policy::PolicyRequest::freeze(&context.model, &context.geometry, &context.run, "Modes", &policy_value).unwrap();
        let bytes = serde_json::to_vec(&request).unwrap();
        let payload = PolicyRequestPayload { wire_sha256: fieldcraft_io::sha256_hex(&bytes), bytes, request_digest: request.digest().unwrap() };
        let provider = IoSnapshotProvider;
        let binding = provider.freeze(&context, &entities).unwrap();
        let mut e = production_envelope(&run, binding, entities.clone());
        e.policy_request_digest = Some(payload.request_digest.clone());
        let mut p = profile(&e);
        p.policy_request = Some(payload);
        p.require_policy_applied = true;
        let host = ReceiverHost::new(context, run, provider).unwrap();
        let rejection = host.begin(&e.run, entities, p);
        let error = rejection.as_ref().err().map(ToString::to_string).unwrap_or_else(|| "unexpected success".into());
        println!("production_policy_unapplied={}", serde_json::json!({"error": error}));
        assert!(matches!(rejection, Err(ContractError::NativeContactPolicyNotApplied)));
    }
    /// Mutate only owned copies of the protocol fixture; no numerical qualification is claimed.
    #[test]
    fn consumer_rejects_wrong_run_payload_tampering_and_missing_frame() {
        let run = RunKey { run_id: "consumer-refusal".into(), epoch: 1, worker_generation: "fixture-worker".into() };
        let (context, entities) = production_context();
        let binding = IoSnapshotProvider.freeze(&context, &entities).unwrap();
        let original = production_envelope(&run, binding, entities.clone());
        let before = serde_json::to_vec(&original).unwrap();
        let root = std::env::temp_dir().join(format!("consumer-refusals-{}", std::process::id()));
        let store = Arc::new(FileResultStore::new(&root).unwrap());
        let host = ReceiverHost::with_store(context, run.clone(), IoSnapshotProvider, store.clone()).unwrap();
        for case in 0..3 {
            let mut changed = original.clone();
            match case {
                0 => changed.run.epoch += 1,
                1 => {
                    let FieldData::Present { bytes, .. } = &mut changed.frames[0].fields[0].data else { panic!("fixture data") };
                    bytes[0] ^= 1;
                }
                _ => {
                    changed.frames.pop().unwrap();
                }
            }
            let tx = host.begin(&run, entities.clone(), profile(&original)).unwrap();
            if case == 2 {
                let admission = tx.validate(&changed).unwrap();
                assert_ne!(admission.coverage, Coverage::Complete { last: 3 });
                assert!(!admission.blockers.is_empty());
            }
            let error = tx.commit(&changed).unwrap_err();
            match case {
                0 => assert_eq!(error, ContractError::StaleRun),
                1 => assert_eq!(error, ContractError::ChecksumMismatch),
                _ => assert!(matches!(error, ContractError::Invalid("result store requires a complete solver success"))),
            }
            println!("consumer_refusal case={case} error={error:?}");
            assert_eq!(std::fs::read_dir(&root).unwrap().count(), 0, "rejected input must not publish");
        }
        assert_eq!(serde_json::to_vec(&original).unwrap(), before);
        let tx = host.begin(&run, entities, profile(&original)).unwrap();
        let id = tx.commit(&original).unwrap();
        assert_eq!(store.reopen(&id).unwrap(), original);
        std::fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn native_document_and_received_result_keep_identity_across_disk_reopen() {
        use serde_json::json;

        let export = std::env::var_os("FIELDCRAFT_CROSS_IDENTITY_EVIDENCE").map(std::path::PathBuf::from);
        let root = export.clone().unwrap_or_else(|| std::env::temp_dir().join(format!("external-cross-identity-{}", std::process::id())));
        std::fs::create_dir(&root).unwrap();
        std::fs::create_dir(root.join("programs")).unwrap();
        let program = std::fs::read(std::env::current_exe().unwrap()).unwrap();
        std::fs::write(root.join("programs/interface-fixture"), &program).unwrap();
        let digest = fieldcraft_io::identity::artifact_digest(&program);
        let (mut context, entities) = production_context();
        context.artifacts = fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES.into_iter().map(|role| (role.into(), digest.clone())).collect();
        let native = fieldcraft_io::write_with(
            &context.model,
            &context.geometry,
            &[],
            &fieldcraft_io::JournalHeader::default(),
            &fieldcraft_io::View::default(),
            &fieldcraft_io::Extras { run: context.run.clone(), ..Default::default() },
        )
        .unwrap();
        std::fs::write(root.join("input.fieldcraft"), native).unwrap();
        std::fs::write(root.join("fixture-source.txt"), b"protocol fixture").unwrap();
        let run = RunKey { run_id: "native-file-cross-identity".into(), epoch: 1, worker_generation: "cpu-protocol-fixture".into() };
        let binding = IoSnapshotProvider.freeze(&context, &entities).unwrap();
        let mut input = production_envelope(&run, binding, entities.clone());
        // Optional velocity rows stay unavailable, not synthetic zeroes.
        for frame in &mut input.frames {
            let velocity = frame.fields.iter_mut().find(|f| f.spec.quantity == Quantity::Velocity).unwrap();
            let FieldData::Present { bytes, available, sha256, .. } = &mut velocity.data else { panic!("velocity") };
            *available = vec![true, false, true, false];
            bytes.truncate(6 * 8);
            *sha256 = fieldcraft_io::sha256_hex(bytes);
        }
        std::fs::write(root.join("input-envelope.json"), serde_json::to_vec_pretty(&input).unwrap()).unwrap();
        let received: ResultEnvelope = serde_json::from_slice(&std::fs::read(root.join("input-envelope.json")).unwrap()).unwrap();
        let store = Arc::new(FileResultStore::new(root.join("store")).unwrap());
        let host = ReceiverHost::with_store(context, run.clone(), IoSnapshotProvider, store).unwrap();
        let tx = host.begin(&run, entities.clone(), profile(&received)).unwrap();
        assert_eq!(tx.binding(), &received.binding);
        let admission = tx.validate(&received).unwrap();
        assert_eq!(admission.transfer, TransferState::Verified);
        assert_eq!(admission.coverage, Coverage::Complete { last: 3 });
        assert_eq!(admission.solve, SolveOutcome::Succeeded, "injected wire label, not numerical evidence");
        let id = tx.commit(&received).unwrap();
        drop(host);

        // Reconstruct BOTH owners from disk, including the actual executable
        // bytes. Neither the original model nor its store is the new authority.
        let document = fieldcraft_io::read(&std::fs::read(root.join("input.fieldcraft")).unwrap()).unwrap();
        let disk_digest = fieldcraft_io::identity::artifact_digest(&std::fs::read(root.join("programs/interface-fixture")).unwrap());
        let disk_context = IoModelContext::new(
            document.model,
            document.geometry,
            document.run,
            fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES.into_iter().map(|role| (role.into(), disk_digest.clone())).collect(),
        );
        assert_eq!(IoSnapshotProvider.freeze(&disk_context, &entities).unwrap(), input.binding);
        let disk_store = Arc::new(FileResultStore::new(root.join("store")).unwrap());
        let reopened = disk_store.reopen(&id).unwrap();
        assert_eq!(reopened, input, "every field byte, mask, frame and binding");
        let fresh = ReceiverHost::with_store(disk_context.clone(), run.clone(), IoSnapshotProvider, disk_store.clone()).unwrap();
        let tx = fresh.begin(&run, entities.clone(), profile(&reopened)).unwrap();
        assert_eq!(tx.validate(&reopened).unwrap(), admission);
        drop(tx);
        assert!(matches!(
            reopened.frames[0].fields.iter().find(|f| f.spec.quantity == Quantity::ContactEnergy).unwrap().data,
            FieldData::Unavailable { .. }
        ));
        let stored_name = format!("store/{}.json", id.strip_prefix("fieldcraft-result/").unwrap());
        let original_store = std::fs::read(root.join(&stored_name)).unwrap();
        let mut refusals = Vec::new();
        for mutation in ["malformed_mesh_identity", "revision", "mesh", "program", "mapping", "run", "payload"] {
            let mut wrong = reopened.clone();
            match mutation {
                "revision" => wrong.binding.snapshot.model_revision = fieldcraft_io::sha256_hex(b"wrong revision"),
                "malformed_mesh_identity" => wrong.binding.snapshot.reference_mesh_digest = fieldcraft_io::sha256_hex(b"wrong mesh"),
                "mesh" => {
                    wrong.binding.snapshot.reference_mesh_digest = fieldcraft_io::sha256_hex(b"wrong mesh");
                    wrong.binding.snapshot.model_id = fieldcraft_io::identity::model_id(&wrong.binding.snapshot.reference_mesh_digest);
                }
                "program" => {
                    wrong.binding.artifacts.insert("receiver".into(), fieldcraft_io::sha256_hex(b"wrong executable"));
                }
                "mapping" => wrong.entities.nodes.swap(0, 1),
                "run" => wrong.run.epoch += 1,
                "payload" => {
                    let FieldData::Present { bytes, .. } = &mut wrong.frames[0].fields[0].data else { panic!("present") };
                    bytes[0] ^= 1;
                }
                _ => unreachable!(),
            }
            let error = fresh.begin(&run, entities.clone(), profile(&reopened)).unwrap().commit(&wrong).unwrap_err();
            match mutation {
                "malformed_mesh_identity" => assert_eq!(error, ContractError::Invalid("snapshot identity fields")),
                "run" => assert_eq!(error, ContractError::StaleRun),
                "payload" => assert_eq!(error, ContractError::ChecksumMismatch),
                _ => assert!(matches!(error, ContractError::IdentityMismatch(_)), "{mutation}: {error:?}"),
            }
            refusals.push(json!({"case":mutation,"error":format!("{error:?}")}));
        }
        // Native identity is rechecked under the live transaction's commit lock.
        for mutation in ["live_mesh", "live_revision", "live_program"] {
            let edited = ReceiverHost::with_store(disk_context.clone(), run.clone(), IoSnapshotProvider, disk_store.clone()).unwrap();
            let tx = edited.begin(&run, entities.clone(), profile(&reopened)).unwrap();
            edited
                .update_model(|current| match mutation {
                    "live_mesh" => current.model.set_node(2, [2.0, 0.0, 0.0]).unwrap(),
                    "live_revision" => current.model.set_element(8, fieldcraft_model::ElementKind::Tet4, vec![1, 3, 2, 4]).unwrap(),
                    "live_program" => {
                        current.artifacts.insert("receiver".into(), fieldcraft_io::sha256_hex(b"replacement executable"));
                    }
                    _ => unreachable!(),
                })
                .unwrap();
            let error = tx.commit(&reopened).unwrap_err();
            assert!(matches!(error, ContractError::IdentityMismatch(_)), "{mutation}: {error:?}");
            refusals.push(json!({"case":mutation,"error":format!("{error:?}")}));
        }
        assert_eq!(std::fs::read_dir(root.join("store")).unwrap().count(), 1, "refusals publish nothing");
        assert_eq!(std::fs::read(root.join(&stored_name)).unwrap(), original_store);
        assert_eq!(disk_store.reopen(&id).unwrap(), input);
        // Rehashing a changed payload still cannot retain the old result id.
        let corrupt_store = FileResultStore::new(root.join("tampered-store")).unwrap();
        let mut stored: serde_json::Value = serde_json::from_slice(&original_store).unwrap();
        let mut changed = reopened.clone();
        changed.source_scene.locator = "changed after publication".into();
        stored["envelope"] = serde_json::to_value(&changed).unwrap();
        stored["payload_sha256"] = json!(fieldcraft_io::sha256_hex(&serde_json::to_vec(&changed).unwrap()));
        std::fs::write(
            root.join("tampered-store").join(format!("{}.json", id.strip_prefix("fieldcraft-result/").unwrap())),
            serde_json::to_vec(&stored).unwrap(),
        )
        .unwrap();
        assert_eq!(corrupt_store.reopen(&id), Err(ContractError::ChecksumMismatch));
        refusals.push(json!({"case":"stored_payload_rehash","error":"ChecksumMismatch"}));

        for (name, value) in [
            ("entities.json", serde_json::to_value(&entities).unwrap()),
            ("binding.json", serde_json::to_value(&input.binding).unwrap()),
            ("admission.json", serde_json::to_value(&admission).unwrap()),
            (
                "request-profile.json",
                json!({"run":run,"goal":input.goal,"evidence_kind":input.evidence_kind,
                "source_scene":input.source_scene,"required_fields":input.required_fields,"contact_controls":null,"control_evidence":null,
                "require_effective":false,"policy_request":null,"require_policy_applied":false}),
            ),
            ("refusals.json", json!(refusals)),
        ] {
            std::fs::write(root.join(name), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
        }
        // Consumable by external_fixture_reopen in an independent process.
        let files: std::collections::BTreeMap<_, _> = [
            "input.fieldcraft",
            "fixture-source.txt",
            "programs/interface-fixture",
            "input-envelope.json",
            "entities.json",
            "binding.json",
            "admission.json",
            "request-profile.json",
            &stored_name,
        ]
        .into_iter()
        .map(|name| (name, fieldcraft_io::sha256_hex(&std::fs::read(root.join(name)).unwrap())))
        .collect();
        std::fs::write(
            root.join("package.json"),
            serde_json::to_vec_pretty(&json!({
                "schema":"fieldcraft-external-fixture-package/1","result_id":id,"files":files,
                "evidence_class":"protocol_fixture","runtime_qualification":"unverified","numerical_solve_verified":false,"libuipc_called":false,
                "wire_labels":"Solver/Succeeded are injected claims for the storage gate, not numerical observations",
                "artifact_roles":"all three roles bind the actual retained CPU test executable"
            }))
            .unwrap(),
        )
        .unwrap();
        println!(
            "CROSS_IDENTITY {}",
            json!({"result_id":id,"reopened_equal":true,"refusals":refusals,
            "binding":input.binding,"evidence_class":"protocol_fixture","libuipc_called":false})
        );
        if export.is_none() {
            std::fs::remove_dir_all(root).unwrap();
        }
    }
    fn open_policy_profile(context: &IoModelContext, e: &ResultEnvelope, value: serde_json::Value) -> ReceiveProfile {
        let request = fieldcraft_io::contact_policy::PolicyRequest::freeze(&context.model, &context.geometry, &context.run, "Modes", &value).unwrap();
        let bytes = serde_json::to_vec_pretty(&request).unwrap();
        let mut p = profile(e);
        p.policy_request =
            Some(PolicyRequestPayload { wire_sha256: fieldcraft_io::sha256_hex(&bytes), bytes, request_digest: request.digest().unwrap() });
        p.require_policy_applied = true;
        p
    }

    #[test]
    // A solved native specimen proves the policy can be applied. The received frames
    // remain a separate protocol fixture: they carry no trusted runtime qualification
    // bound to that base state. Declaration/application must not promote them to a
    // publishable solver result. A qualified successful round trip needs that contract.
    fn applied_open_policy_without_runtime_qualification_is_refused_at_commit() {
        use serde_json::{Value, json};
        let export = std::env::var_os("FIELDCRAFT_CROSS_OPEN_EVIDENCE").map(std::path::PathBuf::from);
        let root = export.clone().unwrap_or_else(|| std::env::temp_dir().join(format!("external-open-{}", std::process::id())));
        std::fs::create_dir(&root).unwrap();
        std::fs::create_dir(root.join("programs")).unwrap();
        let program = std::fs::read(std::env::current_exe().unwrap()).unwrap();
        std::fs::write(root.join("programs/interface-fixture"), &program).unwrap();
        let digest = fieldcraft_io::identity::artifact_digest(&program);
        let mut script: Value = serde_json::from_str(include_str!("fixtures/contact-tangent-free.json")).unwrap();
        for c in script.as_array_mut().unwrap() {
            if c["command"] == "mesh.node.add" && c["params"]["label"].as_u64().unwrap() >= 9 {
                c["params"]["xyz"][2] = json!(c["params"]["xyz"][2].as_f64().unwrap() + 0.02);
            }
            if c["command"] == "model.boundary_condition.create" && c["params"]["name"] == "top" {
                c["params"]["values"][2] = json!(0.0);
            }
            if c["command"] == "model.step.create" && c["params"]["name"] == "Modes" {
                c["params"]["contact_perturbation"]["Interface"] = json!("open");
            }
        }
        std::fs::write(root.join("native-input.json"), serde_json::to_vec_pretty(&script).unwrap()).unwrap();
        let mut s = fieldcraft_engine::Session::default();
        s.run_script(&script).unwrap();
        s.execute("solve.nonlinear", &json!({"step":"Preload","threads":1})).unwrap();
        let modes = s.execute("solve.frequency", &json!({"step":"Modes","threads":1})).unwrap();
        std::fs::write(root.join("native-mode-reply.json"), serde_json::to_vec_pretty(&modes).unwrap()).unwrap();
        let info = s.execute("model.info", &json!({})).unwrap();
        assert_eq!(info["contact_policy_capabilities"]["steps"]["Modes"]["selection"], "explicit_open");
        assert_eq!(info["contact_policy_capabilities"]["steps"]["Modes"]["runtime_qualification"], "unverified");
        let entities = entities_for_model(&s.model);
        let mut artifacts: std::collections::BTreeMap<String, String> =
            fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES.into_iter().map(|role| (role.into(), digest.clone())).collect();
        artifacts.insert("observer".into(), digest);
        let context = IoModelContext::new(s.model, fieldcraft_io::Geometry::default(), fieldcraft_io::Run::default(), artifacts);
        let native = fieldcraft_io::write_with(
            &context.model,
            &context.geometry,
            &[],
            &fieldcraft_io::JournalHeader::default(),
            &fieldcraft_io::View::default(),
            &fieldcraft_io::Extras { run: context.run.clone(), ..Default::default() },
        )
        .unwrap();
        std::fs::write(root.join("input.fieldcraft"), native).unwrap();
        let run = RunKey { run_id: "native-open-policy-cross".into(), epoch: 1, worker_generation: "cpu-protocol-fixture".into() };
        let binding = IoSnapshotProvider.freeze(&context, &entities).unwrap();
        assert_eq!(binding.artifacts.len(), 4, "required roles are a minimum; a legitimate extra role is retained");
        let mut e = production_envelope(&run, binding, entities.clone());
        let p = open_policy_profile(&context, &e, json!({"Interface":"open"}));
        e.policy_request_digest = Some(p.policy_request.as_ref().unwrap().request_digest.clone());
        std::fs::write(root.join("policy-request.json"), &p.policy_request.as_ref().unwrap().bytes).unwrap();
        std::fs::write(root.join("input-envelope.json"), serde_json::to_vec_pretty(&e).unwrap()).unwrap();
        let e: ResultEnvelope = serde_json::from_slice(&std::fs::read(root.join("input-envelope.json")).unwrap()).unwrap();
        let store = Arc::new(FileResultStore::new(root.join("store")).unwrap());
        let host = ReceiverHost::with_store(context.clone(), run.clone(), IoSnapshotProvider, store.clone()).unwrap();
        let tx = host.begin(&run, entities.clone(), p.clone()).unwrap();
        let admission = tx.validate(&e).unwrap();
        assert_eq!(admission.transfer, TransferState::Verified);
        assert_eq!(admission.coverage, Coverage::Complete { last: 3 });
        assert_eq!(admission.commit, CommitState::NotAttempted);
        assert_eq!(admission.solve, SolveOutcome::Unknown);
        let policy = admission.policy.as_ref().unwrap();
        assert_eq!(policy.declaration, EvidenceCheck::Verified);
        assert_eq!(policy.application, EvidenceCheck::Verified);
        assert_eq!(policy.runtime_qualification, EvidenceCheck::Unverified);
        // The legacy blocker text conflates application and qualification. The typed
        // admission above identifies the missing qualification, not an unapplied policy.
        let error = tx.commit(&e).unwrap_err();
        assert_eq!(error, ContractError::Invalid("result store requires a complete solver success"));
        let stored_files = std::fs::read_dir(root.join("store")).unwrap().count();
        assert_eq!(stored_files, 0, "unqualified policy must not create a published result");
        // Failed commit releases its lease; begin/validate still confirm the same
        // applied declaration. This is not a successful commit or result readback.
        let retry = host.begin(&run, entities, p).unwrap();
        assert_eq!(retry.validate(&e).unwrap(), admission);
        drop(retry);
        assert_eq!(std::fs::read_dir(root.join("store")).unwrap().count(), 0);
        let receipt = json!({
            "case":"applied_open_policy_without_runtime_qualification",
            "admission":admission,"commit_error":error.to_string(),"commit_debug":format!("{error:?}"),
            "refusal_reason":"runtime qualification unverified after declaration and application verified",
            "stored_files":stored_files,"lease_released":true,"native_model_info":info,
            "external_evidence_class":"protocol_fixture","external_numerical_solve_verified":false,
            "successful_policy_commit_verified":false,"successful_policy_reopen_verified":false,
            "extra_role_accepted":true
        });
        std::fs::write(root.join("qualification-refusal.json"), serde_json::to_vec_pretty(&receipt).unwrap()).unwrap();
        std::fs::write(root.join("admission.json"), serde_json::to_vec_pretty(&admission).unwrap()).unwrap();
        println!("OPEN_QUALIFICATION_REFUSAL {receipt}");
        if export.is_none() {
            std::fs::remove_dir_all(root).unwrap();
        }
    }
}
