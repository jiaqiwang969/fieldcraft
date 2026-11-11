use fieldcraft_engine::Session;
use fieldcraft_engine::external::load_transfer::*;
use fieldcraft_io::identity;
use fieldcraft_model::{BoundaryCondition, Model, Procedure, Set, Step, Target};
use serde_json::{Value, json};
use std::fs;
use std::path::{Path, PathBuf};

fn bundle_for(model: &Model) -> ArchiveBundle {
    let snapshot = identity::snapshot(model, &fieldcraft_io::Geometry::default(), &fieldcraft_io::Run::default()).unwrap();
    let activation = ActivationData {
        frames: vec![
            ActivationFrame {
                frame_index: 0,
                source_active: vec![ActivationPoint { contact_id: "support".into(), weight: 1.0, gap_m: 0.0 }],
                feedback_active: Some(vec![ActivationPoint { contact_id: "support".into(), weight: 1.0, gap_m: 0.0 }]),
                source_peak_force_n: Some(1.0),
                feedback_peak_force_n: Some(1.0),
            },
            ActivationFrame {
                frame_index: 1,
                source_active: vec![ActivationPoint { contact_id: "support".into(), weight: 1.0, gap_m: 0.0 }],
                feedback_active: Some(vec![ActivationPoint { contact_id: "support".into(), weight: 1.0, gap_m: 0.0 }]),
                source_peak_force_n: Some(1.0),
                feedback_peak_force_n: Some(1.0),
            },
        ],
        activation_length_m: 1.0,
        required_fields: true,
        structural_feedback: true,
        force_precision: ForcePrecisionLimit { kappa_pa: UnknownOr::unknown("not exposed"), truncated: UnknownOr::unknown("not exposed") },
    };
    let qualification = RuntimeQualification::Unverified { reason: "independent qualification is not supplied by an incoming result".into() };
    let applicability = evaluate_applicability(&activation.applicability_input().unwrap(), &qualification).unwrap();
    LoadTransferArchive {
        identity: LoadIdentity {
            archive_key: "support-load".into(),
            run_id: "entry-test".into(),
            epoch: 1,
            worker_generation: "test".into(),
            model_revision: snapshot.model_revision,
            mesh_digest: snapshot.reference_mesh_digest,
            coordinate_frame: "world".into(),
            units: "SI".into(),
            producer_sha256: fieldcraft_io::sha256_hex(b"entry-test-producer-artifact"),
            adapter_sha256: fieldcraft_io::sha256_hex(b"entry-test-adapter-artifact"),
        },
        mapping: MappingSpec {
            origin_m: [0.0, 0.0, 0.0],
            nodes: vec![MappingNode { node_id: 1, position_m: [0.0, 0.0, 0.0] }],
            rows: vec![PointMapping {
                point_id: "p0".into(),
                contributions: vec![NodeContribution { node_id: 1, h_block: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] }],
            }],
            mapping_sha256: "a".repeat(64),
        },
        frames: vec![
            LoadFrame {
                frame_index: 0,
                time_s: 0.0,
                source_dt_s: 1.0,
                points: vec![PointForce { point_id: "p0".into(), position_m: [0.0, 0.0, 0.0], force_n: [0.0, 0.0, 0.0] }],
                facts: FrameFacts {
                    completed: true,
                    converged: true,
                    hit_newton_limit: false,
                    hit_line_search_limit: false,
                    kappa_pa: UnknownOr::unknown("not exposed"),
                    truncated: UnknownOr::unknown("not exposed"),
                },
            },
            LoadFrame {
                frame_index: 1,
                time_s: 1.0,
                source_dt_s: 1.0,
                points: vec![PointForce { point_id: "p0".into(), position_m: [0.0, 0.0, 0.0], force_n: [1.0, 0.0, 0.0] }],
                facts: FrameFacts {
                    completed: true,
                    converged: true,
                    hit_newton_limit: false,
                    hit_line_search_limit: false,
                    kappa_pa: UnknownOr::unknown("not exposed"),
                    truncated: UnknownOr::unknown("not exposed"),
                },
            },
        ],
        activation,
        qualification,
        applicability,
        crash: None,
    }
    .encode()
    .unwrap()
}

#[test]
fn receive_save_reopen_resave_is_byte_identical_and_apply_creates_native_history() {
    let dir = std::env::temp_dir().join(format!("fieldcraft-load-transfer-{}", std::process::id()));
    fs::create_dir_all(&dir).unwrap();
    let bundle_path = dir.join("support-load.json");
    let project_path = dir.join("support.fieldcraft");
    let mut session = Session::default();
    session.model.set_node(1, [0.0, 0.0, 0.0]).unwrap();
    let bundle = bundle_for(&session.model);
    fs::write(&bundle_path, serde_json::to_vec(&bundle).unwrap()).unwrap();

    let reply =
        session.execute("results.external.load_transfer.receive", &json!({"path":bundle_path,"name":"support-load","project":project_path})).unwrap();
    assert_eq!(reply["project_saved"], true);
    assert_eq!(reply["conservation"]["all_frames_passed"], true);
    assert_eq!(reply["applicability"]["geometry_status"], "applicable");
    assert_eq!(reply["runtime_qualification"]["state"], "unverified");
    assert_eq!(reply["backend_status"]["state"], "completed");
    assert_eq!(reply["backend_status"]["solver_success"], true);
    assert_eq!(reply["backend_status"]["coverage_through_frame"], 1);
    assert_eq!(reply["program_identity"]["producer_sha256"], fieldcraft_io::sha256_hex(b"entry-test-producer-artifact"));
    assert_eq!(reply["program_identity"]["adapter_sha256"], fieldcraft_io::sha256_hex(b"entry-test-adapter-artifact"));
    assert_eq!(reply["program_identity"]["receiver_sha256"].as_str().unwrap().len(), 64);
    let first = fs::read(&project_path).unwrap();

    let mut reopened = Session::default();
    reopened.execute("file.open", &json!({"path":project_path})).unwrap();
    reopened.execute("file.save", &json!({"path":project_path,"results":true})).unwrap();
    let second = fs::read(&project_path).unwrap();
    assert_eq!(first, second, "project kind-13 payload changed across reopen/save");
    let applied = reopened.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap();
    assert_eq!(applied["ready_for"], "solve.transient");
    assert_eq!(applied["nodes"], 1);
    assert_eq!(applied["load_components"], 1);
    assert_eq!(applied["program_identity"]["receiver_sha256"], reply["program_identity"]["receiver_sha256"]);
    assert_eq!(applied["program_identity"]["current_receiver_sha256"], reply["program_identity"]["receiver_sha256"]);
    assert_eq!(applied["transient_settings"]["output_every"], 1);
    assert_eq!(applied["transient_settings"]["integration"]["type"], "average_acceleration");
    assert!(applied["boundary_condition_warning"].is_string());
    let _ = fs::remove_dir_all(dir);
}

fn failed_receipt(state: BackendState, exit_code: Option<i32>, signal: Option<i32>, first_error: Option<u64>) -> CrashReceipt {
    CrashReceipt {
        state,
        exit_code,
        signal,
        first_error: first_error.map(|frame_index| FirstError {
            frame_index: Some(frame_index),
            phase: "solve".into(),
            kind: "native_failure".into(),
            message: "real backend failure; qualified prefix retained".into(),
        }),
        last_completed_frame: Some(1),
        last_persisted_frame: Some(1),
        requested_last_frame: Some(3),
        stdout_sha256: "a".repeat(64),
        stderr_sha256: "b".repeat(64),
    }
}

#[test]
fn product_entry_accepts_failed_and_signal_records_but_never_applies_them() {
    for (label, receipt) in [
        ("failed", failed_receipt(BackendState::Failed, Some(2), None, Some(2))),
        ("signal", failed_receipt(BackendState::Failed, None, Some(11), None)),
    ] {
        let dir = test_dir(&format!("backend-{label}"));
        let project = dir.join(format!("{label}.fieldcraft"));
        let mut session = session_with_node();
        let mut archive = LoadTransferArchive::decode(&bundle_for(&session.model)).unwrap().archive;
        archive.crash = Some(receipt);
        let path = write_bundle(&dir, label, &archive.encode().unwrap());
        let reply = session.execute("results.external.load_transfer.receive", &json!({"path":path,"name":"support-load","project":project})).unwrap();
        assert_eq!(reply["backend_status"]["state"], "failed");
        assert_eq!(reply["backend_status"]["solver_success"], false);
        assert_eq!(reply["backend_status"]["last_completed_frame"], 1);
        assert_eq!(reply["backend_status"]["last_persisted_frame"], 1);
        assert_eq!(reply["backend_status"]["requested_last_frame"], 3);
        assert_eq!(reply["backend_status"]["coverage_through_frame"], 1);
        assert_eq!(reply["apply_allowed"], false);
        assert_eq!(reply["structural_step"], "not_ready_for_results.external.load_transfer.apply");
        let first = fs::read(&project).unwrap();

        let mut reopened = Session::default();
        reopened.execute("file.open", &json!({"path":project})).unwrap();
        let listed = reopened.execute("results.external.list", &json!({})).unwrap();
        assert_eq!(listed[0]["backend_status"]["state"], "failed");
        assert_eq!(listed[0]["backend_status"]["coverage_through_frame"], 1);
        reopened.execute("file.save", &json!({"path":project,"results":true})).unwrap();
        let second = fs::read(&project).unwrap();
        assert_eq!(first, second, "failed backend record changed across reopen/save");
        let error = reopened.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap_err().to_string();
        assert!(error.contains("refused backend status") && error.contains("last_completed_frame"), "{error}");
        let _ = fs::remove_dir_all(dir);
    }
}

#[test]
fn product_entry_rejects_bad_frame_facts_and_crash_frame_mismatch() {
    let dir = test_dir("backend-admission-rejects");
    let mut bad_frame = session_with_node();
    let mut archive = LoadTransferArchive::decode(&bundle_for(&bad_frame.model)).unwrap().archive;
    archive.frames[1].facts.converged = false;
    let path = write_bundle(&dir, "bad-frame", &archive.encode().unwrap());
    let error = receive_path(&mut bad_frame, &path, "support-load").unwrap_err();
    assert!(error.contains("frame 1") && error.contains("item=converged"), "{error}");
    no_external_results(&mut bad_frame);

    let mut mismatch = session_with_node();
    let mut archive = LoadTransferArchive::decode(&bundle_for(&mismatch.model)).unwrap().archive;
    archive.crash = Some(failed_receipt(BackendState::Failed, Some(2), None, Some(2)));
    archive.crash.as_mut().unwrap().last_completed_frame = Some(0);
    archive.crash.as_mut().unwrap().last_persisted_frame = Some(0);
    let path = write_bundle(&dir, "crash-mismatch", &archive.encode().unwrap());
    let error = receive_path(&mut mismatch, &path, "support-load").unwrap_err();
    assert!(error.contains("crash/frame mismatch") && error.contains("received frame 1"), "{error}");
    no_external_results(&mut mismatch);
    let _ = fs::remove_dir_all(dir);
}

#[test]
fn product_apply_reuses_existing_transient_step_without_dropping_supports_or_settings() {
    let dir = test_dir("reuse-transient-step");
    let mut session = session_with_node();
    session.model.add_set("clamp", Set { nodes: vec![1], ..Set::default() }).unwrap();
    session.model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("clamp".into()) }).unwrap();
    let transient = fieldcraft_model::dynamics::Transient {
        adaptive: None,
        time_step: 1.0,
        increments: 1,
        output_every: 1,
        integration: fieldcraft_model::dynamics::Integration::AverageAcceleration,
        method: fieldcraft_model::dynamics::Method::Direct,
        damping: fieldcraft_model::harmonic::Damping::default(),
        load_amplitudes: Default::default(),
        initial_displacement: Default::default(),
        initial_velocity: Default::default(),
        base_acceleration: None,
        base_displacement: None,
    };
    let mut step = Step::new("Contact", Procedure::Transient(transient));
    step.boundary_conditions.insert("clamp".into());
    session.model.add_step(step, None).unwrap();
    let path = write_bundle(&dir, "reuse", &bundle_for(&session.model));
    receive_path(&mut session, &path, "support-load").unwrap();
    let reply = session
        .execute(
            "results.external.load_transfer.apply",
            &json!({
                "name":"support-load",
                "step":"Contact",
                "integration":{"type":"average_acceleration"},
                "method":{"type":"direct"},
                "damping":{"alpha":0.0,"beta":0.0,"loss_factor":0.0},
                "output_every":1
            }),
        )
        .unwrap();
    assert_eq!(reply["reused_existing_step"], true);
    assert_eq!(reply["boundary_conditions"], json!(["clamp"]));
    assert_eq!(reply["boundary_condition_count"], 1);
    assert!(reply["boundary_condition_warning"].is_null());
    let applied = session.model.step("Contact").unwrap();
    assert_eq!(applied.boundary_conditions, ["clamp".to_string()].into_iter().collect());
    assert!(matches!(applied.procedure, Procedure::Transient(_)));
    let Procedure::Transient(transient) = &applied.procedure else { unreachable!() };
    assert_eq!(transient.integration, fieldcraft_model::dynamics::Integration::AverageAcceleration);
    assert_eq!(transient.method, fieldcraft_model::dynamics::Method::Direct);
    assert_eq!(transient.output_every, 1);
    assert_eq!(applied.loads.len(), 1);
    let _ = fs::remove_dir_all(dir);
}

#[test]
fn product_apply_new_step_accepts_named_boundary_conditions() {
    let dir = test_dir("new-step-boundary-condition");
    let mut session = session_with_node();
    session.model.add_set("clamp", Set { nodes: vec![1], ..Set::default() }).unwrap();
    session.model.add_boundary_condition("clamp", BoundaryCondition::Fixed { target: Target::Set("clamp".into()) }).unwrap();
    let path = write_bundle(&dir, "new", &bundle_for(&session.model));
    receive_path(&mut session, &path, "support-load").unwrap();
    let reply = session
        .execute("results.external.load_transfer.apply", &json!({"name":"support-load","step":"new-transient","boundary_conditions":["clamp"]}))
        .unwrap();
    assert_eq!(reply["reused_existing_step"], false);
    assert_eq!(reply["boundary_conditions"], json!(["clamp"]));
    assert_eq!(reply["boundary_condition_count"], 1);
    assert!(reply["boundary_condition_warning"].is_null());
    assert_eq!(session.model.step("new-transient").unwrap().boundary_conditions, ["clamp".to_string()].into_iter().collect());
    let _ = fs::remove_dir_all(dir);
}

#[test]
fn product_apply_uses_total_duration_for_accumulated_time_axis() {
    let dir = test_dir("total-duration-time-step");
    let mut session = session_with_node();
    let mut archive = LoadTransferArchive::decode(&bundle_for(&session.model)).unwrap().archive;
    let mut extra = archive.frames[1].clone();
    extra.frame_index = 2;
    archive.frames[0].time_s = 0.0;
    archive.frames[0].source_dt_s = 0.0;
    archive.frames[1].time_s = 0.1000000000005;
    archive.frames[1].source_dt_s = 0.1000000000005;
    extra.time_s = 0.2;
    extra.source_dt_s = 0.0999999999995;
    archive.frames.push(extra);
    let path = write_bundle(&dir, "accumulated", &archive.encode().unwrap());
    receive_path(&mut session, &path, "support-load").unwrap();
    let reply = session.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap();
    assert_eq!(reply["time_step_source"], "total_duration/(frames-1)");
    assert_eq!(reply["time_grid_tolerance_s"], 1e-12);
    assert!((reply["transient_settings"]["time_step"].as_f64().unwrap() - 0.1).abs() < 1e-15);
    assert_eq!(reply["transient_settings"]["increments"], 2);
    let _ = fs::remove_dir_all(dir);
}

fn test_dir(label: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("fieldcraft-load-transfer-{label}-{}", std::process::id()));
    fs::create_dir_all(&dir).unwrap();
    dir
}

fn write_bundle(dir: &Path, label: &str, bundle: &ArchiveBundle) -> PathBuf {
    let path = dir.join(format!("{label}.bundle.json"));
    fs::write(&path, serde_json::to_vec(bundle).unwrap()).unwrap();
    path
}

fn session_with_node() -> Session {
    let mut session = Session::default();
    session.model.set_node(1, [0.0, 0.0, 0.0]).unwrap();
    session
}

fn receive_path(session: &mut Session, path: &Path, name: &str) -> Result<Value, String> {
    session.execute("results.external.load_transfer.receive", &json!({"path":path,"name":name})).map_err(|error| error.to_string())
}

fn no_external_results(session: &mut Session) {
    assert_eq!(session.execute("results.external.list", &json!({})).unwrap(), json!([]));
}

#[test]
fn product_receive_rejects_identity_checksum_applicability_mapping_order_and_trailing_tamper() {
    let cases = ["name", "checksum", "applicability", "nodal", "order", "trailing"];
    for case in cases {
        let dir = test_dir(&format!("reject-{case}"));
        let mut session = session_with_node();
        let mut bundle = bundle_for(&session.model);
        let name = if case == "name" { "different-name" } else { "support-load" };
        match case {
            "checksum" => bundle.point_binary[0] ^= 1,
            "applicability" => {
                let mut manifest: Value = serde_json::from_slice(&bundle.manifest_json).unwrap();
                manifest["applicability"]["geometry_status"] = json!("inapplicable");
                bundle.manifest_json = serde_json::to_vec(&manifest).unwrap();
            }
            "nodal" => {
                bundle.nodal_binary[0] ^= 1;
                let mut manifest: Value = serde_json::from_slice(&bundle.manifest_json).unwrap();
                manifest["nodal_sha256"] = json!(fieldcraft_io::sha256_hex(&bundle.nodal_binary));
                bundle.manifest_json = serde_json::to_vec(&manifest).unwrap();
            }
            "order" => {
                let mut manifest: Value = serde_json::from_slice(&bundle.manifest_json).unwrap();
                manifest["frames"][1]["frame_index"] = json!(0);
                bundle.manifest_json = serde_json::to_vec(&manifest).unwrap();
            }
            "trailing" => {
                bundle.point_binary.push(0);
                let mut manifest: Value = serde_json::from_slice(&bundle.manifest_json).unwrap();
                manifest["point_bytes"] = json!(bundle.point_binary.len());
                manifest["point_sha256"] = json!(fieldcraft_io::sha256_hex(&bundle.point_binary));
                bundle.manifest_json = serde_json::to_vec(&manifest).unwrap();
            }
            _ => {}
        }
        let path = write_bundle(&dir, case, &bundle);
        let error = receive_path(&mut session, &path, name).unwrap_err();
        let expected = match case {
            "name" => "archive_key",
            "checksum" => "checksum",
            "applicability" => "applicability",
            "nodal" => "nodal force",
            "order" => "frame range",
            "trailing" => "trailing binary",
            _ => unreachable!(),
        };
        assert!(error.contains(expected), "{case}: {error}");
        no_external_results(&mut session);
        let _ = fs::remove_dir_all(dir);
    }
}

#[test]
fn product_receive_rejects_producer_claimed_qualification_and_bad_program_identity() {
    let dir = test_dir("reject-qualification-and-program-identity");

    let mut self_claimed = LoadTransferArchive::decode(&bundle_for(&session_with_node().model)).unwrap().archive;
    self_claimed.qualification = RuntimeQualification::Verified { source: "producer".into() };
    self_claimed.applicability =
        evaluate_applicability(&self_claimed.activation.applicability_input().unwrap(), &self_claimed.qualification).unwrap();
    let self_claimed_path = write_bundle(&dir, "self-claimed", &self_claimed.encode().unwrap());
    let mut self_claimed_session = session_with_node();
    let error = receive_path(&mut self_claimed_session, &self_claimed_path, "support-load").unwrap_err();
    assert!(error.contains("producer-claimed runtime qualification"), "{error}");
    no_external_results(&mut self_claimed_session);

    let mut invalid_identity = LoadTransferArchive::decode(&bundle_for(&session_with_node().model)).unwrap().archive;
    invalid_identity.identity.producer_sha256 = "not-a-sha256".into();
    let invalid_identity_path = write_bundle(&dir, "invalid-program-identity", &invalid_identity.encode().unwrap());
    let mut invalid_identity_session = session_with_node();
    let error = receive_path(&mut invalid_identity_session, &invalid_identity_path, "support-load").unwrap_err();
    assert!(error.contains("program identity"), "{error}");
    no_external_results(&mut invalid_identity_session);
    let _ = fs::remove_dir_all(dir);
}

#[test]
fn product_receive_rejects_model_mismatch_and_conservation_before_installing() {
    let dir = test_dir("reject-model-conservation");
    let source = session_with_node();
    let path = write_bundle(&dir, "model", &bundle_for(&source.model));
    let mut different_model = Session::default();
    different_model.model.set_node(2, [0.0, 0.0, 0.0]).unwrap();
    let error = receive_path(&mut different_model, &path, "support-load").unwrap_err();
    assert!(error.contains("identity"), "{error}");
    no_external_results(&mut different_model);

    let mut conservation_bundle = bundle_for(&source.model);
    let mut manifest: Value = serde_json::from_slice(&conservation_bundle.manifest_json).unwrap();
    manifest["mapping"]["nodes"][0]["position_m"] = json!([0.0, 1.0, 0.0]);
    conservation_bundle.manifest_json = serde_json::to_vec(&manifest).unwrap();
    let conservation_path = write_bundle(&dir, "conservation", &conservation_bundle);
    let mut conservation_session = session_with_node();
    let error = receive_path(&mut conservation_session, &conservation_path, "support-load").unwrap_err();
    assert!(error.contains("frame 1") && error.contains("item=moment"), "{error}");
    no_external_results(&mut conservation_session);
    let _ = fs::remove_dir_all(dir);
}

#[test]
fn product_apply_rejects_inapplicable_existing_step_nonuniform_missing_node_and_short_history() {
    let dir = test_dir("reject-apply");

    let mut inapplicable = session_with_node();
    let mut inapplicable_archive = LoadTransferArchive::decode(&bundle_for(&inapplicable.model)).unwrap().archive;
    inapplicable_archive.activation.frames[0].feedback_active.as_mut().unwrap()[0].weight = 0.0;
    inapplicable_archive.applicability =
        evaluate_applicability(&inapplicable_archive.activation.applicability_input().unwrap(), &inapplicable_archive.qualification).unwrap();
    let inapplicable_path = write_bundle(&dir, "inapplicable", &inapplicable_archive.encode().unwrap());
    receive_path(&mut inapplicable, &inapplicable_path, "support-load").unwrap();
    let error = inapplicable.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap_err().to_string();
    assert!(error.contains("inapplicable"), "{error}");

    let mut existing = session_with_node();
    let existing_path = write_bundle(&dir, "existing", &bundle_for(&existing.model));
    receive_path(&mut existing, &existing_path, "support-load").unwrap();
    existing.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap();
    let error = existing.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap_err().to_string();
    assert!(error.contains("already exists"), "{error}");

    let mut nonuniform = session_with_node();
    let mut nonuniform_archive = LoadTransferArchive::decode(&bundle_for(&nonuniform.model)).unwrap().archive;
    let mut extra = nonuniform_archive.frames[1].clone();
    extra.frame_index = 2;
    extra.time_s = 3.0;
    extra.source_dt_s = 2.0;
    nonuniform_archive.frames.push(extra);
    let nonuniform_path = write_bundle(&dir, "nonuniform", &nonuniform_archive.encode().unwrap());
    receive_path(&mut nonuniform, &nonuniform_path, "support-load").unwrap();
    let error = nonuniform.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap_err().to_string();
    assert!(error.contains("uniform"), "{error}");

    let mut missing_node = session_with_node();
    let mut missing_node_archive = LoadTransferArchive::decode(&bundle_for(&missing_node.model)).unwrap().archive;
    missing_node_archive.mapping.nodes[0].node_id = 999;
    missing_node_archive.mapping.rows[0].contributions[0].node_id = 999;
    let missing_path = write_bundle(&dir, "missing-node", &missing_node_archive.encode().unwrap());
    receive_path(&mut missing_node, &missing_path, "support-load").unwrap();
    let error = missing_node.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap_err().to_string();
    assert!(error.contains("not in the open model"), "{error}");

    let mut short = session_with_node();
    let mut short_archive = LoadTransferArchive::decode(&bundle_for(&short.model)).unwrap().archive;
    short_archive.frames.pop();
    let short_path = write_bundle(&dir, "short", &short_archive.encode().unwrap());
    receive_path(&mut short, &short_path, "support-load").unwrap();
    let error = short.execute("results.external.load_transfer.apply", &json!({"name":"support-load"})).unwrap_err().to_string();
    assert!(error.contains("needs 2"), "{error}");
    let _ = fs::remove_dir_all(dir);
}

#[test]
fn product_entry_rejects_missing_directory_and_overlong_path_parameters() {
    let dir = test_dir("reject-params");
    let mut session = session_with_node();
    let error = session.execute("results.external.load_transfer.receive", &json!({})).unwrap_err().to_string();
    assert!(!error.is_empty());
    let error = session.execute("results.external.load_transfer.receive", &json!({"path":dir,"name":"support-load"})).unwrap_err().to_string();
    assert!(!error.is_empty());
    let overlong = dir.join("x".repeat(5000));
    let error = session.execute("results.external.load_transfer.receive", &json!({"path":overlong,"name":"support-load"})).unwrap_err().to_string();
    assert!(!error.is_empty());
    let error = session
        .execute("results.external.load_transfer.receive", &json!({"path":dir.join("missing"),"name":"a".repeat(5000)}))
        .unwrap_err()
        .to_string();
    assert!(!error.is_empty());
    let _ = fs::remove_dir_all(dir);
}

#[test]
#[ignore = "real case22 evidence is run explicitly with the frozen source fixture"]
fn product_entry_accepts_real_case22_failed_prefix_and_records_raw_reply() {
    let fixture_path =
        std::env::var("LOAD_TRANSFER_CASE22_FAILURE_FIXTURE").map(PathBuf::from).expect("LOAD_TRANSFER_CASE22_FAILURE_FIXTURE is required");
    let evidence_dir =
        std::env::var("LOAD_TRANSFER_CASE22_FAILURE_EVIDENCE").map(PathBuf::from).expect("LOAD_TRANSFER_CASE22_FAILURE_EVIDENCE is required");
    fs::create_dir_all(&evidence_dir).unwrap();
    let fixture: Value = serde_json::from_slice(&fs::read(&fixture_path).unwrap()).unwrap();
    let source_frames = fixture["frames"].as_array().expect("source frames");
    assert_eq!(source_frames.len(), 11605);
    assert_eq!(fixture["first_failed_frame"]["frame"], 11606);

    let mut session = session_with_node();
    let snapshot = identity::snapshot(&session.model, &fieldcraft_io::Geometry::default(), &fieldcraft_io::Run::default()).unwrap();
    let activation_frames = source_frames
        .iter()
        .map(|frame| {
            let peak = frame["force_n"].as_array().unwrap().iter().map(|value| value.as_f64().unwrap().abs()).sum::<f64>();
            ActivationFrame {
                frame_index: frame["frame_index"].as_u64().unwrap(),
                source_active: vec![ActivationPoint { contact_id: "case22-contact".into(), weight: 1.0, gap_m: 0.0 }],
                feedback_active: Some(vec![ActivationPoint { contact_id: "case22-contact".into(), weight: 1.0, gap_m: 0.0 }]),
                source_peak_force_n: Some(peak),
                feedback_peak_force_n: Some(peak),
            }
        })
        .collect();
    let activation = ActivationData {
        frames: activation_frames,
        activation_length_m: 1.0,
        required_fields: true,
        structural_feedback: true,
        force_precision: ForcePrecisionLimit {
            kappa_pa: UnknownOr::unknown("case22 producer did not export per-frame effective kappa"),
            truncated: UnknownOr::unknown("case22 producer did not export per-frame truncation"),
        },
    };
    let qualification = RuntimeQualification::Unverified { reason: "case22 failed run is not a runtime qualification".into() };
    let applicability = evaluate_applicability(&activation.applicability_input().unwrap(), &qualification).unwrap();
    let frames = source_frames
        .iter()
        .map(|frame| LoadFrame {
            frame_index: frame["frame_index"].as_u64().unwrap(),
            time_s: frame["time_s"].as_f64().unwrap(),
            source_dt_s: if frame["frame_index"].as_u64().unwrap() == 1 { 0.0 } else { 0.0002 },
            points: vec![PointForce {
                point_id: "case22-foot-force-0".into(),
                position_m: [0.0, 0.0, 0.0],
                force_n: [frame["force_n"][0].as_f64().unwrap(), frame["force_n"][1].as_f64().unwrap(), frame["force_n"][2].as_f64().unwrap()],
            }],
            facts: FrameFacts {
                completed: true,
                converged: true,
                hit_newton_limit: false,
                hit_line_search_limit: false,
                kappa_pa: UnknownOr::unknown("case22 producer did not export per-frame effective kappa"),
                truncated: UnknownOr::unknown("case22 producer did not export per-frame truncation"),
            },
        })
        .collect();
    let archive = LoadTransferArchive {
        identity: LoadIdentity {
            archive_key: "case22-real-failed-prefix-11605".into(),
            run_id: "case22-full-12400-20261110".into(),
            epoch: 0,
            worker_generation: "native-pid-3111910".into(),
            model_revision: snapshot.model_revision,
            mesh_digest: snapshot.reference_mesh_digest,
            coordinate_frame: "world".into(),
            units: "SI".into(),
            producer_sha256: "385cf03b0acb729ad82b5710b85f463ade7be4e02e8143291fca4f849ecdea5e".into(),
            adapter_sha256: "74d14f14a73da4deb112982a644e98bc664489d717db9d8222092e6086e62a74".into(),
        },
        mapping: MappingSpec {
            origin_m: [0.0, 0.0, 0.0],
            nodes: vec![MappingNode { node_id: 1, position_m: [0.0, 0.0, 0.0] }],
            rows: vec![PointMapping {
                point_id: "case22-foot-force-0".into(),
                contributions: vec![NodeContribution { node_id: 1, h_block: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] }],
            }],
            mapping_sha256: "c".repeat(64),
        },
        frames,
        activation,
        qualification,
        applicability,
        crash: Some(CrashReceipt {
            state: BackendState::Failed,
            exit_code: Some(2),
            signal: None,
            first_error: Some(FirstError {
                frame_index: Some(11606),
                phase: "native_step".into(),
                kind: "native_step_not_converged".into(),
                message: "case22 frame 11606 hit Newton limit; native exit 2".into(),
            }),
            last_completed_frame: Some(11605),
            last_persisted_frame: Some(11605),
            requested_last_frame: Some(12400),
            stdout_sha256: "891bb6482690ef1c6c53dccff335ef20f270c892ce3d1ad704ccd2e7b745c66f".into(),
            stderr_sha256: "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855".into(),
        }),
    };
    let bundle_path = evidence_dir.join("case22-real-failed-prefix-11605.bundle.json");
    fs::write(&bundle_path, serde_json::to_vec(&archive.encode().unwrap()).unwrap()).unwrap();
    let project_path = evidence_dir.join("case22-real-failed-prefix-11605.fieldcraft");
    let reply = session
        .execute(
            "results.external.load_transfer.receive",
            &json!({"path":bundle_path,"name":"case22-real-failed-prefix-11605","project":project_path}),
        )
        .unwrap();
    assert_eq!(reply["backend_status"]["state"], "failed");
    assert_eq!(reply["backend_status"]["exit_code"], 2);
    assert_eq!(reply["backend_status"]["last_completed_frame"], 11605);
    assert_eq!(reply["backend_status"]["coverage_through_frame"], 11605);
    assert_eq!(reply["apply_allowed"], false);
    let first = fs::read(&project_path).unwrap();
    let mut reopened = Session::default();
    reopened.execute("file.open", &json!({"path":project_path})).unwrap();
    let list = reopened.execute("results.external.list", &json!({})).unwrap();
    reopened.execute("file.save", &json!({"path":project_path,"results":true})).unwrap();
    let second = fs::read(&project_path).unwrap();
    assert_eq!(first, second);
    assert_eq!(list[0]["backend_status"]["coverage_through_frame"], 11605);
    let apply_error =
        reopened.execute("results.external.load_transfer.apply", &json!({"name":"case22-real-failed-prefix-11605"})).unwrap_err().to_string();
    assert!(apply_error.contains("refused backend status"), "{apply_error}");
    fs::write(
        evidence_dir.join("case22-real-failed-entry-reply.json"),
        serde_json::to_vec_pretty(&json!({
            "source_fixture": fixture_path,
            "source_case": fixture["source_case"],
            "first_failed_frame": fixture["first_failed_frame"],
            "native_exit": fixture["native_exit"],
            "receive_reply": reply,
            "reopen_list_reply": list,
            "project_reopen_resave_byte_identical": first == second,
            "apply_rejection": apply_error,
            "qualified_prefix_frame_count": source_frames.len(),
        }))
        .unwrap(),
    )
    .unwrap();
}
