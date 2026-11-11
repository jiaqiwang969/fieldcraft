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
    let qualification = RuntimeQualification::Verified { source: "entry-test".into() };
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
    assert_eq!(reply["runtime_qualification"]["state"], "verified");
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
    assert_eq!(applied["transient_settings"]["output_every"], 1);
    assert_eq!(applied["transient_settings"]["integration"]["type"], "average_acceleration");
    assert!(applied["boundary_condition_warning"].is_string());
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
                "output_every":1,
                "solve_mass":"consistent"
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
