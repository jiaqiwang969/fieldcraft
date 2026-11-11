use fieldcraft_engine::Session;
use fieldcraft_engine::external::load_transfer::*;
use fieldcraft_io::identity;
use fieldcraft_model::Model;
use serde_json::json;
use std::fs;

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
    let _ = fs::remove_dir_all(dir);
}
