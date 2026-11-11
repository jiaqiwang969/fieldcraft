use fieldcraft_engine::Session;
use fieldcraft_engine::external::load_transfer::*;
use fieldcraft_io::identity;
use serde_json::{Value, json};
use std::fs;
use std::path::PathBuf;

fn v3(value: &Value) -> [f64; 3] {
    let values = value.as_array().unwrap();
    [values[0].as_f64().unwrap(), values[1].as_f64().unwrap(), values[2].as_f64().unwrap()]
}

fn archive_from_input(input: &Value, model: &fieldcraft_model::Model, name: &str, activation: ActivationData) -> LoadTransferArchive {
    let snapshot = identity::snapshot(model, &fieldcraft_io::Geometry::default(), &fieldcraft_io::Run::default()).unwrap();
    let labels: Vec<u64> = input["target_node_labels"].as_array().unwrap().iter().map(|value| value.as_u64().unwrap()).collect();
    let target_xyz: Vec<[f64; 3]> = input["target_xyz_m"].as_array().unwrap().iter().map(v3).collect();
    let frames: Vec<LoadFrame> = input["time_s"]
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
        .map(|(frame_index, time)| {
            let force_row = input["force_n"][frame_index].as_array().unwrap();
            LoadFrame {
                frame_index: frame_index as u64,
                time_s: time.as_f64().unwrap(),
                source_dt_s: if frame_index == 0 { 0.0 } else { time.as_f64().unwrap() - input["time_s"][frame_index - 1].as_f64().unwrap() },
                points: force_row
                    .iter()
                    .enumerate()
                    .map(|(point_index, force)| PointForce {
                        point_id: format!("p{point_index}"),
                        position_m: v3(&input["source_xyz_m"][point_index]),
                        force_n: v3(force),
                    })
                    .collect(),
                facts: FrameFacts {
                    completed: true,
                    converged: true,
                    hit_newton_limit: false,
                    hit_line_search_limit: false,
                    kappa_pa: UnknownOr::unknown("libuipc 0.0.29 does not expose per-frame effective kappa"),
                    truncated: UnknownOr::unknown("per-frame truncation is not exposed"),
                },
            }
        })
        .collect();
    let mapping = MappingSpec {
        origin_m: [0.0, 0.0, 0.0],
        nodes: labels.iter().zip(target_xyz).map(|(node_id, position_m)| MappingNode { node_id: *node_id, position_m }).collect(),
        rows: (0..9)
            .map(|point_index| PointMapping {
                point_id: format!("p{point_index}"),
                contributions: vec![NodeContribution { node_id: labels[point_index], h_block: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] }],
            })
            .collect(),
        mapping_sha256: "0".repeat(64),
    };
    let qualification = RuntimeQualification::Unverified { reason: "runtime qualification is pending independent receiver verification".into() };
    let applicability = evaluate_applicability(&activation.applicability_input().unwrap(), &qualification).unwrap();
    LoadTransferArchive {
        identity: LoadIdentity {
            archive_key: name.to_string(),
            run_id: "S04-dynamic-final-h31".into(),
            epoch: 1,
            worker_generation: "frozen-source".into(),
            model_revision: snapshot.model_revision,
            mesh_digest: snapshot.reference_mesh_digest,
            coordinate_frame: "world".into(),
            units: "SI".into(),
        },
        mapping,
        frames,
        activation,
        qualification,
        applicability,
        crash: None,
    }
}

#[test]
fn real_frozen_point_forces_use_receive_map_conservation_apply_and_impact_gate() {
    let input_path = std::env::var("LOAD_TRANSFER_REAL_INPUT")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..").join("记录/第五阶段/回归/补齐候选/真实载荷输入.json"));
    let input_bytes = fs::read(&input_path).unwrap();
    let input: Value = serde_json::from_slice(&input_bytes).unwrap();
    let mut session = Session::default();
    let labels: Vec<u32> = input["target_node_labels"].as_array().unwrap().iter().map(|value| value.as_u64().unwrap() as u32).collect();
    let target_xyz = input["target_xyz_m"].as_array().unwrap();
    for (label, xyz) in labels.iter().zip(target_xyz) {
        session.model.set_node(*label, v3(xyz)).unwrap();
    }
    let actual_activation: ActivationData = serde_json::from_value(input["activation"].clone()).unwrap();
    let mut unknown_activation = actual_activation.clone();
    unknown_activation.structural_feedback = false;
    for frame in &mut unknown_activation.frames {
        frame.feedback_active = None;
    }
    let load_archive = archive_from_input(&input, &session.model, "real-load", unknown_activation);
    let impact_archive = archive_from_input(&input, &session.model, "real-impact", actual_activation);
    let load_path = input_path.with_file_name("real-load.bundle.json");
    let impact_path = input_path.with_file_name("real-impact.bundle.json");
    fs::write(&load_path, serde_json::to_vec(&load_archive.encode().unwrap()).unwrap()).unwrap();
    fs::write(&impact_path, serde_json::to_vec(&impact_archive.encode().unwrap()).unwrap()).unwrap();
    let project_path = input_path.with_file_name("real-load.fieldcraft");
    let load_reply =
        session.execute("results.external.load_transfer.receive", &json!({"path":load_path,"name":"real-load","project":project_path})).unwrap();
    let impact_reply = session.execute("results.external.load_transfer.receive", &json!({"path":impact_path,"name":"real-impact"})).unwrap();
    assert_eq!(load_reply["conservation"]["all_frames_passed"], true);
    assert_eq!(load_reply["applicability"]["geometry_status"], "unknown");
    assert_eq!(load_reply["runtime_qualification"]["state"], "unverified");
    assert_eq!(impact_reply["applicability"]["geometry_status"], "inapplicable");
    assert_eq!(impact_reply["applicability"]["first_failure"]["metric"], "active_area_difference");
    let first = fs::read(&project_path).unwrap();
    let mut reopened = Session::default();
    reopened.execute("file.open", &json!({"path":project_path})).unwrap();
    reopened.execute("file.save", &json!({"path":project_path,"results":true})).unwrap();
    let second = fs::read(&project_path).unwrap();
    assert_eq!(first, second);
    let apply_reply = reopened.execute("results.external.load_transfer.apply", &json!({"name":"real-load","step":"real-load-transient"})).unwrap();
    assert_eq!(apply_reply["ready_for"], "solve.transient");
    assert_eq!(apply_reply["conclusion_allowed"], false);
    assert_eq!(apply_reply["frames"], 1025);
    let report_path = input_path.with_file_name("真实载荷全链-结果.json");
    fs::write(
        &report_path,
        serde_json::to_vec_pretty(&json!({
            "source": input["source"],
            "input_sha256": fieldcraft_io::sha256_hex(&input_bytes),
            "receive": load_reply,
            "impact_receive": impact_reply,
            "project_reopen_resave_byte_identical": true,
            "apply": apply_reply,
            "two_layer_regression": "see 记录/第五阶段/回归/当前程序重算/two-layer-current-vs-history.json and independent-3d-regression.json",
            "runtime_boundary": "unknown qualification is retained; structural path is diagnostic and conclusion_allowed=false",
        }))
        .unwrap(),
    )
    .unwrap();
    let _ = fs::remove_file(load_path);
    let _ = fs::remove_file(impact_path);
    let _ = fs::remove_file(project_path);
}
