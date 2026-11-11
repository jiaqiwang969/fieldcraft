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

fn relative_error(actual: &[f64], reference: &[f64]) -> f64 {
    assert_eq!(actual.len(), reference.len());
    let numerator = actual.iter().zip(reference).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
    let denominator = reference.iter().map(|value| value.abs()).fold(0.0, f64::max);
    if denominator == 0.0 { numerator } else { numerator / denominator }
}

fn flat_sample_field(samples: &Value, field: &str) -> Vec<f64> {
    samples
        .as_array()
        .unwrap()
        .iter()
        .flat_map(|sample| sample[field].as_array().unwrap().iter().map(|value| value.as_f64().unwrap()).collect::<Vec<_>>())
        .collect()
}

fn history_relative_checks(product: &Value, history: &Value, tip_node: u64, far_node: u64) -> Value {
    let product_frames = product["frames"].as_array().unwrap();
    let history_frames = history["frames"].as_array().unwrap();
    assert_eq!(product_frames.len(), history_frames.len());
    let product_times: Vec<f64> = product_frames.iter().map(|frame| frame["time"].as_f64().unwrap()).collect();
    let history_times: Vec<f64> = history_frames.iter().map(|frame| frame["time"].as_f64().unwrap()).collect();
    let max_time_abs_s = product_times.iter().zip(&history_times).map(|(product, history)| (product - history).abs()).fold(0.0, f64::max);
    assert!(max_time_abs_s <= 1e-12, "real load path changed the structural time axis beyond rounding tolerance: {max_time_abs_s:.17e} s");
    let product_energy: Vec<f64> =
        product_frames.iter().map(|frame| frame["kinetic_energy"].as_f64().unwrap() + frame["strain_energy"].as_f64().unwrap()).collect();
    let history_energy: Vec<f64> =
        history_frames.iter().map(|frame| frame["kinetic_energy"].as_f64().unwrap() + frame["strain_energy"].as_f64().unwrap()).collect();
    let history_nodes: std::collections::BTreeMap<u64, &Value> =
        history["nodes"].as_array().unwrap().iter().map(|node| (node["node"].as_u64().unwrap(), node)).collect();
    let product_nodes: std::collections::BTreeMap<u64, &Value> =
        product["nodes"].as_array().unwrap().iter().map(|node| (node["node"].as_u64().unwrap(), node)).collect();
    assert_eq!(product_nodes.keys().collect::<Vec<_>>(), history_nodes.keys().collect::<Vec<_>>());
    let mut max_displacement = 0.0;
    let mut max_stress = 0.0;
    let mut max_displacement_node = None;
    let mut max_stress_node = None;
    for (node, history_node) in &history_nodes {
        let product_node = product_nodes[node];
        let displacement = relative_error(
            &flat_sample_field(&product_node["samples"], "displacement"),
            &flat_sample_field(&history_node["samples"], "displacement"),
        );
        let stress = relative_error(&flat_sample_field(&product_node["samples"], "stress"), &flat_sample_field(&history_node["samples"], "stress"));
        if displacement > max_displacement {
            max_displacement = displacement;
            max_displacement_node = Some(*node);
        }
        if stress > max_stress {
            max_stress = stress;
            max_stress_node = Some(*node);
        }
    }
    let checks = json!({
        "time_axis_max_abs_s": max_time_abs_s,
        "tip_displacement_all_components": relative_error(
            &flat_sample_field(&product_nodes[&tip_node]["samples"], "displacement"),
            &flat_sample_field(&history_nodes[&tip_node]["samples"], "displacement"),
        ),
        "far_stress_all_components": relative_error(
            &flat_sample_field(&product_nodes[&far_node]["samples"], "stress"),
            &flat_sample_field(&history_nodes[&far_node]["samples"], "stress"),
        ),
        "mechanical_energy": relative_error(&product_energy, &history_energy),
        "max_probe_node_displacement": max_displacement,
        "max_probe_node_stress": max_stress,
        "max_probe_node_displacement_node": max_displacement_node,
        "max_probe_node_stress_node": max_stress_node,
    });
    checks
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

/// The candidate regression must exercise the product entry points against the
/// frozen 621-node model.  Without the Dell paths this remains a fast, explicit
/// skip so the ordinary unit-test suite does not pretend to have run the heavy
/// solve locally.
#[test]
fn real_frozen_point_forces_run_the_product_receive_apply_solve_chain() {
    let Ok(model_path) = std::env::var("LOAD_TRANSFER_REAL_MODEL") else {
        eprintln!("skip real product chain: LOAD_TRANSFER_REAL_MODEL is not set");
        return;
    };
    let Ok(history_path) = std::env::var("LOAD_TRANSFER_REAL_HISTORY") else {
        eprintln!("skip real product chain: LOAD_TRANSFER_REAL_HISTORY is not set");
        return;
    };
    let Ok(output_dir) = std::env::var("LOAD_TRANSFER_REAL_OUTPUT") else {
        eprintln!("skip real product chain: LOAD_TRANSFER_REAL_OUTPUT is not set");
        return;
    };
    let input_path = std::env::var("LOAD_TRANSFER_REAL_INPUT")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..").join("记录/第五阶段/回归/补齐候选/真实载荷输入.json"));
    let output_dir = PathBuf::from(output_dir);
    fs::create_dir_all(&output_dir).unwrap();
    let input_bytes = fs::read(&input_path).unwrap();
    let input: Value = serde_json::from_slice(&input_bytes).unwrap();
    let history: Value = serde_json::from_slice(&fs::read(&history_path).unwrap()).unwrap();
    let probe_nodes: Vec<u64> = history["nodes"].as_array().unwrap().iter().map(|node| node["node"].as_u64().unwrap()).collect();
    let model_input_path = PathBuf::from(&history_path).with_file_name("model-input.json");
    let model_input: Value = serde_json::from_slice(&fs::read(&model_input_path).unwrap()).unwrap();
    let tip_node = std::env::var("LOAD_TRANSFER_REAL_TIP_NODE")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or_else(|| model_input["tip_node"].as_u64().unwrap());
    let far_node = std::env::var("LOAD_TRANSFER_REAL_FAR_NODE")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or_else(|| model_input["far_node"].as_u64().unwrap());

    let mut session = Session::default();
    let open_reply = session.execute("file.open", &json!({"path":model_path})).unwrap();
    // Retain the historical Contact step as the model input.  The product
    // path creates a distinct step with the same support and integration
    // settings, so the historical loads remain an untouched reference.
    let actual_activation: ActivationData = serde_json::from_value(input["activation"].clone()).unwrap();
    let mut unknown_activation = actual_activation.clone();
    unknown_activation.structural_feedback = false;
    for frame in &mut unknown_activation.frames {
        frame.feedback_active = None;
    }
    let archive = archive_from_input(&input, &session.model, "real-load", unknown_activation);
    let bundle_path = output_dir.join("real-load.bundle.json");
    fs::write(&bundle_path, serde_json::to_vec(&archive.encode().unwrap()).unwrap()).unwrap();
    let receive_reply = session.execute("results.external.load_transfer.receive", &json!({"path":bundle_path,"name":"real-load"})).unwrap();
    assert_eq!(receive_reply["conservation"]["all_frames_passed"], true);
    assert_eq!(receive_reply["mapped_frames"], 1025);
    let apply_reply = session
        .execute(
            "results.external.load_transfer.apply",
            &json!({
                "name":"real-load",
                "step":"real-load-transient",
                "boundary_conditions":["clamp"],
                "integration":{"type":"average_acceleration"},
                "method":{"type":"direct"},
                "damping":{"alpha":0.0,"beta":0.0,"loss_factor":0.0},
                "output_every":1,
                "solve_mass":"consistent"
            }),
        )
        .unwrap();
    assert_eq!(apply_reply["ready_for"], "solve.transient");
    assert_eq!(apply_reply["frames"], 1025);
    assert_eq!(apply_reply["conclusion_allowed"], false);
    assert_eq!(apply_reply["transient_settings"]["output_every"], 1);
    assert_eq!(apply_reply["transient_settings"]["solve_mass"], "consistent");

    let solve_reply =
        session.execute("solve.transient", &json!({"step":"real-load-transient","mass":"consistent","threads":8,"max_memory_gib":8.0})).unwrap();
    let product_probe = session.execute("solve.probe", &json!({"step":"real-load-transient","nodes":probe_nodes})).unwrap();
    // Persist the raw product reply before any comparison assertion so a
    // mismatch leaves the exact new-path result for the coordinator.
    fs::write(output_dir.join("product-probe.json"), serde_json::to_vec_pretty(&product_probe).unwrap()).unwrap();
    fs::write(
        output_dir.join("real-chain-replies.json"),
        serde_json::to_vec_pretty(&json!({
            "model":model_path,
            "history":history_path,
            "input":input_path,
            "open":open_reply,
            "receive":receive_reply,
            "apply":apply_reply,
            "solve":solve_reply,
            "product_probe_path":"product-probe.json"
        }))
        .unwrap(),
    )
    .unwrap();
    let comparison = history_relative_checks(&product_probe, &history, tip_node, far_node);
    let mut report = json!({
        "model":model_path,
        "history":history_path,
        "input":input_path,
        "open":open_reply,
        "receive":receive_reply,
        "apply":apply_reply,
        "solve":solve_reply,
        "product_probe":product_probe,
        "historical_comparison":comparison,
        "threshold":1e-9,
        "tip_node":tip_node,
        "far_node":far_node,
    });
    fs::write(output_dir.join("real-chain-replies.json"), serde_json::to_vec_pretty(&report).unwrap()).unwrap();
    let checks = &report["historical_comparison"];
    let passed =
        ["tip_displacement_all_components", "far_stress_all_components", "mechanical_energy", "max_probe_node_displacement", "max_probe_node_stress"]
            .into_iter()
            .all(|key| checks[key].as_f64().unwrap() <= 1e-9);
    report["passed"] = json!(passed);
    fs::write(output_dir.join("historical-comparison.json"), serde_json::to_vec_pretty(&report).unwrap()).unwrap();
    assert!(passed, "real product receive/apply/solve path differs from frozen history: {}", report["historical_comparison"]);
}
