use fieldcraft_engine::external::load_transfer::*;

fn identity() -> LoadIdentity {
    LoadIdentity {
        archive_key: "archive-24".to_string(),
        run_id: "run-1".to_string(),
        epoch: 3,
        worker_generation: "worker-a".to_string(),
        model_revision: "a".repeat(64),
        mesh_digest: "b".repeat(64),
        coordinate_frame: "world".to_string(),
        units: "SI".to_string(),
    }
}

fn facts() -> FrameFacts {
    FrameFacts {
        completed: true,
        converged: true,
        hit_newton_limit: false,
        hit_line_search_limit: false,
        kappa_pa: UnknownOr::unknown("per-frame source field unavailable"),
        truncated: UnknownOr::unknown("per-frame source field unavailable"),
    }
}

fn mapping() -> MappingSpec {
    MappingSpec {
        origin_m: [0.0, 0.0, 0.0],
        nodes: vec![MappingNode { node_id: 10, position_m: [0.0, 0.0, 0.0] }, MappingNode { node_id: 11, position_m: [1.0, 0.0, 0.0] }],
        rows: vec![
            PointMapping { point_id: "p0".to_string(), contributions: vec![NodeContribution { node_id: 10, h_block: identity_h() }] },
            PointMapping { point_id: "p1".to_string(), contributions: vec![NodeContribution { node_id: 11, h_block: identity_h() }] },
        ],
        mapping_sha256: "c".repeat(64),
    }
}

fn identity_h() -> [[f64; 3]; 3] {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}

fn frame(time_s: f64, force_scale: f64) -> LoadFrame {
    LoadFrame {
        frame_index: (time_s * 100.0) as u64,
        time_s,
        source_dt_s: 0.01,
        points: vec![
            PointForce { point_id: "p0".to_string(), position_m: [0.0, 0.0, 0.0], force_n: [1.0 * force_scale, 2.0 * force_scale, 0.0] },
            PointForce { point_id: "p1".to_string(), position_m: [1.0, 0.0, 0.0], force_n: [0.0, 0.0, -3.0 * force_scale] },
        ],
        facts: facts(),
    }
}

fn applicability_input(active_area_difference: f64) -> ApplicabilityInput {
    ApplicabilityInput {
        metrics: vec![ActivationMetric {
            frame_index: 7,
            significant: true,
            transition: true,
            active_area_difference,
            gap_difference: 0.02,
            component_changes: vec![ComponentChange { component_id: "component-a".to_string(), weight_fraction: 0.05 }],
        }],
        peak_sampling_loss: 0.01,
        required_fields: true,
        structural_feedback: true,
        force_precision: ForcePrecisionLimit { kappa_pa: UnknownOr::unknown("not exposed"), truncated: UnknownOr::unknown("not exposed") },
    }
}

#[test]
fn h_transpose_conserves_force_moment_and_virtual_work() {
    let spec = mapping();
    let source = frame(0.0, 1.0);
    let mapped = spec.map_frame(&source).unwrap();
    let report = check_conservation(&source, &mapped, &spec, &[[0.01, -0.02, 0.03], [-0.04, 0.05, -0.06]]).unwrap();
    assert!(report.passed, "{report:?}");
    assert!(report.force_residual_n.iter().all(|value| value.abs() <= ABSOLUTE_TOLERANCE));
    assert!(report.moment_residual_nm.iter().all(|value| value.abs() <= ABSOLUTE_TOLERANCE));
    assert!(report.virtual_work_residual_j.abs() <= ABSOLUTE_TOLERANCE);

    let later = frame(0.01, 2.0);
    let mapped_later = spec.map_frame(&later).unwrap();
    let impulse = check_time_integrals(&[source, later], &[mapped, mapped_later], spec.origin_m).unwrap();
    assert!(impulse.passed, "{impulse:?}");
}

#[test]
fn applicability_keeps_unknown_force_precision_parallel_to_geometry() {
    let report = evaluate_applicability(&applicability_input(0.05), &RuntimeQualification::Unverified { reason: "pending G25".to_string() }).unwrap();
    assert_eq!(report.geometry_status, ApplicabilityStatus::Applicable);
    assert_eq!(report.adoption_status, ApplicabilityStatus::Unknown);
    assert!(report.force_precision.kappa_pa.is_unknown());
    assert!(report.force_precision.truncated.is_unknown());

    let mut failed = applicability_input(0.050001);
    failed.metrics[0].gap_difference = 0.020001;
    let report = evaluate_applicability(&failed, &RuntimeQualification::Verified { source: "independent".to_string() }).unwrap();
    assert_eq!(report.geometry_status, ApplicabilityStatus::Inapplicable);
    assert_eq!(report.adoption_status, ApplicabilityStatus::Inapplicable);
    assert!(report.first_failure.is_some());
}

#[test]
fn impact_fixture_is_inapplicable_at_first_changed_frame() {
    let report = pre_research_impact_report().unwrap();
    assert_eq!(report.geometry_status, ApplicabilityStatus::Inapplicable);
    assert_eq!(report.adoption_status, ApplicabilityStatus::Inapplicable);
    let failure = report.first_failure.unwrap();
    assert_eq!(failure.frame_index, Some(0));
    assert_eq!(failure.metric, "active_area_difference");
}

#[test]
fn crash_receipt_preserves_exit_signal_first_error_and_prefix() {
    let receipt = CrashReceipt {
        state: BackendState::Failed,
        exit_code: Some(2),
        signal: None,
        first_error: Some(FirstError {
            frame_index: Some(11606),
            phase: "newton".to_string(),
            kind: "iteration_limit".to_string(),
            message: "Newton limit".to_string(),
        }),
        last_completed_frame: Some(11605),
        last_persisted_frame: Some(11605),
        requested_last_frame: Some(12399),
        stdout_sha256: "d".repeat(64),
        stderr_sha256: "e".repeat(64),
    };
    assert!(receipt.validate().is_ok());
    assert!(!receipt.solver_success());

    let signal = CrashReceipt {
        state: BackendState::Failed,
        exit_code: None,
        signal: Some(11),
        first_error: None,
        last_completed_frame: Some(3),
        last_persisted_frame: Some(3),
        requested_last_frame: Some(9),
        stdout_sha256: "f".repeat(64),
        stderr_sha256: "0".repeat(64),
    };
    assert!(signal.validate().is_ok());
    assert!(!signal.solver_success());
}

#[test]
fn archive_round_trip_is_reversible_and_tamper_evident() {
    let spec = mapping();
    let frames = vec![frame(0.0, 1.0), frame(0.01, 2.0)];
    let applicability =
        evaluate_applicability(&applicability_input(0.05), &RuntimeQualification::Verified { source: "independent".to_string() }).unwrap();
    let archive = LoadTransferArchive { identity: identity(), mapping: spec, frames, applicability, crash: None };
    let bundle = archive.encode().unwrap();
    let decoded = LoadTransferArchive::decode(&bundle).unwrap();
    assert_eq!(decoded.archive, archive);
    assert_eq!(decoded.mapped_frames.len(), 2);

    let mut damaged = bundle.clone();
    damaged.point_binary[0] ^= 1;
    assert!(matches!(LoadTransferArchive::decode(&damaged), Err(ArchiveError::Checksum)));
}
