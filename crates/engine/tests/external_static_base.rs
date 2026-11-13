//! Product-owned kind-13 receive tests.  The fixture is deliberately small;
//! the real measured evidence is exercised by the external runner and the delivery
//! receipt, while this test covers the receive gate and persistence boundary.

#[path = "common/manifest.rs"]
mod manifest;
use fieldcraft_engine::Session;
use fieldcraft_io::{
    Geometry, Run,
    external_results::{
        ArchivedStaticBase, ConstraintMapping, ContactState, DenseMatrix, ExternalStaticBase, HoldWindow, MassField, StaticBaseState, StiffnessField,
    },
    identity::{Binding, EntityMap, REQUIRED_ARTIFACT_ROLES},
    sha256_hex,
};
use fieldcraft_model::Model;
use serde_json::{Value, json};
use std::{collections::BTreeMap, fs, path::PathBuf};

fn receiver_digest() -> String {
    sha256_hex(&fs::read(std::env::current_exe().unwrap()).unwrap())
}

fn fixture() -> (Model, ArchivedStaticBase, Value) {
    let model = Model::default();
    let map = EntityMap { nodes: vec![], elements: vec![] };
    let mut artifacts = BTreeMap::new();
    artifacts.insert("producer".into(), sha256_hex(b"native-backend.so"));
    artifacts.insert("adapter".into(), sha256_hex(b"driver-and-packer"));
    artifacts.insert("adapter_driver".into(), sha256_hex(b"driver.py"));
    artifacts.insert("adapter_packer".into(), sha256_hex(b"packer.py"));
    artifacts.insert("receiver".into(), receiver_digest());
    for role in REQUIRED_ARTIFACT_ROLES {
        assert!(artifacts.contains_key(role));
    }
    let binding = Binding::freeze(&model, &Geometry::default(), &Run::default(), &map, artifacts, fieldcraft_io::identity::BACKEND_VERSION).unwrap();
    let eye = DenseMatrix::square(vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], 3).unwrap();
    let base = ExternalStaticBase {
        name: "external_static_base".into(),
        state: StaticBaseState {
            reference_coordinates: vec![0.0, 0.0, 0.0],
            coordinates: vec![0.001, 0.002, 0.003],
            velocities: vec![1e-9; 3],
            units: "SI".into(),
            hold: HoldWindow {
                no_reset_steps: 10,
                converged: true,
                solver_cap_hit: false,
                free_residual_norm: 1e-9,
                max_velocity: 1e-9,
                single_step_drift: 1e-12,
                inertia_internal_difference: 1e-9,
                support_error: 1e-13,
            },
        },
        contact: ContactState {
            policy: "closed_bonded".into(),
            policy_digest: sha256_hex(b"closed_bonded"),
            active: vec![true],
            gap_normal: vec![0.0],
            tangential_increment: vec![0.0],
            delta_q_max: None,
            slip_ratio: None,
            friction_coefficient: None,
            normal_force: vec![],
            tangential_force: vec![],
            friction_utilization: None,
            state_changes: 0,
        },
        stiffness: StiffnessField {
            matrix: DenseMatrix::square(vec![4.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 16.0], 3).unwrap(),
            source_backend: "external-reconstruction".into(),
            probe_error: 0.0,
            symmetry_error: 0.0,
            projected: false,
        },
        mass: MassField {
            matrix: eye.clone(),
            kind: "consistent".into(),
            source_backend: "external-reconstruction".into(),
            declared_total: 1.0,
            symmetry_error: 0.0,
        },
        constraints: ConstraintMapping {
            full_dofs: 3,
            reduced_dofs: 3,
            q0: vec![0.0; 3],
            full_coordinates: vec![0.001, 0.002, 0.003],
            reduced_coordinates: vec![0.001, 0.002, 0.003],
            t: eye.clone(),
            reduced_stiffness: DenseMatrix::square(vec![4.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 16.0], 3).unwrap(),
            reduced_mass: eye,
            constraint_residual: 0.0,
            projection_error: 0.0,
        },
        source_backend: "external-reconstruction".into(),
        runtime_qualification: "verified".into(),
        frequency_response: "not_run".into(),
        modal: None,
    };
    let archive = ArchivedStaticBase::from_payload(
        "external-static-base-test".into(),
        binding.clone(),
        map,
        base.clone(),
        br#"{"solve":"succeeded"}"#.to_vec(),
        br#"{"runtime_qualification":"verified"}"#.to_vec(),
        br#"{"source":"engine-test"}"#.to_vec(),
    )
    .unwrap();
    let profile = json!({
        "schema":"fieldcraft-external-static-base/receive-profile/1",
        "archive_key":"external-static-base-test",
        "binding":binding,
        "reference_shapes":[[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],
        "mode_count":3
    });
    (model, archive, profile)
}

fn paths(tag: &str) -> (PathBuf, PathBuf) {
    let root = std::env::temp_dir().join(format!("fieldcraft-static-base-{tag}-{}", std::process::id()));
    let _ = fs::remove_dir_all(&root);
    fs::create_dir_all(&root).unwrap();
    (root.join("package.json"), root.join("profile.json"))
}

fn write_fixture(tag: &str) -> (Model, PathBuf, PathBuf) {
    let (model, archive, profile) = fixture();
    let (package, profile_path) = paths(tag);
    fs::write(&package, serde_json::to_vec_pretty(&archive).unwrap()).unwrap();
    fs::write(&profile_path, serde_json::to_vec_pretty(&profile).unwrap()).unwrap();
    (model, package, profile_path)
}

#[test]
fn static_base_receive_recomputes_unverified_and_roundtrips_byte_identically() {
    let (model, package, profile) = write_fixture("roundtrip");
    let root = package.parent().unwrap();
    let first = root.join("first.fieldcraft");
    let second = root.join("second.fieldcraft");
    let mut session = Session::default();
    session.model = model;
    let reply = session
        .execute("results.external.static_base.receive", &json!({"path":package,"profile":profile,"name":"external-static-base-test"}))
        .unwrap();
    assert_eq!(reply["runtime_qualification"], "unverified");
    assert!(reply["modal_min_mac"].as_f64().unwrap() >= 0.9);
    assert!(reply["modal_min_pair_margin"].as_f64().unwrap() >= 0.05);
    let modes = reply["modal_package"]["modes"].as_array().unwrap();
    assert_eq!(modes.len(), 3);
    for (index, stiffness) in [4.0_f64, 9.0, 16.0].into_iter().enumerate() {
        let expected_frequency = stiffness.sqrt() / (2.0 * std::f64::consts::PI);
        assert_eq!(modes[index]["mode"], json!(index + 1));
        assert!((modes[index]["frequency_hz"].as_f64().unwrap() - expected_frequency).abs() < 1e-12);
        assert!(modes[index]["residual"].as_f64().unwrap() <= 1e-12);
    }
    assert_eq!(reply["modal_package"]["shape_storage"], "stored_in_kind_13_record; omitted_from_command_response");
    session.execute("file.save_as", &json!({"path":first,"results":true})).unwrap();
    let first_bytes = fs::read(&first).unwrap();
    manifest::remove_derived_metadata(&first);
    let mut reopened = Session::default();
    reopened.execute("file.open", &json!({"path":first})).unwrap();
    let listed = reopened.execute("results.external.list", &json!({})).unwrap();
    assert_eq!(listed[0]["modal_package"], reply["modal_package"]);
    reopened.execute("file.save_as", &json!({"path":second,"results":true})).unwrap();
    assert_eq!(first_bytes, fs::read(&second).unwrap());
}

#[test]
fn static_base_receive_rejects_frozen_expectation_and_claim_errors() {
    let (_model, package, profile_path) = write_fixture("reject");
    let profile: Value = serde_json::from_slice(&fs::read(&profile_path).unwrap()).unwrap();
    let cases: Vec<(&str, Box<dyn Fn(&mut Value)>, &str)> = vec![
        ("binding", Box::new(|v: &mut Value| v["binding"]["artifacts"]["producer"] = json!(sha256_hex(b"other"))), "binding"),
        ("backend", Box::new(|v: &mut Value| v["binding"]["backend_version"] = json!("fieldcraft-backend/other")), "backend"),
        ("name", Box::new(|v: &mut Value| v["archive_key"] = json!("other-name")), "archive"),
        ("format", Box::new(|v: &mut Value| v["reference_shapes"] = json!([])), "mode count"),
        ("refs", Box::new(|v: &mut Value| v["mode_count"] = json!(4)), "mode count"),
    ];
    for (label, mutate, expected) in cases {
        let path = profile_path.with_file_name(format!("{label}.json"));
        let mut changed = profile.clone();
        mutate(&mut changed);
        fs::write(&path, serde_json::to_vec(&changed).unwrap()).unwrap();
        let mut session = Session::default();
        let error = session
            .execute("results.external.static_base.receive", &json!({"path":package,"profile":path,"name":"external-static-base-test"}))
            .unwrap_err()
            .to_string();
        assert!(error.to_lowercase().contains(expected), "{label}: {error}");
    }
    let (model, package, profile_path) = write_fixture("model");
    let mut session = Session::default();
    session.model = model;
    session.model.set_node(1, [1.0, 0.0, 0.0]).unwrap();
    let error = session
        .execute("results.external.static_base.receive", &json!({"path":package,"profile":profile_path,"name":"external-static-base-test"}))
        .unwrap_err()
        .to_string();
    assert!(
        error.to_lowercase().contains("model") || error.to_lowercase().contains("snapshot") || error.to_lowercase().contains("reference"),
        "model: {error}"
    );
}
