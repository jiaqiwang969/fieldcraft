use fieldcraft_io::{
    Geometry, Run, View,
    external_results::{
        ArchivedStaticBase, ConstraintMapping, ContactState, DenseMatrix, ExternalStaticBase, HoldWindow, MassField, StaticBaseState, StiffnessField,
    },
    identity::{BACKEND_VERSION, Binding, EntityMap, REQUIRED_ARTIFACT_ROLES, artifact_digest},
    results::{Solved, SolvedRef, StoredRef},
    sha256_hex, write_with,
};
use fieldcraft_model::Model;
use std::collections::BTreeMap;

fn fixture() -> (Model, ArchivedStaticBase) {
    let model = Model::default();
    let entity_map = EntityMap { nodes: vec![], elements: vec![] };
    let artifacts: BTreeMap<_, _> = REQUIRED_ARTIFACT_ROLES.iter().map(|role| ((*role).to_string(), artifact_digest(role.as_bytes()))).collect();
    let binding = Binding::freeze(&model, &Geometry::default(), &Run::default(), &entity_map, artifacts, BACKEND_VERSION).unwrap();
    let identity = DenseMatrix::square(vec![1.0, 0.0, 0.0, 1.0], 2).unwrap();
    let mut base = ExternalStaticBase {
        name: "external_static_base".into(),
        state: StaticBaseState {
            reference_coordinates: vec![0.0, 0.0],
            coordinates: vec![0.002, 0.001],
            velocities: vec![1e-9, 1e-9],
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
            matrix: DenseMatrix::square(vec![4.0, 0.0, 0.0, 9.0], 2).unwrap(),
            source_backend: "external_reconstruction".into(),
            probe_error: 0.0,
            symmetry_error: 0.0,
            projected: false,
        },
        mass: MassField {
            matrix: identity.clone(),
            kind: "consistent".into(),
            source_backend: "external_reconstruction".into(),
            declared_total: 2.0,
            symmetry_error: 0.0,
        },
        constraints: ConstraintMapping {
            full_dofs: 2,
            reduced_dofs: 2,
            q0: vec![0.0, 0.0],
            full_coordinates: vec![0.002, 0.001],
            reduced_coordinates: vec![0.002, 0.001],
            t: identity.clone(),
            reduced_stiffness: DenseMatrix::square(vec![4.0, 0.0, 0.0, 9.0], 2).unwrap(),
            reduced_mass: identity,
            constraint_residual: 0.0,
            projection_error: 0.0,
        },
        source_backend: "external_reconstruction".into(),
        runtime_qualification: "unverified".into(),
        frequency_response: "not_run".into(),
        modal: None,
    };
    let reference_shapes = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
    let modal = base.compute_modal(reference_shapes).unwrap();
    base.modal = Some(modal);
    let archive = ArchivedStaticBase::from_payload(
        "s07-closed-static-base".into(),
        binding,
        entity_map,
        base,
        br#"{"solve":"succeeded","coverage":"complete"}"#.to_vec(),
        br#"{"runtime_qualification":"unverified","frequency_response":"not_run"}"#.to_vec(),
        br#"{"source":"S07","backend":"external_reconstruction","command":"independent-prototype"}"#.to_vec(),
    )
    .unwrap();
    (model, archive)
}

fn save(model: &Model, archive: &ArchivedStaticBase) -> Vec<u8> {
    write_with(
        model,
        &Geometry::default(),
        &[],
        &Default::default(),
        &View::default(),
        &fieldcraft_io::Extras {
            results: vec![(
                archive.archive_key(),
                StoredRef { definition: archive.definition().unwrap(), result: SolvedRef::ExternalStaticBase(archive) },
            )],
            ..Default::default()
        },
    )
    .unwrap()
}

#[test]
fn closed_base_roundtrip_has_modal_package_and_unverified_runtime() {
    let (model, archive) = fixture();
    if let Some(path) = std::env::var_os("FIELDCRAFT_STATIC_BASE_EVIDENCE") {
        let path = std::path::PathBuf::from(path);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, serde_json::to_vec_pretty(&archive).unwrap()).unwrap();
    }
    let bytes = save(&model, &archive);
    let opened = fieldcraft_io::read(&bytes).unwrap();
    let Solved::ExternalStaticBase(restored) = &opened.results[0].result else { panic!("kind 13 static base missing") };
    assert_eq!(restored.as_ref(), &archive);
    assert_eq!(restored.external_static_base().runtime_qualification, "unverified");
    assert_eq!(restored.external_static_base().frequency_response, "not_run");
    let modal = restored.external_static_base().modal.as_ref().unwrap();
    assert_eq!(modal.modes.len(), 2);
    assert!(modal.modes.iter().all(|mode| mode.residual <= 1e-8));
    assert_eq!(save(&opened.model, restored.as_ref()), bytes);
}

#[test]
fn closed_base_rejects_open_sliding_bad_tangent_and_bad_modal() {
    let (model, archive) = fixture();
    let rebuild = |base: ExternalStaticBase| {
        ArchivedStaticBase::from_payload(
            archive.archive_key.clone(),
            archive.binding.clone(),
            archive.entity_map.clone(),
            base,
            archive.admission_json().to_vec(),
            archive.context_json().to_vec(),
            archive.source_json().to_vec(),
        )
    };
    let mut open = archive.external_static_base().clone();
    open.contact.gap_normal[0] = 2e-10;
    let error = rebuild(open).unwrap_err().to_string();
    println!("S07_REJECT_OPEN {error}");
    assert!(error.contains("contact_open"));
    let mut sliding = archive.external_static_base().clone();
    sliding.contact.tangential_increment[0] = 2e-10;
    let error = rebuild(sliding).unwrap_err().to_string();
    println!("S07_REJECT_SLIDING {error}");
    assert!(error.contains("contact_sliding"));
    let mut bad_tangent = archive.external_static_base().clone();
    bad_tangent.stiffness.probe_error = 2e-4;
    let error = rebuild(bad_tangent).unwrap_err().to_string();
    println!("S07_REJECT_TANGENT {error}");
    assert!(error.contains("tangent"));
    let mut bad_modal = archive.external_static_base().clone();
    bad_modal.modal.as_mut().unwrap().modes[0].residual = 2e-6;
    let error = rebuild(bad_modal).unwrap_err().to_string();
    println!("S07_REJECT_MODAL {error}");
    assert!(error.contains("modal"));
    assert!(archive.validate_binding(&model, &Geometry::default(), &Run::default()).is_ok());
}
