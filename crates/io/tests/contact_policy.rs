use fieldcraft_io::{Geometry, Run, contact_policy::*};
use fieldcraft_model::{BoundaryCondition, Contact, ContactSurfaces, ElementKind, Friction, Model, NonlinearStatic, Procedure, Step, Target};
use serde_json::json;

fn fixture() -> Model {
    let mut m = Model::default();
    for (i, xyz) in
        [[0., 0., 0.], [0., 1., 0.], [1., 0., 0.], [0., 0., -1.], [0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].into_iter().enumerate()
    {
        m.set_node(i as u32 + 1, xyz).unwrap();
    }
    m.set_element(10, ElementKind::Tet4, vec![1, 2, 3, 4]).unwrap();
    m.set_element(20, ElementKind::Tet4, vec![5, 6, 7, 8]).unwrap();
    for (name, element) in [("main", 10), ("secondary", 20)] {
        m.add_set(name, serde_json::from_value(json!({"faces":[{"element":element,"face":1}]})).unwrap()).unwrap();
    }
    m.add_set("fixed", serde_json::from_value(json!({"nodes":[1,2,3,4]})).unwrap()).unwrap();
    m.add_boundary_condition("Fixed", BoundaryCondition::Fixed { target: Target::Set("fixed".into()) }).unwrap();
    let mut contact = Contact::new(ContactSurfaces::Pair { main: "main".into(), secondary: "secondary".into() });
    contact.friction = Friction::coulomb(0.3);
    m.add_contact("Interface", contact).unwrap();
    let mut base = Step::new("Preload", Procedure::NonlinearStatic(NonlinearStatic::default()));
    base.boundary_conditions.insert("Fixed".into());
    m.add_step(base, None).unwrap();
    let mut modes = Step::new("Modes", Procedure::Frequency { modes: 1, min_hz: None, max_hz: None });
    modes.base_state = Some("Preload".into());
    modes.boundary_conditions.insert("Fixed".into());
    m.add_step(modes, None).unwrap();
    m
}
#[test]
fn declared_stick_freezes_and_reopens_without_claiming_native_application() {
    let m = fixture();
    let g = Geometry::default();
    let run = Run::default();
    let request = PolicyRequest::freeze(&m, &g, &run, "Modes", &json!({"Interface":"stick"})).unwrap();
    request.validate(&m, &g, &run).unwrap();
    assert!(request.require_applied(&m, &g, &run).unwrap_err().to_string().contains("NativeContactPolicyNotApplied"));
    let bytes = fieldcraft_io::write(&m, &g, &[], &Default::default(), &Default::default()).unwrap();
    let dir = std::env::var_os("FIELDCRAFT_POLICY_EVIDENCE")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join(format!("fieldcraft-policy-{}", std::process::id())));
    fieldcraft_io::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("declaration-model.fieldcraft");
    fieldcraft_io::fs::save_model(&path, &bytes).unwrap();
    let reopened = fieldcraft_io::read(&fieldcraft_io::fs::read_capped(&path, fieldcraft_io::MAX_FILE_BYTES).unwrap()).unwrap();
    let serialized = serde_json::to_vec_pretty(&request).unwrap();
    fieldcraft_io::fs::write_atomic(&dir.join("policy-request.json"), &serialized, false).unwrap();
    let stored = fieldcraft_io::fs::read_capped(&dir.join("policy-request.json"), 1024 * 1024).unwrap();
    let digest = request.digest().unwrap();
    let restored = PolicyRequest::decode(&stored, &digest, &reopened.model, &reopened.geometry, &reopened.run).unwrap();
    assert_eq!(request, restored);
    assert!(restored.require_applied(&reopened.model, &reopened.geometry, &reopened.run).is_err());
    let capabilities = report(&reopened.model).unwrap();
    assert_eq!(capabilities["steps"]["Modes"]["selection"], "legacy_bonded_when_closed");
    assert_eq!(capabilities["steps"]["Modes"]["runtime_qualification"], "unverified");
    fieldcraft_io::fs::write_atomic(&dir.join("policy-capabilities.json"), &serde_json::to_vec_pretty(&capabilities).unwrap(), false).unwrap();
    fieldcraft_io::fs::write_atomic(&dir.join("request.sha256"), digest.as_bytes(), false).unwrap();
}
#[test]
fn names_values_coverage_and_pair_scope_are_checked() {
    let mut m = fixture();
    let g = Geometry::default();
    let r = Run::default();
    for (declaration, reason) in [
        (json!({"Interface":"slide"}), "UnsupportedContactPolicy"),
        (json!({"Interface":"coulomb_sliding"}), "UnsupportedContactPolicy"),
        (json!({"interface":"stick"}), "UnknownPolicyContact"),
    ] {
        assert!(PolicyRequest::freeze(&m, &g, &r, "Modes", &declaration).unwrap_err().to_string().contains(reason));
    }
    m.add_contact("Second", m.contact("Interface").unwrap().clone()).unwrap();
    assert!(PolicyRequest::freeze(&m, &g, &r, "Modes", &json!({"Interface":"stick"})).unwrap_err().to_string().contains("MissingContactPolicy"));
    m.replace_contact("Interface", Contact::new(ContactSurfaces::General { surface: None })).unwrap();
    assert!(
        PolicyRequest::freeze(&m, &g, &r, "Modes", &json!({"Interface":"stick","Second":"stick"}))
            .unwrap_err()
            .to_string()
            .contains("ContactPolicyPairRequired")
    );
}
#[test]
fn analysis_base_and_support_references_are_checked() {
    let mut m = fixture();
    let g = Geometry::default();
    let r = Run::default();
    let p = json!({"Interface":"stick"});
    assert!(PolicyRequest::freeze(&m, &g, &r, "Preload", &p).unwrap_err().to_string().contains("ContactPolicyFrequencyRequired"));
    let mut modes = m.step("Modes").unwrap().clone();
    modes.boundary_conditions.clear();
    m.replace_step(modes).unwrap();
    assert!(PolicyRequest::freeze(&m, &g, &r, "Modes", &p).unwrap_err().to_string().contains("ContactPolicyUnchangedSupportsRequired"));
    let mut base = m.step("Preload").unwrap().clone();
    base.procedure = Procedure::NonlinearStatic(NonlinearStatic { nonlinear_geometry: false, ..Default::default() });
    m.replace_step(base).unwrap();
    assert!(PolicyRequest::freeze(&m, &g, &r, "Modes", &p).unwrap_err().to_string().contains("ContactPolicyNonlinearBaseRequired"));
}
#[test]
fn request_digest_model_change_and_forged_runtime_status_are_rejected() {
    let mut m = fixture();
    let g = Geometry::default();
    let r = Run::default();
    let p = PolicyRequest::freeze(&m, &g, &r, "Modes", &json!({"Interface":"stick"})).unwrap();
    let digest = p.digest().unwrap();
    let mut changed = p.clone();
    changed.contact_perturbation.clear();
    assert!(
        PolicyRequest::decode(&serde_json::to_vec(&changed).unwrap(), &digest, &m, &g, &r)
            .unwrap_err()
            .to_string()
            .contains("ContactPolicyRequestMismatch")
    );
    let mut forged = serde_json::to_value(&p).unwrap();
    forged["runtime_qualification"] = json!("closed_stick");
    assert!(serde_json::from_value::<PolicyRequest>(forged).is_err());
    m.set_node(8, [0., 0., 2.]).unwrap();
    assert!(p.validate(&m, &g, &r).unwrap_err().to_string().contains("ReferenceMeshMismatch"));
}
#[test]
fn empty_declaration_is_legacy_not_stick() {
    let m = fixture();
    let p = PolicyRequest::freeze(&m, &Geometry::default(), &Run::default(), "Modes", &json!({})).unwrap();
    assert!(p.contact_perturbation.is_empty());
    p.require_applied(&m, &Geometry::default(), &Run::default()).unwrap();
    assert_eq!(report(&m).unwrap()["steps"]["Modes"]["runtime_qualification"], "unverified");
}

#[test]
fn native_tangent_free_is_reported_and_frozen_without_claiming_qualification() {
    use fieldcraft_model::ContactPerturbation;
    let mut m = fixture();
    let mut modes = m.step("Modes").unwrap().clone();
    modes.contact_perturbation.insert("Interface".into(), ContactPerturbation::ClosedTangentFree);
    m.replace_step(modes).unwrap();
    let capabilities = report(&m).unwrap();
    assert_eq!(capabilities["steps"]["Modes"]["selection"], "explicit_closed_tangent_free");
    assert_eq!(capabilities["steps"]["Modes"]["runtime_qualification"], "unverified");
    let geometry = Geometry::default();
    let run = Run::default();
    let request = PolicyRequest::freeze(&m, &geometry, &run, "Modes", &json!({"Interface":"closed_tangent_free"})).unwrap();
    request.require_applied(&m, &geometry, &run).unwrap();
    let archive = fieldcraft_io::write(&m, &geometry, &[], &Default::default(), &Default::default()).unwrap();
    let reopened = fieldcraft_io::read(&archive).unwrap();
    assert_eq!(report(&reopened.model).unwrap(), capabilities);
    let restored =
        PolicyRequest::decode(&serde_json::to_vec(&request).unwrap(), &request.digest().unwrap(), &reopened.model, &reopened.geometry, &reopened.run)
            .unwrap();
    restored.require_applied(&reopened.model, &reopened.geometry, &reopened.run).unwrap();
    let mut changed = reopened.model.step("Modes").unwrap().clone();
    changed.contact_perturbation.insert("Interface".into(), ContactPerturbation::Stick);
    m.replace_step(changed).unwrap();
    assert!(request.validate(&m, &geometry, &run).unwrap_err().to_string().contains("ModelRevisionMismatch"));
}

#[test]
fn mixed_native_selection_does_not_report_stick_for_every_contact() {
    use fieldcraft_model::ContactPerturbation;
    let mut m = fixture();
    m.add_contact("Second", m.contact("Interface").unwrap().clone()).unwrap();
    let mut modes = m.step("Modes").unwrap().clone();
    modes.contact_perturbation.insert("Interface".into(), ContactPerturbation::Stick);
    modes.contact_perturbation.insert("Second".into(), ContactPerturbation::ClosedTangentFree);
    m.replace_step(modes).unwrap();
    let capability = report(&m).unwrap();
    assert_eq!(capability["steps"]["Modes"]["selection"], "explicit_mixed");
    assert_eq!(capability["steps"]["Modes"]["runtime_qualification"], "unverified");
    let request =
        PolicyRequest::freeze(&m, &Geometry::default(), &Run::default(), "Modes", &json!({"Interface":"stick","Second":"closed_tangent_free"}))
            .unwrap();
    request.require_applied(&m, &Geometry::default(), &Run::default()).unwrap();
}

#[test]
fn open_declaration_is_frozen_but_does_not_certify_separation() {
    use fieldcraft_model::ContactPerturbation;
    let mut m = fixture();
    let g = Geometry::default();
    let r = Run::default();
    let declared = PolicyRequest::freeze(&m, &g, &r, "Modes", &json!({"Interface":"open"})).unwrap();
    assert!(declared.require_applied(&m, &g, &r).unwrap_err().to_string().contains("NativeContactPolicyNotApplied"));
    let mut step = m.step("Modes").unwrap().clone();
    step.contact_perturbation.insert("Interface".into(), ContactPerturbation::Open);
    m.replace_step(step).unwrap();
    assert!(declared.validate(&m, &g, &r).is_err(), "application changed the frozen model revision");
    let request = PolicyRequest::freeze(&m, &g, &r, "Modes", &json!({"Interface":"open"})).unwrap();
    request.require_applied(&m, &g, &r).unwrap();
    let info = report(&m).unwrap();
    assert_eq!(info["steps"]["Modes"]["selection"], "explicit_open");
    assert_eq!(info["steps"]["Modes"]["runtime_qualification"], "unverified");
    assert_eq!(info["policy_specification"], "fieldcraft-native-contact-perturbation/4");
    assert!(info["qualification_requires"].as_array().unwrap().iter().all(|v| v != "fixed_main_translations"));
    assert!(info["qualification_by_policy"]["closed_tangent_free"].as_array().unwrap().iter().any(|v| v == "fixed_main_translations"));
    assert!(info["qualification_by_policy"]["open"].as_array().unwrap().iter().any(|v| v == "fixed_or_moving_main"));
    let bytes = fieldcraft_io::write(&m, &g, &[], &Default::default(), &Default::default()).unwrap();
    let reopened = fieldcraft_io::read(&bytes).unwrap();
    let recovered =
        PolicyRequest::decode(&serde_json::to_vec(&request).unwrap(), &request.digest().unwrap(), &reopened.model, &reopened.geometry, &reopened.run)
            .unwrap();
    recovered.require_applied(&reopened.model, &reopened.geometry, &reopened.run).unwrap();
    assert_eq!(report(&reopened.model).unwrap(), info);
    // This fixture is geometrically touching. IO certifies identity and application only.
    assert_eq!(m.node(1).unwrap().xyz[2], m.node(5).unwrap().xyz[2]);
}
