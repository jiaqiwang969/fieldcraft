//! A fully closed frictional pair against a fixed main surface: explicit stick,
//! independently predicted shear mode, persistence, and bounded refusals.
use fieldcraft_engine::{Session, StepResult};
use fieldcraft_io::{Geometry, Run, contact_policy::PolicyRequest};
use fieldcraft_solver::StepResult as Implicit;
use serde_json::{Value, json};

fn specimen(compression: f64) -> Session {
    let mut script: Value = serde_json::from_str(include_str!("fixtures/contact-stick.json")).unwrap();
    for c in script.as_array_mut().unwrap() {
        if c["command"] == "model.boundary_condition.create" && c["params"]["name"] == "top" {
            c["params"]["values"][2] = json!(-compression);
        }
    }
    let mut s = Session::default();
    s.run_script(&script).unwrap();
    s
}

#[test]
fn explicit_stick_base_modes_closed_form_and_saved_policy() {
    for compression in [0.01, 0.02] {
        let mut s = specimen(compression);
        let (geometry, run) = (Geometry::default(), Run::default());
        let declaration = json!({"Interface":"stick"});
        let request = PolicyRequest::freeze(&s.model, &geometry, &run, "Modes", &declaration).unwrap();
        request.require_applied(&s.model, &geometry, &run).unwrap();
        let digest = request.digest().unwrap();
        let info = s.execute("model.info", &json!({})).unwrap();
        let report = &info["contact_policy_capabilities"];
        assert_eq!(report["native_field_available"], true);
        assert_eq!(report["steps"]["Modes"]["selection"], "explicit_stick");
        assert_eq!(report["steps"]["Modes"]["runtime_qualification"], "unverified");
        let base_reply = s.execute("solve.nonlinear", &json!({"step":"Preload","threads":1})).unwrap();
        let mode_reply = s.execute("solve.frequency", &json!({"step":"Modes","threads":1})).unwrap();
        let StepResult::Implicit(Implicit::Static(base)) = s.result("Preload").unwrap().as_ref().clone() else { panic!("static") };
        let StepResult::Implicit(Implicit::Frequency(modes)) = s.result("Modes").unwrap().as_ref().clone() else { panic!("modes") };
        let at = |label| base.nodes.binary_search(&label).unwrap();
        let stretch = 1.0 + base.displacement[at(13)][2] - base.displacement[at(9)][2];
        // St Venant–Kirchhoff: F=diag(1,1,lambda), nu=0. The X shear
        // tangent is G+Szz=G*lambda^2; reference consistent mass rho*A*H/3.
        // Four real 1 N/m lateral springs are retained, not solver stabilization.
        let expected = (3.0 * (500.0 * stretch.powi(2) + 4.0)).sqrt() / std::f64::consts::TAU;
        let relative = (modes.modes[0].frequency / expected - 1.0).abs();
        assert!(relative < 1e-6, "f={} reference={expected} error={relative}", modes.modes[0].frequency);
        let normal = -500.0 * stretch * (stretch * stretch - 1.0);
        assert!((base.contacts[0].normal_force / normal - 1.0).abs() < 1e-6);
        assert!(base.reaction_total.iter().all(|r| r.abs() < 1e-8));
        assert!(base.contacts[0].status.iter().all(|s| *s == fieldcraft_solver::contact::Status::Stick));
        let shape = &modes.modes[0].shape;
        assert!((9..=12).all(|n| shape[at(n)].iter().all(|x| x.abs() < 1e-12)));
        assert!((13..=16).all(|n| (shape[at(n)][0] - shape[at(13)][0]).abs() < 1e-8));
        assert!(modes.warnings.iter().any(|s| s.contains("policy=stick state=closed_stick")));
        assert_eq!(modes.stats.equations, 4);
        let history = base.nonlinear.as_ref().unwrap();
        assert!(history.increments.last().unwrap().converged);
        assert_eq!(history.increments.last().unwrap().load_factor, 1.0);
        eprintln!(
            "STICK compression={compression} lambda={stretch:.12e} f={:.12e} reference={expected:.12e} error={relative:.3e} force={normal:.12e} iterations={} final_force_residual={:.12e}",
            modes.modes[0].frequency,
            history.iterations,
            history.increments.last().unwrap().iterations.last().unwrap().force
        );

        s.execute("solve.frequency", &json!({"step":"Modes","threads":4})).unwrap();
        let StepResult::Implicit(Implicit::Frequency(many)) = s.result("Modes").unwrap().as_ref() else { panic!("modes") };
        assert_eq!(&modes, many, "thread-independent entire result");
        let saved = s.result("Modes").unwrap().clone();
        let evidence = std::env::var_os("FIELDCRAFT_CONTACT_EVIDENCE").map(std::path::PathBuf::from);
        let root = evidence.clone().unwrap_or_else(|| std::env::temp_dir().join(format!("contact-stick-{}", std::process::id())));
        std::fs::create_dir_all(&root).unwrap();
        let path = root.join(format!("stick-{compression}.fieldcraft"));
        s.execute("file.save_as", &json!({"path":path})).unwrap();
        let mut reopened = Session::default();
        reopened.execute("file.open", &json!({"path":path})).unwrap();
        let archive = fieldcraft_io::read(&std::fs::read(&path).unwrap()).unwrap();
        let request_path = root.join(format!("stick-{compression}-request.json"));
        std::fs::write(&request_path, serde_json::to_vec_pretty(&request).unwrap()).unwrap();
        let restored =
            PolicyRequest::decode(&std::fs::read(&request_path).unwrap(), &digest, &archive.model, &archive.geometry, &archive.run).unwrap();
        restored.require_applied(&archive.model, &archive.geometry, &archive.run).unwrap();
        assert_eq!(request, restored);
        assert_eq!(s.model.step("Modes"), reopened.model.step("Modes"));
        assert_eq!(Some(&saved), reopened.result("Modes"));
        assert_eq!(s.result("Preload"), reopened.result("Preload"));
        if evidence.is_some() {
            for (name, value) in [
                ("base", serde_json::to_value(&base).unwrap()),
                ("modes", serde_json::to_value(&modes).unwrap()),
                ("base-command", base_reply),
                ("modes-command", mode_reply),
            ] {
                std::fs::write(root.join(format!("stick-{compression}-{name}.json")), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
            }
        } else {
            std::fs::remove_file(path).unwrap();
            std::fs::remove_file(request_path).unwrap();
        }
        let error = fieldcraft_decks::abaqus::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string();
        assert!(error.contains("contact_perturbation"), "{error}");
        eprintln!("writer refusal: {error}");
        let error = fieldcraft_decks::nastran::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string();
        assert!(error.contains("contact_perturbation"), "{error}");
        eprintln!("writer refusal: {error}");
        let error = fieldcraft_decks::lsdyna::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string();
        assert!(error.contains("contact_perturbation"), "{error}");
        eprintln!("writer refusal: {error}");
        s.execute("model.step.edit", &json!({"name":"Modes","contact_perturbation":{}})).unwrap();
        assert!(s.result("Modes").is_none(), "policy participates in result invalidation");
        let stale = request.require_applied(&s.model, &geometry, &run).unwrap_err().to_string();
        assert!(stale.contains("ModelRevisionMismatch"), "{stale}");
        let unapplied = PolicyRequest::freeze(&s.model, &geometry, &run, "Modes", &declaration).unwrap();
        let refused = unapplied.require_applied(&s.model, &geometry, &run).unwrap_err().to_string();
        assert!(refused.contains("NativeContactPolicyNotApplied"), "{refused}");
        eprintln!("request applied and reopened; after edit: {stale}; fresh unapplied: {refused}");
    }
}

#[test]
fn explicit_stick_has_no_silent_fallback() {
    let mut s = specimen(0.01);
    for p in [
        json!({"name":"Modes","contact_perturbation":{"Unknown":"stick"}}),
        json!({"name":"Modes","contact_perturbation":{"Interface":"slide"}}),
        json!({"name":"Modes","base_state":null}),
        json!({"name":"Modes","boundary_conditions":["base","top"]}),
    ] {
        assert!(s.execute("model.step.edit", &p).is_err());
    }
    let mut movable = specimen(0.01);
    movable.model.replace_set("base", fieldcraft_model::Set { nodes: (1..=4).collect(), ..Default::default() }).unwrap();
    let error = movable.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
    assert!(error.contains("stick_requires_full_overlap") && error.contains("Interface"), "{error}");
    // Removing a contact or changing supports after step creation must also be
    // caught on the read/resolve path, not only by the step-edit command.
    let mut stale = s.model.clone();
    stale.remove_contact("Interface").unwrap();
    assert!(stale.resolve().is_err());
    let mut unsupported = s.model.step("Preload").unwrap().clone();
    unsupported.boundary_conditions.remove("upper");
    s.model.replace_step(unsupported).unwrap();
    assert!(s.model.resolve().is_err());
    s = specimen(0.01);
    let mut c = s.model.contact("Interface").unwrap().clone();
    c.friction = fieldcraft_model::Friction::NONE;
    s.model.replace_contact("Interface", c).unwrap();
    let error = s.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
    assert!(error.contains("friction_state_unavailable") && error.contains("Interface"), "{error}");
    let mut legacy = specimen(0.01);
    legacy.execute("model.step.edit", &json!({"name":"Modes","contact_perturbation":{}})).unwrap();
    let reply = legacy.execute("solve.frequency", &json!({"step":"Modes"})).unwrap();
    assert!(reply.to_string().contains("legacy_bonded_when_closed"));
}
