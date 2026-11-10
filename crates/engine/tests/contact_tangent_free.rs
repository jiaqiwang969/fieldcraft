//! Hard-normal, tangentially free contact: a closed-form two-layer shear system.
//! The lower main face is wider, so its clipping boundary stays outside the patch.
use fieldcraft_engine::{Session, StepResult};
use fieldcraft_solver::StepResult as Implicit;
use serde_json::{Value, json};

fn specimen(friction: bool, compression: f64, shear: Option<f64>, interior: bool) -> Session {
    let mut script: Value = serde_json::from_str(include_str!("fixtures/contact-tangent-free.json")).unwrap();
    for c in script.as_array_mut().unwrap() {
        if c["command"] == "model.boundary_condition.create" && c["params"]["name"] == "top" {
            c["params"]["values"][2] = json!(-compression);
            c["params"]["values"][0] = json!(shear);
        }
        if !interior && c["command"] == "mesh.node.add" && c["params"]["label"].as_u64().unwrap() <= 8 {
            for k in 0..2 {
                c["params"]["xyz"][k] = json!((c["params"]["xyz"][k].as_f64().unwrap() + 0.25) / 1.5);
            }
        }
    }
    let mut s = Session::default();
    s.run_script(&script).unwrap();
    if !friction {
        let mut c = s.model.contact("Interface").unwrap().clone();
        c.friction = fieldcraft_model::Friction::NONE;
        s.model.replace_contact("Interface", c).unwrap();
    }
    s
}

#[test]
fn closed_tangent_free_modes_reference_and_persistence() {
    let friction = false;
    let mut s = specimen(friction, 0.01, None, true);
    let base_reply = s.execute("solve.nonlinear", &json!({"step":"Preload","threads":1})).unwrap();
    let mode_reply = s.execute("solve.frequency", &json!({"step":"Modes","threads":1})).unwrap();
    let StepResult::Implicit(Implicit::Static(base)) = s.result("Preload").unwrap().as_ref().clone() else { panic!("static") };
    let StepResult::Implicit(Implicit::Frequency(modes)) = s.result("Modes").unwrap().as_ref().clone() else { panic!("modes") };
    if let Some(root) = std::env::var_os("FIELDCRAFT_TANGENT_FREE_EVIDENCE").map(std::path::PathBuf::from) {
        std::fs::create_dir_all(&root).unwrap();
        for (name, value) in [
            ("base", serde_json::to_value(&base).unwrap()),
            ("modes", serde_json::to_value(&modes).unwrap()),
            ("base-command", base_reply.clone()),
            ("modes-command", mode_reply.clone()),
        ] {
            std::fs::write(root.join(format!("raw-{friction}-{name}.json")), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
        }
        s.execute("file.save_as", &json!({"path":root.join(format!("raw-{friction}.fieldcraft"))})).unwrap();
    }
    let at = |label| base.nodes.binary_search(&label).unwrap();
    for (i, mode) in modes.modes.iter().enumerate() {
        eprintln!(
            "TANGENT_FREE_RAW friction={friction} mode={} f={:.12e} lower={:?} upper={:?}",
            i + 1,
            mode.frequency,
            (9..=12).map(|n| mode.shape[at(n)]).collect::<Vec<_>>(),
            (13..=16).map(|n| mode.shape[at(n)]).collect::<Vec<_>>()
        );
    }
    let stretch = 1.0 + base.displacement[at(13)][2] - base.displacement[at(9)][2];
    // Uniform X motion on each of two layers: K=s[[1,-1],[-1,1]]+diag(0,4),
    // M=[[2,1],[1,2]]/6. s=G+Szz=500*lambda^2; reference mass is 1 kg.
    let shear = 500.0 * stretch.powi(2);
    let b = shear + 4.0 / 3.0;
    let disc = (b * b - 4.0 * shear / 3.0).sqrt();
    let expected = [8.0 * shear / (b + disc), 6.0 * (b + disc)].map(|w2| w2.sqrt() / std::f64::consts::TAU);
    let selected: Vec<_> = modes
        .modes
        .iter()
        .enumerate()
        .filter(|(_, mode)| {
            let sh = &mode.shape;
            let scale = sh.iter().flatten().copied().map(f64::abs).fold(0.0, f64::max);
            (9..=16).all(|n| sh[at(n)][1].abs().max(sh[at(n)][2].abs()) < 1e-7 * scale)
                && (9..=12).all(|n| (sh[at(n)][0] - sh[at(9)][0]).abs() < 1e-7 * scale)
                && (13..=16).all(|n| (sh[at(n)][0] - sh[at(13)][0]).abs() < 1e-7 * scale)
        })
        .collect();
    assert_eq!(selected.len(), 2, "retain the raw modes; select by components, not frequency order");
    for ((index, mode), reference) in selected.iter().zip(expected) {
        let relative = (mode.frequency / reference - 1.0).abs();
        assert!(relative < 1e-6, "mode {} f={} reference={reference} error={relative}", index + 1, mode.frequency);
        let residual = mode_reply["modes"][*index]["equation_residual"]["relative_residual"].as_f64().unwrap();
        assert!(residual < 1e-8, "physical modal residual {residual}");
        assert!(mode.shape[at(9)][0].abs() > 1e-3, "interface tangent is free to move");
        eprintln!(
            "TANGENT_FREE friction={friction} mode={} f={:.15e} reference={reference:.15e} error={relative:.3e} residual={residual:.3e}",
            index + 1,
            mode.frequency
        );
    }
    let normal = -500.0 * stretch * (stretch * stretch - 1.0);
    assert!((base.contacts[0].normal_force / normal - 1.0).abs() < 1e-6);
    assert!(base.reaction_total.iter().all(|v| v.abs() < 1e-8));
    assert_eq!(modes.stats.equations, 8);
    assert!(modes.warnings.iter().any(|w| w.contains("policy=closed_tangent_free") && w.contains("geometry included")));
    let history = base.nonlinear.as_ref().unwrap();
    assert!(history.increments.last().unwrap().converged);
    assert_eq!(history.increments.last().unwrap().load_factor, 1.0);
    eprintln!(
        "TANGENT_FREE friction={friction} normal={normal:.15e} lambda={stretch:.15e} iterations={} final_force={:.3e}",
        history.iterations,
        history.increments.last().unwrap().iterations.last().unwrap().force
    );
    s.execute("solve.frequency", &json!({"step":"Modes","threads":4})).unwrap();
    let StepResult::Implicit(Implicit::Frequency(many)) = s.result("Modes").unwrap().as_ref() else { panic!("modes") };
    assert_eq!(&modes, many, "entire result, independent of thread count");
    let saved = s.result("Modes").unwrap().clone();
    let evidence = std::env::var_os("FIELDCRAFT_TANGENT_FREE_EVIDENCE").map(std::path::PathBuf::from);
    let root = evidence.clone().unwrap_or_else(|| std::env::temp_dir().join(format!("contact-tangent-free-{}", std::process::id())));
    std::fs::create_dir_all(&root).unwrap();
    let path = root.join(format!("free-{friction}.fieldcraft"));
    s.execute("file.save_as", &json!({"path":path})).unwrap();
    let mut reopened = Session::default();
    reopened.execute("file.open", &json!({"path":path})).unwrap();
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
            std::fs::write(root.join(format!("free-{friction}-{name}.json")), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
        }
    } else {
        std::fs::remove_file(path).unwrap();
    }
    for error in [
        fieldcraft_decks::abaqus::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string(),
        fieldcraft_decks::nastran::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string(),
        fieldcraft_decks::lsdyna::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string(),
    ] {
        assert!(error.contains("contact_perturbation"), "{error}");
    }
    s.execute("model.step.edit", &json!({"name":"Modes","contact_perturbation":{"Interface":"stick"}})).unwrap();
    assert!(s.result("Modes").is_none());
}

#[test]
fn closed_tangent_free_refuses_the_actual_frictional_base_shear() {
    // The same finite-penetration base with friction is not a zero-shear state.
    // Keep the original candidate and its refusal; do not relax the shear guard.
    let mut s = specimen(true, 0.01, None, true);
    let reply = s.execute("solve.nonlinear", &json!({"step":"Preload"})).unwrap();
    let StepResult::Implicit(Implicit::Static(base)) = s.result("Preload").unwrap().as_ref() else { panic!("static") };
    assert!(base.nonlinear.as_ref().unwrap().increments.last().unwrap().converged);
    if let Some(root) = std::env::var_os("FIELDCRAFT_TANGENT_FREE_EVIDENCE").map(std::path::PathBuf::from) {
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(root.join("frictional-base.json"), serde_json::to_vec_pretty(base).unwrap()).unwrap();
        std::fs::write(root.join("frictional-base-command.json"), serde_json::to_vec_pretty(&reply).unwrap()).unwrap();
    }
    let error = s.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
    eprintln!("TANGENT_FREE_FRICTIONAL_BASE_REFUSAL {error}");
    assert!(error.contains("shear_release_undefined"), "{error}");
    assert!(s.result("Modes").is_none());
}

#[test]
fn closed_tangent_free_refuses_unqualified_states() {
    // The unloaded touching base fails before policy qualification; a converged-
    // state test separately exercises normal_switch. Applied shear reaches the guard.
    for (compression, shear, interior, expected) in [
        (0.0, None, true, "did not converge"),
        (0.01, Some(0.001), true, "shear_release_undefined"),
        (0.01, None, false, "clipping_branch_unqualified"),
    ] {
        let mut s = specimen(true, compression, shear, interior);
        let error = s.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
        eprintln!("TANGENT_FREE_REFUSAL {expected}: {error}");
        assert!(error.contains(expected), "{error}");
        assert!(s.result("Modes").is_none());
    }
    let mut moving = specimen(false, 0.01, None, true);
    moving.model.replace_set("base", fieldcraft_model::Set { nodes: (1..=4).collect(), ..Default::default() }).unwrap();
    let error = moving.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
    assert!(error.contains("closed_tangent_free_requires_fixed_main"), "{error}");
    let mut s = specimen(false, 0.01, None, true);
    for params in [
        json!({"name":"Modes","base_state":null}),
        json!({"name":"Modes","contact_perturbation":{"Interface":"open"}}),
        json!({"name":"Modes","boundary_conditions":["base","top"]}),
        json!({"name":"Modes","procedure":{"type":"harmonic","frequencies":[1.0]}}),
    ] {
        assert!(s.execute("model.step.edit", &params).is_err());
    }
}
