//! Open unilateral contact has zero incremental stiffness. Original separated blocks,
//! with a real spring restraint, have three coupled shear/rotation modes and one axial mode.
use fieldcraft_engine::{Session, StepResult};
use fieldcraft_solver::StepResult as Implicit;
use serde_json::{Value, json};

fn specimen(gap: f64, compression: f64) -> Session {
    let mut script: Value = serde_json::from_str(include_str!("fixtures/contact-tangent-free.json")).unwrap();
    for c in script.as_array_mut().unwrap() {
        if c["command"] == "mesh.node.add" && c["params"]["label"].as_u64().unwrap() >= 9 {
            c["params"]["xyz"][2] = json!(c["params"]["xyz"][2].as_f64().unwrap() + gap);
        }
        if c["command"] == "model.boundary_condition.create" && c["params"]["name"] == "top" {
            c["params"]["values"][2] = json!(-compression);
        }
        if c["command"] == "model.step.create" && c["params"]["name"] == "Modes" {
            c["params"]["contact_perturbation"]["Interface"] = json!("open");
            c["params"]["procedure"]["modes"] = json!(12);
        }
    }
    if compression == 0.0
        && let Some(root) = std::env::var_os("FIELDCRAFT_OPEN_EVIDENCE").map(std::path::PathBuf::from)
    {
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(root.join(format!("open-{gap}-input.json")), serde_json::to_vec_pretty(&script).unwrap()).unwrap();
    }
    let mut s = Session::default();
    s.run_script(&script).unwrap();
    s
}

#[test]
fn separated_open_modes_closed_form_legacy_and_reopen() {
    // Certify current geometric separation at two distances. Search pairing is
    // recorded, not inferred from distance; the small grid may retain both pairs.
    for gap in [0.02, 2.0] {
        let mut s = specimen(gap, 0.0);
        let base_reply = s.execute("solve.nonlinear", &json!({"step":"Preload","threads":1})).unwrap();
        let mode_reply = s.execute("solve.frequency", &json!({"step":"Modes","threads":1})).unwrap();
        let StepResult::Implicit(Implicit::Static(base)) = s.result("Preload").unwrap().as_ref().clone() else { panic!("base") };
        let StepResult::Implicit(Implicit::Frequency(modes)) = s.result("Modes").unwrap().as_ref().clone() else { panic!("modes") };
        if let Some(root) = std::env::var_os("FIELDCRAFT_OPEN_EVIDENCE").map(std::path::PathBuf::from) {
            for (name, value) in [("raw-base", serde_json::to_value(&base).unwrap()), ("raw-modes", serde_json::to_value(&modes).unwrap())] {
                std::fs::write(root.join(format!("open-{gap}-{name}.json")), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
            }
        }
        let contact = &base.contacts[0];
        assert!(contact.pressure.iter().all(|v| *v == 0.0));
        assert_eq!(contact.normal_force, 0.0);
        assert_eq!(contact.force, [0.0; 3]);
        assert!(contact.status.iter().all(|v| *v == fieldcraft_solver::contact::Status::Open));
        assert!(base.displacement.iter().flatten().all(|v| v.abs() < 1e-12));
        assert!(base.reaction_total.iter().all(|v| v.abs() < 1e-12));
        let history = base.nonlinear.as_ref().unwrap();
        let last = history.increments.last().unwrap();
        assert!(last.converged && last.load_factor == 1.0);
        assert!(modes.warnings.iter().any(|w| w.contains("policy=open") && w.contains("zero contact stiffness and geometry")));
        assert_eq!(modes.stats.equations, 12);
        let at = |n| base.nodes.binary_search(&n).unwrap();
        let mut shear = Vec::new();
        let mut axial = Vec::new();
        for (i, mode) in modes.modes.iter().enumerate() {
            let sh = &mode.shape;
            let scale = sh.iter().flatten().copied().map(f64::abs).fold(0.0, f64::max);
            // Reflection symmetry defines this invariant subspace: layer-uniform Ux,
            // lower Uz = theta*(x-1/2), upper Uz=0. No frequency proximity filter.
            let theta = sh[at(10)][2] - sh[at(9)][2];
            if (9..=16).all(|n| sh[at(n)][1].abs() < 1e-7 * scale)
                && (9..=12).all(|n| (sh[at(n)][0] - sh[at(9)][0]).abs() < 1e-7 * scale)
                && (13..=16).all(|n| (sh[at(n)][0] - sh[at(13)][0]).abs() < 1e-7 * scale)
                && (9..=12).all(|n| (sh[at(n)][2] - theta * (s.model.node(n).unwrap().xyz[0] - 0.5)).abs() < 1e-7 * scale)
            {
                shear.push((i, mode));
            }
            if (9..=16).all(|n| sh[at(n)][0].abs().max(sh[at(n)][1].abs()) < 1e-7 * scale)
                && (9..=12).all(|n| (sh[at(n)][2] - sh[at(9)][2]).abs() < 1e-7 * scale)
            {
                axial.push((i, mode));
            }
        }
        assert_eq!(shear.len(), 3, "mode pairing uses components, not proximity to a target frequency");
        assert_eq!(axial.len(), 1);
        // L=A=rho=1, E=1000, nu=0, G=500. Coordinates (lower Ux, upper Ux, theta).
        // Integrating G*(du/dz+dw/dx)^2 + E*(dw/dz)^2 and the four real springs gives
        // K=[[500,-500,-250],[-500,504,250],[-250,250,250]],
        // M=[[1/3,1/6,0],[1/6,1/3,0],[0,0,1/36]]. Thus det(K-z*M)=-p(z)/432.
        // Each disjoint interval brackets one of the three positive roots of this
        // independently integrated cubic; bisection is not a second FE eigensolver.
        let polynomial = |z: f64| ((z - 15016.0) * z + 27168000.0) * z - 108000000.0;
        let mut expected = Vec::new();
        for (mut lo, mut hi) in [(0.0, 10.0), (1000.0, 3000.0), (12000.0, 16000.0)] {
            assert!(polynomial(lo) * polynomial(hi) < 0.0);
            for _ in 0..60 {
                let mid = (lo + hi) / 2.0;
                if polynomial(lo) * polynomial(mid) <= 0.0 {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            expected.push(((lo + hi) / 2.0).sqrt() / std::f64::consts::TAU);
        }
        // Uniform normal motion of the lower layer has Kz=1000, Mz=1/3.
        expected.push(3000.0_f64.sqrt() / std::f64::consts::TAU);
        for ((i, mode), reference) in shear.iter().chain(&axial).zip(expected) {
            let relative = (mode.frequency / reference - 1.0).abs();
            let residual = mode_reply["modes"][*i]["equation_residual"]["relative_residual"].as_f64().unwrap();
            eprintln!(
                "OPEN_MODE gap={gap} mode={} f={:.15e} reference={reference:.15e} relative={relative:.3e} residual={residual:.3e}",
                i + 1,
                mode.frequency
            );
            assert!(relative < 1e-6);
            assert!(residual < 1e-8);
        }
        eprintln!(
            "OPEN_BASE gap={gap} unpaired={} force={:?} iterations={} last_force={:?}",
            contact.gap.iter().filter(|v| v.is_none()).count(),
            contact.force,
            history.iterations,
            last.iterations.last().map(|v| v.force)
        );
        // Empty selection preserves legacy behaviour. On this truly open base the
        // numerical operator, shapes and residuals must be identical to explicit open.
        let mut legacy = s.clone();
        legacy.execute("model.step.edit", &json!({"name":"Modes","contact_perturbation":{}})).unwrap();
        let legacy_reply = legacy.execute("solve.frequency", &json!({"step":"Modes","threads":1})).unwrap();
        let StepResult::Implicit(Implicit::Frequency(old)) = legacy.result("Modes").unwrap().as_ref() else { panic!("legacy") };
        assert_eq!(old.modes, modes.modes);
        assert_eq!(legacy_reply["modes"], mode_reply["modes"]);
        assert!(old.warnings.iter().any(|w| w.contains("legacy_bonded_when_closed")));
        s.execute("solve.frequency", &json!({"step":"Modes","threads":4})).unwrap();
        assert_eq!(s.result("Modes").unwrap().as_ref(), &StepResult::Implicit(Implicit::Frequency(modes.clone())));
        let evidence = std::env::var_os("FIELDCRAFT_OPEN_EVIDENCE").map(std::path::PathBuf::from);
        let root = evidence.clone().unwrap_or_else(|| std::env::temp_dir().join(format!("contact-open-{}", std::process::id())));
        std::fs::create_dir_all(&root).unwrap();
        let path = root.join(format!("open-{gap}.fieldcraft"));
        s.execute("file.save_as", &json!({"path":path})).unwrap();
        let mut reopened = Session::default();
        reopened.execute("file.open", &json!({"path":path})).unwrap();
        assert_eq!(s.model.step("Modes"), reopened.model.step("Modes"));
        assert_eq!(s.result("Modes"), reopened.result("Modes"));
        assert_eq!(s.result("Preload"), reopened.result("Preload"));
        let info = reopened.execute("model.info", &json!({})).unwrap();
        assert_eq!(info["contact_policy_capabilities"]["steps"]["Modes"]["selection"], "explicit_open");
        assert_eq!(info["contact_policy_capabilities"]["steps"]["Modes"]["runtime_qualification"], "unverified");
        if evidence.is_some() {
            for (name, value) in [
                ("base", serde_json::to_value(&base).unwrap()),
                ("modes", serde_json::to_value(&modes).unwrap()),
                ("base-command", base_reply),
                ("modes-command", mode_reply),
            ] {
                std::fs::write(root.join(format!("open-{gap}-{name}.json")), serde_json::to_vec_pretty(&value).unwrap()).unwrap();
            }
        } else {
            std::fs::remove_file(path).unwrap();
        }
        for error in [
            fieldcraft_decks::abaqus::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string(),
            fieldcraft_decks::nastran::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string(),
            fieldcraft_decks::lsdyna::write(&s.model, fieldcraft_model::units::Preset::MKgS).unwrap_err().to_string(),
        ] {
            eprintln!("OPEN_EXPORT_REFUSAL {error}");
            assert!(error.contains("contact_perturbation"), "{error}");
        }
    }
}

#[test]
fn open_refuses_touching_loaded_and_unqualified_geometry() {
    for (gap, compression, expected) in [(1e-6, 0.0, "normal_switch_or_not_open"), (0.0, 0.01, "normal_switch_or_not_open")] {
        let mut s = specimen(gap, compression);
        // The compressed positive-force base is frictionless to isolate opening.
        let mut c = s.model.contact("Interface").unwrap().clone();
        c.friction = fieldcraft_model::Friction::NONE;
        s.model.replace_contact("Interface", c).unwrap();
        s.execute("solve.nonlinear", &json!({"step":"Preload"})).unwrap();
        let error = s.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
        eprintln!("OPEN_REFUSAL gap={gap} compression={compression} {error}");
        assert!(error.contains(expected), "{error}");
        assert!(s.result("Modes").is_none());
    }
    for (kind, expected) in [("outside", "clipping_branch_unqualified"), ("tilted", "open_requires_planar_pair")] {
        let mut s = specimen(0.02, 0.0);
        for label in 9..=16 {
            let mut x = s.model.node(label).unwrap().xyz;
            if kind == "outside" {
                x[0] += 3.0;
            } else {
                x[2] += 0.001 * x[0];
            }
            s.model.set_node(label, x).unwrap();
        }
        let error = s.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
        eprintln!("OPEN_GEOMETRY_REFUSAL {kind}: {error}");
        assert!(error.contains(expected), "{error}");
        assert!(s.result("Modes").is_none());
    }
}
