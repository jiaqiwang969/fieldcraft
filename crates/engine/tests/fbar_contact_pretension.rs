//! A coupled extension of the existing axial pretension and hard-stick patches.
//! The bolt mid-plane and pad top share ONE vibration coordinate, so both
//! terminal stiffnesses and all three moving cubes contribute to the same mode.
//! Axial neo-Hookean theory: solver/tests/internal_pretension_chain.rs.
//! StVK shear theory: contact_perturbation.rs, extended to nonzero base shear.
use fieldcraft_engine::{Session, StepResult};
use fieldcraft_model::{ContactPerturbation, MassKind, PretensionControl};
use fieldcraft_solver::{FrequencyResult, StaticResult, StepResult as Implicit};
use serde_json::{Value, json};

fn prefix_pad(value: &mut Value) {
    match value {
        Value::String(s)
            if ["base", "upper", "top", "all", "main", "secondary", "Elastic", "Solid", "Lateral", "Interface"].contains(&s.as_str()) =>
        {
            *s = format!("Pad{s}");
        }
        Value::Array(items) => items.iter_mut().for_each(prefix_pad),
        Value::Object(fields) => fields.values_mut().for_each(prefix_pad),
        _ => {}
    }
}

fn specimen(compression: f64) -> Value {
    let bolt: Value = serde_json::from_str(include_str!("fixtures/fbar_pretension.json")).unwrap();
    let pad: Value = serde_json::from_str(include_str!("fixtures/contact-stick.json")).unwrap();
    let mut commands: Vec<Value> = bolt.as_array().unwrap().iter().take_while(|c| c["command"] != "model.step.create").cloned().collect();
    for mut c in pad.as_array().unwrap().iter().take_while(|c| c["command"] != "model.step.create").cloned() {
        let command = c["command"].as_str().unwrap().to_owned();
        let p = &mut c["params"];
        match command.as_str() {
            "model.units" => continue,
            "mesh.node.add" => {
                let label = p["label"].as_u64().unwrap();
                let mut x = p["xyz"][0].as_f64().unwrap();
                let y = p["xyz"][1].as_f64().unwrap();
                let z = p["xyz"][2].as_f64().unwrap();
                // Rotate about Y without reversing element orientation. The
                // fixed main face extends past both pad edges to retain full
                // overlap under the small elastic tangential base motion.
                if label <= 8 {
                    x = 1.5 * x - 0.25;
                }
                p["label"] = json!(label + 200);
                p["xyz"] = json!([4.0 + z, y, 2.0 - x]);
            }
            "mesh.element.add" => {
                p["label"] = json!(p["label"].as_u64().unwrap() + 200);
                for n in p["nodes"].as_array_mut().unwrap() {
                    *n = json!(n.as_u64().unwrap() + 200);
                }
            }
            "model.set.create" => {
                for key in ["nodes", "elements"] {
                    if let Some(items) = p.get_mut(key).and_then(Value::as_array_mut) {
                        for n in items {
                            *n = json!(n.as_u64().unwrap() + 200);
                        }
                    }
                }
                if let Some(faces) = p.get_mut("faces").and_then(Value::as_array_mut) {
                    for face in faces {
                        face["element"] = json!(face["element"].as_u64().unwrap() + 200);
                    }
                }
            }
            "model.boundary_condition.create" => {
                let v = p["values"].as_array().unwrap().clone();
                p["values"] = json!([v[2], v[1], v[0].as_f64().map(|x| -x), null, null, null]);
                if p["name"] == "top" {
                    p["values"][0] = json!(-compression);
                }
            }
            "model.spring.create" => p["direction"]["dof"] = json!(3),
            _ => {}
        }
        prefix_pad(p);
        commands.push(c);
    }
    commands.extend([
        json!({"command":"model.set.create","params":{"name":"PadFoot","nodes":[209,210,211,212]}}),
        json!({"command":"model.reference_point.create","params":{"name":"PadFace","label":300,"xyz":[5,0.5,1.5]}}),
        // Keep both pad faces planar for the closed-form patch. The interface
        // translations remain free in the BASE and are removed by hard stick
        // only in the perturbation, not by prescribed support conditions.
        json!({"command":"model.coupling.create","params":{"name":"PadPlane","reference_point":"PadFace","set":"PadFoot","kind":"kinematic"}}),
        json!({"command":"model.boundary_condition.create","params":{"name":"PadGuide","type":"displacement","target":{"reference_point":"PadFace"},"values":[null,0,null,0,0,0]}}),
        json!({"command":"model.coupling.create","params":{"name":"SharedMotion","reference_point":"Motion","set":"Padtop","kind":"kinematic","dofs":[3]}}),
    ]);
    let supports = json!(["Ends", "Guide", "Padbase", "Padupper", "Padtop", "PadGuide"]);
    let procedure = pad.as_array().unwrap().iter().find(|c| c["params"]["name"] == "Preload").unwrap()["params"]["procedure"].clone();
    // Single-increment settings and all three tolerances are the existing
    // contact patch's; no convergence threshold is relaxed for the combination.
    for name in ["Preload", "Locked"] {
        commands.push(json!({"command":"model.step.create","params":{"name":name,"procedure":procedure,"boundary_conditions":supports}}));
    }
    commands.push(json!({"command":"model.pretension.force","params":{"name":"Cut","step":"Preload","value":1000}}));
    commands.push(json!({"command":"model.step.create","params":{"name":"Modes","procedure":{"type":"frequency","modes":1},"base_state":"Locked","contact_perturbation":{"PadInterface":"stick"},"boundary_conditions":supports}}));
    Value::Array(commands)
}

fn static_result(s: &Session, name: &str) -> StaticResult {
    let StepResult::Implicit(Implicit::Static(result)) = s.result(name).unwrap().as_ref() else { panic!("static result {name}") };
    result.clone()
}

fn modes(s: &Session) -> FrequencyResult {
    let StepResult::Implicit(Implicit::Frequency(result)) = s.result("Modes").unwrap().as_ref() else { panic!("frequency result") };
    result.clone()
}

// The independent uniform neo-Hookean axial law already used by the bolt test.
fn neo_axial(stretch: f64) -> (f64, f64) {
    let (mu, bulk) = (5e5, 1e6 / 3.0);
    (
        (2.0 * mu / 3.0) * (stretch.powf(1.0 / 3.0) - stretch.powf(-5.0 / 3.0)) + bulk * (stretch - 1.0),
        (2.0 * mu / 9.0) * (stretch.powf(-2.0 / 3.0) + 5.0 * stretch.powf(-8.0 / 3.0)) + bulk,
    )
}

fn record(root: &Option<std::path::PathBuf>, name: &str, value: &Value) {
    if let Some(root) = root {
        std::fs::create_dir_all(root).unwrap();
        std::fs::write(root.join(name), serde_json::to_vec_pretty(value).unwrap()).unwrap();
    }
}

#[test]
fn fbar_bolt_and_explicit_stick_share_locked_equilibrium_and_mode() {
    let evidence = std::env::var_os("FIELDCRAFT_JOINT_EVIDENCE").map(std::path::PathBuf::from);
    for compression in [0.01, 0.02] {
        let script = specimen(compression);
        record(&evidence, &format!("input-{compression}.json"), &script);
        let mut s = Session::default();
        s.run_script(&script).unwrap();
        assert!(s.model.section("S").unwrap().formulation.f_bar);
        let step = s.model.step("Modes").unwrap();
        assert_eq!(step.base_state.as_deref(), Some("Locked"));
        assert_eq!(step.contact_perturbation.get("PadInterface"), Some(&ContactPerturbation::Stick));
        assert_eq!(step.boundary_conditions, s.model.step("Locked").unwrap().boundary_conditions);

        let preload_reply = s.execute("solve.nonlinear", &json!({"step":"Preload","threads":1})).unwrap();
        let preload = static_result(&s, "Preload");
        record(&evidence, &format!("preload-{compression}.json"), &json!({"command":preload_reply,"result":preload}));
        let lock_reply = s.execute("model.pretension.lock", &json!({"name":"Cut","step":"Locked","from":"Preload"})).unwrap();
        let Some(PretensionControl::Adjustment { value }) = s.model.step("Locked").unwrap().pretension.get("Cut") else {
            panic!("locked adjustment")
        };
        assert_eq!(value.to_bits(), preload.pretension[0].adjustment.to_bits());
        let locked_reply = s.execute("solve.nonlinear", &json!({"step":"Locked","threads":1})).unwrap();
        let locked = static_result(&s, "Locked");
        record(&evidence, &format!("locked-{compression}.json"), &json!({"lock":lock_reply,"command":locked_reply,"result":locked}));
        assert_eq!(locked.pretension[0].adjustment.to_bits(), preload.pretension[0].adjustment.to_bits());
        for base in [&preload, &locked] {
            let history = base.nonlinear.as_ref().unwrap();
            assert!(history.increments.last().unwrap().converged);
            assert_eq!(history.increments.last().unwrap().load_factor, 1.0);
            assert!((base.pretension[0].section_force - 1000.0).abs() < 1e-6);
            assert_eq!(base.contacts.len(), 1);
            assert!(base.contacts[0].normal_force > 0.0);
            assert!(base.contacts[0].status.iter().all(|status| *status == fieldcraft_solver::contact::Status::Stick));
        }
        let at = |label| locked.nodes.binary_search(&label).unwrap();
        let u = |label, axis| locked.displacement[at(label)][axis];
        let lower = 1.0 + u(5, 2) - u(1, 2);
        let upper = 1.0 + u(9, 2) - u(13, 2);
        let normal_stretch = 1.0 + u(213, 0) - u(209, 0);
        let shear = u(213, 2) - u(209, 2);
        assert!(shear.abs() > 1e-5, "pad actually receives bolt preload through shared Motion");
        assert_eq!(u(213, 2).to_bits(), u(5, 2).to_bits());
        let (lower_force, lower_tangent) = neo_axial(lower);
        let (upper_force, upper_tangent) = neo_axial(upper);
        // For StVK with nu=0: W = G[gamma^2/2 +
        // (gamma^2 + lambda^2 - 1)^2/4]. Its second shear derivative
        // is G(lambda^2 + 3 gamma^2). Four real 1 N/m springs remain.
        let pad_force = 500.0 * shear * (normal_stretch.powi(2) + shear.powi(2));
        let pad_tangent = 500.0 * (normal_stretch.powi(2) + 3.0 * shear.powi(2)) + 4.0;
        let normal_force = -500.0 * normal_stretch * (normal_stretch.powi(2) + shear.powi(2) - 1.0);
        assert!((upper_force - 1000.0).abs() < 1e-6);
        assert!((lower_force - upper_force + pad_force + 4.0 * u(5, 2)).abs() < 1e-6);
        assert!((locked.contacts[0].normal_force / normal_force - 1.0).abs() < 1e-6);
        let stiffness = lower_tangent + upper_tangent + pad_tangent;
        // Each unit moving cube has reference consistent mass rho*A*H/3.
        // The rigid main block is fixed, and reference points are massless.
        let mass = 2.0 / 3.0 + 1.0 / 3.0;
        let expected = stiffness / mass;
        assert!((expected / (2e6 + pad_tangent) - 1.0).abs() > 1e-4, "finite-strain bolt tangent is not the ordinary linear seed");

        let reply = s.execute("solve.frequency", &json!({"step":"Modes","threads":1})).unwrap();
        let result = modes(&s);
        let relative = (result.modes[0].eigenvalue / expected - 1.0).abs();
        record(
            &evidence,
            &format!("modes-{compression}.json"),
            &json!({"command":reply,"result":result,"reference":{"lower_stretch":lower,"upper_stretch":upper,"pad_normal_stretch":normal_stretch,"pad_shear":shear,"bolt_tangents":[lower_tangent,upper_tangent],"pad_tangent":pad_tangent,"mass":mass,"eigenvalue":expected,"relative_error":relative}}),
        );
        assert!(relative < 1e-9, "coupled eigenvalue relative error {relative:e}");
        assert_eq!(result.mass_kind, Some(MassKind::Consistent));
        assert_eq!(result.stats.equations, 1);
        assert!(result.warnings.iter().any(|w| w.contains("linear perturbation about `Locked`: converged consistent tangent")));
        assert!(result.warnings.iter().any(|w| w.contains("contact `PadInterface` policy=stick state=closed_stick")));
        assert!(!result.warnings.iter().any(|w| w.contains("legacy_bonded_when_closed")));
        assert_eq!(static_result(&s, "Locked"), locked, "modal solve must not replace the stored equilibrium");
        assert_eq!(result.nodes, locked.nodes);
        let shape = &result.modes[0].shape;
        for label in 209..=212 {
            assert!(shape[at(label)].iter().all(|component| component.abs() < 1e-12), "stick removes every interface increment");
        }
        let motion = shape[at(5)][2];
        for label in [5, 6, 7, 8, 13, 14, 15, 16, 213, 214, 215, 216] {
            assert!((shape[at(label)][2] - motion).abs() < 1e-8, "one shared mode coordinate at {label}");
        }
        assert!((mass * motion.powi(2) - 1.0).abs() < 1e-9, "independent mass normalization");
        s.execute("solve.frequency", &json!({"step":"Modes","threads":4})).unwrap();
        assert_eq!(modes(&s), result, "whole modal result is thread independent");
        s.execute("solve.nonlinear", &json!({"step":"Locked","threads":1})).unwrap();
        assert_eq!(static_result(&s, "Locked"), locked, "the named equilibrium is exactly reproducible");
        assert_eq!(modes(&s), result, "the modal result remains present before saving");
        eprintln!(
            "FBAR_STICK compression={compression} lower={lower:.12e} upper={upper:.12e} pad_shear={shear:.12e} f={:.12e} eigen_relative={relative:.3e}",
            result.modes[0].frequency
        );

        let root = evidence.clone().unwrap_or_else(|| std::env::temp_dir().join(format!("fbar-stick-{}", std::process::id())));
        std::fs::create_dir_all(&root).unwrap();
        let path = root.join(format!("joint-{compression}.fieldcraft"));
        s.execute("file.save_as", &json!({"path":path})).unwrap();
        let mut reopened = Session::default();
        reopened.execute("file.open", &json!({"path":path})).unwrap();
        assert_eq!(reopened.model.step("Modes"), s.model.step("Modes"));
        assert!(reopened.model.section("S").unwrap().formulation.f_bar);
        assert_eq!(static_result(&reopened, "Locked"), locked);
        assert_eq!(modes(&reopened), result);
        if evidence.is_none() {
            std::fs::remove_file(path).unwrap();
        }
    }
}
