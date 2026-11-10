//! Two original quadratic solids with plane-section guides. Their axial/shear
//! pencils are independently integrated one-dimensional quadratic bars, not saved
//! FE acceptance values. Both contact faces can move in the perturbation.
use fieldcraft_engine::{Session, StepResult};
use fieldcraft_solver::StepResult as Implicit;
use serde_json::{Value, json};

fn specimen(open: bool, compression: f64) -> (Session, Value) {
    let mut commands = Vec::new();
    let mut add = |command: &str, params: Value| commands.push(json!({"command":command,"params":params}));
    add("model.units", json!({"system":"m-kg-s-N-Pa"}));
    let corners = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    let edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)];
    let mut supports = Vec::new();
    for block in 0..2 {
        let mut xyz = corners.to_vec();
        for (a, b) in edges {
            xyz.push(std::array::from_fn(|k| (corners[a][k] + corners[b][k]) / 2.));
        }
        let mut layers: [Vec<usize>; 3] = Default::default();
        for (i, p) in xyz.iter().enumerate() {
            let layer = (2. * p[2]) as usize;
            let label = 20 * block + i + 1;
            layers[layer].push(label);
            let width = if open && block == 0 { 1.5 } else { 1. };
            add(
                "mesh.node.add",
                json!({"label":label,"xyz":[width*(p[0]-0.5)+0.5,width*(p[1]-0.5)+0.5,p[2]+block as f64+if open && block==1 {0.02} else {0.}]}),
            );
        }
        add("mesh.element.add", json!({"label":block+1,"kind":"hex20","nodes":(20*block+1..=20*block+20).collect::<Vec<_>>()}));
        for (layer, nodes) in layers.iter().enumerate() {
            let name = format!("Plane{block}{layer}");
            let rp = 100 + 3 * block + layer;
            add("model.set.create", json!({"name":name,"nodes":nodes}));
            add(
                "model.reference_point.create",
                json!({"name":name,"label":rp,"xyz":[0.5,0.5,block as f64+layer as f64/2.+if open&&block==1 {0.02} else {0.}]}),
            );
            add("model.coupling.create", json!({"name":name,"reference_point":name,"set":name,"kind":"kinematic","dofs":[1,2,3]}));
            let end = (block == 0 && layer == 0) || (block == 1 && layer == 2);
            let uz = if end { Some(if block == 1 { -compression } else { 0. }) } else { None };
            add(
                "model.boundary_condition.create",
                json!({"name":name,"type":"displacement","target":{"reference_point":name},"values":[if end {Some(0.)} else {None},0,uz,0,0,0]}),
            );
            supports.push(name);
        }
    }
    add("model.set.create", json!({"name":"all","elements":[1,2]}));
    add("model.set.create", json!({"name":"main","faces":[{"element":1,"face":2}]}));
    add("model.set.create", json!({"name":"secondary","faces":[{"element":2,"face":1}]}));
    add("model.material.create", json!({"name":"Elastic","elastic":{"type":"isotropic","young":1000,"poisson":0},"density":1}));
    add("model.section.create", json!({"name":"Solid","set":"all","material":"Elastic","kind":{"type":"solid"}}));
    add("interaction.contact.pair", json!({"name":"Interface","main":"main","secondary":"secondary","friction":{"static":0.3}}));
    add(
        "model.step.create",
        json!({"name":"Preload","procedure":{"type":"nonlinear_static","nonlinear_geometry":true,"initial_increment":1,"max_increment":1,"tolerances":{"force":1e-9,"displacement":1e-8,"energy":1e-14}},"boundary_conditions":supports}),
    );
    add(
        "model.step.create",
        json!({"name":"Modes","procedure":{"type":"frequency","modes":if open {8} else {6}},"base_state":"Preload","boundary_conditions":supports,"contact_perturbation":{"Interface":if open {"open"} else {"stick"}}}),
    );
    let script = json!(commands);
    let mut session = Session::default();
    session.run_script(&script).unwrap();
    (session, script)
}

#[test]
fn quadratic_moving_main_closed_forms_and_persistence() {
    for (open, compression) in [(false, 1e-4), (false, 2e-4), (true, 0.)] {
        let (mut s, script) = specimen(open, compression);
        let base_reply = s.execute("solve.nonlinear", &json!({"step":"Preload","threads":1})).unwrap();
        let mode_reply = s.execute("solve.frequency", &json!({"step":"Modes","threads":1})).unwrap();
        let StepResult::Implicit(Implicit::Static(base)) = s.result("Preload").unwrap().as_ref().clone() else { panic!("base") };
        let StepResult::Implicit(Implicit::Frequency(modes)) = s.result("Modes").unwrap().as_ref().clone() else { panic!("modes") };
        let at = |node| base.nodes.binary_search(&node).unwrap();
        let stretch = 1. + base.displacement[at(5)][2] - base.displacement[at(1)][2];
        let upper = 1. + base.displacement[at(25)][2] - base.displacement[at(21)][2];
        assert!((upper - stretch).abs() < 1e-10);
        // Reference quadratic element: K=k/3*[7,-8,1;-8,16,-8;1,-8,7],
        // M=rho/30*[4,2,-1;2,16,2;-1,2,4], length/area/density=1.
        // Two bonded elements with fixed ends have roots 10 and (52±8√31)/3.
        // An open, fixed-free element has the latter two roots; the width cancels.
        let lo = (52. - 8. * 31_f64.sqrt()) / 3.;
        let hi = (52. + 8. * 31_f64.sqrt()) / 3.;
        let roots = if open { vec![lo, lo, hi, hi] } else { vec![lo, 10., hi] };
        for (axis, tangent) in [(0, 500. * stretch * stretch), (2, 500. * (3. * stretch * stretch - 1.))] {
            let mut found = Vec::new();
            for (i, m) in modes.modes.iter().enumerate() {
                let norm = m.shape.iter().flat_map(|u| u.iter()).map(|v| v * v).sum::<f64>().sqrt();
                let other = if axis == 0 { 2 } else { 0 };
                if m.shape.iter().all(|u| u[other].abs() < 1e-7 * norm && u[1].abs() < 1e-7 * norm) {
                    found.push((i, m));
                }
            }
            assert_eq!(found.len(), roots.len(), "pair by components, including repeated roots");
            for ((i, m), root) in found.iter().zip(&roots) {
                let expected = (tangent * root).sqrt() / std::f64::consts::TAU;
                let error = (m.frequency / expected - 1.).abs();
                let residual = mode_reply["modes"][*i]["equation_residual"]["relative_residual"].as_f64().unwrap();
                eprintln!(
                    "QUADRATIC_MODE open={open} compression={compression} axis={axis} mode={} f={:.15e} analytic={expected:.15e} relative={error:.5e} residual={residual:.5e}",
                    i + 1,
                    m.frequency
                );
                assert!(error < 1e-6 && residual < 1e-8);
            }
        }
        let history = base.nonlinear.as_ref().unwrap();
        assert!(history.increments.iter().all(|i| i.converged));
        assert_eq!(history.increments.last().unwrap().load_factor, 1.);
        let contact = &base.contacts[0];
        if open {
            assert_eq!(contact.normal_force, 0.);
        } else {
            let force = -500. * stretch * (stretch * stretch - 1.);
            assert!((contact.normal_force / force - 1.).abs() < 1e-6);
            assert!(contact.pressure.iter().all(|v| *v > 0.));
        }
        assert!(base.reaction_total.iter().all(|r| r.abs() < 1e-8));
        assert!(modes.modes.iter().any(|m| m.shape[at(5)].iter().any(|v| v.abs() > 1e-3)), "main must actually move");
        if !open {
            for m in &modes.modes {
                for (a, b) in [(5, 21), (6, 22), (7, 23), (8, 24), (13, 29), (14, 30), (15, 31), (16, 32)] {
                    for k in 0..3 {
                        assert!((m.shape[at(a)][k] - m.shape[at(b)][k]).abs() < 1e-10);
                    }
                }
            }
        }
        eprintln!(
            "QUADRATIC_BASE open={open} force={:.15e} reaction={:?} iterations={} last={:?}",
            contact.normal_force,
            base.reaction_total,
            history.iterations,
            history.increments.last().unwrap().iterations.last()
        );
        let mut legacy = s.clone();
        legacy.execute("model.step.edit", &json!({"name":"Modes","contact_perturbation":{}})).unwrap();
        let old_reply = legacy.execute("solve.frequency", &json!({"step":"Modes","threads":1})).unwrap();
        let StepResult::Implicit(Implicit::Frequency(old)) = legacy.result("Modes").unwrap().as_ref() else { panic!("legacy") };
        assert_eq!(old.modes, modes.modes);
        assert_eq!(old_reply["modes"], mode_reply["modes"]);
        s.execute("solve.frequency", &json!({"step":"Modes","threads":4})).unwrap();
        assert_eq!(s.result("Modes").unwrap().as_ref(), &StepResult::Implicit(Implicit::Frequency(modes.clone())));
        let evidence = std::env::var_os("FIELDCRAFT_QUADRATIC_EVIDENCE").map(std::path::PathBuf::from);
        let root = evidence.clone().unwrap_or_else(|| std::env::temp_dir().join(format!("quadratic-policy-{}", std::process::id())));
        std::fs::create_dir_all(&root).unwrap();
        let stem = format!("quadratic-{open}-{compression}");
        let path = root.join(format!("{stem}.fieldcraft"));
        s.execute("file.save_as", &json!({"path":path})).unwrap();
        let mut reopened = Session::default();
        reopened.execute("file.open", &json!({"path":path})).unwrap();
        assert_eq!(s.model.step("Modes"), reopened.model.step("Modes"));
        assert_eq!(s.result("Modes"), reopened.result("Modes"));
        assert_eq!(s.result("Preload"), reopened.result("Preload"));
        if evidence.is_some() {
            for (tag, v) in [
                ("input", script),
                ("base", serde_json::to_value(base).unwrap()),
                ("modes", serde_json::to_value(modes).unwrap()),
                ("base-reply", base_reply),
                ("modes-reply", mode_reply),
            ] {
                std::fs::write(root.join(format!("{stem}-{tag}.json")), serde_json::to_vec_pretty(&v).unwrap()).unwrap();
            }
        } else {
            std::fs::remove_file(path).unwrap();
        }
    }
}

#[test]
fn quadratic_policy_refuses_unqualified_states_and_second_stage() {
    for (open, policy, expected) in [
        (true, "stick", "base_evidence_missing"),
        (false, "open", "clipping_branch_unqualified"),
        (false, "closed_tangent_free", "closed_tangent_free_requires_linear_solid_faces"),
    ] {
        let (mut s, _) = specimen(open, if open { 0. } else { 1e-4 });
        s.execute("model.step.edit", &json!({"name":"Modes","contact_perturbation":{"Interface":policy}})).unwrap();
        let error = s.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
        eprintln!("QUADRATIC_REFUSAL policy={policy} base_open={open} {error}");
        assert!(error.contains(expected) || (open && error.contains("normal_switch")), "{error}");
        assert!(s.result("Modes").is_none());
    }
    let (mut s, _) = specimen(true, 0.);
    let mut x = s.model.node(13).unwrap().xyz;
    x[0] += 0.01;
    s.model.set_node(13, x).unwrap();
    let error = s.execute("solve.frequency", &json!({"step":"Modes"})).unwrap_err().to_string();
    eprintln!("QUADRATIC_GEOMETRY_REFUSAL {error}");
    assert!(error.contains("open_quadratic_geometry_unqualified"), "{error}");
}
