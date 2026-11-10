//! Independent uniform axial patch with one vibration coordinate.
//! Two unit cubes, E=1e6, nu=0, rho=1: m=2/3, K=2(E+F) for a
//! linear prestress base; finite StVK gives K=E(3 lambda^2-1).
//! Bathe (2014), sections 6.8.2 and 9.2: tangent perturbations and consistent mass.
use fieldcraft_model::harmonic::{Damping, Frequencies, Harmonic, Method};
use fieldcraft_model::*;
use fieldcraft_solver::{FrequencyOptions, Limits, StaticResult, solve_frequency, solve_harmonic, solve_nonlinear, solve_static};

fn chain(force: f64, finite: bool) -> Model {
    let mut m = Model::default();
    for z in 0..3u32 {
        for (i, [x, y]) in [[0., 0.], [1., 0.], [1., 1.], [0., 1.]].into_iter().enumerate() {
            m.set_node(4 * z + i as u32 + 1, [x, y, f64::from(z)]).unwrap();
        }
    }
    m.set_element(1, ElementKind::Hex8, (1..=8).collect()).unwrap();
    m.set_element(2, ElementKind::Hex8, (5..=12).collect()).unwrap();
    m.add_set("All", Set { elements: vec![1, 2], ..Set::default() }).unwrap();
    m.add_set("Ends", Set { nodes: vec![1, 2, 3, 4, 9, 10, 11, 12], ..Set::default() }).unwrap();
    m.add_set("Mid", Set { nodes: (5..=8).collect(), ..Set::default() }).unwrap();
    m.add_set("CutFace", Set { faces: vec![ElementFace { element: 1, face: 2 }], ..Set::default() }).unwrap();
    m.add_material("Elastic", Material { elastic: Some(Elastic::Isotropic { young: 1e6, poisson: 0. }), density: Some(1.), ..Material::default() })
        .unwrap();
    m.add_section(
        "Solid",
        Section {
            set: "All".into(),
            material: "Elastic".into(),
            kind: SectionKind::Solid {},
            orientation: None,
            formulation: Formulation::default(),
        },
    )
    .unwrap();
    for (name, label) in [("Control", 100), ("Motion", 101)] {
        m.add_reference_point(name, ReferencePoint { xyz: [0.5, 0.5, 1.], label }).unwrap();
    }
    m.add_coupling("Plane", Coupling { reference_point: "Motion".into(), set: "Mid".into(), kind: CouplingKind::Kinematic, dofs: Dofs::all() })
        .unwrap();
    m.add_pretension(
        "Cut",
        PretensionSection {
            first: "Mid".into(),
            second: "Ends".into(),
            face: Some("CutFace".into()),
            normal: [0., 0., 1.],
            control: "Control".into(),
        },
    )
    .unwrap();
    m.add_boundary_condition("Ends", BoundaryCondition::Fixed { target: Target::Set("Ends".into()) }).unwrap();
    m.add_boundary_condition(
        "Guide",
        BoundaryCondition::Displacement {
            target: Target::ReferencePoint("Motion".into()),
            values: [Some(0.), Some(0.), None, Some(0.), Some(0.), Some(0.)],
        },
    )
    .unwrap();
    m.add_load("Probe", Load::Force { target: Target::ReferencePoint("Motion".into()), vector: [0., 0., 1.], distribution: Default::default() })
        .unwrap();
    let procedure = if finite {
        Procedure::NonlinearStatic(NonlinearStatic { nonlinear_geometry: true, ..NonlinearStatic::default() })
    } else {
        Procedure::Static {}
    };
    let mut preload = Step::new("Preload", procedure.clone());
    preload.pretension.insert("Cut".into(), PretensionControl::Force { value: force });
    let mut locked = Step::new("Locked", procedure);
    locked.pretension.insert("Cut".into(), PretensionControl::Fixed);
    let mut modes = Step::new("Modes", Procedure::Frequency { modes: 1, min_hz: None, max_hz: None });
    modes.base_state = Some("Locked".into());
    let mut response = Step::new(
        "Response",
        Procedure::Harmonic(Harmonic {
            frequencies: Frequencies::List(vec![100., 250., 300.]),
            method: Method::Direct,
            damping: Damping { alpha: 2., ..Damping::default() },
            load_factors: Default::default(),
        }),
    );
    response.base_state = Some("Locked".into());
    response.loads.insert("Probe".into());
    for mut step in [preload, locked, modes, response] {
        step.boundary_conditions.extend(["Ends".into(), "Guide".into()]);
        m.add_step(step, None).unwrap();
    }
    m
}

fn equilibrium(m: &Model, name: &str, finite: bool) -> StaticResult {
    if finite { solve_nonlinear(m, Some(name), &Limits::default()).unwrap() } else { solve_static(m, Some(name), &Limits::default()).unwrap() }
}

fn energy(r: &StaticResult, finite: bool) -> f64 {
    if finite {
        let stretch = 1. + r.pretension[0].adjustment / 2.;
        0.25e6 * (stretch * stretch - 1.).powi(2)
    } else {
        r.elements.iter().flat_map(|e| &e.points).map(|p| 0.5 * p.stress.iter().zip(p.strain).map(|(s, e)| s * e).sum::<f64>() / 8.).sum()
    }
}

#[test]
fn fixed_uses_this_equilibrium_and_preserves_force_reaction_and_energy() {
    for finite in [false, true] {
        let m = chain(1000., finite);
        let source = m.clone();
        let pre = equilibrium(&m, "Preload", finite);
        let locked = equilibrium(&m, "Locked", finite);
        let q = pre.pretension[0].adjustment;
        assert_eq!(q.to_bits(), locked.pretension[0].adjustment.to_bits());
        assert!((locked.pretension[0].section_force / 1000. - 1.).abs() < 1e-9);
        assert_eq!(locked.pretension[0].applied_force, 0.);
        assert!((energy(&locked, finite) / energy(&pre, finite) - 1.).abs() < 1e-10);
        for (a, b) in pre.reaction.iter().flatten().zip(locked.reaction.iter().flatten()) {
            assert!((a - b).abs() < 1e-6);
        }
        if finite {
            let stretch = 1. + q / 2.;
            assert!((0.5e6 * stretch * (stretch * stretch - 1.) / 1000. - 1.).abs() < 1e-9);
            for r in [&pre, &locked] {
                let h = r.nonlinear.as_ref().unwrap();
                assert!(h.increments.last().unwrap().converged);
                assert_eq!(h.increments.last().unwrap().load_factor, 1.);
                assert!(h.iterations > 0);
            }
        }
        assert_eq!(m, source);
        println!(
            "finite={finite} q={q:.17e} force={:.17e} U_pre={:.17e} U_locked={:.17e}",
            locked.pretension[0].section_force,
            energy(&pre, finite),
            energy(&locked, finite)
        );
    }
}

#[test]
fn locked_cut_frequency_and_direct_modal_response_match_independent_uniform_tangent() {
    for finite in [false, true] {
        for force in [0., 1000.] {
            let mut m = chain(force, finite);
            let locked = equilibrium(&m, "Locked", finite);
            let q = locked.pretension[0].adjustment;
            let k = if finite { 1e6 * (3. * (1. + q / 2.).powi(2) - 1.) } else { 2. * (1e6 + force) };
            let mass = 2. / 3.;
            let f = solve_frequency(&m, Some("Modes"), &FrequencyOptions::default(), &Limits::default()).unwrap();
            assert_eq!(f.nodes, (1..=12).collect::<Vec<_>>());
            assert_eq!(f.stats.equations, 1);
            assert!((f.total_mass / 2. - 1.).abs() < 1e-12);
            assert!(
                (f.modes[0].eigenvalue / (k / mass) - 1.).abs() < 1e-9,
                "finite={finite} force={force} lambda={} reference={}",
                f.modes[0].eigenvalue,
                k / mass
            );
            for modal in [false, true] {
                if modal {
                    let mut step = m.step("Response").unwrap().clone();
                    let Procedure::Harmonic(h) = &mut step.procedure else { panic!("harmonic") };
                    h.method = Method::Modal { modes: 1, damping_ratios: vec![], residual_vectors: 0 };
                    m.replace_step(step).unwrap();
                }
                let r = solve_harmonic(&m, Some("Response"), &Limits::default()).unwrap();
                assert_eq!(r.nodes, f.nodes);
                let middle = r.nodes.iter().position(|n| *n == 5).unwrap();
                for frame in &r.frames {
                    let omega = 2. * std::f64::consts::PI * frame.frequency;
                    let a = k - omega * omega * mass;
                    let b = omega * 2. * mass;
                    let re = a / (a * a + b * b);
                    let im = -b / (a * a + b * b);
                    assert!((frame.real[middle][2] / re - 1.).abs() < 1e-8, "real {:?} expected {re}", frame.real[middle]);
                    assert!((frame.imaginary[middle][2] / im - 1.).abs() < 1e-8);
                    assert!(frame.relative_residual < 1e-9);
                    assert!((frame.input_power - frame.dissipated_power).abs() < 1e-12);
                }
                println!(
                    "finite={finite} force={force} modal={modal} f={:.17e} K={k:.17e} M={mass:.17e} residuals={:?}",
                    f.modes[0].frequency,
                    r.frames.iter().map(|p| p.relative_residual).collect::<Vec<_>>()
                );
            }
        }
    }
}

#[test]
fn fixed_without_a_predecessor_is_rejected() {
    let mut m = chain(1000., false);
    let mut step = m.step("Preload").unwrap().clone();
    step.pretension.insert("Cut".into(), PretensionControl::Fixed);
    m.replace_step(step).unwrap();
    let error = solve_static(&m, Some("Locked"), &Limits::default()).unwrap_err().to_string();
    assert!(error.contains("preceding static equilibrium"), "{error}");
}

#[test]
fn supported_continuous_cut_keeps_fixed_equilibrium_and_terminal_stiffness() {
    // The positive cut trace rests on a unit-area reference foundation.
    // For extensions y below and x above the cut: P(y)=F, P(x)+k_f x=F,
    // q=x+y, K_perturb=P'(y)+P'(x)+k_f, with P from the StVK bar law.
    let foundation = 2e5;
    for force in [0., 1000.] {
        let mut m = chain(force, true);

        m.add_set("SupportedTrace", Set { faces: vec![ElementFace { element: 2, face: 1 }], ..Set::default() }).unwrap();
        m.add_foundation(
            "Bed",
            Foundation {
                set: "SupportedTrace".into(),
                normal: foundation,
                tangential: 0.,
                normal_damping: 0.,
                tangential_damping: 0.,
                loss_factor: 0.,
            },
        )
        .unwrap();
        let source = m.clone();
        let extension = |support: f64| {
            let mut x = force / (1e6 + support);
            for _ in 0..8 {
                let stretch = 1. + x;
                x -= (0.5e6 * stretch * (stretch * stretch - 1.) + support * x - force) / (0.5e6 * (3. * stretch * stretch - 1.) + support);
            }
            x
        };
        let (x, y) = (extension(foundation), extension(0.));
        let pre = equilibrium(&m, "Preload", true);
        let locked = equilibrium(&m, "Locked", true);
        assert_eq!(pre.pretension[0].adjustment.to_bits(), locked.pretension[0].adjustment.to_bits());
        for r in [&pre, &locked] {
            assert!((r.pretension[0].adjustment - x - y).abs() < 1e-11);
            assert!((r.pretension[0].section_force - force).abs() < 1e-6);
            let middle = r.nodes.binary_search(&5).unwrap();
            assert!((r.displacement[middle][2] + x).abs() < 1e-11);
            assert!((r.reaction_total[2] - foundation * r.displacement[middle][2]).abs() < 1e-6);
            assert!(r.nonlinear.as_ref().unwrap().increments.last().unwrap().converged);
        }
        let k = 0.5e6 * (3. * (1. + y).powi(2) - 1.) + 0.5e6 * (3. * (1. + x).powi(2) - 1.) + foundation;
        let f = solve_frequency(&m, Some("Modes"), &FrequencyOptions::default(), &Limits::default()).unwrap();
        assert!((f.modes[0].eigenvalue / (k / (2. / 3.)) - 1.).abs() < 1e-9);
        assert_eq!(m, source);
        println!("supported cut F={force} q={:.17e} eigenvalue={:.17e}", pre.pretension[0].adjustment, f.modes[0].eigenvalue);
    }
}

#[test]
fn zero_preload_degenerates_to_the_unsplit_frequency_and_response() {
    let cut = chain(0., false);
    let mut plain = cut.clone();
    for mut step in plain.steps().to_vec() {
        step.pretension.clear();
        step.base_state = None;
        plain.replace_step(step).unwrap();
    }
    plain.remove_pretension("Cut").unwrap();
    plain.remove_reference_point("Control").unwrap();
    let modes = |m: &Model| solve_frequency(m, Some("Modes"), &FrequencyOptions::default(), &Limits::default()).unwrap();
    let a = modes(&cut);
    let b = modes(&plain);
    assert_eq!(a.nodes, b.nodes);
    assert!((a.total_mass - b.total_mass).abs() < 1e-12);
    assert!((a.modes[0].eigenvalue / b.modes[0].eigenvalue - 1.).abs() < 1e-12);
    let a = solve_harmonic(&cut, Some("Response"), &Limits::default()).unwrap();
    let b = solve_harmonic(&plain, Some("Response"), &Limits::default()).unwrap();
    let mut error = 0.0_f64;
    for (a, b) in a.frames.iter().zip(&b.frames) {
        for (a, b) in a.real.iter().flatten().chain(a.imaginary.iter().flatten()).zip(b.real.iter().flatten().chain(b.imaginary.iter().flatten())) {
            error = error.max((a - b).abs());
        }
    }
    assert!(error < 1e-18, "zero preload field mismatch {error}");
    println!("zero preload versus unsplit response max absolute difference={error:.17e}");
}

fn fbar_chain(force: f64, enabled: bool) -> Model {
    let mut m = chain(force, true);
    // Explicit coincident cut pairs use the existing transient interface. Continuous
    // face preparation is supported for static/frequency/harmonic, not transient.
    for i in 0..4u32 {
        let x = m.node(5 + i).unwrap().xyz;
        m.set_node(13 + i, x).unwrap();
    }
    m.set_element(2, ElementKind::Hex8, vec![13, 14, 15, 16, 9, 10, 11, 12]).unwrap();
    m.add_set("OtherCut", Set { nodes: vec![13, 14, 15, 16], ..Default::default() }).unwrap();
    let mut cut = m.pretensions()["Cut"].clone();
    cut.face = None;
    cut.second = "OtherCut".into();
    m.replace_pretension("Cut", cut).unwrap();

    // A single increment avoids an extrapolated near-zero work reference in the
    // small-strain patch; retain the production Newton tolerances.
    for name in ["Preload", "Locked"] {
        let mut step = m.step(name).unwrap().clone();
        if let Procedure::NonlinearStatic(n) = &mut step.procedure {
            n.initial_increment = 1.;
            n.max_increment = 1.;
        }
        m.replace_step(step).unwrap();
    }
    let mut section = m.section("Solid").unwrap().clone();
    section.formulation.f_bar = enabled;
    m.replace_section("Solid", section).unwrap();
    let mut material = m.material("Elastic").unwrap().clone();
    material.hyperelastic = Some(Hyperelastic::NeoHookean {});
    m.replace_material("Elastic", material).unwrap();
    if enabled {
        let mut step = m.step("Response").unwrap().clone();
        step.output = [OutputVariable::Displacement].into();
        m.replace_step(step).unwrap();
    }
    for (name, modal, loaded) in [("Direct", false, true), ("Modal", true, true), ("Zero", false, false)] {
        let t: fieldcraft_model::dynamics::Transient = serde_json::from_value(serde_json::json!({"time_step":1e-5,"increments":10,"method":if modal {serde_json::json!({"type":"modal","modes":1})} else {serde_json::json!({"type":"direct"})}})).unwrap();
        let mut step = Step::new(name, Procedure::Transient(t));
        step.output = [OutputVariable::Displacement].into();
        step.base_state = Some("Locked".into());
        step.boundary_conditions.extend(["Ends".into(), "Guide".into()]);
        if loaded {
            step.loads.insert("Probe".into());
        }
        m.add_step(step, None).unwrap();
    }
    m
}
fn chain_evidence(name: &str, value: &serde_json::Value) {
    if let Some(root) = std::env::var_os("FIELDCRAFT_FBAR_CHAIN_EVIDENCE") {
        let root = std::path::PathBuf::from(root);
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(root.join(format!("{name}.json")), serde_json::to_vec_pretty(value).unwrap()).unwrap();
    }
}
fn neo_axial(l: f64) -> (f64, f64) {
    let (mu, bulk) = (5e5, 1e6 / 3.);
    ((2. * mu / 3.) * (l.powf(1. / 3.) - l.powf(-5. / 3.)) + bulk * (l - 1.), (2. * mu / 9.) * (l.powf(-2. / 3.) + 5. * l.powf(-8. / 3.)) + bulk)
}
#[test]
fn fbar_pretension_terminal_and_transients() {
    for force in [0., 0.1, 1000.] {
        let m = fbar_chain(force, true);
        let pre = equilibrium(&m, "Preload", true);
        let locked = equilibrium(&m, "Locked", true);
        let q = locked.pretension[0].adjustment;
        assert_eq!(q.to_bits(), pre.pretension[0].adjustment.to_bits());
        let l = 1. + q / 2.;
        let (p, dp) = neo_axial(l);
        let k = 2. * dp;
        assert!((p - force).abs() < 1e-6);
        assert!((locked.pretension[0].section_force - force).abs() < 1e-6);
        let h = 1e-5;
        let fd = (neo_axial(l + h).0 - neo_axial(l - h).0) / (2. * h);
        assert!((fd / dp - 1.).abs() < 1e-8);
        let freq = solve_frequency(&m, Some("Modes"), &FrequencyOptions::default(), &Limits::default()).unwrap();
        let mass = 2. / 3.;
        let eig_error = (freq.modes[0].eigenvalue / (k / mass) - 1.).abs();
        assert!(eig_error < 1e-9);
        if force <= 0.1 {
            let mut linear = chain(force, false);
            let mut section = linear.section("Solid").unwrap().clone();
            section.formulation.b_bar = true;
            linear.replace_section("Solid", section).unwrap();
            let lr = solve_static(&linear, Some("Preload"), &Limits::default()).unwrap();
            assert!((q - lr.pretension[0].adjustment).abs() < 1e-12);
            println!("FBAR_SMALL force={force:e} q_bbar_difference_m={:e}", (q - lr.pretension[0].adjustment).abs());
            // At zero preload, the finite-strain tangent must reduce to B-bar.
            if force == 0. {
                let lf = solve_frequency(&linear, Some("Modes"), &FrequencyOptions::default(), &Limits::default()).unwrap();
                assert!((freq.modes[0].eigenvalue / lf.modes[0].eigenvalue - 1.).abs() < 1e-12);
            }
        }
        let mut evidence = serde_json::json!({"preload":pre,"locked":locked,"frequency":freq,"reference":{"force":force,"stretch":l,"stiffness":k,"mass":mass,"force_residual_N":p-force,"tangent_fd":fd,"tangent_relative_error":fd/dp-1.,"eigen_relative_error":eig_error}});
        for mass_kind in [MassKind::Consistent, MassKind::Lumped] {
            let reference_mass = if mass_kind == MassKind::Consistent { 2. / 3. } else { 1. };
            let phi = 2. * ((k / reference_mass).sqrt() * 1e-5 / 2.).atan();
            for name in ["Direct", "Modal", "Zero"] {
                let r = fieldcraft_solver::transient::solve_with_mass(&m, Some(name), mass_kind, &Limits::default(), false).unwrap();
                let mid = r.nodes.iter().position(|n| *n == 5).unwrap();
                let mut errors = [0.0_f64; 3];
                let mut comparison = Vec::new();
                for (n, f) in r.frames.iter().enumerate() {
                    let phase = n as f64 * phi;
                    let want = if name == "Zero" {
                        [0.; 3]
                    } else {
                        [(1. - phase.cos()) / k, phase.sin() / (k * reference_mass).sqrt(), phase.cos() / reference_mass]
                    };
                    comparison.push(
                        serde_json::json!({"frame":n,"expected":want,"actual":[f.displacement[mid][2],f.velocity[mid][2],f.acceleration[mid][2]]}),
                    );
                    for (j, actual) in [f.displacement[mid][2], f.velocity[mid][2], f.acceleration[mid][2]].iter().enumerate() {
                        errors[j] = errors[j].max((actual - want[j]).abs());
                    }
                }
                assert!(errors[0] < 1e-12 && errors[1] < 1e-9 && errors[2] < 1e-8, "{name}: {errors:?}");
                println!(
                    "FBAR_CHAIN F={force:e} mass={mass_kind:?} {name} equilibrium_residual_N={:e} eigen_relative={eig_error:e} U_V_A_error={errors:?}",
                    (p - force).abs()
                );
                evidence[format!("{mass_kind:?}-{name}-comparison")] = serde_json::to_value(comparison).unwrap();
                evidence[format!("{mass_kind:?}-{name}")] = serde_json::to_value(r).unwrap();
            }
        }
        for modal in [false, true] {
            let mut hm = m.clone();
            if modal {
                let mut step = hm.step("Response").unwrap().clone();
                if let Procedure::Harmonic(h) = &mut step.procedure {
                    h.method = Method::Modal { modes: 1, damping_ratios: vec![], residual_vectors: 0 };
                }
                hm.replace_step(step).unwrap();
            }
            let hr = solve_harmonic(&hm, Some("Response"), &Limits::default()).unwrap();
            let mid = hr.nodes.iter().position(|n| *n == 5).unwrap();
            let mut error = 0.0_f64;
            for f in &hr.frames {
                let w = 2. * std::f64::consts::PI * f.frequency;
                let a = k - w * w * mass;
                let b = 2. * mass * w;
                error = error.max((f.real[mid][2] - a / (a * a + b * b)).abs()).max((f.imaginary[mid][2] + b / (a * a + b * b)).abs());
            }
            assert!(error < 1e-12);
            println!("FBAR_HARMONIC F={force:e} modal={modal} error_m={error:e}");
            evidence[format!("harmonic-{modal}")] = serde_json::to_value(hr).unwrap();
        }
        chain_evidence(&format!("fbar-{force}"), &evidence);
    }
}
#[test]
fn fbar_off_chain_reference() {
    let m = fbar_chain(1000., false);
    let mut evidence = serde_json::json!({"preload":equilibrium(&m,"Preload",true),"locked":equilibrium(&m,"Locked",true),"frequency":solve_frequency(&m,Some("Modes"),&FrequencyOptions::default(),&Limits::default()).unwrap(),"harmonic":solve_harmonic(&m,Some("Response"),&Limits::default()).unwrap()});
    for mass in [MassKind::Consistent, MassKind::Lumped] {
        for name in ["Direct", "Modal", "Zero"] {
            evidence[format!("{mass:?}-{name}")] =
                serde_json::to_value(fieldcraft_solver::transient::solve_with_mass(&m, Some(name), mass, &Limits::default(), false).unwrap())
                    .unwrap();
        }
    }
    chain_evidence("off-all-results", &evidence);
}

#[test]
fn fbar_stvk_continuous_pretension_and_modes() {
    for force in [0., 0.1, 1000.] {
        let mut m = chain(force, true);
        let mut section = m.section("Solid").unwrap().clone();
        section.formulation.f_bar = true;
        m.replace_section("Solid", section).unwrap();
        if force == 0.1 {
            for name in ["Preload", "Locked"] {
                let mut step = m.step(name).unwrap().clone();
                if let Procedure::NonlinearStatic(n) = &mut step.procedure {
                    n.initial_increment = 1.;
                    n.max_increment = 1.;
                }
                m.replace_step(step).unwrap();
            }
        }
        let pre = equilibrium(&m, "Preload", true);
        let locked = equilibrium(&m, "Locked", true);
        let q = locked.pretension[0].adjustment;
        assert_eq!(q.to_bits(), pre.pretension[0].adjustment.to_bits());
        let stretch = 1. + q / 2.;
        let p = 0.5e6 * stretch * (stretch * stretch - 1.);
        let k = 1e6 * (3. * stretch * stretch - 1.);
        let modes = solve_frequency(&m, Some("Modes"), &FrequencyOptions::default(), &Limits::default()).unwrap();
        let error = (modes.modes[0].eigenvalue / (k / (2. / 3.)) - 1.).abs();
        assert!((p - force).abs() < 1e-6 && (locked.pretension[0].section_force - force).abs() < 1e-6 && error < 1e-9);
        if force <= 0.1 {
            let mut linear = chain(force, false);
            let mut section = linear.section("Solid").unwrap().clone();
            section.formulation.b_bar = true;
            linear.replace_section("Solid", section).unwrap();
            let reference = solve_static(&linear, Some("Preload"), &Limits::default()).unwrap();
            assert!((q - reference.pretension[0].adjustment).abs() < 1e-12);
        }
        if force == 0. {
            let mut zero = m.step("Preload").unwrap().clone();
            zero.name = "Unloaded".into();
            zero.pretension.clear();
            m.add_step(zero, Some("Preload")).unwrap();
            let mut before = m.step("Modes").unwrap().clone();
            before.name = "Before".into();
            before.base_state = Some("Unloaded".into());
            m.add_step(before, Some("Preload")).unwrap();
            let z = equilibrium(&m, "Unloaded", true);
            assert!(z.displacement.iter().flatten().all(|v| *v == 0.));
            let before = solve_frequency(&m, Some("Before"), &FrequencyOptions::default(), &Limits::default()).unwrap();
            assert!((before.modes[0].eigenvalue / modes.modes[0].eigenvalue - 1.).abs() < 1e-12);
        }
        println!(
            "StVK Fbar continuous force={force:e} stretch={stretch:e} stiffness={k:e} force_residual_N={:e} eigen_relative={error:e}",
            p - force
        );
        chain_evidence(
            &format!("stvk-continuous-{force}"),
            &serde_json::json!({"model":m,"preload":pre,"locked":locked,"modes":modes,"reference":{"force":force,"stretch":stretch,"stiffness":k,"mass":2./3.,"force_residual_N":p-force,"eigen_relative":error}}),
        );
    }
}
