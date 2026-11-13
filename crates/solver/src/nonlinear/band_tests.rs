#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use fieldcraft_model::{Contact, ContactSurfaces, Elastic, ElementFace, ElementKind, Formulation, Material, Section, Set, Target};

/// Two original unit hexes, the lower one fixed, the upper one's lateral motion fixed and its
/// top displaced. Four free vertical degrees of freedom make a small real contact problem.
fn blocks() -> Model {
    let mut model = Model::default();
    let corners = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    for b in 0..2u32 {
        for (i, p) in corners.iter().enumerate() {
            model.set_node(8 * b + i as u32 + 1, [p[0], p[1], p[2] + b as f64]).unwrap();
        }
        model.set_element(b + 1, ElementKind::Hex8, (8 * b + 1..=8 * b + 8).collect()).unwrap();
    }
    model.add_set("all", Set { elements: vec![1, 2], ..Set::default() }).unwrap();
    model.add_set("base", Set { nodes: (1..=8).collect(), ..Set::default() }).unwrap();
    model.add_set("upper", Set { nodes: (9..=16).collect(), ..Set::default() }).unwrap();
    model.add_set("top", Set { nodes: (13..=16).collect(), ..Set::default() }).unwrap();
    model.add_set("main", Set { faces: vec![ElementFace { element: 1, face: 2 }], ..Set::default() }).unwrap();
    model.add_set("secondary", Set { faces: vec![ElementFace { element: 2, face: 1 }], ..Set::default() }).unwrap();
    model.add_material("Elastic", Material { elastic: Some(Elastic::Isotropic { young: 1000., poisson: 0. }), ..Material::default() }).unwrap();
    model
        .add_section(
            "Solid",
            Section {
                orientation: None,
                set: "all".into(),
                material: "Elastic".into(),
                kind: SectionKind::Solid {},
                formulation: Formulation::default(),
            },
        )
        .unwrap();
    model
        .add_step(Step::new("Step", Procedure::NonlinearStatic(NonlinearStatic { nonlinear_geometry: false, ..NonlinearStatic::default() })), None)
        .unwrap();
    for (name, values) in [
        ("base", [Some(0.), Some(0.), Some(0.), None, None, None]),
        ("upper", [Some(0.), Some(0.), None, None, None, None]),
        ("top", [None, None, Some(-0.01), None, None, None]),
    ] {
        model.add_boundary_condition(name, BoundaryCondition::Displacement { target: Target::Set(name.into()), values }).unwrap();
        model.set_boundary_condition_active("Step", name, true).unwrap();
    }
    model.add_contact("Interface", Contact::new(ContactSurfaces::Pair { main: "main".into(), secondary: "secondary".into() })).unwrap();
    model
}

/// Test-only assembly of the normal Run, with no production hook or policy override. This
/// small linear-elastic case needs no thermal, follower, plastic or rotational setup.
fn with_run<T>(model: &Model, f: impl FnOnce(&mut Run<'_>) -> T) -> T {
    let limits = Limits::default();
    let resolved = model.resolve().unwrap();
    let step = model.step("Step").unwrap();
    let Procedure::NonlinearStatic(settings) = &step.procedure else { panic!("test step") };
    let mesh = Mesh::new(model, &resolved, limits.max_dofs).unwrap();
    let (graph, order) = pattern(&mesh);
    let mut warnings = Vec::new();
    let setup = Setup { model, resolved: &resolved, mesh: &mesh, step };
    let red = reduction(&setup, true, &order, budget(&limits, &mesh, &graph, 1), &mut warnings).unwrap();
    let kinds: Vec<_> = mesh.elements.iter().map(|e| e.kernel().unwrap().dofs().node).collect();
    let problem = Problem {
        scalar_controls: Default::default(),
        pretension_sections: Vec::new(),
        mesh: &mesh,
        red: &red,
        dead: loads(model, &resolved, &mesh, step, &red.active).unwrap(),
        scheduled: Vec::new(),
        followers: Vec::new(),
        linear_supports: None,
        finite: settings.nonlinear_geometry,
        element_dofs: mesh.elements.iter().zip(&kinds).map(|(e, k)| mesh.layout.element_dofs(&e.nodes, *k).unwrap()).collect(),
        element_kinds: kinds,
        materials: mesh.elements.iter().map(|e| MaterialModel::from_elasticity(&e.d, None)).collect(),
        field: None,
        pattern: graph.zero_matrix(&mesh.layout),
        graph,
        contacts: Interface::all(model, &resolved, &mesh, settings.nonlinear_geometry, &mut warnings).unwrap(),
    };
    let mut run = Run {
        problem: &problem,
        settings,
        limits: &limits,
        force_scale: 0.,
        force_floor: None,
        moment_scale: 0.,
        length: 6f64.sqrt(),
        prescribed: (red.largest_prescribed(), 0.),
        rotation: vec![false; mesh.dofs()],
        rotation_eq: vec![false; red.neq()],
        iterations: 0,
        factor_entries: 0,
        committed: vec![Vec::new(); mesh.elements.len()],
        undeformed: None,
        contact: problem.contacts.iter().map(Interface::committed).collect(),
        reference: vec![0.; mesh.dofs()],
        band: 0.,
        augmentations: 0,
        augmentation: 0,
        retry: 0,
        retry_stick: None,
        converged_scale: 0.,
        converged_moment: 0.,
    };
    f(&mut run)
}

fn solve_increment(
    run: &mut Run<'_>,
    q: &[f64],
    from: f64,
    lambda: f64,
    predictor: Option<&[f64]>,
) -> std::result::Result<(Converged, usize), Failure> {
    let mut iterations = Vec::new();
    let mut failure_iterations = Vec::new();
    run.augmentation = 0;
    let size = (lambda - from).abs();
    let c = run.increment(q, predictor, None, lambda, size, &mut iterations, &mut failure_iterations)?;
    let mut augmentations = 0;
    let c = run.augment(c, q, lambda, size, &mut iterations, &mut failure_iterations, &mut augmentations, &mut Vec::new())?;
    Ok((c, augmentations))
}

fn succeeded<T>(result: std::result::Result<T, Failure>) -> T {
    match result {
        Ok(x) => x,
        Err(Failure::Cutback(s)) => panic!("cutback: {s}"),
        Err(Failure::Fatal(e)) => panic!("fatal: {e}"),
    }
}

fn tangent_error(run: &Run<'_>, q: &[f64], lambda: f64) -> f64 {
    let e = succeeded(run.state(q, lambda, true));
    assert!(e.contact.iter().all(|c| c.pressure.iter().all(|p| *p == 0.)), "the control must be open");
    let [k] = run.problem.red.reduce([e.tangent.as_ref().unwrap()]).unwrap();
    let mut exact = vec![0.; q.len()];
    k.mul(&vec![1.; q.len()], &mut exact);
    let h = 1e-7;
    let a = succeeded(run.state(&q.iter().map(|x| x - h).collect::<Vec<_>>(), lambda, false));
    let b = succeeded(run.state(&q.iter().map(|x| x + h).collect::<Vec<_>>(), lambda, false));
    let difference: Vec<_> = b.internal.iter().zip(&a.internal).map(|(b, a)| (b - a) / (2. * h)).collect();
    let numeric = run.problem.red.restrict(&difference);
    exact.iter().zip(numeric).map(|(a, b)| (a - b).abs()).fold(0., f64::max)
}

#[test]
fn a_committed_contact_increment_does_not_carry_its_chatter_band_into_the_next() {
    for seeded in [false, true] {
        with_run(&blocks(), |run| {
            // Controlled entry to exactly the retry state left by active-set chatter; this avoids
            // relying on platform rounding to create a small naturally chattering contact patch.
            if seeded {
                run.band = contact::ENGAGED_BAND;
                let failed = run.increment(&[0.; 4], None, None, -0.05, 0.05, &mut Vec::new(), &mut Vec::new());
                assert!(matches!(failed, Err(Failure::Cutback(_))), "exercise an actual failed retry");
                assert_eq!(run.band, contact::ENGAGED_BAND, "a failed retry must not clear the band");
            }
            let first = succeeded(solve_increment(run, &[0.; 4], 0., 0.5, None));
            assert!(first.1 > 0, "exercise the actual Uzawa loop");
            assert_eq!(run.band, if seeded { contact::ENGAGED_BAND } else { 0. }, "retries and augmentations retain the band");
            let open = vec![0.0005; 4];
            let stale_error = tangent_error(run, &open, -0.05);
            if seeded {
                assert!(stale_error > 100., "prove the open state's stale-band tangent mismatch: {stale_error:e}");
            } else {
                assert!(stale_error < 1e-5);
            }
            let q = first.0.q;
            run.commit_increment(&q, 0.5, &first.0.evaluation);
            let error = tangent_error(run, &open, -0.05);
            eprintln!("after accepted increment: band {} near-open tangent error {error:e}", run.band);
            let (predictor, _) = succeeded(run.tangent_predictor(&q, 0.5, -0.05)).unwrap();
            let second = succeeded(solve_increment(run, &q, 0.5, -0.05, Some(&predictor)));
            assert!(error < 1e-5, "a later open contact has a force-inconsistent tangent: {error:e}");
            assert!(second.0.evaluation.contact.iter().all(|c| c.pressure.iter().all(|p| p.abs() < 1e-8)));
            assert!(second.0.q.iter().all(|x| (*x - 0.0005).abs() < 1e-8), "{:?}", second.0.q);
        });
    }
}

/// Independent two-spring check: the upper unit cube has k_s=EA/L=1000 N/m;
/// the reference contact law gives k_c=10*3G*A/L=15000 N/m. Keeping its
/// positive-pressure branch yields k_s*k_c/(k_s+k_c), while exact bonding yields
/// k_s. No commercial reference values enter this test.
#[test]
fn preloaded_contact_perturbation_reaction_tangent_and_bonded_constraints() {
    for finite in [false, true] {
        let mut model = blocks();
        let mut step = model.step("Step").unwrap().clone();
        let Procedure::NonlinearStatic(settings) = &mut step.procedure else { panic!("step") };
        settings.nonlinear_geometry = finite;
        model.replace_step(step).unwrap();
        model
            .replace_boundary_condition(
                "top",
                BoundaryCondition::Displacement { target: Target::Set("top".into()), values: [None, None, Some(-1e-4), None, None, None] },
            )
            .unwrap();
        let mut contact = model.contact("Interface").unwrap().clone();
        contact.friction = fieldcraft_model::Friction::coulomb(0.3);
        model.replace_contact("Interface", contact).unwrap();
        with_run(&model, |run| {
            let (base, augmentations) = succeeded(solve_increment(run, &[0.; 4], 0., 1., None));
            assert_eq!(augmentations, 0, "this preload is below the prescribed gap tolerance");
            let q = base.q;
            run.commit_increment(&q, 1., &base.evaluation);
            let e = succeeded(run.state(&q, 1., true));
            let active = e.contact[0].active_set();
            assert!(e.contact[0].pressure.iter().all(|p| *p > 0.));
            assert!(e.contact[0].slipping.iter().all(|s| !s));
            let mesh = run.problem.mesh;
            let n = mesh.dofs();
            let top: Vec<_> = (13..=16).map(|label| mesh.dof(mesh.node(label).unwrap(), fieldcraft_fem::DofKind::Uz).unwrap()).collect();
            let u = run.problem.red.expand_scaled(&q, 1.);
            let k = e.tangent.as_ref().unwrap();
            let mut imposed = vec![0.; n];
            for d in &top {
                imposed[*d] = 1.;
            }
            let mut rhs = vec![0.; n];
            k.mul(&imposed, &mut rhs);
            let mut dq = run.problem.red.restrict(&rhs).into_iter().map(|v| -v).collect::<Vec<_>>();
            let [kff] = run.problem.red.reduce([k]).unwrap();
            factor_raw(&kff, run.limits).unwrap().solve(&mut dq).unwrap();
            let mut direction = run.problem.red.expand(&dq, false);
            for (x, y) in direction.iter_mut().zip(&imposed) {
                *x += y;
            }
            let mut exact = vec![0.; n];
            k.mul(&direction, &mut exact);
            let condensed: f64 = top.iter().map(|d| exact[*d]).sum();
            let structural = run.problem.evaluate_with_contact(&u, 1., true, &run.committed, &run.contact, &run.reference, 0., false).unwrap();
            let ks = structural.tangent.as_ref().unwrap();
            let mut bonded_force = vec![0.; n];
            ks.mul(&imposed, &mut bonded_force);
            let bonded: f64 = top.iter().map(|d| bonded_force[*d]).sum();
            let stretch = 1. - 1e-4 - q.iter().sum::<f64>() / 4.;
            let theory_bonded = if finite { 1000. * (3. * stretch * stretch - 1.) / 2. } else { 1000. };
            let theory_contact = theory_bonded * 15000. / (theory_bonded + 15000.);
            assert!((bonded / theory_bonded - 1.).abs() < 1e-10);
            assert!((condensed / theory_contact - 1.).abs() < 1e-9);
            let rows = run.problem.contacts[0].frozen_rows(mesh, &u, &e.contact[0]).unwrap();
            assert_eq!(rows.len(), 12);
            let violation = |v: &[f64]| rows.iter().map(|r| r.terms.iter().map(|(d, c)| v[*d] * c).sum::<f64>().abs()).fold(0., f64::max);
            assert!(violation(&imposed) < 1e-13);
            assert!(violation(&direction) > 0.01, "finite contact compliance is not exact bonding");
            for h in [1e-7, 5e-8] {
                let mut reactions = Vec::new();
                let mut forces = Vec::new();
                for sign in [-1., 1.] {
                    let trial: Vec<_> = u.iter().zip(&direction).map(|(u, d)| u + sign * h * d).collect();
                    let trial = run.problem.evaluate(&trial, 1., false, &run.committed, &run.contact, &run.reference, 0.).unwrap();
                    assert_eq!(trial.contact[0].active_set(), active);
                    assert!(trial.contact[0].slipping.iter().all(|s| !s));
                    forces.push(trial.internal);
                    let lambda = 1. - sign * h / 1e-4;
                    let trial = succeeded(run.increment(&q, None, None, lambda, h / 1e-4, &mut Vec::new(), &mut Vec::new()));
                    assert_eq!(trial.evaluation.contact[0].active_set(), active);
                    assert!(trial.evaluation.contact[0].slipping.iter().all(|s| !s));
                    reactions.push(top.iter().map(|d| trial.evaluation.internal[*d]).sum::<f64>());
                }
                let fd = (reactions[1] - reactions[0]) / (2. * h);
                let error = exact.iter().enumerate().map(|(d, x)| ((forces[1][d] - forces[0][d]) / (2. * h) - x).abs()).fold(0., f64::max);
                eprintln!(
                    "CONTACT_OPERATOR finite={finite} h={h:e} pressure={:?} gap={:?} k_reaction_fd={fd:.12e} k_tangent={condensed:.12e} k_series={theory_contact:.12e} k_bonded={bonded:.12e} full_residual_error={error:e} finite_constraint_violation={} bonded_constraint_violation={}",
                    e.contact[0].pressure,
                    e.contact[0].gap,
                    violation(&direction),
                    violation(&imposed)
                );
                assert!((fd / condensed - 1.).abs() < 1e-6);
                assert!(error / condensed < 1e-6);
            }
            if finite {
                // Exercise the actual exported terminal operator and actual row elimination.
                let policies = std::collections::BTreeMap::from([("Interface".into(), fieldcraft_model::ContactPerturbation::Stick)]);
                let (_, terminal) = terminal_tangent(&model, "Step", run.limits, &policies).unwrap();
                let resolved = model.resolve().unwrap();
                let setup = Setup { model: &model, resolved: &resolved, mesh, step: model.step("Step").unwrap() };
                let (graph, order) = pattern(mesh);
                let frozen = crate::analysis::reduction_with_frozen(
                    &setup,
                    false,
                    &order,
                    budget(run.limits, mesh, &graph, 1),
                    &mut Vec::new(),
                    Some(&terminal.constraints),
                )
                .unwrap();
                assert_eq!(frozen.neq(), 0, "the four free interface Uz are eliminated");
                let mut force = vec![0.; n];
                terminal.stiffness.mul(&imposed, &mut force);
                let kt: f64 = top.iter().map(|d| force[*d]).sum();
                assert!((kt / bonded - 1.).abs() < 1e-9);
                eprintln!("CONTACT_TERMINAL neq_before=4 neq_after={} rows={} k_exported={kt:.12e}", frozen.neq(), terminal.constraints.len());
            }
        });
    }
}

#[test]
fn explicit_stick_refuses_contact_switches_and_unqualified_gaps() {
    let mut model = blocks();
    let mut step = model.step("Step").unwrap().clone();
    let Procedure::NonlinearStatic(settings) = &mut step.procedure else { panic!("step") };
    settings.nonlinear_geometry = true;
    model.replace_step(step).unwrap();
    let mut c = model.contact("Interface").unwrap().clone();
    c.friction = fieldcraft_model::Friction::coulomb(0.3);
    model.replace_contact("Interface", c).unwrap();
    with_run(&model, |run| {
        let (base, _) = succeeded(solve_increment(run, &[0.; 4], 0., 1., None));
        let u = run.problem.red.expand_scaled(&base.q, 1.);
        let interface = &run.problem.contacts[0];
        let state = &base.evaluation.contact[0];
        let qualify = |s: &contact::State| interface.stick_rows(run.problem.mesh, &u, s, run.problem.red, 1e-8, &mut Vec::new());
        assert_eq!(qualify(state).unwrap().len(), 12);
        let mut critical = state.clone();
        critical.traction[0] = [0.3 * critical.pressure[0], 0., 0.];
        assert!(qualify(&critical).unwrap_err().to_string().contains("friction_switch_or_sliding"));
        let mut slipping = state.clone();
        slipping.slipping[0] = true;
        assert!(qualify(&slipping).unwrap_err().to_string().contains("friction_switch_or_sliding"));
        let mut transition = state.clone();
        transition.pressure[0] = 0.;
        assert!(qualify(&transition).unwrap_err().to_string().contains("normal_switch"));
        let mut unconverged = state.clone();
        unconverged.gap[0] = Some(-2. * contact::GAP_TOLERANCE);
        assert!(qualify(&unconverged).unwrap_err().to_string().contains("base_equilibrium_unqualified"));
        let mut edge = state.clone();
        edge.support[0] *= 0.9;
        assert!(qualify(&edge).unwrap_err().to_string().contains("stick_requires_full_overlap"));
    });
}

fn interior_blocks(friction: bool) -> Model {
    let mut model = blocks();
    for label in 1..=8 {
        let mut x = model.node(label).unwrap().xyz;
        x[0] = 1.5 * x[0] - 0.25;
        x[1] = 1.5 * x[1] - 0.25;
        model.set_node(label, x).unwrap();
    }
    let mut step = model.step("Step").unwrap().clone();
    let Procedure::NonlinearStatic(settings) = &mut step.procedure else { panic!("step") };
    settings.nonlinear_geometry = true;
    model.replace_step(step).unwrap();
    if friction {
        let mut c = model.contact("Interface").unwrap().clone();
        c.friction = fieldcraft_model::Friction::coulomb(0.3);
        model.replace_contact("Interface", c).unwrap();
    }
    model
}

#[test]
fn tangent_free_normal_geometry_is_the_fixed_pressure_force_derivative() {
    with_run(&interior_blocks(false), |run| {
        let (base, _) = succeeded(solve_increment(run, &[0.; 4], 0., 1., None));
        let u = run.problem.red.expand_scaled(&base.q, 1.);
        let mesh = run.problem.mesh;
        let interface = &run.problem.contacts[0];
        let state = &base.evaluation.contact[0];
        let (rows, blocks) = interface.tangent_free_rows(mesh, &u, state, run.problem.red, 1e-8, &mut Vec::new()).unwrap();
        assert_eq!(rows.len(), 4);
        let graph = run.problem.graph.with_blocks(blocks.iter().map(|(nodes, _)| nodes.as_slice()));
        let mut kg = graph.zero_matrix(&mesh.layout);
        for (nodes, data) in &blocks {
            add_block(&mut kg, &mesh.layout, nodes, mesh.translation_kinds(), data).unwrap();
        }
        let mut direction = vec![0.; mesh.dofs()];
        for (i, &label) in mesh.labels.iter().enumerate().filter(|(_, n)| **n >= 9) {
            let x = mesh.xyz[i];
            for (kind, value) in [(fieldcraft_fem::DofKind::Ux, 0.2 * (x[0] - 0.5)), (fieldcraft_fem::DofKind::Uz, 0.1 * (x[1] - 0.5))] {
                direction[mesh.dof(i, kind).unwrap()] = value;
            }
            assert!(label >= 9);
        }
        let mut expected = vec![0.; mesh.dofs()];
        kg.mul(&direction, &mut expected);
        assert!(norm_inf(&expected) > 1e-3, "exercise a nonzero geometric contact action");
        let fixed_pressure_force = |x: &[f64]| {
            let mut committed = run.contact[0].clone();
            let mut force = vec![0.; mesh.dofs()];
            let trial = interface.evaluate(mesh, x, &u, &committed, 0., false, &mut force, &mut Vec::new()).unwrap();
            // Remove the AL penalty derivative by keeping the actual pressures fixed,
            // not by subtracting another implementation of its tangent matrix.
            for ((lambda, p), target) in committed.multipliers.iter_mut().zip(&trial.pressure).zip(&state.pressure) {
                *lambda += target - p;
            }
            force.fill(0.);
            let held = interface.evaluate(mesh, x, &u, &committed, 0., false, &mut force, &mut Vec::new()).unwrap();
            assert!(held.pressure.iter().zip(&state.pressure).all(|(a, b)| (a - b).abs() < 1e-10));
            force
        };
        for h in [1e-6, 5e-7] {
            let plus: Vec<_> = u.iter().zip(&direction).map(|(a, b)| a + h * b).collect();
            let minus: Vec<_> = u.iter().zip(&direction).map(|(a, b)| a - h * b).collect();
            let fp = fixed_pressure_force(&plus);
            let fm = fixed_pressure_force(&minus);
            let error = fp.iter().zip(&fm).zip(&expected).map(|((p, m), v)| ((p - m) / (2. * h) - v).abs()).fold(0., f64::max) / norm_inf(&expected);
            eprintln!("TANGENT_FREE_GEOMETRY h={h:.3e} action={:.12e} relative_error={error:.3e}", norm_inf(&expected));
            assert!(error < 1e-6, "normal geometric tangent {error}");
        }
    });
}

#[test]
fn tangent_free_refuses_shear_history_and_switches() {
    with_run(&interior_blocks(true), |run| {
        let (base, _) = succeeded(solve_increment(run, &[0.; 4], 0., 1., None));
        let u = run.problem.red.expand_scaled(&base.q, 1.);
        let interface = &run.problem.contacts[0];
        let state = &base.evaluation.contact[0];
        let qualify = |s: &contact::State| interface.tangent_free_rows(run.problem.mesh, &u, s, run.problem.red, 1e-8, &mut Vec::new());
        assert_eq!(qualify(state).unwrap().0.len(), 4);
        let mut shear = state.clone();
        shear.traction[0] = [0.05 * shear.pressure[0], 0., 0.];
        let mut slip = state.clone();
        slip.slipping[0] = true;
        let mut critical = state.clone();
        critical.traction[0] = [0.3 * critical.pressure[0], 0., 0.];
        let mut zero = state.clone();
        zero.pressure[0] = 0.;
        let mut gap = state.clone();
        gap.gap[0] = Some(-2. * contact::GAP_TOLERANCE);
        let mut overlap = state.clone();
        overlap.support[0] *= 0.9;
        let mut missing = state.clone();
        missing.slipping.clear();
        for (state, reason) in [
            (shear, "shear_release_undefined"),
            (slip, "friction_switch_or_sliding"),
            (critical, "friction_switch_or_sliding"),
            (zero, "normal_switch"),
            (gap, "base_equilibrium_unqualified"),
            (overlap, "closed_tangent_free_requires_full_overlap"),
            (missing, "base_evidence_missing"),
        ] {
            let error = qualify(&state).unwrap_err().to_string();
            eprintln!("TANGENT_FREE_STATE_REFUSAL {reason}: {error}");
            assert!(error.contains(reason), "{error}");
        }
    });
}

#[test]
fn open_qualification_requires_zero_force_history_and_geometric_separation() {
    let mut model = interior_blocks(true);
    for label in 9..=16 {
        let mut x = model.node(label).unwrap().xyz;
        x[2] += 0.02;
        model.set_node(label, x).unwrap();
    }
    model
        .replace_boundary_condition(
            "top",
            BoundaryCondition::Displacement { target: Target::Set("top".into()), values: [None, None, Some(0.), None, None, None] },
        )
        .unwrap();
    with_run(&model, |run| {
        let (base, _) = succeeded(solve_increment(run, &[0.; 4], 0., 1., None));
        let u = run.problem.red.expand_scaled(&base.q, 1.);
        let interface = &run.problem.contacts[0];
        let state = &base.evaluation.contact[0];
        let history = &run.contact[0];
        let qualify =
            |s: &contact::State, h: &contact::Committed| interface.qualify_open(run.problem.mesh, &u, s, h, run.problem.red, &mut Vec::new());
        qualify(state, history).unwrap();
        // Search absence is acceptable only because the current geometry certifies
        // separation; it must not bypass zero-force/history evidence.
        let mut absent = state.clone();
        absent.gap.fill(None);
        qualify(&absent, history).unwrap();
        let mut force = absent.clone();
        force.nodal_force[0][2] = 1e-12;
        let mut pressure = absent.clone();
        pressure.pressure[0] = 1e-12;
        let mut shear = absent.clone();
        shear.traction[0][0] = 1e-12;
        let mut slip = absent.clone();
        slip.slipping[0] = true;
        let mut missing = absent.clone();
        missing.pressure.clear();
        for (s, reason) in [
            (force, "open_contact_force_nonzero"),
            (pressure, "open_contact_force_nonzero"),
            (shear, "open_history_unqualified"),
            (slip, "open_history_unqualified"),
            (missing, "base_evidence_missing"),
        ] {
            let error = qualify(&s, history).unwrap_err().to_string();
            eprintln!("OPEN_STATE_REFUSAL {reason}: {error}");
            assert!(error.contains(reason), "{error}");
        }
        for tangential in [false, true] {
            let mut held = history.clone();
            if tangential {
                held.traction[0][0] = 1e-12;
            } else {
                held.multipliers[0] = 1e-12;
            }
            let error = qualify(&absent, &held).unwrap_err().to_string();
            assert!(error.contains("open_history_unqualified"), "{error}");
        }
    });
}

/// Quadratic two-solid patch with a deformable main; no rigid-body freedoms.
fn quadratic_policy_blocks(open: bool) -> Model {
    let mut model = blocks();
    let edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)];
    for b in 0..2u32 {
        let mut labels: Vec<_> = (8 * b + 1..=8 * b + 8).collect();
        for &label in &labels {
            let mut x = model.node(label).unwrap().xyz;
            if open {
                if b == 0 {
                    x[0] = 1.5 * (x[0] - 0.5) + 0.5;
                    x[1] = 1.5 * (x[1] - 0.5) + 0.5;
                } else {
                    x[2] += 0.02;
                }
            }
            model.set_node(label, x).unwrap();
        }
        for (i, (a, c)) in edges.iter().enumerate() {
            let xa = model.node(labels[*a]).unwrap().xyz;
            let xc = model.node(labels[*c]).unwrap().xyz;
            let label = 17 + 12 * b + i as u32;
            model.set_node(label, std::array::from_fn(|k| (xa[k] + xc[k]) / 2.)).unwrap();
            labels.push(label);
        }
        model.set_element(b + 1, ElementKind::Hex20, labels).unwrap();
    }
    model.replace_set("base", Set { nodes: (1..=4).chain(17..=20).collect(), ..Default::default() }).unwrap();
    model.replace_set("upper", Set { nodes: (1..=40).collect(), ..Default::default() }).unwrap();
    model.replace_set("top", Set { nodes: (13..=16).chain(33..=36).collect(), ..Default::default() }).unwrap();
    model
        .replace_boundary_condition(
            "top",
            BoundaryCondition::Displacement {
                target: Target::Set("top".into()),
                values: [None, None, Some(if open { 0. } else { -1e-4 }), None, None, None],
            },
        )
        .unwrap();
    let mut contact = model.contact("Interface").unwrap().clone();
    contact.friction = fieldcraft_model::Friction::coulomb(0.3);
    model.replace_contact("Interface", contact).unwrap();
    let mut step = model.step("Step").unwrap().clone();
    let Procedure::NonlinearStatic(settings) = &mut step.procedure else { panic!("settings") };
    settings.nonlinear_geometry = true;
    settings.initial_increment = 1.;
    settings.max_increment = 1.;
    model.replace_step(step).unwrap();
    model
}

/// Tri6 faces use the same qualified policies; layer guides make the base planar.
fn quadratic_tri_policy_blocks(open: bool) -> Model {
    use fieldcraft_model::{Coupling, CouplingKind, Dofs, ReferencePoint};
    let mut m = blocks();
    let corners = [[[0., 0., 1.], [0., 1., 1.], [1., 0., 1.], [0., 0., 0.]], [[0., 0., 1.], [1., 0., 1.], [0., 1., 1.], [0., 0., 2.]]];
    let edges = [(0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3)];
    for b in 0..2u32 {
        let mut xyz = corners[b as usize].to_vec();
        for (a, c) in edges {
            xyz.push(std::array::from_fn(|k| (corners[b as usize][a][k] + corners[b as usize][c][k]) / 2.));
        }
        for (i, mut x) in xyz.into_iter().enumerate() {
            if open {
                if b == 0 {
                    x[0] = 2. * x[0] - 0.25;
                    x[1] = 2. * x[1] - 0.25;
                } else {
                    x[2] += 0.02;
                }
            }
            m.set_node(10 * b + i as u32 + 1, x).unwrap();
        }
        m.set_element(b + 1, ElementKind::Tet10, (10 * b + 1..=10 * b + 10).collect()).unwrap();
    }
    m.replace_set("main", Set { faces: vec![ElementFace { element: 1, face: 1 }], ..Default::default() }).unwrap();
    m.replace_set("secondary", Set { faces: vec![ElementFace { element: 2, face: 1 }], ..Default::default() }).unwrap();
    m.replace_set("base", Set { nodes: vec![4], ..Default::default() }).unwrap();
    m.replace_set("upper", Set { nodes: (1..=20).collect(), ..Default::default() }).unwrap();
    m.replace_set("top", Set { nodes: vec![14], ..Default::default() }).unwrap();
    m.replace_boundary_condition(
        "top",
        BoundaryCondition::Displacement {
            target: Target::Set("top".into()),
            values: [None, None, Some(if open { 0. } else { -1e-4 }), None, None, None],
        },
    )
    .unwrap();
    for (i, nodes) in [vec![1, 2, 3, 5, 6, 7], vec![8, 9, 10], vec![11, 12, 13, 15, 16, 17], vec![18, 19, 20]].into_iter().enumerate() {
        let name = format!("Plane{i}");
        let xyz = m.node(nodes[0]).unwrap().xyz;
        m.add_set(&name, Set { nodes, ..Default::default() }).unwrap();
        m.add_reference_point(&name, ReferencePoint { xyz, label: 100 + i as u32 }).unwrap();
        m.add_coupling(
            &name,
            Coupling {
                reference_point: name.clone(),
                set: name.clone(),
                kind: CouplingKind::Kinematic,
                dofs: serde_json::from_value::<Dofs>(serde_json::json!([3])).unwrap(),
            },
        )
        .unwrap();
        m.add_boundary_condition(
            &name,
            BoundaryCondition::Displacement {
                target: Target::ReferencePoint(name.clone()),
                values: [Some(0.), Some(0.), None, Some(0.), Some(0.), Some(0.)],
            },
        )
        .unwrap();
        m.set_boundary_condition_active("Step", &name, true).unwrap();
    }
    let mut c = m.contact("Interface").unwrap().clone();
    c.friction = fieldcraft_model::Friction::coulomb(0.3);
    m.replace_contact("Interface", c).unwrap();
    let mut step = m.step("Step").unwrap().clone();
    let Procedure::NonlinearStatic(n) = &mut step.procedure else { panic!("step") };
    n.nonlinear_geometry = true;
    n.initial_increment = 1.;
    n.max_increment = 1.;
    m.replace_step(step).unwrap();
    m
}

#[test]
fn quadratic_frozen_policy_terminal_tangent_difference_and_relative_elimination() {
    for (tri, open) in [(false, false), (false, true), (true, false), (true, true)] {
        let model = if tri { quadratic_tri_policy_blocks(open) } else { quadratic_policy_blocks(open) };
        let policy = if open { fieldcraft_model::ContactPerturbation::Open } else { fieldcraft_model::ContactPerturbation::Stick };
        let policies = std::collections::BTreeMap::from([("Interface".into(), policy)]);
        let (base, terminal) = terminal_tangent(&model, "Step", &Limits::default(), &policies).unwrap();
        with_run(&model, |run| {
            let mesh = run.problem.mesh;
            let mut u = vec![0.; mesh.dofs()];
            for (i, label) in base.nodes.iter().enumerate() {
                if let Some(node) = mesh.node(*label) {
                    for (k, kind) in mesh.translation_kinds().iter().enumerate() {
                        u[mesh.dof(node, kind).unwrap()] = base.displacement[i][k];
                    }
                }
            }
            let resolved = model.resolve().unwrap();
            let setup = Setup { model: &model, resolved: &resolved, mesh, step: model.step("Step").unwrap() };
            let (graph, order) = pattern(mesh);
            let frozen = crate::analysis::reduction_with_frozen(
                &setup,
                false,
                &order,
                budget(run.limits, mesh, &graph, 1),
                &mut Vec::new(),
                Some(&terminal.constraints),
            )
            .unwrap();
            let main = mesh.dof(mesh.node(if tri { 1 } else { 5 }).unwrap(), fieldcraft_fem::DofKind::Uz).unwrap();
            assert!(frozen.has_free_motion(main));
            if open {
                assert!(terminal.constraints.is_empty());
            } else {
                assert_eq!(terminal.constraints.len(), if tri { 18 } else { 24 });
                assert!(terminal.constraints.iter().any(|r| run.problem.red.homogeneous_row_residual(&r.terms) > 1e-3));
                assert!(terminal.constraints.iter().all(|r| frozen.homogeneous_row_residual(&r.terms) < 1e-12));
                let bad = run.problem.contacts[0].check_stick_perturbation(&terminal.constraints, run.problem.red).unwrap_err().to_string();
                assert!(bad.contains("stick_constraint_not_eliminated"));
                eprintln!("QUADRATIC_ELIMINATION_REFUSAL {bad}");
            }
            let [k] = frozen.reduce([&terminal.stiffness]).unwrap();
            let q: Vec<_> = (0..frozen.neq()).map(|i| ((i * 7 % 13) as f64 - 6.) / 13.).collect();
            let direction = frozen.expand(&q, false);
            let mut exact = vec![0.; q.len()];
            k.mul(&q, &mut exact);
            for h in [1e-6, 5e-7] {
                let mut forces = Vec::new();
                for sign in [-1., 1.] {
                    let trial: Vec<_> = u.iter().zip(&direction).map(|(v, d)| v + sign * h * d).collect();
                    // The explicit stick law is C0*(u-u0)=0, with C0 held fixed.
                    // Its Lagrange reaction has zero derivative at fixed multiplier.
                    // Structural force differences therefore test its whole reduced K.
                    let e = run.problem.evaluate_with_contact(&trial, 1., false, &run.committed, &run.contact, &run.reference, 0., open).unwrap();
                    if open {
                        assert!(e.contact.iter().all(|c| c.nodal_force.iter().all(|f| *f == [0.; 3])));
                    }
                    forces.push(frozen.restrict(&e.internal));
                }
                let error =
                    exact.iter().enumerate().map(|(i, v)| ((forces[1][i] - forces[0][i]) / (2. * h) - v).abs()).fold(0., f64::max) / norm_inf(&exact);
                eprintln!(
                    "QUADRATIC_TANGENT tri={tri} open={open} h={h:e} neq={} rows={} relative={error:.15e}",
                    frozen.neq(),
                    terminal.constraints.len()
                );
                assert!(error < 1e-6);
            }
        });
    }
}
