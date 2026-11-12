//! Two stacked blocks pressed together by a pressure on the top face, with frictionless
//! contact between them: every increment after the first used to fail, because an increment
//! that starts within the force tolerance left the energy criterion nothing to measure.
//!
//! Kept at the engine boundary so that the mesh, contact surfaces, units and step options are
//! exactly those of a command script a user would write.

use fieldcraft_engine::{Session, StepResult};
use fieldcraft_solver::StepResult as Implicit;
use fieldcraft_solver::contact::Status;
use serde_json::{Value, json};

fn specimen(strict: bool) -> Value {
    let mut procedure = json!({
        "type": "nonlinear_static",
        "nonlinear_geometry": true,
        "initial_increment": 0.25,
        "max_increment": 1.0,
        "max_increments": 20,
        "max_iterations": 20,
        "line_search": true
    });
    if strict {
        procedure["tolerances"] = json!({"force": 1e-6, "displacement": 1e-8, "energy": 1e-10});
    }
    json!([
        {"command": "model.units", "params": {"system": "mm-t-s-N-MPa"}},
        {"command": "mesh.block", "params": {
            "body": "Lower", "kind": "hex8",
            "corners": [[0, 0, 0], [100, 0, 0], [100, 100, 0], [0, 100, 0], [0, 0, 10], [100, 0, 10], [100, 100, 10], [0, 100, 10]],
            "divisions": [8, 8, 4], "faces": {"1": "bottom", "2": "top"}, "merge": 0
        }},
        {"command": "mesh.block", "params": {
            "body": "Upper", "kind": "hex8",
            "corners": [[0, 0, 10], [100, 0, 10], [100, 100, 10], [0, 100, 10], [0, 0, 20], [100, 0, 20], [100, 100, 20], [0, 100, 20]],
            "divisions": [8, 8, 4], "faces": {"1": "bottom", "2": "top"}, "merge": 0
        }},
        {"command": "model.set.create", "params": {"name": "All", "geometry": [{"body": "Lower"}, {"body": "Upper"}]}},
        {"command": "model.set.create", "params": {"name": "Bottom", "geometry": [{"body": "Lower", "face": "bottom"}]}},
        {"command": "model.set.create", "params": {"name": "Top", "geometry": [{"body": "Upper", "face": "top"}]}},
        {"command": "model.set.create", "params": {"name": "LowerTop", "geometry": [{"body": "Lower", "face": "top"}]}},
        {"command": "model.set.create", "params": {"name": "UpperBottom", "geometry": [{"body": "Upper", "face": "bottom"}]}},
        {"command": "model.material.create", "params": {"name": "Steel", "elastic": {"type": "isotropic", "young": 210000, "poisson": 0.3}, "density": 7.85e-09}},
        {"command": "model.section.create", "params": {"name": "Solid", "set": "All", "material": "Steel", "kind": {"type": "solid"}}},
        {"command": "model.boundary_condition.create", "params": {"name": "BaseFix", "type": "fixed", "target": {"set": "Bottom"}}},
        {"command": "model.boundary_condition.create", "params": {"name": "TopGuide", "type": "displacement", "target": {"set": "Top"}, "values": [0, 0, null, null, null, null]}},
        {"command": "model.load.create", "params": {"name": "Pressure", "type": "pressure", "set": "Top", "magnitude": 1.0}},
        {"command": "interaction.contact.pair", "params": {"name": "Interface", "main": "LowerTop", "secondary": "UpperBottom"}},
        {"command": "model.step.create", "params": {"name": "Preload", "procedure": procedure, "boundary_conditions": ["BaseFix", "TopGuide"], "loads": ["Pressure"]}}
    ])
}

#[test]
fn pressure_driven_stacked_blocks_converge_and_balance() {
    for strict in [true, false] {
        let mut session = Session::default();
        session.run_script(&specimen(strict)).unwrap();
        let reply = session.execute("solve.nonlinear", &json!({"step": "Preload", "threads": 8, "max_factor_entries": 2_000_000_000})).unwrap();
        let StepResult::Implicit(Implicit::Static(result)) = session.result("Preload").unwrap().as_ref() else { panic!("nonlinear result") };
        let contact = &result.contacts[0];
        assert!((contact.normal_force - 10_000.0).abs() <= 1e-8 * 10_000.0, "strict={strict}: {:?}", contact.normal_force);
        assert_eq!(contact.closed(), 81, "strict={strict}");
        assert_eq!(contact.status.iter().filter(|status| **status == Status::Slip).count(), 0, "strict={strict}");
        assert!((reply["contacts"][0]["normal_force"].as_f64().unwrap() - 10_000.0).abs() <= 1e-8 * 10_000.0);
    }
}
