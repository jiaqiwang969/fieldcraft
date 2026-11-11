//! Command opt-in, saved-input identity and the no-Newton counterexample.
use fieldcraft_engine::{Session, StepResult};
use serde_json::{Value, json};

fn specimen() -> Session {
    let recipe: Vec<Value> = serde_json::from_str(include_str!("fixtures/fbar_pretension.json")).unwrap();
    let prefix: Vec<_> = recipe.into_iter().take_while(|c| c["command"] != "solve.nonlinear").collect();
    let mut s = Session::default();
    s.run_script(&json!(prefix)).unwrap();
    s.execute("model.pretension.fix", &json!({"name":"Cut","step":"Locked"})).unwrap();
    s.execute("model.step.create", &json!({"name":"Response","procedure":{"type":"harmonic","frequencies":[100,500,600,1000],"method":{"type":"direct"},"damping":{"loss_factor":0.02}},"base_state":"Locked","boundary_conditions":["Ends","Guide"],"loads":["Probe"],"output":["displacement"]})).unwrap();
    s
}

fn result(s: &Session, name: &str) -> Vec<u8> {
    let StepResult::Implicit(r) = s.result(name).unwrap().as_ref() else { panic!("implicit result") };
    serde_json::to_vec(r).unwrap()
}

#[test]
fn stored_base_survives_reopen_skips_newton_and_rejects_stale_input() {
    let mut s = specimen();
    let missing = s.execute("solve.frequency", &json!({"step":"Modes","reuse_base":true})).unwrap_err();
    assert!(missing.to_string().contains("stored_equilibrium.current_result_required"), "{missing}");
    s.execute("solve.nonlinear", &json!({"step":"Locked"})).unwrap();
    let terminal = result(&s, "Locked");
    let mut reference = Vec::new();
    for (command, step) in [("solve.frequency", "Modes"), ("solve.harmonic", "Response")] {
        let error = s.execute(command, &json!({"step":step,"max_newton_iterations":1})).unwrap_err();
        assert!(error.to_string().contains("1 Newton iterations"), "{error}");
        s.execute(command, &json!({"step":step})).unwrap();
        reference.push(result(&s, step));
        s.execute(command, &json!({"step":step,"reuse_base":false})).unwrap();
        assert_eq!(reference.last().unwrap(), &result(&s, step));
    }
    let dir = std::env::temp_dir().join(format!("fieldcraft-stored-equilibrium-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("solved.fieldcraft");
    s.execute("file.save", &json!({"path":path})).unwrap();
    let mut reopened = Session::default();
    reopened.execute("file.open", &json!({"path":path})).unwrap();
    for ((command, step), expected) in [("solve.frequency", "Modes"), ("solve.harmonic", "Response")].into_iter().zip(reference) {
        let reply = reopened.execute(command, &json!({"step":step,"reuse_base":true,"max_newton_iterations":1})).unwrap();
        assert_eq!(reply["equilibrium_reuse"]["newton_iterations"], 0);
        assert_eq!(reply["equilibrium_reuse"]["base"], "Locked");
        assert_eq!(expected, result(&reopened, step));
        assert_eq!(terminal, result(&reopened, "Locked"));
    }
    reopened.execute("model.material.edit", &json!({"name":"M","density":2.0})).unwrap();
    let error = reopened.execute("solve.frequency", &json!({"step":"Modes","reuse_base":true})).unwrap_err();
    assert!(error.to_string().contains("stored_equilibrium.current_result_required"), "{error}");
}
