use fieldcraft_engine::Session;
use serde_json::{Value, json};

#[test]
fn fbar_command_roundtrip_and_explicit_off() {
    let script: Value = serde_json::from_str(include_str!("fixtures/fbar.json")).unwrap();
    let mut s = Session::default();
    s.run_script(&script).unwrap();
    assert!(s.model.section("S").unwrap().formulation.f_bar);
    let root = std::env::var_os("FIELDCRAFT_FBAR_EVIDENCE")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join(format!("fbar-{}", std::process::id())));
    std::fs::create_dir_all(&root).unwrap();
    let path = root.join("stretch.fieldcraft");
    s.execute("file.save_as", &json!({"path":path})).unwrap();
    let mut reopened = Session::default();
    reopened.execute("file.open", &json!({"path":path})).unwrap();
    assert!(reopened.model.section("S").unwrap().formulation.f_bar);
    assert_eq!(s.result("Stretch"), reopened.result("Stretch"));
    reopened.execute("solve.nonlinear", &json!({"step":"Stretch"})).unwrap();
    assert_eq!(s.result("Stretch"), reopened.result("Stretch"));
    let mut off = script.clone();
    for c in off.as_array_mut().unwrap() {
        if c["command"] == "model.section.create" {
            c["params"]["formulation"] = json!({"f_bar":false});
        }
    }
    let mut a = Session::default();
    a.run_script(&off).unwrap();
    for c in off.as_array_mut().unwrap() {
        if c["command"] == "model.section.create" {
            c["params"].as_object_mut().unwrap().remove("formulation");
        }
    }
    let mut b = Session::default();
    b.run_script(&off).unwrap();
    assert_eq!(a.result("Stretch"), b.result("Stretch"));
    let unsupported = fieldcraft_decks::abaqus::element_type(fieldcraft_model::ElementKind::Hex8, s.model.section("S").unwrap()).unwrap_err();
    assert!(unsupported.contains("f_bar"));
    if std::env::var_os("FIELDCRAFT_FBAR_EVIDENCE").is_none() {
        std::fs::remove_file(path).unwrap();
    }
}

#[test]
fn fbar_pretension_base_state_roundtrip() {
    let script: Value = serde_json::from_str(include_str!("fixtures/fbar_pretension.json")).unwrap();
    let mut s = Session::default();
    s.run_script(&script).unwrap();
    let root = std::env::var_os("FIELDCRAFT_FBAR_EVIDENCE")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join(format!("fbar-base-{}", std::process::id())));
    std::fs::create_dir_all(&root).unwrap();
    let path = root.join("preload-locked.fieldcraft");
    s.execute("file.save_as", &json!({"path":path})).unwrap();
    let mut read = Session::default();
    read.execute("file.open", &json!({"path":path})).unwrap();
    assert!(read.model.section("S").unwrap().formulation.f_bar);
    for name in ["Preload", "Locked", "Modes", "Direct", "Modal", "Zero"] {
        assert_eq!(s.result(name), read.result(name));
    }
    for (command, name) in [
        ("solve.nonlinear", "Locked"),
        ("solve.frequency", "Modes"),
        ("solve.transient", "Direct"),
        ("solve.transient", "Modal"),
        ("solve.transient", "Zero"),
    ] {
        read.execute(command, &json!({"step":name})).unwrap();
        assert_eq!(s.result(name), read.result(name));
    }
    if std::env::var_os("FIELDCRAFT_FBAR_EVIDENCE").is_none() {
        std::fs::remove_file(path).unwrap();
    }
}

#[test]
fn fbar_stvk_continuous_native_roundtrip() {
    let mut script: Value = serde_json::from_str(include_str!("fixtures/fbar_pretension.json")).unwrap();
    let commands = script.as_array_mut().unwrap();
    commands.retain(|c| {
        !(c["command"] == "mesh.node.add" && c["params"]["label"].as_u64().unwrap_or(0) >= 13)
            && !(c["command"] == "model.set.create" && c["params"]["name"] == "OtherCut")
            && !(c["command"] == "model.step.create" && ["Direct", "Modal", "Zero"].iter().any(|n| c["params"]["name"] == *n))
            && c["command"] != "solve.transient"
    });
    for c in commands {
        match c["command"].as_str().unwrap() {
            "model.material.create" => {
                c["params"].as_object_mut().unwrap().remove("hyperelastic");
            }
            "mesh.element.add" if c["params"]["label"] == 2 => {
                c["params"]["nodes"] = json!([5, 6, 7, 8, 9, 10, 11, 12]);
            }
            "model.pretension.create" => {
                c["params"]["face"] = json!("CutFace");
                c["params"]["second"] = json!("Ends");
            }
            _ => {}
        }
    }
    let commands = script.as_array_mut().unwrap();
    let at = commands.iter().position(|c| c["command"] == "solve.nonlinear").unwrap();
    commands.splice(at..at, [
        json!({"command":"model.step.create","params":{"name":"Unloaded","before":"Preload","procedure":{"type":"nonlinear_static","nonlinear_geometry":true},"boundary_conditions":["Ends","Guide"]}}),
        json!({"command":"model.step.create","params":{"name":"Before","before":"Preload","procedure":{"type":"frequency","modes":1},"base_state":"Unloaded","boundary_conditions":["Ends","Guide"]}}),
        json!({"command":"solve.nonlinear","params":{"step":"Unloaded"}}),
        json!({"command":"solve.frequency","params":{"step":"Before"}}),
    ]);
    let mut session = Session::default();
    session.run_script(&script).unwrap();
    assert!(session.model.material("M").unwrap().hyperelastic.is_none());
    let root = std::env::var_os("FIELDCRAFT_FBAR_EVIDENCE")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join(format!("fbar-stvk-{}", std::process::id())));
    std::fs::create_dir_all(&root).unwrap();
    let path = root.join("stvk-continuous.fieldcraft");
    session.execute("file.save_as", &json!({"path":path})).unwrap();
    let mut read = Session::default();
    read.execute("file.open", &json!({"path":path})).unwrap();
    assert_eq!(session.model, read.model);
    for name in ["Unloaded", "Before", "Preload", "Locked", "Modes"] {
        assert_eq!(session.result(name), read.result(name));
    }
    read.execute("solve.nonlinear", &json!({"step":"Locked"})).unwrap();
    read.execute("solve.frequency", &json!({"step":"Modes"})).unwrap();
    for name in ["Locked", "Modes"] {
        assert_eq!(session.result(name), read.result(name));
    }
    if std::env::var_os("FIELDCRAFT_FBAR_EVIDENCE").is_some() {
        std::fs::write(root.join("stvk-continuous.commands.json"), serde_json::to_vec_pretty(&script).unwrap()).unwrap();
    } else {
        std::fs::remove_file(path).unwrap();
    }
}
