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
