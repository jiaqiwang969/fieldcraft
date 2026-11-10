//! Formal command/type-13 protocol fixtures. No test here is a real libuipc solve.
mod external_support;
use external_support::Fixture;
use fieldcraft_engine::Session;
use serde_json::json;

#[test]
fn formal_receive_save_and_reopen_keeps_original_bytes_and_states() {
    let f = Fixture::new("formal-positive");
    let mut s = Session::default();
    s.model = f.model.model.clone();
    let out = s.execute("results.external.receive", &f.params()).unwrap();
    assert_eq!(out["session_installed"], true);
    assert_eq!(out["project_saved"], false);
    assert_eq!(out["runtime_qualification"], "unverified");
    assert!(s.is_dirty());
    assert!(s.execute("results.external.receive", &f.params()).is_err());
    let record = s.external_result("external-fixture").unwrap().clone();
    assert_eq!(record.envelope_json, serde_json::to_vec(&f.envelope).unwrap());
    let saved = f.root.join("saved.fieldcraft");
    s.execute("file.save_as", &json!({"path":saved})).unwrap();
    assert!(!s.is_dirty());
    let mut read = Session::default();
    let reply = read.execute("file.open", &json!({"path":saved})).unwrap();
    assert_eq!(read.external_result("external-fixture").unwrap(), &record);
    assert!(!read.is_dirty());
    assert_eq!(reply["external_results"][0]["kind"], 13);
    assert_eq!(reply["external_results"][0]["admission"]["commit"], "not_attempted");
    assert_eq!(reply["external_results"][0]["publication"]["state"], "committed");
    assert!(read.execute("results.contour", &json!({"step":"external-fixture","field":"von_mises"})).is_err());
    // Current-model query refuses changed identity; no relabeling or stale field exposure.
    read.model.set_node(1, [0.1, 0., 0.]).unwrap();
    assert!(read.external_result("external-fixture").is_err());
    assert_eq!(read.external_results_summary()[0]["stale"], true);
    assert!(read.execute("file.save_as", &json!({"path":f.root.join("changed.fieldcraft")})).is_err());
    read.execute("file.save_as", &json!({"path":f.root.join("lean.fieldcraft"),"results":false})).unwrap();
    assert!(fieldcraft_io::read(&std::fs::read(f.root.join("lean.fieldcraft")).unwrap()).unwrap().results.is_empty());
}
#[test]
fn external_context_failure_does_not_replace_an_open_session() {
    let f = Fixture::new("formal-fallible-open");
    let mut s = Session::default();
    s.model = f.model.model.clone();
    s.execute("results.external.receive", &f.params()).unwrap();
    let mut record = s.external_result("external-fixture").unwrap().clone();
    let mut c: serde_json::Value = serde_json::from_slice(&record.context_json).unwrap();
    c["schema"] = json!("unsupported-context/2");
    record.context_json = serde_json::to_vec(&c).unwrap();
    record.record_sha256 = record.record_digest().unwrap();
    let definition = record.definition().unwrap();
    let bytes = fieldcraft_io::write_with(
        &f.model.model,
        &f.model.geometry,
        &[],
        &fieldcraft_io::JournalHeader::default(),
        &fieldcraft_io::View::default(),
        &fieldcraft_io::Extras {
            results: vec![(
                "external-fixture",
                fieldcraft_io::results::StoredRef { definition, result: fieldcraft_io::results::SolvedRef::External(&record) },
            )],
            ..Default::default()
        },
    )
    .unwrap();
    let bad = f.root.join("bad.fieldcraft");
    std::fs::write(&bad, bytes).unwrap();
    let mut read = Session::default();
    read.model.set_node(99, [2., 3., 4.]).unwrap();
    let before = read.model.clone();
    let error = read.execute("file.open", &json!({"path":bad})).unwrap_err().to_string();
    assert!(error.contains("ExternalArchiveContext"), "{error}");
    assert_eq!(before, read.model);
    assert_eq!(read.external_results_summary(), json!([]));
}
#[test]
fn formal_model_mismatch_and_file_roots_refuse_without_installing_or_publishing() {
    let f = Fixture::new("formal-reject");
    let mut s = Session::default();
    s.model = f.model.model.clone();
    s.model.set_node(1, [0.1, 0., 0.]).unwrap();
    assert!(s.execute("results.external.receive", &f.params()).is_err());
    assert_eq!(s.external_results_summary(), json!([]));
    assert!(!f.root.join("store").exists());
    let mut params = f.params();
    params["name"] = json!(" ");
    assert!(s.execute("results.external.receive", &params).is_err());
    s.model = f.model.model.clone();
    s.file_access = fieldcraft_io::roots::Roots::new(None, None).unwrap();
    assert!(s.execute("results.external.receive", &f.params()).is_err());
    assert!(!f.root.join("store").exists());
    assert_eq!(s.external_results_summary(), json!([]));
}
