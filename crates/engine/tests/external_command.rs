//! Formal command/type-13 protocol fixtures. No test here is a real libuipc solve.
mod external_support;
use external_support::Fixture;
use fieldcraft_engine::{EngineError, Session, schema::ProblemKind};
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

#[test]
fn remove_distinguishes_required_name_from_missing_result_and_retains_published_files() {
    let f = Fixture::new("formal-remove");
    let mut s = Session::default();
    s.model = f.model.model.clone();
    let missing = s.execute("results.external.remove", &json!({})).unwrap_err();
    assert!(matches!(missing, EngineError::Param(p) if p.path == "name" && p.kind == ProblemKind::Required));
    s.execute("results.external.receive", &f.params()).unwrap();
    s.execute("file.save_as", &json!({"path":f.root.join("saved.fieldcraft")})).unwrap();
    let before = s.external_results_summary();
    let absent = s.execute("results.external.remove", &json!({"name":"Fresh"})).unwrap_err();
    assert!(matches!(absent, EngineError::Params(ref message) if message == "no external result `Fresh`"));
    assert_eq!(s.external_results_summary(), before);
    assert!(!s.is_dirty());
    let files: Vec<_> = std::fs::read_dir(f.root.join("store")).unwrap().map(|p| p.unwrap().path()).collect();
    let contents: Vec<_> = files.iter().map(|p| std::fs::read(p).unwrap()).collect();
    assert_eq!(files.len(), 1);
    let removed = s.execute("results.external.remove", &json!({"name":"external-fixture"})).unwrap();
    assert_eq!(removed, json!({"removed":"external-fixture","published_files_deleted":false,"project_saved":false}));
    assert_eq!(s.external_results_summary(), json!([]));
    assert!(s.is_dirty());
    assert_eq!(std::fs::read_dir(f.root.join("store")).unwrap().count(), 1);
    assert_eq!(files.iter().map(|p| std::fs::read(p).unwrap()).collect::<Vec<_>>(), contents);
}

#[test]
fn replaced_envelope_names_publication_mismatch_and_does_not_replace_session() {
    let f = Fixture::new("formal-swapped-envelope");
    let mut s = Session::default();
    s.model = f.model.model.clone();
    s.execute("results.external.receive", &f.params()).unwrap();
    let mut record = s.external_result("external-fixture").unwrap().clone();
    // All IO-level digests remain self-consistent, but the frozen publication is unchanged.
    record.envelope_json.push(b' ');
    record.payload_sha256 = fieldcraft_io::sha256_hex(&record.envelope_json);
    record.result_id = format!("fieldcraft-result/{}", record.payload_sha256);
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
    assert_eq!(fieldcraft_io::read(&bytes).unwrap().results.len(), 1);
    let path = f.root.join("replaced-envelope.fieldcraft");
    std::fs::write(&path, bytes).unwrap();
    let before = s.external_results_summary();
    let error = s.execute("file.open", &json!({"path":path})).unwrap_err().to_string();
    assert!(error.contains("ExternalArchivePublicationMismatch:"), "{error}");
    assert!(!error.contains("binary field checksum"), "{error}");
    assert_eq!(s.model, f.model.model);
    assert_eq!(s.external_results_summary(), before);
}
