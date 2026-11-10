//! Read-only independent reopening of a retained protocol fixture package.
//! No solver is run. Solver/Succeeded wire labels in this fixture are injected claims.
use std::collections::BTreeMap;
use std::path::{Component, Path};

use fieldcraft_engine::external::production::{FileResultStore, IoModelContext, IoSnapshotProvider, ResultStore};
use fieldcraft_engine::external::receive::{Admission, EntityMap, ModelBinding, ResultEnvelope};
use fieldcraft_engine::external::transaction::{ReceiveProfile, ReceiverHost, SnapshotProvider};
use serde_json::{Value, json};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn read(path: &Path) -> Result<Vec<u8>> {
    Ok(fieldcraft_io::fs::read_capped(path, 256 * 1024 * 1024)?)
}
fn decode<T: serde::de::DeserializeOwned>(root: &Path, name: &str) -> Result<T> {
    Ok(serde_json::from_slice(&read(&root.join(name))?)?)
}
fn require(ok: bool, why: &'static str) -> Result<()> {
    if ok { Ok(()) } else { Err(why.into()) }
}

fn reopen(root: &Path) -> Result<Value> {
    let manifest: Value = decode(root, "package.json")?;
    require(manifest["schema"] == "fieldcraft-external-fixture-package/1", "package schema")?;
    require(
        manifest["evidence_class"] == "protocol_fixture"
            && manifest["runtime_qualification"] == "unverified"
            && manifest["numerical_solve_verified"] == false
            && manifest["libuipc_called"] == false,
        "fixture evidence boundary",
    )?;
    let files: BTreeMap<String, String> = serde_json::from_value(manifest["files"].clone())?;
    require(files.len() == 9, "package file inventory")?;
    let mut total = 0usize;
    for (name, expected) in &files {
        require(Path::new(name).components().all(|c| matches!(c, Component::Normal(_))), "unsafe package path")?;
        let data = read(&root.join(name))?;
        total = total.checked_add(data.len()).ok_or("package byte count")?;
        require(total <= 512 * 1024 * 1024, "package byte cap")?;
        require(fieldcraft_io::sha256_hex(&data) == *expected, "package file checksum")?;
    }
    let result_id = manifest["result_id"].as_str().ok_or("result_id")?;
    let stored_name = format!("store/{}.json", result_id.strip_prefix("fieldcraft-result/").ok_or("identifier scheme")?);
    for name in [
        "input.fieldcraft",
        "fixture-source.txt",
        "programs/interface-fixture",
        "input-envelope.json",
        "entities.json",
        "binding.json",
        "admission.json",
        "request-profile.json",
        &stored_name,
    ] {
        require(files.contains_key(name), "missing required package file")?;
    }
    let original: ResultEnvelope = decode(root, "input-envelope.json")?;
    let entities: EntityMap = decode(root, "entities.json")?;
    let binding: ModelBinding = decode(root, "binding.json")?;
    let expected: Admission = decode(root, "admission.json")?;
    require(original.entities == entities && original.binding == binding, "original input binding or map changed")?;
    let program_digest = fieldcraft_io::identity::artifact_digest(&read(&root.join("programs/interface-fixture"))?);
    let artifacts = fieldcraft_io::identity::REQUIRED_ARTIFACT_ROLES.into_iter().map(|role| (role.into(), program_digest.clone())).collect();
    let model = fieldcraft_io::read(&read(&root.join("input.fieldcraft"))?)?;
    let context = IoModelContext::new(model.model, model.geometry, model.run, artifacts);
    let recomputed = IoSnapshotProvider.freeze(&context, &entities)?;
    require(recomputed == binding, "frozen model/map/program identity mismatch")?;
    require(fieldcraft_io::sha256_hex(&read(&root.join("fixture-source.txt"))?) == original.source_scene.sha256, "source bytes changed")?;
    let request: Value = decode(root, "request-profile.json")?;
    require(
        request
            == json!({
                "run": original.run, "goal": original.goal, "evidence_kind": original.evidence_kind,
                "source_scene": original.source_scene, "required_fields": original.required_fields,
                "contact_controls": null, "control_evidence": null, "require_effective": false,
                "policy_request": null, "require_policy_applied": false,
            }),
        "frozen request changed",
    )?;
    // new() only ensures the already inventoried directory exists. No publication takes place.
    let store = FileResultStore::new(root.join("store"))?;
    let reopened = store.reopen(result_id)?;
    require(reopened == original, "reopened payload differs from original input")?;
    let host = ReceiverHost::new(context, original.run.clone(), IoSnapshotProvider)?;
    let profile = ReceiveProfile {
        goal: original.goal,
        evidence_kind: original.evidence_kind,
        source_scene: original.source_scene.clone(),
        required_fields: original.required_fields.clone(),
        contact_controls: None,
        control_evidence: None,
        require_effective: false,
        policy_request: None,
        require_policy_applied: false,
    };
    let tx = host.begin(&original.run, entities, profile)?;
    let actual = tx.validate(&reopened)?;
    require(actual == expected, "admission differs on independent reopening")?;
    // Stored admission is data too: do not let a modified record diverge from validation.
    let stored: Value = decode(root, &stored_name)?;
    let stored_admission: Admission = serde_json::from_value(stored["admission"].clone())?;
    require(stored_admission == actual, "stored admission differs from recomputed admission")?;
    Ok(json!({
        "schema": "fieldcraft-external-fixture-reopen/1", "result_id": result_id,
        "run": original.run, "binding": recomputed, "wire_admission": actual,
        "reopened_equal": true, "admission_recomputed": true, "files_verified": files.len(),
        "evidence_class": "protocol_fixture", "runtime_qualification": "unverified",
        "numerical_solve_verified": false, "libuipc_called": false,
        "new_commit_performed": false, "program_sha256": program_digest,
    }))
}

fn main() -> Result<()> {
    let path = std::env::args().nth(1).ok_or("expected retained protocol fixture package directory")?;
    println!("{}", serde_json::to_string_pretty(&reopen(Path::new(&path))?)?);
    Ok(())
}
