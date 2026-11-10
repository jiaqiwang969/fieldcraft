//! Receive external trajectories through the common command registry and file roots.
use crate::external::context::{MAX_BYTES, ReceiverIdentity};
use crate::external::package::{ExpectedReceive, PackageManifest, PreparedReceive};
use crate::external::production::{FileResultStore, IoModelContext};
use crate::external::{ContractError, EvidenceKind, SourceRef, archive};
use crate::{CommandKind, CommandSpec, EngineError, Result, Session};
use fieldcraft_io::roots::Access;
use serde_json::{Value, json};
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

pub const RECEIVE: CommandSpec = CommandSpec {
    id: "results.external.receive",
    label: "Receive External Results",
    params: r#"{"path":"incoming/package.json","profile":"expected.json","name":"impact","store":"accepted-results"} — verify an external trajectory against the current model and an independently frozen receive profile; publish atomically and install a type 13 result. Does not launch a solver or save the project; use file.save next. Partial/failed/cancelled/unknown runs and missing required evidence are refused."#,
    form: Some(super::forms::EXTERNAL_RECEIVE),
    kind: CommandKind::Session,
    requires: &[],
    run: receive,
};
pub const LIST: CommandSpec = CommandSpec {
    id: "results.external.list",
    label: "External Results",
    params: r#"{} — external archive names, original receiver identifiers, source run, availability and four states; stale bindings and unverified qualification remain explicit"#,
    form: Some(&[]),
    kind: CommandKind::Session,
    requires: &[],
    run: |s, _| Ok(s.external_results_summary()),
};
pub const REMOVE: CommandSpec = CommandSpec {
    id: "results.external.remove",
    label: "Remove External Result",
    params: r#"{"name":"impact"} — remove a result from this session; existing published files are retained. Save the project separately."#,
    form: Some(super::forms::EXTERNAL_REMOVE),
    kind: CommandKind::Session,
    requires: &[],
    run: |s, p| {
        let name: String = super::req(p, "name")?;
        if s.external_results.remove(&name).is_none() {
            return Err(EngineError::Params(format!("no external result `{}`", name.chars().take(80).collect::<String>())));
        }
        s.results_stored = s.results_stored.wrapping_add(1);
        Ok(json!({"removed":name,"published_files_deleted":false,"project_saved":false}))
    },
};
fn error(e: impl std::fmt::Display) -> EngineError {
    EngineError::Params(e.to_string())
}
fn receive(s: &mut Session, p: &Value) -> Result<Value> {
    let total = Instant::now();
    let name: String = super::req(p, "name")?;
    s.check_external_name(&name)?;
    let expected_path = super::file_param(s, p, "profile", Access::Read)?;
    // Freeze independent expectations before parsing any producer candidate.
    let expected = ExpectedReceive::decode(&fieldcraft_io::fs::read_capped(Path::new(&expected_path), MAX_BYTES)?).map_err(error)?;
    let path = super::file_param(s, p, "path", Access::Read)?;
    let path = std::path::absolute(path).map_err(error)?;
    let base = path.parent().ok_or_else(|| error("external package has no directory"))?;
    let manifest: PackageManifest = serde_json::from_slice(&fieldcraft_io::fs::read_capped(&path, MAX_BYTES)?).map_err(error)?;
    let store_path = super::file_param(s, p, "store", Access::Write)?;
    // Reading the published bytes is a separate permission from creating the store file.
    s.checked_path(&store_path, Access::Read)?;
    let receiver = receiver_identity();
    let model = IoModelContext::new(s.model.clone(), s.sources.clone(), s.run.clone(), expected.artifacts.clone());
    let checked = PreparedReceive::read(model, expected, manifest, receiver, |name, max| {
        let path = base.join(name);
        let checked = s.checked_path(&path.to_string_lossy(), Access::Read).map_err(|e| ContractError::Provider(e.to_string()))?;
        // Roots applies even to identity-only sources. Symlinks cannot escape the package.
        #[cfg(not(target_arch = "wasm32"))]
        if !std::fs::canonicalize(&checked)
            .map_err(|e| ContractError::Provider(e.to_string()))?
            .starts_with(std::fs::canonicalize(base).map_err(|e| ContractError::Provider(e.to_string()))?)
        {
            return Err(ContractError::Invalid("external package source escaped through a link"));
        }
        fieldcraft_io::fs::read_capped(Path::new(&checked), max).map_err(|e| ContractError::Provider(e.to_string()))
    })
    .map_err(error)?;
    let parse_seconds = total.elapsed().as_secs_f64();
    let publication_start = Instant::now();
    let published = checked.publish(Arc::new(FileResultStore::new(&store_path).map_err(error)?)).map_err(error)?;
    let id = published.raw.result_id.clone();
    let admission = published.admission.clone();
    let publication = published.context.publication.clone();
    let after = || -> Result<_> {
        let record = archive::from_published(name.clone(), published).map_err(error)?;
        let context = IoModelContext::new(s.model.clone(), s.sources.clone(), s.run.clone(), record.binding.artifacts.clone());
        archive::reopen(&record, context).map_err(error)?;
        Ok(record)
    };
    let record = after().map_err(|e| error(format!("ExternalPublishedNotInstalled: result_id={id}; project_saved=false; {e}")))?;
    let publication_seconds = publication_start.elapsed().as_secs_f64();
    // No fallible work follows the single Session installation.
    s.external_results.insert(name.clone(), Arc::new(record));
    s.results_stored = s.results_stored.wrapping_add(1);
    Ok(json!({"archive_key":name,"result_id":id,"admission":admission,"publication":publication,"session_installed":true,"project_saved":false,
        "runtime_qualification":"unverified","timings_seconds":{"parse_and_check":parse_seconds,"publish_and_archive":publication_seconds,"total":total.elapsed().as_secs_f64()}}))
}
fn receiver_identity() -> ReceiverIdentity {
    #[cfg(not(target_arch = "wasm32"))]
    if let Ok(sha256) = crate::external::program_identity::current_program_sha256() {
        return ReceiverIdentity {
            source_commit: None,
            program: Some(SourceRef {
                sha256,
                locator: "receiving-program".into(),
                observer: "running executable opened once and streamed through a 64 KiB SHA256 buffer at receive".into(),
                kind: EvidenceKind::Solver,
            }),
            unknown_reason: Some(
                "source commit is not embedded in this executable; measured program identity is retained separately from the producer".into(),
            ),
        };
    }
    ReceiverIdentity { source_commit: None, program: None, unknown_reason: Some("current receiving executable cannot be read on this host".into()) }
}
