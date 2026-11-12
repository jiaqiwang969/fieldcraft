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
    params: r#"{"path":"incoming/package.json","profile":"expected.json","name":"impact","store":"accepted-results"} — verify an external trajectory against the current model and an independently frozen receive profile; publish atomically and install a type 13 result. Does not launch a solver or save the project; use file.save next. Version 1 requires complete solver success. Version 2 can explicitly preserve a sealed failed/cancelled qualified prefix while retaining its failure; unknown diagnostics and missing required evidence are refused."#,
    form: Some(super::forms::EXTERNAL_RECEIVE),
    kind: CommandKind::Session,
    requires: &[],
    run: receive,
};
pub const STATIC_BASE_RECEIVE: CommandSpec = CommandSpec {
    id: "results.external.static_base.receive",
    label: "Receive External Static Base",
    params: r#"{"path":"incoming/static-base.json","profile":"static-base-profile.json","name":"closed-base"} — receive a dense kind-13 static base, verify its frozen model/mesh/backend binding, then recompute the modal package from K, M and the constraint map inside the product. Producer modal values are never trusted; runtime qualification remains unverified and frequency response stays not_run until damping evidence is supplied."#,
    form: Some(super::forms::EXTERNAL_STATIC_BASE_RECEIVE),
    kind: CommandKind::Session,
    requires: &[],
    run: receive_static_base,
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
        if s.external_results.remove(&name).is_none()
            && s.external_chunked_results.remove(&name).is_none()
            && s.external_static_base_results.remove(&name).is_none()
        {
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
    let expected_bytes = fieldcraft_io::fs::read_capped(Path::new(&expected_path), MAX_BYTES)?;
    #[derive(serde::Deserialize)]
    struct Schema {
        schema: String,
    }
    let schema: Schema = serde_json::from_slice(&expected_bytes).map_err(error)?;
    if schema.schema == crate::external::chunked_archive::REQUEST_SCHEMA {
        return receive_chunked(s, p, name, &expected_bytes, total);
    }
    let expected = ExpectedReceive::decode(&expected_bytes).map_err(error)?;
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

#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct StaticBaseReceiveProfile {
    schema: String,
    archive_key: String,
    binding: fieldcraft_io::identity::Binding,
    reference_shapes: Vec<Vec<f64>>,
    mode_count: usize,
}

/// Receive the kind-13 static-base extension through a product-owned path.
///
/// The profile is frozen independently of the producer package.  After exact
/// binding checks and the normal session identity check, only the profile's
/// reference shapes are used; all producer modal modes, residuals, MAC values
/// and pairing records are discarded before the product recomputes them.
fn receive_static_base(s: &mut Session, p: &Value) -> Result<Value> {
    let total = Instant::now();
    let name: String = super::req(p, "name")?;
    s.check_external_name(&name)?;
    let profile_path = super::file_param(s, p, "profile", Access::Read)?;
    let profile_bytes = fieldcraft_io::fs::read_capped(Path::new(&profile_path), MAX_BYTES)?;
    let profile: StaticBaseReceiveProfile = serde_json::from_slice(&profile_bytes).map_err(error)?;
    if profile.schema != "fieldcraft-external-static-base/receive-profile/1" {
        return Err(error("ExternalStaticBaseProfile: unsupported schema"));
    }
    if profile.archive_key != name || profile.binding.backend_version != fieldcraft_io::identity::BACKEND_VERSION {
        return Err(error("ExternalStaticBaseProfile: archive or backend binding mismatch"));
    }
    if profile.mode_count == 0 || profile.reference_shapes.is_empty() || profile.reference_shapes.len() < profile.mode_count {
        return Err(error("ExternalStaticBaseProfile: mode count or reference shapes"));
    }
    if profile.binding.artifacts.is_empty() || profile.binding.artifacts.values().any(|digest| !fieldcraft_io::identity::digest_valid(digest)) {
        return Err(error("ExternalStaticBaseProfile: artifact identity"));
    }
    let path = super::file_param(s, p, "path", Access::Read)?;
    let bytes = fieldcraft_io::fs::read_capped(Path::new(&path), MAX_BYTES)?;
    let incoming: fieldcraft_io::external_results::ArchivedStaticBase = serde_json::from_slice(&bytes).map_err(error)?;
    if incoming.archive_key() != name || incoming.binding() != &profile.binding {
        return Err(error("ExternalStaticBaseProfile: package binding differs from frozen expectation"));
    }
    let receiver = receiver_identity();
    let receiver_sha256 = receiver
        .program
        .as_ref()
        .map(|program| program.sha256.clone())
        .ok_or_else(|| error("ExternalStaticBaseProfile: receiving executable identity unavailable"))?;
    if profile.binding.artifacts.get("receiver") != Some(&receiver_sha256) {
        return Err(error(format!(
            "ExternalStaticBaseProfile: receiver artifact mismatch measured={} expected={}",
            receiver_sha256,
            profile.binding.artifacts.get("receiver").map(String::as_str).unwrap_or("missing")
        )));
    }
    // Validate the producer package and the current model/mesh before deriving
    // any product result.  This is the authoritative identity gate.
    incoming.validate_binding(&s.model, &s.sources, &s.run).map_err(error)?;
    let producer_payload_sha256 = incoming.payload_sha256().to_string();
    let mut base = incoming.external_static_base().clone();
    // Qualification is an independent decision.  A producer claim is
    // never propagated through the product-owned receive path.
    base.runtime_qualification = "unverified".into();
    let modal = base.compute_modal_with_count(profile.reference_shapes, profile.mode_count).map_err(error)?;
    let mode_count = modal.modes.len();
    let max_residual = modal.modes.iter().map(|mode| mode.residual).fold(0.0, f64::max);
    let mass_off_diagonal =
        modal.mass_gram.values.iter().enumerate().filter(|(i, _)| i / mode_count != i % mode_count).map(|(_, value)| value.abs()).fold(0.0, f64::max);
    let min_mac = modal.pairs.iter().map(|pair| pair.mac).fold(1.0, f64::min);
    let min_mac_margin = modal.pairs.iter().map(|pair| pair.margin).fold(1.0, f64::min);
    base.modal = Some(modal);
    // Re-archive the product-owned payload so the result id and record digest
    // describe what the product actually computed, while retaining the original
    // source/admission/context evidence byte-for-byte.
    let product = fieldcraft_io::external_results::ArchivedStaticBase::from_payload(
        incoming.archive_key().to_string(),
        incoming.binding().clone(),
        incoming.entity_map().clone(),
        base,
        incoming.admission_json().to_vec(),
        incoming.context_json().to_vec(),
        incoming.source_json().to_vec(),
    )
    .map_err(error)?;
    let product_payload_sha256 = product.payload_sha256().to_string();
    let modal_package = product
        .external_static_base()
        .modal
        .as_ref()
        .map(crate::external::session::static_base_modal_summary)
        .ok_or_else(|| error("ExternalStaticBase: product modal package missing"))?;
    s.keep_external_static_base(product).map_err(error)?;
    Ok(json!({
        "archive_key": name,
        "kind": 13,
        "schema": fieldcraft_io::external_results::STATIC_BASE_SCHEMA,
        "producer_payload_sha256": producer_payload_sha256,
        "product_payload_sha256": product_payload_sha256,
        "modal_source": "product_recomputed_from_K_M_T",
        "modal_package": modal_package,
        "modal_mode_count": mode_count,
        "modal_max_residual": max_residual,
        "modal_max_mass_off_diagonal": mass_off_diagonal,
        "modal_min_mac": min_mac,
        "modal_min_pair_margin": min_mac_margin,
        "receiver_program_sha256": receiver_sha256,
        "runtime_qualification": "unverified",
        "frequency_response": "not_run",
        "session_installed": true,
        "project_saved": false,
        "timings_seconds": {"receive_and_recompute": total.elapsed().as_secs_f64()}
    }))
}
fn receive_chunked(s: &mut Session, p: &Value, name: String, expected_bytes: &[u8], total: Instant) -> Result<Value> {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (s, p, name, expected_bytes, total);
        Err(error("chunked archives require a native streaming filesystem"))
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        use crate::external::chunked_archive as large;
        let request = large::Request::decode(expected_bytes).map_err(error)?;
        let path = super::file_param(s, p, "path", Access::Read)?;
        let path = std::path::absolute(path).map_err(error)?;
        let base = path.parent().ok_or_else(|| error("external package has no directory"))?;
        let base = std::fs::canonicalize(base).map_err(error)?;
        let manifest = fieldcraft_io::fs::read_capped(&path, large::METADATA_CAP)?;
        let store = super::file_param(s, p, "store", Access::Write)?;
        s.checked_path(&store, Access::Read)?;
        let model = IoModelContext::new(s.model.clone(), s.sources.clone(), s.run.clone(), request.expected.artifacts.clone());
        let record = large::receive(name.clone(), model, request, manifest, receiver_identity(), Path::new(&store), |spec| {
            let path = base.join(&spec.locator);
            let path = s.checked_path(&path.to_string_lossy(), Access::Read).map_err(|e| ContractError::Provider(e.to_string()))?;
            let path = std::fs::canonicalize(path).map_err(|e| ContractError::Provider(e.to_string()))?;
            if !path.starts_with(&base) {
                return Err(ContractError::Invalid("external package source escaped through a link"));
            }
            let f = std::fs::File::open(path).map_err(|e| ContractError::Provider(e.to_string()))?;
            let meta = f.metadata().map_err(|e| ContractError::Provider(e.to_string()))?;
            if !meta.is_file() || meta.len() != spec.bytes {
                return Err(ContractError::Invalid("ExternalSourceChanged: file size or kind"));
            }
            Ok(Box::new(f))
        })
        .map_err(error)?;
        let id = record.result_id().to_string();
        let admission: crate::external::receive::Admission = serde_json::from_slice(record.admission_json()).map_err(error)?;
        let context: large::Context = serde_json::from_slice(record.context_json()).map_err(error)?;
        // The already checked immutable record is installed once. Reopening separately
        // performs the same source/admission checks; no solve-success claim is invented.
        s.external_chunked_results.insert(name.clone(), Arc::new(record));
        s.results_stored = s.results_stored.wrapping_add(1);
        Ok(json!({"archive_key":name,"result_id":id,"schema":"fieldcraft-external-trajectory/2","admission":admission,
            "publication":context.publication,"observed_frames":context.summary.observed_frames,"session_installed":true,
            "project_saved":false,"runtime_qualification":"unverified","timings_seconds":{"receive_total":total.elapsed().as_secs_f64()}}))
    }
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
