//! Receive external trajectories through the common command registry and file roots.
use crate::external::context::{MAX_BYTES, ReceiverIdentity};
use crate::external::load_transfer::{self, ArchiveBundle, LoadTransferArchive};
use crate::external::package::{ExpectedReceive, PackageManifest, PreparedReceive};
use crate::external::production::{FileResultStore, IoModelContext};
use crate::external::{ContractError, EvidenceKind, SourceRef, archive};
use crate::{CommandKind, CommandSpec, EngineError, Result, Session};
use fieldcraft_io::roots::Access;
use fieldcraft_model::{Distribution, Load, Procedure, Set, Step, Target};
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

const LOAD_HISTORY_TIME_TOLERANCE_S: f64 = 1e-12;

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

pub const LOAD_TRANSFER_RECEIVE: CommandSpec = CommandSpec {
    id: "results.external.load_transfer.receive",
    label: "Receive Load Transfer",
    params: r#"{"path":"load-transfer.json","name":"support-load","project"?:"support.fieldcraft"} — receive a result-kind-13 load-transfer bundle, recompute applicability from raw activation frames, verify model and program identities, install it through the external-results entry, and optionally save it into the project"#,
    form: Some(super::forms::LOAD_TRANSFER_RECEIVE),
    kind: CommandKind::Session,
    requires: &[],
    run: receive_load_transfer,
};

pub const LOAD_TRANSFER_APPLY: CommandSpec = CommandSpec {
    id: "results.external.load_transfer.apply",
    label: "Apply Mapped Load History",
    params: r#"{"name":"support-load","step"?:"support-load-transient","boundary_conditions"?:["clamp"],"integration"?:{"type":"average_acceleration"|"linear_acceleration"|{"type":"hht","alpha":-0.1}},"method"?:{"type":"direct"|{"type":"modal","modes":8}},"damping"?:{"alpha":0,"beta":0,"loss_factor":0},"output_every"?:1} — recheck conservation, then add each nonzero mapped nodal component as a native structural force and tabular amplitude. An existing transient step is reused with its boundary conditions, integration, method, damping, output cadence and other loads unchanged; a new step accepts existing boundary-condition names and warns when none are supplied. Select consistent or lumped mass at solve time."#,
    form: None,
    kind: CommandKind::Model,
    requires: &[],
    run: apply_load_transfer,
};
fn error(e: impl std::fmt::Display) -> EngineError {
    EngineError::Params(e.to_string())
}

fn load_transfer_bundle(bytes: &[u8]) -> Result<ArchiveBundle> {
    if load_transfer::is_external_envelope(bytes) {
        return load_transfer::decode_external_envelope(bytes).map_err(error);
    }
    serde_json::from_slice(bytes).map_err(error)
}

fn current_entity_map(s: &Session) -> Result<fieldcraft_io::identity::EntityMap> {
    let nodes: Vec<u32> = s.model.nodes().keys().copied().collect();
    let index: BTreeMap<u32, usize> = nodes.iter().enumerate().map(|(index, label)| (*label, index)).collect();
    let elements = s
        .model
        .elements()
        .iter()
        .map(|(target, element)| {
            let source_nodes = element
                .nodes
                .iter()
                .map(|label| {
                    index.get(label).copied().ok_or_else(|| {
                        error(format!("model element {target} references missing node {label}; entity mapping refused without a panic"))
                    })
                })
                .collect::<Result<Vec<_>>>()?;
            Ok(fieldcraft_io::identity::MappedElement { target: *target, kind: element.kind, source_nodes })
        })
        .collect::<Result<Vec<_>>>()?;
    Ok(fieldcraft_io::identity::EntityMap { nodes, elements })
}

fn receiver_program_sha256() -> Result<String> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        crate::external::program_identity::current_program_sha256()
            .map_err(|cause| error(format!("cannot measure receiving program identity: {cause}")))
    }
    #[cfg(target_arch = "wasm32")]
    {
        Err(error("cannot measure receiving program identity on this target"))
    }
}

fn load_transfer_source_artifacts(identity: &load_transfer::LoadIdentity) -> Result<BTreeMap<String, String>> {
    identity.validate_program_digests().map_err(error)?;
    Ok(BTreeMap::from([("producer".to_string(), identity.producer_sha256.clone()), ("adapter".to_string(), identity.adapter_sha256.clone())]))
}

fn load_transfer_artifacts(archive: &LoadTransferArchive, receiver_sha256: &str) -> Result<BTreeMap<String, String>> {
    let mut artifacts = load_transfer_source_artifacts(&archive.identity)?;
    if !fieldcraft_io::identity::digest_valid(receiver_sha256) {
        return Err(error("receiving program identity has invalid SHA256"));
    }
    artifacts.insert("receiver".to_string(), receiver_sha256.to_string());
    Ok(artifacts)
}

/// Apply is allowed to run in a different executable from receive.  The producer and adapter
/// artifacts are part of the incoming record and must remain identical; the receiver artifact
/// is a historical fact retained in the project, while the current executable is measured again
/// and reported separately.
fn validate_load_transfer_apply_identity(
    binding_artifacts: &BTreeMap<String, String>,
    identity: &load_transfer::LoadIdentity,
    current_receiver_sha256: &str,
) -> Result<String> {
    let source_artifacts = load_transfer_source_artifacts(identity)?;
    for role in ["producer", "adapter"] {
        if binding_artifacts.get(role) != source_artifacts.get(role) {
            return Err(error(format!("load-transfer {role} identity does not match the received record")));
        }
    }
    let received_receiver_sha256 =
        binding_artifacts.get("receiver").ok_or_else(|| error("load-transfer record is missing the receiving program identity"))?;
    if !fieldcraft_io::identity::digest_valid(received_receiver_sha256) {
        return Err(error("load-transfer record has an invalid receiving program identity"));
    }
    if !fieldcraft_io::identity::digest_valid(current_receiver_sha256) {
        return Err(error("current receiving program identity has invalid SHA256"));
    }
    Ok(received_receiver_sha256.clone())
}

fn archive_binding(
    s: &Session,
    archive: &LoadTransferArchive,
    map: &fieldcraft_io::identity::EntityMap,
    receiver_sha256: &str,
) -> Result<fieldcraft_io::identity::Binding> {
    let binding = fieldcraft_io::identity::Binding::freeze(
        &s.model,
        &s.sources,
        &s.run,
        map,
        load_transfer_artifacts(archive, receiver_sha256)?,
        fieldcraft_io::identity::BACKEND_VERSION,
    )
    .map_err(error)?;
    if archive.identity.model_revision != binding.snapshot.model_revision || archive.identity.mesh_digest != binding.snapshot.reference_mesh_digest {
        return Err(error("load-transfer identity does not match the open project"));
    }
    Ok(binding)
}

fn conservation_response(decoded: &load_transfer::DecodedArchive) -> Result<Value> {
    let mut frames = Vec::with_capacity(decoded.archive.frames.len());
    let virtual_displacements = vec![[0.0, 0.0, 0.0]; decoded.archive.mapping.nodes.len()];
    for (frame_index, (frame, mapped)) in decoded.archive.frames.iter().zip(&decoded.mapped_frames).enumerate() {
        let report = load_transfer::check_conservation(frame, mapped, &decoded.archive.mapping, &virtual_displacements).map_err(error)?;
        if !report.passed {
            return Err(conservation_rejection(frame_index, &report));
        }
        frames.push(report);
    }
    let impulse =
        load_transfer::check_time_integrals(&decoded.archive.frames, &decoded.mapped_frames, decoded.archive.mapping.origin_m).map_err(error)?;
    if !impulse.passed {
        let force = max_abs3(impulse.impulse_residual_ns);
        let angular = max_abs3(impulse.angular_impulse_residual_nms);
        let (item, actual, tolerance) = if force > impulse.impulse_tolerance_ns {
            ("time_integral_force", force, impulse.impulse_tolerance_ns)
        } else {
            ("time_integral_moment", angular, impulse.angular_impulse_tolerance_nms)
        };
        return Err(error(format!(
            "load-transfer conservation rejected at time integral: item={item}, actual={actual:.17e}, tolerance={tolerance:.17e}; impulse_residual={:?}, angular_impulse_residual={:?}",
            impulse.impulse_residual_ns, impulse.angular_impulse_residual_nms
        )));
    }
    Ok(json!({"frames":frames,"time_integrals":impulse,"all_frames_passed":true,"time_integrals_passed":true}))
}

fn max_abs3(value: [f64; 3]) -> f64 {
    value.into_iter().map(f64::abs).fold(0.0, f64::max)
}

fn conservation_rejection(frame_index: usize, report: &load_transfer::ConservationReport) -> EngineError {
    let force = max_abs3(report.force_residual_n);
    let moment = max_abs3(report.moment_residual_nm);
    let (item, actual, tolerance) = if force > report.force_tolerance_n {
        ("force", force, report.force_tolerance_n)
    } else if moment > report.moment_tolerance_nm {
        ("moment", moment, report.moment_tolerance_nm)
    } else {
        ("virtual_work", report.virtual_work_residual_j.abs(), report.virtual_work_tolerance_j)
    };
    error(format!(
        "load-transfer conservation rejected at frame {frame_index}: item={item}, actual={actual:.17e}, tolerance={tolerance:.17e}; force_residual={:?}, moment_residual={:?}, virtual_work_residual={:.17e}",
        report.force_residual_n, report.moment_residual_nm, report.virtual_work_residual_j
    ))
}

fn receive_load_transfer(s: &mut Session, p: &Value) -> Result<Value> {
    let name: String = super::req(p, "name")?;
    s.check_external_name(&name)?;
    let path = super::file_param(s, p, "path", Access::Read)?;
    let bytes = fieldcraft_io::fs::read_capped(Path::new(&path), MAX_BYTES)?;
    let bundle = load_transfer_bundle(&bytes)?;
    let decoded = LoadTransferArchive::decode(&bundle).map_err(error)?;
    load_transfer::validate_receive_status(&decoded.archive).map_err(error)?;
    if decoded.archive.identity.archive_key != name {
        return Err(error("load-transfer archive_key must equal the receiving result name"));
    }
    if matches!(decoded.archive.qualification, load_transfer::RuntimeQualification::Verified { .. }) {
        return Err(error("load-transfer receive rejects a producer-claimed runtime qualification; independent qualification is receiver-owned"));
    }
    decoded.archive.identity.validate_program_digests().map_err(error)?;
    let receiver_sha256 = receiver_program_sha256()?;
    let conservation = conservation_response(&decoded)?;
    let backend_status = load_transfer::backend_status(&decoded.archive);
    let map = current_entity_map(s)?;
    let binding = archive_binding(s, &decoded.archive, &map, &receiver_sha256)?;
    let envelope_json = load_transfer::encode_external_envelope(&bundle).map_err(error)?;
    let payload_sha256 = fieldcraft_io::sha256_hex(&envelope_json);
    let qualification = serde_json::to_value(&decoded.archive.qualification).map_err(error)?;
    let admission_json = serde_json::to_vec(&json!({
        "schema":"fieldcraft-load-transfer-admission/1",
        "transport":"verified",
        "runtime_qualification":qualification,
        "geometry_status":decoded.archive.applicability.geometry_status,
        "adoption_status":decoded.archive.applicability.adoption_status,
        "backend_status":backend_status.clone(),
    }))
    .map_err(error)?;
    let context_json = serde_json::to_vec(&json!({
        "schema":"fieldcraft-load-transfer-context/1",
        "archive_key":name,
        "result_kind":load_transfer::RESULT_KIND,
        "frames":decoded.archive.frames.len(),
        "source":"results.external.load_transfer.receive",
        "backend_status":backend_status.clone(),
    }))
    .map_err(error)?;
    let record = fieldcraft_io::external_results::ArchivedTrajectory::new(fieldcraft_io::external_results::ArchivedTrajectoryParts {
        archive_key: name.clone(),
        binding,
        entity_map: map,
        result_id: format!("fieldcraft-result/{payload_sha256}"),
        payload_sha256,
        envelope_json,
        admission_json,
        context_json,
    })
    .map_err(error)?;
    let result_id = record.result_id.clone();
    s.external_results.insert(name.clone(), Arc::new(record));
    s.results_stored = s.results_stored.wrapping_add(1);
    let mut response = json!({
        "archive_key":name,
        "result_id":result_id,
        "result_kind":load_transfer::RESULT_KIND,
        "session_installed":true,
        "project_saved":false,
        "conservation":conservation,
        "applicability":decoded.archive.applicability,
        "runtime_qualification":decoded.archive.qualification,
        "program_identity": {"producer_sha256": decoded.archive.identity.producer_sha256, "adapter_sha256": decoded.archive.identity.adapter_sha256, "receiver_sha256": receiver_sha256, "current_receiver_sha256": receiver_sha256},
        "crash":decoded.archive.crash,
        "backend_status":backend_status,
        "apply_allowed":backend_status.apply_allowed,
        "mapped_frames":decoded.mapped_frames.len(),
        "node_loads_per_frame":decoded.archive.mapping.nodes.len(),
        "structural_step":if backend_status.apply_allowed {
            "ready_for_results.external.load_transfer.apply"
        } else {
            "not_ready_for_results.external.load_transfer.apply"
        },
    });
    if p.get("project").is_some() {
        let project = super::file_param(s, p, "project", Access::Write)?;
        let saved = super::file::save_to(s, &project, super::file::Keep { results: true, geometry: false })?;
        response["project_saved"] = json!(true);
        response["project"] = saved;
    }
    Ok(response)
}

fn apply_load_transfer(s: &mut Session, p: &Value) -> Result<Value> {
    let name: String = super::req(p, "name")?;
    let step_name = p.get("step").and_then(Value::as_str).map(str::to_owned).unwrap_or_else(|| format!("{name}-transient"));
    let record = s.external_results.get(&name).cloned().ok_or_else(|| error(format!("external result `{name}` not found")))?;
    let generated_prefix = format!("lt_{name}_");
    if s.model.step(&step_name).is_some()
        && (s.model.sets().keys().any(|candidate| candidate.starts_with(&generated_prefix))
            || s.model.loads().keys().any(|candidate| candidate.starts_with(&generated_prefix)))
    {
        return Err(error(format!("step `{step_name}` already exists with load-transfer objects for `{name}`")));
    }
    let bundle = load_transfer::decode_external_envelope(&record.envelope_json).map_err(error)?;
    let decoded = LoadTransferArchive::decode(&bundle).map_err(error)?;
    load_transfer::validate_receive_status(&decoded.archive).map_err(error)?;
    let backend_status = load_transfer::backend_status(&decoded.archive);
    if !backend_status.apply_allowed {
        return Err(error(format!(
            "load-transfer apply refused backend status: state={:?}, exit_code={:?}, signal={:?}, first_error={:?}, last_completed_frame={:?}, last_persisted_frame={:?}, requested_last_frame={:?}, coverage_through_frame={:?}",
            backend_status.state,
            backend_status.exit_code,
            backend_status.signal,
            backend_status.first_error,
            backend_status.last_completed_frame,
            backend_status.last_persisted_frame,
            backend_status.requested_last_frame,
            backend_status.coverage_through_frame,
        )));
    }
    if matches!(decoded.archive.qualification, load_transfer::RuntimeQualification::Verified { .. }) {
        return Err(error("load-transfer apply rejects a producer-claimed runtime qualification; independent qualification is receiver-owned"));
    }
    let current_receiver_sha256 = receiver_program_sha256()?;
    let received_receiver_sha256 =
        validate_load_transfer_apply_identity(&record.binding.artifacts, &decoded.archive.identity, &current_receiver_sha256)?;
    record.validate_binding(&s.model, &s.sources, &s.run).map_err(error)?;
    let conservation = conservation_response(&decoded)?;
    if decoded.archive.applicability.geometry_status == load_transfer::ApplicabilityStatus::Inapplicable {
        return Err(error("load-transfer geometry applicability is inapplicable; mapped one-way structural adoption is refused"));
    }
    let frame_count = decoded.mapped_frames.len();
    if frame_count < 2 || frame_count > u32::MAX as usize {
        return Err(error("a structural load history needs 2..=u32::MAX frames"));
    }
    let first_time = decoded.archive.frames[0].time_s;
    if !first_time.is_finite() || first_time.abs() > 1e-12 {
        return Err(error(format!("structural load history must start at t=0, got {first_time:.17e}")));
    }
    let mut interval: Option<f64> = None;
    for pair in decoded.archive.frames.windows(2) {
        let value = pair[1].time_s - pair[0].time_s;
        if !value.is_finite() || value <= 0.0 {
            return Err(error("structural load history time axis must be strictly increasing"));
        }
        if let Some(expected) = interval {
            let tolerance = LOAD_HISTORY_TIME_TOLERANCE_S * expected.abs().max(1.0);
            if (value - expected).abs() > tolerance {
                return Err(error("structural transient requires a uniform load-history time step"));
            }
        } else {
            interval = Some(value);
        }
    }
    let last_time = decoded.archive.frames.last().ok_or_else(|| error("empty structural load-history frames"))?.time_s;
    let time_step = (last_time - first_time) / (frame_count - 1) as f64;
    if !time_step.is_finite() || time_step <= 0.0 {
        return Err(error("structural load history has an invalid total duration"));
    }
    let requested_integration = super::opt::<fieldcraft_model::dynamics::Integration>(p, "integration")?;
    let requested_method = super::opt::<fieldcraft_model::dynamics::Method>(p, "method")?;
    let requested_damping = super::opt::<fieldcraft_model::harmonic::Damping>(p, "damping")?;
    let requested_output_every = super::opt::<u32>(p, "output_every")?;
    let requested_boundary_conditions = super::opt::<BTreeSet<String>>(p, "boundary_conditions")?;
    let existing_step = s.model.step(&step_name).cloned();
    if let Some(step) = &existing_step
        && !matches!(step.procedure, fieldcraft_model::Procedure::Transient(_))
    {
        return Err(error(format!("step `{step_name}` already exists and is not transient")));
    }
    let (integration, method, damping, output_every, boundary_conditions, reused_existing_step) = if let Some(step) = &existing_step {
        let fieldcraft_model::Procedure::Transient(existing) = &step.procedure else {
            return Err(error(format!("step `{step_name}` already exists and is not transient")));
        };
        if existing.increments as usize != frame_count - 1 {
            return Err(error(format!(
                "existing transient step `{step_name}` has {} increments but the load history has {} intervals; time grid mismatch",
                existing.increments,
                frame_count - 1
            )));
        }
        if (existing.time_step - time_step).abs() > existing.time_step.abs().max(time_step.abs()).max(1.0) * 1e-12 {
            return Err(error(format!(
                "existing transient step `{step_name}` has time_step={:.17e}, incoming history has {:.17e}; time grid mismatch",
                existing.time_step, time_step
            )));
        }
        if let Some(value) = &requested_integration
            && value != &existing.integration
        {
            return Err(error(format!("existing transient step `{step_name}` integration settings are preserved and do not match the request")));
        }
        if let Some(value) = &requested_method
            && value != &existing.method
        {
            return Err(error(format!("existing transient step `{step_name}` method settings are preserved and do not match the request")));
        }
        if let Some(value) = &requested_damping
            && value != &existing.damping
        {
            return Err(error(format!("existing transient step `{step_name}` damping settings are preserved and do not match the request")));
        }
        if let Some(value) = requested_output_every
            && value != existing.output_every
        {
            return Err(error(format!("existing transient step `{step_name}` output cadence is preserved and does not match the request")));
        }
        if let Some(value) = &requested_boundary_conditions
            && value != &step.boundary_conditions
        {
            return Err(error(format!("existing transient step `{step_name}` boundary conditions are preserved and do not match the request")));
        }
        (existing.integration, existing.method.clone(), existing.damping, existing.output_every, step.boundary_conditions.clone(), true)
    } else {
        (
            requested_integration.unwrap_or_default(),
            requested_method.unwrap_or_default(),
            requested_damping.unwrap_or_default(),
            requested_output_every.unwrap_or(1),
            requested_boundary_conditions.unwrap_or_default(),
            false,
        )
    };
    if output_every == 0 || output_every > frame_count.saturating_sub(1) as u32 {
        return Err(error(format!("output_every must be from 1 to {} for {frame_count} frames", frame_count.saturating_sub(1))));
    }

    let mut active_nodes = Vec::new();
    let mut active_components = Vec::new();
    for (node_index, node) in decoded.archive.mapping.nodes.iter().enumerate() {
        let label = u32::try_from(node.node_id).map_err(|_| error("mapped node label exceeds u32"))?;
        if s.model.node(label).is_none() {
            return Err(error(format!("mapped node {label} is not in the open model")));
        }
        for axis in 0..3 {
            if decoded.mapped_frames.iter().any(|frame| frame.nodes[node_index].force_n[axis] != 0.0) {
                active_components.push((node_index, label, axis));
            }
        }
        if active_components.iter().any(|(index, _, _)| *index == node_index) {
            active_nodes.push((node_index, label));
        }
    }
    if active_components.is_empty() {
        return Err(error("load-transfer mapped history has no nonzero nodal components"));
    }
    let set_limit = fieldcraft_model::MAX_DEFINITIONS.saturating_sub(s.model.sets().len());
    if active_nodes.len() > set_limit {
        return Err(error(format!(
            "load-transfer object limit exceeded before mutation: {} node sets required, {} available",
            active_nodes.len(),
            set_limit
        )));
    }
    let load_limit = fieldcraft_model::MAX_DEFINITIONS.saturating_sub(s.model.loads().len());
    if active_components.len() > load_limit {
        return Err(error(format!(
            "load-transfer object limit exceeded before mutation: {} load objects required, {} available",
            active_components.len(),
            load_limit
        )));
    }
    let set_names: Vec<String> = active_nodes.iter().map(|(_, label)| format!("lt_{}_n{}_set", name, label)).collect();
    let load_names: Vec<String> = active_components.iter().map(|(_, label, axis)| format!("lt_{}_n{}_d{}", name, label, axis + 1)).collect();
    if let Some(existing) = set_names.iter().find(|candidate| s.model.sets().contains_key(*candidate)) {
        return Err(error(format!("load-transfer set `{existing}` already exists")));
    }
    if let Some(existing) = load_names.iter().find(|candidate| s.model.loads().contains_key(*candidate)) {
        return Err(error(format!("load-transfer load `{existing}` already exists")));
    }

    let mut model = s.model.clone();
    let mut loads = BTreeSet::new();
    let mut load_amplitudes = BTreeMap::new();
    for (node_index, label) in &active_nodes {
        let set_name = format!("lt_{}_n{}_set", name, label);
        model.add_set(&set_name, Set { nodes: vec![*label], ..Set::default() }).map_err(error)?;
        for (component_node_index, component_label, axis) in active_components.iter().filter(|(index, _, _)| index == node_index) {
            let load_name = format!("lt_{}_n{}_d{}", name, component_label, axis + 1);
            let mut vector = [0.0; 3];
            vector[*axis] = 1.0;
            model
                .add_load(&load_name, Load::Force { target: Target::Set(set_name.clone()), vector, distribution: Distribution::Total })
                .map_err(error)?;
            let values = decoded
                .mapped_frames
                .iter()
                .map(|frame| [frame.frame.time_s - first_time, frame.nodes[*component_node_index].force_n[*axis]])
                .collect();
            loads.insert(load_name.clone());
            load_amplitudes.insert(load_name, fieldcraft_model::amplitude::Amplitude::Tabular { points: values });
        }
    }
    if let Some(mut step) = existing_step {
        match &mut step.procedure {
            Procedure::Transient(transient) => transient.load_amplitudes.extend(load_amplitudes),
            _ => return Err(error(format!("step `{step_name}` already exists and is not transient"))),
        }
        step.loads.extend(loads);
        model.replace_step(step).map_err(error)?;
    } else {
        let procedure = Procedure::Transient(fieldcraft_model::dynamics::Transient {
            adaptive: None,
            time_step,
            increments: (frame_count - 1) as u32,
            output_every,
            integration,
            method: method.clone(),
            damping,
            load_amplitudes,
            initial_displacement: BTreeMap::new(),
            initial_velocity: BTreeMap::new(),
            base_acceleration: None,
            base_displacement: None,
        });
        let mut step = Step::new(step_name.clone(), procedure);
        step.boundary_conditions = boundary_conditions.clone();
        step.loads = loads;
        model.add_step(step, None).map_err(error)?;
    }
    s.model = model;
    Ok(json!({
        "archive_key":name,
        "step":step_name,
        "reused_existing_step":reused_existing_step,
        "frames":frame_count,
        "nodes":active_nodes.len(),
        "load_components":active_components.len(),
        "conservation":conservation,
        "runtime_qualification":decoded.archive.qualification,
        "program_identity":{"producer_sha256":decoded.archive.identity.producer_sha256,"adapter_sha256":decoded.archive.identity.adapter_sha256,"receiver_sha256":received_receiver_sha256,"current_receiver_sha256":current_receiver_sha256},
        "applicability":decoded.archive.applicability,
        "conclusion_allowed":decoded.archive.applicability.adoption_status == load_transfer::ApplicabilityStatus::Applicable,
        "boundary_conditions":boundary_conditions,
        "boundary_condition_count":boundary_conditions.len(),
        "boundary_condition_warning":if boundary_conditions.is_empty() { Some("no boundary conditions were supplied; a newly created structural step may be unconstrained") } else { None },
        "time_step_source":"total_duration/(frames-1)",
        "time_grid_tolerance_s":LOAD_HISTORY_TIME_TOLERANCE_S,
        "transient_settings":{"time_step":time_step,"increments":frame_count - 1,"output_every":output_every,"integration":integration,"method":method,"damping":damping},
        "ready_for":"solve.transient"
    }))
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
#[cfg(test)]
mod load_transfer_identity_tests {
    use super::*;

    fn identity() -> load_transfer::LoadIdentity {
        load_transfer::LoadIdentity {
            archive_key: "load".into(),
            run_id: "run".into(),
            epoch: 1,
            worker_generation: "worker".into(),
            model_revision: "a".repeat(64),
            mesh_digest: "b".repeat(64),
            coordinate_frame: "world".into(),
            units: "SI".into(),
            producer_sha256: fieldcraft_io::sha256_hex(b"producer"),
            adapter_sha256: fieldcraft_io::sha256_hex(b"adapter"),
        }
    }

    #[test]
    fn apply_allows_a_new_receiver_but_rejects_changed_source_artifacts() {
        let identity = identity();
        let received_receiver = fieldcraft_io::sha256_hex(b"receiver-at-receive");
        let current_receiver = fieldcraft_io::sha256_hex(b"receiver-at-apply");
        let binding = BTreeMap::from([
            ("producer".to_string(), identity.producer_sha256.clone()),
            ("adapter".to_string(), identity.adapter_sha256.clone()),
            ("receiver".to_string(), received_receiver.clone()),
        ]);
        assert_eq!(validate_load_transfer_apply_identity(&binding, &identity, &current_receiver).unwrap(), received_receiver);

        let mut changed_producer = binding.clone();
        changed_producer.insert("producer".into(), fieldcraft_io::sha256_hex(b"other-producer"));
        assert!(
            validate_load_transfer_apply_identity(&changed_producer, &identity, &current_receiver)
                .unwrap_err()
                .to_string()
                .contains("producer identity")
        );

        let mut changed_adapter = binding;
        changed_adapter.insert("adapter".into(), fieldcraft_io::sha256_hex(b"other-adapter"));
        assert!(
            validate_load_transfer_apply_identity(&changed_adapter, &identity, &current_receiver)
                .unwrap_err()
                .to_string()
                .contains("adapter identity")
        );
    }
}
