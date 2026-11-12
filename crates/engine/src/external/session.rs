//! External results are an independent Session collection, never fabricated native Steps.
use super::{
    archive,
    context::ReceiveContextV1,
    load_transfer,
    production::IoModelContext,
    receive::{FieldData, ResultEnvelope},
};
use crate::{EngineError, Result, Session};
use fieldcraft_io::external_results::ArchivedTrajectory;
use serde_json::{Value, json};
use std::sync::Arc;

pub(crate) fn static_base_modal_summary(modal: &fieldcraft_io::external_results::ModalPackage) -> Value {
    let mode_count = modal.modes.len();
    let mass_orthogonality_max_off_diagonal = modal
        .mass_gram
        .values
        .iter()
        .enumerate()
        .filter(|(index, _)| index / mode_count != index % mode_count)
        .map(|(_, value)| value.abs())
        .fold(0.0, f64::max);
    json!({
        "source_backend": modal.source_backend,
        "modes": modal.modes.iter().enumerate().map(|(index, mode)| json!({
            "mode": index + 1,
            "frequency_hz": mode.frequency_hz,
            "eigenvalue": mode.eigenvalue,
            "residual": mode.residual,
        })).collect::<Vec<_>>(),
        "pairs": modal.pairs.iter().map(|pair| json!({
            "mode": pair.mode + 1,
            "reference": pair.reference + 1,
            "mac": pair.mac,
            "margin": pair.margin,
        })).collect::<Vec<_>>(),
        "mass_orthogonality_max_off_diagonal": mass_orthogonality_max_off_diagonal,
        "shape_storage": "stored_in_kind_13_record; omitted_from_command_response",
    })
}

impl Session {
    pub(crate) fn check_external_name(&self, name: &str) -> Result<()> {
        if name.trim().is_empty() || name.len() > 1024 {
            return Err(EngineError::Params("ExternalArchiveLimit: result name".into()));
        }
        if self.external_results.contains_key(name)
            || self.external_chunked_results.contains_key(name)
            || self.external_static_base_results.contains_key(name)
            || self.results.contains_key(name)
            || self.model.step(name).is_some()
        {
            return Err(EngineError::Params("ExternalArchiveNameConflict: choose a unique result name".into()));
        }
        Ok(())
    }
    pub(crate) fn keep_external(&mut self, record: ArchivedTrajectory) -> Result<()> {
        self.check_external_name(&record.archive_key)?;
        if load_transfer::is_external_envelope(&record.envelope_json) {
            let bundle = load_transfer::decode_external_envelope(&record.envelope_json).map_err(|e| EngineError::Params(e.to_string()))?;
            let decoded = load_transfer::LoadTransferArchive::decode(&bundle).map_err(|e| EngineError::Params(e.to_string()))?;
            load_transfer::validate_receive_status(&decoded.archive).map_err(EngineError::Params)?;
            if decoded.archive.identity.archive_key != record.archive_key
                || decoded.archive.identity.model_revision != record.binding.snapshot.model_revision
                || decoded.archive.identity.mesh_digest != record.binding.snapshot.reference_mesh_digest
            {
                return Err(EngineError::Params("LoadTransferIdentityMismatch".into()));
            }
            record.validate_binding(&self.model, &self.sources, &self.run).map_err(|e| EngineError::Params(e.to_string()))?;
            self.external_results.insert(record.archive_key.clone(), Arc::new(record));
            self.results_stored = self.results_stored.wrapping_add(1);
            return Ok(());
        }
        let current = IoModelContext::new(self.model.clone(), self.sources.clone(), self.run.clone(), record.binding.artifacts.clone());
        archive::reopen(&record, current).map_err(|e| EngineError::Params(e.to_string()))?;
        self.external_results.insert(record.archive_key.clone(), Arc::new(record));
        self.results_stored = self.results_stored.wrapping_add(1);
        Ok(())
    }
    pub(crate) fn keep_external_chunked(&mut self, record: fieldcraft_io::external_results::ArchivedChunkedTrajectory) -> Result<()> {
        self.check_external_name(record.archive_key())?;
        let current = IoModelContext::new(self.model.clone(), self.sources.clone(), self.run.clone(), record.binding().artifacts.clone());
        super::chunked_archive::reopen(&record, current).map_err(|e| EngineError::Params(e.to_string()))?;
        self.external_chunked_results.insert(record.archive_key().into(), Arc::new(record));
        self.results_stored = self.results_stored.wrapping_add(1);
        Ok(())
    }
    pub(crate) fn keep_external_static_base(&mut self, record: fieldcraft_io::external_results::ArchivedStaticBase) -> Result<()> {
        self.check_external_name(record.archive_key())?;
        record.validate_binding(&self.model, &self.sources, &self.run).map_err(|e| EngineError::Params(e.to_string()))?;
        self.external_static_base_results.insert(record.archive_key().into(), Arc::new(record));
        self.results_stored = self.results_stored.wrapping_add(1);
        Ok(())
    }
    /// Raw diagnostic blobs can contain a failed frame. Only the admission's qualified
    /// prefix is eligible numerical data; persistence never changes that boundary.
    pub fn external_chunked_result(&self, name: &str) -> Result<&fieldcraft_io::external_results::ArchivedChunkedTrajectory> {
        let r = self.external_chunked_results.get(name).ok_or_else(|| EngineError::Params("external chunked result not found".into()))?;
        r.validate_binding(&self.model, &self.sources, &self.run)?;
        Ok(r)
    }
    pub fn external_static_base_result(&self, name: &str) -> Result<&fieldcraft_io::external_results::ArchivedStaticBase> {
        let r = self.external_static_base_results.get(name).ok_or_else(|| EngineError::Params("external static base result not found".into()))?;
        r.validate_binding(&self.model, &self.sources, &self.run).map_err(|e| EngineError::Params(e.to_string()))?;
        Ok(r)
    }
    /// Check current model binding before exposing stored external fields to a caller.
    pub fn external_result(&self, name: &str) -> Result<&ArchivedTrajectory> {
        let record = self.external_results.get(name).ok_or_else(|| EngineError::Params("external result not found".into()))?;
        record.validate_binding(&self.model, &self.sources, &self.run)?;
        Ok(record)
    }
    pub fn external_results_summary(&self) -> Value {
        let mut rows:Vec<Value>=self.external_results.values().map(|record|{
            let current=record.validate_binding(&self.model,&self.sources,&self.run);
            let content=||->std::result::Result<Value,String>{
                if load_transfer::is_external_envelope(&record.envelope_json) {
                    let bundle = load_transfer::decode_external_envelope(&record.envelope_json).map_err(|e| e.to_string())?;
                    let decoded = load_transfer::LoadTransferArchive::decode(&bundle).map_err(|e| e.to_string())?;
                    load_transfer::validate_receive_status(&decoded.archive)?;
                    let backend_status = load_transfer::backend_status(&decoded.archive);
                    let conservation = decoded
                        .archive
                        .frames
                        .iter()
                        .zip(&decoded.mapped_frames)
                        .map(|(frame, mapped)| {
                            load_transfer::check_conservation(
                                frame,
                                mapped,
                                &decoded.archive.mapping,
                                &vec![[0.0, 0.0, 0.0]; decoded.archive.mapping.nodes.len()],
                            )
                        })
                        .collect::<std::result::Result<Vec<_>, _>>()
                        .map_err(|e| e.to_string())?;
                    let integrals = load_transfer::check_time_integrals(&decoded.archive.frames, &decoded.mapped_frames, decoded.archive.mapping.origin_m)
                        .map_err(|e| e.to_string())?;
                    return Ok(json!({"kind":13,"schema":load_transfer::SCHEMA,"archive_key":record.archive_key,"result_id":record.result_id,
                        "frames":decoded.archive.frames.len(),"conservation":conservation,"time_integrals":integrals,
                        "applicability":decoded.archive.applicability,"runtime_qualification":decoded.archive.qualification,
                        "crash":decoded.archive.crash,"backend_status":backend_status,"native_postprocessing":"load_transfer.apply","stale":current.is_err(),
                        "binding_error":current.as_ref().err().map(ToString::to_string)}));
                }
                let context=ReceiveContextV1::decode(&record.context_json).map_err(|e|e.to_string())?;
                let e:ResultEnvelope=serde_json::from_slice(&record.envelope_json).map_err(|e|e.to_string())?;
                let admission:Value=serde_json::from_slice(&record.admission_json).map_err(|e|e.to_string())?;
                let mut quantities=std::collections::BTreeMap::<String,Value>::new();
                for q in ["position","displacement","velocity","object_force","object_moment","contact_energy"] {
                    let mut frames=0;let mut partial=0;let mut reasons=std::collections::BTreeSet::new();
                    for f in &e.frames {for field in &f.fields {
                        if serde_json::to_value(field.spec.quantity).map_err(|e|e.to_string())?==q {
                            match &field.data {FieldData::Present{available,..}=>{frames+=1;if available.iter().any(|v|!*v){partial+=1;}},FieldData::Unavailable{reason}=>{reasons.insert(reason.clone());}}
                        }
                    }}
                    quantities.insert(q.to_string(),json!({"present_frames":frames,"frames_with_partial_mask":partial,"unavailable_reasons":reasons}));
                }
                Ok(json!({"kind":13,"archive_key":record.archive_key,"result_id":record.result_id,"run":context.run,"frames":e.frames.len(),"goal":e.goal,
                    "admission":admission,"publication":context.publication,"runtime_qualification":context.runtime_qualification,
                    "quantities":quantities,"native_postprocessing":"unsupported","stale":current.is_err(),"binding_error":current.as_ref().err().map(ToString::to_string)}))
            };
            content().unwrap_or_else(|error|json!({"archive_key":record.archive_key,"error":error,"stale":true}))
        }).collect();
        for record in self.external_chunked_results.values() {
            let current = record.validate_binding(&self.model, &self.sources, &self.run);
            let parsed = serde_json::from_slice::<super::chunked_archive::Context>(record.context_json());
            let admission = serde_json::from_slice::<super::receive::Admission>(record.admission_json());
            rows.push(match (parsed, admission) {
                (Ok(c), Ok(a)) => json!({"kind":13,"schema":"fieldcraft-external-trajectory/2","archive_key":record.archive_key(),
                    "result_id":record.result_id(),"run":c.request.expected.run,"goal":c.request.expected.profile.goal,
                    "acceptance":c.request.acceptance,"observed_frames":c.summary.observed_frames,"first_failed_facts":c.summary.first_failed_facts,"qualified_coverage":a.coverage,
                    "admission":a,"publication":c.publication,"runtime_qualification":c.runtime_qualification,"quantities":c.summary.quantities,
                    "native_postprocessing":"unsupported","stale":current.is_err(),"binding_error":current.err().map(|e|e.to_string())}),
                _ => json!({"archive_key":record.archive_key(),"stale":true,"error":"ExternalArchiveContext: metadata"}),
            });
        }
        for record in self.external_static_base_results.values() {
            let current = record.validate_binding(&self.model, &self.sources, &self.run);
            let modal_package = record.external_static_base().modal.as_ref().map(static_base_modal_summary);
            rows.push(json!({
                "kind":13,
                "schema":fieldcraft_io::external_results::STATIC_BASE_SCHEMA,
                "archive_key":record.archive_key(),
                "result_id":record.result_id(),
                "source_backend":record.external_static_base().source_backend,
                "runtime_qualification":record.external_static_base().runtime_qualification,
                "frequency_response":record.external_static_base().frequency_response,
                "modal_package":modal_package,
                "native_postprocessing":"unsupported",
                "stale":current.is_err(),
                "binding_error":current.err().map(|e| e.to_string()),
            }));
        }
        rows.sort_by(|a, b| a["archive_key"].as_str().cmp(&b["archive_key"].as_str()));
        json!(rows)
    }
}
