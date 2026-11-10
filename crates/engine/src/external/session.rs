//! External results are an independent Session collection, never fabricated native Steps.
use super::{
    archive,
    context::ReceiveContextV1,
    production::IoModelContext,
    receive::{FieldData, ResultEnvelope},
};
use crate::{EngineError, Result, Session};
use fieldcraft_io::external_results::ArchivedTrajectory;
use serde_json::{Value, json};
use std::sync::Arc;

impl Session {
    pub(crate) fn check_external_name(&self, name: &str) -> Result<()> {
        if name.trim().is_empty() || name.len() > 1024 {
            return Err(EngineError::Params("ExternalArchiveLimit: result name".into()));
        }
        if self.external_results.contains_key(name) || self.results.contains_key(name) || self.model.step(name).is_some() {
            return Err(EngineError::Params("ExternalArchiveNameConflict: choose a unique result name".into()));
        }
        Ok(())
    }
    pub(crate) fn keep_external(&mut self, record: ArchivedTrajectory) -> Result<()> {
        self.check_external_name(&record.archive_key)?;
        let current = IoModelContext::new(self.model.clone(), self.sources.clone(), self.run.clone(), record.binding.artifacts.clone());
        archive::reopen(&record, current).map_err(|e| EngineError::Params(e.to_string()))?;
        self.external_results.insert(record.archive_key.clone(), Arc::new(record));
        self.results_stored = self.results_stored.wrapping_add(1);
        Ok(())
    }
    /// Check current model binding before exposing stored external fields to a caller.
    pub fn external_result(&self, name: &str) -> Result<&ArchivedTrajectory> {
        let record = self.external_results.get(name).ok_or_else(|| EngineError::Params("external result not found".into()))?;
        record.validate_binding(&self.model, &self.sources, &self.run)?;
        Ok(record)
    }
    pub fn external_results_summary(&self) -> Value {
        let rows:Vec<Value>=self.external_results.values().map(|record|{
            let current=record.validate_binding(&self.model,&self.sources,&self.run);
            let content=||->std::result::Result<Value,String>{
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
        json!(rows)
    }
}
