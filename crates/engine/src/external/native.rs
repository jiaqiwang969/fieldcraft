//! A Rust-owned, bounded filesystem mailbox for an explicitly selected native adapter.
//!
//! The adapter loads libuipc in the direct child. Rust owns requests, frame admission,
//! reset epochs, cancellation and OS wait observations. No CUDA is linked into FieldCraft.
//! A native reset rebuilds the World from the cached scene; it does not promise numerical
//! reproducibility or release of the process's CUDA context. Shutdown and wait are separate.

use std::collections::BTreeMap;
use std::path::{Component, Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant};

use serde_json::{Value, json};

use super::run::{FrameFacts, RunGoal, RunLedger, RunState};
use super::worker::{OwnedWorker, ProcessExit};
use super::{ContractError, EvidenceKind, Observed, RunKey, SourceRef};

const PROTOCOL: &str = "route21/2";
const JSON_CAP: u64 = 1024 * 1024;
const BINARY_CAP: u64 = 64 * 1024 * 1024;
const WAIT: Duration = Duration::from_secs(30);

fn err(e: impl std::fmt::Display) -> ContractError {
    ContractError::Worker(e.to_string())
}
fn relative(root: &Path, name: &str) -> Result<PathBuf, ContractError> {
    if name.is_empty() || !Path::new(name).components().all(|p| matches!(p, Component::Normal(_))) {
        return Err(ContractError::Invalid("mailbox relative path"));
    }
    let path = root.join(name);
    if path.exists() && !path.canonicalize().map_err(err)?.starts_with(root) {
        return Err(ContractError::Invalid("mailbox path escaped through a link"));
    }
    Ok(path)
}
fn read_json(path: &Path) -> Result<(Value, Vec<u8>), ContractError> {
    let bytes = fieldcraft_io::fs::read_capped(path, JSON_CAP).map_err(err)?;
    let value = serde_json::from_slice(&bytes).map_err(err)?;
    Ok((value, bytes))
}
fn write_json(path: &Path, value: &Value) -> Result<(), ContractError> {
    let bytes = serde_json::to_vec(value).map_err(err)?;
    if bytes.len() as u64 > JSON_CAP {
        return Err(ContractError::Invalid("mailbox JSON cap"));
    }
    fieldcraft_io::fs::write_atomic(path, &bytes, false).map_err(err)
}

/// Paths and digests are supplied by the host's authorized launch plan, not by a result.
pub struct Launch {
    pub interpreter: PathBuf,
    pub interpreter_sha256: String,
    pub adapter: PathBuf,
    pub adapter_sha256: String,
    pub job: PathBuf,
    pub run: RunKey,
    pub goal: RunGoal,
    pub evidence_kind: EvidenceKind,
}

pub struct NativeBackend {
    worker: OwnedWorker,
    root: PathBuf,
    scene_id: String,
    epoch: u64,
    seq: u64,
    kind: EvidenceKind,
    active: Option<String>,
    ledger: RunLedger,
    last_exit: Option<ProcessExit>,
    nodes: usize,
    dt_s: f64,
}

impl NativeBackend {
    pub fn launch(spec: Launch, scene: Value, scene_bytes: &[u8]) -> Result<Self, ContractError> {
        spec.run.validate()?;
        spec.goal.validate()?;
        if spec.run.epoch != 1 || scene_bytes.len() as u64 > BINARY_CAP {
            return Err(ContractError::Invalid("native initial epoch or scene size"));
        }
        for (path, expected) in [(&spec.interpreter, &spec.interpreter_sha256), (&spec.adapter, &spec.adapter_sha256)] {
            if !fieldcraft_io::identity::digest_valid(expected)
                || fieldcraft_io::sha256_hex(&fieldcraft_io::fs::read_capped(path, BINARY_CAP * 2).map_err(err)?) != *expected
            {
                return Err(ContractError::IdentityMismatch("native launch artifact".into()));
            }
        }
        let scene_id = scene.get("scene_sha256").and_then(Value::as_str).ok_or(ContractError::Invalid("native scene identity"))?.to_string();
        if !fieldcraft_io::identity::digest_valid(&scene_id)
            || scene.pointer("/buffer/sha256").and_then(Value::as_str) != Some(fieldcraft_io::sha256_hex(scene_bytes).as_str())
        {
            return Err(ContractError::ChecksumMismatch);
        }
        let nodes = scene["objects"].as_array().ok_or(ContractError::Invalid("native objects"))?.iter().try_fold(0usize, |n, o| {
            let count = o["vertices"].as_u64().and_then(|v| usize::try_from(v).ok()).ok_or(ContractError::Invalid("native vertex count"))?;
            n.checked_add(count).ok_or(ContractError::Invalid("native vertex count"))
        })?;
        let dt_s = scene["dt_s"].as_f64().filter(|dt| dt.is_finite() && *dt > 0.0).ok_or(ContractError::Invalid("native time step"))?;
        if nodes == 0 || nodes > 1_000_000 || spec.goal.last_frame as f64 * dt_s != spec.goal.end_time_s {
            return Err(ContractError::Invalid("native goal and input do not match"));
        }
        std::fs::create_dir(&spec.job).map_err(err)?; // refuse overwriting any old run
        let root = spec.job.canonicalize().map_err(err)?;
        let input = relative(&root, scene.pointer("/buffer/path").and_then(Value::as_str).ok_or(ContractError::Invalid("native scene path"))?)?;
        if let Some(parent) = input.parent() {
            std::fs::create_dir_all(parent).map_err(err)?;
        }
        fieldcraft_io::fs::write_atomic(&input, scene_bytes, false).map_err(err)?;
        let mut command = Command::new(&spec.interpreter);
        command
            .arg("-B")
            .arg(&spec.adapter)
            .arg("--job")
            .arg(&root)
            .arg("--backend")
            .arg(if spec.evidence_kind == EvidenceKind::Solver { "cuda" } else { "mock" })
            .arg("--parent-pid")
            .arg(std::process::id().to_string())
            .arg("--wall-limit")
            .arg("570")
            .env("PYTHONDONTWRITEBYTECODE", "1")
            .env("OPENBLAS_NUM_THREADS", "1")
            .env("OMP_NUM_THREADS", "1")
            .env("MKL_NUM_THREADS", "1");
        let worker = OwnedWorker::spawn_logged(&mut command, spec.run.clone(), &root.join("worker.log"))?;
        let mut this = Self {
            worker,
            root,
            scene_id,
            epoch: 0,
            seq: 0,
            kind: spec.evidence_kind,
            active: None,
            ledger: RunLedger::new(spec.run, spec.goal, spec.evidence_kind)?,
            last_exit: None,
            nodes,
            dt_s,
        };
        this.wait_file("ready.json")?;
        let (ready, _) = read_json(&this.root.join("ready.json"))?;
        if ready.get("protocol").and_then(Value::as_str) != Some(PROTOCOL)
            || ready.get("pid").and_then(Value::as_u64) != Some(u64::from(this.worker.pid()))
            || ready.get("physics").and_then(Value::as_bool) != Some(this.kind == EvidenceKind::Solver)
        {
            return Err(ContractError::Invalid("native ready identity"));
        }
        let rid = this.submit("create", json!({"scene":scene}))?;
        let (created, _) = this.response(&rid, 0)?;
        this.require_ok(&created)?;
        let initialized = this.initialize(false)?;
        write_json(&this.root.join("rust-initialization.json"), &initialized)?;
        Ok(this)
    }

    pub fn run(&self) -> &RunKey {
        self.ledger.key()
    }
    pub fn pid(&self) -> u32 {
        self.worker.pid()
    }
    pub fn ledger(&self) -> &RunLedger {
        &self.ledger
    }
    pub fn root(&self) -> &Path {
        &self.root
    }

    fn wait_file(&mut self, name: &str) -> Result<(), ContractError> {
        let path = relative(&self.root, name)?;
        let start = Instant::now();
        while !path.is_file() {
            if let Some(exit) = self.worker.try_exit()? {
                self.ledger.observe_exit(&exit)?;
                self.last_exit = Some(exit);
                return Err(ContractError::Worker("native child exited before response".into()));
            }
            if start.elapsed() >= WAIT {
                return Err(ContractError::WorkerTimeout);
            }
            std::thread::sleep(Duration::from_millis(3));
        }
        Ok(())
    }

    fn submit(&mut self, op: &str, extra: Value) -> Result<String, ContractError> {
        self.seq = self.seq.checked_add(1).ok_or(ContractError::Invalid("request sequence overflow"))?;
        let id = format!("req_{:05}", self.seq);
        let mut request = json!({"protocol":PROTOCOL,"request_id":id,"op":op,
            "scene_id":self.scene_id,"epoch":self.epoch});
        let fields = extra.as_object().ok_or(ContractError::Invalid("mailbox arguments"))?;
        for (key, value) in fields {
            if let Some(map) = request.as_object_mut() {
                map.insert(key.clone(), value.clone());
            }
        }
        write_json(&self.root.join("inbox").join(format!("{id}.json")), &request)?;
        Ok(id)
    }

    fn response(&mut self, id: &str, epoch: u64) -> Result<(Value, SourceRef), ContractError> {
        let name = format!("responses/{id}.json");
        self.wait_file(&name)?;
        let (value, bytes) = read_json(&self.root.join(&name))?;
        if value.get("protocol").and_then(Value::as_str) != Some(PROTOCOL)
            || value.get("request_id").and_then(Value::as_str) != Some(id)
            || value.get("pid").and_then(Value::as_u64) != Some(u64::from(self.pid()))
            || value.get("scene_id").and_then(Value::as_str) != Some(self.scene_id.as_str())
            || value.get("epoch").and_then(Value::as_u64) != Some(epoch)
            || value.get("physics").and_then(Value::as_bool) != Some(self.kind == EvidenceKind::Solver)
        {
            return Err(ContractError::StaleRun);
        }
        Ok((
            value,
            SourceRef {
                sha256: fieldcraft_io::sha256_hex(&bytes),
                locator: name,
                observer: "Rust mailbox: native frame_stats and checked binary snapshot".into(),
                kind: self.kind,
            },
        ))
    }

    fn require_ok(&self, value: &Value) -> Result<(), ContractError> {
        if value.get("status").and_then(Value::as_str) != Some("ok") {
            return Err(ContractError::Worker(format!("native refused: {}", value.get("error").unwrap_or(&Value::Null))));
        }
        Ok(())
    }

    fn initialize(&mut self, reset: bool) -> Result<Value, ContractError> {
        let next = self.epoch.checked_add(1).ok_or(ContractError::Invalid("epoch overflow"))?;
        let id = self.submit(if reset { "reset" } else { "initialize" }, json!({}))?;
        let (value, _) = self.response(&id, next)?;
        self.require_ok(&value)?;
        let arrays = binary_arrays(&self.root, &value)?;
        snapshot_layout(&value, self.nodes)?;
        if arrays.get("frame").map(Vec::as_slice) != Some(0_u64.to_le_bytes().as_slice())
            || arrays.get("time").map(Vec::as_slice) != Some(0_f64.to_le_bytes().as_slice())
            || value.get("upload_reads").and_then(Value::as_u64) != Some(1)
            || value.get("last_valid_frame").and_then(Value::as_u64) != Some(0)
        {
            return Err(ContractError::Invalid("native reset initial state"));
        }
        if reset {
            if value.get("scene_reuploaded").and_then(Value::as_bool) != Some(false) {
                return Err(ContractError::Invalid("reset must reuse the original upload"));
            }
            let mut key = self.run().clone();
            key.epoch = next;
            self.worker.reset_epoch(key.clone())?;
            self.ledger = RunLedger::new(key, self.ledger.goal(), self.kind)?;
        }
        self.epoch = next;
        let key = self.run().clone();
        self.ledger.initialize(&key)?;
        Ok(value)
    }

    /// Reset is acknowledged only after native World reconstruction and an initial snapshot.
    pub fn reset(&mut self) -> Result<Value, ContractError> {
        if self.active.is_some() || matches!(self.ledger.state(), RunState::Advancing | RunState::CancelPending) {
            return Err(ContractError::InvalidState);
        }
        self.initialize(true)
    }

    pub fn start_steps(&mut self, expected: &RunKey, steps: u64) -> Result<String, ContractError> {
        if expected != self.run() {
            return Err(ContractError::StaleRun);
        }
        if steps > 1000 || self.active.is_some() {
            return Err(ContractError::Invalid("native batch size"));
        }
        let key = self.run().clone();
        self.ledger.begin_advance(&key, steps)?;
        let id = self.submit("step", json!({"steps":steps}))?;
        self.active = Some(id.clone());
        Ok(id)
    }

    pub fn finish_steps(&mut self, id: &str) -> Result<(Value, Vec<FrameFacts>), ContractError> {
        if self.active.as_deref() != Some(id) {
            return Err(ContractError::InvalidState);
        }
        let result = self.finish_steps_inner(id);
        self.active = None;
        if let Err(e) = &result {
            self.ledger.reject_transport(e.to_string());
        }
        result
    }

    fn finish_steps_inner(&mut self, id: &str) -> Result<(Value, Vec<FrameFacts>), ContractError> {
        let (value, source) = self.response(id, self.epoch)?;
        let arrays = binary_arrays(&self.root, &value)?;
        snapshot_layout(&value, self.nodes)?;
        let facts = frame_facts(self.run(), &value, &arrays, &source)?;
        for fact in &facts {
            self.ledger.accept_frame(fact.clone())?;
        }
        match value.get("status").and_then(Value::as_str) {
            Some("ok") if self.ledger.state() == RunState::Ready => {}
            Some("cancelled") if self.ledger.state() == RunState::CancelPending => {}
            _ => {
                if let Some(frame) = value["failed_frame"].as_u64() {
                    let row = json!({"frame_stats":value["native_failure_diagnostics"]});
                    let failed = facts_from_row(self.run(), frame, frame as f64 * self.dt_s, &row, &source, false);
                    self.ledger.accept_frame(failed)?;
                }
                return Err(ContractError::Worker(format!("native failure or incomplete batch: {}", value.get("error").unwrap_or(&Value::Null))));
            }
        }
        Ok((value, facts))
    }

    /// Wait for actual native progress before requesting cancellation in a controlled check.
    pub fn wait_progress(&mut self, id: &str) -> Result<Value, ContractError> {
        if self.active.as_deref() != Some(id) {
            return Err(ContractError::InvalidState);
        }
        let path = format!("progress/{id}.json");
        self.wait_file(&path)?;
        let (v, _) = read_json(&self.root.join(path))?;
        if v.get("request_id").and_then(Value::as_str) != Some(id) || v.get("epoch").and_then(Value::as_u64) != Some(self.epoch) {
            return Err(ContractError::StaleRun);
        }
        Ok(v)
    }

    pub fn request_cancel(&mut self) -> Result<Value, ContractError> {
        let target = self.active.clone().ok_or(ContractError::InvalidState)?;
        let key = self.run().clone();
        self.ledger.request_cancel(&key)?;
        let id = self.submit("cancel", json!({"target_request_id":target}))?;
        let (v, _) = self.response(&id, self.epoch)?;
        self.require_ok(&v)?;
        if v.get("accepted").and_then(Value::as_bool) != Some(true) {
            return Err(ContractError::Invalid("cancel acknowledgement"));
        }
        Ok(v) // still pending until actual OS wait
    }

    /// An explicit completion is distinct from a completed batch and from shutdown exit zero.
    pub fn finish_run(&mut self, source: SourceRef) -> Result<(), ContractError> {
        let key = self.run().clone();
        self.ledger.finish(&key, source)
    }

    pub fn shutdown(&mut self) -> Result<ProcessExit, ContractError> {
        if let Some(exit) = &self.last_exit {
            return Ok(exit.clone());
        }
        let id = self.submit("shutdown", json!({}))?;
        let (v, _) = self.response(&id, self.epoch)?;
        self.require_ok(&v)?;
        let exit = self.worker.wait_for_exit(WAIT)?;
        self.ledger.observe_exit(&exit)?;
        write_json(&self.root.join("rust-process-exit.json"), &serde_json::to_value(&exit).map_err(err)?)?;
        self.last_exit = Some(exit.clone());
        Ok(exit)
    }

    /// Error cleanup also retains a real OS observation, not a claimed wire exit code.
    pub fn terminate(&mut self) -> Result<ProcessExit, ContractError> {
        let exit = self.worker.terminate_and_reap(WAIT)?;
        self.ledger.observe_exit(&exit)?;
        write_json(&self.root.join("rust-process-exit.json"), &serde_json::to_value(&exit).map_err(err)?)?;
        self.last_exit = Some(exit.clone());
        Ok(exit)
    }
}

/// Decode bounded native arrays without numeric reinterpretation or silent trailing bytes.
pub fn binary_arrays(root: &Path, response: &Value) -> Result<BTreeMap<String, Vec<u8>>, ContractError> {
    let b = response.get("buffer").ok_or(ContractError::Invalid("native result buffer"))?;
    let path = relative(root, b.get("path").and_then(Value::as_str).ok_or(ContractError::Invalid("native buffer path"))?)?;
    let bytes = fieldcraft_io::fs::read_capped(&path, BINARY_CAP).map_err(err)?;
    if b.get("byte_count").and_then(Value::as_u64) != Some(bytes.len() as u64)
        || b.get("sha256").and_then(Value::as_str) != Some(fieldcraft_io::sha256_hex(&bytes).as_str())
        || b.get("endianness").and_then(Value::as_str) != Some("little")
    {
        return Err(ContractError::ChecksumMismatch);
    }
    let specs = b.get("arrays").and_then(Value::as_array).ok_or(ContractError::Invalid("native array descriptors"))?;
    if specs.len() > 32 {
        return Err(ContractError::Invalid("native array count"));
    }
    let mut out = BTreeMap::new();
    let mut offset = 0_usize;
    for s in specs {
        let name = s.get("name").and_then(Value::as_str).ok_or(ContractError::Invalid("native array name"))?;
        let dtype = s.get("dtype").and_then(Value::as_str).ok_or(ContractError::Invalid("native dtype"))?;
        let width = match dtype {
            "<f8" | "<u8" => 8_usize,
            "<i4" => 4,
            _ => return Err(ContractError::Invalid("native dtype")),
        };
        let shape = s.get("shape").and_then(Value::as_array).ok_or(ContractError::Invalid("native shape"))?;
        if shape.is_empty() || shape.len() > 4 || out.contains_key(name) {
            return Err(ContractError::Invalid("native array shape/name"));
        }
        let length = shape.iter().try_fold(width, |n, v| {
            v.as_u64().and_then(|v| usize::try_from(v).ok()).and_then(|v| n.checked_mul(v)).ok_or(ContractError::Invalid("native array size"))
        })?;
        let end = offset.checked_add(length).ok_or(ContractError::Invalid("native array bounds"))?;
        if s.get("offset").and_then(Value::as_u64) != Some(offset as u64) || s.get("byte_count").and_then(Value::as_u64) != Some(length as u64) {
            return Err(ContractError::Invalid("native array bounds"));
        }
        let data = bytes.get(offset..end).ok_or(ContractError::Invalid("native array bounds"))?;
        if dtype == "<f8" && data.as_chunks::<8>().0.iter().any(|b| !f64::from_le_bytes(*b).is_finite()) {
            return Err(ContractError::Invalid("nonfinite native value"));
        }
        out.insert(name.to_string(), data.to_vec());
        offset = end;
    }
    if offset != bytes.len() {
        return Err(ContractError::Invalid("native trailing bytes"));
    }
    Ok(out)
}

fn frame_facts(key: &RunKey, response: &Value, arrays: &BTreeMap<String, Vec<u8>>, source: &SourceRef) -> Result<Vec<FrameFacts>, ContractError> {
    let rows = response.get("diagnostics").and_then(Value::as_array).ok_or(ContractError::Invalid("native diagnostics"))?;
    let frames = arrays.get("frame").ok_or(ContractError::Invalid("native frame array"))?;
    let times = arrays.get("time").ok_or(ContractError::Invalid("native time array"))?;
    if frames.len() != rows.len() * 8
        || times.len() != frames.len()
        || response.get("frames_returned").and_then(Value::as_u64) != Some(rows.len() as u64)
    {
        return Err(ContractError::Invalid("native diagnostics/array lengths"));
    }
    rows.iter()
        .zip(frames.as_chunks::<8>().0)
        .zip(times.as_chunks::<8>().0)
        .map(|((row, frame), time)| {
            if row.pointer("/frame_stats/frame").and_then(Value::as_u64) != Some(u64::from_le_bytes(*frame)) {
                return Err(ContractError::Invalid("native diagnostic frame does not match the snapshot"));
            }
            Ok(facts_from_row(key, u64::from_le_bytes(*frame), f64::from_le_bytes(*time), row, source, true))
        })
        .collect()
}

fn facts_from_row(key: &RunKey, frame: u64, time_s: f64, row: &Value, source: &SourceRef, snapshot: bool) -> FrameFacts {
    let observed = |v: Option<bool>, name: &str| {
        v.map_or_else(
            || Observed::Unknown { reason: format!("native did not export {name}") },
            |value| Observed::Known { value, source: source.clone() },
        )
    };
    let stat = |name: &str| observed(row.get("frame_stats").and_then(|s| s.get(name)).and_then(Value::as_bool), name);
    FrameFacts {
        run: key.clone(),
        frame,
        time_s,
        completed: stat("completed"),
        converged: stat("converged"),
        hit_newton_limit: stat("hit_newton_limit"),
        hit_line_search_limit: stat("hit_line_search_limit"),
        world_valid: observed(row.get("world_valid").and_then(Value::as_bool), "world_valid"),
        snapshot_complete: Observed::Known { value: snapshot, source: source.clone() },
    }
}

fn snapshot_layout(response: &Value, nodes: usize) -> Result<(), ContractError> {
    let count = response["frames_returned"].as_u64().ok_or(ContractError::Invalid("snapshot count"))?;
    let specs = response["buffer"]["arrays"].as_array().ok_or(ContractError::Invalid("snapshot descriptors"))?;
    for (name, dtype, shape, unit) in
        [("frame", "<u8", json!([count]), "1"), ("time", "<f8", json!([count]), "s"), ("positions", "<f8", json!([count, nodes, 3]), "m")]
    {
        let spec = specs.iter().find(|s| s["name"].as_str() == Some(name)).ok_or(ContractError::Invalid("snapshot required field missing"))?;
        if spec["dtype"] != dtype || spec["shape"] != shape || spec["unit"] != unit || spec["coordinate_frame"] != "world" {
            return Err(ContractError::Invalid("snapshot field type/shape/units"));
        }
    }
    if response["diagnostics"].as_array().map(Vec::len) != Some(count as usize) {
        return Err(ContractError::Invalid("snapshot diagnostics count"));
    }
    Ok(())
}

impl Drop for NativeBackend {
    fn drop(&mut self) {
        // Even an early launch/transport error keeps its real wait receipt. Best effort only;
        // explicit shutdown/terminate is required for a positive cleanup claim.
        if self.last_exit.is_none() {
            let _ = self.terminate();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn snapshot_type_and_missing_position_are_not_a_complete_frame() {
        let mut r = json!({"frames_returned":1,"diagnostics":[{}],"buffer":{"arrays":[
            {"name":"frame","dtype":"<u8","shape":[1],"unit":"1","coordinate_frame":"world"},
            {"name":"time","dtype":"<f8","shape":[1],"unit":"s","coordinate_frame":"world"},
            {"name":"positions","dtype":"<f8","shape":[1,2,3],"unit":"m","coordinate_frame":"world"}]}});
        assert!(snapshot_layout(&r, 2).is_ok());
        r["buffer"]["arrays"][0]["dtype"] = json!("<f8");
        assert!(snapshot_layout(&r, 2).is_err());
        r["buffer"]["arrays"][0]["dtype"] = json!("<u8");
        r["buffer"]["arrays"].as_array_mut().unwrap().pop();
        assert!(snapshot_layout(&r, 2).is_err());
    }
    #[test]
    fn native_unknown_and_iteration_limits_are_not_filled_by_snapshot_success() {
        let run = RunKey { run_id: "test".into(), epoch: 1, worker_generation: "child".into() };
        let source = SourceRef {
            sha256: fieldcraft_io::sha256_hex(b"fixture"),
            locator: "fixture.json".into(),
            observer: "protocol fixture".into(),
            kind: EvidenceKind::ProtocolFixture,
        };
        let row = json!({"world_valid":true,"frame_stats":{"completed":true,"hit_newton_limit":false,"hit_line_search_limit":false}});
        let f = facts_from_row(&run, 1, 0.01, &row, &source, true);
        assert!(matches!(f.converged, Observed::Unknown { .. }));
        assert!(matches!(f.issue(source.kind), Some(super::super::run::FrameIssue::UnknownDiagnostic(_))));
        let mut r = row;
        r["frame_stats"]["converged"] = json!(true);
        r["frame_stats"]["hit_line_search_limit"] = json!(true);
        assert_eq!(facts_from_row(&run, 1, 0.01, &r, &source, true).issue(source.kind), Some(super::super::run::FrameIssue::LineSearchLimit));
    }
    #[test]
    fn invalid_binary_transport_seals_first_prefix() {
        let key = RunKey { run_id: "test".into(), epoch: 1, worker_generation: "child".into() };
        let mut l = RunLedger::new(key.clone(), RunGoal { last_frame: 2, end_time_s: 1.0 }, EvidenceKind::ProtocolFixture).unwrap();
        l.initialize(&key).unwrap();
        l.begin_advance(&key, 2).unwrap();
        l.reject_transport("bad binary SHA".into());
        assert_eq!(l.last_qualified(), 0);
        assert_eq!(l.barrier().unwrap().frame, 1);
        l.reject_transport("later error".into());
        assert_eq!(l.barrier().unwrap().issue, super::super::run::FrameIssue::InvalidReceipt("bad binary SHA".into()));
        assert!(l.begin_advance(&key, 1).is_err());
    }
    #[test]
    #[ignore = "requires an explicitly selected retained native run; no GPU is launched"]
    fn retained_native_frames_replay_with_original_sources() {
        let root = PathBuf::from(std::env::var_os("FIELDCRAFT_NATIVE_REPLAY").expect("retained native run directory"));
        let (summary, _) = read_json(&root.join("rust-summary.json")).unwrap();
        let key: RunKey = serde_json::from_value(summary["result"]["run"].clone()).unwrap();
        let mut count = 0;
        for name in summary["result"]["responses"].as_array().unwrap() {
            let name = name.as_str().unwrap();
            let (v, bytes) = read_json(&root.join(name)).unwrap();
            let arrays = binary_arrays(&root, &v).unwrap();
            snapshot_layout(&v, 43).unwrap();
            let source = SourceRef {
                sha256: fieldcraft_io::sha256_hex(&bytes),
                locator: name.into(),
                observer: "retained native replay; no new observation".into(),
                kind: EvidenceKind::Solver,
            };
            let facts = frame_facts(&key, &v, &arrays, &source).unwrap();
            assert!(facts.iter().all(|f| f.issue(EvidenceKind::Solver).is_none()));
            count += facts.len();
            let mut wrong = v;
            wrong["diagnostics"][0]["frame_stats"]["frame"] = json!(999);
            assert!(frame_facts(&key, &wrong, &arrays, &source).is_err());
        }
        assert_eq!(count, 240);
        println!("retained_native_snapshot_replay={{\"frames\":240,\"new_solver_run\":false,\"runtime_qualification\":\"unverified\"}}");
    }
}
