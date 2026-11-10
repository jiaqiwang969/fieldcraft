//! Explicit launch-plan runner for the Rust-owned native adapter, not a solver qualification test.
//! Usage: native_backend launch-plan.json; the plan is host-authorized, never read from a result.
use fieldcraft_engine::external::native::{Launch, NativeBackend, binary_arrays};
use fieldcraft_engine::external::run::{RunGoal, RunState};
use fieldcraft_engine::external::{EvidenceKind, RunKey, SourceRef};
use serde::Deserialize;
use serde_json::{Value, json};
use std::path::{Path, PathBuf};
use std::time::Instant;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Plan {
    mode: String,
    interpreter: PathBuf,
    interpreter_sha256: String,
    adapter: PathBuf,
    adapter_sha256: String,
    scene: PathBuf,
    scene_sha256: String,
    scene_binary: PathBuf,
    job: PathBuf,
    run: RunKey,
    goal: RunGoal,
    evidence_kind: EvidenceKind,
}
fn read(path: &Path) -> Result<Vec<u8>> {
    Ok(fieldcraft_io::fs::read_capped(path, 128 * 1024 * 1024)?)
}
fn write(path: &Path, v: &Value) -> Result<()> {
    Ok(fieldcraft_io::fs::write_atomic(path, &serde_json::to_vec_pretty(v)?, false)?)
}
fn resource(pid: u32) -> Value {
    let gpu = std::process::Command::new("nvidia-smi")
        .args(["--query-compute-apps=pid,process_name,used_gpu_memory", "--format=csv,noheader,nounits"])
        .output();
    let gpu = match gpu {
        Ok(v) => json!({"exit_code":v.status.code(),"stdout":String::from_utf8_lossy(&v.stdout),"stderr":String::from_utf8_lossy(&v.stderr)}),
        Err(e) => json!({"unobserved":e.to_string()}),
    };
    json!({"pid":pid,"proc_exists":Path::new(&format!("/proc/{pid}")).exists(),
        "status":std::fs::read_to_string(format!("/proc/{pid}/status")).ok(),"gpu_compute":gpu})
}
fn loaded(pid: u32) -> Value {
    let mut paths = std::collections::BTreeSet::new();
    if let Ok(maps) = std::fs::read_to_string(format!("/proc/{pid}/maps")) {
        for line in maps.lines() {
            if let Some((_, name)) = line.split_once('/')
                && name.contains("uipc/_native/")
            {
                paths.insert(format!("/{name}"));
            }
        }
    }
    let files: Vec<_> = paths
        .into_iter()
        .map(|p| match read(Path::new(&p)) {
            Ok(b) => json!({"path":p,"bytes":b.len(),"sha256":fieldcraft_io::sha256_hex(&b)}),
            Err(e) => json!({"path":p,"error":e.to_string()}),
        })
        .collect();
    json!(files)
}
fn operations(b: &mut NativeBackend, mode: &str) -> Result<Value> {
    let started = Instant::now();
    let mut facts = Vec::new();
    let run = b.run().clone();
    let initial: Value = serde_json::from_slice(&read(&b.root().join("rust-initialization.json"))?)?;
    write(&b.root().join("loaded-native.json"), &loaded(b.pid()))?;
    write(&b.root().join("resources-initialized.json"), &resource(b.pid()))?;
    let mut responses = Vec::new();
    if mode == "normal" {
        for _ in 0..b.ledger().goal().last_frame {
            let id = b.start_steps(&run, 1)?;
            let (_, rows) = b.finish_steps(&id)?;
            facts.extend(rows);
            responses.push(format!("responses/{id}.json"));
        }
        let name = responses.last().ok_or("no frames")?;
        let source = SourceRef {
            sha256: fieldcraft_io::sha256_hex(&read(&b.root().join(name))?),
            locator: name.clone(),
            observer: "Rust checked all requested native snapshots and individual frame_stats".into(),
            kind: EvidenceKind::Solver,
        };
        b.finish_run(source)?;
        write(&b.root().join("frames.json"), &serde_json::to_value(&facts)?)?;
        let before = b.ledger().state();
        let exit = b.shutdown()?;
        if !exit.success() || b.ledger().state() != RunState::Succeeded {
            return Err("normal child did not exit successfully".into());
        }
        write(&b.root().join("resources-after-wait.json"), &resource(b.pid()))?;
        Ok(
            json!({"mode":mode,"run":run,"responses":responses,"frames":facts.len(),"before_shutdown":before,"after_wait":b.ledger().state(),"exit":exit,
            "wall_seconds":started.elapsed().as_secs_f64(),"simulation_end_time_s":b.ledger().goal().end_time_s,"runtime_qualification":"unverified","per_frame_effective_kappa":"unknown","per_frame_clipping":"unknown"}),
        )
    } else if mode == "lifecycle" {
        let id = b.start_steps(&run, 3)?;
        let (_, old_facts) = b.finish_steps(&id)?;
        write(&b.root().join("frames-before-reset.json"), &serde_json::to_value(&old_facts)?)?;
        let reset = b.reset()?;
        write(&b.root().join("rust-reset.json"), &reset)?;
        let before = binary_arrays(b.root(), &initial)?;
        let after = binary_arrays(b.root(), &reset)?;
        let reset_same_arrays = before == after;
        let stale = b.start_steps(&run, 1).expect_err("old epoch must be refused").to_string();
        let reset_run = b.run().clone();
        write(
            &b.root().join("reset-check.json"),
            &json!({"old_run":run,"new_run":reset_run,"same_pid":reset["pid"]==initial["pid"],"same_initial_arrays":reset_same_arrays,
            "native_frame_after_reset":reset["last_valid_frame"],"upload_reads":reset["upload_reads"],"old_epoch_error":stale,"resources":resource(b.pid())}),
        )?;
        if !reset_same_arrays {
            return Err("native reset initial arrays differ".into());
        }
        let id = b.start_steps(&reset_run, b.ledger().goal().last_frame)?;
        let progress = b.wait_progress(&id)?;
        let cancel = b.request_cancel()?;
        write(&b.root().join("rust-cancel-request.json"), &json!({"progress":progress,"cancel":cancel}))?;
        let (partial, facts) = b.finish_steps(&id)?;
        write(&b.root().join("frames-after-cancel.json"), &serde_json::to_value(&facts)?)?;
        let before = b.ledger().state();
        if before != RunState::CancelPending
            || partial["status"] != "cancelled"
            || facts.is_empty()
            || facts.len() as u64 >= b.ledger().goal().last_frame
        {
            return Err("cancel did not yield the expected real partial trajectory".into());
        }
        let finish_refusal = b
            .finish_run(SourceRef {
                sha256: fieldcraft_io::sha256_hex(&read(&b.root().join(format!("responses/{id}.json")))?),
                locator: format!("responses/{id}.json"),
                observer: "native cancellation response".into(),
                kind: EvidenceKind::Solver,
            })
            .expect_err("cancelled prefix cannot succeed")
            .to_string();
        let exit = b.shutdown()?;
        let resources = resource(b.pid());
        write(&b.root().join("resources-after-wait.json"), &resources)?;
        if !exit.success() || b.ledger().state() != RunState::Cancelled {
            return Err("cancel child cleanup failed".into());
        }
        Ok(
            json!({"mode":mode,"old_run":run,"run":reset_run,"reset_arrays_equal":reset_same_arrays,"cancelled_frames":facts.len(),"last_valid_frame":b.ledger().last_qualified(),
            "before_shutdown":before,"after_wait":b.ledger().state(),"exit":exit,"result_state":"not_published","finish_refusal":finish_refusal,"wall_seconds":started.elapsed().as_secs_f64(),"runtime_qualification":"unverified"}),
        )
    } else {
        Err("unknown launch mode".into())
    }
}
fn main() -> Result<()> {
    let path = std::env::args_os().nth(1).ok_or("launch-plan path required")?;
    let plan_bytes = read(Path::new(&path))?;
    let plan: Plan = serde_json::from_slice(&plan_bytes)?;
    let scene = read(&plan.scene)?;
    if fieldcraft_io::sha256_hex(&scene) != plan.scene_sha256 {
        return Err("input scene hash differs from authorized plan".into());
    }
    let all = Instant::now();
    let spec = Launch {
        interpreter: plan.interpreter,
        interpreter_sha256: plan.interpreter_sha256,
        adapter: plan.adapter,
        adapter_sha256: plan.adapter_sha256,
        job: plan.job.clone(),
        run: plan.run,
        goal: plan.goal,
        evidence_kind: plan.evidence_kind,
    };
    let mut backend = NativeBackend::launch(spec, serde_json::from_slice(&scene)?, &read(&plan.scene_binary)?)?;
    let initialized_seconds = all.elapsed().as_secs_f64();
    let result = operations(&mut backend, &plan.mode);
    let summary = match &result {
        Ok(v) => {
            json!({"plan_sha256":fieldcraft_io::sha256_hex(&plan_bytes),"initialize_wall_seconds":initialized_seconds,"total_wall_seconds":all.elapsed().as_secs_f64(),"result":v})
        }
        Err(e) => {
            let cleanup = backend.terminate();
            json!({"first_error":e.to_string(),"state":backend.ledger().state(),"last_qualified":backend.ledger().last_qualified(),"barrier":backend.ledger().barrier(),
                "cleanup":cleanup.as_ref().ok(),"cleanup_error":cleanup.as_ref().err().map(ToString::to_string),"resources":resource(backend.pid())})
        }
    };
    write(&plan.job.join("rust-summary.json"), &summary)?;
    println!("{}", serde_json::to_string(&summary)?);
    result.map(|_| ())
}
