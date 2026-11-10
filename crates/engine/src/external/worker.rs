//! Own one direct child and observe its OS wait status. No shell status decoding.
//!
//! The launched program must be the worker itself, not a launcher that leaves descendants
//! running. Descendant process-group cancellation is not implemented by this first seam.
//! A wait receipt describes only this child; it never claims GPU or driver resource release.

use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde::Serialize;

use super::{ContractError, RunKey};

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ExitOutcome {
    Exited { code: i32 },
    Signaled { number: i32, core_dumped: bool },
    Unknown,
}

/// Deliberately not Deserialize: wire claims cannot manufacture an OS observation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ProcessExit {
    run: RunKey,
    pid: u32,
    parent_pid: u32,
    program: String,
    args: Vec<String>,
    observer: &'static str,
    layer: &'static str,
    observed_unix_ms: Option<u128>,
    outcome: ExitOutcome,
    raw_posix_wait_status: Option<i32>,
}

impl ProcessExit {
    pub fn run(&self) -> &RunKey {
        &self.run
    }
    pub fn pid(&self) -> u32 {
        self.pid
    }
    pub fn outcome(&self) -> &ExitOutcome {
        &self.outcome
    }
    pub fn success(&self) -> bool {
        matches!(self.outcome, ExitOutcome::Exited { code: 0 })
    }
}

pub struct OwnedWorker {
    child: Child,
    run: RunKey,
    program: String,
    args: Vec<String>,
    exit: Option<ProcessExit>,
}

impl OwnedWorker {
    /// Host code supplies an already-authorized executable, never an untrusted JSON command.
    /// Pipes are disabled here so an unconsumed stream cannot deadlock the worker.
    pub fn spawn(command: &mut Command, run: RunKey) -> Result<Self, ContractError> {
        run.validate()?;
        let program = command.get_program().to_string_lossy().into_owned();
        let args = command.get_args().map(|a| a.to_string_lossy().into_owned()).collect();
        command.stdin(Stdio::null()).stdout(Stdio::null()).stderr(Stdio::null());
        let child = command.spawn().map_err(|e| ContractError::Worker(e.to_string()))?;
        Ok(Self { child, run, program, args, exit: None })
    }

    pub fn pid(&self) -> u32 {
        self.child.id()
    }

    pub fn try_exit(&mut self) -> Result<Option<ProcessExit>, ContractError> {
        if let Some(exit) = &self.exit {
            return Ok(Some(exit.clone()));
        }
        let status = self.child.try_wait().map_err(|e| ContractError::Worker(e.to_string()))?;
        if let Some(status) = status {
            let (outcome, raw_posix_wait_status) = outcome(status);
            self.exit = Some(ProcessExit {
                run: self.run.clone(),
                pid: self.child.id(),
                parent_pid: std::process::id(),
                program: self.program.clone(),
                args: self.args.clone(),
                observer: "std::process::Child::try_wait",
                layer: "direct_child",
                observed_unix_ms: SystemTime::now().duration_since(UNIX_EPOCH).ok().map(|d| d.as_millis()),
                outcome,
                raw_posix_wait_status,
            });
        }
        Ok(self.exit.clone())
    }

    pub fn wait_for_exit(&mut self, budget: Duration) -> Result<ProcessExit, ContractError> {
        if budget.is_zero() || budget > Duration::from_secs(60) {
            return Err(ContractError::Invalid("wait budget must be in (0, 60] seconds"));
        }
        let began = Instant::now();
        loop {
            if let Some(exit) = self.try_exit()? {
                return Ok(exit);
            }
            if began.elapsed() >= budget {
                return Err(ContractError::WorkerTimeout);
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    /// Forced termination is a request until wait observes an exit; no synthetic signal.
    pub fn terminate_and_reap(&mut self, budget: Duration) -> Result<ProcessExit, ContractError> {
        if budget.is_zero() || budget > Duration::from_secs(60) {
            return Err(ContractError::Invalid("wait budget must be in (0, 60] seconds"));
        }
        if let Some(exit) = self.try_exit()? {
            return Ok(exit);
        }
        if let Err(e) = self.child.kill() {
            if let Some(exit) = self.try_exit()? {
                return Ok(exit);
            }
            return Err(ContractError::Worker(e.to_string()));
        }
        self.wait_for_exit(budget)
    }
}

impl Drop for OwnedWorker {
    fn drop(&mut self) {
        if self.exit.is_none() {
            // Best-effort RAII guard for early error paths; explicit callers retain receipts.
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

fn outcome(status: ExitStatus) -> (ExitOutcome, Option<i32>) {
    #[cfg(unix)]
    {
        use std::os::unix::process::ExitStatusExt;
        let result = match (status.code(), status.signal()) {
            (Some(code), _) => ExitOutcome::Exited { code },
            (_, Some(number)) => ExitOutcome::Signaled { number, core_dumped: status.core_dumped() },
            _ => ExitOutcome::Unknown,
        };
        (result, Some(status.into_raw()))
    }
    #[cfg(not(unix))]
    {
        (status.code().map_or(ExitOutcome::Unknown, |code| ExitOutcome::Exited { code }), None)
    }
}
