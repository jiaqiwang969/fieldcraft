//! In-memory run ledger; not a persistent identity or exactly-once execution service.

use serde::{Deserialize, Serialize};

use super::{ContractError, EvidenceKind, Observed, RunKey, SourceRef};

pub const MAX_FRAMES: u64 = 1_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunGoal {
    pub last_frame: u64,
    pub end_time_s: f64,
}

impl RunGoal {
    pub fn validate(self) -> Result<(), ContractError> {
        if self.last_frame == 0 || self.last_frame > MAX_FRAMES || !self.end_time_s.is_finite() || self.end_time_s <= 0.0 {
            return Err(ContractError::Invalid("run goal"));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FrameFacts {
    pub run: RunKey,
    /// Advanced frames start at one. Initialization is never a solved frame.
    pub frame: u64,
    pub time_s: f64,
    pub completed: Observed<bool>,
    pub converged: Observed<bool>,
    pub hit_newton_limit: Observed<bool>,
    pub hit_line_search_limit: Observed<bool>,
    pub world_valid: Observed<bool>,
    pub snapshot_complete: Observed<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FrameIssue {
    NotCompleted,
    NotConverged,
    NewtonLimit,
    LineSearchLimit,
    InvalidWorld,
    IncompleteSnapshot,
    UnknownDiagnostic(String),
    InvalidReceipt(String),
    MissingFrame,
}

impl FrameFacts {
    pub fn issue(&self, kind: EvidenceKind) -> Option<FrameIssue> {
        let facts = [
            (&self.completed, true, FrameIssue::NotCompleted, "completed"),
            (&self.converged, true, FrameIssue::NotConverged, "converged"),
            (&self.hit_newton_limit, false, FrameIssue::NewtonLimit, "hit_newton_limit"),
            (&self.hit_line_search_limit, false, FrameIssue::LineSearchLimit, "hit_line_search_limit"),
            (&self.world_valid, true, FrameIssue::InvalidWorld, "world_valid"),
            (&self.snapshot_complete, true, FrameIssue::IncompleteSnapshot, "snapshot_complete"),
        ];
        // Known failures take precedence, without rewriting any source fact.
        for (fact, expected, issue, _) in &facts {
            if let Err(e) = fact.validate(kind) {
                return Some(FrameIssue::InvalidReceipt(e.to_string()));
            }
            if matches!(fact, Observed::Known { value, .. } if value != expected) {
                return Some(issue.clone());
            }
        }
        for (fact, _, _, name) in facts {
            if let Observed::Unknown { reason } = fact {
                return Some(FrameIssue::UnknownDiagnostic(format!("{name}: {reason}")));
            }
        }
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RunState {
    Created,
    Ready,
    Advancing,
    CancelPending,
    Succeeded,
    Failed,
    Cancelled,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SolveOutcome {
    Running,
    Succeeded,
    Failed,
    Cancelled,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FailureBoundary {
    pub frame: u64,
    pub issue: FrameIssue,
    /// Present for an observed frame, absent for a transport gap.
    pub facts: Option<FrameFacts>,
}

/// Private state cannot be reconstructed from a producer's claimed watermarks.
#[derive(Debug)]
pub struct RunLedger {
    key: RunKey,
    goal: RunGoal,
    kind: EvidenceKind,
    state: RunState,
    last_qualified: u64,
    last_time_s: f64,
    operation_last: u64,
    barrier: Option<FailureBoundary>,
    terminal_source: Option<SourceRef>,
}

impl RunLedger {
    pub fn new(key: RunKey, goal: RunGoal, kind: EvidenceKind) -> Result<Self, ContractError> {
        key.validate()?;
        goal.validate()?;
        Ok(Self {
            key,
            goal,
            kind,
            state: RunState::Created,
            last_qualified: 0,
            last_time_s: 0.0,
            operation_last: 0,
            barrier: None,
            terminal_source: None,
        })
    }

    pub fn key(&self) -> &RunKey {
        &self.key
    }
    pub fn goal(&self) -> RunGoal {
        self.goal
    }
    pub fn evidence_kind(&self) -> EvidenceKind {
        self.kind
    }
    pub fn state(&self) -> RunState {
        self.state
    }
    pub fn last_qualified(&self) -> u64 {
        self.last_qualified
    }
    pub fn barrier(&self) -> Option<&FailureBoundary> {
        self.barrier.as_ref()
    }

    fn check_key(&self, key: &RunKey) -> Result<(), ContractError> {
        if key != &self.key {
            return Err(ContractError::StaleRun);
        }
        Ok(())
    }

    pub fn initialize(&mut self, key: &RunKey) -> Result<(), ContractError> {
        self.check_key(key)?;
        if self.state != RunState::Created {
            return Err(ContractError::InvalidState);
        }
        self.state = RunState::Ready;
        Ok(())
    }

    pub fn begin_advance(&mut self, key: &RunKey, steps: u64) -> Result<(), ContractError> {
        self.check_key(key)?;
        if self.state != RunState::Ready || self.barrier.is_some() {
            return Err(ContractError::InvalidState);
        }
        let end = self.last_qualified.checked_add(steps).ok_or(ContractError::Invalid("step count"))?;
        if steps == 0 || end > self.goal.last_frame {
            return Err(ContractError::Invalid("step count"));
        }
        self.operation_last = end;
        self.state = RunState::Advancing;
        Ok(())
    }

    pub fn accept_frame(&mut self, facts: FrameFacts) -> Result<(), ContractError> {
        self.check_key(&facts.run)?;
        if !matches!(self.state, RunState::Advancing | RunState::CancelPending) || self.barrier.is_some() {
            return Err(ContractError::InvalidState);
        }
        let next = self.last_qualified + 1;
        let issue = if facts.frame != next || facts.frame > self.operation_last {
            Some(FrameIssue::MissingFrame)
        } else if !facts.time_s.is_finite()
            || facts.time_s <= self.last_time_s
            || facts.time_s > self.goal.end_time_s
            || (facts.frame == self.goal.last_frame && facts.time_s != self.goal.end_time_s)
        {
            Some(FrameIssue::InvalidReceipt("time axis".into()))
        } else {
            facts.issue(self.kind)
        };
        if let Some(issue) = issue {
            self.state = if matches!(issue, FrameIssue::UnknownDiagnostic(_) | FrameIssue::MissingFrame | FrameIssue::InvalidReceipt(_)) {
                RunState::Unknown
            } else {
                RunState::Failed
            };
            self.barrier = Some(FailureBoundary { frame: next, issue: issue.clone(), facts: Some(facts) });
            return Err(ContractError::FrameRejected(issue));
        }
        self.last_qualified = facts.frame;
        self.last_time_s = facts.time_s;
        if self.last_qualified == self.operation_last && self.state != RunState::CancelPending {
            self.state = RunState::Ready;
        }
        Ok(())
    }

    /// Run completion requires an explicit terminal source, beyond any batch completion.
    pub fn finish(&mut self, key: &RunKey, source: SourceRef) -> Result<(), ContractError> {
        self.check_key(key)?;
        source.validate()?;
        if self.state != RunState::Ready || self.last_qualified != self.goal.last_frame || source.kind != self.kind {
            return Err(ContractError::InvalidState);
        }
        self.state = RunState::Succeeded;
        self.terminal_source = Some(source);
        Ok(())
    }

    pub fn request_cancel(&mut self, key: &RunKey) -> Result<(), ContractError> {
        self.check_key(key)?;
        if !matches!(self.state, RunState::Ready | RunState::Advancing) {
            return Err(ContractError::InvalidState);
        }
        self.state = RunState::CancelPending;
        Ok(())
    }

    pub fn solve_outcome(&self) -> SolveOutcome {
        match self.state {
            RunState::Succeeded => SolveOutcome::Succeeded,
            RunState::Failed => SolveOutcome::Failed,
            RunState::Cancelled => SolveOutcome::Cancelled,
            RunState::Unknown => SolveOutcome::Unknown,
            _ => SolveOutcome::Running,
        }
    }

    /// Exit evidence is produced only by the parent waiting on its owned child.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn observe_exit(&mut self, exit: &super::worker::ProcessExit) -> Result<(), ContractError> {
        self.check_key(exit.run())?;
        if self.barrier.is_some() {
            return Ok(());
        }
        if self.state == RunState::CancelPending {
            self.state = RunState::Cancelled;
        } else if !exit.success() {
            self.state = RunState::Failed;
        } else if self.state != RunState::Succeeded {
            // Exit zero does not supply missing frame or terminal solve receipts.
            self.state = RunState::Unknown;
        }
        Ok(())
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn reset_after_exit(&mut self, exit: &super::worker::ProcessExit, next_generation: String) -> Result<(), ContractError> {
        self.check_key(exit.run())?;
        if matches!(self.state, RunState::Advancing | RunState::CancelPending) || next_generation == self.key.worker_generation {
            return Err(ContractError::InvalidState);
        }
        let next = RunKey {
            run_id: self.key.run_id.clone(),
            epoch: self.key.epoch.checked_add(1).ok_or(ContractError::Invalid("epoch overflow"))?,
            worker_generation: next_generation,
        };
        *self = Self::new(next, self.goal, self.kind)?;
        Ok(())
    }
}
