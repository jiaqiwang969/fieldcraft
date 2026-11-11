//! G26 load-transfer contract.
//!
//! This module is deliberately isolated from the existing type-13 codecs.  It provides a
//! bounded, reversible representation for point loads, the work-conjugate `Hᵀ` mapping, the
//! frozen conservation and one-way applicability checks, and a crash receipt.  Existing result
//! records are not decoded or rewritten here.

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

pub const SCHEMA: &str = "fieldcraft-load-transfer/1";
pub const RESULT_KIND: u8 = 13;
pub const RELATIVE_TOLERANCE: f64 = 1e-10;
pub const ABSOLUTE_TOLERANCE: f64 = 1e-12;
pub const ACTIVE_SET_TOLERANCE: f64 = 0.05;
pub const GAP_TOLERANCE: f64 = 0.02;
pub const PEAK_SAMPLE_LOSS_TOLERANCE: f64 = 0.01;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case", deny_unknown_fields)]
pub enum UnknownOr<T> {
    Known { value: T },
    Unknown { reason: String },
}

impl<T> UnknownOr<T> {
    pub fn unknown(reason: impl Into<String>) -> Self {
        Self::Unknown { reason: reason.into() }
    }

    pub fn is_unknown(&self) -> bool {
        matches!(self, Self::Unknown { .. })
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoadIdentity {
    pub archive_key: String,
    pub run_id: String,
    pub epoch: u64,
    pub worker_generation: String,
    pub model_revision: String,
    pub mesh_digest: String,
    pub coordinate_frame: String,
    pub units: String,
}

impl LoadIdentity {
    pub fn validate(&self) -> Result<(), LoadTransferError> {
        let values =
            [&self.archive_key, &self.run_id, &self.worker_generation, &self.model_revision, &self.mesh_digest, &self.coordinate_frame, &self.units];
        if values.iter().any(|value| !token_valid(value)) {
            return Err(LoadTransferError::Invalid("load identity"));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PointForce {
    pub point_id: String,
    pub position_m: [f64; 3],
    pub force_n: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeContribution {
    pub node_id: u64,
    /// The 3×3 block of `H` for this point and node.
    pub h_block: [[f64; 3]; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PointMapping {
    pub point_id: String,
    pub contributions: Vec<NodeContribution>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MappingNode {
    pub node_id: u64,
    pub position_m: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MappingSpec {
    pub origin_m: [f64; 3],
    pub nodes: Vec<MappingNode>,
    pub rows: Vec<PointMapping>,
    pub mapping_sha256: String,
}

impl MappingSpec {
    pub fn validate(&self) -> Result<(), LoadTransferError> {
        finite_vec(&self.origin_m)?;
        if self.nodes.is_empty() || self.rows.is_empty() || !is_digest(&self.mapping_sha256) {
            return Err(LoadTransferError::Invalid("mapping shape or digest"));
        }
        let mut node_ids = BTreeSet::new();
        for node in &self.nodes {
            finite_vec(&node.position_m)?;
            if !node_ids.insert(node.node_id) {
                return Err(LoadTransferError::DuplicateNode(node.node_id));
            }
        }
        let mut point_ids = BTreeSet::new();
        for row in &self.rows {
            if !token_valid(&row.point_id) || !point_ids.insert(row.point_id.clone()) {
                return Err(LoadTransferError::DuplicatePoint(row.point_id.clone()));
            }
            if row.contributions.is_empty() {
                return Err(LoadTransferError::Invalid("empty mapping row"));
            }
            let mut row_nodes = BTreeSet::new();
            for contribution in &row.contributions {
                if !node_ids.contains(&contribution.node_id) || !row_nodes.insert(contribution.node_id) {
                    return Err(LoadTransferError::Invalid("mapping node"));
                }
                for component in contribution.h_block.iter().flatten() {
                    finite(*component)?;
                }
            }
        }
        Ok(())
    }

    pub fn map_frame(&self, frame: &LoadFrame) -> Result<MappedFrame, LoadTransferError> {
        self.validate()?;
        frame.validate()?;
        if frame.points.len() != self.rows.len() {
            return Err(LoadTransferError::MappingMismatch);
        }
        let mut force_by_node: BTreeMap<u64, [f64; 3]> = self.nodes.iter().map(|node| (node.node_id, [0.0; 3])).collect();
        let mut seen = BTreeSet::new();
        for point in &frame.points {
            if !seen.insert(point.point_id.clone()) {
                return Err(LoadTransferError::DuplicatePoint(point.point_id.clone()));
            }
            let row = self.rows.iter().find(|row| row.point_id == point.point_id).ok_or(LoadTransferError::MissingPoint(point.point_id.clone()))?;
            for contribution in &row.contributions {
                let target = force_by_node.get_mut(&contribution.node_id).ok_or(LoadTransferError::MappingMismatch)?;
                let contribution_force = transpose_mul(contribution.h_block, point.force_n);
                for axis in 0..3 {
                    target[axis] += contribution_force[axis];
                    finite(target[axis])?;
                }
            }
        }
        if seen.len() != self.rows.len() {
            return Err(LoadTransferError::MappingMismatch);
        }
        let nodes = self
            .nodes
            .iter()
            .map(|node| NodeLoad { node_id: node.node_id, position_m: node.position_m, force_n: force_by_node[&node.node_id] })
            .collect();
        Ok(MappedFrame { frame: frame.clone(), nodes })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FrameFacts {
    pub completed: bool,
    pub converged: bool,
    pub hit_newton_limit: bool,
    pub hit_line_search_limit: bool,
    pub kappa_pa: UnknownOr<f64>,
    pub truncated: UnknownOr<bool>,
}

impl FrameFacts {
    fn validate(&self) -> Result<(), LoadTransferError> {
        if let UnknownOr::Known { value } = &self.kappa_pa {
            if !value.is_finite() || *value < 0.0 {
                return Err(LoadTransferError::Invalid("kappa"));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LoadFrame {
    pub frame_index: u64,
    pub time_s: f64,
    pub source_dt_s: f64,
    pub points: Vec<PointForce>,
    pub facts: FrameFacts,
}

impl LoadFrame {
    pub fn validate(&self) -> Result<(), LoadTransferError> {
        if !self.time_s.is_finite() || !self.source_dt_s.is_finite() || self.source_dt_s < 0.0 {
            return Err(LoadTransferError::NonFinite("frame time"));
        }
        self.facts.validate()?;
        let mut ids = BTreeSet::new();
        for point in &self.points {
            if !token_valid(&point.point_id) || !ids.insert(point.point_id.clone()) {
                return Err(LoadTransferError::DuplicatePoint(point.point_id.clone()));
            }
            finite_vec(&point.position_m)?;
            finite_vec(&point.force_n)?;
        }
        if self.points.is_empty() {
            return Err(LoadTransferError::Invalid("empty load frame"));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NodeLoad {
    pub node_id: u64,
    pub position_m: [f64; 3],
    pub force_n: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MappedFrame {
    pub frame: LoadFrame,
    pub nodes: Vec<NodeLoad>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConservationReport {
    pub force_residual_n: [f64; 3],
    pub moment_residual_nm: [f64; 3],
    pub virtual_work_residual_j: f64,
    pub force_tolerance_n: f64,
    pub moment_tolerance_nm: f64,
    pub virtual_work_tolerance_j: f64,
    pub passed: bool,
}

pub fn check_conservation(
    frame: &LoadFrame,
    mapped: &MappedFrame,
    mapping: &MappingSpec,
    virtual_node_displacements_m: &[[f64; 3]],
) -> Result<ConservationReport, LoadTransferError> {
    frame.validate()?;
    if mapped.frame != *frame || mapped.nodes.len() != mapping.nodes.len() || virtual_node_displacements_m.len() != mapping.nodes.len() {
        return Err(LoadTransferError::MappingMismatch);
    }
    let source_force = frame.points.iter().fold([0.0; 3], |sum, point| add(sum, point.force_n));
    let target_force = mapped.nodes.iter().fold([0.0; 3], |sum, node| add(sum, node.force_n));
    let source_moment = frame.points.iter().fold([0.0; 3], |sum, point| add(sum, cross(sub(point.position_m, mapping.origin_m), point.force_n)));
    let target_moment = mapped.nodes.iter().fold([0.0; 3], |sum, node| add(sum, cross(sub(node.position_m, mapping.origin_m), node.force_n)));
    let force_residual_n = sub(target_force, source_force);
    let moment_residual_nm = sub(target_moment, source_moment);

    let mut point_displacements: BTreeMap<&str, [f64; 3]> = BTreeMap::new();
    for row in &mapping.rows {
        let mut displacement = [0.0; 3];
        for contribution in &row.contributions {
            let node_index = mapping.nodes.iter().position(|node| node.node_id == contribution.node_id).ok_or(LoadTransferError::MappingMismatch)?;
            displacement = add(displacement, mat_mul(contribution.h_block, virtual_node_displacements_m[node_index]));
        }
        point_displacements.insert(row.point_id.as_str(), displacement);
    }
    let source_work = frame.points.iter().try_fold(0.0, |sum, point| {
        let displacement = point_displacements.get(point.point_id.as_str()).ok_or(LoadTransferError::MissingPoint(point.point_id.clone()))?;
        Ok::<f64, LoadTransferError>(sum + dot(point.force_n, *displacement))
    })?;
    let target_work = mapped.nodes.iter().enumerate().fold(0.0, |sum, (index, node)| sum + dot(node.force_n, virtual_node_displacements_m[index]));
    let virtual_work_residual_j = target_work - source_work;

    let force_scale = frame.points.iter().map(|point| norm1(point.force_n)).sum::<f64>().max(norm1(source_force)).max(1.0);
    let moment_scale = frame
        .points
        .iter()
        .map(|point| norm1(cross(sub(point.position_m, mapping.origin_m), point.force_n)))
        .sum::<f64>()
        .max(norm1(source_moment))
        .max(1.0);
    let work_scale = source_work.abs().max(target_work.abs()).max(1.0);
    let force_tolerance_n = tolerance(force_scale);
    let moment_tolerance_nm = tolerance(moment_scale);
    let virtual_work_tolerance_j = tolerance(work_scale);
    let passed = norm_inf(force_residual_n) <= force_tolerance_n
        && norm_inf(moment_residual_nm) <= moment_tolerance_nm
        && virtual_work_residual_j.abs() <= virtual_work_tolerance_j;
    Ok(ConservationReport {
        force_residual_n,
        moment_residual_nm,
        virtual_work_residual_j,
        force_tolerance_n,
        moment_tolerance_nm,
        virtual_work_tolerance_j,
        passed,
    })
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImpulseReport {
    pub impulse_residual_ns: [f64; 3],
    pub angular_impulse_residual_nms: [f64; 3],
    pub impulse_tolerance_ns: f64,
    pub angular_impulse_tolerance_nms: f64,
    pub passed: bool,
}

pub fn check_time_integrals(frames: &[LoadFrame], mapped: &[MappedFrame], origin_m: [f64; 3]) -> Result<ImpulseReport, LoadTransferError> {
    if frames.is_empty() || frames.len() != mapped.len() {
        return Err(LoadTransferError::MappingMismatch);
    }
    for (index, frame) in frames.iter().enumerate() {
        frame.validate()?;
        if mapped[index].frame != *frame {
            return Err(LoadTransferError::MappingMismatch);
        }
        if index > 0 && frame.time_s < frames[index - 1].time_s {
            return Err(LoadTransferError::Invalid("non-monotonic time axis"));
        }
    }
    let mut source_impulse = [0.0; 3];
    let mut target_impulse = [0.0; 3];
    let mut source_angular = [0.0; 3];
    let mut target_angular = [0.0; 3];
    for index in 1..frames.len() {
        let dt = frames[index].time_s - frames[index - 1].time_s;
        let source_prev = frame_force(&frames[index - 1]);
        let source_now = frame_force(&frames[index]);
        let target_prev = node_force(&mapped[index - 1]);
        let target_now = node_force(&mapped[index]);
        source_impulse = add(source_impulse, scale(add(source_prev, source_now), 0.5 * dt));
        target_impulse = add(target_impulse, scale(add(target_prev, target_now), 0.5 * dt));
        source_angular =
            add(source_angular, scale(add(frame_moment(&frames[index - 1], origin_m), frame_moment(&frames[index], origin_m)), 0.5 * dt));
        target_angular = add(target_angular, scale(add(node_moment(&mapped[index - 1], origin_m), node_moment(&mapped[index], origin_m)), 0.5 * dt));
    }
    let impulse_residual_ns = sub(target_impulse, source_impulse);
    let angular_impulse_residual_nms = sub(target_angular, source_angular);
    let impulse_tolerance_ns = tolerance(norm1(source_impulse).max(1.0));
    let angular_impulse_tolerance_nms = tolerance(norm1(source_angular).max(1.0));
    let passed = norm_inf(impulse_residual_ns) <= impulse_tolerance_ns && norm_inf(angular_impulse_residual_nms) <= angular_impulse_tolerance_nms;
    Ok(ImpulseReport { impulse_residual_ns, angular_impulse_residual_nms, impulse_tolerance_ns, angular_impulse_tolerance_nms, passed })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ApplicabilityStatus {
    Unknown,
    Applicable,
    Inapplicable,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentChange {
    pub component_id: String,
    pub weight_fraction: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ActivationMetric {
    pub frame_index: u64,
    pub significant: bool,
    pub transition: bool,
    pub active_area_difference: f64,
    pub gap_difference: f64,
    pub component_changes: Vec<ComponentChange>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ForcePrecisionLimit {
    pub kappa_pa: UnknownOr<f64>,
    pub truncated: UnknownOr<bool>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case", deny_unknown_fields)]
pub enum RuntimeQualification {
    Verified { source: String },
    Unverified { reason: String },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ApplicabilityInput {
    pub metrics: Vec<ActivationMetric>,
    pub peak_sampling_loss: f64,
    pub required_fields: bool,
    pub structural_feedback: bool,
    pub force_precision: ForcePrecisionLimit,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ApplicabilityFailure {
    pub frame_index: Option<u64>,
    pub metric: String,
    pub observed: f64,
    pub limit: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ApplicabilityReport {
    pub geometry_status: ApplicabilityStatus,
    pub adoption_status: ApplicabilityStatus,
    pub first_failure: Option<ApplicabilityFailure>,
    pub force_precision: ForcePrecisionLimit,
}

pub fn evaluate_applicability(input: &ApplicabilityInput, qualification: &RuntimeQualification) -> Result<ApplicabilityReport, LoadTransferError> {
    if !input.peak_sampling_loss.is_finite() || input.peak_sampling_loss < 0.0 {
        return Err(LoadTransferError::Invalid("peak sampling loss"));
    }
    let mut geometry_status = if input.required_fields && input.structural_feedback && !input.metrics.is_empty() {
        ApplicabilityStatus::Applicable
    } else {
        ApplicabilityStatus::Unknown
    };
    let mut first_failure = None;
    if geometry_status == ApplicabilityStatus::Applicable {
        for metric in &input.metrics {
            if !metric.active_area_difference.is_finite()
                || !metric.gap_difference.is_finite()
                || metric.active_area_difference < 0.0
                || metric.gap_difference < 0.0
            {
                return Err(LoadTransferError::Invalid("activation metric"));
            }
            if !(metric.significant || metric.transition) {
                continue;
            }
            if metric.active_area_difference > ACTIVE_SET_TOLERANCE {
                first_failure = Some(ApplicabilityFailure {
                    frame_index: Some(metric.frame_index),
                    metric: "active_area_difference".to_string(),
                    observed: metric.active_area_difference,
                    limit: ACTIVE_SET_TOLERANCE,
                });
                break;
            }
            if let Some(component) = metric.component_changes.iter().find(|change| change.weight_fraction > ACTIVE_SET_TOLERANCE) {
                first_failure = Some(ApplicabilityFailure {
                    frame_index: Some(metric.frame_index),
                    metric: format!("component_weight:{}", component.component_id),
                    observed: component.weight_fraction,
                    limit: ACTIVE_SET_TOLERANCE,
                });
                break;
            }
            if metric.gap_difference > GAP_TOLERANCE {
                first_failure = Some(ApplicabilityFailure {
                    frame_index: Some(metric.frame_index),
                    metric: "gap_difference".to_string(),
                    observed: metric.gap_difference,
                    limit: GAP_TOLERANCE,
                });
                break;
            }
        }
        if first_failure.is_none() && input.peak_sampling_loss > PEAK_SAMPLE_LOSS_TOLERANCE {
            first_failure = Some(ApplicabilityFailure {
                frame_index: None,
                metric: "peak_sampling_loss".to_string(),
                observed: input.peak_sampling_loss,
                limit: PEAK_SAMPLE_LOSS_TOLERANCE,
            });
        }
        if first_failure.is_some() {
            geometry_status = ApplicabilityStatus::Inapplicable;
        }
    }
    let adoption_status = match geometry_status {
        ApplicabilityStatus::Inapplicable => ApplicabilityStatus::Inapplicable,
        ApplicabilityStatus::Unknown => ApplicabilityStatus::Unknown,
        ApplicabilityStatus::Applicable => match qualification {
            RuntimeQualification::Verified { .. } => ApplicabilityStatus::Applicable,
            RuntimeQualification::Unverified { .. } => ApplicabilityStatus::Unknown,
        },
    };
    Ok(ApplicabilityReport { geometry_status, adoption_status, first_failure, force_precision: input.force_precision.clone() })
}

pub fn pre_research_impact_report() -> Result<ApplicabilityReport, LoadTransferError> {
    let metrics = (0..154)
        .map(|index| ActivationMetric {
            frame_index: index,
            significant: true,
            transition: false,
            active_area_difference: if index < 27 { 1.0 } else { 0.0 },
            gap_difference: 0.0,
            component_changes: Vec::new(),
        })
        .collect();
    evaluate_applicability(
        &ApplicabilityInput {
            metrics,
            peak_sampling_loss: 0.0,
            required_fields: true,
            structural_feedback: true,
            force_precision: ForcePrecisionLimit {
                kappa_pa: UnknownOr::unknown("libuipc 0.0.29 does not expose per-frame effective kappa"),
                truncated: UnknownOr::unknown("per-frame truncation is not exposed"),
            },
        },
        &RuntimeQualification::Unverified { reason: "G25 independent runtime qualification is pending".to_string() },
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BackendState {
    Running,
    Completed,
    Failed,
    Cancelled,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FirstError {
    pub frame_index: Option<u64>,
    pub phase: String,
    pub kind: String,
    pub message: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CrashReceipt {
    pub state: BackendState,
    pub exit_code: Option<i32>,
    pub signal: Option<i32>,
    pub first_error: Option<FirstError>,
    pub last_completed_frame: Option<u64>,
    pub last_persisted_frame: Option<u64>,
    pub requested_last_frame: Option<u64>,
    pub stdout_sha256: String,
    pub stderr_sha256: String,
}

impl CrashReceipt {
    pub fn validate(&self) -> Result<(), LoadTransferError> {
        if !is_digest(&self.stdout_sha256) || !is_digest(&self.stderr_sha256) {
            return Err(LoadTransferError::Invalid("crash log digest"));
        }
        if self.exit_code == Some(0) && self.signal.is_some() {
            return Err(LoadTransferError::Invalid("exit code and signal"));
        }
        if self.state == BackendState::Completed
            && (self.exit_code != Some(0)
                || self.signal.is_some()
                || self.first_error.is_some()
                || self.last_completed_frame != self.requested_last_frame)
        {
            return Err(LoadTransferError::Invalid("completed crash receipt"));
        }
        Ok(())
    }

    pub fn solver_success(&self) -> bool {
        self.validate().is_ok() && self.state == BackendState::Completed && self.exit_code == Some(0)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct LoadTransferArchive {
    pub identity: LoadIdentity,
    pub mapping: MappingSpec,
    pub frames: Vec<LoadFrame>,
    pub applicability: ApplicabilityReport,
    pub crash: Option<CrashReceipt>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DecodedArchive {
    pub archive: LoadTransferArchive,
    pub mapped_frames: Vec<MappedFrame>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ArchiveBundle {
    pub manifest_json: Vec<u8>,
    pub point_binary: Vec<u8>,
    pub nodal_binary: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum LoadTransferError {
    #[error("invalid load-transfer value: {0}")]
    Invalid(&'static str),
    #[error("non-finite load-transfer value: {0}")]
    NonFinite(&'static str),
    #[error("duplicate point `{0}`")]
    DuplicatePoint(String),
    #[error("duplicate node `{0}`")]
    DuplicateNode(u64),
    #[error("missing point `{0}`")]
    MissingPoint(String),
    #[error("mapping does not match the frame")]
    MappingMismatch,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ArchiveError {
    #[error("invalid archive: {0}")]
    Invalid(&'static str),
    #[error("archive JSON: {0}")]
    Json(String),
    #[error("archive checksum mismatch")]
    Checksum,
    #[error("archive mapping: {0}")]
    Mapping(#[from] LoadTransferError),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ArchiveWire {
    schema: String,
    result_kind: u8,
    identity: LoadIdentity,
    mapping: MappingSpec,
    frames: Vec<FrameWire>,
    applicability: ApplicabilityReport,
    crash: Option<CrashReceipt>,
    point_bytes: u64,
    point_sha256: String,
    nodal_bytes: u64,
    nodal_sha256: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrameWire {
    frame_index: u64,
    time_s: f64,
    source_dt_s: f64,
    facts: FrameFacts,
    points: Vec<PointMeta>,
    point_offset: u64,
    nodal_offset: u64,
    node_count: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PointMeta {
    point_id: String,
    position_m: [f64; 3],
}

impl LoadTransferArchive {
    pub fn encode(&self) -> Result<ArchiveBundle, ArchiveError> {
        self.identity.validate().map_err(ArchiveError::Mapping)?;
        self.mapping.validate().map_err(ArchiveError::Mapping)?;
        if self.frames.is_empty() {
            return Err(ArchiveError::Invalid("empty archive"));
        }
        if let Some(crash) = &self.crash {
            crash.validate().map_err(ArchiveError::Mapping)?;
        }
        let mut point_binary = Vec::new();
        let mut nodal_binary = Vec::new();
        let mut frames = Vec::with_capacity(self.frames.len());
        for frame in &self.frames {
            let mapped = self.mapping.map_frame(frame).map_err(ArchiveError::Mapping)?;
            let point_offset = point_binary.len() as u64;
            for point in &frame.points {
                append_vec3(&mut point_binary, point.force_n);
            }
            let nodal_offset = nodal_binary.len() as u64;
            for node in &mapped.nodes {
                append_vec3(&mut nodal_binary, node.force_n);
            }
            frames.push(FrameWire {
                frame_index: frame.frame_index,
                time_s: frame.time_s,
                source_dt_s: frame.source_dt_s,
                facts: frame.facts.clone(),
                points: frame.points.iter().map(|point| PointMeta { point_id: point.point_id.clone(), position_m: point.position_m }).collect(),
                point_offset,
                nodal_offset,
                node_count: mapped.nodes.len() as u64,
            });
        }
        let manifest = ArchiveWire {
            schema: SCHEMA.to_string(),
            result_kind: RESULT_KIND,
            identity: self.identity.clone(),
            mapping: self.mapping.clone(),
            frames,
            applicability: self.applicability.clone(),
            crash: self.crash.clone(),
            point_bytes: point_binary.len() as u64,
            point_sha256: fieldcraft_io::sha256_hex(&point_binary),
            nodal_bytes: nodal_binary.len() as u64,
            nodal_sha256: fieldcraft_io::sha256_hex(&nodal_binary),
        };
        let manifest_json = serde_json::to_vec(&manifest).map_err(|error| ArchiveError::Json(error.to_string()))?;
        Ok(ArchiveBundle { manifest_json, point_binary, nodal_binary })
    }

    pub fn decode(bundle: &ArchiveBundle) -> Result<DecodedArchive, ArchiveError> {
        let wire: ArchiveWire = serde_json::from_slice(&bundle.manifest_json).map_err(|error| ArchiveError::Json(error.to_string()))?;
        if wire.schema != SCHEMA
            || wire.result_kind != RESULT_KIND
            || wire.point_bytes != bundle.point_binary.len() as u64
            || wire.nodal_bytes != bundle.nodal_binary.len() as u64
        {
            return Err(ArchiveError::Invalid("schema, kind or byte count"));
        }
        if fieldcraft_io::sha256_hex(&bundle.point_binary) != wire.point_sha256
            || fieldcraft_io::sha256_hex(&bundle.nodal_binary) != wire.nodal_sha256
        {
            return Err(ArchiveError::Checksum);
        }
        wire.identity.validate().map_err(ArchiveError::Mapping)?;
        wire.mapping.validate().map_err(ArchiveError::Mapping)?;
        if let Some(crash) = &wire.crash {
            crash.validate().map_err(ArchiveError::Mapping)?;
        }
        let mut frames = Vec::with_capacity(wire.frames.len());
        let mut mapped_frames = Vec::with_capacity(wire.frames.len());
        let mut expected_point_offset = 0_usize;
        let mut expected_nodal_offset = 0_usize;
        let mut previous_frame_index = None;
        for frame_wire in &wire.frames {
            let point_bytes = frame_wire.points.len().checked_mul(24).ok_or(ArchiveError::Invalid("point size overflow"))?;
            let nodal_bytes = (frame_wire.node_count as usize).checked_mul(24).ok_or(ArchiveError::Invalid("node size overflow"))?;
            let point_offset = usize::try_from(frame_wire.point_offset).map_err(|_| ArchiveError::Invalid("point offset"))?;
            let nodal_offset = usize::try_from(frame_wire.nodal_offset).map_err(|_| ArchiveError::Invalid("node offset"))?;
            let point_end = point_offset.checked_add(point_bytes).ok_or(ArchiveError::Invalid("point range"))?;
            let nodal_end = nodal_offset.checked_add(nodal_bytes).ok_or(ArchiveError::Invalid("node range"))?;
            if point_offset != expected_point_offset
                || nodal_offset != expected_nodal_offset
                || previous_frame_index.is_some_and(|previous| frame_wire.frame_index <= previous)
                || point_end > bundle.point_binary.len()
                || nodal_end > bundle.nodal_binary.len()
                || frame_wire.node_count != wire.mapping.nodes.len() as u64
            {
                return Err(ArchiveError::Invalid("frame range"));
            }
            expected_point_offset = point_end;
            expected_nodal_offset = nodal_end;
            previous_frame_index = Some(frame_wire.frame_index);
            let mut points = Vec::with_capacity(frame_wire.points.len());
            for (index, meta) in frame_wire.points.iter().enumerate() {
                let force = read_vec3(&bundle.point_binary[point_offset + index * 24..point_offset + (index + 1) * 24])?;
                points.push(PointForce { point_id: meta.point_id.clone(), position_m: meta.position_m, force_n: force });
            }
            let frame = LoadFrame {
                frame_index: frame_wire.frame_index,
                time_s: frame_wire.time_s,
                source_dt_s: frame_wire.source_dt_s,
                points,
                facts: frame_wire.facts.clone(),
            };
            let mapped = wire.mapping.map_frame(&frame).map_err(ArchiveError::Mapping)?;
            for (index, node) in mapped.nodes.iter().enumerate() {
                let stored = read_vec3(&bundle.nodal_binary[nodal_offset + index * 24..nodal_offset + (index + 1) * 24])?;
                if stored != node.force_n {
                    return Err(ArchiveError::Invalid("nodal force does not match Hᵀ"));
                }
            }
            frames.push(frame);
            mapped_frames.push(mapped);
        }
        if expected_point_offset != bundle.point_binary.len() || expected_nodal_offset != bundle.nodal_binary.len() {
            return Err(ArchiveError::Invalid("trailing binary bytes"));
        }
        Ok(DecodedArchive {
            archive: LoadTransferArchive {
                identity: wire.identity,
                mapping: wire.mapping,
                frames,
                applicability: wire.applicability,
                crash: wire.crash,
            },
            mapped_frames,
        })
    }
}

fn token_valid(value: &str) -> bool {
    !value.trim().is_empty() && value.len() <= 1024 && !value.chars().any(char::is_control)
}

fn is_digest(value: &str) -> bool {
    value.len() == 64 && value.as_bytes().iter().all(u8::is_ascii_hexdigit)
}

fn finite(value: f64) -> Result<(), LoadTransferError> {
    if value.is_finite() { Ok(()) } else { Err(LoadTransferError::NonFinite("number")) }
}

fn finite_vec(value: &[f64; 3]) -> Result<(), LoadTransferError> {
    value.iter().try_for_each(|component| finite(*component))
}

fn tolerance(scale: f64) -> f64 {
    (RELATIVE_TOLERANCE * scale).max(ABSOLUTE_TOLERANCE)
}

fn transpose_mul(matrix: [[f64; 3]; 3], vector: [f64; 3]) -> [f64; 3] {
    [
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2],
    ]
}

fn mat_mul(matrix: [[f64; 3]; 3], vector: [f64; 3]) -> [f64; 3] {
    [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]
}

fn append_vec3(bytes: &mut Vec<u8>, value: [f64; 3]) {
    for component in value {
        bytes.extend_from_slice(&component.to_le_bytes());
    }
}

fn read_vec3(bytes: &[u8]) -> Result<[f64; 3], ArchiveError> {
    if bytes.len() != 24 {
        return Err(ArchiveError::Invalid("f64 vector size"));
    }
    let mut value = [0.0; 3];
    for (index, slot) in value.iter_mut().enumerate() {
        let mut raw = [0_u8; 8];
        raw.copy_from_slice(&bytes[index * 8..(index + 1) * 8]);
        *slot = f64::from_le_bytes(raw);
        if !slot.is_finite() {
            return Err(ArchiveError::Invalid("non-finite binary force"));
        }
    }
    Ok(value)
}

fn add(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

fn sub(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn scale(value: [f64; 3], scalar: f64) -> [f64; 3] {
    [value[0] * scalar, value[1] * scalar, value[2] * scalar]
}

fn cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[1] * right[2] - left[2] * right[1], left[2] * right[0] - left[0] * right[2], left[0] * right[1] - left[1] * right[0]]
}

fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn norm1(value: [f64; 3]) -> f64 {
    value.iter().map(|component| component.abs()).sum()
}

fn norm_inf(value: [f64; 3]) -> f64 {
    value.iter().map(|component| component.abs()).fold(0.0, f64::max)
}

fn frame_force(frame: &LoadFrame) -> [f64; 3] {
    frame.points.iter().fold([0.0; 3], |sum, point| add(sum, point.force_n))
}

fn node_force(frame: &MappedFrame) -> [f64; 3] {
    frame.nodes.iter().fold([0.0; 3], |sum, node| add(sum, node.force_n))
}

fn frame_moment(frame: &LoadFrame, origin: [f64; 3]) -> [f64; 3] {
    frame.points.iter().fold([0.0; 3], |sum, point| add(sum, cross(sub(point.position_m, origin), point.force_n)))
}

fn node_moment(frame: &MappedFrame, origin: [f64; 3]) -> [f64; 3] {
    frame.nodes.iter().fold([0.0; 3], |sum, node| add(sum, cross(sub(node.position_m, origin), node.force_n)))
}
