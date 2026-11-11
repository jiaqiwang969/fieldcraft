//! The named `external_static_base` part carried by results kind 13.
//!
//! This is a new, explicitly selected kind-13 schema.  Existing trajectory schemas are kept
//! byte-for-byte compatible.  The checks in this module are contract checks for a bounded dense
//! small example; they do not certify the producer runtime or a physical model.

use crate::{Error, Geometry, Result, Run, codec, identity, invalid, sha256_hex};
use fieldcraft_model::Model;
use serde::{Deserialize, Serialize};

pub const STATIC_BASE_SCHEMA: &str = "fieldcraft-external-static-base/1";
pub const STATIC_BASE_NAME: &str = "external_static_base";
pub const MAX_DOF: usize = 4096;
pub const MAX_MATRIX_ENTRIES: usize = 16 * 1024 * 1024;
pub const MAX_HOLD_STEPS: u32 = 1_000_000;

pub const STATIC_RESIDUAL_MAX: f64 = 1e-6;
pub const STATIC_VELOCITY_MAX: f64 = 1e-6;
pub const STATIC_DRIFT_MAX: f64 = 1e-10;
pub const STATIC_INERTIA_MAX: f64 = 1e-6;
pub const STATIC_SUPPORT_MAX: f64 = 1e-12;
pub const GAP_MAX: f64 = 1e-10;
pub const BONDED_TANGENTIAL_MAX: f64 = 1e-10;
pub const STICK_SLIP_RATIO_MAX: f64 = 0.1;
pub const STICK_FRICTION_UTILIZATION_MAX: f64 = 0.9;
pub const TANGENT_ERROR_MAX: f64 = 1e-5;
pub const TANGENT_SYMMETRY_MAX: f64 = 1e-8;
pub const MASS_SYMMETRY_MAX: f64 = 1e-12;
pub const PROJECTION_ERROR_MAX: f64 = 1e-10;
pub const MODAL_RESIDUAL_MAX: f64 = 1e-8;
pub const MASS_ORTHOGONALITY_MAX: f64 = 1e-8;
pub const MAC_MIN: f64 = 0.90;
pub const MAC_MARGIN_MIN: f64 = 0.05;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DenseMatrix {
    pub rows: u32,
    pub cols: u32,
    pub values: Vec<f64>,
}

impl DenseMatrix {
    pub fn new(rows: usize, cols: usize, values: Vec<f64>) -> Result<Self> {
        let expected = rows.checked_mul(cols).ok_or_else(|| Error::Invalid("ExternalStaticBase: matrix size overflow".into()))?;
        if rows == 0 || cols == 0 || rows > MAX_DOF || cols > MAX_DOF || expected > MAX_MATRIX_ENTRIES || values.len() != expected {
            return invalid("ExternalStaticBase: matrix shape or bound");
        }
        if values.iter().any(|v| !v.is_finite()) {
            return invalid("ExternalStaticBase: matrix has non-finite value");
        }
        Ok(Self { rows: rows as u32, cols: cols as u32, values })
    }

    pub fn square(values: Vec<f64>, n: usize) -> Result<Self> {
        Self::new(n, n, values)
    }

    pub fn shape(&self) -> Result<(usize, usize)> {
        let rows = usize::try_from(self.rows).map_err(|_| Error::Invalid("ExternalStaticBase: matrix rows".into()))?;
        let cols = usize::try_from(self.cols).map_err(|_| Error::Invalid("ExternalStaticBase: matrix cols".into()))?;
        let expected = rows.checked_mul(cols).ok_or_else(|| Error::Invalid("ExternalStaticBase: matrix size overflow".into()))?;
        if rows == 0 || cols == 0 || rows > MAX_DOF || cols > MAX_DOF || expected > MAX_MATRIX_ENTRIES || self.values.len() != expected {
            return invalid("ExternalStaticBase: matrix shape or bound");
        }
        if self.values.iter().any(|v| !v.is_finite()) {
            return invalid("ExternalStaticBase: matrix has non-finite value");
        }
        Ok((rows, cols))
    }

    fn at(&self, row: usize, col: usize) -> f64 {
        self.values[row * self.cols as usize + col]
    }

    fn frobenius(&self) -> f64 {
        self.values.iter().map(|v| v * v).sum::<f64>().sqrt()
    }

    pub fn relative_difference(&self, other: &Self) -> Result<f64> {
        let (rows, cols) = self.shape()?;
        let (other_rows, other_cols) = other.shape()?;
        if (rows, cols) != (other_rows, other_cols) {
            return invalid("ExternalStaticBase: matrix shape mismatch");
        }
        let norm = self.values.iter().zip(&other.values).map(|(a, b)| (a - b) * (a - b)).sum::<f64>().sqrt();
        Ok(norm / self.frobenius().max(other.frobenius()).max(1e-30))
    }

    pub fn symmetry_error(&self) -> Result<f64> {
        let (rows, cols) = self.shape()?;
        if rows != cols {
            return invalid("ExternalStaticBase: matrix must be square");
        }
        let mut sum = 0.0;
        for i in 0..rows {
            for j in 0..cols {
                let d = self.at(i, j) - self.at(j, i);
                sum += d * d;
            }
        }
        Ok(sum.sqrt() / self.frobenius().max(1e-30))
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HoldWindow {
    pub no_reset_steps: u32,
    pub converged: bool,
    pub solver_cap_hit: bool,
    pub free_residual_norm: f64,
    pub max_velocity: f64,
    pub single_step_drift: f64,
    pub inertia_internal_difference: f64,
    pub support_error: f64,
}

impl HoldWindow {
    fn validate(&self) -> Result<()> {
        if self.no_reset_steps < 10 || self.no_reset_steps > MAX_HOLD_STEPS {
            return invalid("ExternalStaticBase: static hold window");
        }
        if !self.converged || self.solver_cap_hit {
            return invalid("ExternalStaticBase: static hold did not converge");
        }
        if !self.free_residual_norm.is_finite()
            || self.free_residual_norm >= STATIC_RESIDUAL_MAX
            || !self.max_velocity.is_finite()
            || self.max_velocity >= STATIC_VELOCITY_MAX
            || !self.single_step_drift.is_finite()
            || self.single_step_drift >= STATIC_DRIFT_MAX
            || !self.inertia_internal_difference.is_finite()
            || self.inertia_internal_difference >= STATIC_INERTIA_MAX
            || !self.support_error.is_finite()
            || self.support_error >= STATIC_SUPPORT_MAX
        {
            return invalid("ExternalStaticBase: static hold threshold");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StaticBaseState {
    pub reference_coordinates: Vec<f64>,
    pub coordinates: Vec<f64>,
    pub velocities: Vec<f64>,
    pub units: String,
    pub hold: HoldWindow,
}

impl StaticBaseState {
    fn validate(&self) -> Result<()> {
        if self.units != "SI"
            || self.reference_coordinates.is_empty()
            || self.reference_coordinates.len() != self.coordinates.len()
            || self.coordinates.len() != self.velocities.len()
            || self.coordinates.len() > MAX_DOF
            || self.reference_coordinates.iter().chain(&self.coordinates).chain(&self.velocities).any(|v| !v.is_finite())
        {
            return invalid("ExternalStaticBase: static state shape or units");
        }
        self.hold.validate()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactState {
    pub policy: String,
    pub policy_digest: String,
    pub active: Vec<bool>,
    pub gap_normal: Vec<f64>,
    pub tangential_increment: Vec<f64>,
    pub delta_q_max: Option<f64>,
    pub slip_ratio: Option<f64>,
    pub friction_coefficient: Option<f64>,
    pub normal_force: Vec<f64>,
    pub tangential_force: Vec<f64>,
    pub friction_utilization: Option<f64>,
    pub state_changes: u32,
}

impl ContactState {
    fn validate(&self) -> Result<()> {
        if self.policy != "closed_bonded" && self.policy != "closed_sticking" {
            return invalid("contact_state_unknown: unsupported policy");
        }
        if !identity::digest_valid(&self.policy_digest)
            || self.active.is_empty()
            || self.active.len() != self.gap_normal.len()
            || self.active.len() != self.tangential_increment.len()
            || self.active.len() > MAX_DOF
            || self.gap_normal.iter().chain(&self.tangential_increment).any(|v| !v.is_finite())
        {
            return invalid("contact_state_unknown: missing or malformed pair state");
        }
        if self.state_changes != 0 {
            return invalid("contact_open: contact state changed during hold");
        }
        if self.active.iter().any(|active| !*active) || self.gap_normal.iter().any(|gap| *gap > GAP_MAX) {
            return invalid("contact_open: gap or active mask");
        }
        if self.policy == "closed_bonded" {
            if self.tangential_increment.iter().any(|slip| *slip > BONDED_TANGENTIAL_MAX) {
                return invalid("contact_sliding: bonded tangential increment");
            }
            return Ok(());
        }
        let (Some(delta), Some(slip), Some(mu), Some(utilization)) =
            (self.delta_q_max, self.slip_ratio, self.friction_coefficient, self.friction_utilization)
        else {
            return invalid("contact_state_unknown: sticking metrics unavailable");
        };
        if !delta.is_finite()
            || delta <= 0.0
            || !slip.is_finite()
            || !utilization.is_finite()
            || !mu.is_finite()
            || mu <= 0.0
            || self.normal_force.len() != self.active.len()
            || self.tangential_force.len() != self.active.len()
            || self.normal_force.iter().chain(&self.tangential_force).any(|v| !v.is_finite())
            || self.normal_force.iter().any(|n| *n <= 0.0)
        {
            return invalid("contact_state_unknown: sticking metrics malformed");
        }
        if slip > STICK_SLIP_RATIO_MAX || utilization >= STICK_FRICTION_UTILIZATION_MAX {
            return invalid("contact_sliding: sticking slip or friction limit");
        }
        let derived = self.normal_force.iter().zip(&self.tangential_force).map(|(normal, tangent)| tangent.abs() / (mu * normal)).fold(0.0, f64::max);
        if (derived - utilization).abs() > 1e-12 {
            return invalid("contact_state_unknown: friction utilization mismatch");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StiffnessField {
    pub matrix: DenseMatrix,
    pub source_backend: String,
    pub probe_error: f64,
    pub symmetry_error: f64,
    pub projected: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MassField {
    pub matrix: DenseMatrix,
    pub kind: String,
    pub source_backend: String,
    pub declared_total: f64,
    pub symmetry_error: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConstraintMapping {
    pub full_dofs: u32,
    pub reduced_dofs: u32,
    pub q0: Vec<f64>,
    pub full_coordinates: Vec<f64>,
    pub reduced_coordinates: Vec<f64>,
    pub t: DenseMatrix,
    pub reduced_stiffness: DenseMatrix,
    pub reduced_mass: DenseMatrix,
    pub constraint_residual: f64,
    pub projection_error: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModalMode {
    pub eigenvalue: f64,
    pub frequency_hz: f64,
    pub shape: Vec<f64>,
    pub residual: f64,
    pub mass_norm: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModalPair {
    pub mode: u32,
    pub reference: u32,
    pub mac: f64,
    pub margin: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ModalPackage {
    pub source_backend: String,
    pub modes: Vec<ModalMode>,
    pub mass_gram: DenseMatrix,
    pub reference_shapes: Vec<Vec<f64>>,
    pub mac: DenseMatrix,
    pub pairs: Vec<ModalPair>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExternalStaticBase {
    pub name: String,
    pub state: StaticBaseState,
    pub contact: ContactState,
    pub stiffness: StiffnessField,
    pub mass: MassField,
    pub constraints: ConstraintMapping,
    pub source_backend: String,
    pub runtime_qualification: String,
    pub frequency_response: String,
    pub modal: Option<ModalPackage>,
}

impl ExternalStaticBase {
    pub fn validate(&self) -> Result<()> {
        if self.name != STATIC_BASE_NAME {
            return invalid("ExternalStaticBase: name");
        }
        if self.source_backend.is_empty() || (self.runtime_qualification != "unverified" && self.runtime_qualification != "verified") {
            return invalid("ExternalStaticBase: source or runtime qualification");
        }
        if self.frequency_response != "not_run" && self.frequency_response != "eligible" {
            return invalid("ExternalStaticBase: frequency response state");
        }
        self.state.validate()?;
        self.contact.validate()?;
        let (n, n2) = self.stiffness.matrix.shape()?;
        if n != n2 || n != self.state.coordinates.len() {
            return invalid("ExternalStaticBase: stiffness/state shape");
        }
        if self.stiffness.source_backend.is_empty()
            || self.stiffness.projected
            || !self.stiffness.probe_error.is_finite()
            || self.stiffness.probe_error > TANGENT_ERROR_MAX
            || !self.stiffness.symmetry_error.is_finite()
            || self.stiffness.symmetry_error > TANGENT_SYMMETRY_MAX
            || self.stiffness.matrix.symmetry_error()? > TANGENT_SYMMETRY_MAX
        {
            return invalid("ExternalStaticBase: stiffness tangent check");
        }
        let (mass_rows, mass_cols) = self.mass.matrix.shape()?;
        if (mass_rows, mass_cols) != (n, n)
            || self.mass.kind != "consistent" && self.mass.kind != "lumped"
            || self.mass.source_backend.is_empty()
            || !self.mass.declared_total.is_finite()
            || self.mass.declared_total <= 0.0
            || !self.mass.symmetry_error.is_finite()
            || self.mass.symmetry_error > MASS_SYMMETRY_MAX
            || self.mass.matrix.symmetry_error()? > MASS_SYMMETRY_MAX
        {
            return invalid("ExternalStaticBase: mass check");
        }
        let mass_total = self.mass.matrix.values.iter().sum::<f64>();
        if (mass_total - self.mass.declared_total).abs() > 1e-12_f64.max(1e-9 * self.mass.declared_total) {
            return invalid("ExternalStaticBase: mass total");
        }
        let (full, reduced) = self.validate_mapping(n)?;
        if full != n || reduced == 0 {
            return invalid("ExternalStaticBase: mapping dimensions");
        }
        cholesky(&self.constraints.reduced_mass, reduced)?;
        if self.frequency_response == "eligible" {
            return invalid("ExternalStaticBase: damping source absent for eligible response");
        }
        if let Some(modal) = &self.modal {
            self.validate_modal(modal)?;
        }
        Ok(())
    }

    fn validate_mapping(&self, n: usize) -> Result<(usize, usize)> {
        let full = usize::try_from(self.constraints.full_dofs).map_err(|_| Error::Invalid("ExternalStaticBase: full dofs".into()))?;
        let reduced = usize::try_from(self.constraints.reduced_dofs).map_err(|_| Error::Invalid("ExternalStaticBase: reduced dofs".into()))?;
        if full == 0
            || reduced == 0
            || full > MAX_DOF
            || reduced > full
            || full != n
            || self.constraints.q0.len() != full
            || self.constraints.full_coordinates.len() != full
            || self.constraints.reduced_coordinates.len() != reduced
            || self.constraints.t.shape()? != (full, reduced)
            || self.constraints.reduced_stiffness.shape()? != (reduced, reduced)
            || self.constraints.reduced_mass.shape()? != (reduced, reduced)
        {
            return invalid("ExternalStaticBase: mapping shape");
        }
        if self.constraints.q0.iter().chain(&self.constraints.full_coordinates).chain(&self.constraints.reduced_coordinates).any(|v| !v.is_finite()) {
            return invalid("ExternalStaticBase: mapping coordinate");
        }
        let mut residual: f64 = 0.0;
        let scale = self.constraints.full_coordinates.iter().chain(&self.constraints.q0).map(|v| v.abs()).fold(0.0, f64::max).max(1.0);
        for i in 0..full {
            let projected =
                self.constraints.q0[i] + (0..reduced).map(|j| self.constraints.t.at(i, j) * self.constraints.reduced_coordinates[j]).sum::<f64>();
            residual = residual.max((projected - self.constraints.full_coordinates[i]).abs());
        }
        if !self.constraints.constraint_residual.is_finite()
            || self.constraints.constraint_residual > (1e-12_f64).max(1e-9 * scale)
            || residual > (1e-12_f64).max(1e-9 * scale)
        {
            return invalid("ExternalStaticBase: constraint residual");
        }
        if !full_column_rank(&self.constraints.t, full, reduced)? {
            return invalid("ExternalStaticBase: constraint map rank");
        }
        let k_projected = project(&self.stiffness.matrix, &self.constraints.t)?;
        let m_projected = project(&self.mass.matrix, &self.constraints.t)?;
        if k_projected.relative_difference(&self.constraints.reduced_stiffness)? > PROJECTION_ERROR_MAX
            || m_projected.relative_difference(&self.constraints.reduced_mass)? > PROJECTION_ERROR_MAX
            || !self.constraints.projection_error.is_finite()
            || self.constraints.projection_error > PROJECTION_ERROR_MAX
        {
            return invalid("ExternalStaticBase: projection error");
        }
        Ok((full, reduced))
    }

    pub fn compute_modal(&self, reference_shapes: Vec<Vec<f64>>) -> Result<ModalPackage> {
        self.compute_modal_with_count(reference_shapes, self.constraints.reduced_dofs as usize)
    }

    /// Recompute a bounded leading modal package from the accepted K/M mapping.
    ///
    /// The full generalized eigenproblem is still solved, but only the requested
    /// leading modes are published and paired.  This lets a product receiver
    /// retain a reviewable first-N package for a dense small model without
    /// treating producer-provided modes or residuals as evidence.
    pub fn compute_modal_with_count(&self, reference_shapes: Vec<Vec<f64>>, requested_modes: usize) -> Result<ModalPackage> {
        self.validate_without_modal()?;
        let n = self.constraints.reduced_stiffness.rows as usize;
        let mode_count = requested_modes.min(n);
        if mode_count == 0 {
            return invalid("ExternalStaticBase: modal mode count");
        }
        let l = cholesky(&self.constraints.reduced_mass, n)?;
        let inv_l = invert_lower(&l, n)?;
        let inv_lt = transpose(&inv_l, n, n);
        let a = multiply(&multiply(&inv_l, n, n, &self.constraints.reduced_stiffness.values, n, n)?, n, n, &inv_lt, n, n)?;
        let (values, vectors) =
            fieldcraft_solver::eigen::symmetric_eigen(&a, n).map_err(|e| Error::Invalid(format!("ExternalStaticBase: modal eigenproblem: {e}")))?;
        if values.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return invalid("ExternalStaticBase: non-positive modal eigenvalue");
        }
        let mut modes = Vec::with_capacity(mode_count);
        for (mode, value) in values.iter().take(mode_count).enumerate() {
            let mut shape = vec![0.0; n];
            for i in 0..n {
                shape[i] = (0..n).map(|j| inv_lt[i * n + j] * vectors[j * n + mode]).sum();
            }
            let mass_shape = mat_vec(&self.constraints.reduced_mass, &shape)?;
            let norm = dot(&shape, &mass_shape).sqrt();
            if !(norm > 0.0 && norm.is_finite()) {
                return invalid("ExternalStaticBase: modal mass normalization");
            }
            for x in &mut shape {
                *x /= norm;
            }
            let k_shape = mat_vec(&self.constraints.reduced_stiffness, &shape)?;
            let m_shape = mat_vec(&self.constraints.reduced_mass, &shape)?;
            let numerator = k_shape.iter().zip(&m_shape).map(|(k, m)| (k - value * m).powi(2)).sum::<f64>().sqrt();
            let denominator =
                k_shape.iter().map(|x| x * x).sum::<f64>().sqrt() + m_shape.iter().map(|x| value * x).map(|x| x * x).sum::<f64>().sqrt();
            modes.push(ModalMode {
                eigenvalue: *value,
                frequency_hz: value.sqrt() / (2.0 * std::f64::consts::PI),
                shape,
                residual: numerator / denominator.max(1e-30),
                mass_norm: 1.0,
            });
        }
        let mass_gram = gram(&modes, &self.constraints.reduced_mass)?;
        let (reference_shapes, mac, pairs) = pair_modes(&modes, &reference_shapes, &self.constraints.reduced_mass)?;
        Ok(ModalPackage { source_backend: self.source_backend.clone(), modes, mass_gram, reference_shapes, mac, pairs })
    }

    fn validate_without_modal(&self) -> Result<()> {
        let mut copy = self.clone();
        copy.modal = None;
        copy.validate()
    }

    fn validate_modal(&self, modal: &ModalPackage) -> Result<()> {
        let n = self.constraints.reduced_stiffness.rows as usize;
        if modal.source_backend != self.source_backend || modal.modes.is_empty() || modal.modes.len() > n {
            return invalid("ExternalStaticBase: modal source or count");
        }
        for mode in &modal.modes {
            if mode.shape.len() != n
                || !mode.eigenvalue.is_finite()
                || mode.eigenvalue <= 0.0
                || !mode.frequency_hz.is_finite()
                || mode.frequency_hz <= 0.0
                || (mode.frequency_hz - mode.eigenvalue.sqrt() / (2.0 * std::f64::consts::PI)).abs() > 1e-10
                || !mode.residual.is_finite()
                || mode.residual > MODAL_RESIDUAL_MAX
                || !mode.mass_norm.is_finite()
                || (mode.mass_norm - 1.0).abs() > MASS_ORTHOGONALITY_MAX
                || mode.shape.iter().any(|v| !v.is_finite())
            {
                return invalid("ExternalStaticBase: modal residual or eigenvalue");
            }
        }
        if modal.mass_gram.shape()? != (modal.modes.len(), modal.modes.len()) {
            return invalid("ExternalStaticBase: modal Gram shape");
        }
        let expected_gram = gram(&modal.modes, &self.constraints.reduced_mass)?;
        if expected_gram.relative_difference(&modal.mass_gram)? > MASS_ORTHOGONALITY_MAX
            || (0..modal.modes.len()).any(|i| {
                (0..modal.modes.len()).any(|j| {
                    let value = modal.mass_gram.at(i, j);
                    if i == j { (value - 1.0).abs() > MASS_ORTHOGONALITY_MAX } else { value.abs() > MASS_ORTHOGONALITY_MAX }
                })
            })
        {
            return invalid("ExternalStaticBase: modal mass orthogonality");
        }
        if modal.reference_shapes.is_empty() {
            if modal.mac.shape()? != (modal.modes.len(), 0) {
                return invalid("ExternalStaticBase: modal reference shape absence");
            }
            return Ok(());
        }
        if modal.reference_shapes.iter().any(|shape| shape.len() != n || shape.iter().any(|v| !v.is_finite()))
            || modal.mac.shape()? != (modal.modes.len(), modal.reference_shapes.len())
            || modal.pairs.len() != modal.modes.len()
        {
            return invalid("ExternalStaticBase: modal pairing shape");
        }
        let (_, expected_mac, expected_pairs) = pair_modes(&modal.modes, &modal.reference_shapes, &self.constraints.reduced_mass)?;
        if expected_mac.relative_difference(&modal.mac)? > MASS_ORTHOGONALITY_MAX {
            return invalid("ExternalStaticBase: MAC mismatch");
        }
        for pair in &expected_pairs {
            if pair.mac < MAC_MIN || pair.margin < MAC_MARGIN_MIN {
                return invalid("ExternalStaticBase: MAC pairing threshold");
            }
        }
        if expected_pairs.iter().zip(&modal.pairs).any(|(expected, actual)| expected != actual) {
            return invalid("ExternalStaticBase: MAC pairing record mismatch");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ArchivedStaticBaseParts {
    pub archive_key: String,
    pub binding: identity::Binding,
    pub entity_map: identity::EntityMap,
    pub result_id: String,
    pub payload_sha256: String,
    pub external_static_base: ExternalStaticBase,
    pub admission_json: Vec<u8>,
    pub context_json: Vec<u8>,
    pub source_json: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ArchivedStaticBase {
    pub schema: String,
    pub archive_key: String,
    pub binding: identity::Binding,
    pub entity_map: identity::EntityMap,
    pub result_id: String,
    pub payload_sha256: String,
    pub external_static_base: ExternalStaticBase,
    pub admission_json: Vec<u8>,
    pub context_json: Vec<u8>,
    pub source_json: Vec<u8>,
    pub record_sha256: String,
}

impl ArchivedStaticBase {
    pub fn from_payload(
        archive_key: String,
        binding: identity::Binding,
        entity_map: identity::EntityMap,
        external_static_base: ExternalStaticBase,
        admission_json: Vec<u8>,
        context_json: Vec<u8>,
        source_json: Vec<u8>,
    ) -> Result<Self> {
        let payload_sha256 = payload_digest(&external_static_base)?;
        let result_id = format!("fieldcraft-result/{payload_sha256}");
        Self::new(ArchivedStaticBaseParts {
            archive_key,
            binding,
            entity_map,
            result_id,
            payload_sha256,
            external_static_base,
            admission_json,
            context_json,
            source_json,
        })
    }

    pub fn new(parts: ArchivedStaticBaseParts) -> Result<Self> {
        let mut value = Self {
            schema: STATIC_BASE_SCHEMA.into(),
            archive_key: parts.archive_key,
            binding: parts.binding,
            entity_map: parts.entity_map,
            result_id: parts.result_id,
            payload_sha256: parts.payload_sha256,
            external_static_base: parts.external_static_base,
            admission_json: parts.admission_json,
            context_json: parts.context_json,
            source_json: parts.source_json,
            record_sha256: String::new(),
        };
        value.check_parts()?;
        value.record_sha256 = value.record_digest()?;
        value.validate()?;
        Ok(value)
    }

    fn check_parts(&self) -> Result<()> {
        if self.schema != STATIC_BASE_SCHEMA || self.archive_key.trim().is_empty() || self.archive_key.len() > 1024 {
            return invalid("ExternalStaticBase: schema or archive key");
        }
        self.external_static_base.validate()?;
        for bytes in [&self.admission_json, &self.context_json, &self.source_json] {
            if bytes.len() > 16 * 1024 * 1024 {
                return invalid("ExternalStaticBase: JSON limit");
            }
            let mut d = serde_json::Deserializer::from_slice(bytes);
            serde::de::IgnoredAny::deserialize(&mut d).map_err(|e| Error::Invalid(format!("ExternalStaticBase: JSON: {e}")))?;
            d.end().map_err(|e| Error::Invalid(format!("ExternalStaticBase: JSON: {e}")))?;
        }
        if self.binding.snapshot.scheme != identity::SCHEME || self.binding.backend_version != identity::BACKEND_VERSION {
            return invalid("ExternalStaticBase: identity scheme or backend");
        }
        if !identity::digest_valid(&self.binding.snapshot.model_revision)
            || !identity::digest_valid(&self.binding.snapshot.reference_mesh_digest)
            || !identity::digest_valid(&self.binding.entity_map_digest)
            || !identity::digest_valid(&self.payload_sha256)
            || self.binding.snapshot.model_id != identity::model_id(&self.binding.snapshot.reference_mesh_digest)
            || identity::REQUIRED_ARTIFACT_ROLES.iter().any(|k| !self.binding.artifacts.contains_key(*k))
            || self.binding.artifacts.values().any(|v| !identity::digest_valid(v))
        {
            return invalid("ExternalStaticBase: identity digest");
        }
        if payload_digest(&self.external_static_base)? != self.payload_sha256 {
            return invalid("ExternalStaticBase: payload digest");
        }
        if self.result_id != format!("fieldcraft-result/{}", self.payload_sha256) {
            return invalid("ExternalStaticBase: result id");
        }
        Ok(())
    }

    pub fn record_digest(&self) -> Result<String> {
        self.check_parts()?;
        let bytes = codec::encode(&(
            &self.schema,
            &self.archive_key,
            &self.binding,
            &self.entity_map,
            &self.result_id,
            &self.payload_sha256,
            &self.external_static_base,
            &self.admission_json,
            &self.context_json,
            &self.source_json,
        ))
        .map_err(|e| Error::Invalid(e.to_string()))?;
        Ok(sha256_hex(&bytes))
    }

    pub fn validate(&self) -> Result<()> {
        self.check_parts()?;
        if !identity::digest_valid(&self.record_sha256) || self.record_digest()? != self.record_sha256 {
            return invalid("ExternalStaticBase: record digest");
        }
        Ok(())
    }

    pub fn archive_key(&self) -> &str {
        &self.archive_key
    }
    pub fn binding(&self) -> &identity::Binding {
        &self.binding
    }
    pub fn entity_map(&self) -> &identity::EntityMap {
        &self.entity_map
    }
    pub fn result_id(&self) -> &str {
        &self.result_id
    }
    pub fn payload_sha256(&self) -> &str {
        &self.payload_sha256
    }
    pub fn external_static_base(&self) -> &ExternalStaticBase {
        &self.external_static_base
    }
    pub fn admission_json(&self) -> &[u8] {
        &self.admission_json
    }
    pub fn context_json(&self) -> &[u8] {
        &self.context_json
    }
    pub fn source_json(&self) -> &[u8] {
        &self.source_json
    }

    pub fn definition(&self) -> Result<[u8; 32]> {
        self.validate()?;
        let mut out = [0; 32];
        for (dst, pair) in out.iter_mut().zip(self.binding.snapshot.model_revision.as_bytes().as_chunks::<2>().0) {
            let text = std::str::from_utf8(pair).map_err(|e| Error::Invalid(e.to_string()))?;
            *dst = u8::from_str_radix(text, 16).map_err(|e| Error::Invalid(e.to_string()))?;
        }
        Ok(out)
    }

    pub fn validate_binding(&self, model: &Model, geometry: &Geometry, run: &Run) -> Result<()> {
        self.validate()?;
        self.binding.validate(model, geometry, run, &self.entity_map, self.binding.artifacts.clone(), &self.binding.backend_version)
    }
}

pub fn payload_digest(base: &ExternalStaticBase) -> Result<String> {
    let bytes = codec::encode(base).map_err(|e| Error::Invalid(e.to_string()))?;
    Ok(sha256_hex(&bytes))
}

fn project(a: &DenseMatrix, t: &DenseMatrix) -> Result<DenseMatrix> {
    let (n, n2) = a.shape()?;
    let (full, reduced) = t.shape()?;
    if n != n2 || n != full {
        return invalid("ExternalStaticBase: projection shape");
    }
    let mut values = vec![0.0; reduced * reduced];
    for i in 0..reduced {
        for j in 0..reduced {
            values[i * reduced + j] = (0..n).map(|r| (0..n).map(|c| t.at(r, i) * a.at(r, c) * t.at(c, j)).sum::<f64>()).sum();
        }
    }
    DenseMatrix::new(reduced, reduced, values)
}

fn full_column_rank(t: &DenseMatrix, full: usize, reduced: usize) -> Result<bool> {
    let mut basis: Vec<Vec<f64>> = Vec::with_capacity(reduced);
    for j in 0..reduced {
        let mut column: Vec<f64> = (0..full).map(|i| t.at(i, j)).collect();
        for prior in &basis {
            let projection = dot(&column, prior);
            for (value, p) in column.iter_mut().zip(prior) {
                *value -= projection * p;
            }
        }
        let norm = dot(&column, &column).sqrt();
        if !norm.is_finite() || norm <= 1e-12 {
            return Ok(false);
        }
        for value in &mut column {
            *value /= norm;
        }
        basis.push(column);
    }
    Ok(true)
}

fn cholesky(matrix: &DenseMatrix, n: usize) -> Result<Vec<f64>> {
    if matrix.shape()? != (n, n) {
        return invalid("ExternalStaticBase: Cholesky shape");
    }
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut value = matrix.at(i, j) - (0..j).map(|k| l[i * n + k] * l[j * n + k]).sum::<f64>();
            if i == j {
                if !(value > 0.0 && value.is_finite()) {
                    return invalid("ExternalStaticBase: mass is not positive definite");
                }
                value = value.sqrt();
            } else {
                value /= l[j * n + j];
            }
            l[i * n + j] = value;
        }
    }
    Ok(l)
}

fn invert_lower(l: &[f64], n: usize) -> Result<Vec<f64>> {
    let mut inverse = vec![0.0; n * n];
    for col in 0..n {
        for i in 0..n {
            let mut value = if i == col { 1.0 } else { 0.0 };
            for k in 0..i {
                value -= l[i * n + k] * inverse[k * n + col];
            }
            if l[i * n + i] == 0.0 {
                return invalid("ExternalStaticBase: singular mass factor");
            }
            inverse[i * n + col] = value / l[i * n + i];
        }
    }
    Ok(inverse)
}

fn transpose(a: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    let mut out = vec![0.0; rows * cols];
    for i in 0..rows {
        for j in 0..cols {
            out[j * rows + i] = a[i * cols + j];
        }
    }
    out
}

fn multiply(a: &[f64], a_rows: usize, a_cols: usize, b: &[f64], b_rows: usize, b_cols: usize) -> Result<Vec<f64>> {
    if a_cols != b_rows || a.len() != a_rows * a_cols || b.len() != b_rows * b_cols {
        return invalid("ExternalStaticBase: dense product shape");
    }
    let mut out = vec![0.0; a_rows * b_cols];
    for i in 0..a_rows {
        for j in 0..b_cols {
            out[i * b_cols + j] = (0..a_cols).map(|k| a[i * a_cols + k] * b[k * b_cols + j]).sum();
        }
    }
    Ok(out)
}

fn mat_vec(a: &DenseMatrix, x: &[f64]) -> Result<Vec<f64>> {
    let (rows, cols) = a.shape()?;
    if cols != x.len() {
        return invalid("ExternalStaticBase: matrix/vector shape");
    }
    Ok((0..rows).map(|i| (0..cols).map(|j| a.at(i, j) * x[j]).sum()).collect())
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn gram(modes: &[ModalMode], mass: &DenseMatrix) -> Result<DenseMatrix> {
    let n = modes.len();
    let mut values = vec![0.0; n * n];
    for i in 0..n {
        let weighted = mat_vec(mass, &modes[i].shape)?;
        for j in 0..n {
            values[i * n + j] = dot(&modes[j].shape, &weighted);
        }
    }
    DenseMatrix::new(n, n, values)
}

fn pair_modes(modes: &[ModalMode], refs: &[Vec<f64>], mass: &DenseMatrix) -> Result<(Vec<Vec<f64>>, DenseMatrix, Vec<ModalPair>)> {
    if refs.is_empty() {
        return Ok((Vec::new(), DenseMatrix { rows: modes.len() as u32, cols: 0, values: Vec::new() }, Vec::new()));
    }
    let n = mass.rows as usize;
    if refs.iter().any(|shape| shape.len() != n) {
        return invalid("ExternalStaticBase: reference shape dimension");
    }
    let mut mac = vec![0.0; modes.len() * refs.len()];
    for (i, mode) in modes.iter().enumerate() {
        let mode_mass = dot(&mode.shape, &mat_vec(mass, &mode.shape)?);
        for (j, reference) in refs.iter().enumerate() {
            let ref_mass = dot(reference, &mat_vec(mass, reference)?);
            if !(ref_mass > 0.0 && ref_mass.is_finite()) {
                return invalid("ExternalStaticBase: reference mode has no mass");
            }
            let cross = dot(&mode.shape, &mat_vec(mass, reference)?);
            mac[i * refs.len() + j] = cross * cross / (mode_mass * ref_mass);
        }
    }
    let mut pairs = Vec::with_capacity(modes.len());
    for i in 0..modes.len() {
        let mut order: Vec<usize> = (0..refs.len()).collect();
        order.sort_by(|a, b| mac[i * refs.len() + *b].total_cmp(&mac[i * refs.len() + *a]).then(a.cmp(b)));
        let best = order[0];
        let second = order.get(1).map_or(0.0, |j| mac[i * refs.len() + *j]);
        pairs.push(ModalPair {
            mode: i as u32,
            reference: best as u32,
            mac: mac[i * refs.len() + best],
            margin: mac[i * refs.len() + best] - second,
        });
    }
    Ok((refs.to_vec(), DenseMatrix::new(modes.len(), refs.len(), mac)?, pairs))
}
