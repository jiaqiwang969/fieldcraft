//! Reference-volume averaged-J F-bar for a full Hex8 and neo-Hookean elasticity.
//!
//! Foster & Mohammad Nejad (2015), Engineering 7:765–788,
//! doi:10.4236/eng.2015.711067, equations (4),(5),(24),(51),(53),(54).
//! This is not the centroid-J/ordinary-test-gradient variant. With a decoupled
//! neo-Hookean energy, W_iso(sF)=W_iso(F), so the exact potential is
//! sum(w W_iso(F)) + V0 U(Jbar). Its volume derivatives below include both
//! U' Jbar,ij and U'' Jbar,i Jbar,j; no numerical differentiation is used.
use super::*;
use fieldcraft_model::ElementKind;

/// Element-local reference-J F-bar evaluation. No history, thermal or initial stress inputs.
#[derive(Debug, Clone, PartialEq)]
pub struct FbarEvaluation {
    pub response: Finite,
    /// Integral of W(Fbar) over the reference volume, in joules for SI input.
    pub energy: f64,
    pub mean_j: f64,
    /// Modified deformation gradients, in the element's 2×2×2 quadrature order.
    pub mapped_gradients: Vec<Mat3>,
}

impl Solid<'_> {
    /// Evaluate the reference-volume mean-J F-bar kernel, with its exact energy Hessian.
    /// Only full Hex8 and finite positive neo-Hookean shear/bulk moduli are supported.
    /// This explicit kernel API does not change the meaning of the linear B-bar option.
    pub fn fbar_reference(&self, m: &MaterialModel, u: &[[f64; 3]], tangent: bool) -> Result<FbarEvaluation> {
        if self.ty.kind != ElementKind::Hex8 || self.formulation != Formulation::Full {
            return Err(Error::NotSupported("reference-J F-bar requires full Hex8".into()));
        }
        let MaterialModel::NeoHookean { shear, bulk } = m else {
            return Err(Error::NotSupported("reference-J F-bar requires neo-Hookean elasticity without history".into()));
        };
        if !(*shear > 0.0 && shear.is_finite() && *bulk > 0.0 && bulk.is_finite()) {
            return Err(Error::Invalid("F-bar moduli must be finite and positive".into()));
        }
        check_displacements(u, 8)?;
        let points = self.kinematics(u, Kinematics::Finite, None)?;
        let volume: f64 = points.iter().map(|p| p.weight).sum();
        let mean_j = points.iter().map(|p| p.weight * p.j).sum::<f64>() / volume;
        if !(volume > 0.0 && volume.is_finite() && mean_j > 0.0 && mean_j.is_finite()) {
            return Err(Error::Degenerate("F-bar volume and mean J must be finite and positive".into()));
        }
        let mut force = vec![0.0; 24];
        let mut k = tangent.then(|| Matrix::zeros(24));
        let mut dj = [0.0; 24];
        let pressure = bulk * (mean_j - 1.0);
        let mut energy = volume * bulk * (mean_j - 1.0).powi(2) / 2.0;
        let mut mapped_gradients = Vec::with_capacity(8);
        for p in &points {
            let s = (mean_j / p.j).cbrt();
            mapped_gradients.push(p.f.map(|row| row.map(|x| s * x)));
            // Isochoric energy and its derivatives are invariant under the F-bar scaling.
            let r = m.isochoric(&p.f, None, &[], &mut [])?;
            add_point(&mut force, k.as_mut(), &p.gradients, &r, p.weight, true);
            let i1: f64 = p.f.iter().flatten().map(|x| x * x).sum();
            energy += p.weight * shear * (p.j.powf(-2.0 / 3.0) * i1 - 3.0) / 2.0;
            for (a, ga) in p.gradients.iter().enumerate() {
                for i in 0..3 {
                    let d = p.weight * p.j * ga[i];
                    dj[3 * a + i] += d;
                    force[3 * a + i] += pressure * d;
                    if let Some(k) = &mut k {
                        for (b, gb) in p.gradients.iter().enumerate() {
                            for j in 0..3 {
                                // J_,ai,bj = J (g_a,i g_b,j - g_a,j g_b,i).
                                k.add(3 * a + i, 3 * b + j, p.weight * pressure * p.j * (ga[i] * gb[j] - ga[j] * gb[i]));
                            }
                        }
                    }
                }
            }
        }
        if let Some(k) = &mut k {
            for (i, di) in dj.iter().enumerate() {
                for (j, djj) in dj.iter().enumerate() {
                    k.add(i, j, bulk / volume * di * djj);
                }
            }
        }
        if !energy.is_finite() || force.iter().any(|x| !x.is_finite()) || k.as_ref().is_some_and(|k| k.data.iter().any(|x| !x.is_finite())) {
            return Err(Error::Invalid("nonfinite reference-J F-bar response".into()));
        }
        Ok(FbarEvaluation { response: Finite { force, tangent: k }, energy, mean_j, mapped_gradients })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::isotropic;

    fn geometry() -> Vec<[f64; 3]> {
        vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [-0.1, 0., 1.], [1., 0., 1.], [1.2, 0.9, 1.1], [0., 1., 1.]]
    }
    fn det(a: Mat3) -> f64 {
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    }
    // Independent scalar potential: reference gradients + explicit projected F, without
    // production force/tangent, isochoric response, or kinematics helpers.
    fn potential(s: &Solid<'_>, u: &[[f64; 3]]) -> f64 {
        let mut points = Vec::new();
        for q in s.integration_points() {
            let g = s.gradients(q.xi).unwrap();
            let f: Mat3 =
                std::array::from_fn(|i| std::array::from_fn(|j| f64::from(i == j) + u.iter().zip(&g.dx).map(|(v, d)| v[i] * d[j]).sum::<f64>()));
            points.push((g.det * q.weight, f, det(f)));
        }
        let v: f64 = points.iter().map(|p| p.0).sum();
        let mean = points.iter().map(|p| p.0 * p.2).sum::<f64>() / v;
        points
            .iter()
            .map(|(w, f, j)| {
                let fb = f.map(|row| row.map(|x| x * (mean / j).cbrt()));
                let jb = det(fb);
                w * (150.0 * (jb.powf(-2.0 / 3.0) * fb.iter().flatten().map(|x| x * x).sum::<f64>() - 3.0) + 2500.0 * (jb - 1.0).powi(2))
            })
            .sum()
    }
    #[test]
    fn fbar_energy_force_tangent_and_mapping() {
        let xyz = geometry();
        let s = Solid::new(ElementKind::Hex8, &xyz, Formulation::Full).unwrap();
        let m = MaterialModel::NeoHookean { shear: 300., bulk: 5000. };
        let u: Vec<[f64; 3]> =
            xyz.iter().map(|p| [0.12 * p[0] + 0.08 * p[1] * p[2], -0.04 * p[1] + 0.03 * p[0] * p[2], 0.06 * p[2] + 0.02 * p[0] * p[1]]).collect();
        let r = s.fbar_reference(&m, &u, true).unwrap();
        let k = r.response.tangent.as_ref().unwrap();
        let mapping = r.mapped_gradients.iter().map(|f| (det(*f) - r.mean_j).abs()).fold(0.0, f64::max);
        let energy = (r.energy - potential(&s, &u)).abs();
        assert!(mapping < 2e-14 && energy < 1e-10);
        for h in [1e-4, 1e-5, 1e-6] {
            let mut ef: f64 = 0.;
            let mut ek: f64 = 0.;
            for col in 0..24 {
                let mut up = u.clone();
                let mut um = u.clone();
                up[col / 3][col % 3] += h;
                um[col / 3][col % 3] -= h;
                let fp = s.fbar_reference(&m, &up, false).unwrap();
                let fm = s.fbar_reference(&m, &um, false).unwrap();
                ef = ef.max(((potential(&s, &up) - potential(&s, &um)) / (2. * h) - r.response.force[col]).abs());
                for row in 0..24 {
                    ek = ek.max(((fp.response.force[row] - fm.response.force[row]) / (2. * h) - k.get(row, col)).abs());
                }
            }
            println!("Fbar warped Hex8 h={h:e} mapping_abs={mapping:e} energy_abs_J={energy:e} force_FD_abs_N={ef:e} tangent_FD_abs_N_per_m={ek:e}");
            assert!(ef < 2e-4 && ek < 2e-4);
        }
    }
    #[test]
    fn fbar_zero_state_matches_bbar_and_rigid_rotation() {
        let xyz = geometry();
        let s = Solid::new(ElementKind::Hex8, &xyz, Formulation::Full).unwrap();
        // E=900, nu=0.25 -> mu=360, bulk=600 (test-only analytic constants).
        let m = MaterialModel::NeoHookean { shear: 360., bulk: 600. };
        let r = s.fbar_reference(&m, &[[0.; 3]; 8], true).unwrap();
        let bb = Solid::new(ElementKind::Hex8, &xyz, Formulation::BBar).unwrap().stiffness(&isotropic(900., 0.25).unwrap()).unwrap();
        let k = r.response.tangent.unwrap();
        let err = k.data.iter().zip(&bb.data).map(|(a, b)| (a - b).abs()).fold(0., f64::max);
        let u: Vec<_> = xyz.iter().map(|p| [-p[1] - p[0] + 0.3, p[0] - p[1] - 0.2, 0.1]).collect();
        let rigid = s.fbar_reference(&m, &u, true).unwrap();
        let force = rigid.response.force.iter().map(|f| f.abs()).fold(0., f64::max);
        println!("Fbar zero/bbar max_abs_N_per_m={err:e}; rigid_90deg force_N={force:e} energy_J={:e}", rigid.energy);
        assert!(err < 1e-10 && force < 1e-10 && rigid.energy.abs() < 1e-10);
    }
    #[test]
    fn fbar_refuses_unsupported_and_invalid_inputs() {
        let xyz = geometry();
        let s = Solid::new(ElementKind::Hex8, &xyz, Formulation::Full).unwrap();
        let m = MaterialModel::NeoHookean { shear: 300., bulk: 5000. };
        assert!(s.fbar_reference(&m, &[], true).is_err());
        let mut u = [[0.; 3]; 8];
        u[0][0] = f64::NAN;
        assert!(s.fbar_reference(&m, &u, true).is_err());
        let inverted: Vec<_> = xyz.iter().map(|p| [-2. * p[0], 0., 0.]).collect();
        assert!(s.fbar_reference(&m, &inverted, true).is_err());
        let stvk = MaterialModel::StVenantKirchhoff(Box::new(isotropic(1000., 0.3).unwrap()));
        assert!(s.fbar_reference(&stvk, &[[0.; 3]; 8], true).is_err());
        let bb = Solid::new(ElementKind::Hex8, &xyz, Formulation::BBar).unwrap();
        assert!(bb.fbar_reference(&m, &[[0.; 3]; 8], true).is_err());
    }
}
