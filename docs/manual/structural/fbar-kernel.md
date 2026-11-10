# Reference-volume F-bar element kernel

`Solid::fbar_reference(material, displacement, tangent)` evaluates one full-integration
Hex8 with positive neo-Hookean shear and bulk moduli. The section option `formulation.f_bar: true` selects this kernel in
`solve.nonlinear` with nonlinear geometry. Linear `b_bar` retains its existing meaning.
The default is false; explicit false serializes like the prior default.
Only isotropic neo-Hookean full Hex8 without other formulation flags, material
orientation, plasticity or thermal expansion is accepted. Other procedures and faithful
commercial deck export are refused. Preload and base-state integration are not validated yet.
Other formulations, materials and invalid/inverted states return named errors.
No thermal strain, initial stress or constitutive history is accepted by this API.

The reference-volume average is `Jbar = sum(w J) / sum(w)`, with
`Fbar = (Jbar/J)^(1/3) F` at all eight integration points. The potential is
`sum(w W_iso(F)) + V0 U(Jbar)`, since the neo-Hookean isochoric energy is unchanged
by a positive scalar multiple of F. Forces and tangent are exact first and second
variations. In particular, both `U' Jbar_,ij` and `U'' Jbar_,i Jbar_,j` are included.
The returned energy is in joules, force in newtons and tangent in N/m for SI input;
J and mapped deformation gradients are dimensionless.

Source: Foster and Mohammad Nejad (2015), *Engineering* 7:765–788,
[doi:10.4236/eng.2015.711067](https://doi.org/10.4236/eng.2015.711067),
equations (4), (5), (24), (51), (53), (54). This reference-J, modified-virtual-gradient
variant is not asserted to be the centroid formulation of de Souza Neto et al. (1996)
or equivalent to a commercial element. The implementation uses the decoupled energy
identity rather than copying the paper's spatial tangent expansion.

Verification: `cargo test -p fieldcraft-fem fbar -- --nocapture` checks a warped Hex8
against an independently evaluated scalar potential, a 24-column internal-force
finite difference, the zero-state linear B-bar matrix and a 90-degree rigid motion.
These element checks do not establish Cook/beam convergence, global Newton behavior,
prestress/modal compatibility, commercial equivalence or the performance budget.

Global verification: `cargo test -p fieldcraft-solver --test nonlinear fbar`
and `cargo test -p fieldcraft-engine --test fbar`. The latter executes the native
command fixture `crates/engine/tests/fixtures/fbar.json`, saves/reopens results and
checks explicit false against omitted settings. Recovered stress is the effective
Cauchy stress `tau_iso(F)/J + U'(Jbar) I`, with logarithmic strain of physical F;
it is not a claim of Cauchy stress at the projected configuration.
