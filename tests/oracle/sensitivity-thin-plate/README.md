# Cylindrical thin strip sensitivities

This case is a four-by-one Quad4 shell strip, length `L=4`, width `b=1`, and thickness `t=0.02`. The root at `x=0` is clamped and a transverse tip force is applied at `x=4`. Both long edges (`y=0` and `y=1`) impose `u_y=0` and `r_x=0`. These edge conditions suppress width-direction membrane strain and curvature, so the strip is a cylindrical-bending problem with the Kirchhoff plate stiffness

`D = E t^3 / [12 (1-nu^2)]`.

That constraint is deliberate: a free narrow strip would relax by transverse Poisson contraction and approach a beam stiffness, so the plate `D` formula would not be the right independent reference. The material is `E=1000`, `nu=0.3`, density `2`, and the section uses `kappa=5/6` for the shear estimate below.

The independent Kirchhoff logarithmic derivatives are

- `t d(log w)/dt = -3`, `t d(log f)/dt = 1`;
- `nu d(log w)/dnu = -2 nu^2/(1-nu^2) = -0.1978021978021978`;
- `nu d(log f)/dnu = nu^2/(1-nu^2) = 0.0989010989010989`.

The tolerance is derived before looking at the final fixture. For the static tip response, the closed-form Timoshenko correction uses

`w = P [L^3/(3 E I) + L/(kappa G A)]`,

with `G=E/[2(1+nu)]`, `A=b t`, `I=b t^3/12`, and `r=3 E I/(kappa G A L^2)=1.95e-5`. It predicts `-2.9999610008` for the thickness ratio and `-0.1977938408` for the Poisson ratio, relative corrections `1.30e-5` and `4.22e-5`. For the first frequency, with `beta_1=1.875104`,

`eta_s=beta_1^2 E I/(kappa G A L^2)=2.2854e-5`,
`eta_r=beta_1^2 I/(A L^2)=7.3250e-6`.

The corresponding estimates are `1-(eta_s+eta_r)/(1+eta_s+eta_r)=0.9999698218` for the thickness ratio and `0.0988962017` for the Poisson ratio, relative corrections `3.02e-5` and `4.95e-5`. Repeating the recipe with eight elements along the length changes the four ratios by relative `8.47e-6`, `2.16e-7`, `2.76e-5`, and `1.24e-7`. A relative tolerance of `1e-4` is therefore the stated sum bound for mesh and shear/rotary corrections; no broad empirical band is used.
