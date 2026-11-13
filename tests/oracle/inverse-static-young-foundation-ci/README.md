# Static identification of Young modulus and foundation stiffness

A unit-length, unit-area axial member has a unit force and a Winkler surface support at x=0. The independent virtual-work solution is `u(0)=F/k` and `u(L)=F/k+FL/(EA)`. The written observations are `u(0)=0.008 m` and `u(1)=0.0088 m`, so direct substitution gives E=1250 Pa and k=125 N/m³. The two-by-two covariance and joint 95% projections use `(J^T W J)^-1`, with both displacement standard deviations 1e-5 m; all substitutions are listed in `expected.json`.
