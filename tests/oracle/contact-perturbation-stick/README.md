# Closed contact with stick perturbation

Two unit-area elastic layers (E = 1000 Pa, ν = 0, density = 1 kg/m³) are compressed by 0.01 m. The contact pair is selected with the `stick` perturbation policy. The one-dimensional shear mode has tangent `G λ²` with `G = E/2`, four unit lateral springs, and reduced mass `M = 1/3 kg` for the moving layer. The factor `G λ²` comes from the full-Lagrangian St. Venant–Kirchhoff tangent: in the shear direction the material term and the axial initial-stress term combine as `G + S_zz = G λ²`. Thus `f = sqrt(3(500 λ² + 4))/(2π)`, with `λ = 1 − 0.01`. This is a direct closed-form reduction of the stated uniform layer field.

The run reports a closed-gap-to-thickness ratio of `2.278e-6`; it matches the 2.28e-6 relative frequency difference from the nominal closed-form value. The 5e-6 relative tolerance is this measured closure effect with one additional margin, not an arbitrary widening of the reference.
