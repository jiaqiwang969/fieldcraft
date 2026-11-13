# Closed contact with stick perturbation

Two unit-area elastic layers (E = 1000 Pa, ν = 0, density = 1 kg/m³) are compressed by 0.01 m. The contact pair is selected with the `stick` perturbation policy. The one-dimensional shear mode has tangent `G λ²` with `G = E/2`, four unit lateral springs, and reduced mass `M = 1/3 kg` for the moving layer. Thus `f = sqrt(3(500 λ² + 4))/(2π)`, with `λ = 1 − 0.01`. This is a direct closed-form reduction of the stated uniform layer field.


The tolerance includes the small change in stretch caused by the documented numerical contact closure tolerance; it is bounded at 5e-6 relative and is separate from the closed-form reference.
