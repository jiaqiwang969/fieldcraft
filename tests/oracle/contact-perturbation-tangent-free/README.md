# Closed contact with a tangentially free perturbation

The same two-layer patch is compressed by 0.01 m, with zero friction and the `closed_tangent_free` policy. The normal contact remains closed while tangential motion is free. For `λ = 1 − 0.01`, set `s = 500 λ²`; the uniform layer translations reduce to

`K = [[s, −s], [−s, s + 4]]`, `M = [[2, 1], [1, 2]]/6`.

The factor `500 λ²` again follows from the full-Lagrangian St. Venant–Kirchhoff tangent: `G + S_zz = G λ²` in the shear direction. The two positive roots of `det(K − ω²M) = 0`, divided by `2π`, are the references. This is an independent two-degree-of-freedom closed form.

The run reports a closed-gap-to-thickness ratio of `2.277e-6`; it matches the 2.27e-6 relative difference from the nominal closed-form frequencies. The 5e-6 relative tolerance is this measured closure effect with one additional margin, not an arbitrary widening of the reference.
