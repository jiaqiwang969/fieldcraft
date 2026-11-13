# Closed contact with a tangentially free perturbation

The same two-layer patch is compressed by 0.01 m, with zero friction and the `closed_tangent_free` policy. The normal contact remains closed while tangential motion is free. For `λ = 1 − 0.01`, set `s = 500 λ²`; the uniform layer translations reduce to

`K = [[s, −s], [−s, s + 4]]`, `M = [[2, 1], [1, 2]]/6`.

The two positive roots of `det(K − ω²M) = 0`, divided by `2π`, are the references. This is an independent two-degree-of-freedom closed form.


The tolerance includes the small change in stretch caused by the documented numerical contact closure tolerance; it is bounded at 5e-6 relative and is separate from the closed-form reference.
