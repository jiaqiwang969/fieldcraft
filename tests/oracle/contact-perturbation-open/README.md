# Open contact perturbation

Two unit-area elastic layers are separated by a 0.02 m gap and carry no preload. The `open` perturbation policy removes contact stiffness; the separated state is therefore a real input condition, rather than the compressed state used by the other two contact cases.

For the independent reduction, take the upper block's eight nodes (its lower and upper surfaces), not all nodes of both layers. The free lateral coordinates are the x motion of those eight nodes, and the free normal coordinates are the z motion of the four nodes on its lower surface. The mass matrix is the consistent brick mass; the tangent is the full-Lagrangian elastic stiffness, with four unit lateral springs on the upper x coordinates.

A three-coordinate reduction makes the cubic hand-checkable. Let `a` be the lower-surface x displacement, `b` the upper-surface x displacement, and `c` the lower-surface rotation about y. On the unit cube, use `u_x = a(1−z) + b z` and `u_z = c(x−1/2)(1−z)`, with E = 1000, Poisson ratio 0, density 1, and four unit springs on the upper surface. The reduced matrices are

`K = [[500, −500, −250], [−500, 504, 250], [−250, 250, 250]]`,

`M = [[1/3, 1/6, 0], [1/6, 1/3, 0], [0, 0, 1/36]]`.

Expanding `det(K − z M) = 0` gives `z^3 − 15016 z^2 + 27168000 z − 108000000 = 0`; its three positive roots are the three shear-mode squared frequencies. The axial mode is the four lower-surface nodes moving together in z, with `K = E = 1000`, `M = 1/3`, and hence `z = 3000`. The cubic roots are the first, second and eighth modes of the complete twelve-mode spectrum; the axial root is the third mode. Thus the four reported frequencies are modes 1, 2, 3 and 8 of the full system, each computed as `sqrt(z)/(2π)`. The matrices and polynomial are written directly from the stated reduced model, so no solver result supplies the reference.
