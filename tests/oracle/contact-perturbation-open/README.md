# Open contact perturbation

Two unit-area elastic layers are separated by a 0.02 m gap and carry no preload. The `open` perturbation policy removes contact stiffness; the separated state is therefore a real input condition, rather than the compressed state used by the other two contact cases.

For the independent eight-node-brick reduction, the free coordinates are the x displacement of all eight nodes (the two layers' lateral motion) and the z displacement of the four nodes on the lower face. The mass matrix is the consistent brick mass; the tangent is the full-Lagrangian elastic stiffness, with four unit lateral springs on the upper x coordinates. The three shear modes of this 12-coordinate system have squared frequencies `z` equal to the positive roots of

`((z − 15016) z + 27168000) z − 108000000 = 0`.

The fourth reference is the axial mode with `z = 3000`. The cubic roots are the first, second and eighth modes of the complete twelve-mode spectrum; the axial root is the third mode. Thus the four reported frequencies are modes 1, 2, 3 and 8 of the full system, each computed as `sqrt(z)/(2π)`. The matrices and polynomial are written directly from the stated reduced model, so no solver result supplies the reference.
