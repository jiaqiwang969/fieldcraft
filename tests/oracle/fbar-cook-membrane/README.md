# Cook membrane: finite-strain F-bar and standard-brick checks

This case is the tapered, swept Cook membrane from the public OpenGeoSys F-bar benchmark. The four corner points are `(0,0)`, `(48,44)`, `(48,60)`, `(0,44)` mm, represented in SI metres by the `mesh.block` corners; the thickness is 1 mm. The left face is clamped, both z faces have zero z displacement for plane strain, and the right face carries a uniform `(0, 6.25e6, 0) Pa` traction, whose resultant is 100 N. The material is neo-Hookean with `E=240.565 MPa` and `nu=0.499`, matching the input actually used by the public benchmark (the page's K/G prose prints a nearby rounded `nu=0.4999`).

The recipe runs independent fresh models for F-bar meshes with 4, 10 and 30 elements per side, and for standard (F-bar disabled) bricks with 4 and 10 elements per side. The public reference values are copied from the benchmark page's code, not from a local replay. The standard-brick rows are a setup check: they use the same geometry, traction, material and mesh and have a separate relative tolerance.

The final quantity is the nonuniform-state small-load limit. On the 10-per-side mesh, the F-bar response at 0.1 N is multiplied by 1000 and divided by the same mesh's linear B-bar response at 100 N; its reference is 1. This is a consistency check between the two formulations, not an independent public reference.

Reference: OpenGeoSys, “F-bar method for total Lagrangian formulation — Cook's membrane,” revised 2024-07-03: <https://www.opengeosys.org/stable/docs/benchmarks/large-deformations/cooksmembranefbar/>.
