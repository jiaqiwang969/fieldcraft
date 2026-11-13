# Cook membrane: linear B-bar benchmark

This is the small-strain Cook membrane from the public OpenGeoSys B-bar benchmark. It uses the tapered, swept corners `(0,0)`, `(48,44)`, `(48,60)`, `(0,44)` mm, 1 mm thickness, a clamped left face, zero z displacement on both z faces, and a right-face shear traction whose 100 N resultant is represented as `(0, 6.25e6, 0) Pa` in SI. The public page does not state the material constants; comparing its standard-brick rows identifies `nu=0.4999` (using `nu=0.499` misses by 6–80%), so this recipe records `E=240.565 MPa`, `nu=0.4999`.

The recipe runs fresh linear static models for B-bar meshes with 4, 10 and 30 elements per side, plus standard-brick 4 and 10 element meshes. The B-bar references use the public page's 2e-4 m allowance; the standard-brick references use 1e-4 relative because the same element, mesh, material and loading match the public values to about 1e-5.

Reference: OpenGeoSys, “Cook's membrane example” (Wang, revised 2024-06-11): <https://www.opengeosys.org/stable/docs/benchmarks/small-deformations/cooksmembranebbar/>. The benchmark cites Hughes (1980) and Elguedj et al. (2008) for the B-bar projection.
