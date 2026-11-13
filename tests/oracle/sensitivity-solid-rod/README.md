# Solid rod material sensitivities

A four element unit-area axial solid rod is fixed at x=0 and carries a unit pressure at x=1. Lateral degrees of freedom are guided so the assembled problem is the one-dimensional axial bar. The recipe checks Young modulus and density in static, frequency and one-frequency harmonic analyses.

The independent references are the exact discrete scaling identities u proportional to 1/E, f proportional to sqrt(E/rho), and E du/dE + rho du/drho + u = 0 for fixed loads. These identities hold for every mesh of this guided rod; the 1e-8 and 1e-8 absolute bands only cover solver roundoff.
