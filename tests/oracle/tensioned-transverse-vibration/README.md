# Transverse vibration of a tensioned rectangular bar

A rectangular solid bar has length `L = 1 m`, width `B = 0.04 m` in y, thickness `H = 0.02 m` in z, `E = 1e9 Pa`, Poisson ratio zero, and density `rho = 1000 kg/m^3`. A 20-node brick mesh uses `[40, 2, 2]` divisions. The root end is fixed. The tensioned static step prescribes `u_x = 0.005 m` and `u_y = u_z = 0` on the tip; the control step prescribes zero displacement. Thus `A = B H = 8e-4 m^2`, `T = E A (0.005)/L = 4000 N`, and the axial stress is `5e6 Pa`.

The independent reference is the clamped-clamped Euler–Bernoulli beam with axial tension. For each bending inertia `I`, solve by bisection

`2ab(1 − cosh(aL) cos(bL)) + (a^2 − b^2) sinh(aL) sin(bL) = 0`,

where

`a^2, b^2 = (sqrt(T^2 + 4 E I rho A omega^2) ± T)/(2 E I)`.

The weak-axis inertia is `I = B H^3/12 = 2.6666666667e-8 m^4`; the strong-axis inertia is `I = H B^3/12 = 1.0666666667e-7 m^4`. The standard-library `reference.py` performs the bisection. Sorting the first four weak-axis and first three strong-axis roots gives the reported modes:

| state | mode | beam reference (Hz) | bending direction |
|---|---:|---:|---|
| tensioned | 1 | 43.56620368774269 | weak axis, n=1 |
| tensioned | 2 | 56.73351468763029 | strong axis, n=1 |
| tensioned | 3 | 94.37479727189415 | weak axis, n=2 |
| zero-tension control | 1 | 20.55839792172684 | weak axis, n=1 |
| zero-tension control | 2 | 41.11679584345368 | strong axis, n=1 |
| zero-tension control | 3 | 56.67001551552384 | weak axis, n=2 |

The tension raises the first frequency from about 20.5 Hz to 43.5 Hz, a first-order effect. The tolerances reflect the Euler–Bernoulli approximation to a three-dimensional solid: weak-axis first mode 0.5%, weak-axis second mode 1%, and strong-axis first mode 1.5%. As a scale estimate, the Timoshenko shear correction `rho omega^2/(kappa G)` with `kappa = 5/6` and `G = E/2` is `4.0e-5, 1.6e-4, 3.1e-4` for the three control references and `1.8e-4, 3.0e-4, 8.5e-4` for the tensioned references; the measured solid-versus-beam differences are respectively `0.20%, 0.85%, 0.46%` and `0.09%, 0.52%, 0.21%`. The remaining difference is the beam model's omitted shear, rotary-inertia, and three-dimensional effects, rather than mesh refinement (the `[80,4,4]` check changes each frequency by less than 0.02%).
