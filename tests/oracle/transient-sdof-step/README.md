# Undamped single-degree-of-freedom step response

A unit mass and a stiffness of 4 N/m receive a unit step force. The average-acceleration Newmark method uses `dt=0.001 s`; the recipe reports 0.1, 0.5 and 1.0 s. The continuous reference is `u=(F/k)(1-cos(omega t))`, `v=(F/(m omega)) sin(omega t)`, with `omega=2 rad/s`.

The added reference is the exact response of this time discretisation. Its phase is `omega_tilde=(2/dt) atan(omega dt/2)`, so `u=(F/k)(1-cos(omega_tilde t))` and `v=(F/(m omega)) sin(omega_tilde t)`. Here `omega_tilde=1.9999993333337336 rad/s`, about `3.3e-7` below the continuous frequency. The continuous rows use the fixed-step phase-error band; the scheme-exact rows use `1e-12` to expose implementation error.

Reference: N. M. Newmark (1959), “A method of computation for structural dynamics,” *Journal of the Engineering Mechanics Division* 85(EM3), 67–94.
