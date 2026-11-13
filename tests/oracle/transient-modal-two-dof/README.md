# Coupled two-degree-of-freedom modal step response

Two unit masses are joined in a chain: a 4 N/m spring connects ground to mass 1 and a second 4 N/m spring connects mass 1 to mass 2. A unit step acts on mass 2, and the modal transient retains both modes. The eigenvalues are `6−sqrt(20)` and `6+sqrt(20)`, so the angular frequencies are 1.2360679775 and 3.2360679775 rad/s. The recipe samples 0.5, 1.0, 1.5 and 2.0 s.

Two references are recorded. The continuous reference is the closed-form two-mode step superposition. The second is the exact response of the average-acceleration time discretisation: in each modal term, replace `omega` by `omega_tilde=(2/dt) atan(omega dt/2)`. The continuous comparison uses the fixed-step phase-error allowance; the scheme-exact comparison uses 1e-12 because it is the same discrete formula evaluated independently.

Reference: Chopra, *Dynamics of Structures*, 5th ed., §9.4, modal superposition of a coupled undamped two-degree-of-freedom step response.
