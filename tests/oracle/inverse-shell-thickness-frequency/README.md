# Thin shell thickness identification from frequency

The strip is 2 m long and 1 m wide, with E=1200 Pa, density 1 kg/m³ and a clamped left edge. The oracle model uses 16 quad8 elements along its length. An 8-element copy of the same strip is used only to measure the mesh contribution: its first frequency at t=0.005 m is 0.006994825643627156 Hz, while the 16-element value is 0.006994839209354149 Hz. Their relative difference is 1.939390826041693e-6.

The independent reference is the first Euler–Bernoulli cantilever frequency. With beta1=1.875104068711961, I=b t³/12, A=b t and L=2 m,

`f1=beta1² t sqrt(E/(12 rho))/(2 pi L²)=0.006994890124604707 Hz` at the reference thickness `t=0.005 m`.

The thin-beam discrepancy budget also includes the closed-form first-mode Timoshenko shear and rotary-inertia correction

`fT/fEB = [1 + beta1² EI/(kappa G A L²) + beta1⁴ I/(A L²)]^(-1/2)`,

with `kappa=5/6`, `G=E/[2(1+nu)]`, `nu=0`, and `t/L=0.0025`. This gives a relative correction of 5.416830989868338e-6. Adding it to the measured mesh difference gives 7.356221815910031e-6, rounded upward to the 7.4e-6 relative tolerance used for the estimate and predicted frequency. The confidence-interval endpoints use `df/dt=f/t` and a known 1e-5 Hz observation error; their absolute model-discrepancy allowance is 4e-8 m.

The inverse starts from thickness 0.004 m, away from the reference, so the fit exercises the finite-element evaluations before reaching the self-consistent plateau.
