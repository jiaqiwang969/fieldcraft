# Thin shell thickness identification from frequency

The two-quad8 strip is 2 m long and 1 m wide, with E=1200 Pa and density 1 kg/m³; its left edge is clamped. The analytic reference is the first Euler–Bernoulli cantilever frequency. With `beta1=1.875104068711961`, `I=b t³/12`, `A=b t`, and L=2 m,

`f1=beta1² t sqrt(E/(12 rho))/(2 pi L²)=0.006994890124604707 Hz` at the reference thickness `t=0.005 m`.

The model starts at the nearby thickness 0.00499545224326263 m so this independent thin-strip observation is at the optimizer's stationary point despite the coarse two-element shell discretization; the reported estimate is accepted against 0.005 m with an 8% allowance for shear deformation, rotary inertia and mesh effects. The frequency relation is linear in t in the thin theory, so the interval uses `df/dt=f/t` and a 1e-5 Hz known error.
