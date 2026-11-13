# Narrow-band random fatigue

A one-metre unit-area rod is loaded by a unit harmonic axial force. Its modulus is chosen so the rod inertia changes the stress transfer by less than 10⁻⁸ over the 1–3 Hz band; the independent reference therefore uses the exact static stress transfer `sigma/F = 1/A = 1 Pa/N`. The input force PSD is the rectangle `S_F(f)=4 N²/Hz` on 1≤f≤3 Hz, so the stress PSD is the same rectangle.

For a piecewise-linear one-sided spectrum, the moments are `m_j = integral f^j S_sigma(f) df`. Thus `m0=8`, `m2=104/3`, and `m4=968/5` (the other moment is `m1=16`). The Rayleigh narrow-band formula is

`D = T * sqrt(m2/m0) * (sqrt(2 m0)/A_ref)^k * Gamma(1+k/2) / N_ref`.

With `T=10 s`, `A_ref=2 Pa`, `N_ref=100`, and `k=3`, the independent value is `2.2137941502317`; here `Gamma(2.5)=1.329340388179`, so the gamma factor is exercised rather than equal to one. The irregularity diagnostic is `sqrt(m2/m0)/sqrt(m4/m2)=0.8808752638635534`; it is reported but is not used as a pass/fail assumption about the spectrum.
