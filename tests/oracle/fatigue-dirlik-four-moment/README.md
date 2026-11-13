# Dirlik four-moment spectral fatigue

A one-metre unit-area rod is used only to provide a unit axial stress transfer over the 1–5 Hz band; its very large modulus makes the inertia correction less than 10⁻⁸. The independent stress PSD is the rectangular one-sided spectrum `S_sigma(f)=1 Pa²/Hz` for 1≤f≤5 Hz. Its four moments are

`(m0,m1,m2,m4) = (4, 12, 124/3, 3124/5)`.

For Basquin `N=100(1/a)^3`, compute `alpha1=(m1/sqrt(m0))/sqrt(m2)` and `alpha2=sqrt(m2/m0)/sqrt(m4/m2)`. Dirlik's coefficients are

`D1=2(alpha1 alpha2-alpha2²)/(1+alpha2²)`, `R=(alpha2-alpha1 alpha2-D1²)/(1-alpha2-D1+D1²)`, `D2=(1-alpha2-D1+D1²)/(1-R)`, `D3=1-D1-D2`, and `Q=1.25 D1`.

The equivalent form in Dirlik's original notation is `Q=1.25 (alpha2-D3-D2 R)/D1`; since `D1+D2+D3=1` and the definition of `D2` makes `alpha2-D3-D2 R = D1²`, the two expressions are identical.

The independent amplitude-density check uses the normalized amplitude `z=a/sqrt(m0)` and

`p(z)=D1/Q exp(-z/Q) + D2 z/R² exp(-z²/(2R²)) + D3 z exp(-z²/2)`.

The Simpson integral of `z³ p(z)` on `[0,20]` with 200000 equal panels, divided by the narrow-band Rayleigh third moment `alpha2*2^(3/2)*Gamma(2.5)`, gives the Dirlik damage ratio `0.9039446267785842`. The resulting independent damage for 10 s is `8.740438621002562`. The finite integration interval omits only the exponentially small positive tail.
