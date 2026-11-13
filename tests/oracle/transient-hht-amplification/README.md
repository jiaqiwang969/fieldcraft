# HHT-alpha amplification matrix and multi-step damping

Two identical `m=1`, `k=4` oscillators start from the two basis states `(u,v)=(1,0)` and `(0,1)`. HHT uses `alpha=-0.1`, `beta=(1-alpha)^2/4`, `gamma=0.5-alpha`, and `dt=0.1 s`. The one-step rows are the real amplification matrix entries, while the added rows replay 1000 steps and sample steps 250, 500, 750 and 1000.

The HHT state has three quantities `(u,v,a)`, so the amplification matrix is 3x3. Its principal pair is `0.999984171903922 exp(±i 0.1991707793577174)`, giving numerical damping ratio `7.947e-5` and numerical period `3.154672 s`; the exact undamped period is `pi s`, so the numerical period is longer by about 0.4163%. The third root is `-0.000801`. The multi-step references are an independent three-state recursion, not a 2x2 displacement/velocity shortcut.

Reference: Hilber, Hughes and Taylor (1977), “Improved numerical dissipation for time integration algorithms in structural dynamics,” *Earthquake Engineering & Structural Dynamics* 5, 283–292.
