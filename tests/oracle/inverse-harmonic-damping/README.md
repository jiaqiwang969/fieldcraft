# Complex harmonic identification of loss-factor damping

A unit mass and a grounded spring k=4 N/m are excited by a unit force at omega=1 rad/s. The harmonic step stores the frequency as `1/(2 pi)` Hz, which the solver converts to omega=1 rad/s. The step starts with loss factor 0.2; the fixed complex observations are generated at eta=0.5. The independent scalar impedance is `Z=k−omega²m+i eta k=3+2i`, hence `u=1/Z=3/13−2i/13` m. The derivative `du/deta=−i k/Z²` gives the local 95% interval from two independent 1e-4 m Cartesian errors. No phase wrapping is involved.
