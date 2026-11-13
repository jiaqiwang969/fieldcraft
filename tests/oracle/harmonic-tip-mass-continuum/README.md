# Harmonic tip-mass continuum reference

This case reuses the independently meshed 40-beam Wang–Meng–Feng tip-mass sweep. Its frozen expected values come from the case-local Euler–Bernoulli boundary-value script `reference_bvp.py`: 0.230891, 0.537825, 0.936603, 1.399176 and 1.898464 mm at the five probe positions, with a 81.48 Hz peak. The 0.5% amplitude and 0.2% frequency bands are declared before replay; no solver result is used to set them.

The measured-amplitude case remains separate and unchanged.
