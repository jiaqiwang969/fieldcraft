# NAFEMS 5R beam random response

This route adds the 40-beam-element variant of the NAFEMS 5R random response. It uses the same frequency sweep, modal step and random input as `random-nafems-5r`, with a 1 N/m line load, and checks the published 180.90 mm²/Hz and 42.65 Hz targets at the midspan probe.

The existing `random-nafems-5r` Hex20 solid case remains unchanged. Its approximately −1.68% displacement difference is retained as the documented continuum end-face/support versus beam-theory limitation; it is not used to widen this beam case's tolerances.
