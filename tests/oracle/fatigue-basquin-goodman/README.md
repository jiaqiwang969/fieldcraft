# Equal-amplitude Basquin life and Goodman correction

The stress record is `(2, 10, 2, 10, 2)` Pa at one-second intervals. ASTM E1049 finite-record counting gives four half cycles, or two cycles, each with range 8 Pa, amplitude 4 Pa and mean 6 Pa. The record duration is 4 s.

Use the fully reversed Basquin law `N = 100 (10/a)^3`, with `a` in Pa. Without a mean correction, `N = 1562.5` cycles and the two-cycle Miner damage is `2/1562.5 = 0.00128`; the repeated-record life is `4/0.00128 = 3125 s`.

For the second calculation, use the Goodman relation `a_eq = a/(1 - sigma_m/S_u)` with `S_u = 20 Pa`. Thus `a_eq = 4/(1-6/20) = 40/7 Pa`, `N = 100(10/(40/7))^3 = 535.9375` cycles, damage `2/N = 0.003731778425655976`, and repeated-record life `4/D = 1071.875 s`.
