# ASTM E1049 rainflow and Miner damage

This case uses the nine-point stress sequence `(-2, 1, -3, 5, -1, 3, -4, 4, -2)` Pa. The finite record is not closed periodically. The three-point procedure in ASTM E1049 §5.4.4 gives the following ranges, means and counts, in counting order:

| range (Pa) | mean (Pa) | count | amplitude (Pa) | damage per counted cycle |
|---:|---:|---:|---:|---:|
| 3 | -0.5 | 0.5 | 1.5 | `0.5*(1.5/10)^3/100` |
| 4 | -1.0 | 0.5 | 2 | `0.5*(2/10)^3/100` |
| 4 | 1.0 | 1 | 2 | `(2/10)^3/100` |
| 8 | 1.0 | 0.5 | 4 | `0.5*(4/10)^3/100` |
| 9 | 0.5 | 0.5 | 4.5 | `0.5*(4.5/10)^3/100` |
| 8 | 0.0 | 0.5 | 4 | `0.5*(4/10)^3/100` |
| 6 | 1.0 | 0.5 | 3 | `0.5*(3/10)^3/100` |

The fully reversed Basquin curve is `N = 100 (10/a)^3` cycles, where `a` is the stress amplitude in Pa. The independent hand sum is a cycle count of 4 and damage `0.0013675`.
