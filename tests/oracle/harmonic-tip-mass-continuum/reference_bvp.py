#!/usr/bin/env python3
"""Reference values for the validation case harmonic-tip-mass-continuum.

A cantilever beam carries a rigid tip mass whose centre of gravity is offset along the beam axis;
the clamped end is driven by a harmonic transverse displacement. The steady-state response is
obtained here as an Euler-Bernoulli boundary-value problem, independently of any finite-element
discretisation:

    D w''''(x) = mu w^2 w(x)                                   for 0 < x < L
    w(0) = B,  w'(0) = 0
    D w''(L)  =  w^2 [ M rc w(L) + (J + M rc^2) w'(L) ]         (moment balance of the tip mass)
    D w'''(L) = -w^2 M [ w(L) + rc w'(L) ]                      (force balance of the tip mass)

with D = E I (1 + i beta w), stiffness-proportional damping matched to the damping ratio at the
reference frequency. Beam, tip mass, damping and base amplitude are the published ones of
Wang Hongjin, Meng Qingfeng and Feng Wuwei, Shock and Vibration 2014, 981053, Tables 2-3 and
section 4.2 (https://doi.org/10.1155/2014/981053).

Run it to print the peak frequency of the tip response and the displacement amplitude at the five
probe positions; these are the reference values of the case. Pure standard library.
"""
import cmath
import math

E, rho, L, b, h = 69e9, 2630.0, 0.1204, 0.0102, 0.0049   # Young's modulus, density, length, width, thickness
M, rc, J = 0.03383, 0.00801, 3.33e-6                      # tip mass, offset of its centre of gravity, its inertia
B = 1e-5                                                  # base displacement amplitude
zeta, f_ref = 0.0025, 81.2                                # damping ratio and the frequency it is matched at
beta = 2 * zeta / (2 * math.pi * f_ref)
A, I = b * h, b * h**3 / 12
mu = rho * A


def solve4(m, r):
    n = 4
    a = [row[:] + [r[i]] for i, row in enumerate(m)]
    for c in range(n):
        p = max(range(c, n), key=lambda i: abs(a[i][c]))
        a[c], a[p] = a[p], a[c]
        for i in range(c + 1, n):
            f = a[i][c] / a[c][c]
            for j in range(c, n + 1):
                a[i][j] -= f * a[c][j]
    x = [0j] * n
    for i in reversed(range(n)):
        x[i] = (a[i][n] - sum(a[i][j] * x[j] for j in range(i + 1, n))) / a[i][i]
    return x


def response(f):
    w = 2 * math.pi * f
    D = E * I * (1 + 1j * beta * w)
    k = (mu * w * w / D) ** 0.25

    def basis(x, order):  # cosh, sinh, cos, sin and their derivatives
        z = k * x
        c, s, C, S = cmath.cosh(z), cmath.sinh(z), cmath.cos(z), cmath.sin(z)
        return {0: [c, s, C, S],
                1: [k * s, k * c, -k * S, k * C],
                2: [k * k * c, k * k * s, -k * k * C, -k * k * S],
                3: [k**3 * s, k**3 * c, k**3 * S, -k**3 * C]}[order]

    w0, w1 = basis(0, 0), basis(0, 1)
    wL, dL, mL, sL = basis(L, 0), basis(L, 1), basis(L, 2), basis(L, 3)
    row3 = [D * mL[i] - w * w * (M * rc * wL[i] + (J + M * rc * rc) * dL[i]) for i in range(4)]
    row4 = [D * sL[i] + w * w * M * (wL[i] + rc * dL[i]) for i in range(4)]
    a = solve4([w0, w1, row3, row4], [B, 0, 0, 0])
    return lambda x: sum(a[i] * basis(x, 0)[i] for i in range(4))


if __name__ == "__main__":
    best, f = (0.0, None), 78.0
    while f <= 85.0 + 1e-9:  # 1 mHz scan of the tip amplitude
        amp = abs(response(f)(L))
        if amp > best[0]:
            best = (amp, f)
        f += 0.001
    peak = best[1]
    r = response(peak)
    print(f"peak frequency {peak:.2f} Hz")
    for x in (0.0364, 0.0574, 0.0784, 0.0994, 0.1204):
        print(f"x = {x:.4f} m: {abs(r(x)) * 1e3:.6f} mm")
