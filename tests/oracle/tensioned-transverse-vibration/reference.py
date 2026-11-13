#!/usr/bin/env python3
"""Closed-form references for the tensioned rectangular bar."""
import json
import math

L = 1.0
B = 0.04
H = 0.02
E = 1.0e9
RHO = 1000.0
AREA = B * H


def beam_roots(inertia, tension, count):
    mass_per_length = RHO * AREA

    def equation(omega):
        root = math.sqrt(tension * tension + 4.0 * inertia * mass_per_length * omega * omega)
        a = math.sqrt((root + tension) / (2.0 * inertia))
        b = math.sqrt((root - tension) / (2.0 * inertia))
        return (
            2.0 * a * b * (1.0 / math.cosh(a * L) - math.cos(b * L))
            + (a * a - b * b) * math.tanh(a * L) * math.sin(b * L)
        )

    roots = []
    omega = 1.0
    step = 0.05
    previous = equation(omega)
    while len(roots) < count:
        next_omega = omega + step
        current = equation(next_omega)
        if previous == 0.0 or previous * current < 0.0:
            low, high = omega, next_omega
            for _ in range(200):
                middle = 0.5 * (low + high)
                if equation(low) * equation(middle) <= 0.0:
                    high = middle
                else:
                    low = middle
            roots.append(0.5 * (low + high) / (2.0 * math.pi))
        omega, previous = next_omega, current
    return roots


def references(tension):
    weak = beam_roots(E * B * H**3 / 12.0, tension, 4)
    strong = beam_roots(E * H * B**3 / 12.0, tension, 3)
    modes = [(frequency, f"weak-axis-{i + 1}") for i, frequency in enumerate(weak)]
    modes += [(frequency, f"strong-axis-{i + 1}") for i, frequency in enumerate(strong)]
    modes.sort()
    return [{"frequency_hz": frequency, "mode": mode} for frequency, mode in modes[:3]]


if __name__ == "__main__":
    print(json.dumps({"tensioned": references(4000.0), "zero_tension": references(0.0)}, indent=2))
