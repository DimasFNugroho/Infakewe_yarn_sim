"""
coords_01_vectors.py
====================

Chrono vector basics: create 3D vectors, add/subtract, dot/cross products, and norms.

Works with PyChrono 8.x (no matrix helpers required).

Run:
    python scripts/tutorials/coords_01_vectors.py

Expected:
    Prints a, b, a±b, a·b, a×b, |a| and a couple of sanity assertions, then "OK ✓".
"""

import math
import pychrono as chrono


def vdot(a: "chrono.ChVectorD", b: "chrono.ChVectorD") -> float:
    """
    Compute the dot product between two Chrono vectors.

    Args:
        a: First vector (ChVectorD).
        b: Second vector (ChVectorD).

    Returns:
        Dot product (float).
    """
    return a.x * b.x + a.y * b.y + a.z * b.z


def vcross(a: "chrono.ChVectorD", b: "chrono.ChVectorD") -> "chrono.ChVectorD":
    """
    Compute the cross product a × b between two Chrono vectors.

    Args:
        a: First vector (ChVectorD).
        b: Second vector (ChVectorD).

    Returns:
        New ChVectorD representing the cross product.
    """
    return chrono.ChVectorD(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )


def vnorm(a: "chrono.ChVectorD") -> float:
    """
    Compute the Euclidean norm (length) of a Chrono vector.

    Args:
        a: Vector (ChVectorD).

    Returns:
        Length of the vector (float).
    """
    return math.sqrt(vdot(a, a))


def fmt(v: "chrono.ChVectorD") -> str:
    """Format a Chrono vector as a short string."""
    return f"({v.x:.6f}, {v.y:.6f}, {v.z:.6f})"


def main() -> None:
    """Demonstrate vector arithmetic and basic identities."""
    a = chrono.ChVectorD(1, 2, 3)
    b = chrono.ChVectorD(4, -1, 0.5)

    print("a       =", fmt(a))
    print("b       =", fmt(b))
    print("a + b   =", fmt(a + b))
    print("a - b   =", fmt(a - b))
    print("a · b   =", f"{vdot(a, b):.6f}")
    print("a × b   =", fmt(vcross(a, b)))
    print("|a|     =", f"{vnorm(a):.6f}")

    # Tiny checks
    assert abs(vdot(a, a) - vnorm(a) ** 2) < 1e-12
    assert abs(vdot(a, b) - vdot(b, a)) < 1e-12
    print("OK ✓")


if __name__ == "__main__":
    main()
