"""
coords_02_quaternions.py
========================

Quaternion basics for Chrono: build rotations, compose them, and rotate vectors without
needing any matrix helpers (compatible with PyChrono 8.x).

Run:
    python scripts/tutorials/coords_02_quaternions.py

Expected:
    Shows a +90° about Z rotating X→Y, inverse rotation via conjugate, and a composed rotation.
"""

import math
import pychrono as chrono


def vcross(a: "chrono.ChVectorD", b: "chrono.ChVectorD") -> "chrono.ChVectorD":
    """
    Chrono vector cross product a × b.

    Returns:
        New ChVectorD.
    """
    return chrono.ChVectorD(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )


def q_conj(q: "chrono.ChQuaternionD") -> "chrono.ChQuaternionD":
    """
    Conjugate of a Chrono quaternion.

    Args:
        q: Quaternion (e1, e2, e3, e0) where e0 is scalar part.

    Returns:
        Conjugate quaternion (negated vector part, same scalar).
    """
    qc = chrono.ChQuaternionD(q)
    qc.e1, qc.e2, qc.e3, qc.e0 = -q.e1, -q.e2, -q.e3, q.e0
    return qc


def q_rotate(q: "chrono.ChQuaternionD", v: "chrono.ChVectorD") -> "chrono.ChVectorD":
    """
    Rotate vector v by quaternion q using a matrix-free formula:
        v' = v + 2*s*(u×v) + 2*(u×(u×v)), with q = (u, s).

    Args:
        q: Rotation quaternion.
        v: Vector to rotate.

    Returns:
        Rotated vector (ChVectorD).
    """
    u = chrono.ChVectorD(q.e1, q.e2, q.e3)
    s = q.e0
    t = vcross(u, v)
    term1 = chrono.ChVectorD(2 * s * t.x, 2 * s * t.y, 2 * s * t.z)
    term2 = vcross(u, chrono.ChVectorD(2 * t.x, 2 * t.y, 2 * t.z))
    return chrono.ChVectorD(
        v.x + term1.x + term2.x,
        v.y + term1.y + term2.y,
        v.z + term1.z + term2.z,
    )


def main() -> None:
    """Demonstrate building and composing quaternion rotations and rotating vectors."""
    qz90 = chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(0, 0, 1))  # +90° about Z
    x = chrono.ChVectorD(1, 0, 0)
    y_from_x = q_rotate(qz90, x)
    print("Rz90 * x =", f"({y_from_x.x:.6f}, {y_from_x.y:.6f}, {y_from_x.z:.6f}) ~ (0,1,0)")

    x_back = q_rotate(q_conj(qz90), y_from_x)
    print("Rz90^-1 * (Rz90*x) =", f"({x_back.x:.6f}, {x_back.y:.6f}, {x_back.z:.6f}) ~ (1,0,0)")

    qx90 = chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(1, 0, 0))
    q_combo = qz90 * qx90  # apply X after Z
    v = chrono.ChVectorD(0, 0, 1)
    v_rot = q_rotate(q_combo, v)
    print("Rz90 then Rx90 applied to z:", f"({v_rot.x:.6f}, {v_rot.y:.6f}, {v_rot.z:.6f})")
    print("OK ✓")


if __name__ == "__main__":
    main()
