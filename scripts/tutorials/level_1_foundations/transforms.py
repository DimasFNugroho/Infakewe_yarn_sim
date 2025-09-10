"""
coords_03_transforms.py
=======================

Local ↔ world point transforms using quaternion math (no matrix helpers required).
Compatible with PyChrono 8.x.

Run:
    python scripts/tutorials/coords_03_transforms.py

Expected:
    Local (1,0,0) at a frame rotated +90° about Z and positioned at (1,2,3) maps to (1,3,3).
"""

import math
import pychrono as chrono


def vcross(a: "chrono.ChVectorD", b: "chrono.ChVectorD") -> "chrono.ChVectorD":
    """Chrono vector cross product a × b."""
    return chrono.ChVectorD(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )


def q_conj(q: "chrono.ChQuaternionD") -> "chrono.ChQuaternionD":
    """Return the conjugate quaternion (inverse for unit quaternions)."""
    qc = chrono.ChQuaternionD(q)
    qc.e1, qc.e2, qc.e3, qc.e0 = -q.e1, -q.e2, -q.e3, q.e0
    return qc


def q_rotate(q: "chrono.ChQuaternionD", v: "chrono.ChVectorD") -> "chrono.ChVectorD":
    """Rotate vector v by quaternion q (matrix-free)."""
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
    """
    Transform a local point using a pose (position + rotation) and invert the transform.

    Demonstrates:
        - world = R(q) * local + pos
        - local = R(q)^T * (world - pos)  (via quaternion conjugate)
    """
    pos = chrono.ChVectorD(1, 2, 3)
    q = chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(0, 0, 1))  # +90° about Z
    p_local = chrono.ChVectorD(1, 0, 0)

    # world = R(q)*local + pos
    p_world_rot = q_rotate(q, p_local)
    p_world = chrono.ChVectorD(
        p_world_rot.x + pos.x, p_world_rot.y + pos.y, p_world_rot.z + pos.z
    )
    print(
        "Local (1,0,0) -> World =",
        f"({p_world.x:.6f}, {p_world.y:.6f}, {p_world.z:.6f}) ~ (1,3,3)",
    )

    # local = R(q)^T * (world - pos)
    w_minus_p = chrono.ChVectorD(p_world.x - pos.x, p_world.y - pos.y, p_world.z - pos.z)
    p_back = q_rotate(q_conj(q), w_minus_p)
    print(
        "Back-transform -> Local =",
        f"({p_back.x:.6f}, {p_back.y:.6f}, {p_back.z:.6f}) ~ (1,0,0)",
    )
    print("OK ✓")


if __name__ == "__main__":
    main()
