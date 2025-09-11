"""
coords_05_coordsys.py
=====================

ChCoordsysD (pose without velocities) vs ChFrameD (pose with convenience methods).
Shows they represent the same pose information for transforming points; we do the
math manually for PyChrono 8.x compatibility.

Run:
    python scripts/tutorials/coords_05_coordsys.py

Expected:
    Local (1,0,0) maps to the same world coordinates using either coordsys or frame.
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


def fmt(v: "chrono.ChVectorD") -> str:
    """Format a Chrono vector as a short string."""
    return f"({v.x:.6f}, {v.y:.6f}, {v.z:.6f})"


def main() -> None:
    """
    Compare a ChCoordsysD and a ChFrameD representing the same pose.

    Demonstrates:
        - Using the same position & rotation to transform a local point with either type.
        - The results match to numerical precision.
    """
    pos = chrono.ChVectorD(1, 2, 3)
    rot = chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(0, 0, 1))

    # coordsys (pose: pos + rot)
    C = chrono.ChCoordsysD(pos, rot)
    # frame (pose with convenience funcs)
    F = chrono.ChFrameD(pos, rot)

    p_local = chrono.ChVectorD(1, 0, 0)

    # via coordsys
    pw_c = q_rotate(C.rot, p_local)
    pw_c = chrono.ChVectorD(pw_c.x + C.pos.x, pw_c.y + C.pos.y, pw_c.z + C.pos.z)

    # via frame (manual, to avoid relying on helpers)
    pw_f = q_rotate(F.GetRot(), p_local)
    pw_f = chrono.ChVectorD(pw_f.x + F.GetPos().x, pw_f.y + F.GetPos().y, pw_f.z + F.GetPos().z)

    print("World via coordsys:", fmt(pw_c), "  ~ (1,3,3)")
    print("World via frame:   ", fmt(pw_f), "  ~ (1,3,3)")

    # they should match closely
    assert abs(pw_c.x - pw_f.x) < 1e-12 and abs(pw_c.y - pw_f.y) < 1e-12 and abs(pw_c.z - pw_f.z) < 1e-12
    print("OK ✓")


if __name__ == "__main__":
    main()
