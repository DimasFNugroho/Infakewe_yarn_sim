"""
coords_04_frames.py
===================

Chrono frames (ChFrameD): store a pose (position + rotation) and transform points.
Uses quaternion math for portability with PyChrono 8.x; optionally prints helper results
if your build exposes TransformLocalToParent/TransformParentToLocal.

Run:
    python scripts/tutorials/coords_04_frames.py
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
    Show how to use ChFrameD for point transforms.

    Steps:
        1) Build a frame F at pos=(1,2,3) with rot=+90° about Z.
        2) Manually transform local point (1,0,0) to world and print result.
        3) If helper methods exist, call them for comparison.
    """
    pos = chrono.ChVectorD(1, 2, 3)
    rot = chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(0, 0, 1))
    F = chrono.ChFrameD(pos, rot)

    p_local = chrono.ChVectorD(1, 0, 0)

    # manual local -> world
    pw = q_rotate(F.GetRot(), p_local)
    pw = chrono.ChVectorD(pw.x + F.GetPos().x, pw.y + F.GetPos().y, pw.z + F.GetPos().z)
    print("Manual F·local =", f"({pw.x:.6f}, {pw.y:.6f}, {pw.z:.6f}) ~ (1,3,3)")

    # optional helpers (if present in your build)
    if hasattr(F, "TransformLocalToParent"):
        tp = F.TransformLocalToParent(p_local)
        print("Helper F.TransformLocalToParent =", f"({tp.x:.6f}, {tp.y:.6f}, {tp.z:.6f})")
    if hasattr(F, "TransformParentToLocal"):
        back = F.TransformParentToLocal(pw)
        print("Helper F.TransformParentToLocal =", f"({back.x:.6f}, {back.y:.6f}, {back.z:.6f})")

    print("OK ✓")


if __name__ == "__main__":
    main()
