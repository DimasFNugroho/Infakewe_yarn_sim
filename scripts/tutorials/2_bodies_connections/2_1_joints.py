"""
links_01_constraints_viz.py (Chrono 8.x safe, Irrlicht)
=======================================================

Level 2.1 — Links & joints (visual, comprehensive)

What this shows
---------------
Five classic Chrono joints, each connecting a *dynamic* body to a *fixed* ground:
  1) Revolute (hinge): 1 rotational DOF (about the joint axis).
  2) Spherical (ball): 3 rotational DOFs at a point.
  3) Prismatic (slider): 1 translational DOF along the joint axis.
  4) Cylindrical: translation + rotation along the joint axis (2 DOFs).
  5) Distance: constrains the distance between two markers (rope/rod).

Why this version is robust on PyChrono 8.x
------------------------------------------
- Uses ChVisualMaterial (not deprecated assets) to color shapes.
- Iterates no SWIG containers directly; we add materials to shapes we create.
- Builds simple manual bodies (box/sphere) and disables collisions (focus on joints).
- Forces Bullet collision system (even though unused here) for consistency.
- Clear, axis-annotated initialization frames for each joint.

Controls
--------
- Left-drag: rotate | Right-drag: pan | Wheel: zoom | Close window or ESC to quit.

Run
---
    conda activate chrono
    python -u scripts/tutorials/level_2_1_links_and_joints/links_01_constraints_viz.py
"""

import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr


# -------------------- helpers: visuals & materials --------------------

def add_color(shape, rgb):
    """Attach a ChVisualMaterial with the given RGB color to a visual shape."""
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    # Newer binding:
    if hasattr(shape, "GetMaterials"):
        shape.GetMaterials().push_back(mat)
    # Older fallback:
    elif hasattr(shape, "material_list"):
        shape.material_list.push_back(mat)


def add_axis_gizmo(body, axis_len=0.12, frame=chrono.ChFrameD()):
    """
    Add a tiny RGB axis gizmo (X=red, Y=green, Z=blue) in the body's visual model.
    Helps visualize the *joint frame* orientation at a pivot.
    """
    # X
    bx = chrono.ChBoxShape()
    bx.GetBoxGeometry().Size = chrono.ChVectorD(axis_len, 0.005, 0.005)
    add_color(bx, (0.9, 0.1, 0.1))
    body.AddVisualShape(bx, chrono.ChFrameD(frame.GetPos() + frame.GetA() * chrono.ChVectorD(axis_len * 0.5, 0, 0)))
    # Y
    by = chrono.ChBoxShape()
    by.GetBoxGeometry().Size = chrono.ChVectorD(0.005, axis_len, 0.005)
    add_color(by, (0.1, 0.8, 0.1))
    body.AddVisualShape(by, chrono.ChFrameD(frame.GetPos() + frame.GetA() * chrono.ChVectorD(0, axis_len * 0.5, 0)))
    # Z
    bz = chrono.ChBoxShape()
    bz.GetBoxGeometry().Size = chrono.ChVectorD(0.005, 0.005, axis_len)
    add_color(bz, (0.2, 0.5, 0.9))
    body.AddVisualShape(bz, chrono.ChFrameD(frame.GetPos() + frame.GetA() * chrono.ChVectorD(0, 0, axis_len * 0.5)))


# -------------------- system & basic bodies --------------------

def make_system():
    """Create NSC system, force Bullet collision, set gravity."""
    sys = chrono.ChSystemNSC()
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    return sys


def make_ground(sys):
    """Fixed ground (no collision), with a horizontal reference beam."""
    g = chrono.ChBody()
    g.SetBodyFixed(True)
    g.SetCollide(False)
    sys.Add(g)

    beam = chrono.ChBoxShape()
    beam.GetBoxGeometry().Size = chrono.ChVectorD(2.2, 0.01, 0.05)
    add_color(beam, (0.55, 0.55, 0.55))
    g.AddVisualShape(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    return g


def make_rod(sys, length=0.6, thickness=0.04, density=500.0, color=(0.2, 0.6, 0.9)):
    """
    Slender rectangular bar aligned with local +Y (length direction).
    Collisions off; we only visualize and use kinematics.
    """
    sx, sy, sz = thickness, length, thickness
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    volume = sx * sy * sz
    mass = max(1e-6, density * volume)
    Ix = (1.0 / 12.0) * mass * (sy * sy + sz * sz)
    Iy = (1.0 / 12.0) * mass * (sx * sx + sz * sz)
    Iz = (1.0 / 12.0) * mass * (sx * sx + sy * sy)

    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(Ix, Iy, Iz))
    b.SetCollide(False)

    v = chrono.ChBoxShape()
    v.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    add_color(v, color)
    b.AddVisualShape(v)

    sys.Add(b)
    return b


def make_block(sys, size=(0.12, 0.08, 0.08), density=500.0, color=(0.9, 0.8, 0.2)):
    """Simple brick aligned with local axes, collisions off."""
    sx, sy, sz = size
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    volume = sx * sy * sz
    mass = max(1e-6, density * volume)
    Ix = (1.0 / 12.0) * mass * (sy * sy + sz * sz)
    Iy = (1.0 / 12.0) * mass * (sx * sx + sz * sz)
    Iz = (1.0 / 12.0) * mass * (sx * sx + sy * sy)

    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(Ix, Iy, Iz))
    b.SetCollide(False)

    vb = chrono.ChBoxShape()
    vb.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    add_color(vb, color)
    b.AddVisualShape(vb)

    sys.Add(b)
    return b


def make_ball(sys, radius=0.06, density=500.0, color=(0.2, 0.9, 0.2)):
    """Small sphere (collisions off)."""
    volume = (4.0 / 3.0) * math.pi * radius**3
    mass = max(1e-6, density * volume)
    I = 0.4 * mass * radius * radius

    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetInertiaXX(chrono.ChVectorD(I, I, I))
    b.SetCollide(False)

    vs = chrono.ChSphereShape()
    vs.GetSphereGeometry().rad = radius
    add_color(vs, color)
    b.AddVisualShape(vs)

    sys.Add(b)
    return b


# -------------------- placement helpers --------------------

def place_pendulum_along_y(body, pivot, length, theta_deg, axis=chrono.ChVectorD(0, 0, 1)):
    """
    Position a pendulum rod so its top end is at 'pivot' and it is rotated by
    'theta_deg' about 'axis' from vertical-down. The rod's local +Y is lengthwise.
    """
    theta = math.radians(theta_deg)
    q = chrono.Q_from_AngAxis(theta, axis)
    R = chrono.ChMatrix33D(q)
    top_local_from_com = chrono.ChVectorD(0, length * 0.5, 0)
    com_world = pivot - R * top_local_from_com
    body.SetPos(com_world)
    body.SetRot(q)


# -------------------- scene build --------------------

def main():
    sys = make_system()
    ground = make_ground(sys)

    # Layout (left -> right)
    y0 = 0.0
    X = {
        "rev":  -1.2,
        "sph":  -0.4,
        "pri":  +0.4,
        "cyl":  +1.2,
        "dist": +2.0,  # slightly off-screen to the right; camera centers around 0
    }

    # ---------------- 1) REVOLUTE (hinge) ----------------
    # DOF: 1 rotation about joint axis (here we use +Z of joint frame)
    L = 0.6
    pivot_rev = chrono.ChVectorD(X["rev"], y0, 0.0)
    rod_rev = make_rod(sys, length=L, color=(0.2, 0.6, 0.9))
    place_pendulum_along_y(rod_rev, pivot_rev, L, theta_deg=25, axis=chrono.ChVectorD(0, 0, 1))

    # Joint frame: origin at pivot, identity rotation => joint Z is world Z
    J_rev = chrono.ChCoordsysD(pivot_rev, chrono.ChQuaternionD(1, 0, 0, 0))
    link_rev = chrono.ChLinkLockRevolute()
    link_rev.Initialize(rod_rev, ground, J_rev)
    sys.AddLink(link_rev)

    # visualize joint frame
    add_axis_gizmo(ground, axis_len=0.10, frame=chrono.ChFrameD(pivot_rev))

    # ---------------- 2) SPHERICAL (ball) ----------------
    # DOF: 3 rotations at a point (no relative translation)
    pivot_sph = chrono.ChVectorD(X["sph"], y0, 0.0)
    rod_sph = make_rod(sys, length=L, color=(0.2, 0.9, 0.2))
    place_pendulum_along_y(rod_sph, pivot_sph, L, theta_deg=25, axis=chrono.ChVectorD(0, 0, 1))

    link_sph = chrono.ChLinkLockSpherical()
    # Joint frame orientation is irrelevant for a pure spherical; origin must match pivot
    link_sph.Initialize(rod_sph, ground, chrono.ChCoordsysD(pivot_sph, chrono.ChQuaternionD(1, 0, 0, 0)))
    sys.AddLink(link_sph)

    # give small out-of-plane spin to show 3D motion
    rod_sph.SetWvel_loc(chrono.ChVectorD(0.6, 0.0, 0.0))
    add_axis_gizmo(ground, axis_len=0.10, frame=chrono.ChFrameD(pivot_sph))

    # ---------------- 3) PRISMATIC (slider) ----------------
    # DOF: 1 translation along the joint axis.
    # We'll align the *joint X axis* with world +X by identity rotation and give the block an initial Vx.
    pivot_pri = chrono.ChVectorD(X["pri"], y0 + 0.2, 0.0)
    block = make_block(sys, size=(0.12, 0.08, 0.08), color=(0.9, 0.8, 0.2))
    block.SetPos(pivot_pri + chrono.ChVectorD(0.15, 0.0, 0.0))  # start a bit to +X
    block.SetRot(chrono.QUNIT)  # no rotation
    block.SetPos_dt(chrono.ChVectorD(+0.5, 0, 0))  # initial speed along +X

    # Joint frame: identity => X axis of joint is world +X
    J_pri = chrono.ChCoordsysD(pivot_pri, chrono.ChQuaternionD(1, 0, 0, 0))
    link_pri = chrono.ChLinkLockPrismatic()
    link_pri.Initialize(block, ground, J_pri)
    sys.AddLink(link_pri)

    # draw the slider rail as a thin visual bar
    rail = chrono.ChBoxShape()
    rail.GetBoxGeometry().Size = chrono.ChVectorD(0.35, 0.005, 0.005)
    add_color(rail, (0.7, 0.7, 0.7))
    ground.AddVisualShape(rail, chrono.ChFrameD(pivot_pri + chrono.ChVectorD(0.35, 0, 0)))
    add_axis_gizmo(ground, axis_len=0.10, frame=chrono.ChFrameD(pivot_pri))

    # ---------------- 4) CYLINDRICAL ----------------
    # DOF: rotation about axis + translation along the same axis (2 DOFs).
    # We'll align joint X with world +X; give initial Vx and spin about +X.
    pivot_cyl = chrono.ChVectorD(X["cyl"], y0 + 0.2, 0.0)
    bar_cyl = make_block(sys, size=(0.16, 0.04, 0.04), color=(0.2, 0.6, 0.9))
    bar_cyl.SetPos(pivot_cyl + chrono.ChVectorD(0.10, 0.0, 0.0))
    bar_cyl.SetRot(chrono.QUNIT)
    bar_cyl.SetPos_dt(chrono.ChVectorD(+0.4, 0, 0))
    bar_cyl.SetWvel_loc(chrono.ChVectorD(+6.0, 0, 0))  # spin about +X

    J_cyl = chrono.ChCoordsysD(pivot_cyl, chrono.ChQuaternionD(1, 0, 0, 0))
    link_cyl = chrono.ChLinkLockCylindrical()
    link_cyl.Initialize(bar_cyl, ground, J_cyl)
    sys.AddLink(link_cyl)

    # draw a guide tube
    tube = chrono.ChBoxShape()
    tube.GetBoxGeometry().Size = chrono.ChVectorD(0.35, 0.005, 0.005)
    add_color(tube, (0.7, 0.7, 0.7))
    ground.AddVisualShape(tube, chrono.ChFrameD(pivot_cyl + chrono.ChVectorD(0.35, 0, 0)))
    add_axis_gizmo(ground, axis_len=0.10, frame=chrono.ChFrameD(pivot_cyl))

    # ---------------- 5) DISTANCE ----------------
    # Constrain the distance between two markers: a mass attached to a ground point by a "rod".
    pivot_dist = chrono.ChVectorD(0.0, y0 + 0.6, -0.8)  # tucked back in Z so it doesn't overlap others
    ball = make_ball(sys, radius=0.06, color=(0.2, 0.9, 0.9))

    Ld = 0.5  # desired fixed length
    # Place the ball at exactly 'Ld' below the pivot so auto-rest-length matches Ld
    ball.SetPos(pivot_dist + chrono.ChVectorD(0.0, -Ld, 0.0))
    ball.SetRot(chrono.QUNIT)

    link_dist = chrono.ChLinkDistance()
    # Initialize with: bodies, auto, pos1 (world), pos2 (world), auto_distance=False then set distance
    ## Try the 5-arg signature first; fall back to the older 6-arg one if needed.
    try:
        # Newer/typical binding: Initialize(body1, body2, auto_rest, pos1, pos2)
        link_dist.Initialize(ball, ground, True, ball.GetPos(), pivot_dist)
    except TypeError:
        # Older overload includes two trailing bools; set both to True (auto compute)
        link_dist.Initialize(ball, ground, True, ball.GetPos(), pivot_dist, True)

    sys.AddLink(link_dist)

    # (Optional) small axis gizmo at the ground pivot so you can see the anchor point
    add_axis_gizmo(ground, axis_len=0.10, frame=chrono.ChFrameD(pivot_dist))

    # visual rod
    rod_vis = chrono.ChCylinderShape()
    rod_vis.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
    rod_vis.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -Ld, 0)
    rod_vis.GetCylinderGeometry().rad = 0.005
    add_color(rod_vis, (0.8, 0.8, 0.2))
    # attach to an auxiliary body placed at the pivot so the cylinder displays from pivot down
    pivot_vis = chrono.ChBody()
    pivot_vis.SetBodyFixed(True)
    pivot_vis.SetCollide(False)
    pivot_vis.SetPos(pivot_dist)
    sys.Add(pivot_vis)
    pivot_vis.AddVisualShape(rod_vis)

    # ---------------- visualization ----------------
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 760)
    vis.SetWindowTitle("Level 2.1 — Links & joints: revolute, spherical, prismatic, cylindrical, distance")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(3.0, 1.2, 3.0), chrono.ChVectorD(0.0, 0.1, 0.0))

    try:
        vis.BindAll()
    except Exception:
        pass

    timestep = 1e-3
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(timestep)


if __name__ == "__main__":
    main()
