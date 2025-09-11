"""
2_2_motors_viz.py (Chrono 8.x safe, Irrlicht)
=============================================

Level 2.2 — Motors

What you will see
-----------------
One window with four rigs (left -> right):
  1) Rotation Speed Motor (constant)  — bar spins steadily about Z.
  2) Rotation Speed Motor (sine)      — bar oscillates (ω(t) = A*sin(2πft + φ)).
  3) Linear  Speed Motor (constant)   — block slides along +X at constant speed.
  4) Linear  Speed Motor (sine)       — block slides back & forth (v(t) = A*sin(2πft + φ)).

Key ideas
---------
- *Motors* add **actuation** on a **single relative DOF** defined by the motor's frame:
  - `ChLinkMotorRotationSpeed` drives **angular speed** about the motor frame **Z** axis.
  - `ChLinkMotorLinearSpeed`   drives **linear speed** along the motor frame **X** axis.
- Motion profiles are fed with `ChFunction_*`:
  - `ChFunction_Const` for constant values.
  - `ChFunction_Sine`  for sinusoidal variation.

Robustness notes (PyChrono 8.x, Windows)
----------------------------------------
- Uses manual bodies (no ChBodyEasy* pitfalls) and collisions OFF (we focus on joints).
- Forces Bullet collision system (even if unused) for consistency.
- Colors via `ChVisualMaterial`; no deprecated assets; no iteration over SWIG containers.

Controls
--------
Left-drag: rotate | Right-drag: pan | Wheel: zoom | ESC/close window: quit

Run
---
    conda activate chrono
    python -u scripts/tutorials/level_2_2_motors/2_2_motors_viz.py
"""

import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr


# -------------------- small helpers --------------------

def colorize(vshape, rgb):
    """Attach a ChVisualMaterial (RGB) to a visual shape (Chrono 8.x-safe)."""
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))
    if hasattr(vshape, "GetMaterials"):
        vshape.GetMaterials().push_back(mat)
    elif hasattr(vshape, "material_list"):
        vshape.material_list.push_back(mat)


def make_const_function(val: float):
    """Return a Chrono constant function, tolerant to binding differences."""
    try:
        return chrono.ChFunction_Const(float(val))
    except TypeError:
        f = chrono.ChFunction_Const()
        if hasattr(f, "Set_yconst"):
            f.Set_yconst(float(val))
        elif hasattr(f, "SetConstant"):
            f.SetConstant(float(val))
        return f


def make_sine_function(amplitude: float, freq_hz: float, phase: float = 0.0):
    """
    Return a Chrono sine function with amplitude and frequency (Hz).
    Handles 8.x name differences (SetAmp/SetAmplitude, SetFreq/SetFrequency).
    """
    f = chrono.ChFunction_Sine()
    if hasattr(f, "SetAmp"):
        f.SetAmp(float(amplitude))
    elif hasattr(f, "SetAmplitude"):
        f.SetAmplitude(float(amplitude))
    if hasattr(f, "SetFreq"):
        f.SetFreq(float(freq_hz))
    elif hasattr(f, "SetFrequency"):
        f.SetFrequency(float(freq_hz))
    if hasattr(f, "SetPhase"):
        f.SetPhase(float(phase))
    return f


def set_min_bounce_zero_if_supported(ch_material):
    """Zero min-bounce threshold if present (not critical here, but harmless)."""
    for name in ("SetMinBounceSpeed", "SetMinBounceVelocity"):
        if hasattr(ch_material, name):
            getattr(ch_material, name)(0.0)


# -------------------- system & simple bodies --------------------

def make_system():
    sys = chrono.ChSystemNSC()
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    return sys


def make_ground(sys):
    g = chrono.ChBody()
    g.SetBodyFixed(True)
    g.SetCollide(False)
    sys.Add(g)

    # draw a beam for reference
    beam = chrono.ChBoxShape()
    beam.GetBoxGeometry().Size = chrono.ChVectorD(1.6, 0.01, 0.05)
    set_color(beam, (0.55, 0.55, 0.55))
    g.AddVisualShape(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    return g


def make_bar(sys, length=0.4, thickness=0.04, density=500.0, color=(0.7, 0.7, 0.7)):
    # aligned along local +X
    sx, sy, sz = length, thickness, thickness
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
    colorize(vb, color)
    b.AddVisualShape(vb)

    sys.Add(b)
    return b


def make_block(sys, size=(0.14, 0.08, 0.08), density=500.0, color=(0.9, 0.8, 0.2)):
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
    colorize(vb, color)
    b.AddVisualShape(vb)

    sys.Add(b)
    return b


# -------------------- placement helpers --------------------

def place_bar_with_left_end_at(bar, pivot, length, angle_rad):
    """Put the bar so its left end is at 'pivot' and it makes 'angle_rad' about +Z in the XY plane."""
    q = chrono.Q_from_AngAxis(angle_rad, chrono.ChVectorD(0, 0, 1))
    R = chrono.ChMatrix33D(q)
    com_local = chrono.ChVectorD(length * 0.5, 0, 0)  # from left end to COM (local)
    bar.SetPos(pivot + R * com_local)
    bar.SetRot(q)


# -------------------- build scene --------------------

def main():
    sys = make_system()
    ground = make_ground(sys)

    # Layout X positions (left -> right)
    x_rot_const  = -1.6
    x_rot_sine   = -0.5
    x_lin_const  = +0.6
    x_lin_sine   = +1.7
    y0 = 0.0

    # ---------- (1) Rotation speed motor — constant ----------
    L = 0.40
    pivot_rc = chrono.ChVectorD(x_rot_const, y0, 0.0)
    bar_rc = make_bar(sys, length=L, color=(0.2, 0.6, 0.9))
    place_bar_with_left_end_at(bar_rc, pivot_rc, L, angle_rad=0.0)

    motor_rc = chrono.ChLinkMotorRotationSpeed()
    # Motor frame: origin at pivot, identity rot -> motor axis is +Z (Chrono convention)
    motor_rc.Initialize(bar_rc, ground, chrono.ChFrameD(pivot_rc, chrono.QUNIT))
    motor_rc.SetSpeedFunction(make_const_function(omega:=2.0))  # rad/s
    sys.AddLink(motor_rc)

    # ---------- (2) Rotation speed motor — sine ----------
    pivot_rs = chrono.ChVectorD(x_rot_sine, y0, 0.0)
    bar_rs = make_bar(sys, length=L, color=(0.9, 0.8, 0.2))
    place_bar_with_left_end_at(bar_rs, pivot_rs, L, angle_rad=0.0)

    motor_rs = chrono.ChLinkMotorRotationSpeed()
    motor_rs.Initialize(bar_rs, ground, chrono.ChFrameD(pivot_rs, chrono.QUNIT))
    # ω(t) = A * sin(2π f t + φ)
    A_w = 3.0       # rad/s amplitude
    f_w = 0.5       # Hz
    ph  = 0.0       # rad
    motor_rs.SetSpeedFunction(make_sine_function(A_w, f_w, ph))
    sys.AddLink(motor_rs)

    # ---------- (3) Linear speed motor — constant ----------
    pivot_lc = chrono.ChVectorD(x_lin_const, y0 + 0.20, 0.0)
    blk_lc = make_block(sys, size=(0.14, 0.08, 0.08), color=(0.2, 0.9, 0.2))
    blk_lc.SetPos(pivot_lc + chrono.ChVectorD(0.10, 0.0, 0.0))
    blk_lc.SetRot(chrono.QUNIT)

    motor_lc = chrono.ChLinkMotorLinearSpeed()
    # Motor frame identity -> translation axis is +X (Chrono convention)
    motor_lc.Initialize(blk_lc, ground, chrono.ChFrameD(pivot_lc, chrono.QUNIT))
    motor_lc.SetSpeedFunction(make_const_function(v:=0.4))  # m/s
    sys.AddLink(motor_lc)

    # visual "rail" for linear guide
    rail_lc = chrono.ChBoxShape()
    rail_lc.GetBoxGeometry().Size = chrono.ChVectorD(0.45, 0.005, 0.005)
    colorize(rail_lc, (0.7, 0.7, 0.7))
    ground.AddVisualShape(rail_lc, chrono.ChFrameD(pivot_lc + chrono.ChVectorD(0.45, 0, 0)))

    # ---------- (4) Linear speed motor — sine ----------
    pivot_ls = chrono.ChVectorD(x_lin_sine, y0 + 0.20, 0.0)
    blk_ls = make_block(sys, size=(0.14, 0.08, 0.08), color=(0.8, 0.3, 0.8))
    blk_ls.SetPos(pivot_ls + chrono.ChVectorD(0.10, 0.0, 0.0))
    blk_ls.SetRot(chrono.QUNIT)

    motor_ls = chrono.ChLinkMotorLinearSpeed()
    motor_ls.Initialize(blk_ls, ground, chrono.ChFrameD(pivot_ls, chrono.QUNIT))
    # v(t) = A * sin(2π f t + φ)
    A_v = 0.35      # m/s amplitude
    f_v = 0.5       # Hz
    motor_ls.SetSpeedFunction(make_sine_function(A_v, f_v, 0.0))
    sys.AddLink(motor_ls)

    rail_ls = chrono.ChBoxShape()
    rail_ls.GetBoxGeometry().Size = chrono.ChVectorD(0.45, 0.005, 0.005)
    colorize(rail_ls, (0.7, 0.7, 0.7))
    ground.AddVisualShape(rail_ls, chrono.ChFrameD(pivot_ls + chrono.ChVectorD(0.45, 0, 0)))

    # ---------------- visualization ----------------
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 760)
    vis.SetWindowTitle("Level 2.2 — Motors: rotation & linear (const + sine)")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(4.0, 1.4, 3.6), chrono.ChVectorD(0.0, 0.2, 0.0))

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
