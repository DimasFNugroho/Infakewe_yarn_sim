"""
visualization_basics.py (Chrono 8.x safe, Irrlicht)
===================================================

Level 1.3 — Visualization basics

What this does
--------------
- Opens an Irrlicht window and renders:
  * fixed floor at y = 0
  * one falling box
  * two spheres with different restitution (low vs high bounce)
- Runs interactively until you close the window.

Why this works with PyChrono 8.x
--------------------------------
- Uses ChVisualMaterial (no deprecated ChColorAsset).
- Sets colors by pushing materials onto each visual shape’s material list.
- Avoids iterating SWIG vectors directly.
- Builds bodies manually (visual + collision) and forces Bullet collision.

Controls
--------
- Left-drag: rotate | Right-drag: pan | Wheel: zoom | Close window or ESC: quit

Run
---
    conda activate chrono
    python -u scripts/tutorials/1_3_visualization_basics/visualization_basics.py
"""

import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr


# -------------------- helpers: visuals & materials --------------------

def set_color_on_visual_shape(vshape, rgb=(0.6, 0.6, 0.6)):
    """
    Attach a ChVisualMaterial with the given RGB color to a visual shape.

    Works across PyChrono 8.x Python bindings by trying both APIs:
      - vshape.GetMaterials().push_back(mat)
      - vshape.material_list.push_back(mat)
    """
    mat = chrono.ChVisualMaterial()
    mat.SetDiffuseColor(chrono.ChColor(*rgb))

    if hasattr(vshape, "GetMaterials"):
        vshape.GetMaterials().push_back(mat)
    elif hasattr(vshape, "material_list"):
        vshape.material_list.push_back(mat)


def set_min_bounce_zero_if_supported(ch_material):
    """Zero min bounce speed if the binding exposes it."""
    for name in ("SetMinBounceSpeed", "SetMinBounceVelocity"):
        if hasattr(ch_material, name):
            getattr(ch_material, name)(0.0)


# -------------------- system & bodies --------------------

def make_system():
    """Create an NSC system, force Bullet collision, set gravity."""
    sys = chrono.ChSystemNSC()
    try:
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    except Exception:
        pass
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    return sys


def make_floor(sys, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0), color=(0.55, 0.55, 0.55)):
    """Fixed floor: box visual + box collision. Floor top ends at y = 0."""
    sx, sy, sz = size
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5

    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(0.5)
    mat.SetRestitution(0.2)
    set_min_bounce_zero_if_supported(mat)

    body = chrono.ChBody()
    body.SetBodyFixed(True)
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))

    vbox = chrono.ChBoxShape()
    vbox.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    set_color_on_visual_shape(vbox, color)
    body.AddVisualShape(vbox)

    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddBox(mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()

    sys.Add(body)
    return body


def make_box(sys, size=(0.2, 0.2, 0.2), pos=(0.0, 0.5, 0.0), color=(0.2, 0.6, 0.9)):
    """Dynamic box: box visual + box collision, with proper mass/inertia."""
    sx, sy, sz = size
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    density = 500.0

    volume = sx * sy * sz
    mass = max(1e-6, density * volume)
    Ix = (1.0 / 12.0) * mass * (sy * sy + sz * sz)
    Iy = (1.0 / 12.0) * mass * (sx * sx + sz * sz)
    Iz = (1.0 / 12.0) * mass * (sx * sx + sy * sy)

    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(0.4)
    mat.SetRestitution(0.3)
    set_min_bounce_zero_if_supported(mat)

    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVectorD(Ix, Iy, Iz))

    vbox = chrono.ChBoxShape()
    vbox.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    set_color_on_visual_shape(vbox, color)
    body.AddVisualShape(vbox)

    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddBox(mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()

    sys.Add(body)
    return body


def make_sphere(sys, radius=0.10, pos=(0.5, 0.8, 0.0), restitution=0.8, color=(0.2, 0.9, 0.2)):
    """Dynamic sphere: sphere visual + sphere collision, with proper mass/inertia."""
    density = 500.0
    volume = (4.0 / 3.0) * math.pi * radius**3
    mass = max(1e-6, density * volume)
    I = 0.4 * mass * radius * radius  # 2/5 m r^2

    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(0.3)
    mat.SetRestitution(float(restitution))
    set_min_bounce_zero_if_supported(mat)

    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(mass)
    body.SetInertiaXX(chrono.ChVectorD(I, I, I))

    vs = chrono.ChSphereShape()
    vs.GetSphereGeometry().rad = radius
    set_color_on_visual_shape(vs, color)
    body.AddVisualShape(vs)

    body.SetCollide(True)
    cm = body.GetCollisionModel()
    cm.ClearModel()
    cm.AddSphere(mat, radius, chrono.ChVectorD(0, 0, 0))
    cm.BuildModel()

    sys.Add(body)
    return body


# -------------------- main (Irrlicht loop) --------------------

def main():
    """Build the scene, initialize Irrlicht, and run an interactive loop."""
    sys = make_system()

    # Scene
    make_floor(sys, size=(2.0, 0.05, 2.0), pos=(0.0, -0.025, 0.0), color=(0.55, 0.55, 0.55))
    make_box(sys,  size=(0.2, 0.2, 0.2), pos=(0.0, 0.5, 0.0),  color=(0.2, 0.6, 0.9))
    # Low-bounce sphere (yellow-ish) and high-bounce sphere (green)
    make_sphere(sys, radius=0.10, pos=(+0.5, 0.8, 0.0), restitution=0.1, color=(0.9, 0.8, 0.2))
    make_sphere(sys, radius=0.10, pos=(-0.5, 1.0, 0.0), restitution=0.9, color=(0.2, 0.9, 0.2))

    # Visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Chrono Visualization Basics (Level 1.3)")
    vis.Initialize()
    # vis.AddLogo()  # Optional (no args); omitted to avoid binding differences
    vis.AddSkyBox()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVectorD(1.5, 1.0, 1.5), chrono.ChVectorD(0, 0, 0))

    # Bind all assets (materials/shapes) to the visual system
    try:
        vis.BindAll()
    except Exception:
        pass

    # Interactive loop
    timestep = 1e-3
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(timestep)


if __name__ == "__main__":
    main()
