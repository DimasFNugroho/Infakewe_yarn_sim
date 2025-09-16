# tests/test_sim_basic.py
"""
Basic smoke tests for NSC & SMC:
- Drop a cube onto a thick, axis-aligned floor.
- Verify it falls and settles near the expected height.

Notes:
- We build collision manually to avoid helper defaults.
- We prefer Bullet and single-thread stepping for stability.
- SMC uses a softer Young's modulus and smaller dt to prevent blow-ups.
"""

import math
import pychrono as chrono
from scripts.common.compat import (
    set_gravity, prefer_bullet, tune_collision_defaults, set_single_thread
)


# ---------- Materials ----------
def _make_nsc_material(mu=0.3, e=0.05):
    m = chrono.ChMaterialSurfaceNSC()
    if hasattr(m, "SetFriction"):     m.SetFriction(mu)
    if hasattr(m, "SetRestitution"):  m.SetRestitution(e)
    return m

def _make_smc_material(mu=0.3, e=0.05, young=5e6, nu=0.3, roll=0.0):
    m = chrono.ChMaterialSurfaceSMC()
    if hasattr(m, "SetFriction"):        m.SetFriction(mu)
    if hasattr(m, "SetRestitution"):     m.SetRestitution(e)
    if hasattr(m, "SetYoungModulus"):    m.SetYoungModulus(young)
    if hasattr(m, "SetPoissonRatio"):    m.SetPoissonRatio(nu)
    if hasattr(m, "SetRollingFriction"): m.SetRollingFriction(roll)
    # If damping setters exist in this build, add a little to avoid ringing
    for name, val in (("SetDampingF", 0.1), ("SetDampingT", 0.02)):
        if hasattr(m, name):
            try: getattr(m, name)(val)
            except Exception: pass
    return m


# ---------- Geometry ----------
def _add_box_with_manual_collision(sys, half, pos, fixed, mat, color=(0.7, 0.7, 0.7)):
    hx, hy, hz = half.x, half.y, half.z

    b = chrono.ChBody()
    b.SetPos(pos)
    b.SetBodyFixed(fixed)

    vol = (2*hx)*(2*hy)*(2*hz)
    mass = max(1e-6, 800.0 * vol)
    b.SetMass(mass)
    Ixx = (1.0/12.0)*mass*((2*hy)**2 + (2*hz)**2)
    Iyy = (1.0/12.0)*mass*((2*hx)**2 + (2*hz)**2)
    Izz = (1.0/12.0)*mass*((2*hx)**2 + (2*hy)**2)
    b.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz))

    # visual
    vs = chrono.ChBoxShape()
    vs.GetBoxGeometry().Size = chrono.ChVectorD(hx, hy, hz)
    vm = chrono.ChVisualMaterial(); vm.SetDiffuseColor(chrono.ChColor(*color))
    (vs.GetMaterials() if hasattr(vs, "GetMaterials") else vs.material_list).push_back(vm)
    b.AddVisualShape(vs)

    # collision
    cm = b.GetCollisionModel(); cm.ClearModel()
    cm.AddBox(mat, hx, hy, hz, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    cm.BuildModel()
    b.SetCollide(True)

    sys.Add(b)
    return b


def _build_scene(sys, model="NSC"):
    """
    Returns:
      box: the falling body
      floor_top: y of the floor top surface
      expected_y: expected box center height at rest (floor_top + box_half)
    """
    mat = _make_nsc_material() if model == "NSC" else _make_smc_material()

    # Thick, axis-aligned floor: top at y = floor_pos.y + floor_half
    floor_half = 0.05
    floor_pos_y = +floor_half
    floor = _add_box_with_manual_collision(
        sys, chrono.ChVectorD(1.0, floor_half, 1.0),
        chrono.ChVectorD(0, floor_pos_y, 0),
        fixed=True, mat=mat, color=(0.6, 0.6, 0.6)
    )
    floor_top = floor_pos_y + floor_half  # <-- correct top

    # Box (0.1 cube) starts at 0.50 m
    box_half = 0.05
    box = _add_box_with_manual_collision(
        sys, chrono.ChVectorD(box_half, box_half, box_half),
        chrono.ChVectorD(0, 0.50, 0),
        fixed=False, mat=mat, color=(0.2, 0.6, 0.9)
    )

    expected_y = floor_top + box_half
    return box, floor_top, expected_y


# ---------- Runner ----------
def _run_once(model="NSC", dt=None, t_end=0.8):
    sys = chrono.ChSystemNSC() if model == "NSC" else chrono.ChSystemSMC()
    prefer_bullet(sys)
    tune_collision_defaults(envelope=0.003, margin=0.002)
    set_single_thread(sys)
    set_gravity(sys, chrono.ChVectorD(0, -9.81, 0))

    # Model-specific gentle settings
    if model == "SMC":
        # softer + smaller dt stabilizes penalty contacts
        if dt is None:
            dt = 2e-4
    else:
        if dt is None:
            dt = 1e-3

    box, floor_top, y_expected = _build_scene(sys, model=model)
    y0 = box.GetPos().y

    steps = int(t_end / dt)
    for _ in range(steps):
        sys.DoStepDynamics(dt)
        p = box.GetPos()
        if any(map(math.isnan, (p.x, p.y, p.z))):
            raise AssertionError("NaN in position; unstable step")

    return y0, box.GetPos().y, y_expected


# ---------- Tests ----------
def test_drop_nsc_converges_height():
    y0, yf, yexp = _run_once("NSC", dt=1e-3, t_end=0.8)
    # It must fall and settle near floor_top + half_box (~0.15)
    assert yf < y0
    assert (yexp - 0.03) <= yf <= (yexp + 0.03)

def test_drop_smc_converges_height():
    y0, yf, yexp = _run_once("SMC", dt=2e-4, t_end=0.8)
    assert yf < y0
    # permit slightly wider band for SMC compliance
    assert (yexp - 0.04) <= yf <= (yexp + 0.04)
