# tests/test_sim_basic.py
import math
import pychrono as chrono
from scripts.common.compat import set_gravity, prefer_bullet

def _build_scene(sys):
    # Floor: 1x0.02x1 at y=0 (top ≈ +0.01)
    floor = chrono.ChBodyEasyBox(1.0, 0.02, 1.0, 1000.0, True, True)
    floor.SetBodyFixed(True)
    floor.SetPos(chrono.ChVectorD(0, 0, 0))
    sys.Add(floor)

    # Box: 0.1 cube starting at y=0.5
    box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 500.0, True, True)
    box.SetPos(chrono.ChVectorD(0, 0.5, 0))
    sys.Add(box)
    return box

def _run_once(sys_kind="NSC", t_end=0.6, dt=1e-3):
    sys = chrono.ChSystemNSC() if sys_kind == "NSC" else chrono.ChSystemSMC()
    prefer_bullet(sys)
    set_gravity(sys, chrono.ChVectorD(0, -9.81, 0))
    box = _build_scene(sys)

    y0 = box.GetPos().y
    while sys.GetChTime() < t_end:
        sys.DoStepDynamics(dt)
    yf = box.GetPos().y
    return y0, yf

def test_drop_nsc_converges_height():
    y0, yf = _run_once("NSC")
    # should fall (yf << y0) and settle above floor top + half box = 0.01 + 0.05 ≈ 0.06
    assert yf < y0
    assert 0.04 <= yf <= 0.08   # broad tolerance across platforms

def test_drop_smc_converges_height():
    y0, yf = _run_once("SMC")
    assert yf < y0
    assert 0.04 <= yf <= 0.08
