# scripts/hello_world.py
# Minimal PyChrono step test (headless)

import pychrono as chrono

sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Floor (fixed)
floor = chrono.ChBodyEasyBox(1.0, 0.02, 1.0, 1000.0, True, True)
floor.SetBodyFixed(True)
sys.Add(floor)

# Falling box
box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000.0, True, True)
box.SetPos(chrono.ChVectorD(0, 0.5, 0))
sys.Add(box)

# Step a bit
t_end, dt = 0.2, 1e-3
while sys.GetChTime() < t_end:
    sys.DoStepDynamics(dt)

print(f"time={sys.GetChTime():.3f}s  box_y={box.GetPos().y:.6f}")

