# tests/test_install.py
import pychrono as chrono

def test_import_and_core_types():
    s_nsc = chrono.ChSystemNSC()
    s_smc = chrono.ChSystemSMC()
    assert s_nsc is not None
    assert s_smc is not None
