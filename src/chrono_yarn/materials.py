"""Contact material factory helpers for PyChrono systems.

This module centralizes NSC/SMC material creation so scene builders can remain
focused on geometry and composition.
"""

from __future__ import annotations

import pychrono as chrono

from .config import ContactModel


def make_nsc_material(mu: float = 0.3, e: float = 0.05) -> chrono.ChMaterialSurfaceNSC:
    """Create an NSC surface material with common friction/restitution settings."""
    m = chrono.ChMaterialSurfaceNSC()
    if hasattr(m, "SetFriction"):
        m.SetFriction(mu)
    if hasattr(m, "SetRestitution"):
        m.SetRestitution(e)
    return m


def make_smc_material(
    mu: float = 0.3,
    e: float = 0.05,
    young: float = 5e6,
    nu: float = 0.3,
) -> chrono.ChMaterialSurfaceSMC:
    """Create an SMC surface material with basic elastic properties."""
    m = chrono.ChMaterialSurfaceSMC()
    if hasattr(m, "SetFriction"):
        m.SetFriction(mu)
    if hasattr(m, "SetRestitution"):
        m.SetRestitution(e)
    if hasattr(m, "SetYoungModulus"):
        m.SetYoungModulus(young)
    if hasattr(m, "SetPoissonRatio"):
        m.SetPoissonRatio(nu)
    return m


def make_contact_material(
    model: ContactModel,
    friction: float = 0.3,
    restitution: float = 0.05,
):
    """Create a contact material matching the selected contact model."""
    if model == "NSC":
        return make_nsc_material(mu=friction, e=restitution)
    if model == "SMC":
        return make_smc_material(mu=friction, e=restitution)
    raise ValueError(f"Unsupported contact model: {model}")
