"""Simple metrics for FEA cable/yarn examples."""

from __future__ import annotations


def cable_tip_position(cable) -> tuple[float, float, float]:
    """Return the free-end node position as a tuple."""
    node = cable.last_node
    p = node.GetPos()
    return (float(p.x), float(p.y), float(p.z))


def cable_max_sag(cable, reference_y: float) -> float:
    """Return the maximum downward sag from `reference_y` across all cable nodes."""
    max_drop = 0.0
    for node in cable.nodes:
        y = float(node.GetPos().y)
        max_drop = max(max_drop, reference_y - y)
    return max_drop

