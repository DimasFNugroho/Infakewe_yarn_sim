"""Visualization helpers for FEA cable/yarn examples."""

from __future__ import annotations

import pychrono as chrono

from .config import FEAVisualizationConfig


def attach_fea_cable_visuals(mesh, vis_cfg: FEAVisualizationConfig) -> list:
    """Attach default FEA visual shapes to a mesh and return the created shapes."""
    shapes = []

    # Beam/cable line visualization.
    beam_vis = chrono.ChVisualShapeFEA(mesh)
    beam_vis.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    beam_vis.SetWireframe(bool(vis_cfg.wireframe))
    if hasattr(beam_vis, "SetBeamResolution"):
        beam_vis.SetBeamResolution(int(vis_cfg.beam_resolution))
    if hasattr(beam_vis, "SetBeamResolutionSection"):
        beam_vis.SetBeamResolutionSection(int(vis_cfg.beam_section_resolution))
    mesh.AddVisualShapeFEA(beam_vis)
    shapes.append(beam_vis)

    if vis_cfg.draw_node_glyphs:
        node_vis = chrono.ChVisualShapeFEA(mesh)
        node_vis.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
        node_vis.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
        node_vis.SetSymbolsScale(float(vis_cfg.node_glyph_scale))
        node_vis.SetSymbolsThickness(float(vis_cfg.node_glyph_thickness))
        mesh.AddVisualShapeFEA(node_vis)
        shapes.append(node_vis)

    return shapes
