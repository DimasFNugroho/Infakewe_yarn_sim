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


def attach_fea_strain_overlay(
    mesh,
    *,
    data_type=chrono.ChVisualShapeFEA.DataType_ANCF_BEAM_AX,
    min_max: tuple[float, float] = (-0.02, 0.02),
    smooth_faces: bool = True,
) -> object:
    """Attach a scalar overlay (strain/stress-like) on top of cable visuals."""
    overlay = chrono.ChVisualShapeFEA(mesh)
    overlay.SetFEMdataType(data_type)
    overlay.SetWireframe(False)
    if hasattr(overlay, "SetBeamResolution"):
        overlay.SetBeamResolution(8)
    if hasattr(overlay, "SetBeamResolutionSection"):
        overlay.SetBeamResolutionSection(6)
    if hasattr(overlay, "SetSmoothFaces"):
        overlay.SetSmoothFaces(bool(smooth_faces))
    if hasattr(overlay, "SetColorscaleMinMax"):
        overlay.SetColorscaleMinMax(float(min_max[0]), float(min_max[1]))
    mesh.AddVisualShapeFEA(overlay)
    return overlay
