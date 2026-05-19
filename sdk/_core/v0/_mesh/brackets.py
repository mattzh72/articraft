from __future__ import annotations

from typing import Sequence

from sdk._dependencies import require_cadquery

from .cadquery_helpers import _mesh_geometry_from_cadquery_model
from .primitives import MeshGeometry, _adopt_mesh_geometry, _mesh_geometry_shifted_to_z0


class ClevisBracketGeometry(MeshGeometry):
    """
    Build a U-shaped clevis bracket with a bottom base and a transverse pin bore.
    """

    def __init__(
        self,
        overall_size: Sequence[float],
        *,
        gap_width: float,
        bore_diameter: float,
        bore_center_z: float,
        base_thickness: float,
        corner_radius: float = 0.0,
        center: bool = True,
    ):
        super().__init__()
        width = float(overall_size[0])
        depth = float(overall_size[1])
        height = float(overall_size[2])
        gap_width = float(gap_width)
        bore_diameter = float(bore_diameter)
        bore_center_z = float(bore_center_z)
        base_thickness = float(base_thickness)
        corner_radius = max(0.0, float(corner_radius))

        if width <= 0.0 or depth <= 0.0 or height <= 0.0:
            raise ValueError("overall_size values must be positive")
        if gap_width <= 0.0 or gap_width >= width:
            raise ValueError("gap_width must be positive and less than overall_size[0]")
        cheek_thickness = 0.5 * (width - gap_width)
        if cheek_thickness <= 1e-6:
            raise ValueError("gap_width leaves no side wall material")
        if bore_diameter <= 0.0 or bore_diameter >= min(cheek_thickness * 2.0, depth, height):
            raise ValueError("bore_diameter is too large for the clevis envelope")
        if base_thickness <= 0.0 or base_thickness >= height:
            raise ValueError("base_thickness must be positive and less than overall_size[2]")
        bore_radius = bore_diameter * 0.5
        if bore_center_z - bore_radius <= base_thickness or bore_center_z + bore_radius >= height:
            raise ValueError(
                "bore_center_z must leave material above the base and below the top edge"
            )

        cq = require_cadquery(feature="ClevisBracketGeometry")
        shape = cq.Workplane("XY").box(width, depth, height)
        slot_cut = (
            cq.Workplane("XY")
            .box(gap_width, depth + 0.004, height - base_thickness)
            .translate((0.0, 0.0, base_thickness * 0.5))
        )
        shape = shape.cut(slot_cut)
        if corner_radius > 0.0:
            shape = shape.edges("|Z").fillet(
                min(corner_radius, cheek_thickness * 0.6, depth * 0.25, height * 0.25)
            )

        bore_z = -height * 0.5 + bore_center_z
        bore = (
            cq.Workplane("YZ")
            .circle(bore_radius)
            .extrude(width + 0.01, both=True)
            .translate((0.0, 0.0, bore_z))
        )
        shape = shape.cut(bore)

        geom = _mesh_geometry_from_cadquery_model(shape)
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)


class PivotForkGeometry(MeshGeometry):
    """
    Build an open-front pivot fork with a rear bridge and a transverse pin bore.
    """

    def __init__(
        self,
        overall_size: Sequence[float],
        *,
        gap_width: float,
        bore_diameter: float,
        bore_center_z: float,
        bridge_thickness: float,
        corner_radius: float = 0.0,
        center: bool = True,
    ):
        super().__init__()
        width = float(overall_size[0])
        depth = float(overall_size[1])
        height = float(overall_size[2])
        gap_width = float(gap_width)
        bore_diameter = float(bore_diameter)
        bore_center_z = float(bore_center_z)
        bridge_thickness = float(bridge_thickness)
        corner_radius = max(0.0, float(corner_radius))

        if width <= 0.0 or depth <= 0.0 or height <= 0.0:
            raise ValueError("overall_size values must be positive")
        if gap_width <= 0.0 or gap_width >= width:
            raise ValueError("gap_width must be positive and less than overall_size[0]")
        cheek_thickness = 0.5 * (width - gap_width)
        if cheek_thickness <= 1e-6:
            raise ValueError("gap_width leaves no side wall material")
        if bridge_thickness <= 0.0 or bridge_thickness >= depth:
            raise ValueError("bridge_thickness must be positive and less than overall_size[1]")
        if bore_diameter <= 0.0 or bore_diameter >= min(cheek_thickness * 2.0, depth, height):
            raise ValueError("bore_diameter is too large for the pivot fork envelope")
        bore_radius = bore_diameter * 0.5
        if bore_center_z - bore_radius <= 0.0 or bore_center_z + bore_radius >= height:
            raise ValueError("bore_center_z must keep the bore inside the fork cheeks")

        cq = require_cadquery(feature="PivotForkGeometry")
        tine_depth = depth
        left_tine = (
            cq.Workplane("XY")
            .box(cheek_thickness, tine_depth, height)
            .translate((-(gap_width * 0.5 + cheek_thickness * 0.5), 0.0, 0.0))
        )
        right_tine = (
            cq.Workplane("XY")
            .box(cheek_thickness, tine_depth, height)
            .translate(((gap_width * 0.5 + cheek_thickness * 0.5), 0.0, 0.0))
        )
        rear_bridge = (
            cq.Workplane("XY")
            .box(width, bridge_thickness, height)
            .translate((0.0, -depth * 0.5 + bridge_thickness * 0.5, 0.0))
        )
        shape = left_tine.union(right_tine).union(rear_bridge)
        if corner_radius > 0.0:
            shape = shape.edges("|Z").fillet(
                min(corner_radius, cheek_thickness * 0.6, bridge_thickness * 0.6, height * 0.25)
            )

        bore_z = -height * 0.5 + bore_center_z
        bore = (
            cq.Workplane("YZ")
            .circle(bore_radius)
            .extrude(width + 0.01, both=True)
            .translate((0.0, 0.0, bore_z))
        )
        shape = shape.cut(bore)

        geom = _mesh_geometry_from_cadquery_model(shape)
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)


class TrunnionYokeGeometry(MeshGeometry):
    """
    Build a trunnion support yoke with a bottom base and cheek-mounted trunnion bores.
    """

    def __init__(
        self,
        overall_size: Sequence[float],
        *,
        span_width: float,
        trunnion_diameter: float,
        trunnion_center_z: float,
        base_thickness: float,
        corner_radius: float = 0.0,
        center: bool = True,
    ):
        super().__init__()
        width = float(overall_size[0])
        depth = float(overall_size[1])
        height = float(overall_size[2])
        span_width = float(span_width)
        trunnion_diameter = float(trunnion_diameter)
        trunnion_center_z = float(trunnion_center_z)
        base_thickness = float(base_thickness)
        corner_radius = max(0.0, float(corner_radius))

        if width <= 0.0 or depth <= 0.0 or height <= 0.0:
            raise ValueError("overall_size values must be positive")
        if span_width <= 0.0 or span_width >= width:
            raise ValueError("span_width must be positive and less than overall_size[0]")
        cheek_thickness = 0.5 * (width - span_width)
        if cheek_thickness <= 1e-6:
            raise ValueError("span_width leaves no side wall material")
        if base_thickness <= 0.0 or base_thickness >= height:
            raise ValueError("base_thickness must be positive and less than overall_size[2]")
        if trunnion_diameter <= 0.0 or trunnion_diameter >= min(
            cheek_thickness * 2.0, depth, height
        ):
            raise ValueError("trunnion_diameter is too large for the yoke envelope")
        trunnion_radius = trunnion_diameter * 0.5
        if (
            trunnion_center_z - trunnion_radius <= base_thickness
            or trunnion_center_z + trunnion_radius >= height
        ):
            raise ValueError(
                "trunnion_center_z must leave material above the base and below the top edge"
            )

        cq = require_cadquery(feature="TrunnionYokeGeometry")
        base = (
            cq.Workplane("XY")
            .box(width, depth, base_thickness)
            .translate((0.0, 0.0, -height * 0.5 + base_thickness * 0.5))
        )
        cheek_height = height - base_thickness
        cheek_z = -height * 0.5 + base_thickness + cheek_height * 0.5
        left_cheek = (
            cq.Workplane("XY")
            .box(cheek_thickness, depth, cheek_height)
            .translate((-(span_width * 0.5 + cheek_thickness * 0.5), 0.0, cheek_z))
        )
        right_cheek = (
            cq.Workplane("XY")
            .box(cheek_thickness, depth, cheek_height)
            .translate(((span_width * 0.5 + cheek_thickness * 0.5), 0.0, cheek_z))
        )
        boss_radius = max(trunnion_radius * 1.4, cheek_thickness * 0.55)
        boss_length = min(cheek_thickness * 0.75, depth * 0.35)
        boss_z = -height * 0.5 + trunnion_center_z
        left_boss = (
            cq.Workplane("YZ")
            .circle(boss_radius)
            .extrude(boss_length, both=False)
            .translate((-(span_width * 0.5 + cheek_thickness), 0.0, boss_z))
        )
        right_boss = (
            cq.Workplane("YZ")
            .circle(boss_radius)
            .extrude(-boss_length, both=False)
            .translate(((span_width * 0.5 + cheek_thickness), 0.0, boss_z))
        )
        shape = base.union(left_cheek).union(right_cheek).union(left_boss).union(right_boss)
        if corner_radius > 0.0:
            shape = shape.edges("|Z").fillet(
                min(corner_radius, cheek_thickness * 0.5, depth * 0.2, height * 0.2)
            )

        trunnion_bore = (
            cq.Workplane("YZ")
            .circle(trunnion_radius)
            .extrude(
                width + boss_length * 2.0 + 0.01,
                both=True,
            )
            .translate((0.0, 0.0, boss_z))
        )
        shape = shape.cut(trunnion_bore)

        geom = _mesh_geometry_from_cadquery_model(shape)
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)
