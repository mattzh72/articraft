from __future__ import annotations

from typing import Optional

from sdk._dependencies import require_cadquery

from .cadquery_helpers import (
    _centered_pattern_positions,
    _mesh_geometry_from_cadquery_model,
    _rounded_slot_profile,
)
from .primitives import MeshGeometry, _adopt_mesh_geometry, _mesh_geometry_shifted_to_z0
from .specs import HingeHolePattern, HingePinStyle


class BarrelHingeGeometry(MeshGeometry):
    """
    Build an exposed barrel hinge with two leaves around a local Z pin axis.
    """

    def __init__(
        self,
        length: float,
        *,
        leaf_width_a: float,
        leaf_width_b: Optional[float] = None,
        leaf_thickness: float,
        pin_diameter: float,
        knuckle_outer_diameter: Optional[float] = None,
        knuckle_count: int = 5,
        clearance: float = 0.0005,
        open_angle_deg: float = 180.0,
        holes_a: Optional[HingeHolePattern] = None,
        holes_b: Optional[HingeHolePattern] = None,
        pin: Optional[HingePinStyle] = None,
        center: bool = True,
    ):
        super().__init__()
        length = float(length)
        leaf_width_a = float(leaf_width_a)
        leaf_width_b = float(leaf_width_b) if leaf_width_b is not None else leaf_width_a
        leaf_thickness = float(leaf_thickness)
        pin_diameter = float(pin_diameter)
        knuckle_outer_diameter = (
            float(knuckle_outer_diameter)
            if knuckle_outer_diameter is not None
            else pin_diameter * 1.75
        )
        clearance = float(clearance)
        open_angle_deg = float(open_angle_deg)
        holes_a = holes_a or HingeHolePattern()
        holes_b = holes_b or HingeHolePattern()
        pin = pin or HingePinStyle()

        if (
            min(
                length,
                leaf_width_a,
                leaf_width_b,
                leaf_thickness,
                pin_diameter,
                knuckle_outer_diameter,
            )
            <= 0.0
        ):
            raise ValueError(
                "length, leaf widths, thickness, pin_diameter, and knuckle size must be positive"
            )
        if knuckle_count < 3:
            raise ValueError("knuckle_count must be at least 3")
        if pin_diameter >= knuckle_outer_diameter:
            raise ValueError("pin_diameter must be less than knuckle_outer_diameter")
        segment_length = (length - clearance * float(knuckle_count - 1)) / float(knuckle_count)
        if segment_length <= 0.0:
            raise ValueError("clearance/knuckle_count leave no knuckle length")

        cq = require_cadquery(feature="BarrelHingeGeometry")
        leaf_overlap = min(leaf_thickness * 0.75, knuckle_outer_diameter * 0.12)
        leaf_a = (
            cq.Workplane("XY")
            .box(leaf_width_a, leaf_thickness, length)
            .translate(
                (-(knuckle_outer_diameter * 0.5 + leaf_width_a * 0.5 - leaf_overlap), 0.0, 0.0)
            )
        )
        leaf_b = (
            cq.Workplane("XY")
            .box(leaf_width_b, leaf_thickness, length)
            .translate(
                ((knuckle_outer_diameter * 0.5 + leaf_width_b * 0.5 - leaf_overlap), 0.0, 0.0)
            )
        )
        leaf_b = leaf_b.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 180.0 - open_angle_deg)
        shape = leaf_a.union(leaf_b)

        z_start = -length * 0.5
        for index in range(knuckle_count):
            center_z = z_start + segment_length * 0.5 + index * (segment_length + clearance)
            knuckle = (
                cq.Workplane("XY")
                .circle(knuckle_outer_diameter * 0.5)
                .extrude(segment_length)
                .translate((0.0, 0.0, center_z - segment_length * 0.5))
            )
            shape = shape.union(knuckle)

        pin_len = length + pin.exposed_end * 2.0
        pin_bottom = -length * 0.5 - pin.exposed_end
        pin_shape = (
            cq.Workplane("XY")
            .circle(pin_diameter * 0.5)
            .extrude(pin_len)
            .translate((0.0, 0.0, pin_bottom))
        )
        if pin.head_style != "plain" and pin.head_height > 1e-6:
            head_d = pin.head_diameter or pin_diameter * 1.6
            head = cq.Workplane("XY").circle(head_d * 0.5).extrude(pin.head_height)
            pin_shape = pin_shape.union(head.translate((0.0, 0.0, pin_bottom - pin.head_height)))
            if pin.head_style != "peened":
                pin_shape = pin_shape.union(head.translate((0.0, 0.0, pin_bottom + pin_len)))
        shape = shape.union(pin_shape)

        def _apply_holes(base_shape, pattern: HingeHolePattern, side: float):
            if pattern.style == "none" or pattern.count <= 0:
                return base_shape
            if pattern.diameter is None and pattern.slot_size is None:
                raise ValueError("HingeHolePattern requires diameter or slot_size")
            count = pattern.count
            z_positions = _centered_pattern_positions(
                count,
                pattern.pitch
                if pattern.pitch is not None
                else (length - pattern.edge_margin * 2.0) / max(count - 1, 1),
            )
            x_center = side * (
                knuckle_outer_diameter * 0.5 + (leaf_width_a if side < 0.0 else leaf_width_b) * 0.5
            )
            for z_pos in z_positions:
                if pattern.style == "slotted":
                    if pattern.slot_size is None:
                        raise ValueError("Slotted hinge holes require slot_size")
                    slot_profile = _rounded_slot_profile(pattern.slot_size[0], pattern.slot_size[1])
                    cutter = (
                        cq.Workplane("XZ")
                        .polyline(
                            [(x_center + point[0], z_pos + point[1]) for point in slot_profile]
                        )
                        .close()
                        .extrude(leaf_thickness + 0.01, both=True)
                    )
                else:
                    hole_r = (pattern.diameter or 0.0) * 0.5
                    cutter = (
                        cq.Workplane("XZ")
                        .circle(hole_r)
                        .extrude(leaf_thickness + 0.01, both=True)
                        .translate((x_center, 0.0, z_pos))
                    )
                base_shape = base_shape.cut(cutter)
            return base_shape

        shape = _apply_holes(shape, holes_a, -1.0)
        shape = _apply_holes(shape, holes_b, 1.0)

        geom = _mesh_geometry_from_cadquery_model(shape)
        if not center:
            geom = _mesh_geometry_shifted_to_z0(geom)
        _adopt_mesh_geometry(self, geom)


class PianoHingeGeometry(MeshGeometry):
    """
    Build a continuous piano hinge strip around a local Z pin axis.
    """

    def __init__(
        self,
        length: float,
        *,
        leaf_width_a: float,
        leaf_width_b: Optional[float] = None,
        leaf_thickness: float,
        pin_diameter: float,
        knuckle_pitch: float,
        clearance: float = 0.0005,
        open_angle_deg: float = 180.0,
        holes_a: Optional[HingeHolePattern] = None,
        holes_b: Optional[HingeHolePattern] = None,
        pin: Optional[HingePinStyle] = None,
        center: bool = True,
    ):
        knuckle_pitch = float(knuckle_pitch)
        if knuckle_pitch <= 0.0:
            raise ValueError("knuckle_pitch must be positive")
        knuckle_count = max(3, int(length / knuckle_pitch))
        if knuckle_count % 2 == 0:
            knuckle_count += 1
        knuckle_outer_diameter = pin_diameter * 1.55
        base = BarrelHingeGeometry(
            length,
            leaf_width_a=leaf_width_a,
            leaf_width_b=leaf_width_b,
            leaf_thickness=leaf_thickness,
            pin_diameter=pin_diameter,
            knuckle_outer_diameter=knuckle_outer_diameter,
            knuckle_count=knuckle_count,
            clearance=clearance,
            open_angle_deg=open_angle_deg,
            holes_a=holes_a,
            holes_b=holes_b,
            pin=pin,
            center=center,
        )
        _adopt_mesh_geometry(self, base)
