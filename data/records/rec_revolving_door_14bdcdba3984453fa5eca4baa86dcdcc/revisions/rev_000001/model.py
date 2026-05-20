from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float):
    """CadQuery mesh for a centered hollow bearing ring."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _rot_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_three_wing_revolving_door")

    concrete = model.material("warm_concrete", rgba=(0.58, 0.56, 0.52, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.05, 0.055, 0.06, 1.0))
    brushed_metal = model.material("brushed_stainless", rgba=(0.66, 0.68, 0.67, 1.0))
    glass = model.material("pale_blue_glass", rgba=(0.55, 0.78, 0.95, 0.38))
    rubber = model.material("black_rubber_seal", rgba=(0.005, 0.005, 0.006, 1.0))

    corner = model.part("corner_junction")
    corner.visual(
        Box((3.30, 3.30, 0.10)),
        origin=Origin(xyz=(0.35, 0.35, -0.05)),
        material=concrete,
        name="floor_slab",
    )
    corner.visual(
        Cylinder(radius=0.055, length=2.75),
        origin=Origin(xyz=(0.0, 0.0, 1.375)),
        material=brushed_metal,
        name="central_post",
    )
    corner.visual(
        Cylinder(radius=0.17, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_metal,
        name="floor_pedestal",
    )
    corner.visual(
        Cylinder(radius=0.065, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_metal,
        name="floor_bearing",
    )
    corner.visual(
        Cylinder(radius=0.14, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.71)),
        material=dark_metal,
        name="ceiling_bearing",
    )

    # Perpendicular fixed facade returns show the door set into a building corner.
    corner.visual(
        Box((1.42, 0.026, 2.18)),
        origin=Origin(xyz=(1.82, 0.0, 1.17)),
        material=glass,
        name="facade_x_glass",
    )
    corner.visual(
        Box((0.026, 1.42, 2.18)),
        origin=Origin(xyz=(0.0, 1.82, 1.17)),
        material=glass,
        name="facade_y_glass",
    )
    corner.visual(
        Box((1.54, 0.085, 0.09)),
        origin=Origin(xyz=(1.82, 0.0, 0.045)),
        material=dark_metal,
        name="facade_x_sill",
    )
    corner.visual(
        Box((0.085, 1.54, 0.09)),
        origin=Origin(xyz=(0.0, 1.82, 0.045)),
        material=dark_metal,
        name="facade_y_sill",
    )
    corner.visual(
        Box((1.55, 0.18, 0.14)),
        origin=Origin(xyz=(1.78, 0.0, 2.64)),
        material=dark_metal,
        name="facade_x_header",
    )
    corner.visual(
        Box((0.18, 1.55, 0.14)),
        origin=Origin(xyz=(0.0, 1.78, 2.64)),
        material=dark_metal,
        name="facade_y_header",
    )
    for x in (1.09, 2.55):
        corner.visual(
            Box((0.07, 0.075, 2.64)),
            origin=Origin(xyz=(x, 0.0, 1.32)),
            material=dark_metal,
            name=f"facade_x_mullion_{int(round(x * 100))}",
        )
    for y in (1.09, 2.55):
        corner.visual(
            Box((0.075, 0.07, 2.64)),
            origin=Origin(xyz=(0.0, y, 1.32)),
            material=dark_metal,
            name=f"facade_y_mullion_{int(round(y * 100))}",
        )

    wings = model.part("wing_carrier")
    wings.visual(
        mesh_from_cadquery(_annular_cylinder(0.155, 0.074, 0.12), "bottom_hub_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=brushed_metal,
        name="bottom_hub_ring",
    )
    wings.visual(
        mesh_from_cadquery(_annular_cylinder(0.155, 0.074, 0.12), "top_hub_ring"),
        origin=Origin(xyz=(0.0, 0.0, 2.45)),
        material=brushed_metal,
        name="top_hub_ring",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        yaw = angle

        def wing_origin(radial_center: float, z_center: float) -> Origin:
            x, y = _rot_xy(radial_center, angle)
            return Origin(xyz=(x, y, z_center), rpy=(0.0, 0.0, yaw))

        wings.visual(
            Box((0.070, 0.050, 2.38)),
            origin=wing_origin(0.155, 1.275),
            material=brushed_metal,
            name=f"inner_stile_{index}",
        )
        wings.visual(
            Box((0.050, 0.050, 2.20)),
            origin=wing_origin(0.980, 1.275),
            material=brushed_metal,
            name=f"outer_stile_{index}",
        )
        wings.visual(
            Box((0.820, 0.044, 0.050)),
            origin=wing_origin(0.580, 2.335),
            material=brushed_metal,
            name=f"top_rail_{index}",
        )
        wings.visual(
            Box((0.820, 0.044, 0.050)),
            origin=wing_origin(0.580, 0.215),
            material=brushed_metal,
            name=f"bottom_rail_{index}",
        )
        wings.visual(
            Box((0.790, 0.018, 2.04)),
            origin=wing_origin(0.575, 1.275),
            material=glass,
            name=f"glass_wing_{index}",
        )
        wings.visual(
            Box((0.770, 0.030, 0.036)),
            origin=wing_origin(0.575, 0.245),
            material=rubber,
            name=f"lower_sweep_seal_{index}",
        )

    model.articulation(
        "post_to_wings",
        ArticulationType.CONTINUOUS,
        parent=corner,
        child=wings,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    corner = object_model.get_part("corner_junction")
    wings = object_model.get_part("wing_carrier")
    spin = object_model.get_articulation("post_to_wings")

    ctx.check(
        "door uses a continuous vertical spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "three distinct panel wings are modeled",
        len([v for v in wings.visuals if v.name and v.name.startswith("glass_wing_")]) == 3,
        details=[v.name for v in wings.visuals],
    )
    ctx.expect_origin_distance(
        wings,
        corner,
        axes="xy",
        max_dist=0.001,
        name="wing carrier axis is coincident with the corner post",
    )
    ctx.expect_overlap(
        wings,
        corner,
        axes="xy",
        elem_a="bottom_hub_ring",
        elem_b="central_post",
        min_overlap=0.05,
        name="bottom bearing is concentric around the central post",
    )

    def element_center(elem_name: str):
        aabb = ctx.part_element_world_aabb(wings, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    rest_center = element_center("glass_wing_0")
    with ctx.pose({spin: 2.0 * math.pi / 3.0}):
        rotated_center = element_center("glass_wing_0")

    ctx.check(
        "panel wing rotates one third turn around the post",
        rest_center is not None
        and rotated_center is not None
        and rest_center[0] > 0.45
        and abs(rest_center[1]) < 0.05
        and rotated_center[0] < -0.20
        and rotated_center[1] > 0.35,
        details=f"rest={rest_center}, rotated={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()
