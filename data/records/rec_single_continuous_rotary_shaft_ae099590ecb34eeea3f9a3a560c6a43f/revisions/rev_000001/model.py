from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SHAFT_Z = 0.20
SUPPORT_XS = (-0.32, 0.32)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _pillow_profile() -> list[tuple[float, float]]:
    """Side profile in local (horizontal, vertical) coordinates around the shaft."""

    half_width = 0.135
    bottom = -0.172
    shoulder = 0.030
    radius = half_width
    pts: list[tuple[float, float]] = [(-half_width, bottom), (half_width, bottom), (half_width, shoulder)]
    for i in range(1, 17):
        angle = math.pi * i / 16.0
        pts.append((radius * math.cos(angle), shoulder + radius * math.sin(angle)))
    return pts


def _oriented_extrusion(geometry, *, x: float, z: float):
    """Map an XY profile extruded along local Z into a support sleeve along world X."""

    return geometry.rotate_y(math.pi / 2.0).rotate_x(math.pi / 2.0).translate(x, 0.0, z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_rotary_shaft_module")

    cast_iron = Material("painted_cast_iron", rgba=(0.08, 0.18, 0.24, 1.0))
    dark_edge = Material("dark_machined_edges", rgba=(0.03, 0.035, 0.04, 1.0))
    brass = Material("brass_bearing", rgba=(0.76, 0.58, 0.26, 1.0))
    shaft_steel = Material("brushed_shaft_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    flange_steel = Material("darkened_flange_steel", rgba=(0.28, 0.30, 0.32, 1.0))

    bearing_block = model.part("bearing_block")
    bearing_block.visual(
        Box((0.86, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="base_plate",
    )
    bearing_block.visual(
        Box((0.76, 0.045, 0.030)),
        origin=Origin(xyz=(0.0, 0.175, 0.055)),
        material=dark_edge,
        name="front_foot_rail",
    )
    bearing_block.visual(
        Box((0.76, 0.045, 0.030)),
        origin=Origin(xyz=(0.0, -0.175, 0.055)),
        material=dark_edge,
        name="rear_foot_rail",
    )

    support_profile = _pillow_profile()
    bearing_hole = _circle_profile(0.061, 64)
    sleeve_outer = _circle_profile(0.054, 64)
    sleeve_inner = _circle_profile(0.033, 64)
    support_0 = ExtrudeWithHolesGeometry(
        support_profile,
        [bearing_hole],
        0.095,
        cap=True,
        center=True,
    )
    bearing_block.visual(
        mesh_from_geometry(_oriented_extrusion(support_0, x=SUPPORT_XS[0], z=SHAFT_Z), "end_support_0"),
        material=cast_iron,
        name="end_support_0",
    )

    sleeve_0 = ExtrudeWithHolesGeometry(
        sleeve_outer,
        [sleeve_inner],
        0.112,
        cap=True,
        center=True,
    )
    bearing_block.visual(
        mesh_from_geometry(_oriented_extrusion(sleeve_0, x=SUPPORT_XS[0], z=SHAFT_Z), "bearing_sleeve_0"),
        material=brass,
        name="bearing_sleeve_0",
    )

    support_1 = ExtrudeWithHolesGeometry(
        support_profile,
        [bearing_hole],
        0.095,
        cap=True,
        center=True,
    )
    bearing_block.visual(
        mesh_from_geometry(_oriented_extrusion(support_1, x=SUPPORT_XS[1], z=SHAFT_Z), "end_support_1"),
        material=cast_iron,
        name="end_support_1",
    )

    sleeve_1 = ExtrudeWithHolesGeometry(
        sleeve_outer,
        [sleeve_inner],
        0.112,
        cap=True,
        center=True,
    )
    bearing_block.visual(
        mesh_from_geometry(_oriented_extrusion(sleeve_1, x=SUPPORT_XS[1], z=SHAFT_Z), "bearing_sleeve_1"),
        material=brass,
        name="bearing_sleeve_1",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.027, length=0.92),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shaft_steel,
        name="shaft_bar",
    )
    shaft.visual(
        Cylinder(radius=0.052, length=0.160),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="center_hub",
    )
    shaft.visual(
        Cylinder(radius=0.095, length=0.035),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="flange_disk",
    )
    shaft.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(-0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="hub_collar",
    )
    for i in range(6):
        angle = 2.0 * math.pi * i / 6.0
        y = 0.068 * math.cos(angle)
        z = 0.068 * math.sin(angle)
        shaft.visual(
            Cylinder(radius=0.0075, length=0.014),
            origin=Origin(xyz=(0.129, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=shaft_steel,
            name=f"flange_bolt_{i}",
        )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=bearing_block,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    block = object_model.get_part("bearing_block")
    shaft = object_model.get_part("shaft")
    spin = object_model.get_articulation("shaft_spin")

    ctx.allow_overlap(
        block,
        shaft,
        elem_a="bearing_sleeve_0",
        elem_b="shaft_bar",
        reason="The shaft is intentionally captured inside the bearing sleeve bore.",
    )
    ctx.allow_overlap(
        block,
        shaft,
        elem_a="bearing_sleeve_1",
        elem_b="shaft_bar",
        reason="The shaft is intentionally captured inside the bearing sleeve bore.",
    )
    ctx.allow_overlap(
        block,
        shaft,
        elem_a="end_support_0",
        elem_b="shaft_bar",
        reason="The shaft intentionally passes through the bored end support.",
    )
    ctx.allow_overlap(
        block,
        shaft,
        elem_a="end_support_1",
        elem_b="shaft_bar",
        reason="The shaft intentionally passes through the bored end support.",
    )

    ctx.expect_within(
        shaft,
        block,
        axes="yz",
        inner_elem="shaft_bar",
        outer_elem="bearing_sleeve_0",
        margin=0.002,
        name="shaft is centered in first bearing sleeve",
    )
    ctx.expect_within(
        shaft,
        block,
        axes="yz",
        inner_elem="shaft_bar",
        outer_elem="bearing_sleeve_1",
        margin=0.002,
        name="shaft is centered in second bearing sleeve",
    )
    ctx.expect_overlap(
        shaft,
        block,
        axes="x",
        elem_a="shaft_bar",
        elem_b="bearing_sleeve_0",
        min_overlap=0.08,
        name="shaft passes through first support",
    )
    ctx.expect_overlap(
        shaft,
        block,
        axes="x",
        elem_a="shaft_bar",
        elem_b="bearing_sleeve_1",
        min_overlap=0.08,
        name="shaft passes through second support",
    )
    ctx.expect_within(
        shaft,
        block,
        axes="yz",
        inner_elem="shaft_bar",
        outer_elem="end_support_0",
        margin=0.002,
        name="shaft is centered in first support bore",
    )
    ctx.expect_within(
        shaft,
        block,
        axes="yz",
        inner_elem="shaft_bar",
        outer_elem="end_support_1",
        margin=0.002,
        name="shaft is centered in second support bore",
    )
    ctx.expect_overlap(
        shaft,
        block,
        axes="x",
        elem_a="shaft_bar",
        elem_b="end_support_0",
        min_overlap=0.07,
        name="shaft traverses first bored support",
    )
    ctx.expect_overlap(
        shaft,
        block,
        axes="x",
        elem_a="shaft_bar",
        elem_b="end_support_1",
        min_overlap=0.07,
        name="shaft traverses second bored support",
    )

    rest_aabb = ctx.part_world_aabb(shaft)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(shaft)

    ctx.check(
        "continuous joint keeps shaft on its own axis",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(rest_aabb[0][0] - turned_aabb[0][0]) < 0.002
        and abs(rest_aabb[1][0] - turned_aabb[1][0]) < 0.002,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
