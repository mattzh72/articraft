from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_plate(length: float, width: float, thickness: float, corner_radius: float):
    """A simple milled plate with rounded vertical corners, centered on its frame."""
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(corner_radius)
    )


def _add_bolt_heads(
    part,
    points: tuple[tuple[float, float], ...],
    *,
    z_top: float,
    radius: float,
    height: float,
    material: Material,
    prefix: str,
) -> None:
    for index, (x, y) in enumerate(points):
        part.visual(
            Cylinder(radius=radius, length=height),
            # A tiny embed keeps the screw heads visually seated in the plate.
            origin=Origin(xyz=(x, y, z_top + height * 0.5 - 0.0004)),
            material=material,
            name=f"{prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_cartesian_stage")

    aluminum = Material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_aluminum = Material("hard_anodized_aluminum", rgba=(0.05, 0.055, 0.06, 1.0))
    blue_aluminum = Material("blue_anodized_aluminum", rgba=(0.12, 0.24, 0.55, 1.0))
    bearing_black = Material("black_bearing_blocks", rgba=(0.015, 0.016, 0.018, 1.0))
    steel = Material("polished_steel", rgba=(0.78, 0.80, 0.80, 1.0))
    rubber = Material("rubber_feet", rgba=(0.01, 0.01, 0.01, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_plate(0.42, 0.28, 0.025, 0.014), "base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=aluminum,
        name="base_plate",
    )

    for index, (x, y) in enumerate(
        ((-0.175, -0.105), (-0.175, 0.105), (0.175, -0.105), (0.175, 0.105))
    ):
        base.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(x, y, -0.0046)),
            material=rubber,
            name=f"foot_{index}",
        )

    _add_bolt_heads(
        base,
        ((-0.175, -0.105), (-0.175, 0.105), (0.175, -0.105), (0.175, 0.105)),
        z_top=0.025,
        radius=0.0065,
        height=0.004,
        material=dark_aluminum,
        prefix="base_bolt",
    )

    x_rail_radius = 0.010
    x_rail_z = 0.061
    for rail_index, y in enumerate((-0.090, 0.090)):
        for block_index, x in enumerate((-0.170, 0.170)):
            base.visual(
                Box((0.032, 0.046, 0.036)),
                origin=Origin(xyz=(x, y, 0.0430)),
                material=dark_aluminum,
                name=f"x_rail_pedestal_{rail_index}_{block_index}",
            )
        base.visual(
            Cylinder(radius=x_rail_radius, length=0.360),
            origin=Origin(xyz=(0.0, y, x_rail_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"x_rail_{rail_index}",
        )

    x_slide = model.part("x_slide")

    # Four U-shaped linear bearing housings straddle the lower X rails.
    x_block_xs = (-0.055, 0.055)
    for rail_index, y in enumerate((-0.090, 0.090)):
        for block_index, x in enumerate(x_block_xs):
            name_index = rail_index * 2 + block_index
            x_slide.visual(
                Box((0.062, 0.050, 0.018)),
                origin=Origin(xyz=(x, y, 0.0800)),
                material=bearing_black,
                name=f"x_bearing_cap_{name_index}",
            )
            for cheek_index, sign in enumerate((-1.0, 1.0)):
                x_slide.visual(
                    Box((0.062, 0.008, 0.032)),
                    origin=Origin(xyz=(x, y + sign * 0.017, 0.0600)),
                    material=bearing_black,
                    name=f"x_bearing_cheek_{name_index}_{cheek_index}",
                )

    x_slide.visual(
        mesh_from_cadquery(_rounded_plate(0.270, 0.200, 0.018, 0.010), "x_carriage_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0980)),
        material=blue_aluminum,
        name="x_carriage_plate",
    )
    _add_bolt_heads(
        x_slide,
        ((-0.105, -0.070), (-0.105, 0.070), (0.105, -0.070), (0.105, 0.070)),
        z_top=0.107,
        radius=0.005,
        height=0.0035,
        material=dark_aluminum,
        prefix="carriage_bolt",
    )

    y_rail_radius = 0.008
    y_rail_z = 0.126
    for rail_index, x in enumerate((-0.070, 0.070)):
        for block_index, y in enumerate((-0.095, 0.095)):
            x_slide.visual(
                Box((0.036, 0.020, 0.026)),
                origin=Origin(xyz=(x, y, 0.1200)),
                material=dark_aluminum,
                name=f"y_rail_pedestal_{rail_index}_{block_index}",
            )
        x_slide.visual(
            Cylinder(radius=y_rail_radius, length=0.210),
            origin=Origin(xyz=(x, 0.0, y_rail_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"y_rail_{rail_index}",
        )

    y_slide = model.part("y_slide")

    # Four smaller bearing blocks ride on the Y rails and carry the top plate.
    for rail_index, x in enumerate((-0.070, 0.070)):
        for block_index, y in enumerate((-0.030, 0.030)):
            name_index = rail_index * 2 + block_index
            y_slide.visual(
                Box((0.046, 0.034, 0.016)),
                origin=Origin(xyz=(x, y, 0.1420)),
                material=bearing_black,
                name=f"y_bearing_cap_{name_index}",
            )
            for cheek_index, sign in enumerate((-1.0, 1.0)):
                y_slide.visual(
                    Box((0.007, 0.034, 0.027)),
                    origin=Origin(xyz=(x + sign * 0.0145, y, 0.1230)),
                    material=bearing_black,
                    name=f"y_bearing_cheek_{name_index}_{cheek_index}",
                )

    y_slide.visual(
        mesh_from_cadquery(_rounded_plate(0.220, 0.160, 0.018, 0.009), "top_stage_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.1590)),
        material=aluminum,
        name="top_stage_plate",
    )
    _add_bolt_heads(
        y_slide,
        ((-0.085, -0.055), (-0.085, 0.055), (0.085, -0.055), (0.085, 0.055)),
        z_top=0.168,
        radius=0.0047,
        height=0.0035,
        material=dark_aluminum,
        prefix="top_bolt",
    )

    model.articulation(
        "base_to_x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_slide,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=-0.055, upper=0.055),
    )
    model.articulation(
        "x_slide_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=x_slide,
        child=y_slide,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.22, lower=-0.035, upper=0.035),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_slide = object_model.get_part("x_slide")
    y_slide = object_model.get_part("y_slide")
    x_joint = object_model.get_articulation("base_to_x_slide")
    y_joint = object_model.get_articulation("x_slide_to_y_slide")

    ctx.expect_contact(
        x_slide,
        base,
        elem_a="x_bearing_cap_0",
        elem_b="x_rail_0",
        contact_tol=0.001,
        name="lower bearing block sits on X rail",
    )
    ctx.expect_contact(
        y_slide,
        x_slide,
        elem_a="y_bearing_cap_0",
        elem_b="y_rail_0",
        contact_tol=0.001,
        name="upper bearing block sits on Y rail",
    )
    ctx.expect_overlap(
        x_slide,
        base,
        axes="x",
        elem_a="x_bearing_cap_0",
        elem_b="x_rail_0",
        min_overlap=0.050,
        name="X bearing has retained rail length",
    )
    ctx.expect_overlap(
        y_slide,
        x_slide,
        axes="y",
        elem_a="y_bearing_cap_0",
        elem_b="y_rail_0",
        min_overlap=0.030,
        name="Y bearing has retained rail length",
    )

    x_rest = ctx.part_world_position(x_slide)
    y_rest = ctx.part_world_position(y_slide)
    with ctx.pose({x_joint: 0.055, y_joint: 0.035}):
        ctx.expect_overlap(
            x_slide,
            base,
            axes="x",
            elem_a="x_bearing_cap_1",
            elem_b="x_rail_0",
            min_overlap=0.050,
            name="X slide remains supported at full travel",
        )
        ctx.expect_overlap(
            y_slide,
            x_slide,
            axes="y",
            elem_a="y_bearing_cap_1",
            elem_b="y_rail_0",
            min_overlap=0.030,
            name="Y slide remains supported at full travel",
        )
        x_extended = ctx.part_world_position(x_slide)
        y_extended = ctx.part_world_position(y_slide)

    ctx.check(
        "X joint translates the lower carriage",
        x_rest is not None
        and x_extended is not None
        and x_extended[0] > x_rest[0] + 0.050,
        details=f"rest={x_rest}, extended={x_extended}",
    )
    ctx.check(
        "Y joint translates the top plate",
        y_rest is not None
        and y_extended is not None
        and y_extended[1] > y_rest[1] + 0.030,
        details=f"rest={y_rest}, extended={y_extended}",
    )

    return ctx.report()


object_model = build_object_model()
