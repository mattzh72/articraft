from __future__ import annotations

import math

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
)


def _add_rectangular_tube(
    part,
    *,
    length: float,
    width_y: float,
    height_z: float,
    wall: float,
    origin_x: float,
    material: Material,
    prefix: str,
) -> None:
    """Add four connected wall plates that read as an open rectangular tube."""

    side_x = origin_x
    side_y = width_y / 2.0 - wall / 2.0
    cap_z = height_z / 2.0 - wall / 2.0

    part.visual(
        Box((length, wall, height_z)),
        origin=Origin(xyz=(side_x, side_y, 0.0)),
        material=material,
        name=f"{prefix}_side_0",
    )
    part.visual(
        Box((length, wall, height_z)),
        origin=Origin(xyz=(side_x, -side_y, 0.0)),
        material=material,
        name=f"{prefix}_side_1",
    )
    part.visual(
        Box((length, width_y, wall)),
        origin=Origin(xyz=(side_x, 0.0, cap_z)),
        material=material,
        name=f"{prefix}_top",
    )
    part.visual(
        Box((length, width_y, wall)),
        origin=Origin(xyz=(side_x, 0.0, -cap_z)),
        material=material,
        name=f"{prefix}_bottom",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_rectangular_boom")

    black_powder = Material("black_powder_coat", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_trim = Material("dark_polymer_wear_pad", rgba=(0.015, 0.016, 0.018, 1.0))
    outer_aluminum = Material("brushed_aluminum", rgba=(0.62, 0.65, 0.66, 1.0))
    inner_aluminum = Material("light_anodized_aluminum", rgba=(0.76, 0.78, 0.77, 1.0))
    steel = Material("dark_steel_pin", rgba=(0.10, 0.105, 0.11, 1.0))

    housing = model.part("housing")
    _add_rectangular_tube(
        housing,
        length=0.70,
        width_y=0.170,
        height_z=0.120,
        wall=0.016,
        origin_x=-0.150,
        material=black_powder,
        prefix="sleeve",
    )
    _add_rectangular_tube(
        housing,
        length=0.045,
        width_y=0.195,
        height_z=0.145,
        wall=0.025,
        origin_x=0.212,
        material=black_powder,
        prefix="front_collar",
    )
    housing.visual(
        Box((0.68, 0.230, 0.030)),
        origin=Origin(xyz=(-0.155, 0.0, -0.0745)),
        material=black_powder,
        name="base_foot",
    )
    housing.visual(
        Box((0.060, 0.225, 0.012)),
        origin=Origin(xyz=(0.180, 0.0, -0.053)),
        material=dark_trim,
        name="front_wear_strip",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.180, 0.088, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="side_fastener_0",
    )
    housing.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.180, -0.088, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="side_fastener_1",
    )

    outer_stage = model.part("outer_stage")
    _add_rectangular_tube(
        outer_stage,
        length=0.85,
        width_y=0.112,
        height_z=0.072,
        wall=0.008,
        origin_x=-0.075,
        material=outer_aluminum,
        prefix="tube",
    )
    _add_rectangular_tube(
        outer_stage,
        length=0.040,
        width_y=0.132,
        height_z=0.092,
        wall=0.015,
        origin_x=0.355,
        material=outer_aluminum,
        prefix="front_collar",
    )
    outer_stage.visual(
        Box((0.065, 0.100, 0.006)),
        origin=Origin(xyz=(0.310, 0.0, 0.039)),
        material=dark_trim,
        name="top_wear_pad",
    )
    outer_stage.visual(
        Box((0.120, 0.080, 0.008)),
        origin=Origin(xyz=(-0.300, 0.0, 0.040)),
        material=dark_trim,
        name="rear_top_bearing_pad",
    )
    outer_stage.visual(
        Box((0.120, 0.080, 0.008)),
        origin=Origin(xyz=(-0.300, 0.0, -0.040)),
        material=dark_trim,
        name="rear_bottom_bearing_pad",
    )

    inner_stage = model.part("inner_stage")
    _add_rectangular_tube(
        inner_stage,
        length=0.85,
        width_y=0.078,
        height_z=0.044,
        wall=0.006,
        origin_x=-0.075,
        material=inner_aluminum,
        prefix="tube",
    )
    inner_stage.visual(
        Box((0.030, 0.086, 0.052)),
        origin=Origin(xyz=(0.362, 0.0, 0.0)),
        material=inner_aluminum,
        name="end_plug",
    )
    inner_stage.visual(
        Box((0.100, 0.060, 0.006)),
        origin=Origin(xyz=(-0.300, 0.0, 0.025)),
        material=dark_trim,
        name="rear_top_bearing_pad",
    )
    inner_stage.visual(
        Box((0.100, 0.060, 0.006)),
        origin=Origin(xyz=(-0.300, 0.0, -0.025)),
        material=dark_trim,
        name="rear_bottom_bearing_pad",
    )

    head_bracket = model.part("head_bracket")
    head_bracket.visual(
        Box((0.020, 0.090, 0.062)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=black_powder,
        name="mount_plate",
    )
    head_bracket.visual(
        Box((0.105, 0.014, 0.064)),
        origin=Origin(xyz=(0.072, 0.039, 0.0)),
        material=black_powder,
        name="cheek_0",
    )
    head_bracket.visual(
        Box((0.105, 0.014, 0.064)),
        origin=Origin(xyz=(0.072, -0.039, 0.0)),
        material=black_powder,
        name="cheek_1",
    )
    head_bracket.visual(
        Cylinder(radius=0.008, length=0.096),
        origin=Origin(xyz=(0.092, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cross_pin",
    )

    model.articulation(
        "housing_to_outer_stage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=outer_stage,
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=0.0, upper=0.250),
    )
    model.articulation(
        "outer_stage_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.250),
    )
    model.articulation(
        "inner_stage_to_head_bracket",
        ArticulationType.FIXED,
        parent=inner_stage,
        child=head_bracket,
        origin=Origin(xyz=(0.377, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    outer_stage = object_model.get_part("outer_stage")
    inner_stage = object_model.get_part("inner_stage")
    head_bracket = object_model.get_part("head_bracket")
    outer_slide = object_model.get_articulation("housing_to_outer_stage")
    inner_slide = object_model.get_articulation("outer_stage_to_inner_stage")

    ctx.check(
        "outer stage has 250 mm travel",
        outer_slide.motion_limits is not None
        and abs((outer_slide.motion_limits.upper or 0.0) - 0.250) < 1e-6,
    )
    ctx.check(
        "inner stage has 250 mm travel",
        inner_slide.motion_limits is not None
        and abs((inner_slide.motion_limits.upper or 0.0) - 0.250) < 1e-6,
    )

    ctx.expect_overlap(
        outer_stage,
        housing,
        axes="x",
        min_overlap=0.45,
        name="outer stage is deeply retained when collapsed",
    )
    ctx.expect_overlap(
        inner_stage,
        outer_stage,
        axes="x",
        min_overlap=0.45,
        name="inner stage is deeply retained when collapsed",
    )
    ctx.expect_contact(
        head_bracket,
        inner_stage,
        elem_a="mount_plate",
        elem_b="end_plug",
        contact_tol=0.002,
        name="head bracket mounts to inner stage nose",
    )

    rest_head = ctx.part_world_position(head_bracket)
    with ctx.pose({outer_slide: 0.250, inner_slide: 0.250}):
        ctx.expect_overlap(
            outer_stage,
            housing,
            axes="x",
            min_overlap=0.23,
            name="outer stage retains overlap at full travel",
        )
        ctx.expect_overlap(
            inner_stage,
            outer_stage,
            axes="x",
            min_overlap=0.23,
            name="inner stage retains overlap at full travel",
        )
        extended_head = ctx.part_world_position(head_bracket)

    ctx.check(
        "head bracket advances by combined travel",
        rest_head is not None
        and extended_head is not None
        and extended_head[0] > rest_head[0] + 0.49,
        details=f"rest={rest_head}, extended={extended_head}",
    )

    return ctx.report()


object_model = build_object_model()
