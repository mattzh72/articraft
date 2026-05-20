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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="optics_lift_stage")

    matte_black = model.material("matte_black_anodized", rgba=(0.015, 0.017, 0.018, 1.0))
    dark_gray = model.material("dark_gray_hardcoat", rgba=(0.09, 0.095, 0.10, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    rail_steel = model.material("polished_rail_steel", rgba=(0.86, 0.88, 0.84, 1.0))
    satin_blue = model.material("satin_blue_carriage", rgba=(0.05, 0.12, 0.20, 1.0))
    brass = model.material("brass_fasteners", rgba=(0.86, 0.62, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.22, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=matte_black,
        name="base_plate",
    )
    base.visual(
        Box((0.405, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=dark_gray,
        name="center_rail",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.420),
        origin=Origin(xyz=(0.0, -0.055, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="rail_0",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.420),
        origin=Origin(xyz=(0.0, 0.055, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="rail_1",
    )
    for index, x in enumerate((-0.215, 0.215)):
        base.visual(
            Box((0.030, 0.170, 0.056)),
            origin=Origin(xyz=(x, 0.0, 0.057)),
            material=dark_gray,
            name=f"end_block_{index}",
        )
    for index, x in enumerate((-0.150, -0.075, 0.0, 0.075, 0.150)):
        base.visual(
            Box((0.003, 0.030, 0.002)),
            origin=Origin(xyz=(x, 0.096, 0.031)),
            material=steel,
            name=f"travel_tick_{index}",
        )

    mast = model.part("mast")
    mast.visual(
        Box((0.094, 0.164, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=satin_blue,
        name="carriage_block",
    )
    mast.visual(
        Box((0.080, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, -0.055, -0.012)),
        material=steel,
        name="rail_shoe_0",
    )
    mast.visual(
        Box((0.080, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, 0.055, -0.012)),
        material=steel,
        name="rail_shoe_1",
    )
    mast.visual(
        Box((0.058, 0.046, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.231)),
        material=matte_black,
        name="upright_box",
    )
    for index, x in enumerate((-0.030, 0.030)):
        mast.visual(
            Box((0.016, 0.040, 0.085)),
            origin=Origin(xyz=(x, 0.0, 0.052)),
            material=satin_blue,
            name=f"base_gusset_{index}",
        )
    mast.visual(
        Box((0.010, 0.0065, 0.350)),
        origin=Origin(xyz=(-0.018, 0.02575, 0.250)),
        material=rail_steel,
        name="guide_rail_0",
    )
    mast.visual(
        Box((0.010, 0.0065, 0.350)),
        origin=Origin(xyz=(0.018, 0.02575, 0.250)),
        material=rail_steel,
        name="guide_rail_1",
    )
    mast.visual(
        Box((0.004, 0.002, 0.320)),
        origin=Origin(xyz=(0.028, 0.0240, 0.250)),
        material=steel,
        name="height_scale",
    )
    for index, z in enumerate((0.120, 0.180, 0.240, 0.300, 0.360)):
        mast.visual(
            Box((0.015, 0.0022, 0.002)),
            origin=Origin(xyz=(0.034, 0.0242, z)),
            material=steel,
            name=f"height_tick_{index}",
        )

    top_bracket = model.part("top_bracket")
    top_bracket.visual(
        Box((0.016, 0.010, 0.078)),
        origin=Origin(xyz=(-0.018, 0.034, 0.0)),
        material=steel,
        name="guide_pad_0",
    )
    top_bracket.visual(
        Box((0.016, 0.010, 0.078)),
        origin=Origin(xyz=(0.018, 0.034, 0.0)),
        material=steel,
        name="guide_pad_1",
    )
    top_bracket.visual(
        Box((0.086, 0.014, 0.086)),
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
        material=satin_blue,
        name="slide_plate",
    )
    top_bracket.visual(
        Box((0.074, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.081, 0.034)),
        material=matte_black,
        name="optic_platform",
    )
    for index, x in enumerate((-0.036, 0.036)):
        top_bracket.visual(
            Box((0.012, 0.055, 0.050)),
            origin=Origin(xyz=(x, 0.079, 0.015)),
            material=satin_blue,
            name=f"side_cheek_{index}",
        )
    top_bracket.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.113, 0.034), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="front_clamp_knob",
    )
    top_bracket.visual(
        Box((0.034, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.105, 0.034)),
        material=steel,
        name="optic_stop",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(-0.120, 0.0, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.240),
    )
    model.articulation(
        "mast_to_top_bracket",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=top_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.12, lower=0.0, upper=0.220),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    top_bracket = object_model.get_part("top_bracket")
    x_slide = object_model.get_articulation("base_to_mast")
    z_slide = object_model.get_articulation("mast_to_top_bracket")

    ctx.check(
        "lower articulation is X prismatic",
        x_slide.articulation_type == ArticulationType.PRISMATIC and tuple(x_slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={x_slide.articulation_type}, axis={x_slide.axis}",
    )
    ctx.check(
        "upper articulation is Z prismatic",
        z_slide.articulation_type == ArticulationType.PRISMATIC and tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={z_slide.articulation_type}, axis={z_slide.axis}",
    )

    base_aabb = ctx.part_world_aabb(base)
    mast_aabb = ctx.part_world_aabb(mast)
    if base_aabb is not None:
        base_dims = tuple(base_aabb[1][i] - base_aabb[0][i] for i in range(3))
        ctx.check(
            "base is short and broad",
            base_dims[0] > 0.40 and base_dims[1] > 0.18 and base_dims[2] < 0.090,
            details=f"base_dims={base_dims}",
        )
    if mast_aabb is not None:
        mast_dims = tuple(mast_aabb[1][i] - mast_aabb[0][i] for i in range(3))
        ctx.check(
            "mast is slender and tall",
            mast_dims[2] > 0.40 and mast_dims[0] < 0.12 and mast_dims[1] < 0.18,
            details=f"mast_dims={mast_dims}",
        )

    ctx.expect_contact(
        mast,
        base,
        elem_a="rail_shoe_0",
        elem_b="rail_0",
        contact_tol=0.001,
        name="lower shoe rides first rail",
    )
    ctx.expect_contact(
        mast,
        base,
        elem_a="rail_shoe_1",
        elem_b="rail_1",
        contact_tol=0.001,
        name="lower shoe rides second rail",
    )
    ctx.expect_contact(
        top_bracket,
        mast,
        elem_a="guide_pad_0",
        elem_b="guide_rail_0",
        contact_tol=0.001,
        name="top bracket rides first mast guide",
    )
    ctx.expect_contact(
        top_bracket,
        mast,
        elem_a="guide_pad_1",
        elem_b="guide_rail_1",
        contact_tol=0.001,
        name="top bracket rides second mast guide",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({x_slide: 0.240}):
        extended_mast_pos = ctx.part_world_position(mast)
        ctx.expect_within(
            mast,
            base,
            axes="x",
            inner_elem="carriage_block",
            outer_elem="base_plate",
            margin=0.0,
            name="carriage remains on broad base at X travel limit",
        )
        ctx.expect_contact(
            mast,
            base,
            elem_a="rail_shoe_0",
            elem_b="rail_0",
            contact_tol=0.001,
            name="first shoe remains on rail at X limit",
        )
    ctx.check(
        "mast translates along X",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[0] > rest_mast_pos[0] + 0.20
        and abs(extended_mast_pos[2] - rest_mast_pos[2]) < 0.002,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    rest_bracket_pos = ctx.part_world_position(top_bracket)
    with ctx.pose({z_slide: 0.220}):
        raised_bracket_pos = ctx.part_world_position(top_bracket)
        ctx.expect_overlap(
            top_bracket,
            mast,
            axes="z",
            elem_a="guide_pad_0",
            elem_b="guide_rail_0",
            min_overlap=0.060,
            name="raised bracket keeps insertion on mast guide",
        )
        ctx.expect_contact(
            top_bracket,
            mast,
            elem_a="guide_pad_0",
            elem_b="guide_rail_0",
            contact_tol=0.001,
            name="raised bracket still rides mast guide",
        )
    ctx.check(
        "top bracket translates along Z",
        rest_bracket_pos is not None
        and raised_bracket_pos is not None
        and raised_bracket_pos[2] > rest_bracket_pos[2] + 0.20
        and abs(raised_bracket_pos[0] - rest_bracket_pos[0]) < 0.002,
        details=f"rest={rest_bracket_pos}, raised={raised_bracket_pos}",
    )

    return ctx.report()


object_model = build_object_model()
