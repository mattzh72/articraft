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

    matte_black = model.material("matte_black", rgba=(0.02, 0.022, 0.025, 1.0))
    anodized = model.material("dark_anodized", rgba=(0.08, 0.09, 0.10, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.23, 0.25, 0.27, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    brass = model.material("brass_wear", rgba=(0.83, 0.63, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.05, 0.38, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=matte_black,
        name="base_plate",
    )
    base.visual(
        Box((0.94, 0.245, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=gunmetal,
        name="raised_way",
    )
    base.visual(
        Box((0.86, 0.065, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=steel,
        name="x_rail",
    )
    for y, name in ((0.104, "front_cover_strip"), (-0.104, "rear_cover_strip")):
        base.visual(
            Box((0.89, 0.022, 0.014)),
            origin=Origin(xyz=(0.0, y, 0.082)),
            material=anodized,
            name=name,
        )
    base.visual(
        Box((0.036, 0.155, 0.088)),
        origin=Origin(xyz=(-0.455, 0.0, 0.119)),
        material=anodized,
        name="left_end_stop",
    )
    base.visual(
        Box((0.036, 0.155, 0.088)),
        origin=Origin(xyz=(0.455, 0.0, 0.119)),
        material=anodized,
        name="right_end_stop",
    )
    base.visual(
        Box((0.010, 0.105, 0.030)),
        origin=Origin(xyz=(-0.430, 0.0, 0.121)),
        material=rubber,
        name="left_rubber_stop",
    )
    base.visual(
        Box((0.010, 0.105, 0.030)),
        origin=Origin(xyz=(0.430, 0.0, 0.121)),
        material=rubber,
        name="right_rubber_stop",
    )
    for x, y, name in (
        (-0.43, -0.145, "foot_0"),
        (-0.43, 0.145, "foot_1"),
        (0.43, -0.145, "foot_2"),
        (0.43, 0.145, "foot_3"),
    ):
        base.visual(
            Box((0.090, 0.055, 0.016)),
            origin=Origin(xyz=(x, y, -0.008)),
            material=rubber,
            name=name,
        )

    mast_carriage = model.part("mast_carriage")
    mast_carriage.visual(
        Box((0.240, 0.112, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=anodized,
        name="top_shoe",
    )
    for y, name in ((0.048, "front_side_shoe"), (-0.048, "rear_side_shoe")):
        mast_carriage.visual(
            Box((0.185, 0.020, 0.056)),
            origin=Origin(xyz=(0.0, y, -0.024)),
            material=anodized,
            name=name,
        )
        mast_carriage.visual(
            Box((0.168, 0.004, 0.045)),
            origin=Origin(xyz=(0.0, y * 0.79, -0.025)),
            material=brass,
            name=f"{name}_liner",
        )
    mast_carriage.visual(
        Box((0.170, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=gunmetal,
        name="socket_floor",
    )
    mast_carriage.visual(
        Box((0.170, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, 0.069, 0.078)),
        material=gunmetal,
        name="socket_front_wall",
    )
    mast_carriage.visual(
        Box((0.170, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, -0.069, 0.078)),
        material=gunmetal,
        name="socket_rear_wall",
    )
    mast_carriage.visual(
        Box((0.014, 0.126, 0.090)),
        origin=Origin(xyz=(0.078, 0.0, 0.078)),
        material=gunmetal,
        name="socket_side_wall_0",
    )
    mast_carriage.visual(
        Box((0.014, 0.126, 0.090)),
        origin=Origin(xyz=(-0.078, 0.0, 0.078)),
        material=gunmetal,
        name="socket_side_wall_1",
    )
    mast_carriage.visual(
        Box((0.070, 0.055, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, 0.343)),
        material=matte_black,
        name="box_mast",
    )
    mast_carriage.visual(
        Box((0.030, 0.012, 0.520)),
        origin=Origin(xyz=(0.0, 0.0335, 0.370)),
        material=steel,
        name="mast_front_way",
    )
    mast_carriage.visual(
        Box((0.010, 0.011, 0.500)),
        origin=Origin(xyz=(-0.026, 0.033, 0.365)),
        material=brass,
        name="mast_wear_strip_0",
    )
    mast_carriage.visual(
        Box((0.010, 0.011, 0.500)),
        origin=Origin(xyz=(0.026, 0.033, 0.365)),
        material=brass,
        name="mast_wear_strip_1",
    )
    mast_carriage.visual(
        Box((0.090, 0.075, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=anodized,
        name="mast_top_cap",
    )

    top_bracket = model.part("top_bracket")
    top_bracket.visual(
        Box((0.112, 0.024, 0.130)),
        origin=Origin(xyz=(0.0, 0.012, 0.065)),
        material=anodized,
        name="guide_plate",
    )
    for x, name in ((0.050, "side_guide_0"), (-0.050, "side_guide_1")):
        top_bracket.visual(
            Box((0.018, 0.032, 0.126)),
            origin=Origin(xyz=(x, -0.010, 0.064)),
            material=anodized,
            name=name,
        )
        top_bracket.visual(
            Box((0.004, 0.024, 0.104)),
            origin=Origin(xyz=(x * 0.84, -0.006, 0.064)),
            material=brass,
            name=f"{name}_liner",
        )
    top_bracket.visual(
        Box((0.180, 0.110, 0.028)),
        origin=Origin(xyz=(0.0, 0.078, 0.118)),
        material=gunmetal,
        name="top_arm",
    )
    top_bracket.visual(
        Box((0.140, 0.035, 0.080)),
        origin=Origin(xyz=(0.0, 0.140, 0.074)),
        material=gunmetal,
        name="optic_face",
    )
    top_bracket.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.163, 0.074), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="mount_screw",
    )
    top_bracket.visual(
        Box((0.070, 0.025, 0.018)),
        origin=Origin(xyz=(0.0, 0.117, 0.141)),
        material=matte_black,
        name="top_cover",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast_carriage,
        origin=Origin(xyz=(-0.240, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.440),
    )
    model.articulation(
        "mast_to_bracket",
        ArticulationType.PRISMATIC,
        parent=mast_carriage,
        child=top_bracket,
        origin=Origin(xyz=(0.0, 0.0395, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=0.300),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("mast_carriage")
    bracket = object_model.get_part("top_bracket")
    lower = object_model.get_articulation("base_to_carriage")
    upper = object_model.get_articulation("mast_to_bracket")

    base_box = ctx.part_world_aabb(base)
    mast_box = ctx.part_world_aabb(carriage)
    if base_box is not None:
        base_dims = tuple(base_box[1][i] - base_box[0][i] for i in range(3))
        ctx.check(
            "base is short and broad",
            base_dims[0] > 0.95 and base_dims[1] > 0.32 and base_dims[2] < 0.20,
            details=f"base dimensions={base_dims}",
        )
    if mast_box is not None:
        mast_dims = tuple(mast_box[1][i] - mast_box[0][i] for i in range(3))
        ctx.check(
            "mast is slender and tall",
            mast_dims[2] > 0.65 and mast_dims[2] > 2.3 * max(mast_dims[0], mast_dims[1]),
            details=f"mast carriage dimensions={mast_dims}",
        )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="top_shoe",
        negative_elem="x_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower shoe rests on rail without clipping",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        elem_a="top_shoe",
        elem_b="x_rail",
        min_overlap=0.050,
        name="lower shoe remains captured on the rail",
    )
    ctx.expect_gap(
        bracket,
        carriage,
        axis="y",
        positive_elem="guide_plate",
        negative_elem="mast_front_way",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper guide plate rides on mast way without clipping",
    )
    ctx.expect_overlap(
        bracket,
        carriage,
        axes="xz",
        elem_a="guide_plate",
        elem_b="mast_front_way",
        min_overlap=0.020,
        name="upper bracket stays engaged with the mast way",
    )

    lower_rest = ctx.part_world_position(carriage)
    with ctx.pose({lower: 0.440}):
        lower_extended = ctx.part_world_position(carriage)
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem="right_rubber_stop",
            negative_elem="top_shoe",
            min_gap=0.040,
            name="lower slide clears right stop at full travel",
        )
    with ctx.pose({lower: 0.0}):
        ctx.expect_gap(
            carriage,
            base,
            axis="x",
            positive_elem="top_shoe",
            negative_elem="left_rubber_stop",
            min_gap=0.035,
            name="lower slide clears left stop at home",
        )
    ctx.check(
        "lower articulation travels along positive X",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[0] > lower_rest[0] + 0.42,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(bracket)
    with ctx.pose({upper: 0.300}):
        upper_extended = ctx.part_world_position(bracket)
        ctx.expect_gap(
            carriage,
            bracket,
            axis="z",
            positive_elem="mast_top_cap",
            negative_elem="guide_plate",
            min_gap=0.020,
            name="upper slide clears mast top cap at full lift",
        )
        ctx.expect_gap(
            bracket,
            carriage,
            axis="y",
            positive_elem="guide_plate",
            negative_elem="mast_front_way",
            max_gap=0.001,
            max_penetration=0.0,
            name="upper guide stays seated throughout lift",
        )
    with ctx.pose({upper: 0.0}):
        ctx.expect_gap(
            bracket,
            carriage,
            axis="z",
            positive_elem="guide_plate",
            negative_elem="socket_front_wall",
            min_gap=0.040,
            name="upper slide clears lower socket wall at home",
        )
    ctx.check(
        "upper articulation travels along positive Z",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[2] > upper_rest[2] + 0.28,
        details=f"rest={upper_rest}, raised={upper_extended}",
    )

    return ctx.report()


object_model = build_object_model()
