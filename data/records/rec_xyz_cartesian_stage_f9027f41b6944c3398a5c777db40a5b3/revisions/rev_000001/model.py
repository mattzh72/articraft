from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_xyz_module")

    cast = model.material("dark_cast_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    rail = model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    carriage = model.material("anodized_blue", rgba=(0.05, 0.18, 0.42, 1.0))
    black = model.material("black_bearing", rgba=(0.02, 0.02, 0.025, 1.0))
    face = model.material("plain_aluminum", rgba=(0.82, 0.84, 0.82, 1.0))
    stop = model.material("matte_stop", rgba=(0.16, 0.16, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.75, 0.38, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast,
        name="base_plate",
    )
    for y, idx, rail_name in [(-0.12, 0, "x_rail_0"), (0.12, 1, "x_rail_1")]:
        base.visual(
            Box((0.62, 0.026, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.043)),
            material=rail,
            name=rail_name,
        )
        for x, end_idx in [(-0.335, 0), (0.335, 1)]:
            base.visual(
                Box((0.035, 0.07, 0.052)),
                origin=Origin(xyz=(x, y, 0.056)),
                material=stop,
                name=f"x_stop_{idx}_{end_idx}",
            )

    lower = model.part("lower_stage")
    for x, x_idx, y, rail_idx, cap_name in [
        (-0.10, 0, -0.12, 0, "x_bearing_cap_0_0"),
        (0.10, 1, -0.12, 0, "x_bearing_cap_0_1"),
        (-0.10, 0, 0.12, 1, "x_bearing_cap_1_0"),
        (0.10, 1, 0.12, 1, "x_bearing_cap_1_1"),
    ]:
        lower.visual(
            Box((0.095, 0.070, 0.018)),
            origin=Origin(xyz=(x, y, 0.009)),
            material=black,
            name=cap_name,
        )
        for side, sy in [(-1, y - 0.025), (1, y + 0.025)]:
            lower.visual(
                Box((0.095, 0.012, 0.046)),
                origin=Origin(xyz=(x, sy, 0.023)),
                material=black,
                name=f"x_bearing_cheek_{rail_idx}_{x_idx}_{0 if side < 0 else 1}",
            )
    lower.visual(
        Box((0.36, 0.30, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=carriage,
        name="lower_table",
    )
    for x, idx, rail_name in [(-0.11, 0, "y_rail_0"), (0.11, 1, "y_rail_1")]:
        lower.visual(
            Box((0.026, 0.32, 0.022)),
            origin=Origin(xyz=(x, 0.0, 0.089)),
            material=rail,
            name=rail_name,
        )
        for y, end_idx in [(-0.173, 0), (0.173, 1)]:
            lower.visual(
                Box((0.06, 0.026, 0.040)),
                origin=Origin(xyz=(x, y, 0.098)),
                material=stop,
                name=f"y_stop_{idx}_{end_idx}",
            )

    middle = model.part("middle_stage")
    for x, rail_idx, y, block_idx, cap_name in [
        (-0.11, 0, -0.055, 0, "y_bearing_cap_0_0"),
        (-0.11, 0, 0.055, 1, "y_bearing_cap_0_1"),
        (0.11, 1, -0.055, 0, "y_bearing_cap_1_0"),
        (0.11, 1, 0.055, 1, "y_bearing_cap_1_1"),
    ]:
        middle.visual(
            Box((0.066, 0.075, 0.018)),
            origin=Origin(xyz=(x, y, 0.009)),
            material=black,
            name=cap_name,
        )
        for side, sx in [(-1, x - 0.025), (1, x + 0.025)]:
            middle.visual(
                Box((0.012, 0.075, 0.040)),
                origin=Origin(xyz=(sx, y, 0.020)),
                material=black,
                name=f"y_bearing_cheek_{rail_idx}_{block_idx}_{0 if side < 0 else 1}",
            )
    middle.visual(
        Box((0.30, 0.28, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=carriage,
        name="cross_table",
    )
    middle.visual(
        Box((0.16, 0.16, 0.32)),
        origin=Origin(xyz=(-0.075, 0.0, 0.232)),
        material=cast,
        name="vertical_column",
    )
    for y, idx, rail_name in [(-0.055, 0, "z_rail_0"), (0.055, 1, "z_rail_1")]:
        middle.visual(
            Box((0.026, 0.024, 0.300)),
            origin=Origin(xyz=(0.018, y, 0.232)),
            material=rail,
            name=rail_name,
        )
    middle.visual(
        Box((0.11, 0.17, 0.018)),
        origin=Origin(xyz=(-0.050, 0.0, 0.082)),
        material=stop,
        name="z_lower_stop",
    )
    middle.visual(
        Box((0.11, 0.17, 0.018)),
        origin=Origin(xyz=(-0.050, 0.0, 0.382)),
        material=stop,
        name="z_upper_stop",
    )

    upper = model.part("upper_stage")
    for y, idx, z, block_idx, shoe_name in [
        (-0.055, 0, -0.040, 0, "z_carriage_shoe_0_0"),
        (-0.055, 0, 0.040, 1, "z_carriage_shoe_0_1"),
        (0.055, 1, -0.040, 0, "z_carriage_shoe_1_0"),
        (0.055, 1, 0.040, 1, "z_carriage_shoe_1_1"),
    ]:
        upper.visual(
            Box((0.040, 0.052, 0.055)),
            origin=Origin(xyz=(0.051, y, z)),
            material=black,
            name=shoe_name,
        )
    upper.visual(
        Box((0.050, 0.170, 0.120)),
        origin=Origin(xyz=(0.082, 0.0, 0.0)),
        material=carriage,
        name="faceplate_adapter",
    )
    upper.visual(
        Box((0.018, 0.200, 0.180)),
        origin=Origin(xyz=(0.116, 0.0, 0.0)),
        material=face,
        name="faceplate",
    )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=-0.12, upper=0.12),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.16, lower=-0.065, upper=0.065),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.12, lower=0.0, upper=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")
    z_slide = object_model.get_articulation("z_slide")

    ctx.check(
        "three orthogonal prismatic slides",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(y_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axes={x_slide.axis}, {y_slide.axis}, {z_slide.axis}",
    )

    base = object_model.get_part("base")
    lower = object_model.get_part("lower_stage")
    middle = object_model.get_part("middle_stage")
    upper = object_model.get_part("upper_stage")

    ctx.expect_overlap(
        lower,
        base,
        axes="xy",
        elem_a="x_bearing_cap_0_0",
        elem_b="x_rail_0",
        min_overlap=0.02,
        name="lower stage rides on X rail",
    )
    ctx.expect_overlap(
        middle,
        lower,
        axes="xy",
        elem_a="y_bearing_cap_0_0",
        elem_b="y_rail_0",
        min_overlap=0.02,
        name="middle stage rides on Y rail",
    )
    ctx.expect_overlap(
        upper,
        middle,
        axes="yz",
        elem_a="z_carriage_shoe_0_0",
        elem_b="z_rail_0",
        min_overlap=0.02,
        name="upper stage rides on Z rail",
    )
    ctx.expect_gap(
        upper,
        middle,
        axis="x",
        positive_elem="faceplate",
        negative_elem="z_rail_0",
        min_gap=0.05,
        max_gap=0.09,
        name="plain faceplate sits in front of vertical guide",
    )

    lower_rest = ctx.part_world_position(lower)
    with ctx.pose({x_slide: x_slide.motion_limits.upper}):
        lower_extended = ctx.part_world_position(lower)
        ctx.expect_within(
            lower,
            base,
            axes="x",
            inner_elem="lower_table",
            outer_elem="base_plate",
            margin=0.0,
            name="X table remains over grounded base at travel limit",
        )

    middle_rest = ctx.part_world_position(middle)
    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        middle_extended = ctx.part_world_position(middle)
        ctx.expect_overlap(
            middle,
            lower,
            axes="y",
            elem_a="y_bearing_cap_1_1",
            elem_b="y_rail_1",
            min_overlap=0.03,
            name="Y carriage remains engaged at travel limit",
        )

    upper_rest = ctx.part_world_position(upper)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        upper_extended = ctx.part_world_position(upper)
        ctx.expect_overlap(
            upper,
            middle,
            axes="z",
            elem_a="z_carriage_shoe_0_1",
            elem_b="z_rail_0",
            min_overlap=0.05,
            name="Z carriage remains engaged at top travel",
        )

    ctx.check(
        "X slide advances lower stage",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[0] > lower_rest[0] + 0.10
        and abs(lower_extended[1] - lower_rest[1]) < 1e-6
        and abs(lower_extended[2] - lower_rest[2]) < 1e-6,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )
    ctx.check(
        "Y slide advances middle stage",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[1] > middle_rest[1] + 0.05
        and abs(middle_extended[0] - middle_rest[0]) < 1e-6
        and abs(middle_extended[2] - middle_rest[2]) < 1e-6,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "Z slide raises upper stage",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[2] > upper_rest[2] + 0.10
        and abs(upper_extended[0] - upper_rest[0]) < 1e-6
        and abs(upper_extended[1] - upper_rest[1]) < 1e-6,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    return ctx.report()


object_model = build_object_model()
