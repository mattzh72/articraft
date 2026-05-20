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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_xy_positioning_module")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    polished_rail = model.material("polished_steel_rail", rgba=(0.88, 0.90, 0.92, 1.0))
    bronze = model.material("bronze_bearing_liner", rgba=(0.70, 0.46, 0.18, 1.0))
    black = model.material("black_fastener", rgba=(0.01, 0.01, 0.012, 1.0))

    side_support = model.part("side_support")
    side_support.visual(
        Box((0.82, 0.04, 0.46)),
        origin=Origin(xyz=(0.0, -0.02, 0.23)),
        material=dark_anodized,
        name="wall_plate",
    )
    side_support.visual(
        Box((0.86, 0.22, 0.05)),
        origin=Origin(xyz=(0.0, 0.05, 0.025)),
        material=dark_anodized,
        name="floor_foot",
    )
    for x in (-0.34, 0.34):
        side_support.visual(
            Box((0.026, 0.20, 0.18)),
            origin=Origin(xyz=(x, 0.05, 0.09)),
            material=dark_anodized,
            name=f"side_web_{'neg' if x < 0 else 'pos'}",
        )
    side_support.visual(
        Box((0.74, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, 0.007, 0.18)),
        material=aluminum,
        name="lower_rail_backer_low",
    )
    side_support.visual(
        Cylinder(radius=0.0115, length=0.72),
        origin=Origin(xyz=(0.0, 0.019, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rail,
        name="lower_rail_low",
    )
    side_support.visual(
        Box((0.74, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, 0.007, 0.28)),
        material=aluminum,
        name="lower_rail_backer_high",
    )
    side_support.visual(
        Cylinder(radius=0.0115, length=0.72),
        origin=Origin(xyz=(0.0, 0.019, 0.28), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_rail,
        name="lower_rail_high",
    )
    for x in (-0.39, 0.39):
        side_support.visual(
            Box((0.03, 0.052, 0.19)),
            origin=Origin(xyz=(x, 0.026, 0.23)),
            material=aluminum,
            name=f"end_stop_{'neg' if x < 0 else 'pos'}",
        )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        Box((0.26, 0.05, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="lower_backplate",
    )
    lower_carriage.visual(
        Box((0.28, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.085, 0.1075)),
        material=aluminum,
        name="lower_deck",
    )
    for x in (-0.105, 0.105):
        lower_carriage.visual(
            Box((0.018, 0.18, 0.095)),
            origin=Origin(xyz=(x, 0.075, 0.0575)),
            material=aluminum,
            name=f"deck_rib_{'neg' if x < 0 else 'pos'}",
        )
    lower_carriage.visual(
        Box((0.19, 0.022, 0.034)),
        origin=Origin(xyz=(0.0, -0.0285, -0.05)),
        material=bronze,
        name="lower_bearing_shoe_low",
    )
    lower_carriage.visual(
        Box((0.19, 0.022, 0.034)),
        origin=Origin(xyz=(0.0, -0.0285, 0.05)),
        material=bronze,
        name="lower_bearing_shoe_high",
    )
    lower_carriage.visual(
        Box((0.030, 0.215, 0.008)),
        origin=Origin(xyz=(-0.075, 0.085, 0.129)),
        material=dark_anodized,
        name="cross_rail_base_neg",
    )
    lower_carriage.visual(
        Cylinder(radius=0.008, length=0.205),
        origin=Origin(xyz=(-0.075, 0.085, 0.141), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished_rail,
        name="cross_rail_neg",
    )
    lower_carriage.visual(
        Box((0.030, 0.215, 0.008)),
        origin=Origin(xyz=(0.075, 0.085, 0.129)),
        material=dark_anodized,
        name="cross_rail_base_pos",
    )
    lower_carriage.visual(
        Cylinder(radius=0.008, length=0.205),
        origin=Origin(xyz=(0.075, 0.085, 0.141), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished_rail,
        name="cross_rail_pos",
    )

    upper_slide = model.part("upper_slide")
    upper_slide.visual(
        Box((0.20, 0.14, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=aluminum,
        name="upper_saddle",
    )
    upper_slide.visual(
        Box((0.22, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=dark_anodized,
        name="top_plate",
    )
    for x in (-0.075, 0.075):
        for y in (-0.055, 0.055):
            upper_slide.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(x, y, 0.052)),
                material=black,
                name=f"cap_screw_{'neg' if x < 0 else 'pos'}_{'neg' if y < 0 else 'pos'}",
            )

    model.articulation(
        "support_to_lower_carriage",
        ArticulationType.PRISMATIC,
        parent=side_support,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.070, 0.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=-0.20, upper=0.20),
    )
    model.articulation(
        "lower_carriage_to_upper_slide",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_slide,
        origin=Origin(xyz=(0.0, 0.085, 0.149)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=-0.04, upper=0.04),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_carriage")
    upper = object_model.get_part("upper_slide")
    lower_axis = object_model.get_articulation("support_to_lower_carriage")
    upper_axis = object_model.get_articulation("lower_carriage_to_upper_slide")

    ctx.check(
        "two orthogonal horizontal prismatic axes",
        lower_axis.articulation_type == ArticulationType.PRISMATIC
        and upper_axis.articulation_type == ArticulationType.PRISMATIC
        and abs(lower_axis.axis[2]) < 1e-9
        and abs(upper_axis.axis[2]) < 1e-9
        and abs(sum(a * b for a, b in zip(lower_axis.axis, upper_axis.axis))) < 1e-9,
        details=f"lower_axis={lower_axis.axis}, upper_axis={upper_axis.axis}",
    )

    ctx.expect_within(
        upper,
        lower,
        axes="xy",
        inner_elem="top_plate",
        outer_elem="lower_deck",
        margin=0.001,
        name="top plate centered over lower deck at neutral",
    )
    ctx.expect_contact(
        upper,
        lower,
        elem_a="upper_saddle",
        elem_b="cross_rail_neg",
        contact_tol=0.001,
        name="upper saddle rides on cross rail",
    )
    ctx.expect_contact(
        lower,
        "side_support",
        elem_a="lower_bearing_shoe_high",
        elem_b="lower_rail_high",
        contact_tol=0.006,
        name="lower bearing shoe rides near fixed rail",
    )

    lower_rest = ctx.part_world_position(lower)
    upper_rest = ctx.part_world_position(upper)
    with ctx.pose({lower_axis: 0.18, upper_axis: 0.0}):
        lower_extended = ctx.part_world_position(lower)
    with ctx.pose({lower_axis: 0.0, upper_axis: 0.04}):
        upper_extended = ctx.part_world_position(upper)
        ctx.expect_within(
            upper,
            lower,
            axes="xy",
            inner_elem="top_plate",
            outer_elem="lower_deck",
            margin=0.001,
            name="top plate remains over lower carriage at cross-slide limit",
        )

    ctx.check(
        "lower carriage travels along x",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[0] > lower_rest[0] + 0.17
        and abs(lower_extended[1] - lower_rest[1]) < 1e-6
        and abs(lower_extended[2] - lower_rest[2]) < 1e-6,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )
    ctx.check(
        "upper slide travels along y",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[1] > upper_rest[1] + 0.035
        and abs(upper_extended[0] - upper_rest[0]) < 1e-6
        and abs(upper_extended[2] - upper_rest[2]) < 1e-6,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    return ctx.report()


object_model = build_object_model()
