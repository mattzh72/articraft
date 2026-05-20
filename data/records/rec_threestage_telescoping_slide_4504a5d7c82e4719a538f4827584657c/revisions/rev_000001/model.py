from __future__ import annotations

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
    model = ArticulatedObject(name="nested_three_stage_slide")

    brushed_steel = Material("brushed_steel", color=(0.62, 0.64, 0.65, 1.0))
    dark_bearing = Material("dark_bearing_strip", color=(0.08, 0.09, 0.10, 1.0))
    yellow_tip = Material("yellow_tip_plate", color=(0.95, 0.74, 0.18, 1.0))
    screw_dark = Material("dark_screw_heads", color=(0.03, 0.03, 0.035, 1.0))

    # Root link: a grounded outer U-guide with side mounting flanges.
    outer = model.part("outer_guide")
    outer.visual(
        Box((0.82, 0.13, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_steel,
        name="outer_floor",
    )
    outer.visual(
        Box((0.82, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.059, 0.042)),
        material=brushed_steel,
        name="outer_side_0",
    )
    outer.visual(
        Box((0.82, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, -0.059, 0.042)),
        material=brushed_steel,
        name="outer_side_1",
    )
    outer.visual(
        Box((0.010, 0.13, 0.072)),
        origin=Origin(xyz=(-0.405, 0.0, 0.036)),
        material=brushed_steel,
        name="rear_stop",
    )
    outer.visual(
        Box((0.82, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.080, 0.004)),
        material=brushed_steel,
        name="mount_flange_0",
    )
    outer.visual(
        Box((0.82, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, -0.080, 0.004)),
        material=brushed_steel,
        name="mount_flange_1",
    )
    for index, x in enumerate((-0.30, 0.30)):
        for side, y in enumerate((-0.080, 0.080)):
            outer.visual(
                Cylinder(radius=0.008, length=0.004),
                origin=Origin(xyz=(x, y, 0.010)),
                material=screw_dark,
                name=f"mount_screw_{index}_{side}",
            )

    # Middle link: a low carriage riding in the outer guide and presenting a
    # smaller upper channel for the inner slide.
    middle = model.part("middle_stage")
    middle.visual(
        Box((0.60, 0.070, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_bearing,
        name="middle_floor",
    )
    middle.visual(
        Box((0.60, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.030, 0.037)),
        material=brushed_steel,
        name="middle_side_0",
    )
    middle.visual(
        Box((0.60, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.030, 0.037)),
        material=brushed_steel,
        name="middle_side_1",
    )
    middle.visual(
        Box((0.60, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.022, 0.055)),
        material=brushed_steel,
        name="middle_lip_0",
    )
    middle.visual(
        Box((0.60, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.022, 0.055)),
        material=brushed_steel,
        name="middle_lip_1",
    )

    # Inner link: a slender tongue captured by the middle channel, with a
    # visible tip plate just beyond the channel face when retracted.
    inner = model.part("inner_stage")
    inner.visual(
        Box((0.44, 0.040, 0.012)),
        origin=Origin(xyz=(-0.220, 0.0, 0.028)),
        material=dark_bearing,
        name="inner_rail",
    )
    inner.visual(
        Box((0.020, 0.030, 0.016)),
        origin=Origin(xyz=(0.005, 0.0, 0.028)),
        material=dark_bearing,
        name="tip_neck",
    )
    inner.visual(
        Box((0.032, 0.082, 0.050)),
        origin=Origin(xyz=(0.016, 0.0, 0.043)),
        material=yellow_tip,
        name="tip_plate",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.6, lower=0.0, upper=0.28),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.305, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.6, lower=0.0, upper=0.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.expect_gap(
        middle,
        outer,
        axis="z",
        positive_elem="middle_floor",
        negative_elem="outer_floor",
        max_gap=0.001,
        max_penetration=1e-6,
        name="middle carriage rides on outer floor",
    )
    ctx.expect_gap(
        inner,
        middle,
        axis="z",
        positive_elem="inner_rail",
        negative_elem="middle_floor",
        max_gap=0.001,
        max_penetration=1e-6,
        name="inner rail rides on middle floor",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="y",
        inner_elem="middle_floor",
        outer_elem="outer_floor",
        margin=0.0,
        name="middle stage is laterally captured by outer guide",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="y",
        inner_elem="inner_rail",
        outer_elem="middle_floor",
        margin=0.0,
        name="inner stage is laterally captured by middle guide",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="middle_floor",
        elem_b="outer_floor",
        min_overlap=0.50,
        name="middle stage is substantially nested when retracted",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_rail",
        elem_b="middle_floor",
        min_overlap=0.40,
        name="inner stage is substantially nested when retracted",
    )

    middle_rest = ctx.part_world_position(middle)
    with ctx.pose({outer_slide: 0.28}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_floor",
            elem_b="outer_floor",
            min_overlap=0.40,
            name="middle stage remains retained at full travel",
        )
        middle_extended = ctx.part_world_position(middle)

    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({inner_slide: 0.25}):
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_rail",
            elem_b="middle_floor",
            min_overlap=0.18,
            name="inner stage remains retained at full travel",
        )
        inner_extended = ctx.part_world_position(inner)

    ctx.check(
        "middle slide translates forward",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.20,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )
    ctx.check(
        "inner slide translates forward",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.20,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    return ctx.report()


object_model = build_object_model()
