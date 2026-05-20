from __future__ import annotations

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
    model = ArticulatedObject(name="under_slung_telescoping_runner")

    zinc = model.material("zinc_plated_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_steel = model.material("blackened_runner_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    screw_dark = model.material("dark_screw_heads", rgba=(0.025, 0.025, 0.025, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.72, 0.11, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=zinc,
        name="mount_plate",
    )
    top_support.visual(
        Box((0.64, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=zinc,
        name="center_rib",
    )
    for index, x in enumerate((-0.27, 0.27)):
        for side, y in enumerate((-0.035, 0.035)):
            top_support.visual(
                Cylinder(radius=0.009, length=0.003),
                origin=Origin(xyz=(x, y, 0.0235)),
                material=screw_dark,
                name=f"screw_{index}_{side}",
            )

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        Box((0.62, 0.064, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=zinc,
        name="top_web",
    )
    for side, y in enumerate((-0.029, 0.029)):
        outer_sleeve.visual(
            Box((0.62, 0.006, 0.048)),
            origin=Origin(xyz=(0.0, y, -0.030)),
            material=zinc,
            name=f"side_wall_{side}",
        )
    for name, y in (("lower_lip_0", -0.020), ("lower_lip_1", 0.020)):
        outer_sleeve.visual(
            Box((0.62, 0.014, 0.006)),
            origin=Origin(xyz=(0.0, y, -0.051)),
            material=zinc,
            name=name,
        )

    runner = model.part("runner")
    runner.visual(
        Box((0.52, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=dark_steel,
        name="captured_head",
    )
    runner.visual(
        Box((0.66, 0.016, 0.026)),
        origin=Origin(xyz=(0.03, 0.0, -0.052)),
        material=dark_steel,
        name="hanging_stem",
    )
    runner.visual(
        Box((0.74, 0.040, 0.016)),
        origin=Origin(xyz=(0.04, 0.0, -0.073)),
        material=dark_steel,
        name="lower_rail",
    )
    runner.visual(
        Box((0.030, 0.048, 0.032)),
        origin=Origin(xyz=(0.425, 0.0, -0.060)),
        material=dark_steel,
        name="front_stop",
    )

    model.articulation(
        "support_to_sleeve",
        ArticulationType.FIXED,
        parent=top_support,
        child=outer_sleeve,
        origin=Origin(),
    )
    model.articulation(
        "sleeve_to_runner",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=runner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    outer_sleeve = object_model.get_part("outer_sleeve")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("sleeve_to_runner")

    ctx.expect_contact(
        top_support,
        outer_sleeve,
        elem_a="center_rib",
        elem_b="top_web",
        name="support rib bears on sleeve top web",
    )
    ctx.expect_within(
        runner,
        outer_sleeve,
        axes="yz",
        inner_elem="captured_head",
        name="captured head sits inside sleeve throat",
    )
    ctx.expect_gap(
        outer_sleeve,
        runner,
        axis="z",
        min_gap=0.002,
        max_gap=0.012,
        positive_elem="lower_lip_0",
        negative_elem="lower_rail",
        name="runner hangs just below sleeve lips",
    )
    ctx.expect_overlap(
        runner,
        outer_sleeve,
        axes="x",
        min_overlap=0.45,
        elem_a="captured_head",
        name="closed runner is nested in sleeve",
    )

    rest_pos = ctx.part_world_position(runner)
    with ctx.pose({slide: 0.24}):
        ctx.expect_within(
            runner,
            outer_sleeve,
            axes="yz",
            inner_elem="captured_head",
            name="extended runner remains captured in sleeve throat",
        )
        ctx.expect_overlap(
            runner,
            outer_sleeve,
            axes="x",
            min_overlap=0.30,
            elem_a="captured_head",
            name="extended runner retains insertion",
        )
        extended_pos = ctx.part_world_position(runner)

    ctx.check(
        "prismatic joint extends along slide direction",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
