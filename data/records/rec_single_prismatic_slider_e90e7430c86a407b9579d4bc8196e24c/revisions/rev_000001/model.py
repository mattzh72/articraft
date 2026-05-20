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
    model = ArticulatedObject(name="fork_captured_linear_runner")

    painted_steel = model.material("painted_steel", color=(0.12, 0.15, 0.17, 1.0))
    worn_steel = model.material("worn_steel", color=(0.62, 0.64, 0.62, 1.0))
    guide_polymer = model.material("guide_polymer", color=(0.04, 0.045, 0.045, 1.0))
    fastener_dark = model.material("fastener_dark", color=(0.02, 0.02, 0.018, 1.0))

    # Dimensions in meters.  The fork is a grounded, open-ended guide: two
    # parallel arms stand on a base plate and carry thin low-friction guide
    # liners.  The runner bar is deliberately smaller than the clear opening so
    # the side clearance is visible instead of being hidden by overlap.
    base_length = 0.78
    base_width = 0.34
    base_thickness = 0.035
    arm_length = 0.62
    arm_thickness = 0.045
    arm_height = 0.120
    arm_center_x = 0.035
    arm_center_y = 0.105
    arm_center_z = base_thickness + arm_height / 2.0 - 0.001

    frame = model.part("fork_frame")
    frame.visual(
        Box((base_length, base_width, base_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=painted_steel,
        name="base_plate",
    )
    frame.visual(
        Box((arm_length, arm_thickness, arm_height)),
        origin=Origin(xyz=(arm_center_x, arm_center_y, arm_center_z)),
        material=painted_steel,
        name="arm_0",
    )
    frame.visual(
        Box((arm_length, arm_thickness, arm_height)),
        origin=Origin(xyz=(arm_center_x, -arm_center_y, arm_center_z)),
        material=painted_steel,
        name="arm_1",
    )

    # Rear bridge makes the fork frame a single U-shaped member while leaving
    # the front open for the runner's plain end plate.
    frame.visual(
        Box((0.060, 0.255, arm_height)),
        origin=Origin(xyz=(-0.300, 0.0, arm_center_z)),
        material=painted_steel,
        name="rear_bridge",
    )

    # Replaceable guide liners sit just inside the fork arms.  They are slightly
    # embedded into the arms but leave 19 mm side clearance to the sliding bar.
    frame.visual(
        Box((0.540, 0.006, 0.050)),
        origin=Origin(xyz=(0.055, 0.0795, 0.096)),
        material=guide_polymer,
        name="guide_liner_0",
    )
    frame.visual(
        Box((0.540, 0.006, 0.050)),
        origin=Origin(xyz=(0.055, -0.0795, 0.096)),
        material=guide_polymer,
        name="guide_liner_1",
    )
    frame.visual(
        Box((0.540, 0.050, 0.0325)),
        origin=Origin(xyz=(0.055, 0.0, base_thickness + 0.0325 / 2.0)),
        material=guide_polymer,
        name="support_rail",
    )

    # Grounding fasteners make the frame read as fixed to a surface.
    for i, x in enumerate((-0.300, 0.300)):
        for j, y in enumerate((-0.125, 0.125)):
            frame.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(x, y, base_thickness + 0.003)),
                material=fastener_dark,
                name=f"bolt_{i}_{j}",
            )

    runner = model.part("runner")
    runner.visual(
        Box((0.570, 0.115, 0.055)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=worn_steel,
        name="slide_bar",
    )
    runner.visual(
        Box((0.028, 0.160, 0.100)),
        origin=Origin(xyz=(0.372, 0.0, 0.0)),
        material=worn_steel,
        name="end_plate",
    )

    model.articulation(
        "frame_to_runner",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=runner,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("fork_frame")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("frame_to_runner")

    ctx.check(
        "single prismatic guide joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type is {slide.articulation_type}",
    )

    # Prove that the bar is captured between the two fork-arm liners with real
    # guide clearance rather than interpenetration.
    ctx.expect_gap(
        frame,
        runner,
        axis="y",
        positive_elem="guide_liner_0",
        negative_elem="slide_bar",
        min_gap=0.015,
        max_gap=0.025,
        name="positive side guide clearance",
    )
    ctx.expect_gap(
        runner,
        frame,
        axis="y",
        positive_elem="slide_bar",
        negative_elem="guide_liner_1",
        min_gap=0.015,
        max_gap=0.025,
        name="negative side guide clearance",
    )
    ctx.expect_gap(
        runner,
        frame,
        axis="z",
        positive_elem="slide_bar",
        negative_elem="support_rail",
        min_gap=0.0,
        max_gap=0.001,
        name="bar rides on support rail",
    )
    ctx.expect_gap(
        runner,
        frame,
        axis="x",
        positive_elem="end_plate",
        negative_elem="arm_0",
        min_gap=0.010,
        max_gap=0.020,
        name="end plate sits just outside fork mouth",
    )
    ctx.expect_overlap(
        runner,
        frame,
        axes="x",
        elem_a="slide_bar",
        elem_b="guide_liner_0",
        min_overlap=0.45,
        name="bar remains long inside guide at rest",
    )

    rest_pos = ctx.part_world_position(runner)
    with ctx.pose({slide: 0.22}):
        ctx.expect_gap(
            frame,
            runner,
            axis="y",
            positive_elem="guide_liner_0",
            negative_elem="slide_bar",
            min_gap=0.015,
            max_gap=0.025,
            name="extended positive side guide clearance",
        )
        ctx.expect_gap(
            runner,
            frame,
            axis="y",
            positive_elem="slide_bar",
            negative_elem="guide_liner_1",
            min_gap=0.015,
            max_gap=0.025,
            name="extended negative side guide clearance",
        )
        ctx.expect_overlap(
            runner,
            frame,
            axes="x",
            elem_a="slide_bar",
            elem_b="guide_liner_0",
            min_overlap=0.25,
            name="bar remains captured at full travel",
        )
        extended_pos = ctx.part_world_position(runner)

    ctx.check(
        "runner travels along guide axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20
        and abs(extended_pos[1] - rest_pos[1]) < 1.0e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1.0e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
