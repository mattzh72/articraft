from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="warehouse_three_stage_shuttle")

    painted_steel = model.material("painted_steel", color=(0.19, 0.23, 0.26, 1.0))
    dark_rail = model.material("dark_linear_rail", color=(0.04, 0.045, 0.05, 1.0))
    safety_blue = model.material("safety_blue", color=(0.05, 0.22, 0.62, 1.0))
    safety_orange = model.material("safety_orange", color=(1.0, 0.42, 0.04, 1.0))
    rubber = model.material("black_rubber", color=(0.006, 0.006, 0.005, 1.0))
    zinc = model.material("zinc_hardware", color=(0.70, 0.72, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.00, 0.62, 0.060)),
        origin=Origin(xyz=(0.15, 0.0, 0.030)),
        material=painted_steel,
        name="deck",
    )
    base.visual(
        Box((0.95, 0.050, 0.070)),
        origin=Origin(xyz=(0.15, -0.235, 0.095)),
        material=painted_steel,
        name="base_rail_0",
    )
    base.visual(
        Box((0.88, 0.056, 0.015)),
        origin=Origin(xyz=(0.18, -0.235, 0.1375)),
        material=dark_rail,
        name="base_pad_0",
    )
    base.visual(
        Box((0.95, 0.050, 0.070)),
        origin=Origin(xyz=(0.15, 0.235, 0.095)),
        material=painted_steel,
        name="base_rail_1",
    )
    base.visual(
        Box((0.88, 0.056, 0.015)),
        origin=Origin(xyz=(0.18, 0.235, 0.1375)),
        material=dark_rail,
        name="base_pad_1",
    )
    base.visual(
        Box((0.16, 0.55, 0.160)),
        origin=Origin(xyz=(-0.43, 0.0, 0.140)),
        material=painted_steel,
        name="rear_drive",
    )
    base.visual(
        Box((0.080, 0.60, 0.070)),
        origin=Origin(xyz=(0.61, 0.0, 0.095)),
        material=painted_steel,
        name="front_crossmember",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.48),
        origin=Origin(xyz=(-0.43, 0.0, 0.235), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="drive_shaft",
    )
    base.visual(
        Box((0.10, 0.50, 0.030)),
        origin=Origin(xyz=(-0.43, 0.0, 0.205)),
        material=painted_steel,
        name="shaft_mount",
    )

    stage_1 = model.part("stage_1")
    stage_1.visual(
        Box((0.94, 0.46, 0.035)),
        origin=Origin(xyz=(0.47, 0.0, 0.0175)),
        material=safety_blue,
        name="stage_bed",
    )
    stage_1.visual(
        Box((0.88, 0.040, 0.056)),
        origin=Origin(xyz=(0.49, -0.180, 0.0620)),
        material=dark_rail,
        name="rail_0",
    )
    stage_1.visual(
        Box((0.80, 0.034, 0.016)),
        origin=Origin(xyz=(0.50, -0.1404, 0.0970)),
        material=rubber,
        name="upper_pad_0",
    )
    stage_1.visual(
        Box((0.88, 0.040, 0.056)),
        origin=Origin(xyz=(0.49, 0.180, 0.0620)),
        material=dark_rail,
        name="rail_1",
    )
    stage_1.visual(
        Box((0.80, 0.034, 0.016)),
        origin=Origin(xyz=(0.50, 0.1404, 0.0970)),
        material=rubber,
        name="upper_pad_1",
    )
    stage_1.visual(
        Box((0.080, 0.46, 0.092)),
        origin=Origin(xyz=(0.90, 0.0, 0.046)),
        material=safety_blue,
        name="front_crosshead",
    )
    stage_1.visual(
        Box((0.070, 0.46, 0.062)),
        origin=Origin(xyz=(0.035, 0.0, 0.066)),
        material=safety_blue,
        name="rear_bolster",
    )
    stage_1.visual(
        Box((0.78, 0.050, 0.036)),
        origin=Origin(xyz=(0.50, 0.0, 0.052)),
        material=painted_steel,
        name="cable_trough",
    )

    stage_2 = model.part("stage_2")
    stage_2.visual(
        Box((0.80, 0.33, 0.030)),
        origin=Origin(xyz=(0.40, 0.0, 0.015)),
        material=painted_steel,
        name="stage_bed",
    )
    stage_2.visual(
        Box((0.74, 0.035, 0.046)),
        origin=Origin(xyz=(0.43, -0.130, 0.052)),
        material=dark_rail,
        name="rail_0",
    )
    stage_2.visual(
        Box((0.68, 0.030, 0.013)),
        origin=Origin(xyz=(0.42, -0.1001, 0.0805)),
        material=rubber,
        name="upper_pad_0",
    )
    stage_2.visual(
        Box((0.74, 0.035, 0.046)),
        origin=Origin(xyz=(0.43, 0.130, 0.052)),
        material=dark_rail,
        name="rail_1",
    )
    stage_2.visual(
        Box((0.68, 0.030, 0.013)),
        origin=Origin(xyz=(0.42, 0.1001, 0.0805)),
        material=rubber,
        name="upper_pad_1",
    )
    stage_2.visual(
        Box((0.070, 0.33, 0.077)),
        origin=Origin(xyz=(0.765, 0.0, 0.0385)),
        material=painted_steel,
        name="front_crosshead",
    )
    stage_2.visual(
        Box((0.060, 0.33, 0.056)),
        origin=Origin(xyz=(0.030, 0.0, 0.058)),
        material=painted_steel,
        name="rear_bolster",
    )

    stage_3 = model.part("stage_3")
    stage_3.visual(
        Box((0.72, 0.22, 0.025)),
        origin=Origin(xyz=(0.36, 0.0, 0.0125)),
        material=safety_orange,
        name="shuttle_blade",
    )
    for index, y in enumerate((-0.070, 0.070)):
        stage_3.visual(
            Box((0.84, 0.045, 0.036)),
            origin=Origin(xyz=(0.49, y, 0.042)),
            material=safety_orange,
            name=f"fork_{index}",
        )
    stage_3.visual(
        Box((0.050, 0.22, 0.056)),
        origin=Origin(xyz=(0.90, 0.0, 0.053)),
        material=safety_orange,
        name="nose_bar",
    )
    stage_3.visual(
        Cylinder(radius=0.025, length=0.22),
        origin=Origin(xyz=(0.93, 0.0, 0.085), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="nose_roller",
    )
    stage_3.visual(
        Box((0.030, 0.18, 0.035)),
        origin=Origin(xyz=(0.91, 0.0, 0.067)),
        material=zinc,
        name="roller_bracket",
    )

    model.articulation(
        "base_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_1,
        origin=Origin(xyz=(-0.27, 0.0, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.70, lower=0.0, upper=0.42),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.08, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.65, lower=0.0, upper=0.36),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.07, 0.0, 0.087)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.60, lower=0.0, upper=0.34),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    joint_1 = object_model.get_articulation("base_to_stage_1")
    joint_2 = object_model.get_articulation("stage_1_to_stage_2")
    joint_3 = object_model.get_articulation("stage_2_to_stage_3")

    ctx.expect_gap(
        stage_1,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="stage_bed",
        negative_elem="base_pad_0",
        name="first stage rides on fixed base pads",
    )
    ctx.expect_gap(
        stage_2,
        stage_1,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="stage_bed",
        negative_elem="upper_pad_0",
        name="second stage rides on first stage pads",
    )
    ctx.expect_gap(
        stage_3,
        stage_2,
        axis="z",
        max_gap=0.001,
        max_penetration=0.000001,
        positive_elem="shuttle_blade",
        negative_elem="upper_pad_0",
        name="third stage rides on second stage pads",
    )
    ctx.expect_within(
        stage_3,
        stage_2,
        axes="y",
        margin=0.001,
        inner_elem="shuttle_blade",
        outer_elem="stage_bed",
        name="top shuttle blade stays centered between guides",
    )

    stage_1_rest = ctx.part_world_position(stage_1)
    stage_2_rest = ctx.part_world_position(stage_2)
    stage_3_rest = ctx.part_world_position(stage_3)
    with ctx.pose({joint_1: 0.42, joint_2: 0.36, joint_3: 0.34}):
        ctx.expect_overlap(
            stage_1,
            base,
            axes="x",
            min_overlap=0.25,
            elem_a="stage_bed",
            elem_b="base_pad_0",
            name="first stage remains captured at full extension",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="x",
            min_overlap=0.25,
            elem_a="stage_bed",
            elem_b="upper_pad_0",
            name="second stage remains captured at full extension",
        )
        ctx.expect_overlap(
            stage_3,
            stage_2,
            axes="x",
            min_overlap=0.24,
            elem_a="shuttle_blade",
            elem_b="upper_pad_0",
            name="third stage remains captured at full extension",
        )
        stage_1_extended = ctx.part_world_position(stage_1)
        stage_2_extended = ctx.part_world_position(stage_2)
        stage_3_extended = ctx.part_world_position(stage_3)

    ctx.check(
        "all prismatic stages extend forward",
        stage_1_rest is not None
        and stage_2_rest is not None
        and stage_3_rest is not None
        and stage_1_extended is not None
        and stage_2_extended is not None
        and stage_3_extended is not None
        and stage_1_extended[0] > stage_1_rest[0] + 0.40
        and stage_2_extended[0] > stage_2_rest[0] + 0.75
        and stage_3_extended[0] > stage_3_rest[0] + 1.05,
        details=(
            f"rest={(stage_1_rest, stage_2_rest, stage_3_rest)}, "
            f"extended={(stage_1_extended, stage_2_extended, stage_3_extended)}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
