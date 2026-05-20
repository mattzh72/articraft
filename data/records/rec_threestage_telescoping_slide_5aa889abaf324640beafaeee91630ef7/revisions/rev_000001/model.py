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
    model = ArticulatedObject(name="bridge_backed_three_stage_extension_slide")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    bright_steel = model.material("polished_inner_steel", rgba=(0.78, 0.80, 0.76, 1.0))
    black = model.material("black_fastener", rgba=(0.02, 0.02, 0.018, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.86, 0.018, 0.170)),
        origin=Origin(xyz=(0.390, -0.078, 0.120)),
        material=galvanized,
        name="backing_plate",
    )
    rear_support.visual(
        Box((0.86, 0.031, 0.020)),
        origin=Origin(xyz=(0.390, -0.0545, 0.174)),
        material=galvanized,
        name="upper_bridge",
    )
    rear_support.visual(
        Box((0.86, 0.031, 0.020)),
        origin=Origin(xyz=(0.390, -0.0545, 0.066)),
        material=galvanized,
        name="lower_bridge",
    )
    rear_support.visual(
        Box((0.065, 0.030, 0.122)),
        origin=Origin(xyz=(-0.020, -0.054, 0.120)),
        material=galvanized,
        name="rear_stop",
    )
    for i, (x, z) in enumerate(((0.06, 0.165), (0.06, 0.075), (0.58, 0.165), (0.58, 0.075))):
        rear_support.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, -0.089, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"support_screw_{i}",
        )

    outer_body = model.part("outer_body")
    outer_body.visual(
        Box((0.720, 0.019, 0.104)),
        origin=Origin(xyz=(0.360, -0.0300, 0.120)),
        material=galvanized,
        name="outer_web",
    )
    outer_body.visual(
        Box((0.720, 0.071, 0.014)),
        origin=Origin(xyz=(0.360, -0.0035, 0.177)),
        material=galvanized,
        name="outer_top_lip",
    )
    outer_body.visual(
        Box((0.720, 0.071, 0.014)),
        origin=Origin(xyz=(0.360, -0.0035, 0.063)),
        material=galvanized,
        name="outer_bottom_lip",
    )
    outer_body.visual(
        Box((0.030, 0.071, 0.090)),
        origin=Origin(xyz=(0.015, -0.0035, 0.120)),
        material=galvanized,
        name="outer_rear_cap",
    )
    outer_body.visual(
        Box((0.660, 0.006, 0.006)),
        origin=Origin(xyz=(0.375, 0.0290, 0.177)),
        material=dark_steel,
        name="outer_race_upper",
    )
    outer_body.visual(
        Box((0.660, 0.006, 0.006)),
        origin=Origin(xyz=(0.375, 0.0290, 0.063)),
        material=dark_steel,
        name="outer_race_lower",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        Box((0.620, 0.012, 0.080)),
        origin=Origin(xyz=(0.310, -0.0040, 0.120)),
        material=dark_steel,
        name="middle_web",
    )
    middle_stage.visual(
        Box((0.620, 0.040, 0.011)),
        origin=Origin(xyz=(0.310, 0.0120, 0.1645)),
        material=dark_steel,
        name="middle_top_lip",
    )
    middle_stage.visual(
        Box((0.620, 0.040, 0.011)),
        origin=Origin(xyz=(0.310, 0.0120, 0.0755)),
        material=dark_steel,
        name="middle_bottom_lip",
    )
    middle_stage.visual(
        Box((0.018, 0.040, 0.100)),
        origin=Origin(xyz=(0.009, 0.0120, 0.120)),
        material=dark_steel,
        name="middle_rear_stop",
    )
    middle_stage.visual(
        Box((0.580, 0.004, 0.006)),
        origin=Origin(xyz=(0.330, 0.0280, 0.1650)),
        material=bright_steel,
        name="middle_inner_race_upper",
    )
    middle_stage.visual(
        Box((0.580, 0.004, 0.006)),
        origin=Origin(xyz=(0.330, 0.0280, 0.0750)),
        material=bright_steel,
        name="middle_inner_race_lower",
    )

    inner_stage = model.part("inner_stage")
    inner_stage.visual(
        Box((0.540, 0.022, 0.078)),
        origin=Origin(xyz=(0.270, 0.0170, 0.120)),
        material=bright_steel,
        name="inner_bar",
    )
    inner_stage.visual(
        Box((0.045, 0.030, 0.078)),
        origin=Origin(xyz=(0.5625, 0.0170, 0.120)),
        material=bright_steel,
        name="terminal_tab",
    )
    inner_stage.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.585, 0.034, 0.137), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="terminal_screw_upper",
    )
    inner_stage.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.585, 0.034, 0.103), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="terminal_screw_lower",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=rear_support,
        child=outer_body,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=middle_stage,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.320),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.280),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    outer_body = object_model.get_part("outer_body")
    middle_stage = object_model.get_part("middle_stage")
    inner_stage = object_model.get_part("inner_stage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.expect_contact(
        rear_support,
        outer_body,
        elem_a="upper_bridge",
        elem_b="outer_web",
        contact_tol=0.0005,
        name="rear bridge bears against outer body",
    )
    ctx.expect_within(
        middle_stage,
        outer_body,
        axes="yz",
        margin=0.001,
        name="middle stage nests inside outer body cross section",
    )
    ctx.expect_within(
        inner_stage,
        middle_stage,
        axes="yz",
        inner_elem="inner_bar",
        margin=0.001,
        name="inner stage nests inside middle stage cross section",
    )
    ctx.expect_overlap(
        middle_stage,
        outer_body,
        axes="x",
        min_overlap=0.55,
        name="middle stage has long collapsed insertion",
    )
    ctx.expect_overlap(
        inner_stage,
        middle_stage,
        axes="x",
        min_overlap=0.48,
        name="inner stage has long collapsed insertion",
    )

    rest_inner_position = ctx.part_world_position(inner_stage)
    with ctx.pose({outer_to_middle: 0.320, middle_to_inner: 0.280}):
        ctx.expect_overlap(
            middle_stage,
            outer_body,
            axes="x",
            min_overlap=0.30,
            name="middle stage remains retained at full travel",
        )
        ctx.expect_overlap(
            inner_stage,
            middle_stage,
            axes="x",
            min_overlap=0.24,
            name="inner stage remains retained at full travel",
        )
        ctx.expect_within(
            inner_stage,
            middle_stage,
            axes="yz",
            inner_elem="inner_bar",
            margin=0.001,
            name="extended inner stage stays on shared slide axis",
        )
        extended_inner_position = ctx.part_world_position(inner_stage)

    ctx.check(
        "serial slides extend along positive x",
        rest_inner_position is not None
        and extended_inner_position is not None
        and extended_inner_position[0] > rest_inner_position[0] + 0.55,
        details=f"rest={rest_inner_position}, extended={extended_inner_position}",
    )

    return ctx.report()


object_model = build_object_model()
