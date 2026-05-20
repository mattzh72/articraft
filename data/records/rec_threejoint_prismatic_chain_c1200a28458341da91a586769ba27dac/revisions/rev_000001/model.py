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
    model = ArticulatedObject(name="three_stage_service_fixture")

    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.10, 0.24, 0.46, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.70, 1.0))
    hard_chrome = model.material("hard_chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    yellow_stop = model.material("yellow_stop", rgba=(0.95, 0.70, 0.12, 1.0))
    black_wear = model.material("black_wear", rgba=(0.025, 0.025, 0.025, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        Box((0.94, 0.36, 0.040)),
        origin=Origin(xyz=(0.09, 0.0, 0.020)),
        material=dark_steel,
        name="base_plate",
    )
    guide_body.visual(
        Box((0.82, 0.045, 0.090)),
        origin=Origin(xyz=(0.08, 0.105, 0.085)),
        material=blue_anodized,
        name="fixed_left_rail",
    )
    guide_body.visual(
        Box((0.82, 0.045, 0.090)),
        origin=Origin(xyz=(0.08, -0.105, 0.085)),
        material=blue_anodized,
        name="fixed_right_rail",
    )
    guide_body.visual(
        Box((0.055, 0.260, 0.105)),
        origin=Origin(xyz=(-0.355, 0.0, 0.0925)),
        material=blue_anodized,
        name="rear_bulkhead",
    )
    guide_body.visual(
        Box((0.040, 0.250, 0.035)),
        origin=Origin(xyz=(-0.330, 0.0, 0.1475)),
        material=yellow_stop,
        name="rear_stop_bar",
    )
    guide_body.visual(
        Box((0.055, 0.045, 0.040)),
        origin=Origin(xyz=(0.480, 0.105, 0.150)),
        material=yellow_stop,
        name="front_stop_left",
    )
    guide_body.visual(
        Box((0.055, 0.045, 0.040)),
        origin=Origin(xyz=(0.480, -0.105, 0.150)),
        material=yellow_stop,
        name="front_stop_right",
    )

    stage_0 = model.part("stage_0")
    stage_0.visual(
        Box((0.700, 0.120, 0.050)),
        origin=Origin(xyz=(-0.200, 0.0, 0.0)),
        material=brushed_aluminum,
        name="lower_tongue",
    )
    stage_0.visual(
        Box((0.480, 0.030, 0.020)),
        origin=Origin(xyz=(-0.240, 0.040, -0.035)),
        material=black_wear,
        name="wear_pad_left",
    )
    stage_0.visual(
        Box((0.480, 0.030, 0.020)),
        origin=Origin(xyz=(-0.240, -0.040, -0.035)),
        material=black_wear,
        name="wear_pad_right",
    )
    stage_0.visual(
        Box((0.480, 0.150, 0.035)),
        origin=Origin(xyz=(-0.120, 0.0, 0.0425)),
        material=brushed_aluminum,
        name="upper_deck",
    )
    stage_0.visual(
        Box((0.460, 0.022, 0.055)),
        origin=Origin(xyz=(-0.100, 0.065, 0.0875)),
        material=blue_anodized,
        name="upper_left_rail",
    )
    stage_0.visual(
        Box((0.460, 0.022, 0.055)),
        origin=Origin(xyz=(-0.100, -0.065, 0.0875)),
        material=blue_anodized,
        name="upper_right_rail",
    )
    stage_0.visual(
        Box((0.034, 0.116, 0.040)),
        origin=Origin(xyz=(0.145, 0.0, 0.002)),
        material=yellow_stop,
        name="front_stop_bar",
    )
    stage_0.visual(
        Box((0.030, 0.116, 0.030)),
        origin=Origin(xyz=(-0.500, 0.0, 0.020)),
        material=yellow_stop,
        name="rear_stop_bar",
    )
    stage_0.visual(
        Box((0.025, 0.022, 0.025)),
        origin=Origin(xyz=(0.140, 0.065, 0.1275)),
        material=yellow_stop,
        name="upper_stop_left",
    )
    stage_0.visual(
        Box((0.025, 0.022, 0.025)),
        origin=Origin(xyz=(0.140, -0.065, 0.1275)),
        material=yellow_stop,
        name="upper_stop_right",
    )

    stage_1 = model.part("stage_1")
    stage_1.visual(
        Box((0.520, 0.074, 0.035)),
        origin=Origin(xyz=(-0.170, 0.0, 0.0)),
        material=hard_chrome,
        name="lower_slider",
    )
    stage_1.visual(
        Box((0.380, 0.030, 0.010)),
        origin=Origin(xyz=(-0.200, 0.020, -0.0225)),
        material=black_wear,
        name="wear_pad_left",
    )
    stage_1.visual(
        Box((0.380, 0.030, 0.010)),
        origin=Origin(xyz=(-0.200, -0.020, -0.0225)),
        material=black_wear,
        name="wear_pad_right",
    )
    stage_1.visual(
        Box((0.340, 0.096, 0.025)),
        origin=Origin(xyz=(-0.090, 0.0, 0.030)),
        material=hard_chrome,
        name="top_deck",
    )
    stage_1.visual(
        Box((0.320, 0.016, 0.040)),
        origin=Origin(xyz=(-0.080, 0.042, 0.0625)),
        material=blue_anodized,
        name="top_left_rail",
    )
    stage_1.visual(
        Box((0.320, 0.016, 0.040)),
        origin=Origin(xyz=(-0.080, -0.042, 0.0625)),
        material=blue_anodized,
        name="top_right_rail",
    )
    stage_1.visual(
        Box((0.030, 0.070, 0.030)),
        origin=Origin(xyz=(0.085, 0.0, 0.004)),
        material=yellow_stop,
        name="front_stop_bar",
    )
    stage_1.visual(
        Box((0.026, 0.070, 0.026)),
        origin=Origin(xyz=(-0.405, 0.0, 0.014)),
        material=yellow_stop,
        name="rear_stop_bar",
    )
    stage_1.visual(
        Box((0.022, 0.016, 0.022)),
        origin=Origin(xyz=(0.090, 0.042, 0.0935)),
        material=yellow_stop,
        name="top_stop_left",
    )
    stage_1.visual(
        Box((0.022, 0.016, 0.022)),
        origin=Origin(xyz=(0.090, -0.042, 0.0935)),
        material=yellow_stop,
        name="top_stop_right",
    )

    stage_2 = model.part("stage_2")
    stage_2.visual(
        Box((0.400, 0.050, 0.026)),
        origin=Origin(xyz=(-0.130, 0.0, 0.0)),
        material=brushed_aluminum,
        name="final_slider",
    )
    stage_2.visual(
        Box((0.280, 0.026, 0.007)),
        origin=Origin(xyz=(-0.140, 0.0, -0.0165)),
        material=black_wear,
        name="wear_pad",
    )
    stage_2.visual(
        Box((0.064, 0.120, 0.110)),
        origin=Origin(xyz=(0.096, 0.0, 0.020)),
        material=dark_steel,
        name="tool_plate",
    )
    stage_2.visual(
        Box((0.035, 0.084, 0.030)),
        origin=Origin(xyz=(0.045, 0.0, 0.000)),
        material=yellow_stop,
        name="front_stop_bar",
    )
    stage_2.visual(
        Box((0.022, 0.050, 0.024)),
        origin=Origin(xyz=(-0.315, 0.0, 0.006)),
        material=yellow_stop,
        name="rear_stop_bar",
    )

    model.articulation(
        "guide_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=stage_0,
        origin=Origin(xyz=(0.440, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.300),
    )
    model.articulation(
        "stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.120, 0.0, 0.0875)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=0.0, upper=0.220),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.075, 0.0, 0.0625)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.14, lower=0.0, upper=0.160),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    guide = object_model.get_part("guide_body")
    stage_0 = object_model.get_part("stage_0")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    joint_0 = object_model.get_articulation("guide_to_stage_0")
    joint_1 = object_model.get_articulation("stage_0_to_stage_1")
    joint_2 = object_model.get_articulation("stage_1_to_stage_2")

    joints = (joint_0, joint_1, joint_2)
    ctx.check(
        "three prismatic joints",
        len(joints) == 3
        and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in joints),
        details=f"joint types={[joint.articulation_type for joint in joints]}",
    )
    ctx.check(
        "serial slider stack",
        (joint_0.parent, joint_0.child, joint_1.parent, joint_1.child, joint_2.parent, joint_2.child)
        == ("guide_body", "stage_0", "stage_0", "stage_1", "stage_1", "stage_2"),
        details=(
            f"chain={(joint_0.parent, joint_0.child, joint_1.parent, joint_1.child, joint_2.parent, joint_2.child)}"
        ),
    )
    ctx.check(
        "positive x stage travel",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in joints)
        and all(
            joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 0.10
            for joint in joints
        ),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    ctx.expect_contact(
        stage_0,
        guide,
        elem_a="wear_pad_left",
        elem_b="base_plate",
        contact_tol=0.0005,
        name="first stage rides on fixed guide pad",
    )
    ctx.expect_contact(
        stage_1,
        stage_0,
        elem_a="wear_pad_left",
        elem_b="upper_deck",
        contact_tol=0.0005,
        name="second stage rides on first carrier pad",
    )
    ctx.expect_contact(
        stage_2,
        stage_1,
        elem_a="wear_pad",
        elem_b="top_deck",
        contact_tol=0.0005,
        name="third stage rides on second carrier pad",
    )

    ctx.expect_overlap(
        stage_0,
        guide,
        axes="x",
        elem_a="lower_tongue",
        elem_b="fixed_left_rail",
        min_overlap=0.45,
        name="first stage visibly overlaps fixed guide",
    )
    ctx.expect_overlap(
        stage_1,
        stage_0,
        axes="x",
        elem_a="lower_slider",
        elem_b="upper_left_rail",
        min_overlap=0.35,
        name="second stage visibly overlaps first carrier",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="x",
        elem_a="final_slider",
        elem_b="top_left_rail",
        min_overlap=0.25,
        name="third stage visibly overlaps second carrier",
    )

    rest_tip = ctx.part_world_position(stage_2)
    with ctx.pose({joint_0: 0.300, joint_1: 0.220, joint_2: 0.160}):
        ctx.expect_overlap(
            stage_0,
            guide,
            axes="x",
            elem_a="lower_tongue",
            elem_b="fixed_left_rail",
            min_overlap=0.25,
            name="first stage remains retained at full travel",
        )
        ctx.expect_overlap(
            stage_1,
            stage_0,
            axes="x",
            elem_a="lower_slider",
            elem_b="upper_left_rail",
            min_overlap=0.18,
            name="second stage remains retained at full travel",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="x",
            elem_a="final_slider",
            elem_b="top_left_rail",
            min_overlap=0.14,
            name="third stage remains retained at full travel",
        )
        extended_tip = ctx.part_world_position(stage_2)

    ctx.check(
        "stack extends along positive x",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.60,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
