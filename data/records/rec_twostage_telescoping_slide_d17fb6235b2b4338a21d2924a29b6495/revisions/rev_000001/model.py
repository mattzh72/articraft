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


OUTER_LENGTH = 0.420
INTERMEDIATE_LENGTH = 0.340
CARRIAGE_LENGTH = 0.240

OUTER_TRAVEL = 0.180
CARRIAGE_TRAVEL = 0.160


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_slide")

    model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("light_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("black_polymer", rgba=(0.035, 0.036, 0.038, 1.0))

    fixed = model.part("fixed_rail")
    fixed.visual(
        Box((OUTER_LENGTH, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="dark_steel",
        name="base_web",
    )
    fixed.visual(
        Box((OUTER_LENGTH, 0.007, 0.026)),
        origin=Origin(xyz=(0.0, 0.032, 0.018)),
        material="dark_steel",
        name="side_wall_0",
    )
    fixed.visual(
        Box((OUTER_LENGTH, 0.007, 0.026)),
        origin=Origin(xyz=(0.0, -0.032, 0.018)),
        material="dark_steel",
        name="side_wall_1",
    )
    fixed.visual(
        Box((OUTER_LENGTH, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.022, 0.029)),
        material="dark_steel",
        name="return_lip_0",
    )
    fixed.visual(
        Box((OUTER_LENGTH, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, -0.022, 0.029)),
        material="dark_steel",
        name="return_lip_1",
    )
    fixed.visual(
        Box((0.055, 0.010, 0.0004)),
        origin=Origin(xyz=(-0.130, 0.026, 0.0062)),
        material="black_polymer",
        name="mount_slot_0",
    )
    fixed.visual(
        Box((0.055, 0.010, 0.0004)),
        origin=Origin(xyz=(0.130, -0.026, 0.0062)),
        material="black_polymer",
        name="mount_slot_1",
    )

    intermediate = model.part("intermediate_rail")
    intermediate.visual(
        Box((INTERMEDIATE_LENGTH, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material="brushed_steel",
        name="lower_runner",
    )
    intermediate.visual(
        Box((INTERMEDIATE_LENGTH, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, 0.022, 0.0185)),
        material="brushed_steel",
        name="side_shoulder_0",
    )
    intermediate.visual(
        Box((INTERMEDIATE_LENGTH, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, -0.022, 0.0185)),
        material="brushed_steel",
        name="side_shoulder_1",
    )
    intermediate.visual(
        Box((INTERMEDIATE_LENGTH, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="brushed_steel",
        name="center_rib",
    )
    intermediate.visual(
        Box((0.012, 0.046, 0.014)),
        origin=Origin(xyz=(-INTERMEDIATE_LENGTH / 2.0 + 0.006, 0.0, 0.017)),
        material="black_polymer",
        name="wiper_0",
    )
    intermediate.visual(
        Box((0.012, 0.046, 0.014)),
        origin=Origin(xyz=(INTERMEDIATE_LENGTH / 2.0 - 0.006, 0.0, 0.017)),
        material="black_polymer",
        name="wiper_1",
    )

    carriage = model.part("terminal_carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material="light_aluminum",
        name="flat_plate",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH * 0.92, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material="light_aluminum",
        name="guide_tongue",
    )
    carriage.visual(
        Box((0.052, 0.012, 0.0025)),
        origin=Origin(xyz=(-0.065, 0.0, 0.04025)),
        material="black_polymer",
        name="top_slot_0",
    )
    carriage.visual(
        Box((0.052, 0.012, 0.0025)),
        origin=Origin(xyz=(0.065, 0.0, 0.04025)),
        material="black_polymer",
        name="top_slot_1",
    )

    model.articulation(
        "fixed_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=intermediate,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=OUTER_TRAVEL, effort=120.0, velocity=0.35),
    )
    model.articulation(
        "intermediate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=intermediate,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=CARRIAGE_TRAVEL, effort=90.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_rail")
    intermediate = object_model.get_part("intermediate_rail")
    carriage = object_model.get_part("terminal_carriage")
    fixed_slide = object_model.get_articulation("fixed_to_intermediate")
    carriage_slide = object_model.get_articulation("intermediate_to_carriage")

    ctx.expect_overlap(
        intermediate,
        fixed,
        axes="x",
        min_overlap=0.32,
        name="intermediate rail is deeply stowed in fixed rail",
    )
    ctx.expect_within(
        intermediate,
        fixed,
        axes="yz",
        margin=0.002,
        name="intermediate rail sits inside the fixed channel envelope",
    )
    ctx.expect_overlap(
        carriage,
        intermediate,
        axes="x",
        min_overlap=0.22,
        name="terminal carriage is stowed over the intermediate rail",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({fixed_slide: OUTER_TRAVEL, carriage_slide: CARRIAGE_TRAVEL}):
        ctx.expect_overlap(
            intermediate,
            fixed,
            axes="x",
            min_overlap=0.18,
            name="extended intermediate rail retains overlap with fixed rail",
        )
        ctx.expect_overlap(
            carriage,
            intermediate,
            axes="x",
            min_overlap=0.07,
            name="extended carriage retains overlap with intermediate rail",
        )
        extended_position = ctx.part_world_position(carriage)

    ctx.check(
        "serial prismatic stages extend along common axis",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + OUTER_TRAVEL + CARRIAGE_TRAVEL - 0.005,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
