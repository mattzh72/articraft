from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LEN = 0.84
OUTER_WIDTH = 0.20
OUTER_HEIGHT = 0.085
OUTER_BASE_THICKNESS = 0.014
OUTER_WALL_THICKNESS = 0.012
OUTER_REAR_BULKHEAD = 0.020

MIDDLE_LEN = 0.74
MIDDLE_WIDTH = 0.164
MIDDLE_HEIGHT = 0.058
MIDDLE_BASE_THICKNESS = 0.010
MIDDLE_WALL_THICKNESS = 0.010

INNER_LEN = 0.60
INNER_WIDTH = 0.128
INNER_HEIGHT = 0.034
INNER_TOOL_PAD_LEN = 0.120
INNER_TOOL_PAD_WIDTH = 0.140
INNER_TOOL_PAD_HEIGHT = 0.012
INNER_TOOL_PAD_BLEND = 0.004

MIDDLE_CAPTURE_LEN = 0.090
MIDDLE_CAPTURE_HEIGHT = 0.012
MIDDLE_CAPTURE_OVERLAP = 0.006

OUTER_TO_MIDDLE_X = 0.032
OUTER_TO_MIDDLE_Z = OUTER_BASE_THICKNESS
OUTER_TO_MIDDLE_TRAVEL = 0.36

MIDDLE_TO_INNER_X = 0.040
MIDDLE_TO_INNER_Z = MIDDLE_BASE_THICKNESS
MIDDLE_TO_INNER_TRAVEL = 0.30


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_machine_slide")

    model.material("outer_guide_steel", rgba=(0.32, 0.35, 0.38, 1.0))
    model.material("middle_runner_steel", rgba=(0.57, 0.60, 0.63, 1.0))
    model.material("inner_runner_steel", rgba=(0.78, 0.80, 0.82, 1.0))

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        Box((OUTER_LEN, OUTER_WIDTH, OUTER_BASE_THICKNESS)),
        origin=Origin(xyz=(OUTER_LEN / 2.0, 0.0, OUTER_BASE_THICKNESS / 2.0)),
        material="outer_guide_steel",
        name="outer_base",
    )
    outer_guide.visual(
        Box(
            (
                OUTER_LEN - OUTER_REAR_BULKHEAD,
                OUTER_WALL_THICKNESS,
                OUTER_HEIGHT - OUTER_BASE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                OUTER_REAR_BULKHEAD + (OUTER_LEN - OUTER_REAR_BULKHEAD) / 2.0,
                OUTER_WIDTH / 2.0 - OUTER_WALL_THICKNESS / 2.0,
                OUTER_BASE_THICKNESS + (OUTER_HEIGHT - OUTER_BASE_THICKNESS) / 2.0,
            )
        ),
        material="outer_guide_steel",
        name="outer_left_wall",
    )
    outer_guide.visual(
        Box(
            (
                OUTER_LEN - OUTER_REAR_BULKHEAD,
                OUTER_WALL_THICKNESS,
                OUTER_HEIGHT - OUTER_BASE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                OUTER_REAR_BULKHEAD + (OUTER_LEN - OUTER_REAR_BULKHEAD) / 2.0,
                -(OUTER_WIDTH / 2.0 - OUTER_WALL_THICKNESS / 2.0),
                OUTER_BASE_THICKNESS + (OUTER_HEIGHT - OUTER_BASE_THICKNESS) / 2.0,
            )
        ),
        material="outer_guide_steel",
        name="outer_right_wall",
    )
    outer_guide.visual(
        Box((OUTER_REAR_BULKHEAD, OUTER_WIDTH, OUTER_HEIGHT - OUTER_BASE_THICKNESS)),
        origin=Origin(
            xyz=(
                OUTER_REAR_BULKHEAD / 2.0,
                0.0,
                OUTER_BASE_THICKNESS + (OUTER_HEIGHT - OUTER_BASE_THICKNESS) / 2.0,
            )
        ),
        material="outer_guide_steel",
        name="outer_bulkhead",
    )
    outer_guide.inertial = Inertial.from_geometry(
        Box((OUTER_LEN, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=9.5,
        origin=Origin(xyz=(OUTER_LEN / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    middle_runner = model.part("middle_runner")
    middle_runner.visual(
        Box((MIDDLE_LEN, MIDDLE_WIDTH, MIDDLE_BASE_THICKNESS)),
        origin=Origin(xyz=(MIDDLE_LEN / 2.0, 0.0, MIDDLE_BASE_THICKNESS / 2.0)),
        material="middle_runner_steel",
        name="middle_base",
    )
    middle_runner.visual(
        Box(
            (
                MIDDLE_LEN,
                MIDDLE_WALL_THICKNESS,
                MIDDLE_HEIGHT - MIDDLE_BASE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                MIDDLE_LEN / 2.0,
                MIDDLE_WIDTH / 2.0 - MIDDLE_WALL_THICKNESS / 2.0,
                MIDDLE_BASE_THICKNESS + (MIDDLE_HEIGHT - MIDDLE_BASE_THICKNESS) / 2.0,
            )
        ),
        material="middle_runner_steel",
        name="middle_left_wall",
    )
    middle_runner.visual(
        Box(
            (
                MIDDLE_LEN,
                MIDDLE_WALL_THICKNESS,
                MIDDLE_HEIGHT - MIDDLE_BASE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                MIDDLE_LEN / 2.0,
                -(MIDDLE_WIDTH / 2.0 - MIDDLE_WALL_THICKNESS / 2.0),
                MIDDLE_BASE_THICKNESS + (MIDDLE_HEIGHT - MIDDLE_BASE_THICKNESS) / 2.0,
            )
        ),
        material="middle_runner_steel",
        name="middle_right_wall",
    )
    middle_runner.visual(
        Box((MIDDLE_CAPTURE_LEN, MIDDLE_WIDTH, MIDDLE_CAPTURE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_LEN - MIDDLE_CAPTURE_LEN / 2.0,
                0.0,
                MIDDLE_HEIGHT - MIDDLE_CAPTURE_OVERLAP + MIDDLE_CAPTURE_HEIGHT / 2.0,
            )
        ),
        material="middle_runner_steel",
        name="middle_capture_bridge",
    )
    middle_runner.inertial = Inertial.from_geometry(
        Box((MIDDLE_LEN, MIDDLE_WIDTH, MIDDLE_HEIGHT + 0.008)),
        mass=5.8,
        origin=Origin(xyz=(MIDDLE_LEN / 2.0, 0.0, (MIDDLE_HEIGHT + 0.008) / 2.0)),
    )

    inner_runner = model.part("inner_runner")
    inner_runner.visual(
        Box((INNER_LEN, INNER_WIDTH, INNER_HEIGHT)),
        origin=Origin(xyz=(INNER_LEN / 2.0, 0.0, INNER_HEIGHT / 2.0)),
        material="inner_runner_steel",
        name="inner_body",
    )
    inner_runner.visual(
        Box((INNER_TOOL_PAD_LEN, INNER_TOOL_PAD_WIDTH, INNER_TOOL_PAD_HEIGHT)),
        origin=Origin(
            xyz=(
                INNER_LEN - INNER_TOOL_PAD_LEN / 2.0,
                0.0,
                INNER_HEIGHT - INNER_TOOL_PAD_BLEND + INNER_TOOL_PAD_HEIGHT / 2.0,
            )
        ),
        material="inner_runner_steel",
        name="inner_tool_pad",
    )
    inner_runner.inertial = Inertial.from_geometry(
        Box((INNER_LEN, INNER_TOOL_PAD_WIDTH, INNER_HEIGHT + INNER_TOOL_PAD_HEIGHT)),
        mass=3.6,
        origin=Origin(
            xyz=(
                INNER_LEN / 2.0,
                0.0,
                (INNER_HEIGHT + INNER_TOOL_PAD_HEIGHT) / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=middle_runner,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_X, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.45,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_X, 0.0, MIDDLE_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1100.0,
            velocity=0.50,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    required_parts = ("outer_guide", "middle_runner", "inner_runner")
    required_joints = ("outer_to_middle", "middle_to_inner")

    missing_parts: list[str] = []
    for part_name in required_parts:
        try:
            object_model.get_part(part_name)
        except Exception:
            missing_parts.append(part_name)
    ctx.check(
        "required parts present",
        not missing_parts,
        details=f"missing_parts={missing_parts}",
    )

    missing_joints: list[str] = []
    for joint_name in required_joints:
        try:
            object_model.get_articulation(joint_name)
        except Exception:
            missing_joints.append(joint_name)
    ctx.check(
        "required joints present",
        not missing_joints,
        details=f"missing_joints={missing_joints}",
    )

    outer_guide = object_model.get_part("outer_guide")
    middle_runner = object_model.get_part("middle_runner")
    inner_runner = object_model.get_part("inner_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    ctx.expect_within(
        middle_runner,
        outer_guide,
        axes="yz",
        margin=0.0,
        name="middle runner stays nested inside outer guide laterally",
    )
    ctx.expect_contact(
        middle_runner,
        outer_guide,
        name="middle runner is supported on the outer guide ways",
    )
    ctx.expect_overlap(
        middle_runner,
        outer_guide,
        axes="x",
        min_overlap=0.44,
        name="middle runner is retained in the outer guide at rest",
    )
    ctx.expect_within(
        inner_runner,
        middle_runner,
        axes="yz",
        margin=0.0,
        name="inner runner stays nested inside middle runner laterally",
    )
    ctx.expect_contact(
        inner_runner,
        middle_runner,
        name="inner runner is supported on the middle runner ways",
    )
    ctx.expect_overlap(
        inner_runner,
        middle_runner,
        axes="x",
        min_overlap=0.40,
        name="inner runner is retained in the middle runner at rest",
    )

    rest_middle_position = ctx.part_world_position(middle_runner)
    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL}):
        ctx.expect_within(
            middle_runner,
            outer_guide,
            axes="yz",
            margin=0.0,
            name="extended middle runner stays aligned in the outer guide",
        )
        ctx.expect_overlap(
            middle_runner,
            outer_guide,
            axes="x",
            min_overlap=0.44,
            name="extended middle runner keeps retained insertion",
        )
        extended_middle_position = ctx.part_world_position(middle_runner)

    ctx.check(
        "middle runner extends along +X",
        rest_middle_position is not None
        and extended_middle_position is not None
        and extended_middle_position[0] > rest_middle_position[0] + 0.25,
        details=f"rest={rest_middle_position}, extended={extended_middle_position}",
    )

    rest_inner_position = ctx.part_world_position(inner_runner)
    with ctx.pose({middle_to_inner: MIDDLE_TO_INNER_TRAVEL}):
        ctx.expect_within(
            inner_runner,
            middle_runner,
            axes="yz",
            margin=0.0,
            name="extended inner runner stays aligned in the middle runner",
        )
        ctx.expect_overlap(
            inner_runner,
            middle_runner,
            axes="x",
            min_overlap=0.39,
            name="extended inner runner keeps retained insertion",
        )
        extended_inner_position = ctx.part_world_position(inner_runner)

    ctx.check(
        "inner runner extends along +X",
        rest_inner_position is not None
        and extended_inner_position is not None
        and extended_inner_position[0] > rest_inner_position[0] + 0.20,
        details=f"rest={rest_inner_position}, extended={extended_inner_position}",
    )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_within(
            inner_runner,
            middle_runner,
            axes="yz",
            margin=0.0,
            name="fully extended inner runner remains captured laterally",
        )
        ctx.expect_overlap(
            inner_runner,
            middle_runner,
            axes="x",
            min_overlap=0.39,
            name="fully extended inner runner still remains inserted",
        )
        fully_extended_inner_position = ctx.part_world_position(inner_runner)

    ctx.check(
        "three-stage slide gains substantial overall reach",
        rest_inner_position is not None
        and fully_extended_inner_position is not None
        and fully_extended_inner_position[0] > rest_inner_position[0] + 0.55,
        details=f"rest={rest_inner_position}, fully_extended={fully_extended_inner_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
