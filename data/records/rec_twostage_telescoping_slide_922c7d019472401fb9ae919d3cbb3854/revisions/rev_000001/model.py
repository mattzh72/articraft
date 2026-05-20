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


OUTER_LENGTH = 0.460
OUTER_WIDTH = 0.055
OUTER_HEIGHT = 0.045
OUTER_WALL = 0.003
OUTER_REAR_BULKHEAD = 0.018

MIDDLE_LENGTH = 0.400
MIDDLE_WIDTH = 0.047
MIDDLE_HEIGHT = 0.030
MIDDLE_WALL = 0.0025
MIDDLE_RUNNER_HEIGHT = 0.003
MIDDLE_RUNNER_WIDTH = 0.006
MIDDLE_REAR_BULKHEAD = 0.012

INNER_LENGTH = 0.270
INNER_WIDTH = 0.028
INNER_HEIGHT = 0.014
INNER_FACE_THICKNESS = 0.004
INNER_FACE_WIDTH = 0.036
INNER_FACE_HEIGHT = 0.022

OUTER_TO_MIDDLE_HOME_X = 0.028
OUTER_TO_MIDDLE_TRAVEL = 0.180
MIDDLE_TO_INNER_HOME_X = 0.056
MIDDLE_TO_INNER_TRAVEL = 0.150
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_drawer_slide")

    model.material("outer_steel", rgba=(0.25, 0.28, 0.31, 1.0))
    model.material("zinc_stage", rgba=(0.68, 0.71, 0.75, 1.0))
    model.material("output_steel", rgba=(0.82, 0.84, 0.87, 1.0))

    outer = model.part("outer_channel")
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_WALL)),
        material="outer_steel",
        origin=Origin(xyz=(OUTER_LENGTH * 0.5, 0.0, OUTER_WALL * 0.5)),
        name="outer_bottom",
    )
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT - OUTER_WALL)),
        material="outer_steel",
        origin=Origin(
            xyz=(
                OUTER_LENGTH * 0.5,
                -(OUTER_WIDTH - OUTER_WALL) * 0.5,
                OUTER_WALL + (OUTER_HEIGHT - OUTER_WALL) * 0.5,
            )
        ),
        name="outer_left_wall",
    )
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT - OUTER_WALL)),
        material="outer_steel",
        origin=Origin(
            xyz=(
                OUTER_LENGTH * 0.5,
                (OUTER_WIDTH - OUTER_WALL) * 0.5,
                OUTER_WALL + (OUTER_HEIGHT - OUTER_WALL) * 0.5,
            )
        ),
        name="outer_right_wall",
    )
    outer.visual(
        Box((OUTER_REAR_BULKHEAD, OUTER_WIDTH - 2.0 * OUTER_WALL, OUTER_HEIGHT - OUTER_WALL)),
        material="outer_steel",
        origin=Origin(
            xyz=(
                OUTER_REAR_BULKHEAD * 0.5,
                0.0,
                OUTER_WALL + (OUTER_HEIGHT - OUTER_WALL) * 0.5,
            )
        ),
        name="outer_rear_bulkhead",
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(OUTER_LENGTH * 0.5, 0.0, OUTER_HEIGHT * 0.5)),
    )

    middle = model.part("middle_stage")
    middle.visual(
        Box((MIDDLE_LENGTH * 0.94, MIDDLE_RUNNER_WIDTH, MIDDLE_RUNNER_HEIGHT)),
        material="zinc_stage",
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH * 0.5,
                -(MIDDLE_WIDTH - MIDDLE_RUNNER_WIDTH) * 0.5,
                MIDDLE_RUNNER_HEIGHT * 0.5,
            )
        ),
        name="middle_left_runner",
    )
    middle.visual(
        Box((MIDDLE_LENGTH * 0.94, MIDDLE_RUNNER_WIDTH, MIDDLE_RUNNER_HEIGHT)),
        material="zinc_stage",
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH * 0.5,
                (MIDDLE_WIDTH - MIDDLE_RUNNER_WIDTH) * 0.5,
                MIDDLE_RUNNER_HEIGHT * 0.5,
            )
        ),
        name="middle_right_runner",
    )
    middle.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_WALL)),
        material="zinc_stage",
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH * 0.5,
                0.0,
                MIDDLE_RUNNER_HEIGHT + MIDDLE_WALL * 0.5,
            )
        ),
        name="middle_floor",
    )
    middle.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT - MIDDLE_RUNNER_HEIGHT - MIDDLE_WALL)),
        material="zinc_stage",
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH * 0.5,
                -(MIDDLE_WIDTH - MIDDLE_WALL) * 0.5,
                MIDDLE_RUNNER_HEIGHT
                + MIDDLE_WALL
                + (MIDDLE_HEIGHT - MIDDLE_RUNNER_HEIGHT - MIDDLE_WALL) * 0.5,
            )
        ),
        name="middle_left_wall",
    )
    middle.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT - MIDDLE_RUNNER_HEIGHT - MIDDLE_WALL)),
        material="zinc_stage",
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH * 0.5,
                (MIDDLE_WIDTH - MIDDLE_WALL) * 0.5,
                MIDDLE_RUNNER_HEIGHT
                + MIDDLE_WALL
                + (MIDDLE_HEIGHT - MIDDLE_RUNNER_HEIGHT - MIDDLE_WALL) * 0.5,
            )
        ),
        name="middle_right_wall",
    )
    middle.visual(
        Box(
            (
                MIDDLE_REAR_BULKHEAD,
                MIDDLE_WIDTH - 2.0 * MIDDLE_WALL,
                MIDDLE_HEIGHT - MIDDLE_RUNNER_HEIGHT - MIDDLE_WALL,
            )
        ),
        material="zinc_stage",
        origin=Origin(
            xyz=(
                MIDDLE_REAR_BULKHEAD * 0.5,
                0.0,
                MIDDLE_RUNNER_HEIGHT
                + MIDDLE_WALL
                + (MIDDLE_HEIGHT - MIDDLE_RUNNER_HEIGHT - MIDDLE_WALL) * 0.5,
            )
        ),
        name="middle_rear_bulkhead",
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=0.9,
        origin=Origin(xyz=(MIDDLE_LENGTH * 0.5, 0.0, MIDDLE_HEIGHT * 0.5)),
    )

    output = model.part("output_stage")
    output.visual(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        material="output_steel",
        origin=Origin(xyz=(INNER_LENGTH * 0.5, 0.0, INNER_HEIGHT * 0.5)),
        name="output_rail",
    )
    output.visual(
        Box((INNER_FACE_THICKNESS, INNER_FACE_WIDTH, INNER_FACE_HEIGHT)),
        material="output_steel",
        origin=Origin(
            xyz=(
                INNER_LENGTH - INNER_FACE_THICKNESS * 0.5,
                0.0,
                INNER_FACE_HEIGHT * 0.5,
            )
        ),
        name="output_face",
    )
    output.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_FACE_WIDTH, INNER_FACE_HEIGHT)),
        mass=0.45,
        origin=Origin(
            xyz=(INNER_LENGTH * 0.5, 0.0, INNER_FACE_HEIGHT * 0.5),
        ),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME_X, 0.0, OUTER_WALL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
            effort=160.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "middle_to_output",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=output,
        origin=Origin(
            xyz=(MIDDLE_TO_INNER_HOME_X, 0.0, MIDDLE_RUNNER_HEIGHT + MIDDLE_WALL),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
            effort=110.0,
            velocity=0.40,
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

    outer = object_model.get_part("outer_channel")
    middle = object_model.get_part("middle_stage")
    output = object_model.get_part("output_stage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_output = object_model.get_articulation("middle_to_output")

    ctx.check(
        "expected parts resolve",
        all(part is not None for part in (outer, middle, output)),
    )
    ctx.check(
        "slide joints are prismatic along +X",
        outer_to_middle.joint_type == ArticulationType.PRISMATIC
        and middle_to_output.joint_type == ArticulationType.PRISMATIC
        and outer_to_middle.axis == (1.0, 0.0, 0.0)
        and middle_to_output.axis == (1.0, 0.0, 0.0),
        details=(
            f"outer_to_middle={outer_to_middle.joint_type, outer_to_middle.axis}, "
            f"middle_to_output={middle_to_output.joint_type, middle_to_output.axis}"
        ),
    )

    ctx.expect_contact(
        middle,
        outer,
        name="middle stage sits on the outer channel runners",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0015,
        name="middle stage is laterally contained by the outer channel",
    )
    ctx.expect_contact(
        output,
        middle,
        name="output stage sits on the middle stage floor",
    )
    ctx.expect_within(
        output,
        middle,
        axes="yz",
        margin=0.0015,
        name="output stage stays centered inside the middle stage",
    )

    middle_rest = ctx.part_world_position(middle)
    output_rest = ctx.part_world_position(output)

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL}):
        ctx.expect_contact(
            middle,
            outer,
            name="extended middle stage remains supported by the outer channel",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0015,
            name="extended middle stage remains laterally captured",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.22,
            name="middle stage retains insertion at full first-stage extension",
        )
        middle_extended = ctx.part_world_position(middle)

    ctx.check(
        "middle stage extends forward",
        middle_rest is not None
        and middle_extended is not None
        and middle_extended[0] > middle_rest[0] + 0.16,
        details=f"rest={middle_rest}, extended={middle_extended}",
    )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_output: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_contact(
            output,
            middle,
            name="fully extended output stage remains supported by the middle stage",
        )
        ctx.expect_within(
            output,
            middle,
            axes="yz",
            margin=0.0015,
            name="fully extended output stage stays laterally guided",
        )
        ctx.expect_overlap(
            output,
            middle,
            axes="x",
            min_overlap=0.10,
            name="output stage retains insertion at full second-stage extension",
        )
        output_extended = ctx.part_world_position(output)

    ctx.check(
        "output stage extends beyond the first stage",
        output_rest is not None
        and output_extended is not None
        and output_extended[0] > output_rest[0] + 0.30,
        details=f"rest={output_rest}, extended={output_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
