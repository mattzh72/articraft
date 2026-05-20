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


BASE_LENGTH = 0.18
BASE_WIDTH = 0.14
BASE_THICKNESS = 0.02

BACK_WEB_LENGTH = 0.03
BACK_WEB_HEIGHT = 0.12

ARM_LENGTH = 0.24
ARM_WIDTH = 0.11
ARM_THICKNESS = 0.02

GUIDE_LENGTH = 0.22
GUIDE_THICKNESS = 0.01
SLOT_CENTER_Z = 0.09
INNER_SLOT_HEIGHT = 0.05
INNER_SLOT_WIDTH = 0.09
PAD_WIDTH = 0.014

RUNNER_LENGTH = 0.30
RUNNER_WIDTH = 0.07
RUNNER_HEIGHT = 0.03

SLIDE_TRAVEL = 0.10
SLIDE_JOINT_X = -0.054

TOP_CLEARANCE = (INNER_SLOT_HEIGHT - RUNNER_HEIGHT) / 2.0
SIDE_CLEARANCE = (INNER_SLOT_WIDTH - RUNNER_WIDTH) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_captured_linear_runner")

    model.material("frame_paint", rgba=(0.25, 0.28, 0.31, 1.0))
    model.material("runner_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(-0.01, 0.0, BASE_THICKNESS / 2.0)),
        material="frame_paint",
        name="base_foot",
    )
    frame.visual(
        Box((BACK_WEB_LENGTH, BASE_WIDTH, BACK_WEB_HEIGHT)),
        origin=Origin(
            xyz=(
                (-0.08 - BACK_WEB_LENGTH / 2.0),
                0.0,
                BASE_THICKNESS + (BACK_WEB_HEIGHT / 2.0),
            )
        ),
        material="frame_paint",
        name="back_web",
    )
    frame.visual(
        Box((ARM_LENGTH, ARM_WIDTH, ARM_THICKNESS)),
        origin=Origin(xyz=(0.04, 0.0, SLOT_CENTER_Z - (INNER_SLOT_HEIGHT / 2.0) - (ARM_THICKNESS / 2.0))),
        material="frame_paint",
        name="lower_arm",
    )
    frame.visual(
        Box((ARM_LENGTH, ARM_WIDTH, ARM_THICKNESS)),
        origin=Origin(xyz=(0.04, 0.0, SLOT_CENTER_Z + (INNER_SLOT_HEIGHT / 2.0) + (ARM_THICKNESS / 2.0))),
        material="frame_paint",
        name="upper_arm",
    )
    frame.visual(
        Box((GUIDE_LENGTH, GUIDE_THICKNESS, INNER_SLOT_HEIGHT + (2.0 * ARM_THICKNESS))),
        origin=Origin(
            xyz=(
                0.03,
                (INNER_SLOT_WIDTH / 2.0) + (GUIDE_THICKNESS / 2.0),
                SLOT_CENTER_Z,
            )
        ),
        material="frame_paint",
        name="left_guide",
    )
    frame.visual(
        Box((GUIDE_LENGTH, GUIDE_THICKNESS, INNER_SLOT_HEIGHT + (2.0 * ARM_THICKNESS))),
        origin=Origin(
            xyz=(
                0.03,
                -((INNER_SLOT_WIDTH / 2.0) + (GUIDE_THICKNESS / 2.0)),
                SLOT_CENTER_Z,
            )
        ),
        material="frame_paint",
        name="right_guide",
    )
    frame.visual(
        Box((GUIDE_LENGTH, PAD_WIDTH, TOP_CLEARANCE)),
        origin=Origin(
            xyz=(
                0.03,
                0.02,
                SLOT_CENTER_Z - (INNER_SLOT_HEIGHT / 2.0) + (TOP_CLEARANCE / 2.0),
            )
        ),
        material="runner_steel",
        name="left_pad",
    )
    frame.visual(
        Box((GUIDE_LENGTH, PAD_WIDTH, TOP_CLEARANCE)),
        origin=Origin(
            xyz=(
                0.03,
                -0.02,
                SLOT_CENTER_Z - (INNER_SLOT_HEIGHT / 2.0) + (TOP_CLEARANCE / 2.0),
            )
        ),
        material="runner_steel",
        name="right_pad",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.27, BASE_WIDTH, BACK_WEB_HEIGHT + BASE_THICKNESS)),
        mass=5.5,
        origin=Origin(xyz=(0.025, 0.0, 0.07)),
    )

    runner = model.part("runner")
    runner.visual(
        Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(RUNNER_LENGTH / 2.0, 0.0, 0.0)),
        material="runner_steel",
        name="runner_body",
    )
    runner.inertial = Inertial.from_geometry(
        Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(RUNNER_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_runner",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=runner,
        origin=Origin(xyz=(SLIDE_JOINT_X, 0.0, SLOT_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=150.0,
            velocity=0.25,
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

    frame = object_model.get_part("frame")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("frame_to_runner")

    upper_arm = frame.get_visual("upper_arm")
    lower_arm = frame.get_visual("lower_arm")
    left_guide = frame.get_visual("left_guide")
    right_guide = frame.get_visual("right_guide")
    left_pad = frame.get_visual("left_pad")
    runner_body = runner.get_visual("runner_body")

    ctx.expect_contact(
        frame,
        runner,
        elem_a=left_pad,
        elem_b=runner_body,
        name="runner is supported by the lower wear pad",
    )
    ctx.expect_gap(
        frame,
        runner,
        axis="z",
        min_gap=TOP_CLEARANCE - 0.001,
        max_gap=TOP_CLEARANCE + 0.001,
        positive_elem=upper_arm,
        negative_elem=runner_body,
        name="runner clears upper arm",
    )
    ctx.expect_gap(
        runner,
        frame,
        axis="z",
        min_gap=TOP_CLEARANCE - 0.001,
        max_gap=TOP_CLEARANCE + 0.001,
        positive_elem=runner_body,
        negative_elem=lower_arm,
        name="runner clears lower arm",
    )
    ctx.expect_gap(
        frame,
        runner,
        axis="y",
        min_gap=SIDE_CLEARANCE - 0.001,
        max_gap=SIDE_CLEARANCE + 0.001,
        positive_elem=left_guide,
        negative_elem=runner_body,
        name="runner clears left guide",
    )
    ctx.expect_gap(
        runner,
        frame,
        axis="y",
        min_gap=SIDE_CLEARANCE - 0.001,
        max_gap=SIDE_CLEARANCE + 0.001,
        positive_elem=runner_body,
        negative_elem=right_guide,
        name="runner clears right guide",
    )
    ctx.expect_overlap(
        runner,
        frame,
        axes="x",
        min_overlap=0.19,
        elem_a=runner_body,
        elem_b=left_guide,
        name="runner remains deeply inserted at rest",
    )

    rest_pos = ctx.part_world_position(runner)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            frame,
            runner,
            elem_a=left_pad,
            elem_b=runner_body,
            name="runner stays supported when extended",
        )
        ctx.expect_gap(
            frame,
            runner,
            axis="z",
            min_gap=TOP_CLEARANCE - 0.001,
            max_gap=TOP_CLEARANCE + 0.001,
            positive_elem=upper_arm,
            negative_elem=runner_body,
            name="runner keeps upper clearance extended",
        )
        ctx.expect_gap(
            frame,
            runner,
            axis="y",
            min_gap=SIDE_CLEARANCE - 0.001,
            max_gap=SIDE_CLEARANCE + 0.001,
            positive_elem=left_guide,
            negative_elem=runner_body,
            name="runner keeps left clearance extended",
        )
        ctx.expect_overlap(
            runner,
            frame,
            axes="x",
            min_overlap=0.09,
            elem_a=runner_body,
            elem_b=left_guide,
            name="runner stays captured at full extension",
        )
        extended_pos = ctx.part_world_position(runner)

    ctx.check(
        "runner extends forward along +x",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + (SLIDE_TRAVEL - 0.005),
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
