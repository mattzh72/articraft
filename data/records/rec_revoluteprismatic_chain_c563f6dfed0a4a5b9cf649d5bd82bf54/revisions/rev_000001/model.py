from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LENGTH = 0.18
BASE_WIDTH = 0.14
BASE_THICKNESS = 0.02

PEDESTAL_LENGTH = 0.07
PEDESTAL_WIDTH = 0.09
PEDESTAL_HEIGHT = 0.05

UPRIGHT_LENGTH = 0.028
UPRIGHT_WIDTH = 0.014
UPRIGHT_HEIGHT = 0.10

PIVOT_Z = 0.155
CHEEK_LENGTH = 0.05
CHEEK_THICKNESS = 0.012
CHEEK_HEIGHT = 0.075
ARM_WIDTH = 0.032
CHEEK_CENTER_Y = ARM_WIDTH / 2.0 + CHEEK_THICKNESS / 2.0

ARM_BODY_END_X = 0.25
ARM_NOSE_END_X = 0.34
ARM_WINDOW_CENTER_X = 0.15
ARM_WINDOW_LENGTH = 0.11

SLIDE_SLEEVE_START_X = 0.26
SLIDE_SLEEVE_END_X = 0.34
SLIDE_SLEEVE_LENGTH = SLIDE_SLEEVE_END_X - SLIDE_SLEEVE_START_X
SLIDE_BAR_LENGTH = 0.15
SLIDE_BAR_WIDTH = 0.022
SLIDE_BAR_HEIGHT = 0.024
SLIDE_TRAVEL = 0.04

ARM_SWING_LOWER = radians(-20.0)
ARM_SWING_UPPER = radians(68.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_extension_chain")

    model.material("powder_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("aluminum_arm", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("slide_steel", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("zinc_cap", rgba=(0.78, 0.80, 0.84, 1.0))

    support = model.part("support")
    support.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="powder_steel",
        name="base_plate",
    )
    support.visual(
        Box((PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT)),
        origin=Origin(xyz=(-0.020, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0)),
        material="machined_dark",
        name="pedestal",
    )
    support.visual(
        Box((UPRIGHT_LENGTH, UPRIGHT_WIDTH, UPRIGHT_HEIGHT)),
        origin=Origin(xyz=(-0.006, CHEEK_CENTER_Y, BASE_THICKNESS + UPRIGHT_HEIGHT / 2.0)),
        material="powder_steel",
        name="left_upright",
    )
    support.visual(
        Box((UPRIGHT_LENGTH, UPRIGHT_WIDTH, UPRIGHT_HEIGHT)),
        origin=Origin(xyz=(-0.006, -CHEEK_CENTER_Y, BASE_THICKNESS + UPRIGHT_HEIGHT / 2.0)),
        material="powder_steel",
        name="right_upright",
    )
    support.visual(
        Box((CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.004, CHEEK_CENTER_Y, PIVOT_Z)),
        material="powder_steel",
        name="left_cheek",
    )
    support.visual(
        Box((CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.004, -CHEEK_CENTER_Y, PIVOT_Z)),
        material="powder_steel",
        name="right_cheek",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.055, -0.040),
            (-0.055, 0.040),
            (0.055, -0.040),
            (0.055, 0.040),
        ),
        start=1,
    ):
        support.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, BASE_THICKNESS + 0.002)),
            material="zinc_cap",
            name=f"bolt_cap_{index}",
        )
    support.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, PIVOT_Z + CHEEK_HEIGHT / 2.0)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, (PIVOT_Z + CHEEK_HEIGHT / 2.0) / 2.0)),
    )

    arm = model.part("arm_link")
    arm.visual(
        Cylinder(radius=0.024, length=ARM_WIDTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="aluminum_arm",
        name="root_hub",
    )
    arm.visual(
        Box((0.040, 0.024, 0.028)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material="aluminum_arm",
        name="root_spine",
    )
    arm.visual(
        Box((0.192, ARM_WIDTH, 0.038)),
        origin=Origin(xyz=(0.156, 0.0, 0.0)),
        material="aluminum_arm",
        name="main_beam",
    )
    arm.visual(
        Box((0.008, ARM_WIDTH, 0.044)),
        origin=Origin(xyz=(SLIDE_SLEEVE_START_X - 0.004, 0.0, 0.0)),
        material="aluminum_arm",
        name="sleeve_rear_wall",
    )
    arm.visual(
        Box((SLIDE_SLEEVE_LENGTH, ARM_WIDTH, 0.006)),
        origin=Origin(xyz=(SLIDE_SLEEVE_START_X + SLIDE_SLEEVE_LENGTH / 2.0, 0.0, 0.019)),
        material="aluminum_arm",
        name="sleeve_top_wall",
    )
    arm.visual(
        Box((SLIDE_SLEEVE_LENGTH, ARM_WIDTH, 0.006)),
        origin=Origin(xyz=(SLIDE_SLEEVE_START_X + SLIDE_SLEEVE_LENGTH / 2.0, 0.0, -0.019)),
        material="aluminum_arm",
        name="sleeve_bottom_wall",
    )
    arm.visual(
        Box((SLIDE_SLEEVE_LENGTH, 0.005, 0.032)),
        origin=Origin(xyz=(SLIDE_SLEEVE_START_X + SLIDE_SLEEVE_LENGTH / 2.0, 0.0135, 0.0)),
        material="aluminum_arm",
        name="sleeve_left_wall",
    )
    arm.visual(
        Box((SLIDE_SLEEVE_LENGTH, 0.005, 0.032)),
        origin=Origin(xyz=(SLIDE_SLEEVE_START_X + SLIDE_SLEEVE_LENGTH / 2.0, -0.0135, 0.0)),
        material="aluminum_arm",
        name="sleeve_right_wall",
    )
    arm.inertial = Inertial.from_geometry(
        Box((ARM_NOSE_END_X, ARM_WIDTH, 0.06)),
        mass=1.9,
        origin=Origin(xyz=(ARM_NOSE_END_X / 2.0, 0.0, 0.0)),
    )

    slide = model.part("nose_slide")
    slide.visual(
        Box((SLIDE_BAR_LENGTH, SLIDE_BAR_WIDTH, SLIDE_BAR_HEIGHT)),
        origin=Origin(xyz=(SLIDE_BAR_LENGTH / 2.0, 0.0, 0.0)),
        material="slide_steel",
        name="slide_bar",
    )
    slide.visual(
        Box((0.018, 0.030, 0.032)),
        origin=Origin(xyz=(SLIDE_BAR_LENGTH - 0.009, 0.0, 0.0)),
        material="slide_steel",
        name="nose_pad",
    )
    slide.inertial = Inertial.from_geometry(
        Box((SLIDE_BAR_LENGTH, 0.03, 0.03)),
        mass=0.5,
        origin=Origin(xyz=(SLIDE_BAR_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_arm",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ARM_SWING_LOWER,
            upper=ARM_SWING_UPPER,
            effort=35.0,
            velocity=1.3,
        ),
    )
    model.articulation(
        "arm_to_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slide,
        origin=Origin(xyz=(SLIDE_SLEEVE_START_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=22.0,
            velocity=0.18,
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

    support = object_model.get_part("support")
    arm = object_model.get_part("arm_link")
    slide = object_model.get_part("nose_slide")
    arm_joint = object_model.get_articulation("support_to_arm")
    slide_joint = object_model.get_articulation("arm_to_slide")

    ctx.expect_contact(
        arm,
        support,
        elem_a="root_hub",
        elem_b="left_cheek",
        name="arm hub bears on the left support cheek",
    )
    ctx.expect_contact(
        arm,
        support,
        elem_a="root_hub",
        elem_b="right_cheek",
        name="arm hub bears on the right support cheek",
    )
    ctx.expect_contact(
        slide,
        arm,
        elem_a="slide_bar",
        elem_b="sleeve_left_wall",
        name="slide bar is guided by the left sleeve wall",
    )
    ctx.expect_contact(
        slide,
        arm,
        elem_a="slide_bar",
        elem_b="sleeve_right_wall",
        name="slide bar is guided by the right sleeve wall",
    )

    ctx.expect_within(
        slide,
        arm,
        axes="yz",
        margin=0.0,
        name="nose slide stays centered inside the arm sleeve at rest",
    )
    ctx.expect_overlap(
        slide,
        arm,
        axes="x",
        min_overlap=SLIDE_SLEEVE_LENGTH,
        name="nose slide is deeply inserted in the arm sleeve at rest",
    )

    rest_slide_pos = ctx.part_world_position(slide)
    with ctx.pose({slide_joint: SLIDE_TRAVEL}):
        ctx.expect_within(
            slide,
            arm,
            axes="yz",
            margin=0.0,
            name="extended nose slide stays centered in the sleeve",
        )
        ctx.expect_overlap(
            slide,
            arm,
            axes="x",
            min_overlap=SLIDE_SLEEVE_LENGTH - SLIDE_TRAVEL,
            name="extended nose slide still retains insertion",
        )
        extended_slide_pos = ctx.part_world_position(slide)

    ctx.check(
        "nose slide extends forward along +X",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.03,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    rest_nose_pos = ctx.part_world_position(slide)
    with ctx.pose({arm_joint: ARM_SWING_UPPER}):
        raised_nose_pos = ctx.part_world_position(slide)

    ctx.check(
        "arm swings the nose upward",
        rest_nose_pos is not None
        and raised_nose_pos is not None
        and raised_nose_pos[2] > rest_nose_pos[2] + 0.10
        and raised_nose_pos[0] < rest_nose_pos[0] - 0.03,
        details=f"rest={rest_nose_pos}, raised={raised_nose_pos}",
    )

    ctx.check(
        "all prompt-critical parts resolve",
        support is not None and arm is not None and slide is not None,
        details="support, arm_link, and nose_slide should all exist",
    )

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
