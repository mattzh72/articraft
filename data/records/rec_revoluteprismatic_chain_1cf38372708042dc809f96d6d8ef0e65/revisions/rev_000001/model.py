from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isfinite, pi

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


BASE_LENGTH = 0.34
BASE_WIDTH = 0.24
BASE_THICKNESS = 0.018

PIVOT_X = -0.092
PIVOT_Z = 0.05
PLINTH_LENGTH = 0.11
PLINTH_WIDTH = 0.13
PLINTH_HEIGHT = 0.012
PLINTH_CENTER_X = PIVOT_X + 0.01

EAR_LENGTH = 0.03
EAR_THICKNESS = 0.012
EAR_HEIGHT = 0.042
EAR_INNER_GAP = 0.06

ARM_BARREL_RADIUS = 0.015
ARM_BARREL_WIDTH = EAR_INNER_GAP
ARM_NECK_LENGTH = 0.11
ARM_NECK_WIDTH = 0.048
ARM_NECK_HEIGHT = 0.016
ARM_NECK_CENTER_X = 0.045
ARM_NECK_CENTER_Z = -0.012

ARM_CHANNEL_START_X = 0.09
ARM_CHANNEL_END_X = 0.39
ARM_CHANNEL_LENGTH = ARM_CHANNEL_END_X - ARM_CHANNEL_START_X
ARM_CHANNEL_OUTER_WIDTH = 0.078
ARM_CHANNEL_WALL_THICKNESS = 0.009
ARM_FLOOR_THICKNESS = 0.006
ARM_FLOOR_TOP_Z = -0.017
ARM_FLOOR_CENTER_Z = ARM_FLOOR_TOP_Z - ARM_FLOOR_THICKNESS / 2.0
ARM_RAIL_HEIGHT = 0.014
ARM_RAIL_CENTER_Z = ARM_FLOOR_CENTER_Z + ARM_FLOOR_THICKNESS / 2.0 + ARM_RAIL_HEIGHT / 2.0

SLIDER_JOINT_X = 0.115
SLIDER_JOINT_Z = ARM_FLOOR_TOP_Z
SLIDER_BODY_LENGTH = 0.095
SLIDER_NOSE_LENGTH = 0.06
SLIDER_WIDTH = 0.046
SLIDER_HEIGHT = 0.012
SLIDER_NOSE_WIDTH = 0.032
SLIDER_NOSE_HEIGHT = 0.01
SLIDER_PAD_LENGTH = 0.018
SLIDER_PAD_WIDTH = 0.026
SLIDER_PAD_HEIGHT = 0.008
SLIDER_TRAVEL = 0.13

ARM_PITCH_LOWER = 0.0
ARM_PITCH_UPPER = 1.05


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_pivot_arm")

    model.material("powder_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machine_gray", rgba=(0.63, 0.66, 0.69, 1.0))
    model.material("anodized_graphite", rgba=(0.28, 0.30, 0.33, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="powder_black",
        name="base_plate",
    )
    base.visual(
        Box((PLINTH_LENGTH, PLINTH_WIDTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(PLINTH_CENTER_X, 0.0, BASE_THICKNESS + PLINTH_HEIGHT / 2.0)),
        material="powder_black",
        name="mount_plinth",
    )
    ear_center_y = EAR_INNER_GAP / 2.0 + EAR_THICKNESS / 2.0
    for side, sign in (("left", 1.0), ("right", -1.0)):
        base.visual(
            Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
            origin=Origin(
                xyz=(
                    PIVOT_X,
                    sign * ear_center_y,
                    BASE_THICKNESS + PLINTH_HEIGHT + EAR_HEIGHT / 2.0,
                )
            ),
            material="powder_black",
            name=f"{side}_ear",
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=ARM_BARREL_RADIUS, length=ARM_BARREL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="machine_gray",
        name="hinge_barrel",
    )
    arm.visual(
        Box((ARM_NECK_LENGTH, ARM_NECK_WIDTH, ARM_NECK_HEIGHT)),
        origin=Origin(xyz=(ARM_NECK_CENTER_X, 0.0, ARM_NECK_CENTER_Z)),
        material="machine_gray",
        name="arm_neck",
    )
    arm.visual(
        Box((ARM_CHANNEL_LENGTH, ARM_CHANNEL_OUTER_WIDTH, ARM_FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                ARM_CHANNEL_START_X + ARM_CHANNEL_LENGTH / 2.0,
                0.0,
                ARM_FLOOR_CENTER_Z,
            )
        ),
        material="machine_gray",
        name="guide_floor",
    )
    rail_center_y = ARM_CHANNEL_OUTER_WIDTH / 2.0 - ARM_CHANNEL_WALL_THICKNESS / 2.0
    arm.visual(
        Box((ARM_CHANNEL_LENGTH, ARM_CHANNEL_WALL_THICKNESS, ARM_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                ARM_CHANNEL_START_X + ARM_CHANNEL_LENGTH / 2.0,
                rail_center_y,
                ARM_RAIL_CENTER_Z,
            )
        ),
        material="machine_gray",
        name="left_rail",
    )
    arm.visual(
        Box((ARM_CHANNEL_LENGTH, ARM_CHANNEL_WALL_THICKNESS, ARM_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                ARM_CHANNEL_START_X + ARM_CHANNEL_LENGTH / 2.0,
                -rail_center_y,
                ARM_RAIL_CENTER_Z,
            )
        ),
        material="machine_gray",
        name="right_rail",
    )

    slider = model.part("slider")
    slider.visual(
        Box((SLIDER_BODY_LENGTH, SLIDER_WIDTH, SLIDER_HEIGHT)),
        origin=Origin(xyz=(SLIDER_BODY_LENGTH / 2.0, 0.0, SLIDER_HEIGHT / 2.0)),
        material="anodized_graphite",
        name="slider_body",
    )
    slider.visual(
        Box((SLIDER_NOSE_LENGTH, SLIDER_NOSE_WIDTH, SLIDER_NOSE_HEIGHT)),
        origin=Origin(
            xyz=(
                SLIDER_BODY_LENGTH + SLIDER_NOSE_LENGTH / 2.0 - 0.004,
                0.0,
                SLIDER_NOSE_HEIGHT / 2.0,
            )
        ),
        material="anodized_graphite",
        name="slider_nose",
    )
    slider.visual(
        Box((SLIDER_PAD_LENGTH, SLIDER_PAD_WIDTH, SLIDER_PAD_HEIGHT)),
        origin=Origin(
            xyz=(
                SLIDER_BODY_LENGTH + SLIDER_NOSE_LENGTH - SLIDER_PAD_LENGTH / 2.0,
                0.0,
                SLIDER_PAD_HEIGHT / 2.0,
            )
        ),
        material="anodized_graphite",
        name="terminal_pad",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=ARM_PITCH_LOWER,
            upper=ARM_PITCH_UPPER,
        ),
    )

    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slider,
        origin=Origin(xyz=(SLIDER_JOINT_X, 0.0, SLIDER_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.22,
            lower=0.0,
            upper=SLIDER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    slider = object_model.get_part("slider")
    base_to_arm = object_model.get_articulation("base_to_arm")
    arm_to_slider = object_model.get_articulation("arm_to_slider")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        arm,
        base,
        contact_tol=0.001,
        name="arm hinge barrel is physically supported by the base clevis",
    )
    ctx.expect_contact(
        slider,
        arm,
        contact_tol=0.001,
        name="terminal slider rides on the arm guide surfaces",
    )
    ctx.expect_within(
        slider,
        arm,
        axes="yz",
        margin=0.0,
        name="slider stays within the arm guide envelope",
    )

    with ctx.pose({arm_to_slider: SLIDER_TRAVEL}):
        ctx.expect_contact(
            slider,
            arm,
            contact_tol=0.001,
            name="slider remains supported at full extension",
        )
        ctx.expect_within(
            slider,
            arm,
            axes="yz",
            margin=0.0,
            name="slider remains laterally captured when extended",
        )

    with ctx.pose({base_to_arm: 0.0, arm_to_slider: 0.0}):
        retracted_position = ctx.part_world_position(slider)
    with ctx.pose({base_to_arm: 0.0, arm_to_slider: SLIDER_TRAVEL}):
        extended_position = ctx.part_world_position(slider)

    positions_ready = (
        retracted_position is not None
        and extended_position is not None
        and all(isfinite(value) for value in retracted_position)
        and all(isfinite(value) for value in extended_position)
    )
    if not positions_ready:
        ctx.fail(
            "slider motion positions resolve",
            "Could not resolve stable slider positions for retracted and extended poses.",
        )
    else:
        x_gain = extended_position[0] - retracted_position[0]
        ctx.check(
            "positive slider travel extends the nose forward",
            x_gain > SLIDER_TRAVEL - 0.01,
            details=f"Expected forward x motion > {SLIDER_TRAVEL - 0.01:.3f} m, got {x_gain:.3f} m.",
        )

    with ctx.pose({base_to_arm: 0.0, arm_to_slider: 0.0}):
        arm_low_pose = ctx.part_world_position(slider)
    with ctx.pose({base_to_arm: ARM_PITCH_UPPER * 0.85, arm_to_slider: 0.0}):
        arm_high_pose = ctx.part_world_position(slider)

    lift_ready = (
        arm_low_pose is not None
        and arm_high_pose is not None
        and all(isfinite(value) for value in arm_low_pose)
        and all(isfinite(value) for value in arm_high_pose)
    )
    if not lift_ready:
        ctx.fail(
            "arm lift positions resolve",
            "Could not resolve stable slider positions for low and raised arm poses.",
        )
    else:
        z_gain = arm_high_pose[2] - arm_low_pose[2]
        ctx.check(
            "positive hinge rotation lifts the arm",
            z_gain > 0.08,
            details=f"Expected raised-pose z gain > 0.080 m, got {z_gain:.3f} m.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
