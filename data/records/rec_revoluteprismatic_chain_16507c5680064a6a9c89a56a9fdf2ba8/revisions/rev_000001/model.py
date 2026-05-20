from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_FOOT_L = 0.16
BASE_FOOT_W = 0.09
BASE_FOOT_T = 0.012
BASE_FOOT_X = -0.03
HINGE_AXIS_Z = 0.092

BRACKET_UPRIGHT_L = 0.018
BRACKET_UPRIGHT_W = 0.05
BRACKET_UPRIGHT_H = 0.07
BRACKET_UPRIGHT_X = -0.042
BRACKET_UPRIGHT_Z = BASE_FOOT_T + (BRACKET_UPRIGHT_H / 2.0)

BRACKET_EAR_L = 0.012
BRACKET_EAR_W = 0.008
BRACKET_EAR_H = 0.03
BRACKET_EAR_Y = 0.014

BRACKET_BRACE_L = 0.05
BRACKET_BRACE_W = 0.008
BRACKET_BRACE_H = 0.012
BRACKET_BRACE_X = -0.025
BRACKET_BRACE_Z = 0.086

HINGE_BARREL_R = 0.012
HINGE_BARREL_L = 0.02

ARM_ROOT_BLOCK_L = 0.04
ARM_ROOT_BLOCK_W = 0.022
ARM_ROOT_BLOCK_H = 0.03
ARM_ROOT_BLOCK_X = 0.025
ARM_ROOT_BLOCK_Z = -0.015

ARM_BEAM_L = 0.22
ARM_BEAM_W = 0.036
ARM_BEAM_H = 0.016
ARM_BEAM_X = 0.15
ARM_BEAM_Z = -0.028

GUIDE_L = 0.15
GUIDE_W = 0.02
GUIDE_H = 0.006
GUIDE_X = 0.24
GUIDE_Z = -0.017
GUIDE_TOP_Z = GUIDE_Z + (GUIDE_H / 2.0)

TIP_STOP_L = 0.006
TIP_STOP_W = 0.032
TIP_STOP_H = 0.018
TIP_STOP_X = 0.318
TIP_STOP_Z = -0.019

SLIDER_HOME_X = 0.225
SLIDER_TRAVEL = 0.065

SLIDER_SHOE_L = 0.044
SLIDER_SHOE_W = 0.018
SLIDER_SHOE_H = 0.004

SLIDER_BODY_L = 0.046
SLIDER_BODY_W = 0.042
SLIDER_BODY_H = 0.018
SLIDER_BODY_Z = 0.013

SLIDER_CAP_L = 0.024
SLIDER_CAP_W = 0.032
SLIDER_CAP_H = 0.01
SLIDER_CAP_Z = 0.027


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinge_ram_chain")

    model.material("base_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("arm_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("slider_orange", rgba=(0.77, 0.47, 0.18, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((BASE_FOOT_L, BASE_FOOT_W, BASE_FOOT_T)),
        origin=Origin(xyz=(BASE_FOOT_X, 0.0, BASE_FOOT_T / 2.0)),
        material="base_gray",
        name="foot",
    )
    base_bracket.visual(
        Box((BRACKET_UPRIGHT_L, BRACKET_UPRIGHT_W, BRACKET_UPRIGHT_H)),
        origin=Origin(xyz=(BRACKET_UPRIGHT_X, 0.0, BRACKET_UPRIGHT_Z)),
        material="base_gray",
        name="upright",
    )
    base_bracket.visual(
        Box((BRACKET_BRACE_L, BRACKET_BRACE_W, BRACKET_BRACE_H)),
        origin=Origin(xyz=(BRACKET_BRACE_X, BRACKET_EAR_Y, BRACKET_BRACE_Z)),
        material="base_gray",
        name="brace_pos",
    )
    base_bracket.visual(
        Box((BRACKET_BRACE_L, BRACKET_BRACE_W, BRACKET_BRACE_H)),
        origin=Origin(xyz=(BRACKET_BRACE_X, -BRACKET_EAR_Y, BRACKET_BRACE_Z)),
        material="base_gray",
        name="brace_neg",
    )
    base_bracket.visual(
        Box((BRACKET_EAR_L, BRACKET_EAR_W, BRACKET_EAR_H)),
        origin=Origin(xyz=(0.0, BRACKET_EAR_Y, HINGE_AXIS_Z)),
        material="base_gray",
        name="ear_pos",
    )
    base_bracket.visual(
        Box((BRACKET_EAR_L, BRACKET_EAR_W, BRACKET_EAR_H)),
        origin=Origin(xyz=(0.0, -BRACKET_EAR_Y, HINGE_AXIS_Z)),
        material="base_gray",
        name="ear_neg",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.16, 0.09, 0.11)),
        mass=2.3,
        origin=Origin(xyz=(-0.03, 0.0, 0.055)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_L),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="arm_silver",
        name="hinge_barrel",
    )
    arm.visual(
        Box((ARM_ROOT_BLOCK_L, ARM_ROOT_BLOCK_W, ARM_ROOT_BLOCK_H)),
        origin=Origin(xyz=(ARM_ROOT_BLOCK_X, 0.0, ARM_ROOT_BLOCK_Z)),
        material="arm_silver",
        name="root_block",
    )
    arm.visual(
        Box((ARM_BEAM_L, ARM_BEAM_W, ARM_BEAM_H)),
        origin=Origin(xyz=(ARM_BEAM_X, 0.0, ARM_BEAM_Z)),
        material="arm_silver",
        name="beam",
    )
    arm.visual(
        Box((GUIDE_L, GUIDE_W, GUIDE_H)),
        origin=Origin(xyz=(GUIDE_X, 0.0, GUIDE_Z)),
        material="arm_silver",
        name="guide",
    )
    arm.visual(
        Box((TIP_STOP_L, TIP_STOP_W, TIP_STOP_H)),
        origin=Origin(xyz=(TIP_STOP_X, 0.0, TIP_STOP_Z)),
        material="arm_silver",
        name="tip_stop",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.33, 0.04, 0.06)),
        mass=0.9,
        origin=Origin(xyz=(0.165, 0.0, -0.015)),
    )

    slider = model.part("slider")
    slider.visual(
        Box((SLIDER_SHOE_L, SLIDER_SHOE_W, SLIDER_SHOE_H)),
        origin=Origin(xyz=(0.0, 0.0, SLIDER_SHOE_H / 2.0)),
        material="slider_orange",
        name="shoe",
    )
    slider.visual(
        Box((SLIDER_BODY_L, SLIDER_BODY_W, SLIDER_BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, SLIDER_BODY_Z)),
        material="slider_orange",
        name="body",
    )
    slider.visual(
        Box((SLIDER_CAP_L, SLIDER_CAP_W, SLIDER_CAP_H)),
        origin=Origin(xyz=(0.0, 0.0, SLIDER_CAP_Z)),
        material="slider_orange",
        name="cap",
    )
    slider.inertial = Inertial.from_geometry(
        Box((SLIDER_BODY_L, SLIDER_BODY_W, 0.04)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.1),
    )
    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slider,
        origin=Origin(xyz=(SLIDER_HOME_X, 0.0, GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    arm = object_model.get_part("arm")
    slider = object_model.get_part("slider")
    hinge = object_model.get_articulation("base_to_arm")
    ram = object_model.get_articulation("arm_to_slider")

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

    hinge_limits = hinge.motion_limits
    ram_limits = ram.motion_limits

    ctx.check(
        "hinge_axis_is_pitch_up",
        tuple(hinge.axis) == (0.0, -1.0, 0.0),
        details=f"Expected hinge axis (0.0, -1.0, 0.0), got {hinge.axis!r}",
    )
    ctx.check(
        "slider_axis_tracks_arm_x",
        tuple(ram.axis) == (1.0, 0.0, 0.0),
        details=f"Expected prismatic axis (1.0, 0.0, 0.0), got {ram.axis!r}",
    )
    ctx.check(
        "joint_limits_match_requested_topology",
        hinge_limits is not None
        and ram_limits is not None
        and hinge_limits.lower == 0.0
        and hinge_limits.upper is not None
        and hinge_limits.upper >= 1.0
        and ram_limits.lower == 0.0
        and ram_limits.upper is not None
        and ram_limits.upper >= 0.065,
        details=(
            f"Unexpected joint limits: hinge={hinge_limits!r}, "
            f"ram={ram_limits!r}"
        ),
    )

    ctx.expect_contact(
        arm,
        base_bracket,
        contact_tol=0.001,
        name="arm_hinge_barrel_seats_in_base_bracket",
    )
    ctx.expect_contact(
        slider,
        arm,
        contact_tol=0.001,
        name="slider_carriage_is_supported_on_guide_at_home",
    )

    with ctx.pose({ram: SLIDER_TRAVEL}):
        ctx.expect_contact(
            slider,
            arm,
            contact_tol=0.001,
            name="slider_carriage_stays_supported_at_full_extension",
        )

    with ctx.pose({hinge: 0.0, ram: 0.0}):
        slider_home = ctx.part_world_position(slider)
    with ctx.pose({hinge: hinge_limits.upper, ram: 0.0}):
        slider_lifted = ctx.part_world_position(slider)
    with ctx.pose({hinge: 0.0, ram: ram_limits.upper}):
        slider_extended = ctx.part_world_position(slider)

    ctx.check(
        "positive_hinge_rotation_lifts_distal_stage",
        slider_home is not None
        and slider_lifted is not None
        and slider_lifted[2] > slider_home[2] + 0.12,
        details=f"Home vs lifted slider origins: {slider_home!r} -> {slider_lifted!r}",
    )
    ctx.check(
        "positive_prismatic_motion_extends_outward_along_arm",
        slider_home is not None
        and slider_extended is not None
        and slider_extended[0] > slider_home[0] + 0.06
        and abs(slider_extended[2] - slider_home[2]) < 0.002,
        details=f"Home vs extended slider origins: {slider_home!r} -> {slider_extended!r}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
