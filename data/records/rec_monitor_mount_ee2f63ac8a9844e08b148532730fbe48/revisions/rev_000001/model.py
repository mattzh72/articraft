from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.100
BASE_WIDTH = 0.074
BASE_THICKNESS = 0.008
UPRIGHT_X = -0.030
UPRIGHT_THICKNESS = 0.018
UPRIGHT_WIDTH = 0.054
UPRIGHT_HEIGHT = 0.082
SHOULDER_X = 0.0
SHOULDER_Z = 0.090

JOINT_GAP = 0.018
FORK_ARM_Y = 0.015
FORK_ARM_THICKNESS = 0.010
FORK_ARM_X = 0.016
FORK_ARM_Z = 0.028

LINK1_LENGTH = 0.110
LINK2_LENGTH = 0.098
SWIVEL_PAD_Z = 0.014
TILT_X = 0.026
TILT_Z = 0.024


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="console_monitor_arm")

    model.material("powder_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("cast_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("dark_polymer", rgba=(0.14, 0.15, 0.17, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)), origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)), material="powder_steel", name="base_plate")
    base_bracket.visual(Box((0.032, BASE_WIDTH, 0.014)), origin=Origin(xyz=(-0.032, 0.0, 0.015)), material="powder_steel", name="base_heel")
    base_bracket.visual(Box((UPRIGHT_THICKNESS, UPRIGHT_WIDTH, UPRIGHT_HEIGHT)), origin=Origin(xyz=(UPRIGHT_X, 0.0, BASE_THICKNESS + UPRIGHT_HEIGHT / 2.0)), material="powder_steel", name="upright")
    base_bracket.visual(Box((0.030, 0.022, 0.016)), origin=Origin(xyz=(-0.016, 0.0, 0.068)), material="powder_steel", name="gusset")
    base_bracket.visual(Box((0.010, 0.040, 0.028)), origin=Origin(xyz=(-0.022, 0.0, SHOULDER_Z)), material="powder_steel", name="shoulder_bridge")
    base_bracket.visual(Box((0.016, FORK_ARM_THICKNESS, 0.034)), origin=Origin(xyz=(-0.008, -FORK_ARM_Y, SHOULDER_Z)), material="powder_steel", name="shoulder_arm_left")
    base_bracket.visual(Box((0.016, FORK_ARM_THICKNESS, 0.034)), origin=Origin(xyz=(-0.008, FORK_ARM_Y, SHOULDER_Z)), material="powder_steel", name="shoulder_arm_right")

    lower_link = model.part("lower_link")
    lower_link.visual(Box((0.014, 0.020, 0.026)), origin=Origin(xyz=(0.007, 0.0, 0.0)), material="cast_aluminum", name="shoulder_tongue")
    lower_link.visual(Box((0.078, JOINT_GAP, 0.016)), origin=Origin(xyz=(0.049, 0.0, 0.0)), material="cast_aluminum", name="lower_beam")
    lower_link.visual(Box((0.060, 0.010, 0.010)), origin=Origin(xyz=(0.050, 0.0, 0.010)), material="cast_aluminum", name="lower_rib")
    lower_link.visual(Box((0.012, 0.040, 0.028)), origin=Origin(xyz=(LINK1_LENGTH - 0.020, 0.0, 0.0)), material="cast_aluminum", name="elbow_bridge")
    lower_link.visual(Box((FORK_ARM_X, FORK_ARM_THICKNESS, FORK_ARM_Z)), origin=Origin(xyz=(LINK1_LENGTH - 0.008, -FORK_ARM_Y, 0.0)), material="cast_aluminum", name="elbow_arm_left")
    lower_link.visual(Box((FORK_ARM_X, FORK_ARM_THICKNESS, FORK_ARM_Z)), origin=Origin(xyz=(LINK1_LENGTH - 0.008, FORK_ARM_Y, 0.0)), material="cast_aluminum", name="elbow_arm_right")

    upper_link = model.part("upper_link")
    upper_link.visual(Box((0.014, 0.020, 0.024)), origin=Origin(xyz=(0.007, 0.0, 0.0)), material="cast_aluminum", name="elbow_tongue")
    upper_link.visual(Box((0.070, JOINT_GAP, 0.014)), origin=Origin(xyz=(0.047, 0.0, 0.0)), material="cast_aluminum", name="upper_beam")
    upper_link.visual(Box((0.052, 0.010, 0.009)), origin=Origin(xyz=(0.050, 0.0, 0.009)), material="cast_aluminum", name="upper_rib")
    upper_link.visual(Box((0.020, 0.022, 0.030)), origin=Origin(xyz=(LINK2_LENGTH - 0.020, 0.0, 0.011)), material="cast_aluminum", name="swivel_riser")
    upper_link.visual(Box((0.016, 0.022, 0.004)), origin=Origin(xyz=(LINK2_LENGTH - 0.008, 0.0, SWIVEL_PAD_Z - 0.002)), material="cast_aluminum", name="swivel_pad")

    head_swivel = model.part("head_swivel")
    head_swivel.visual(Cylinder(radius=0.013, length=0.004), origin=Origin(xyz=(0.0, 0.0, 0.002)), material="dark_polymer", name="swivel_disc")
    head_swivel.visual(Cylinder(radius=0.009, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.013)), material="dark_polymer", name="swivel_post")
    head_swivel.visual(Box((0.018, 0.018, 0.010)), origin=Origin(xyz=(0.011, 0.0, 0.020)), material="dark_polymer", name="swivel_arm")
    head_swivel.visual(Box((0.010, 0.040, 0.024)), origin=Origin(xyz=(0.014, 0.0, TILT_Z)), material="dark_polymer", name="tilt_bridge")
    head_swivel.visual(Box((FORK_ARM_X, FORK_ARM_THICKNESS, 0.024)), origin=Origin(xyz=(TILT_X - 0.008, -FORK_ARM_Y, TILT_Z)), material="dark_polymer", name="tilt_arm_left")
    head_swivel.visual(Box((FORK_ARM_X, FORK_ARM_THICKNESS, 0.024)), origin=Origin(xyz=(TILT_X - 0.008, FORK_ARM_Y, TILT_Z)), material="dark_polymer", name="tilt_arm_right")

    head_frame = model.part("head_frame")
    head_frame.visual(Box((0.014, 0.020, 0.020)), origin=Origin(xyz=(0.007, 0.0, 0.010)), material="dark_polymer", name="tilt_tongue")
    head_frame.visual(Box((0.018, JOINT_GAP, 0.020)), origin=Origin(xyz=(0.023, 0.0, 0.018)), material="dark_polymer", name="neck")
    head_frame.visual(Box((0.016, 0.018, 0.040)), origin=Origin(xyz=(0.040, 0.0, 0.030)), material="dark_polymer", name="center_bridge")
    head_frame.visual(Box((0.008, 0.010, 0.048)), origin=Origin(xyz=(0.050, -0.030, 0.030)), material="dark_polymer", name="frame_rail_left")
    head_frame.visual(Box((0.008, 0.010, 0.048)), origin=Origin(xyz=(0.050, 0.030, 0.030)), material="dark_polymer", name="frame_rail_right")
    head_frame.visual(Box((0.008, 0.060, 0.008)), origin=Origin(xyz=(0.050, 0.0, 0.052)), material="dark_polymer", name="frame_bar_top")
    head_frame.visual(Box((0.008, 0.060, 0.008)), origin=Origin(xyz=(0.050, 0.0, 0.008)), material="dark_polymer", name="frame_bar_bottom")

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=lower_link,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=0.0, upper=1.20),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_link,
        child=upper_link,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=0.0, upper=2.40),
    )
    model.articulation(
        "head_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=head_swivel,
        origin=Origin(xyz=(LINK2_LENGTH, 0.0, SWIVEL_PAD_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.4, lower=-1.60, upper=1.60),
    )
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=head_frame,
        origin=Origin(xyz=(TILT_X, 0.0, TILT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.2, lower=-0.80, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    lower_link = object_model.get_part("lower_link")
    upper_link = object_model.get_part("upper_link")
    head_swivel = object_model.get_part("head_swivel")
    head_frame = object_model.get_part("head_frame")

    shoulder = object_model.get_articulation("shoulder_pitch")
    elbow = object_model.get_articulation("elbow_pitch")
    swivel = object_model.get_articulation("head_swivel_joint")
    tilt = object_model.get_articulation("head_tilt_joint")

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

    ctx.expect_contact(base_bracket, lower_link, name="base_bracket_contacts_lower_link")
    ctx.expect_contact(lower_link, upper_link, name="lower_link_contacts_upper_link")
    ctx.expect_contact(upper_link, head_swivel, name="upper_link_contacts_head_swivel")
    ctx.expect_contact(head_swivel, head_frame, name="head_swivel_contacts_head_frame")

    ctx.check(
        "joint_axes_match_monitor_arm_mechanism",
        shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and swivel.axis == (0.0, 0.0, 1.0)
        and tilt.axis == (0.0, 1.0, 0.0),
        (
            f"axes were shoulder={shoulder.axis}, elbow={elbow.axis}, "
            f"swivel={swivel.axis}, tilt={tilt.axis}"
        ),
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, swivel: 0.0, tilt: 0.0}):
        ctx.expect_origin_gap(
            head_frame,
            base_bracket,
            axis="x",
            min_gap=0.22,
            name="extended_head_projects_forward_of_base",
        )

    with ctx.pose({shoulder: 0.70, elbow: 2.30, swivel: 0.0, tilt: 0.40}):
        ctx.expect_origin_distance(
            head_frame,
            base_bracket,
            axes="xz",
            max_dist=0.16,
            name="folded_head_stays_close_to_base",
        )
        ctx.expect_gap(
            head_frame,
            base_bracket,
            axis="z",
            max_penetration=0.0,
            name="folded_head_clears_base_bracket",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
