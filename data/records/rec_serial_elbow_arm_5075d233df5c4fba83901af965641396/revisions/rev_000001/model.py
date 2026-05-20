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


HALF_PI = 1.5707963267948966

BASE_HEIGHT = 0.035
SHOULDER_X = 0.120
SHOULDER_Z = 0.170
UPPER_ARM_LENGTH = 0.290
FOREARM_LENGTH = 0.230

BASE_CHEEK_X = 0.030
BASE_CHEEK_Y = 0.012
BASE_CHEEK_Z = 0.060
BASE_CHEEK_CENTER_Y = 0.023

SHOULDER_HUB_RADIUS = 0.014
SHOULDER_HUB_LENGTH = 0.034

ELBOW_FORK_X = 0.035
ELBOW_FORK_Y = 0.012
ELBOW_FORK_Z = 0.042
ELBOW_FORK_CENTER_Y = 0.020
ELBOW_HUB_RADIUS = 0.010
ELBOW_HUB_LENGTH = 0.028

TOOL_PAD_LENGTH = 0.045


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_link_elbow_arm")

    base_color = model.material("base_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    upper_color = model.material("upper_orange", rgba=(0.88, 0.45, 0.14, 1.0))
    forearm_color = model.material("forearm_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    tool_color = model.material("tool_black", rgba=(0.16, 0.17, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.22, 0.18, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=base_color,
        name="pedestal",
    )
    base.visual(
        Box((0.060, 0.090, 0.110)),
        origin=Origin(xyz=(0.030, 0.0, BASE_HEIGHT + 0.055)),
        material=base_color,
        name="column",
    )
    base.visual(
        Box((0.100, 0.050, 0.020)),
        origin=Origin(xyz=(0.075, 0.0, 0.140)),
        material=base_color,
        name="shoulder_block",
    )
    base.visual(
        Box((BASE_CHEEK_X, BASE_CHEEK_Y, BASE_CHEEK_Z)),
        origin=Origin(xyz=(SHOULDER_X, -BASE_CHEEK_CENTER_Y, SHOULDER_Z)),
        material=base_color,
        name="left_shoulder_cheek",
    )
    base.visual(
        Box((BASE_CHEEK_X, BASE_CHEEK_Y, BASE_CHEEK_Z)),
        origin=Origin(xyz=(SHOULDER_X, BASE_CHEEK_CENTER_Y, SHOULDER_Z)),
        material=base_color,
        name="right_shoulder_cheek",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=SHOULDER_HUB_RADIUS, length=SHOULDER_HUB_LENGTH),
        origin=Origin(rpy=(-HALF_PI, 0.0, 0.0)),
        material=upper_color,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.060, 0.030, 0.030)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=upper_color,
        name="root_block",
    )
    upper_arm.visual(
        Box((0.190, 0.032, 0.028)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=upper_color,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.030, 0.038, 0.026)),
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        material=upper_color,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.040, ELBOW_FORK_Y, ELBOW_FORK_Z)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.010, -ELBOW_FORK_CENTER_Y, 0.0)),
        material=upper_color,
        name="left_elbow_fork",
    )
    upper_arm.visual(
        Box((0.040, ELBOW_FORK_Y, ELBOW_FORK_Z)),
        origin=Origin(xyz=(UPPER_ARM_LENGTH - 0.010, ELBOW_FORK_CENTER_Y, 0.0)),
        material=upper_color,
        name="right_elbow_fork",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=ELBOW_HUB_RADIUS, length=ELBOW_HUB_LENGTH),
        origin=Origin(rpy=(-HALF_PI, 0.0, 0.0)),
        material=forearm_color,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.190, 0.026, 0.024)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=forearm_color,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.030, 0.038, 0.036)),
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        material=forearm_color,
        name="wrist_block",
    )

    forearm.visual(
        Box((TOOL_PAD_LENGTH, 0.055, 0.010)),
        origin=Origin(xyz=(FOREARM_LENGTH + TOOL_PAD_LENGTH / 2.0, 0.0, 0.0)),
        material=tool_color,
        name="tool_pad_body",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
            lower=-1.0,
            upper=0.40,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=0.0,
            upper=1.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")
    tool_pad_visual = forearm.get_visual("tool_pad_body")

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

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (base, upper_arm, forearm)),
        "Expected base, upper_arm, and forearm parts.",
    )
    ctx.check(
        "all_joints_present",
        all(joint is not None for joint in (shoulder, elbow)),
        "Expected shoulder_joint and elbow_joint articulations.",
    )
    ctx.check(
        "tool_pad_visual_present",
        tool_pad_visual is not None,
        "Expected the forearm to include a tool_pad_body visual at its tip.",
    )
    ctx.check(
        "parallel_horizontal_joint_axes",
        shoulder.axis == (0.0, 1.0, 0.0) and elbow.axis == (0.0, 1.0, 0.0),
        f"Shoulder axis {shoulder.axis} and elbow axis {elbow.axis} should both be parallel to +Y.",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        ctx.expect_contact(base, upper_arm, name="base_to_upper_arm_contact")
        ctx.expect_contact(upper_arm, forearm, name="upper_arm_to_forearm_contact")
        ctx.expect_overlap(
            base,
            upper_arm,
            axes="yz",
            min_overlap=0.03,
            name="base_upper_arm_joint_footprint",
        )
        ctx.expect_overlap(
            upper_arm,
            forearm,
            axes="yz",
            min_overlap=0.024,
            name="upper_arm_forearm_joint_footprint",
        )
        ctx.expect_origin_gap(
            forearm,
            upper_arm,
            axis="x",
            min_gap=0.29,
            max_gap=0.31,
            name="elbow_origin_forward_of_shoulder_origin_at_rest",
        )
        tool_aabb = ctx.part_element_world_aabb(forearm, elem="tool_pad_body")
        wrist_aabb = ctx.part_element_world_aabb(forearm, elem="wrist_block")
        forearm_origin = ctx.part_world_position(forearm)
        if tool_aabb is None or wrist_aabb is None or forearm_origin is None:
            ctx.fail("rest_pose_tool_geometry_available", "Could not measure the forearm tool-pad geometry.")
        else:
            tool_center_x = 0.5 * (tool_aabb[0][0] + tool_aabb[1][0])
            tool_size_z = tool_aabb[1][2] - tool_aabb[0][2]
            ctx.check(
                "tool_pad_extends_beyond_wrist_block",
                tool_aabb[0][0] >= wrist_aabb[1][0] - 1e-6,
                (
                    "Tool pad should start at or beyond the wrist block front face; "
                    f"tool_min_x={tool_aabb[0][0]:.4f}, wrist_max_x={wrist_aabb[1][0]:.4f}."
                ),
            )
            ctx.check(
                "tool_pad_origin_at_forearm_tip",
                0.24 <= tool_center_x - forearm_origin[0] <= 0.28,
                (
                    "Tool pad center should sit near the forearm tip; "
                    f"delta_x={tool_center_x - forearm_origin[0]:.4f}."
                ),
            )
            ctx.check(
                "tool_pad_is_thin",
                0.009 <= tool_size_z <= 0.011,
                f"Tool pad thickness should be about 10 mm, got {tool_size_z:.4f} m.",
            )

    shoulder_limits = shoulder.motion_limits
    if shoulder_limits is not None and shoulder_limits.lower is not None and shoulder_limits.upper is not None:
        for label, shoulder_pose in (
            ("shoulder_lower", shoulder_limits.lower),
            ("shoulder_upper", shoulder_limits.upper),
        ):
            with ctx.pose({shoulder: shoulder_pose, elbow: 0.7}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_no_floating")
                ctx.expect_contact(base, upper_arm, name=f"{label}_base_contact")
                ctx.expect_contact(upper_arm, forearm, name=f"{label}_elbow_contact")

    elbow_limits = elbow.motion_limits
    if elbow_limits is not None and elbow_limits.lower is not None and elbow_limits.upper is not None:
        for label, elbow_pose in (
            ("elbow_lower", elbow_limits.lower),
            ("elbow_upper", elbow_limits.upper),
        ):
            with ctx.pose({shoulder: 0.35, elbow: elbow_pose}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_no_floating")
                ctx.expect_contact(upper_arm, forearm, name=f"{label}_joint_contact")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
