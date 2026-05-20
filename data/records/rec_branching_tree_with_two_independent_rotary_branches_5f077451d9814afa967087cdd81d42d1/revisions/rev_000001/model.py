from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_rotary_fixture")

    frame_mat = model.material("frame_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    arm_mat = model.material("arm_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    pad_mat = model.material("pad_black", rgba=(0.14, 0.15, 0.16, 1.0))

    base_x = 0.26
    base_y = 0.20
    base_t = 0.018
    mast_r = 0.024
    mast_h = 0.34
    pedestal_r = 0.036
    pedestal_h = 0.016

    lower_z = 0.155
    upper_z = 0.255
    hub_x = 0.039
    hub_len = 0.078
    hub_y = 0.05
    hub_z = 0.038
    cheek_x = 0.026
    cheek_y = 0.008
    cheek_z = 0.05
    cheek_center_y = 0.014
    rib_x = 0.091
    rib_len = 0.026
    rib_y = 0.010
    rib_side_offset = 0.020
    rib_z = 0.008
    rib_z_offset = 0.022
    pivot_x = 0.095
    brace_len = 0.065
    brace_y = 0.024
    brace_z = 0.014
    brace_pitch = 0.63

    pivot_r = 0.014
    pivot_len = 0.020

    frame = model.part("frame")
    frame.visual(
        Box((base_x, base_y, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=frame_mat,
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=pedestal_r, length=pedestal_h),
        origin=Origin(xyz=(0.0, 0.0, base_t + pedestal_h / 2.0)),
        material=frame_mat,
        name="mast_pedestal",
    )
    frame.visual(
        Cylinder(radius=mast_r, length=mast_h),
        origin=Origin(xyz=(0.0, 0.0, base_t + mast_h / 2.0)),
        material=frame_mat,
        name="mast",
    )
    frame.visual(
        Box((hub_len, hub_y, hub_z)),
        origin=Origin(xyz=(hub_x, 0.0, lower_z)),
        material=frame_mat,
        name="lower_hub_block",
    )
    frame.visual(
        Box((rib_len, rib_y, rib_z)),
        origin=Origin(xyz=(rib_x, rib_side_offset, lower_z + rib_z_offset)),
        material=frame_mat,
        name="lower_hub_rib_top_pos",
    )
    frame.visual(
        Box((rib_len, rib_y, rib_z)),
        origin=Origin(xyz=(rib_x, -rib_side_offset, lower_z + rib_z_offset)),
        material=frame_mat,
        name="lower_hub_rib_top_neg",
    )
    frame.visual(
        Box((rib_len, rib_y, rib_z)),
        origin=Origin(xyz=(rib_x, rib_side_offset, lower_z - rib_z_offset)),
        material=frame_mat,
        name="lower_hub_rib_bottom_pos",
    )
    frame.visual(
        Box((rib_len, rib_y, rib_z)),
        origin=Origin(xyz=(rib_x, -rib_side_offset, lower_z - rib_z_offset)),
        material=frame_mat,
        name="lower_hub_rib_bottom_neg",
    )
    frame.visual(
        Box((cheek_x, cheek_y, cheek_z)),
        origin=Origin(xyz=(pivot_x, cheek_center_y, lower_z)),
        material=frame_mat,
        name="lower_cheek_pos",
    )
    frame.visual(
        Box((cheek_x, cheek_y, cheek_z)),
        origin=Origin(xyz=(pivot_x, -cheek_center_y, lower_z)),
        material=frame_mat,
        name="lower_cheek_neg",
    )
    frame.visual(
        Box((brace_len, brace_y, brace_z)),
        origin=Origin(xyz=(0.028, 0.0, 0.122), rpy=(0.0, brace_pitch, 0.0)),
        material=frame_mat,
        name="lower_brace",
    )
    frame.visual(
        Box((hub_len, hub_y, hub_z)),
        origin=Origin(xyz=(-hub_x, 0.0, upper_z)),
        material=frame_mat,
        name="upper_hub_block",
    )
    frame.visual(
        Box((rib_len, rib_y, rib_z)),
        origin=Origin(xyz=(-rib_x, rib_side_offset, upper_z + rib_z_offset)),
        material=frame_mat,
        name="upper_hub_rib_top_pos",
    )
    frame.visual(
        Box((rib_len, rib_y, rib_z)),
        origin=Origin(xyz=(-rib_x, -rib_side_offset, upper_z + rib_z_offset)),
        material=frame_mat,
        name="upper_hub_rib_top_neg",
    )
    frame.visual(
        Box((rib_len, rib_y, rib_z)),
        origin=Origin(xyz=(-rib_x, rib_side_offset, upper_z - rib_z_offset)),
        material=frame_mat,
        name="upper_hub_rib_bottom_pos",
    )
    frame.visual(
        Box((rib_len, rib_y, rib_z)),
        origin=Origin(xyz=(-rib_x, -rib_side_offset, upper_z - rib_z_offset)),
        material=frame_mat,
        name="upper_hub_rib_bottom_neg",
    )
    frame.visual(
        Box((cheek_x, cheek_y, cheek_z)),
        origin=Origin(xyz=(-pivot_x, cheek_center_y, upper_z)),
        material=frame_mat,
        name="upper_cheek_pos",
    )
    frame.visual(
        Box((cheek_x, cheek_y, cheek_z)),
        origin=Origin(xyz=(-pivot_x, -cheek_center_y, upper_z)),
        material=frame_mat,
        name="upper_cheek_neg",
    )
    frame.visual(
        Box((brace_len, brace_y, brace_z)),
        origin=Origin(xyz=(-0.028, 0.0, 0.222), rpy=(0.0, -brace_pitch, 0.0)),
        material=frame_mat,
        name="upper_brace",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=pivot_r, length=pivot_len),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=arm_mat,
        name="pivot_barrel",
    )
    lower_arm.visual(
        Box((0.115, 0.016, 0.018)),
        origin=Origin(xyz=(0.0715, 0.0, 0.0)),
        material=arm_mat,
        name="arm_beam",
    )
    lower_arm.visual(
        Box((0.018, 0.022, 0.022)),
        origin=Origin(xyz=(0.138, 0.0, 0.0)),
        material=arm_mat,
        name="pad_neck",
    )
    lower_arm.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.153, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=pad_mat,
        name="pad",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=pivot_r, length=pivot_len),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=arm_mat,
        name="pivot_barrel",
    )
    upper_arm.visual(
        Box((0.110, 0.016, 0.018)),
        origin=Origin(xyz=(-0.069, 0.0, 0.0)),
        material=arm_mat,
        name="arm_beam",
    )
    upper_arm.visual(
        Box((0.014, 0.018, 0.048)),
        origin=Origin(xyz=(-0.131, 0.0, 0.0)),
        material=arm_mat,
        name="fork_back",
    )
    upper_arm.visual(
        Box((0.028, 0.014, 0.010)),
        origin=Origin(xyz=(-0.152, 0.0, 0.014)),
        material=pad_mat,
        name="fork_top",
    )
    upper_arm.visual(
        Box((0.028, 0.014, 0.010)),
        origin=Origin(xyz=(-0.152, 0.0, -0.014)),
        material=pad_mat,
        name="fork_bottom",
    )

    model.articulation(
        "frame_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lower_arm,
        origin=Origin(xyz=(pivot_x, 0.0, lower_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.45,
            upper=1.20,
        ),
    )
    model.articulation(
        "frame_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=upper_arm,
        origin=Origin(xyz=(-pivot_x, 0.0, upper_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.45,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_joint = object_model.get_articulation("frame_to_lower_arm")
    upper_joint = object_model.get_articulation("frame_to_upper_arm")
    lower_pad = lower_arm.get_visual("pad")
    lower_barrel = lower_arm.get_visual("pivot_barrel")
    upper_barrel = upper_arm.get_visual("pivot_barrel")
    upper_fork_top = upper_arm.get_visual("fork_top")

    def center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

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
        lower_arm,
        frame,
        elem_a=lower_barrel,
        name="lower_arm_supported_in_lower_hub",
    )
    ctx.expect_contact(
        upper_arm,
        frame,
        elem_a=upper_barrel,
        name="upper_arm_supported_in_upper_hub",
    )
    ctx.expect_gap(
        upper_arm,
        lower_arm,
        axis="z",
        min_gap=0.04,
        name="branches_are_staggered_in_height",
    )

    lower_origin = ctx.part_world_position(lower_arm)
    upper_origin = ctx.part_world_position(upper_arm)
    ctx.check(
        "branches_mount_on_opposite_sides_of_mast",
        lower_origin is not None
        and upper_origin is not None
        and lower_origin[0] > 0.05
        and upper_origin[0] < -0.05,
        details=f"lower_origin={lower_origin}, upper_origin={upper_origin}",
    )

    with ctx.pose({lower_joint: 0.0}):
        lower_pad_closed = center_z(ctx.part_element_world_aabb(lower_arm, elem=lower_pad))
    with ctx.pose({lower_joint: 0.9}):
        lower_pad_open = center_z(ctx.part_element_world_aabb(lower_arm, elem=lower_pad))
    ctx.check(
        "lower_arm_positive_rotation_lifts_pad",
        lower_pad_closed is not None
        and lower_pad_open is not None
        and lower_pad_open > lower_pad_closed + 0.05,
        details=f"closed_z={lower_pad_closed}, open_z={lower_pad_open}",
    )

    with ctx.pose({upper_joint: 0.0}):
        upper_fork_closed = center_z(ctx.part_element_world_aabb(upper_arm, elem=upper_fork_top))
    with ctx.pose({upper_joint: 0.9}):
        upper_fork_open = center_z(ctx.part_element_world_aabb(upper_arm, elem=upper_fork_top))
    ctx.check(
        "upper_arm_positive_rotation_lifts_fork",
        upper_fork_closed is not None
        and upper_fork_open is not None
        and upper_fork_open > upper_fork_closed + 0.05,
        details=f"closed_z={upper_fork_closed}, open_z={upper_fork_open}",
    )

    with ctx.pose({lower_joint: 0.9, upper_joint: 0.9}):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened_pose_has_no_part_overlaps")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
