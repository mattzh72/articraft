from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_HEIGHT = 0.42
FRAME_WIDTH = 0.12
FRAME_DEPTH = 0.026
RAIL_WIDTH = 0.018
RUNG_DEPTH = 0.022
RUNG_HEIGHT = 0.014
RUNG_ZS = (0.055, 0.145, 0.255, 0.365)

LEFT_SUPPORT_Z = 0.14
RIGHT_SUPPORT_Z = 0.30
SUPPORT_X = 0.072

SUPPORT_OUTER_SPAN_Y = 0.044
SUPPORT_HEIGHT = 0.058
SUPPORT_BLOCK_LENGTH = 0.030
SUPPORT_INWARD_REACH = 0.012
SUPPORT_NOSE_WIDTH = 0.026
SUPPORT_NOSE_HEIGHT = 0.038
SUPPORT_BACKPLATE_THICKNESS = 0.006

LEFT_ARM_REACH = 0.088
RIGHT_ARM_REACH = 0.138
ROOT_PAD_LENGTH = 0.010
ROOT_HEEL_RADIUS = 0.012


def _frame_shape() -> cq.Workplane:
    rung_length = FRAME_WIDTH - 2.0 * RAIL_WIDTH + 0.004
    rail_x = FRAME_WIDTH / 2.0 - RAIL_WIDTH / 2.0

    solids = [
        cq.Workplane("XY")
        .box(RAIL_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)
        .translate((rail_x, 0.0, FRAME_HEIGHT / 2.0))
        .val(),
        cq.Workplane("XY")
        .box(RAIL_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)
        .translate((-rail_x, 0.0, FRAME_HEIGHT / 2.0))
        .val(),
    ]

    for z in RUNG_ZS:
        solids.append(
            cq.Workplane("XY")
            .box(rung_length, RUNG_DEPTH, RUNG_HEIGHT)
            .translate((0.0, 0.0, z))
            .val()
        )

    frame = cq.Workplane(obj=solids[0])
    for solid in solids[1:]:
        frame = frame.union(cq.Workplane(obj=solid))

    return frame


def _cheek_block_shape(side: int) -> cq.Workplane:
    cheek_face = (
        cq.Workplane("XY")
        .box(SUPPORT_INWARD_REACH * 0.45, SUPPORT_NOSE_WIDTH, SUPPORT_NOSE_HEIGHT)
        .translate((side * (SUPPORT_INWARD_REACH * 0.225), 0.0, 0.0))
    )
    backplate = (
        cq.Workplane("XY")
        .box(SUPPORT_BACKPLATE_THICKNESS, SUPPORT_OUTER_SPAN_Y, SUPPORT_HEIGHT)
        .translate(
            (
                side * (SUPPORT_INWARD_REACH - SUPPORT_BACKPLATE_THICKNESS / 2.0),
                0.0,
                0.0,
            )
        )
    )
    web = (
        cq.Workplane("XZ")
        .moveTo(0.0, -SUPPORT_NOSE_HEIGHT / 2.0)
        .lineTo(0.0, SUPPORT_NOSE_HEIGHT / 2.0)
        .lineTo(side * (SUPPORT_INWARD_REACH - SUPPORT_BACKPLATE_THICKNESS), SUPPORT_HEIGHT / 2.0)
        .lineTo(side * (SUPPORT_INWARD_REACH - SUPPORT_BACKPLATE_THICKNESS), -SUPPORT_HEIGHT / 2.0)
        .close()
        .extrude(SUPPORT_NOSE_WIDTH * 0.42, both=True)
    )
    return cheek_face.union(backplate).union(web)


def _arm_shape(direction: int, reach: float, arm_width: float, arm_thickness: float) -> cq.Workplane:
    beam_center_x = direction * (ROOT_PAD_LENGTH + reach / 2.0)
    tip_center_x = direction * (ROOT_PAD_LENGTH + reach)
    heel_center_x = direction * ROOT_HEEL_RADIUS

    root_pad = (
        cq.Workplane("XY")
        .box(ROOT_PAD_LENGTH, arm_width, arm_thickness)
        .translate((direction * (ROOT_PAD_LENGTH / 2.0), 0.0, 0.0))
    )
    heel = (
        cq.Workplane("XZ")
        .circle(ROOT_HEEL_RADIUS)
        .extrude(arm_width / 2.0, both=True)
        .translate((heel_center_x, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(reach, arm_width, arm_thickness * 0.86)
        .translate((beam_center_x, 0.0, 0.0))
    )
    tip = (
        cq.Workplane("XZ")
        .circle(arm_thickness * 0.44)
        .extrude(arm_width / 2.0, both=True)
        .translate((tip_center_x, 0.0, 0.0))
    )
    brace = (
        cq.Workplane("XZ")
        .moveTo(direction * ROOT_PAD_LENGTH, -arm_thickness * 0.43)
        .lineTo(direction * (ROOT_PAD_LENGTH + reach * 0.46), -arm_thickness * 0.43)
        .lineTo(direction * (ROOT_PAD_LENGTH + reach * 0.22), arm_thickness * 0.58)
        .close()
        .extrude(arm_width * 0.34, both=True)
    )

    return root_pad.union(heel).union(beam).union(tip).union(brace)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_two_branch_fixture")

    frame_mat = model.material("frame_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    cheek_mat = model.material("cheek_steel", rgba=(0.42, 0.45, 0.49, 1.0))
    arm_mat = model.material("arm_amber", rgba=(0.72, 0.50, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame_shell"),
        origin=Origin(),
        material=frame_mat,
        name="frame_shell",
    )

    left_cheek = model.part("left_cheek")
    left_cheek.visual(
        mesh_from_cadquery(_cheek_block_shape(side=1), "left_cheek_shell"),
        origin=Origin(),
        material=cheek_mat,
        name="left_cheek_shell",
    )

    right_cheek = model.part("right_cheek")
    right_cheek.visual(
        mesh_from_cadquery(_cheek_block_shape(side=-1), "right_cheek_shell"),
        origin=Origin(),
        material=cheek_mat,
        name="right_cheek_shell",
    )

    left_arm = model.part("left_arm")
    left_arm.visual(
        mesh_from_cadquery(
            _arm_shape(direction=-1, reach=LEFT_ARM_REACH, arm_width=0.016, arm_thickness=0.018),
            "left_arm_shell",
        ),
        origin=Origin(),
        material=arm_mat,
        name="left_arm_shell",
    )

    right_arm = model.part("right_arm")
    right_arm.visual(
        mesh_from_cadquery(
            _arm_shape(direction=1, reach=RIGHT_ARM_REACH, arm_width=0.018, arm_thickness=0.022),
            "right_arm_shell",
        ),
        origin=Origin(),
        material=arm_mat,
        name="right_arm_shell",
    )

    model.articulation(
        "frame_to_left_cheek",
        ArticulationType.FIXED,
        parent=frame,
        child=left_cheek,
        origin=Origin(xyz=(-SUPPORT_X, 0.0, LEFT_SUPPORT_Z)),
    )
    model.articulation(
        "frame_to_right_cheek",
        ArticulationType.FIXED,
        parent=frame,
        child=right_cheek,
        origin=Origin(xyz=(SUPPORT_X, 0.0, RIGHT_SUPPORT_Z)),
    )
    model.articulation(
        "left_cheek_to_left_arm",
        ArticulationType.REVOLUTE,
        parent=left_cheek,
        child=left_arm,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.35, upper=1.05),
    )
    model.articulation(
        "right_cheek_to_right_arm",
        ArticulationType.REVOLUTE,
        parent=right_cheek,
        child=right_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.20, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_cheek = object_model.get_part("left_cheek")
    right_cheek = object_model.get_part("right_cheek")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    left_joint = object_model.get_articulation("left_cheek_to_left_arm")
    right_joint = object_model.get_articulation("right_cheek_to_right_arm")

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

    ctx.expect_contact(left_cheek, frame, contact_tol=0.0005, name="left_cheek_is_mounted_to_frame")
    ctx.expect_contact(right_cheek, frame, contact_tol=0.0005, name="right_cheek_is_mounted_to_frame")
    ctx.expect_contact(left_arm, left_cheek, contact_tol=0.0005, name="left_arm_seats_in_left_cheek")
    ctx.expect_contact(right_arm, right_cheek, contact_tol=0.0005, name="right_arm_seats_in_right_cheek")
    ctx.expect_origin_gap(
        right_cheek,
        left_cheek,
        axis="z",
        min_gap=0.12,
        name="branch_supports_are_staggered_in_height",
    )

    left_aabb = ctx.part_world_aabb(left_arm)
    right_aabb = ctx.part_world_aabb(right_arm)
    if left_aabb is not None and right_aabb is not None:
        left_dx = left_aabb[1][0] - left_aabb[0][0]
        right_dx = right_aabb[1][0] - right_aabb[0][0]
        ctx.check(
            "branch_reaches_are_intentionally_uneven",
            right_dx > left_dx + 0.035,
            details=f"left_dx={left_dx:.4f}, right_dx={right_dx:.4f}",
        )
    else:
        ctx.fail("branch_reaches_are_intentionally_uneven", "missing arm bounds")

    closed_left_aabb = ctx.part_world_aabb(left_arm)
    closed_right_aabb = ctx.part_world_aabb(right_arm)
    closed_left_pos = ctx.part_world_position(left_arm)
    closed_right_pos = ctx.part_world_position(right_arm)

    with ctx.pose({left_joint: 0.75}):
        opened_left_aabb = ctx.part_world_aabb(left_arm)
        moved_right_pos = ctx.part_world_position(right_arm)
        if closed_left_aabb is not None and opened_left_aabb is not None:
            ctx.check(
                "left_arm_positive_rotation_lifts_upward",
                opened_left_aabb[1][2] > closed_left_aabb[1][2] + 0.03,
                details=(
                    f"closed_max_z={closed_left_aabb[1][2]:.4f}, "
                    f"opened_max_z={opened_left_aabb[1][2]:.4f}"
                ),
            )
        else:
            ctx.fail("left_arm_positive_rotation_lifts_upward", "missing left arm bounds")
        if closed_right_pos is not None and moved_right_pos is not None:
            unchanged = max(abs(a - b) for a, b in zip(closed_right_pos, moved_right_pos)) < 1e-6
            ctx.check(
                "left_joint_does_not_drive_right_arm",
                unchanged,
                details=f"closed_right={closed_right_pos}, moved_right={moved_right_pos}",
            )
        else:
            ctx.fail("left_joint_does_not_drive_right_arm", "missing right arm positions")

    with ctx.pose({right_joint: 0.75}):
        opened_right_aabb = ctx.part_world_aabb(right_arm)
        moved_left_pos = ctx.part_world_position(left_arm)
        if closed_right_aabb is not None and opened_right_aabb is not None:
            ctx.check(
                "right_arm_positive_rotation_lifts_upward",
                opened_right_aabb[1][2] > closed_right_aabb[1][2] + 0.03,
                details=(
                    f"closed_max_z={closed_right_aabb[1][2]:.4f}, "
                    f"opened_max_z={opened_right_aabb[1][2]:.4f}"
                ),
            )
        else:
            ctx.fail("right_arm_positive_rotation_lifts_upward", "missing right arm bounds")
        if closed_left_pos is not None and moved_left_pos is not None:
            unchanged = max(abs(a - b) for a, b in zip(closed_left_pos, moved_left_pos)) < 1e-6
            ctx.check(
                "right_joint_does_not_drive_left_arm",
                unchanged,
                details=f"closed_left={closed_left_pos}, moved_left={moved_left_pos}",
            )
        else:
            ctx.fail("right_joint_does_not_drive_left_arm", "missing left arm positions")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
