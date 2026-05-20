from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


WALL_PLATE_W = 0.11
WALL_PLATE_H = 0.20
WALL_PLATE_T = 0.012
WALL_BOSS_LEN = 0.014
WALL_BOSS_W = 0.064
WALL_BOSS_H = 0.072

INNER_ARM_LEN = 0.185
OUTER_ARM_LEN = 0.165
LINK_W = 0.050
LINK_T = 0.028

HEAD_BASE_LEN = 0.016
HEAD_BASE_W = 0.026
HEAD_BASE_T = 0.020
HEAD_TILT_X = 0.040
CHEEK_T = 0.004
CHEEK_H = 0.020
BARREL_LEN = 0.018
BARREL_R = 0.0065

FRAME_SIZE = 0.120
FRAME_RING = 0.016
FRAME_T = 0.006
FRAME_X0 = 0.016
FRAME_HOLE_SPACING = 0.075
FRAME_HOLE_R = 0.003


def _wall_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .rect(WALL_PLATE_W, WALL_PLATE_H)
        .extrude(WALL_PLATE_T)
        .edges("|X")
        .fillet(0.012)
    )
    boss = (
        cq.Workplane("YZ")
        .rect(WALL_BOSS_W, WALL_BOSS_H)
        .extrude(WALL_BOSS_LEN)
        .translate((WALL_PLATE_T, 0.0, 0.0))
        .edges("|X")
        .fillet(0.006)
    )
    holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.032, -0.068),
                (0.032, -0.068),
                (-0.032, 0.068),
                (0.032, 0.068),
            ]
        )
        .circle(0.0042)
        .extrude(WALL_PLATE_T + WALL_BOSS_LEN + 0.004)
    )
    return plate.union(boss).cut(holes)


def _arm_link_shape(length: float, name_scale: float) -> cq.Workplane:
    body_w = LINK_W * 0.64
    rear_pad_r = LINK_W * 0.34
    front_pad_r = LINK_W * 0.30
    slot_length = length * 0.46
    slot_width = LINK_W * 0.23

    body = cq.Workplane("XY").box(length, body_w, LINK_T, centered=(False, True, True))
    rear_pad = (
        cq.Workplane("XY")
        .circle(rear_pad_r)
        .extrude(LINK_T, both=True)
        .translate((rear_pad_r, 0.0, 0.0))
    )
    front_pad = (
        cq.Workplane("XY")
        .circle(front_pad_r)
        .extrude(LINK_T, both=True)
        .translate((length - front_pad_r, 0.0, 0.0))
    )
    slot = (
        cq.Workplane("XY")
        .slot2D(slot_length, slot_width)
        .extrude(LINK_T * 1.4, both=True)
        .translate((length * (0.52 + 0.02 * name_scale), 0.0, 0.0))
    )

    link = body.union(rear_pad).union(front_pad).cut(slot)
    return link.edges("|Z").fillet(0.0045)


def _head_swivel_shape() -> cq.Workplane:
    cheek_center_y = BARREL_LEN / 2.0 + CHEEK_T / 2.0

    base = cq.Workplane("XY").box(
        HEAD_BASE_LEN,
        HEAD_BASE_W,
        HEAD_BASE_T,
        centered=(False, True, True),
    )
    neck = (
        cq.Workplane("XY")
        .box(0.014, 0.014, 0.014, centered=(False, True, True))
        .translate((HEAD_BASE_LEN, 0.0, 0.0))
    )
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.017, BARREL_LEN + 0.004, 0.006, centered=(False, True, True))
        .translate((0.015, 0.0, -0.007))
    )
    cheek_left = (
        cq.Workplane("XY")
        .box(0.012, CHEEK_T, CHEEK_H, centered=(False, True, True))
        .translate((HEAD_TILT_X - 0.012, cheek_center_y, 0.0))
    )
    cheek_right = (
        cq.Workplane("XY")
        .box(0.012, CHEEK_T, CHEEK_H, centered=(False, True, True))
        .translate((HEAD_TILT_X - 0.012, -cheek_center_y, 0.0))
    )
    face_pad = (
        cq.Workplane("YZ")
        .rect(0.020, 0.020)
        .extrude(0.004)
        .translate((0.0, 0.0, 0.0))
    )
    return (
        base.union(neck)
        .union(lower_bridge)
        .union(cheek_left)
        .union(cheek_right)
        .union(face_pad)
    )


def _monitor_frame_shape() -> cq.Workplane:
    frame_plate = (
        cq.Workplane("YZ")
        .rect(FRAME_SIZE, FRAME_SIZE)
        .extrude(FRAME_T)
        .translate((FRAME_X0, 0.0, 0.0))
    )
    center_opening = (
        cq.Workplane("YZ")
        .rect(FRAME_SIZE - 2.0 * FRAME_RING, FRAME_SIZE - 2.0 * FRAME_RING)
        .extrude(FRAME_T + 0.004)
        .translate((FRAME_X0 - 0.002, 0.0, 0.0))
    )
    vesa_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-FRAME_HOLE_SPACING / 2.0, -FRAME_HOLE_SPACING / 2.0),
                (FRAME_HOLE_SPACING / 2.0, -FRAME_HOLE_SPACING / 2.0),
                (-FRAME_HOLE_SPACING / 2.0, FRAME_HOLE_SPACING / 2.0),
                (FRAME_HOLE_SPACING / 2.0, FRAME_HOLE_SPACING / 2.0),
            ]
        )
        .circle(FRAME_HOLE_R)
        .extrude(FRAME_T + 0.004)
        .translate((FRAME_X0 - 0.002, 0.0, 0.0))
    )
    barrel = cq.Workplane("XZ").circle(BARREL_R).extrude(BARREL_LEN, both=True)
    spine = (
        cq.Workplane("XY")
        .box(0.024, 0.014, 0.020, centered=(False, True, True))
        .translate((0.004, 0.0, 0.0))
    )
    vertical_rib = (
        cq.Workplane("YZ")
        .rect(0.012, FRAME_SIZE - 2.0 * FRAME_RING)
        .extrude(FRAME_T)
        .translate((FRAME_X0, 0.0, 0.0))
    )
    horizontal_rib = (
        cq.Workplane("YZ")
        .rect(FRAME_SIZE - 2.0 * FRAME_RING, 0.012)
        .extrude(FRAME_T)
        .translate((FRAME_X0, 0.0, 0.0))
    )
    frame = frame_plate.cut(center_opening).cut(vesa_holes)
    return barrel.union(spine).union(frame).union(vertical_rib).union(horizontal_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_display_arm")

    plate_metal = model.material("plate_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    head_metal = model.material("head_metal", rgba=(0.14, 0.15, 0.17, 1.0))
    frame_metal = model.material("frame_metal", rgba=(0.28, 0.29, 0.31, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate"),
        origin=Origin(),
        material=plate_metal,
        name="wall_plate_body",
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        mesh_from_cadquery(_arm_link_shape(INNER_ARM_LEN, 0.0), "inner_arm"),
        origin=Origin(),
        material=arm_metal,
        name="inner_arm_body",
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        mesh_from_cadquery(_arm_link_shape(OUTER_ARM_LEN, 1.0), "outer_arm"),
        origin=Origin(),
        material=arm_metal,
        name="outer_arm_body",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        mesh_from_cadquery(_head_swivel_shape(), "head_swivel"),
        origin=Origin(),
        material=head_metal,
        name="head_swivel_body",
    )

    monitor_frame = model.part("monitor_frame")
    monitor_frame.visual(
        mesh_from_cadquery(_monitor_frame_shape(), "monitor_frame"),
        origin=Origin(),
        material=frame_metal,
        name="monitor_frame_body",
    )

    model.articulation(
        "shoulder_fold",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=inner_arm,
        origin=Origin(xyz=(WALL_PLATE_T + WALL_BOSS_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-2.5,
            upper=2.5,
        ),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(INNER_ARM_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "head_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=head_swivel,
        origin=Origin(xyz=(OUTER_ARM_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-1.8,
            upper=1.8,
        ),
    )
    model.articulation(
        "head_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=monitor_frame,
        origin=Origin(xyz=(HEAD_TILT_X + BARREL_R, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    head_swivel = object_model.get_part("head_swivel")
    monitor_frame = object_model.get_part("monitor_frame")

    shoulder = object_model.get_articulation("shoulder_fold")
    elbow = object_model.get_articulation("elbow_fold")
    head_swivel_joint = object_model.get_articulation("head_swivel_joint")
    head_tilt_joint = object_model.get_articulation("head_tilt_joint")

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
        "four_revolute_joints_present",
        len(object_model.articulations) == 4,
        details=f"expected 4 articulations, found {len(object_model.articulations)}",
    )
    ctx.check(
        "joint_axes_match_display_arm_convention",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 0.0, 1.0)
        and head_swivel_joint.axis == (0.0, 0.0, 1.0)
        and head_tilt_joint.axis == (0.0, 1.0, 0.0),
        details="arm folds and head swivel should use vertical axes; tilt should use a lateral axis",
    )

    with ctx.pose(
        {
            shoulder: 0.0,
            elbow: 0.0,
            head_swivel_joint: 0.0,
            head_tilt_joint: 0.0,
        }
    ):
        ctx.expect_contact(
            inner_arm,
            wall_plate,
            name="inner_arm_mounts_to_wall_plate",
        )
        ctx.expect_contact(
            outer_arm,
            inner_arm,
            name="outer_arm_mounts_to_inner_arm",
        )
        ctx.expect_contact(
            head_swivel,
            outer_arm,
            name="head_swivel_mounts_to_outer_arm",
        )
        ctx.expect_contact(
            monitor_frame,
            head_swivel,
            name="monitor_frame_mounts_to_head_yoke",
        )

    with ctx.pose({shoulder: 0.8, elbow: -1.7}):
        ctx.expect_gap(
            monitor_frame,
            wall_plate,
            axis="x",
            min_gap=0.09,
            name="frame_stands_off_from_wall_when_unfolded",
        )

    neutral_frame_pos = None
    swivel_frame_pos = None
    with ctx.pose({head_swivel_joint: 0.0}):
        neutral_frame_pos = ctx.part_world_position(monitor_frame)
    with ctx.pose({head_swivel_joint: 0.9}):
        swivel_frame_pos = ctx.part_world_position(monitor_frame)
    if neutral_frame_pos is not None and swivel_frame_pos is not None:
        ctx.check(
            "head_swivel_moves_frame_laterally",
            abs(swivel_frame_pos[1] - neutral_frame_pos[1]) > 0.015,
            details=f"expected noticeable lateral shift, got {neutral_frame_pos} -> {swivel_frame_pos}",
        )

    neutral_aabb = None
    tilted_aabb = None
    with ctx.pose({head_tilt_joint: 0.0}):
        neutral_aabb = ctx.part_world_aabb(monitor_frame)
    with ctx.pose({head_tilt_joint: 0.35}):
        tilted_aabb = ctx.part_world_aabb(monitor_frame)
    if neutral_aabb is not None and tilted_aabb is not None:
        ctx.check(
            "head_tilt_changes_frame_pitch",
            tilted_aabb[1][0] > neutral_aabb[1][0] + 0.012,
            details=f"expected tilt to push frame farther forward, got {neutral_aabb} -> {tilted_aabb}",
        )

    outer_aabb = ctx.part_world_aabb(outer_arm)
    head_aabb = ctx.part_world_aabb(head_swivel)
    if outer_aabb is not None and head_aabb is not None:
        outer_y = outer_aabb[1][1] - outer_aabb[0][1]
        outer_z = outer_aabb[1][2] - outer_aabb[0][2]
        head_y = head_aabb[1][1] - head_aabb[0][1]
        head_z = head_aabb[1][2] - head_aabb[0][2]
        ctx.check(
            "head_hardware_stays_smaller_than_folding_links",
            head_y < outer_y and head_z < outer_z,
            details=(
                "expected compact head hardware smaller than link section, "
                f"got head ({head_y:.4f}, {head_z:.4f}) vs link ({outer_y:.4f}, {outer_z:.4f})"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
