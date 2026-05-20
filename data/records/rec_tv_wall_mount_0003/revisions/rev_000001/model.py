from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

PLATE_W = 0.18
PLATE_H = 0.40
PLATE_T = 0.008
SHOULDER_BARREL_R = 0.014
JOINT_BARREL_LEN = 0.020
JOINT_LUG_T = 0.006
JOINT_CLEARANCE = 0.0
JOINT_LUG_OFFSET = (JOINT_BARREL_LEN / 2.0) + (JOINT_LUG_T / 2.0) + JOINT_CLEARANCE

UPPER_ARM_LEN = 0.18
FOREARM_LEN = 0.16
ARM_W = 0.05
ARM_H = 0.02
ARM_ROOT_CLEAR = 0.022

HEAD_NECK = 0.080
FRAME_W = 0.34
FRAME_H = 0.24
FRAME_DEPTH = 0.012
TILT_BARREL_R = 0.012
TILT_BARREL_LEN = 0.020


def _wall_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H).translate((-0.030, 0.0, 0.0))
    upper_rib = cq.Workplane("XY").box(0.018, 0.070, 0.034).translate((-0.018, 0.0, 0.033))
    lower_rib = cq.Workplane("XY").box(0.018, 0.070, 0.034).translate((-0.018, 0.0, -0.033))
    upper_lug = cq.Workplane("XY").box(0.022, 0.050, JOINT_LUG_T).translate(
        (0.000, 0.0, JOINT_LUG_OFFSET)
    )
    lower_lug = cq.Workplane("XY").box(0.022, 0.050, JOINT_LUG_T).translate(
        (0.000, 0.0, -JOINT_LUG_OFFSET)
    )
    shape = plate.union(upper_rib).union(lower_rib).union(upper_lug).union(lower_lug)

    for slot_y in (-0.055, 0.055):
        for slot_z in (-0.120, 0.120):
            slot = cq.Workplane("XY").box(PLATE_T + 0.004, 0.014, 0.050).translate(
                (-0.030, slot_y, slot_z)
            )
            shape = shape.cut(slot)
    return shape


def _arm_link_shape(length: float) -> cq.Workplane:
    tube_len = length - ARM_ROOT_CLEAR - 0.018
    outer = cq.Workplane("XY").box(tube_len, ARM_W, ARM_H).translate(
        (ARM_ROOT_CLEAR + tube_len / 2.0, 0.0, 0.0)
    )
    inner = cq.Workplane("XY").box(tube_len - 0.040, 0.032, 0.012).translate(
        (ARM_ROOT_CLEAR + tube_len / 2.0 + 0.002, 0.0, 0.0)
    )
    tube = outer.cut(inner)
    barrel = cq.Workplane("XY").circle(SHOULDER_BARREL_R).extrude(JOINT_BARREL_LEN).translate(
        (0.0, 0.0, -JOINT_BARREL_LEN / 2.0)
    )
    upper_lug = cq.Workplane("XY").box(0.022, 0.045, JOINT_LUG_T).translate(
        (length, 0.0, JOINT_LUG_OFFSET)
    )
    lower_lug = cq.Workplane("XY").box(0.022, 0.045, JOINT_LUG_T).translate(
        (length, 0.0, -JOINT_LUG_OFFSET)
    )
    return tube.union(barrel).union(upper_lug).union(lower_lug)


def _head_shape() -> cq.Workplane:
    barrel = cq.Workplane("XY").circle(0.0135).extrude(JOINT_BARREL_LEN).translate(
        (0.0, 0.0, -JOINT_BARREL_LEN / 2.0)
    )
    neck = cq.Workplane("XY").box(0.046, 0.030, 0.016).translate((0.034, 0.0, 0.0))
    left_cheek = cq.Workplane("XY").box(0.022, JOINT_LUG_T, 0.140).translate(
        (HEAD_NECK, -JOINT_LUG_OFFSET, 0.0)
    )
    right_cheek = cq.Workplane("XY").box(0.022, JOINT_LUG_T, 0.140).translate(
        (HEAD_NECK, JOINT_LUG_OFFSET, 0.0)
    )
    upper_bridge = cq.Workplane("XY").box(0.024, 0.020, 0.024).translate((0.060, 0.0, 0.046))
    lower_bridge = cq.Workplane("XY").box(0.024, 0.020, 0.024).translate((0.060, 0.0, -0.046))
    return barrel.union(neck).union(left_cheek).union(right_cheek).union(upper_bridge).union(lower_bridge)


def _frame_shape() -> cq.Workplane:
    left_rail = cq.Workplane("XY").box(FRAME_DEPTH, 0.018, FRAME_H).translate((0.018, -0.155, 0.0))
    right_rail = cq.Workplane("XY").box(FRAME_DEPTH, 0.018, FRAME_H).translate((0.018, 0.155, 0.0))
    top_bar = cq.Workplane("XY").box(FRAME_DEPTH, FRAME_W, 0.018).translate((0.018, 0.0, 0.111))
    bottom_bar = cq.Workplane("XY").box(FRAME_DEPTH, FRAME_W, 0.018).translate((0.018, 0.0, -0.111))
    center_spine = cq.Workplane("XY").box(0.016, 0.020, 0.180).translate((0.010, 0.0, 0.0))
    mid_plate = cq.Workplane("XY").box(0.012, 0.120, 0.014).translate((0.016, 0.0, 0.0))
    shape = left_rail.union(right_rail).union(top_bar).union(bottom_bar).union(center_spine).union(mid_plate)

    for slot_y in (-0.155, 0.155):
        for slot_z in (-0.060, 0.060):
            slot = cq.Workplane("XY").box(FRAME_DEPTH + 0.004, 0.008, 0.040).translate(
                (0.018, slot_y, slot_z)
            )
            shape = shape.cut(slot)
    return shape


def _wall_plate_mount_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H).translate((-0.030, 0.0, 0.0))
    upper_rib = cq.Workplane("XY").box(0.018, 0.070, 0.040).translate((-0.017, 0.0, 0.031))
    lower_rib = cq.Workplane("XY").box(0.018, 0.070, 0.040).translate((-0.017, 0.0, -0.031))
    upper_lug = cq.Workplane("XY").box(0.016, 0.050, JOINT_LUG_T).translate((0.000, 0.0, JOINT_LUG_OFFSET))
    lower_lug = cq.Workplane("XY").box(0.016, 0.050, JOINT_LUG_T).translate((0.000, 0.0, -JOINT_LUG_OFFSET))
    shape = plate.union(upper_rib).union(lower_rib).union(upper_lug).union(lower_lug)
    for slot_y in (-0.055, 0.055):
        for slot_z in (-0.120, 0.120):
            slot = cq.Workplane("XY").box(PLATE_T + 0.004, 0.014, 0.050).translate((-0.030, slot_y, slot_z))
            shape = shape.cut(slot)
    return shape


def _arm_shape(length: float, beam_width: float) -> cq.Workplane:
    barrel = cq.Workplane("XY").circle(SHOULDER_BARREL_R).extrude(JOINT_BARREL_LEN).translate(
        (SHOULDER_BARREL_R, 0.0, -JOINT_BARREL_LEN / 2.0)
    )
    root_block = cq.Workplane("XY").box(0.020, beam_width, 0.020).translate((0.024, 0.0, 0.0))
    beam = cq.Workplane("XY").box(length - 0.052, beam_width, 0.020).translate(
        ((length - 0.052) / 2.0 + 0.036, 0.0, 0.0)
    )
    upper_fork = cq.Workplane("XY").box(0.016, beam_width, JOINT_LUG_T).translate(
        (length - 0.008, 0.0, JOINT_LUG_OFFSET)
    )
    lower_fork = cq.Workplane("XY").box(0.016, beam_width, JOINT_LUG_T).translate(
        (length - 0.008, 0.0, -JOINT_LUG_OFFSET)
    )
    return barrel.union(root_block).union(beam).union(upper_fork).union(lower_fork)


def _tilt_head_mount_shape() -> cq.Workplane:
    barrel = cq.Workplane("XY").circle(0.0135).extrude(JOINT_BARREL_LEN).translate(
        (0.0135, 0.0, -JOINT_BARREL_LEN / 2.0)
    )
    neck = cq.Workplane("XY").box(0.040, 0.026, 0.020).translate((0.034, 0.0, 0.0))
    left_cheek = cq.Workplane("XY").box(0.018, JOINT_LUG_T, 0.136).translate((HEAD_NECK, -0.013, 0.0))
    right_cheek = cq.Workplane("XY").box(0.018, JOINT_LUG_T, 0.136).translate((HEAD_NECK, 0.013, 0.0))
    upper_bridge = cq.Workplane("XY").box(0.030, 0.026, 0.022).translate((0.064, 0.0, 0.046))
    lower_bridge = cq.Workplane("XY").box(0.030, 0.026, 0.022).translate((0.064, 0.0, -0.046))
    return barrel.union(neck).union(left_cheek).union(right_cheek).union(upper_bridge).union(lower_bridge)


def _mount_frame_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("YZ")
        .circle(TILT_BARREL_R)
        .extrude(TILT_BARREL_LEN)
        .translate((0.0, 0.0, 0.0))
    )
    center_spine = cq.Workplane("XY").box(0.018, 0.060, 0.150).translate((0.020, 0.0, 0.0))
    left_rail = cq.Workplane("XY").box(FRAME_DEPTH, 0.018, FRAME_H).translate((0.030, -0.155, 0.0))
    right_rail = cq.Workplane("XY").box(FRAME_DEPTH, 0.018, FRAME_H).translate((0.030, 0.155, 0.0))
    top_bar = cq.Workplane("XY").box(FRAME_DEPTH, FRAME_W, 0.018).translate((0.030, 0.0, 0.111))
    bottom_bar = cq.Workplane("XY").box(FRAME_DEPTH, FRAME_W, 0.018).translate((0.030, 0.0, -0.111))
    mid_bar = cq.Workplane("XY").box(0.012, 0.150, 0.014).translate((0.024, 0.0, 0.0))
    shape = barrel.union(center_spine).union(left_rail).union(right_rail).union(top_bar).union(bottom_bar).union(mid_bar)
    for slot_y in (-0.155, 0.155):
        for slot_z in (-0.060, 0.060):
            slot = cq.Workplane("XY").box(FRAME_DEPTH + 0.004, 0.008, 0.040).translate((0.024, slot_y, slot_z))
            shape = shape.cut(slot)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tv_wall_mount", assets=ASSETS)

    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.13, 0.14, 1.0))
    hardware = model.material("hardware", rgba=(0.33, 0.34, 0.37, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_mount_shape(), "wall_plate.obj", assets=ASSETS),
        material=dark_steel,
        name="plate_shell",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.060, PLATE_W, PLATE_H)),
        mass=3.2,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_arm_shape(UPPER_ARM_LEN, 0.045), "upper_arm.obj", assets=ASSETS),
        material=satin_black,
        name="upper_arm_shell",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LEN + 0.020, ARM_W, 0.040)),
        mass=1.5,
        origin=Origin(xyz=(UPPER_ARM_LEN / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_arm_shape(FOREARM_LEN, 0.042), "forearm.obj", assets=ASSETS),
        material=satin_black,
        name="forearm_shell",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LEN + 0.020, ARM_W, 0.040)),
        mass=1.2,
        origin=Origin(xyz=(FOREARM_LEN / 2.0, 0.0, 0.0)),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        mesh_from_cadquery(_tilt_head_mount_shape(), "tilt_head.obj", assets=ASSETS),
        material=hardware,
        name="head_shell",
    )
    tilt_head.inertial = Inertial.from_geometry(
        Box((0.100, 0.040, 0.140)),
        mass=0.8,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    mount_frame = model.part("mount_frame")
    mount_frame.visual(
        mesh_from_cadquery(_mount_frame_shape(), "mount_frame.obj", assets=ASSETS),
        material=dark_steel,
        name="frame_shell",
    )
    mount_frame.inertial = Inertial.from_geometry(
        Box((0.040, FRAME_W, FRAME_H)),
        mass=1.4,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=upper_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=tilt_head,
        origin=Origin(xyz=(FOREARM_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.0,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=tilt_head,
        child=mount_frame,
        origin=Origin(xyz=(HEAD_NECK, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.0,
            lower=-math.radians(15.0),
            upper=math.radians(15.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    wall_plate = object_model.get_part("wall_plate")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    tilt_head = object_model.get_part("tilt_head")
    mount_frame = object_model.get_part("mount_frame")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    head_swivel = object_model.get_articulation("head_swivel")
    tilt = object_model.get_articulation("tilt")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        upper_arm,
        wall_plate,
        reason="Shoulder hinge barrel is intentionally captured by the wall-plate fork.",
    )
    ctx.allow_overlap(
        mount_frame,
        tilt_head,
        reason="Tilt trunnion is intentionally nested inside the head yoke.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(upper_arm, wall_plate, contact_tol=1e-5, name="shoulder_joint_is_seated")
    ctx.expect_contact(forearm, upper_arm, contact_tol=1e-5, name="elbow_joint_is_seated")
    ctx.expect_contact(tilt_head, forearm, contact_tol=1e-5, name="swivel_joint_is_seated")
    ctx.expect_contact(mount_frame, tilt_head, contact_tol=1e-5, name="tilt_joint_is_seated")

    rest_forearm_pos = ctx.part_world_position(forearm)
    rest_head_pos = ctx.part_world_position(tilt_head)
    rest_frame_aabb = ctx.part_world_aabb(mount_frame)
    ctx.check(
        "rest_pose_extends_out_from_wall",
        rest_forearm_pos is not None and rest_head_pos is not None and rest_head_pos[0] > rest_forearm_pos[0] > 0.15,
        details=f"forearm={rest_forearm_pos}, head={rest_head_pos}",
    )
    ctx.check(
        "frame_is_taller_than_it_is_deep_at_rest",
        rest_frame_aabb is not None
        and (rest_frame_aabb[1][2] - rest_frame_aabb[0][2]) > 0.20
        and (rest_frame_aabb[1][0] - rest_frame_aabb[0][0]) < 0.05,
        details=f"frame_aabb={rest_frame_aabb}",
    )

    with ctx.pose({shoulder: math.pi / 2.0}):
        swung_frame_pos = ctx.part_world_position(mount_frame)
        ctx.check(
            "shoulder_rotates_about_vertical_axis",
            swung_frame_pos is not None and swung_frame_pos[1] > 0.30 and swung_frame_pos[0] < 0.10,
            details=f"mount_frame_pos={swung_frame_pos}",
        )

    with ctx.pose({elbow: math.pi / 2.0}):
        bent_frame_pos = ctx.part_world_position(mount_frame)
        ctx.check(
            "elbow_folds_second_link_sideways",
            bent_frame_pos is not None and bent_frame_pos[0] >= 0.17 and bent_frame_pos[1] > 0.12,
            details=f"mount_frame_pos={bent_frame_pos}",
        )

    with ctx.pose({head_swivel: math.pi / 4.0}):
        swiveled_frame_pos = ctx.part_world_position(mount_frame)
        ctx.check(
            "head_swivel_turns_frame_about_vertical_axis",
            swiveled_frame_pos is not None and swiveled_frame_pos[1] > 0.025,
            details=f"mount_frame_pos={swiveled_frame_pos}",
        )

    with ctx.pose({tilt: math.radians(15.0)}):
        tilted_frame_aabb = ctx.part_world_aabb(mount_frame)
        ctx.check(
            "tilt_joint_changes_frame_pitch",
            tilted_frame_aabb is not None
            and (tilted_frame_aabb[1][0] - tilted_frame_aabb[0][0]) > 0.07,
            details=f"tilted_frame_aabb={tilted_frame_aabb}",
        )

    ctx.check(
        "joint_limits_match_requested_ranges",
        shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and head_swivel.motion_limits is not None
        and tilt.motion_limits is not None
        and math.isclose(shoulder.motion_limits.lower, -math.pi / 2.0, abs_tol=1e-6)
        and math.isclose(shoulder.motion_limits.upper, math.pi / 2.0, abs_tol=1e-6)
        and math.isclose(elbow.motion_limits.lower, -math.pi / 2.0, abs_tol=1e-6)
        and math.isclose(elbow.motion_limits.upper, math.pi / 2.0, abs_tol=1e-6)
        and math.isclose(head_swivel.motion_limits.lower, -math.pi / 4.0, abs_tol=1e-6)
        and math.isclose(head_swivel.motion_limits.upper, math.pi / 4.0, abs_tol=1e-6)
        and math.isclose(tilt.motion_limits.lower, -math.radians(15.0), abs_tol=1e-6)
        and math.isclose(tilt.motion_limits.upper, math.radians(15.0), abs_tol=1e-6),
        details=(
            f"shoulder={shoulder.motion_limits}, elbow={elbow.motion_limits}, "
            f"head_swivel={head_swivel.motion_limits}, tilt={tilt.motion_limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
