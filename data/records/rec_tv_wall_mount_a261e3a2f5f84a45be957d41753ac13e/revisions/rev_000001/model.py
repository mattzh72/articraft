from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_THICKNESS = 0.018
PLATE_WIDTH = 0.180
PLATE_HEIGHT = 0.400

PIVOT_RADIUS = 0.018
PIVOT_HALF_HEIGHT = 0.004
WALL_JOINT_X = 0.050

INNER_ARM_LENGTH = 0.240
OUTER_ARM_LENGTH = 0.210
ARM_WIDTH = 0.042
ARM_HEIGHT = 0.028

SWIVEL_BODY_LENGTH = 0.070
SUPPORT_PLATE_X = 0.020
SUPPORT_PLATE_Y = 0.014
SUPPORT_PLATE_Z = 0.190

FRAME_OUTER_Y = 0.340
FRAME_OUTER_Z = 0.240
FRAME_RING = 0.030
FRAME_THICKNESS = 0.016
FRAME_FORWARD_OFFSET = 0.010

TRUNNION_RADIUS = 0.010
TRUNNION_LENGTH = 0.014
SUPPORT_CENTER_Y = FRAME_OUTER_Y / 2.0 + TRUNNION_LENGTH + SUPPORT_PLATE_Y / 2.0
SUPPORT_MOUNT_X = SWIVEL_BODY_LENGTH + SUPPORT_PLATE_X / 2.0


def _wall_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)
        .translate((-PLATE_THICKNESS / 2.0, 0.0, 0.0))
    )
    standoff = (
        cq.Workplane("XY")
        .box(WALL_JOINT_X - PIVOT_RADIUS, 0.060, 0.090, centered=(False, True, True))
        .translate((0.0, 0.0, 0.0))
    )
    for y in (-0.050, 0.050):
        for z in (-0.110, 0.110):
            cutout = (
                cq.Workplane("XY")
                .box(PLATE_THICKNESS + 0.004, 0.028, 0.085)
                .translate((-PLATE_THICKNESS / 2.0, y, z))
            )
            plate = plate.cut(cutout)
    return plate.union(standoff)


def _arm_body_shape(length: float) -> cq.Workplane:
    body_start = PIVOT_RADIUS
    body_end = length - PIVOT_RADIUS
    body_length = body_end - body_start
    window_start = 0.060
    window_length = body_length - 0.084

    body = (
        cq.Workplane("XY")
        .box(body_length, ARM_WIDTH, ARM_HEIGHT, centered=(False, True, True))
        .translate((body_start, 0.0, 0.0))
    )
    if window_length > 0.025:
        window = (
            cq.Workplane("XY")
            .box(window_length, ARM_WIDTH - 0.014, ARM_HEIGHT + 0.004, centered=(False, True, True))
            .translate((window_start, 0.0, 0.0))
        )
        body = body.cut(window)
    return body


def _swivel_body_shape() -> cq.Workplane:
    neck = (
        cq.Workplane("XY")
        .box(0.038, 0.038, 0.030, centered=(False, True, True))
        .translate((PIVOT_RADIUS, 0.0, 0.0))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.014, 2.0 * SUPPORT_CENTER_Y - SUPPORT_PLATE_Y, 0.028, centered=(False, True, True))
        .translate((SWIVEL_BODY_LENGTH - 0.032, 0.0, 0.0))
    )
    left_tab = (
        cq.Workplane("XY")
        .box(0.018, SUPPORT_PLATE_Y, 0.030, centered=(False, True, True))
        .translate((SWIVEL_BODY_LENGTH - 0.018, -SUPPORT_CENTER_Y, 0.0))
    )
    right_tab = (
        cq.Workplane("XY")
        .box(0.018, SUPPORT_PLATE_Y, 0.030, centered=(False, True, True))
        .translate((SWIVEL_BODY_LENGTH - 0.018, SUPPORT_CENTER_Y, 0.0))
    )
    return neck.union(bridge).union(left_tab).union(right_tab)


def _head_frame_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(FRAME_THICKNESS, FRAME_OUTER_Y, FRAME_OUTER_Z)
        .translate((FRAME_FORWARD_OFFSET, 0.0, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(FRAME_THICKNESS + 0.004, FRAME_OUTER_Y - 2.0 * FRAME_RING, FRAME_OUTER_Z - 2.0 * FRAME_RING)
        .translate((FRAME_FORWARD_OFFSET, 0.0, 0.0))
    )
    return outer.cut(inner)


def _axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
    return all(isclose(a, b, abs_tol=1e-9) for a, b in zip(axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_motion_tv_wall_mount")

    steel = model.material("steel", rgba=(0.26, 0.28, 0.31, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    hardware = model.material("hardware", rgba=(0.45, 0.47, 0.50, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate_body"),
        material=steel,
        name="plate_body",
    )
    wall_plate.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_HALF_HEIGHT),
        origin=Origin(xyz=(WALL_JOINT_X, 0.0, -PIVOT_HALF_HEIGHT / 2.0)),
        material=dark_steel,
        name="pivot_cap",
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        mesh_from_cadquery(_arm_body_shape(INNER_ARM_LENGTH), "inner_arm_body"),
        material=steel,
        name="arm_body",
    )
    inner_arm.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_HALF_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HALF_HEIGHT / 2.0)),
        material=dark_steel,
        name="prox_cap",
    )
    inner_arm.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_HALF_HEIGHT),
        origin=Origin(xyz=(INNER_ARM_LENGTH, 0.0, -PIVOT_HALF_HEIGHT / 2.0)),
        material=dark_steel,
        name="dist_cap",
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        mesh_from_cadquery(_arm_body_shape(OUTER_ARM_LENGTH), "outer_arm_body"),
        material=steel,
        name="arm_body",
    )
    outer_arm.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_HALF_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HALF_HEIGHT / 2.0)),
        material=dark_steel,
        name="prox_cap",
    )
    outer_arm.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_HALF_HEIGHT),
        origin=Origin(xyz=(OUTER_ARM_LENGTH, 0.0, -PIVOT_HALF_HEIGHT / 2.0)),
        material=dark_steel,
        name="dist_cap",
    )

    swivel_block = model.part("swivel_block")
    swivel_block.visual(
        mesh_from_cadquery(_swivel_body_shape(), "swivel_block_body"),
        material=steel,
        name="swivel_body",
    )
    swivel_block.visual(
        Cylinder(radius=PIVOT_RADIUS, length=PIVOT_HALF_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HALF_HEIGHT / 2.0)),
        material=dark_steel,
        name="prox_cap",
    )

    left_tilt_support = model.part("left_tilt_support")
    left_tilt_support.visual(
        Box((SUPPORT_PLATE_X, SUPPORT_PLATE_Y, SUPPORT_PLATE_Z)),
        material=dark_steel,
        name="support_plate",
    )

    right_tilt_support = model.part("right_tilt_support")
    right_tilt_support.visual(
        Box((SUPPORT_PLATE_X, SUPPORT_PLATE_Y, SUPPORT_PLATE_Z)),
        material=dark_steel,
        name="support_plate",
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        mesh_from_cadquery(_head_frame_shape(), "head_frame_ring"),
        material=steel,
        name="frame_ring",
    )
    head_frame.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, -(FRAME_OUTER_Y / 2.0 + TRUNNION_LENGTH / 2.0), 0.0),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=hardware,
        name="left_trunnion",
    )
    head_frame.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, FRAME_OUTER_Y / 2.0 + TRUNNION_LENGTH / 2.0, 0.0),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=hardware,
        name="right_trunnion",
    )

    model.articulation(
        "wall_to_inner_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=inner_arm,
        origin=Origin(xyz=(WALL_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "inner_to_outer_arm",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(INNER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.6, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "outer_to_swivel_block",
        ArticulationType.REVOLUTE,
        parent=outer_arm,
        child=swivel_block,
        origin=Origin(xyz=(OUTER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "swivel_block_to_left_support",
        ArticulationType.FIXED,
        parent=swivel_block,
        child=left_tilt_support,
        origin=Origin(xyz=(SUPPORT_MOUNT_X, -SUPPORT_CENTER_Y, 0.0)),
    )
    model.articulation(
        "swivel_block_to_right_support",
        ArticulationType.FIXED,
        parent=swivel_block,
        child=right_tilt_support,
        origin=Origin(xyz=(SUPPORT_MOUNT_X, SUPPORT_CENTER_Y, 0.0)),
    )
    model.articulation(
        "swivel_block_to_head_frame",
        ArticulationType.REVOLUTE,
        parent=swivel_block,
        child=head_frame,
        origin=Origin(xyz=(SUPPORT_MOUNT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.35, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    swivel_block = object_model.get_part("swivel_block")
    left_tilt_support = object_model.get_part("left_tilt_support")
    right_tilt_support = object_model.get_part("right_tilt_support")
    head_frame = object_model.get_part("head_frame")

    wall_to_inner_arm = object_model.get_articulation("wall_to_inner_arm")
    inner_to_outer_arm = object_model.get_articulation("inner_to_outer_arm")
    outer_to_swivel_block = object_model.get_articulation("outer_to_swivel_block")
    swivel_block_to_head_frame = object_model.get_articulation("swivel_block_to_head_frame")

    plate_cap = wall_plate.get_visual("pivot_cap")
    inner_prox_cap = inner_arm.get_visual("prox_cap")
    inner_dist_cap = inner_arm.get_visual("dist_cap")
    outer_prox_cap = outer_arm.get_visual("prox_cap")
    outer_dist_cap = outer_arm.get_visual("dist_cap")
    swivel_prox_cap = swivel_block.get_visual("prox_cap")
    left_support_plate = left_tilt_support.get_visual("support_plate")
    right_support_plate = right_tilt_support.get_visual("support_plate")
    left_trunnion = head_frame.get_visual("left_trunnion")
    right_trunnion = head_frame.get_visual("right_trunnion")

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
    ctx.allow_overlap(
        left_tilt_support,
        swivel_block,
        elem_a=left_support_plate,
        elem_b="swivel_body",
        reason="Fixed tilt side plate is modeled as a separate welded bracket sharing a coincident mounting face with the swivel carriage.",
    )
    ctx.allow_overlap(
        right_tilt_support,
        swivel_block,
        elem_a=right_support_plate,
        elem_b="swivel_body",
        reason="Fixed tilt side plate is modeled as a separate welded bracket sharing a coincident mounting face with the swivel carriage.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name, part in (
        ("wall_plate", wall_plate),
        ("inner_arm", inner_arm),
        ("outer_arm", outer_arm),
        ("swivel_block", swivel_block),
        ("left_tilt_support", left_tilt_support),
        ("right_tilt_support", right_tilt_support),
        ("head_frame", head_frame),
    ):
        ctx.check(f"{part_name}_present", part is not None, f"missing part: {part_name}")

    ctx.check(
        "wall_to_inner_arm_axis_vertical",
        _axis_matches(wall_to_inner_arm.axis, (0.0, 0.0, 1.0)),
        f"unexpected axis: {wall_to_inner_arm.axis}",
    )
    ctx.check(
        "inner_to_outer_arm_axis_vertical",
        _axis_matches(inner_to_outer_arm.axis, (0.0, 0.0, 1.0)),
        f"unexpected axis: {inner_to_outer_arm.axis}",
    )
    ctx.check(
        "outer_to_swivel_block_axis_vertical",
        _axis_matches(outer_to_swivel_block.axis, (0.0, 0.0, 1.0)),
        f"unexpected axis: {outer_to_swivel_block.axis}",
    )
    ctx.check(
        "head_tilt_axis_horizontal",
        _axis_matches(swivel_block_to_head_frame.axis, (0.0, 1.0, 0.0)),
        f"unexpected axis: {swivel_block_to_head_frame.axis}",
    )

    ctx.expect_gap(
        inner_arm,
        wall_plate,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=inner_prox_cap,
        negative_elem=plate_cap,
        name="wall_plate_to_inner_arm_knuckle_seating",
    )
    ctx.expect_gap(
        outer_arm,
        inner_arm,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=outer_prox_cap,
        negative_elem=inner_dist_cap,
        name="inner_to_outer_arm_knuckle_seating",
    )
    ctx.expect_gap(
        swivel_block,
        outer_arm,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=swivel_prox_cap,
        negative_elem=outer_dist_cap,
        name="outer_arm_to_swivel_knuckle_seating",
    )
    ctx.expect_contact(
        left_tilt_support,
        swivel_block,
        elem_a=left_support_plate,
        elem_b="swivel_body",
        name="left_support_mounted_to_swivel_block",
    )
    ctx.expect_contact(
        right_tilt_support,
        swivel_block,
        elem_a=right_support_plate,
        elem_b="swivel_body",
        name="right_support_mounted_to_swivel_block",
    )
    ctx.expect_gap(
        head_frame,
        left_tilt_support,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=left_trunnion,
        negative_elem=left_support_plate,
        name="left_trunnion_seated_in_support",
    )
    ctx.expect_gap(
        right_tilt_support,
        head_frame,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=right_support_plate,
        negative_elem=right_trunnion,
        name="right_trunnion_seated_in_support",
    )

    ctx.expect_overlap(
        head_frame,
        left_tilt_support,
        axes="z",
        min_overlap=0.150,
        name="head_frame_and_left_support_share_vertical_band",
    )
    ctx.expect_overlap(
        head_frame,
        right_tilt_support,
        axes="z",
        min_overlap=0.150,
        name="head_frame_and_right_support_share_vertical_band",
    )

    plate_aabb = ctx.part_world_aabb(wall_plate)
    frame_aabb = ctx.part_world_aabb(head_frame)
    if plate_aabb is not None:
        plate_h = plate_aabb[1][2] - plate_aabb[0][2]
        plate_w = plate_aabb[1][1] - plate_aabb[0][1]
        ctx.check(
            "wall_plate_realistic_size",
            0.34 <= plate_h <= 0.45 and 0.14 <= plate_w <= 0.22,
            f"wall plate dims were {(plate_w, plate_h)}",
        )
    if frame_aabb is not None:
        frame_w = frame_aabb[1][1] - frame_aabb[0][1]
        frame_h = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "head_frame_realistic_size",
            0.30 <= frame_w <= 0.38 and 0.20 <= frame_h <= 0.28,
            f"head frame dims were {(frame_w, frame_h)}",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    for joint in (
        wall_to_inner_arm,
        inner_to_outer_arm,
        outer_to_swivel_block,
        swivel_block_to_head_frame,
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
