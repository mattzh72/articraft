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


FRAME_PLATE_X = -0.054
FRAME_PLATE_T = 0.008
FRAME_PLATE_W = 0.112
FRAME_PLATE_H = 0.168
FRAME_HOLE_Y = 0.036
FRAME_HOLE_Z = 0.054

ARM_PIVOT_R = 0.018
ARM_PIVOT_T = 0.012
HEAD_PIVOT_R = 0.014
HEAD_PIVOT_T = 0.010

LOWER_ARM_LENGTH = 0.240
UPPER_ARM_LENGTH = 0.220
HEAD_TILT_OFFSET = 0.034


def _combine(*items: cq.Workplane) -> cq.Workplane:
    wp = cq.Workplane("XY")
    for item in items:
        wp = wp.add(item.val())
    return wp.combine(clean=True)


def _rounded_beam(length: float, width: float, height: float, *, center_x: float, fillet: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|X")
        .fillet(fillet)
        .translate((center_x, 0.0, 0.0))
    )


def _pivot_disk(center_x: float, radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(thickness / 2.0, both=True)
        .translate((center_x, 0.0, 0.0))
    )


def _pivot_receiver_joint(*, pivot_center_x: float, pivot_radius: float, pivot_thickness: float, neck_end_x: float, neck_width: float, neck_height: float) -> cq.Workplane:
    disk = _pivot_disk(pivot_center_x, pivot_radius, pivot_thickness)
    neck_len = neck_end_x - (pivot_center_x + pivot_thickness / 2.0)
    neck = (
        cq.Workplane("XY")
        .box(neck_len, neck_width, neck_height)
        .translate((pivot_center_x + pivot_thickness / 2.0 + neck_len / 2.0, 0.0, 0.0))
    )
    return _combine(disk, neck)


def _pivot_parent_joint(*, axis_x: float, pivot_radius: float, pivot_thickness: float, x_start: float, strut_width: float, strut_height: float, strut_gap_z: float) -> cq.Workplane:
    pivot_center_x = axis_x - pivot_thickness / 2.0
    disk = _pivot_disk(pivot_center_x, pivot_radius, pivot_thickness)
    strut_len = pivot_center_x - x_start
    upper_strut = (
        cq.Workplane("XY")
        .box(strut_len, strut_width, strut_height)
        .translate(((x_start + pivot_center_x) / 2.0, 0.0, strut_gap_z))
    )
    lower_strut = (
        cq.Workplane("XY")
        .box(strut_len, strut_width, strut_height)
        .translate(((x_start + pivot_center_x) / 2.0, 0.0, -strut_gap_z))
    )
    return _combine(disk, upper_strut, lower_strut)


def _rear_frame_shape() -> cq.Workplane:
    wall_plate = (
        cq.Workplane("XY")
        .box(FRAME_PLATE_T, FRAME_PLATE_W, FRAME_PLATE_H)
        .edges("|X")
        .fillet(0.006)
        .translate((FRAME_PLATE_X, 0.0, 0.0))
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-FRAME_HOLE_Y, -FRAME_HOLE_Z),
                (-FRAME_HOLE_Y, FRAME_HOLE_Z),
                (FRAME_HOLE_Y, -FRAME_HOLE_Z),
                (FRAME_HOLE_Y, FRAME_HOLE_Z),
            ]
        )
        .hole(0.007)
    )
    upper_rail = _rounded_beam(0.056, 0.032, 0.014, center_x=-0.028, fillet=0.0035).translate((0.0, 0.0, 0.020))
    lower_rail = _rounded_beam(0.056, 0.032, 0.014, center_x=-0.028, fillet=0.0035).translate((0.0, 0.0, -0.020))
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.014, 0.032, 0.074)
        .edges("|X")
        .fillet(0.003)
        .translate((-0.028, 0.0, 0.0))
    )
    shoulder_joint = _pivot_parent_joint(
        axis_x=0.0,
        pivot_radius=ARM_PIVOT_R,
        pivot_thickness=ARM_PIVOT_T,
        x_start=-0.028,
        strut_width=0.014,
        strut_height=0.010,
        strut_gap_z=0.018,
    )
    return _combine(wall_plate, upper_rail, lower_rail, rear_bridge, shoulder_joint)


def _lower_arm_shape() -> cq.Workplane:
    rear_receiver = _pivot_receiver_joint(
        pivot_center_x=ARM_PIVOT_T / 2.0,
        pivot_radius=ARM_PIVOT_R,
        pivot_thickness=ARM_PIVOT_T,
        neck_end_x=0.036,
        neck_width=0.026,
        neck_height=0.042,
    )
    body = _rounded_beam(0.164, 0.032, 0.050, center_x=0.118, fillet=0.007)
    front_neck = _rounded_beam(0.020, 0.026, 0.042, center_x=0.208, fillet=0.004)
    elbow_joint = _pivot_parent_joint(
        axis_x=LOWER_ARM_LENGTH,
        pivot_radius=ARM_PIVOT_R,
        pivot_thickness=ARM_PIVOT_T,
        x_start=0.198,
        strut_width=0.014,
        strut_height=0.010,
        strut_gap_z=0.018,
    )
    return _combine(rear_receiver, body, front_neck, elbow_joint)


def _upper_arm_shape() -> cq.Workplane:
    rear_receiver = _pivot_receiver_joint(
        pivot_center_x=ARM_PIVOT_T / 2.0,
        pivot_radius=ARM_PIVOT_R,
        pivot_thickness=ARM_PIVOT_T,
        neck_end_x=0.034,
        neck_width=0.024,
        neck_height=0.040,
    )
    body = _rounded_beam(0.150, 0.030, 0.046, center_x=0.108, fillet=0.0065)
    front_neck = _rounded_beam(0.018, 0.022, 0.036, center_x=0.192, fillet=0.0035)
    swivel_joint = _pivot_parent_joint(
        axis_x=UPPER_ARM_LENGTH,
        pivot_radius=ARM_PIVOT_R,
        pivot_thickness=ARM_PIVOT_T,
        x_start=0.186,
        strut_width=0.012,
        strut_height=0.008,
        strut_gap_z=0.016,
    )
    return _combine(rear_receiver, body, front_neck, swivel_joint)


def _head_swivel_shape() -> cq.Workplane:
    rear_receiver = _pivot_receiver_joint(
        pivot_center_x=HEAD_PIVOT_T / 2.0,
        pivot_radius=HEAD_PIVOT_R,
        pivot_thickness=HEAD_PIVOT_T,
        neck_end_x=0.020,
        neck_width=0.018,
        neck_height=0.026,
    )
    body = _rounded_beam(0.018, 0.020, 0.028, center_x=0.016, fillet=0.003)
    front_neck = _rounded_beam(0.018, 0.016, 0.022, center_x=0.025, fillet=0.0025)
    tilt_joint = _pivot_parent_joint(
        axis_x=HEAD_TILT_OFFSET,
        pivot_radius=HEAD_PIVOT_R,
        pivot_thickness=HEAD_PIVOT_T,
        x_start=0.022,
        strut_width=0.010,
        strut_height=0.006,
        strut_gap_z=0.011,
    )
    return _combine(rear_receiver, body, front_neck, tilt_joint)


def _display_plate_shape() -> cq.Workplane:
    tilt_receiver = _pivot_receiver_joint(
        pivot_center_x=HEAD_PIVOT_T / 2.0,
        pivot_radius=0.013,
        pivot_thickness=HEAD_PIVOT_T,
        neck_end_x=0.022,
        neck_width=0.020,
        neck_height=0.028,
    )
    neck = _rounded_beam(0.020, 0.024, 0.034, center_x=0.018, fillet=0.0025)
    plate = (
        cq.Workplane("XY")
        .box(0.004, 0.082, 0.082)
        .edges("|X")
        .fillet(0.004)
        .translate((0.030, 0.0, 0.0))
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.030, -0.030), (-0.030, 0.030), (0.030, -0.030), (0.030, 0.030)])
        .hole(0.006)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.024, 0.024)
        .cutThruAll()
    )
    return _combine(tilt_receiver, neck, plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_monitor_mount")

    model.material("powder_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("arm_silver", rgba=(0.63, 0.67, 0.71, 1.0))
    model.material("hardware_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("plate_gray", rgba=(0.50, 0.53, 0.57, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        mesh_from_cadquery(_rear_frame_shape(), "rear_frame"),
        material="powder_charcoal",
        name="rear_frame_body",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_lower_arm_shape(), "lower_arm"),
        material="arm_silver",
        name="lower_arm_body",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_upper_arm_shape(), "upper_arm"),
        material="arm_silver",
        name="upper_arm_body",
    )

    head_swivel = model.part("head_swivel")
    head_swivel.visual(
        mesh_from_cadquery(_head_swivel_shape(), "head_swivel"),
        material="hardware_black",
        name="head_swivel_body",
    )

    display_plate = model.part("display_plate")
    display_plate.visual(
        mesh_from_cadquery(_display_plate_shape(), "display_plate"),
        material="plate_gray",
        name="display_plate_body",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.6, lower=-1.9, upper=1.9),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8, lower=-2.2, upper=2.2),
    )
    model.articulation(
        "swivel_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head_swivel,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=display_plate,
        origin=Origin(xyz=(HEAD_TILT_OFFSET, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.6, lower=-0.65, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_size(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (maxs[0] - mins[0], maxs[1] - mins[1], maxs[2] - mins[2])

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            0.5 * (mins[0] + maxs[0]),
            0.5 * (mins[1] + maxs[1]),
            0.5 * (mins[2] + maxs[2]),
        )

    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head_swivel = object_model.get_part("head_swivel")
    display_plate = object_model.get_part("display_plate")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    swivel_joint = object_model.get_articulation("swivel_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")

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
        "all_primary_parts_present",
        all(part is not None for part in (rear_frame, lower_arm, upper_arm, head_swivel, display_plate)),
        "One or more primary parts could not be resolved.",
    )
    ctx.check(
        "all_primary_joints_present",
        all(joint is not None for joint in (shoulder_joint, elbow_joint, swivel_joint, tilt_joint)),
        "One or more primary articulations could not be resolved.",
    )

    ctx.expect_contact(lower_arm, rear_frame, contact_tol=0.0005, name="shoulder_has_real_support_contact")
    ctx.expect_contact(upper_arm, lower_arm, contact_tol=0.0005, name="elbow_has_real_support_contact")
    ctx.expect_contact(head_swivel, upper_arm, contact_tol=0.0005, name="swivel_has_real_support_contact")
    ctx.expect_contact(display_plate, head_swivel, contact_tol=0.0005, name="tilt_has_real_support_contact")

    ctx.expect_origin_gap(display_plate, rear_frame, axis="x", min_gap=0.46, name="display_plate_projects_forward_of_frame")

    rear_aabb = ctx.part_world_aabb(rear_frame)
    lower_aabb = ctx.part_world_aabb(lower_arm)
    upper_aabb = ctx.part_world_aabb(upper_arm)
    head_aabb = ctx.part_world_aabb(head_swivel)
    plate_aabb = ctx.part_world_aabb(display_plate)

    ctx.check(
        "head_hardware_smaller_than_upper_arm_section",
        (
            head_aabb is not None
            and upper_aabb is not None
            and _aabb_size(head_aabb)[1] < _aabb_size(upper_aabb)[1]
            and _aabb_size(head_aabb)[2] < _aabb_size(upper_aabb)[2]
        ),
        f"Head size={_aabb_size(head_aabb)} upper arm size={_aabb_size(upper_aabb)}",
    )
    ctx.check(
        "display_plate_compact_relative_to_rear_frame",
        (
            rear_aabb is not None
            and plate_aabb is not None
            and _aabb_size(plate_aabb)[1] < _aabb_size(rear_aabb)[1]
            and _aabb_size(plate_aabb)[2] < _aabb_size(rear_aabb)[2]
        ),
        f"Plate size={_aabb_size(plate_aabb)} frame size={_aabb_size(rear_aabb)}",
    )

    with ctx.pose({shoulder_joint: 0.55}):
        moved_lower = _aabb_center(ctx.part_world_aabb(lower_arm))
        ctx.check(
            "shoulder_positive_rotates_lower_arm_leftward",
            moved_lower is not None and moved_lower[1] > 0.05,
            f"Lower arm center after +shoulder pose: {moved_lower}",
        )

    with ctx.pose({elbow_joint: 0.65}):
        moved_upper = _aabb_center(ctx.part_world_aabb(upper_arm))
        ctx.check(
            "elbow_positive_rotates_upper_arm_leftward",
            moved_upper is not None and moved_upper[1] > 0.06,
            f"Upper arm center after +elbow pose: {moved_upper}",
        )

    with ctx.pose({swivel_joint: 0.65}):
        moved_plate = _aabb_center(ctx.part_world_aabb(display_plate))
        ctx.check(
            "swivel_positive_rotates_head_leftward",
            moved_plate is not None and moved_plate[1] > 0.02,
            f"Display plate center after +swivel pose: {moved_plate}",
        )

    rest_plate_size = _aabb_size(ctx.part_world_aabb(display_plate))
    with ctx.pose({tilt_joint: 0.55}):
        tilted_plate_size = _aabb_size(ctx.part_world_aabb(display_plate))
        ctx.check(
            "tilt_changes_display_plate_pitch",
            rest_plate_size is not None and tilted_plate_size is not None and tilted_plate_size[0] > rest_plate_size[0] + 0.035,
            f"Rest plate size={rest_plate_size}, tilted plate size={tilted_plate_size}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
