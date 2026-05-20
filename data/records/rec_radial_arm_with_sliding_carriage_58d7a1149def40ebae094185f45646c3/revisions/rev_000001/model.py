from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.19
BASE_W = 0.15
BASE_T = 0.014

SHOULDER_Z = 0.195
SHOULDER_Y = 0.0
TURN_PLATE_R = 0.026
TURN_PLATE_T = 0.008
HUB_OUTER_R = 0.028
HUB_LEN = 0.020
ARM_BEAM_Y = 0.0
ARM_ROOT_X = 0.022
ARM_ROOT_L = 0.078
ARM_ROOT_W = 0.030
ARM_ROOT_H = 0.026
ARM_BEAM_L = 0.285
ARM_BEAM_W = 0.022
ARM_BEAM_H = 0.020
RAIL_H = 0.004
RAIL_W = 0.006
RAIL_OFFSET_Y = 0.006
RAIL_START_X = 0.10
RAIL_L = 0.245
TIP_STOP_X = 0.355
TIP_STOP_L = 0.012
TRUCK_JOINT_X = 0.215
TRUCK_TRAVEL = 0.085
TRUCK_MOUNT_Z = 0.024
TRUCK_WALL_CENTER_Y = 0.013


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _support_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T).translate((0.0, 0.0, BASE_T * 0.5))

    rear_spine = (
        cq.Workplane("XY")
        .box(0.026, 0.070, 0.160)
        .translate((-0.064, 0.0, 0.090))
    )

    bridge = (
        cq.Workplane("XY")
        .box(0.070, 0.050, 0.018)
        .translate((-0.038, 0.0, 0.157))
    )

    shoulder_pedestal = (
        cq.Workplane("XY")
        .box(0.024, 0.046, 0.030)
        .translate((-0.010, 0.0, SHOULDER_Z - TURN_PLATE_T - 0.015))
    )

    turn_plate = (
        cq.Workplane("XY")
        .circle(TURN_PLATE_R)
        .extrude(TURN_PLATE_T)
        .translate((0.0, 0.0, SHOULDER_Z - TURN_PLATE_T))
    )

    front_feet = (
        cq.Workplane("XY")
        .box(0.040, 0.058, 0.010)
        .translate((0.052, 0.0, 0.005))
    )

    return _combine(base, rear_spine, bridge, shoulder_pedestal, turn_plate, front_feet)


def _arm_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(HUB_OUTER_R).extrude(HUB_LEN)

    root_block = (
        cq.Workplane("XY")
        .box(ARM_ROOT_L, ARM_ROOT_W, ARM_ROOT_H)
        .translate((ARM_ROOT_X + (ARM_ROOT_L * 0.5), ARM_BEAM_Y, ARM_ROOT_H * 0.5))
    )

    beam = (
        cq.Workplane("XY")
        .box(ARM_BEAM_L, ARM_BEAM_W, ARM_BEAM_H)
        .translate((0.085 + (ARM_BEAM_L * 0.5), ARM_BEAM_Y, ARM_BEAM_H * 0.5))
    )

    rail_pos = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H)
        .translate(
            (
                RAIL_START_X + (RAIL_L * 0.5),
                ARM_BEAM_Y + RAIL_OFFSET_Y,
                ARM_BEAM_H + (RAIL_H * 0.5),
            )
        )
    )
    rail_neg = (
        cq.Workplane("XY")
        .box(RAIL_L, RAIL_W, RAIL_H)
        .translate(
            (
                RAIL_START_X + (RAIL_L * 0.5),
                ARM_BEAM_Y - RAIL_OFFSET_Y,
                ARM_BEAM_H + (RAIL_H * 0.5),
            )
        )
    )

    tip_stop = (
        cq.Workplane("XY")
        .box(TIP_STOP_L, 0.022, 0.018)
        .translate((TIP_STOP_X, ARM_BEAM_Y, ARM_BEAM_H * 0.5))
    )

    nose_cap = (
        cq.Workplane("XY")
        .box(0.028, 0.022, 0.018)
        .translate((0.345, ARM_BEAM_Y, ARM_BEAM_H * 0.5))
    )

    return _combine(hub, root_block, beam, rail_pos, rail_neg, tip_stop, nose_cap)


def _truck_shape() -> cq.Workplane:
    top_bridge = (
        cq.Workplane("XY")
        .box(0.082, 0.052, 0.010)
        .translate((0.0, 0.0, 0.025))
    )

    side_wall_pos = (
        cq.Workplane("XY")
        .box(0.082, 0.008, 0.022)
        .translate((0.0, TRUCK_WALL_CENTER_Y, 0.011))
    )
    side_wall_neg = (
        cq.Workplane("XY")
        .box(0.082, 0.008, 0.022)
        .translate((0.0, -TRUCK_WALL_CENTER_Y, 0.011))
    )

    top_pad = (
        cq.Workplane("XY")
        .box(0.046, 0.040, 0.012)
        .translate((0.0, 0.0, 0.036))
    )

    nose_pad = (
        cq.Workplane("XY")
        .box(0.024, 0.030, 0.008)
        .translate((0.020, 0.0, 0.015))
    )

    tail_pad = (
        cq.Workplane("XY")
        .box(0.018, 0.026, 0.008)
        .translate((-0.024, 0.0, 0.015))
    )

    return _combine(top_bridge, side_wall_pos, side_wall_neg, top_pad, nose_pad, tail_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_radial_slide_arm")

    model.material("support_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("arm_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("truck_orange", rgba=(0.86, 0.43, 0.14, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_support_shape(), "support_shell"),
        material="support_steel",
        name="support_shell",
    )
    support.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.24)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_shape(), "arm_shell"),
        material="arm_aluminum",
        name="arm_shell",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.39, 0.060, 0.060)),
        mass=2.2,
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
    )

    truck = model.part("truck")
    truck.visual(
        mesh_from_cadquery(_truck_shape(), "truck_shell"),
        material="truck_orange",
        name="truck_shell",
    )
    truck.inertial = Inertial.from_geometry(
        Box((0.082, 0.064, 0.044)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.0, SHOULDER_Y, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=0.95,
            effort=30.0,
            velocity=1.2,
        ),
    )

    model.articulation(
        "truck_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=truck,
        origin=Origin(
            xyz=(
                TRUCK_JOINT_X,
                ARM_BEAM_Y,
                TRUCK_MOUNT_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TRUCK_TRAVEL,
            effort=16.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    arm = object_model.get_part("arm")
    truck = object_model.get_part("truck")
    shoulder = object_model.get_articulation("shoulder")
    truck_slide = object_model.get_articulation("truck_slide")

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
        "shoulder_axis_points_for_upward_lift",
        tuple(shoulder.axis) == (0.0, -1.0, 0.0),
        details=f"expected shoulder axis (0,-1,0), got {shoulder.axis}",
    )
    ctx.check(
        "truck_slide_axis_runs_along_arm",
        tuple(truck_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected truck slide axis (1,0,0), got {truck_slide.axis}",
    )
    ctx.check(
        "shoulder_origin_is_at_support_head",
        tuple(shoulder.origin.xyz) == (0.0, SHOULDER_Y, SHOULDER_Z),
        details=f"expected shoulder origin at {(0.0, SHOULDER_Y, SHOULDER_Z)}, got {shoulder.origin.xyz}",
    )

    with ctx.pose({shoulder: 0.0, truck_slide: 0.0}):
        ctx.expect_contact(arm, truck, name="truck_guides_touch_arm_rails")
        ctx.expect_overlap(arm, truck, axes="xy", min_overlap=0.03, name="truck_footprint_stays_over_arm")

    with ctx.pose({shoulder: 0.55, truck_slide: truck_slide.motion_limits.upper}):
        ctx.expect_contact(arm, truck, name="truck_guides_touch_arm_when_extended")

    with ctx.pose({shoulder: 0.0, truck_slide: truck_slide.motion_limits.lower}):
        truck_retracted = ctx.part_world_position(truck)
        arm_closed_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")

    with ctx.pose({shoulder: 0.0, truck_slide: truck_slide.motion_limits.upper}):
        truck_extended = ctx.part_world_position(truck)

    with ctx.pose({shoulder: 0.75, truck_slide: 0.0}):
        arm_raised_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")

    ctx.check(
        "truck_extension_moves_forward",
        truck_retracted is not None
        and truck_extended is not None
        and (truck_extended[0] - truck_retracted[0]) > 0.07,
        details=f"retracted={truck_retracted}, extended={truck_extended}",
    )
    ctx.check(
        "positive_shoulder_motion_lifts_arm_tip",
        arm_closed_aabb is not None
        and arm_raised_aabb is not None
        and (arm_raised_aabb[1][2] - arm_closed_aabb[1][2]) > 0.10,
        details=f"closed={arm_closed_aabb}, raised={arm_raised_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
