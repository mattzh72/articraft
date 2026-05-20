from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


WALL_PLATE_DEPTH = 0.016
WALL_PLATE_WIDTH = 0.180
WALL_PLATE_HEIGHT = 0.460
WALL_PLATE_CENTER_X = -0.068
WALL_HUB_RADIUS = 0.042
WALL_HUB_HEIGHT = 0.014

FIRST_ARM_LENGTH = 0.400
FIRST_ARM_BODY_WIDTH = 0.082
FIRST_ARM_BODY_HEIGHT = 0.016
FIRST_ARM_BODY_Z = 0.024
FIRST_ARM_REAR_RING_OUTER = 0.042
FIRST_ARM_FRONT_HUB_RADIUS = 0.034
FIRST_ARM_FRONT_HUB_HEIGHT = 0.014

SECOND_ARM_LENGTH = 0.260
SECOND_ARM_BODY_WIDTH = 0.070
SECOND_ARM_BODY_HEIGHT = 0.016
SECOND_ARM_BODY_Z = -0.024
SECOND_ARM_REAR_RING_OUTER = 0.034
SECOND_ARM_FRONT_HUB_RADIUS = 0.032
SECOND_ARM_FRONT_HUB_HEIGHT = 0.014

HEAD_SWIVEL_RING_OUTER = 0.034
HEAD_FRAME_NECK_X = 0.065
HEAD_FRAME_OUTER = 0.206
HEAD_FRAME_INNER = 0.160
HEAD_FRAME_DEPTH = 0.014
HEAD_TILT_HUB_RADIUS = 0.010
HEAD_TILT_HUB_LENGTH = 0.014
HEAD_TILT_HUB_Y = 0.066

CRADLE_OUTER = 0.145
CRADLE_INNER = 0.092
CRADLE_DEPTH = 0.010
CRADLE_TUBE_OUTER = 0.018
CRADLE_TUBE_INNER = HEAD_TILT_HUB_RADIUS
CRADLE_TUBE_LENGTH = 0.124

WALL_JOINT_X = 0.050
HEAD_SWIVEL_STANDOFF = 0.008
HEAD_TILT_AXIS_X = 0.112

WALL_TO_FIRST_LIMITS = (-0.55, 0.55)
FIRST_TO_SECOND_LIMITS = (-2.20, 2.20)
SECOND_TO_HEAD_LIMITS = (-0.45, 0.45)
HEAD_TILT_LIMITS = (-0.28, 0.18)


def _box(x: float, y: float, z: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(x, y, z).translate(center)


def _cylinder_z(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(height / 2.0, both=True)
        .translate(center)
    )


def _ring_z(
    outer_radius: float,
    inner_radius: float,
    height: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height / 2.0, both=True)
        .translate(center)
    )


def _ring_y(
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _slot_cut_x(
    length_y: float,
    length_z: float,
    center: tuple[float, float, float],
    *,
    vertical: bool = False,
) -> cq.Workplane:
    angle = 90 if vertical else 0
    return (
        cq.Workplane("YZ")
        .slot2D(length_y, length_z, angle)
        .extrude(0.030, both=True)
        .translate(center)
    )


def _wall_plate_shape() -> cq.Workplane:
    plate = _box(
        WALL_PLATE_DEPTH,
        WALL_PLATE_WIDTH,
        WALL_PLATE_HEIGHT,
        (WALL_PLATE_CENTER_X, 0.0, 0.0),
    ).edges("|Z").fillet(0.006)

    spine = _box(0.022, 0.068, 0.265, (-0.057, 0.0, 0.0))
    pivot_housing = _box(0.060, 0.036, 0.050, (0.020, 0.0, 0.014)).edges("|X").fillet(0.005)
    upper_gusset = _box(0.038, 0.020, 0.046, (-0.030, 0.0, 0.054))
    lower_gusset = _box(0.038, 0.020, 0.046, (-0.030, 0.0, -0.030))
    trim_cover = _box(0.016, 0.084, 0.150, (-0.058, 0.0, -0.020))
    cable_saddle = _box(0.018, 0.040, 0.030, (-0.024, 0.0, -0.074))

    shape = plate.union(spine).union(trim_cover).union(pivot_housing)
    shape = shape.union(upper_gusset).union(lower_gusset).union(cable_saddle)

    slot_centers = (
        (WALL_PLATE_CENTER_X, -0.050, 0.155),
        (WALL_PLATE_CENTER_X, 0.050, 0.155),
        (WALL_PLATE_CENTER_X, -0.050, 0.030),
        (WALL_PLATE_CENTER_X, 0.050, 0.030),
        (WALL_PLATE_CENTER_X, -0.050, -0.095),
        (WALL_PLATE_CENTER_X, 0.050, -0.095),
    )
    for center in slot_centers:
        shape = shape.cut(_slot_cut_x(0.020, 0.008, center, vertical=True))

    cable_window = (
        cq.Workplane("YZ")
        .rect(0.044, 0.066)
        .extrude(0.030, both=True)
        .translate((WALL_PLATE_CENTER_X, 0.0, -0.160))
    )
    trim_recess = _box(0.010, 0.110, 0.160, (WALL_PLATE_CENTER_X + 0.002, 0.0, -0.030))
    shape = shape.cut(cable_window).cut(trim_recess)
    return shape


def _arm_shape(
    *,
    length: float,
    body_width: float,
    body_height: float,
    body_z: float,
    rear_outer_radius: float,
    front_hub_radius: float,
    front_hub_height: float,
    pivot_z: float,
    body_start: float,
    body_end: float,
) -> cq.Workplane:
    rear_block = _box(0.042, 0.060, 0.024, (0.021, 0.0, (body_z + pivot_z) / 2.0)).edges("|X").fillet(0.004)
    rear_cap = _box(0.020, 0.072, 0.030, (0.010, 0.0, (body_z + pivot_z) / 2.0))

    body = _box(
        body_end - body_start,
        body_width,
        body_height,
        ((body_start + body_end) / 2.0, 0.0, body_z),
    ).edges("|X").fillet(0.004)

    upper_web = _box(
        body_end - body_start + 0.012,
        0.016,
        0.010,
        ((body_start + body_end) / 2.0, 0.0, body_z + body_height / 2.0 + 0.004),
    )
    lower_web = _box(
        body_end - body_start + 0.012,
        0.016,
        0.010,
        ((body_start + body_end) / 2.0, 0.0, body_z - body_height / 2.0 - 0.004),
    )

    front_transition = _box(
        0.040,
        min(body_width * 0.44, 0.034),
        0.022,
        (body_end + 0.016, 0.0, (body_z + pivot_z) / 2.0),
    )
    front_block = _box(0.040, 0.050, 0.024, (length - 0.020, 0.0, (body_z + pivot_z) / 2.0)).edges("|X").fillet(0.004)

    shape = rear_cap.union(rear_block).union(body).union(upper_web).union(lower_web).union(front_transition).union(front_block)

    side_relief_len = max(body_end - body_start - 0.060, 0.100)
    side_relief_height = body_height * 0.62
    side_relief_depth = 0.007
    relief_x = (body_start + body_end) / 2.0
    upper_center_y = body_width / 2.0 - side_relief_depth / 2.0
    lower_center_y = -upper_center_y
    for relief_y in (upper_center_y, lower_center_y):
        shape = shape.cut(
            _box(
                side_relief_len,
                side_relief_depth,
                side_relief_height,
                (relief_x, relief_y, body_z),
            )
        )

    cable_channel = _box(
        max(body_end - body_start - 0.040, 0.090),
        body_width * 0.36,
        0.006,
        ((body_start + body_end) / 2.0, 0.0, body_z - body_height / 2.0 + 0.003),
    )
    shape = shape.cut(cable_channel)

    inspection_slot = _box(
        0.075,
        body_width * 0.34,
        body_height * 0.55,
        (body_start + 0.028, 0.0, body_z),
    )
    return shape.cut(inspection_slot)


def _first_arm_shape() -> cq.Workplane:
    arm = _arm_shape(
        length=FIRST_ARM_LENGTH,
        body_width=FIRST_ARM_BODY_WIDTH,
        body_height=FIRST_ARM_BODY_HEIGHT,
        body_z=FIRST_ARM_BODY_Z,
        rear_outer_radius=FIRST_ARM_REAR_RING_OUTER,
        front_hub_radius=FIRST_ARM_FRONT_HUB_RADIUS,
        front_hub_height=FIRST_ARM_FRONT_HUB_HEIGHT,
        pivot_z=0.007,
        body_start=0.084,
        body_end=0.312,
    )
    front_contact_pad = _box(0.024, 0.036, 0.008, (FIRST_ARM_LENGTH - 0.012, 0.0, 0.0045))
    return arm.union(front_contact_pad)


def _second_arm_shape() -> cq.Workplane:
    arm = _arm_shape(
        length=SECOND_ARM_LENGTH,
        body_width=SECOND_ARM_BODY_WIDTH,
        body_height=SECOND_ARM_BODY_HEIGHT,
        body_z=SECOND_ARM_BODY_Z,
        rear_outer_radius=SECOND_ARM_REAR_RING_OUTER,
        front_hub_radius=SECOND_ARM_FRONT_HUB_RADIUS,
        front_hub_height=SECOND_ARM_FRONT_HUB_HEIGHT,
        pivot_z=-0.007,
        body_start=0.068,
        body_end=0.206,
    )
    rear_contact_pad = _box(0.024, 0.036, 0.008, (0.012, 0.0, -0.0035))
    front_contact_pad = _box(0.024, 0.030, 0.008, (SECOND_ARM_LENGTH - 0.006, 0.0, -0.0035))
    return arm.union(rear_contact_pad).union(front_contact_pad)


def _head_frame_shape() -> cq.Workplane:
    rear_pad = _box(0.004, 0.030, 0.008, (0.0, 0.0, -0.004))
    root_block = _box(0.018, 0.040, 0.016, (0.017, 0.0, -0.004)).edges("|X").fillet(0.003)
    upper_strut = _box(0.064, 0.014, 0.010, (0.050, 0.0, 0.050)).edges("|X").fillet(0.003)
    lower_strut = _box(0.064, 0.014, 0.010, (0.050, 0.0, -0.050)).edges("|X").fillet(0.003)

    top_bar = _box(HEAD_FRAME_DEPTH, HEAD_FRAME_OUTER, 0.016, (0.094, 0.0, 0.082))
    bottom_bar = _box(HEAD_FRAME_DEPTH, HEAD_FRAME_OUTER, 0.016, (0.094, 0.0, -0.082))
    left_upright = _box(0.012, 0.014, 0.140, (0.094, -0.081, 0.0))
    right_upright = _box(0.012, 0.014, 0.140, (0.094, 0.081, 0.0))
    left_yoke = _box(0.006, 0.012, 0.048, (HEAD_TILT_AXIS_X - 0.003, -HEAD_TILT_HUB_Y, 0.0))
    right_yoke = _box(0.006, 0.012, 0.048, (HEAD_TILT_AXIS_X - 0.003, HEAD_TILT_HUB_Y, 0.0))
    left_yoke_arm = _box(0.024, 0.006, 0.014, (0.100, -HEAD_TILT_HUB_Y, 0.0)).edges("|X").fillet(0.002)
    right_yoke_arm = _box(0.024, 0.006, 0.014, (0.100, HEAD_TILT_HUB_Y, 0.0)).edges("|X").fillet(0.002)

    return (
        rear_pad.union(root_block)
        .union(upper_strut)
        .union(lower_strut)
        .union(top_bar)
        .union(bottom_bar)
        .union(left_upright)
        .union(right_upright)
        .union(left_yoke)
        .union(right_yoke)
        .union(left_yoke_arm)
        .union(right_yoke_arm)
    )


def _tilt_cradle_shape() -> cq.Workplane:
    left_tab = _box(0.002, 0.010, 0.042, (0.001, -HEAD_TILT_HUB_Y, 0.0))
    right_tab = _box(0.002, 0.010, 0.042, (0.001, HEAD_TILT_HUB_Y, 0.0))
    rear_bridge = _box(0.028, 0.104, 0.012, (0.022, 0.0, 0.0))
    front_spine = _box(0.094, 0.030, 0.014, (0.072, 0.0, 0.0)).edges("|X").fillet(0.003)

    outer_plate = _box(CRADLE_DEPTH, CRADLE_OUTER, CRADLE_OUTER, (0.176, 0.0, 0.0))
    center_relief = _box(CRADLE_DEPTH + 0.004, 0.064, 0.064, (0.176, 0.0, 0.0))
    outer_plate = outer_plate.cut(center_relief)

    lower_lip = _box(0.016, 0.126, 0.016, (0.174, 0.0, -0.062))

    shape = left_tab.union(right_tab).union(rear_bridge).union(front_spine).union(outer_plate).union(lower_lip)

    vesa_holes = (
        (-0.050, -0.050),
        (0.050, -0.050),
        (-0.050, 0.050),
        (0.050, 0.050),
    )
    for y_pos, z_pos in vesa_holes:
        hole = (
            cq.Workplane("YZ")
            .circle(0.0045)
            .extrude(0.020, both=True)
            .translate((0.176, y_pos, z_pos))
        )
        shape = shape.cut(hole)

    slot_pair = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, -0.034), (0.0, 0.034)])
        .slot2D(0.012, 0.005, 90)
        .extrude(0.020, both=True)
        .translate((0.176, 0.0, 0.0))
    )
    return shape.cut(slot_pair)


def _axis_close(axis: tuple[float, float, float], target: tuple[float, float, float]) -> bool:
    return all(abs(a - b) < 1e-6 for a, b in zip(axis, target))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_tv_wall_mount")

    model.material("powder_black", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("satin_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("cast_gray", rgba=(0.47, 0.49, 0.52, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate"),
        material="powder_black",
        name="wall_plate_shell",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.090, WALL_PLATE_WIDTH, WALL_PLATE_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
    )

    first_arm = model.part("first_arm")
    first_arm.visual(
        mesh_from_cadquery(_first_arm_shape(), "first_arm"),
        material="satin_graphite",
        name="first_arm_shell",
    )
    first_arm.inertial = Inertial.from_geometry(
        Box((FIRST_ARM_LENGTH, 0.090, 0.060)),
        mass=2.4,
        origin=Origin(xyz=(FIRST_ARM_LENGTH / 2.0, 0.0, 0.010)),
    )

    second_arm = model.part("second_arm")
    second_arm.visual(
        mesh_from_cadquery(_second_arm_shape(), "second_arm"),
        material="satin_graphite",
        name="second_arm_shell",
    )
    second_arm.inertial = Inertial.from_geometry(
        Box((SECOND_ARM_LENGTH, 0.080, 0.055)),
        mass=1.6,
        origin=Origin(xyz=(SECOND_ARM_LENGTH / 2.0, 0.0, -0.008)),
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        mesh_from_cadquery(_head_frame_shape(), "head_frame"),
        material="cast_gray",
        name="head_frame_shell",
    )
    head_frame.inertial = Inertial.from_geometry(
        Box((0.120, HEAD_FRAME_OUTER, HEAD_FRAME_OUTER)),
        mass=1.4,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        mesh_from_cadquery(_tilt_cradle_shape(), "tilt_cradle"),
        material="powder_black",
        name="tilt_cradle_shell",
    )
    tilt_cradle.inertial = Inertial.from_geometry(
        Box((0.030, CRADLE_OUTER, CRADLE_OUTER)),
        mass=0.9,
        origin=Origin(),
    )

    model.articulation(
        "wall_to_first_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=first_arm,
        origin=Origin(xyz=(WALL_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=WALL_TO_FIRST_LIMITS[0],
            upper=WALL_TO_FIRST_LIMITS[1],
            effort=55.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "first_to_second_arm",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(FIRST_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=FIRST_TO_SECOND_LIMITS[0],
            upper=FIRST_TO_SECOND_LIMITS[1],
            effort=40.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "second_to_head",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=head_frame,
        origin=Origin(xyz=(SECOND_ARM_LENGTH + HEAD_SWIVEL_STANDOFF, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=SECOND_TO_HEAD_LIMITS[0],
            upper=SECOND_TO_HEAD_LIMITS[1],
            effort=22.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=tilt_cradle,
        origin=Origin(xyz=(HEAD_TILT_AXIS_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=HEAD_TILT_LIMITS[0],
            upper=HEAD_TILT_LIMITS[1],
            effort=18.0,
            velocity=1.3,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    first_arm = object_model.get_part("first_arm")
    second_arm = object_model.get_part("second_arm")
    head_frame = object_model.get_part("head_frame")
    tilt_cradle = object_model.get_part("tilt_cradle")

    wall_to_first = object_model.get_articulation("wall_to_first_arm")
    first_to_second = object_model.get_articulation("first_to_second_arm")
    second_to_head = object_model.get_articulation("second_to_head")
    head_tilt = object_model.get_articulation("head_tilt")

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
        "swivel_axes_vertical",
        _axis_close(wall_to_first.axis, (0.0, 0.0, 1.0))
        and _axis_close(first_to_second.axis, (0.0, 0.0, 1.0))
        and _axis_close(second_to_head.axis, (0.0, 0.0, 1.0)),
        "all three reach joints should swivel about vertical z axes",
    )
    ctx.check(
        "head_tilt_axis_horizontal",
        _axis_close(head_tilt.axis, (0.0, 1.0, 0.0)),
        "tilt cradle should rotate about a horizontal y axis",
    )

    ctx.expect_contact(wall_plate, first_arm, name="wall_plate_contacts_first_arm")
    ctx.expect_contact(first_arm, second_arm, name="first_arm_contacts_second_arm")
    ctx.expect_contact(second_arm, head_frame, name="second_arm_contacts_head_frame")
    ctx.expect_contact(head_frame, tilt_cradle, name="head_frame_contacts_tilt_cradle")
    ctx.expect_within(
        tilt_cradle,
        head_frame,
        axes="yz",
        margin=0.0,
        name="tilt_cradle_within_head_frame_outline",
    )

    with ctx.pose(
        {
            wall_to_first: 0.0,
            first_to_second: 0.0,
            second_to_head: 0.0,
            head_tilt: 0.0,
        }
    ):
        ctx.expect_origin_gap(
            head_frame,
            wall_plate,
            axis="x",
            min_gap=0.62,
            name="extended_reach_from_wall_plate",
        )

    with ctx.pose({head_tilt: HEAD_TILT_LIMITS[0]}):
        ctx.expect_contact(
            head_frame,
            tilt_cradle,
            name="tilt_contact_at_lower_stop_pose",
        )

    with ctx.pose({head_tilt: HEAD_TILT_LIMITS[1]}):
        ctx.expect_contact(
            head_frame,
            tilt_cradle,
            name="tilt_contact_at_upper_stop_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
