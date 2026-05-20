from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FLANGE_DIAMETER = 0.170
FLANGE_THICKNESS = 0.0035
CUP_OUTER_DIAMETER = 0.132
CUP_INNER_DIAMETER = 0.118
CUP_DEPTH = 0.018
FLOOR_THICKNESS = 0.005
CONTROL_CENTER_Z = 0.036

OUTER_RING_OUTER_RADIUS = 0.030
OUTER_RING_INNER_RADIUS = 0.023
OUTER_RING_WIDTH = 0.010
OUTER_RING_HUB_RADIUS = 0.0085
OUTER_RING_HUB_LENGTH = 0.024
OUTER_JOURNAL_RADIUS = 0.0056
OUTER_JOURNAL_LENGTH = 0.018
OUTER_RETAINER_RADIUS = 0.009
OUTER_RETAINER_LENGTH = 0.004
OUTER_JOURNAL_BORE_RADIUS = 0.0061
OUTER_BOSS_RADIUS = 0.0095
OUTER_BOSS_LENGTH = 0.012
OUTER_BOSS_CENTER_Y = 0.039
OUTER_INNER_COLLAR_CENTER_X = 0.028
OUTER_OUTER_COLLAR_CENTER_X = 0.046

CRADLE_FRAME_WIDTH = 0.026
CRADLE_FRAME_HEIGHT = 0.020
CRADLE_SIDE_PLATE_WIDTH = 0.006
CRADLE_SIDE_PLATE_DEPTH = 0.010
CRADLE_SIDE_PLATE_HEIGHT = 0.020
CRADLE_RAIL_WIDTH = 0.004
CRADLE_RAIL_HEIGHT = 0.006
CRADLE_THICKNESS = 0.010
INNER_TRUNNION_RADIUS = 0.0048
INNER_TRUNNION_BORE_RADIUS = 0.0053
INNER_TRUNNION_LENGTH = 0.012
INNER_TRUNNION_COLLAR_RADIUS = 0.008
INNER_TRUNNION_COLLAR_LENGTH = 0.004
SOCKET_OUTER_RADIUS = 0.011
SOCKET_HEIGHT = 0.012
STICK_BORE_RADIUS = 0.0078
SOCKET_CENTER_Z = 0.006
STICK_JOINT_OFFSET_Z = 0.012

STICK_RADIUS = 0.0065
STICK_INSERT_DEPTH = 0.0
STICK_EXPOSED_LENGTH = 0.040
STICK_SEAT_RADIUS = 0.010
STICK_SEAT_HEIGHT = 0.004
COLLAR_RADIUS = 0.014
COLLAR_HEIGHT = 0.010
COLLAR_BASE_Z = 0.044

OUTER_LIMIT = 0.42
INNER_LIMIT = 0.42


def _centered_z_cylinder(radius: float, length: float):
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -(length / 2.0)))


def _x_cylinder(radius: float, length: float, *, center_x: float, y: float = 0.0, z: float = 0.0):
    return (
        _centered_z_cylinder(radius, length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((center_x, y, z))
    )


def _y_cylinder(radius: float, length: float, *, center_y: float, x: float = 0.0, z: float = 0.0):
    return (
        _centered_z_cylinder(radius, length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((x, center_y, z))
    )


def _z_cylinder(radius: float, length: float, *, center_z: float, x: float = 0.0, y: float = 0.0):
    return _centered_z_cylinder(radius, length).translate((x, y, center_z))


def _make_base_shape():
    cup_body = cq.Workplane("XY").circle(CUP_OUTER_DIAMETER / 2.0).extrude(-CUP_DEPTH)
    cavity = (
        cq.Workplane("XY")
        .circle(CUP_INNER_DIAMETER / 2.0)
        .extrude(-(CUP_DEPTH - FLOOR_THICKNESS))
    )
    flange = (
        cq.Workplane("XY")
        .circle(FLANGE_DIAMETER / 2.0)
        .circle(CUP_INNER_DIAMETER / 2.0)
        .extrude(FLANGE_THICKNESS)
    )
    base = cup_body.cut(cavity).union(flange)

    yoke_center_x = 0.046
    foot_width = 0.020
    foot_depth = 0.016
    foot_height = 0.010
    upright_width = 0.016
    upright_depth = 0.014
    upright_height = 0.042
    cap_height = 0.010
    floor_z = -CUP_DEPTH

    for sign in (-1.0, 1.0):
        foot = (
            cq.Workplane("XY")
            .box(foot_width, foot_depth, foot_height, centered=(True, True, False))
            .translate((sign * yoke_center_x, 0.0, floor_z))
        )
        upright = (
            cq.Workplane("XY")
            .box(upright_width, upright_depth, upright_height, centered=(True, True, False))
            .translate((sign * yoke_center_x, 0.0, floor_z + 0.005))
        )
        cap = (
            cq.Workplane("XY")
            .box(0.018, 0.018, cap_height, centered=(True, True, True))
            .translate((sign * yoke_center_x, 0.0, CONTROL_CENTER_Z))
        )
        saddle = foot.union(upright).union(cap)
        bore_length = OUTER_JOURNAL_LENGTH
        bore = _x_cylinder(
            OUTER_JOURNAL_BORE_RADIUS,
            bore_length,
            center_x=sign * yoke_center_x,
            z=CONTROL_CENTER_Z,
        )
        base = base.union(saddle.cut(bore))

    return base


def _make_outer_ring_shape():
    lower_bridge = (
        cq.Workplane("XY").box(0.056, 0.052, 0.006).edges("|Z").fillet(0.0015).translate((0.0, 0.0, -0.019))
    )
    front_rib = (
        cq.Workplane("XY").box(0.052, 0.010, 0.020).edges("|Z").fillet(0.0015).translate((0.0, 0.026, -0.009))
    )
    rear_rib = (
        cq.Workplane("XY").box(0.052, 0.010, 0.020).edges("|Z").fillet(0.0015).translate((0.0, -0.026, -0.009))
    )
    left_web = (
        cq.Workplane("XY").box(0.010, 0.060, 0.012).edges("|Z").fillet(0.0012).translate((-0.021, 0.0, -0.013))
    )
    right_web = (
        cq.Workplane("XY").box(0.010, 0.060, 0.012).edges("|Z").fillet(0.0012).translate((0.021, 0.0, -0.013))
    )

    left_hub = _x_cylinder(OUTER_RING_HUB_RADIUS, OUTER_RING_HUB_LENGTH, center_x=-0.027)
    right_hub = _x_cylinder(OUTER_RING_HUB_RADIUS, OUTER_RING_HUB_LENGTH, center_x=0.027)
    left_journal = _x_cylinder(
        OUTER_JOURNAL_RADIUS,
        OUTER_JOURNAL_LENGTH,
        center_x=-0.046,
    )
    right_journal = _x_cylinder(
        OUTER_JOURNAL_RADIUS,
        OUTER_JOURNAL_LENGTH,
        center_x=0.046,
    )
    left_inner_collar = _x_cylinder(
        OUTER_RETAINER_RADIUS,
        OUTER_RETAINER_LENGTH,
        center_x=-0.034,
    )
    right_inner_collar = _x_cylinder(
        OUTER_RETAINER_RADIUS,
        OUTER_RETAINER_LENGTH,
        center_x=0.034,
    )
    left_retainer = _x_cylinder(
        OUTER_RETAINER_RADIUS,
        OUTER_RETAINER_LENGTH,
        center_x=-0.054,
    )
    right_retainer = _x_cylinder(
        OUTER_RETAINER_RADIUS,
        OUTER_RETAINER_LENGTH,
        center_x=0.054,
    )
    ring = (
        lower_bridge.union(front_rib)
        .union(rear_rib)
        .union(left_web)
        .union(right_web)
        .union(left_hub)
        .union(right_hub)
        .union(left_journal)
        .union(right_journal)
        .union(left_inner_collar)
        .union(right_inner_collar)
        .union(left_retainer)
        .union(right_retainer)
    )

    front_boss = _y_cylinder(
        OUTER_BOSS_RADIUS,
        OUTER_BOSS_LENGTH,
        center_y=OUTER_BOSS_CENTER_Y,
    )
    rear_boss = _y_cylinder(
        OUTER_BOSS_RADIUS,
        OUTER_BOSS_LENGTH,
        center_y=-OUTER_BOSS_CENTER_Y,
    )
    ring = ring.union(front_boss).union(rear_boss)

    bore_length = OUTER_BOSS_LENGTH + 0.001
    front_bore = _y_cylinder(
        INNER_TRUNNION_BORE_RADIUS,
        bore_length,
        center_y=OUTER_BOSS_CENTER_Y,
    )
    rear_bore = _y_cylinder(
        INNER_TRUNNION_BORE_RADIUS,
        bore_length,
        center_y=-OUTER_BOSS_CENTER_Y,
    )
    return ring.cut(front_bore).cut(rear_bore)


def _make_inner_cradle_shape():
    front_trunnion = _y_cylinder(INNER_TRUNNION_RADIUS, INNER_TRUNNION_LENGTH, center_y=OUTER_BOSS_CENTER_Y)
    rear_trunnion = _y_cylinder(INNER_TRUNNION_RADIUS, INNER_TRUNNION_LENGTH, center_y=-OUTER_BOSS_CENTER_Y)
    front_collar = _y_cylinder(
        INNER_TRUNNION_COLLAR_RADIUS,
        INNER_TRUNNION_COLLAR_LENGTH,
        center_y=OUTER_BOSS_CENTER_Y + 0.008,
    )
    rear_collar = _y_cylinder(
        INNER_TRUNNION_COLLAR_RADIUS,
        INNER_TRUNNION_COLLAR_LENGTH,
        center_y=-(OUTER_BOSS_CENTER_Y + 0.008),
    )
    front_arm = cq.Workplane("XY").box(0.018, 0.020, 0.006).translate((0.0, 0.025, -0.004))
    rear_arm = cq.Workplane("XY").box(0.018, 0.020, 0.006).translate((0.0, -0.025, -0.004))
    left_cheek = cq.Workplane("XY").box(0.006, 0.032, 0.016).translate((-0.010, 0.0, 0.001))
    right_cheek = cq.Workplane("XY").box(0.006, 0.032, 0.016).translate((0.010, 0.0, 0.001))
    lower_bridge = cq.Workplane("XY").box(0.020, 0.010, 0.005).translate((0.0, 0.0, -0.008))
    socket = _z_cylinder(SOCKET_OUTER_RADIUS, SOCKET_HEIGHT, center_z=SOCKET_CENTER_Z)
    socket_bore = _z_cylinder(STICK_BORE_RADIUS, SOCKET_HEIGHT + 0.002, center_z=SOCKET_CENTER_Z)
    cradle = (
        front_trunnion.union(rear_trunnion)
        .union(front_collar)
        .union(rear_collar)
        .union(front_arm)
        .union(rear_arm)
        .union(left_cheek)
        .union(right_cheek)
        .union(lower_bridge)
        .union(socket)
    )
    return cradle.cut(socket_bore)


def _make_stick_shape():
    shaft = _z_cylinder(
        STICK_RADIUS,
        STICK_EXPOSED_LENGTH,
        center_z=STICK_SEAT_HEIGHT + (STICK_EXPOSED_LENGTH / 2.0),
    )
    seat = _z_cylinder(
        STICK_SEAT_RADIUS,
        STICK_SEAT_HEIGHT,
        center_z=STICK_SEAT_HEIGHT / 2.0,
    )
    collar = _z_cylinder(
        COLLAR_RADIUS,
        COLLAR_HEIGHT,
        center_z=0.030 + (COLLAR_HEIGHT / 2.0),
    )
    stick = shaft.union(seat).union(collar)

    collar_center_z = 0.030 + (COLLAR_HEIGHT / 2.0)
    for angle in range(0, 360, 20):
        flat = (
            cq.Workplane("XY")
            .box(0.0032, COLLAR_RADIUS * 2.4, COLLAR_HEIGHT * 1.15, centered=(True, True, True))
            .translate((COLLAR_RADIUS * 0.92, 0.0, collar_center_z))
            .rotate((0.0, 0.0, collar_center_z), (0.0, 0.0, collar_center_z + 1.0), angle)
        )
        stick = stick.cut(flat)

    return stick


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="console_joystick_module")

    cast_gray = model.material("cast_gray", color=(0.30, 0.32, 0.34, 1.0))
    machined_gray = model.material("machined_gray", color=(0.46, 0.47, 0.49, 1.0))
    black_oxide = model.material("black_oxide", color=(0.13, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "joystick_base"),
        material=cast_gray,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((FLANGE_DIAMETER, FLANGE_DIAMETER, CUP_DEPTH + FLANGE_THICKNESS)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(_make_outer_ring_shape(), "outer_gimbal_ring"),
        material=machined_gray,
        name="ring_shell",
    )
    outer_ring.inertial = Inertial.from_geometry(
        Box((0.070, 0.100, 0.090)),
        mass=0.34,
        origin=Origin(),
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(_make_inner_cradle_shape(), "inner_trunnion_cradle"),
        material=cast_gray,
        name="cradle_shell",
    )
    inner_cradle.inertial = Inertial.from_geometry(
        Box((0.056, 0.080, 0.042)),
        mass=0.22,
        origin=Origin(),
    )

    stick = model.part("stick")
    stick.visual(
        mesh_from_cadquery(_make_stick_shape(), "joystick_stick"),
        material=black_oxide,
        name="stick_shell",
    )
    stick.inertial = Inertial.from_geometry(
        Cylinder(radius=COLLAR_RADIUS, length=STICK_INSERT_DEPTH + STICK_EXPOSED_LENGTH + COLLAR_HEIGHT),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    model.articulation(
        "base_to_outer_ring",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, CONTROL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-OUTER_LIMIT,
            upper=OUTER_LIMIT,
        ),
    )
    model.articulation(
        "outer_ring_to_inner_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-INNER_LIMIT,
            upper=INNER_LIMIT,
        ),
    )
    model.articulation(
        "inner_cradle_to_stick",
        ArticulationType.FIXED,
        parent=inner_cradle,
        child=stick,
        origin=Origin(xyz=(0.0, 0.0, STICK_JOINT_OFFSET_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_ring = object_model.get_part("outer_ring")
    inner_cradle = object_model.get_part("inner_cradle")
    stick = object_model.get_part("stick")
    outer_joint = object_model.get_articulation("base_to_outer_ring")
    inner_joint = object_model.get_articulation("outer_ring_to_inner_cradle")
    stick_joint = object_model.get_articulation("inner_cradle_to_stick")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        base,
        outer_ring,
        reason="outer ring journals are modeled as fitted pivots captured inside the base yoke bores",
    )
    ctx.allow_overlap(
        outer_ring,
        inner_cradle,
        reason="inner cradle trunnions are modeled as fitted pivots nested inside the outer ring bearing bosses",
    )
    ctx.allow_overlap(
        inner_cradle,
        stick,
        reason="the stick base is intentionally seated into the cradle socket as a fitted assembly",
    )

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
        "outer gimbal axis is left-right",
        tuple(outer_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {outer_joint.axis}",
    )
    ctx.check(
        "inner cradle axis is front-back",
        tuple(inner_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {inner_joint.axis}",
    )
    ctx.check(
        "stick is rigidly mounted to cradle",
        stick_joint.articulation_type == ArticulationType.FIXED,
        details=f"expected FIXED articulation, got {stick_joint.articulation_type}",
    )
    ctx.check(
        "gimbal limits stay centered about neutral",
        outer_joint.motion_limits is not None
        and inner_joint.motion_limits is not None
        and outer_joint.motion_limits.lower == -OUTER_LIMIT
        and outer_joint.motion_limits.upper == OUTER_LIMIT
        and inner_joint.motion_limits.lower == -INNER_LIMIT
        and inner_joint.motion_limits.upper == INNER_LIMIT,
        details="revolute limits should remain symmetric around the centered control position",
    )

    ctx.expect_contact(base, outer_ring, name="outer ring is carried by base pivots")
    ctx.expect_contact(outer_ring, inner_cradle, name="inner cradle is carried by outer ring trunnions")
    ctx.expect_contact(inner_cradle, stick, name="stick is seated into the cradle socket")
    ctx.expect_gap(
        stick,
        base,
        axis="z",
        min_gap=0.002,
        name="stick clears the panel cup above the base",
    )

    with ctx.pose({outer_joint: OUTER_LIMIT * 0.95, inner_joint: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap at outer pitch limit")
    with ctx.pose({outer_joint: 0.0, inner_joint: INNER_LIMIT * 0.95}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap at inner roll limit")
    with ctx.pose({outer_joint: -OUTER_LIMIT * 0.85, inner_joint: INNER_LIMIT * 0.85}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap in compound gimbal pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
