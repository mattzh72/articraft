from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
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
    mesh_from_cadquery,
)


TRAY_SIZE = 0.160
TRAY_FLOOR = 0.006
RIM_HEIGHT = 0.012
RIM_THICKNESS = 0.012
MOUNT_HOLE_DIAMETER = 0.0065
MOUNT_HOLE_OFFSET = 0.056

PIVOT_Z = 0.052
TOWER_WIDTH = 0.038
TOWER_THICKNESS = 0.016
TOWER_HEIGHT = 0.066
TOWER_CENTER_Y = 0.052

SEAL_OUTER_RADIUS = 0.049
SEAL_INNER_RADIUS = 0.036
SEAL_HEIGHT = 0.018

OUTER_YOKE_WIDTH = 0.118
OUTER_YOKE_THICKNESS = 0.018
OUTER_YOKE_DEPTH = 0.050
OUTER_YOKE_BOTTOM_Z = -0.018
OUTER_YOKE_TOP_Z = 0.050
OUTER_YOKE_WINDOW_WIDTH = 0.074
OUTER_YOKE_WINDOW_TOP_Z = 0.034
OUTER_SHAFT_RADIUS = 0.0065
OUTER_BORE_RADIUS = 0.0095
OUTER_SHAFT_TOTAL = 2.0 * (TOWER_CENTER_Y + TOWER_THICKNESS / 2.0)
OUTER_COLLAR_RADIUS = 0.010
OUTER_COLLAR_THICKNESS = 0.006
OUTER_COLLAR_CENTER_Y = TOWER_CENTER_Y + TOWER_THICKNESS / 2.0 + OUTER_COLLAR_THICKNESS / 2.0

INNER_CRADLE_THICKNESS = 0.016
INNER_CRADLE_WIDTH = 0.070
INNER_CRADLE_BOTTOM_Z = -0.022
INNER_CRADLE_TOP_Z = 0.028
INNER_CRADLE_WINDOW_WIDTH = 0.040
INNER_CRADLE_WINDOW_BOTTOM_Z = -0.010
INNER_SHAFT_RADIUS = 0.0056
INNER_BORE_RADIUS = 0.0080
INNER_SHAFT_TOTAL = OUTER_YOKE_WIDTH
INNER_COLLAR_RADIUS = 0.0085
INNER_COLLAR_THICKNESS = 0.006
INNER_COLLAR_CENTER_X = OUTER_YOKE_WIDTH / 2.0 + INNER_COLLAR_THICKNESS / 2.0

SOCKET_RADIUS = 0.018
SOCKET_HEIGHT = 0.018
STICK_SLEEVE_RADIUS = 0.0145
STICK_SLEEVE_HEIGHT = 0.016
STICK_RADIUS = 0.012
STICK_HEIGHT = 0.102
STICK_TIP_RADIUS = 0.015
STICK_TIP_HEIGHT = 0.016


def _box(x: float, y: float, z: float, sx: float, sy: float, sz: float) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).translate((x, y, z))


def _tower(y_center: float) -> cq.Workplane:
    tower = _box(0.0, y_center, TOWER_HEIGHT / 2.0, TOWER_WIDTH, TOWER_THICKNESS, TOWER_HEIGHT)
    bore = (
        cq.Workplane("XZ")
        .circle(OUTER_BORE_RADIUS)
        .extrude(TOWER_THICKNESS / 2.0 + 0.003, both=True)
        .translate((0.0, y_center, PIVOT_Z))
    )
    return tower.cut(bore)


def make_mounting_tray_shell() -> cq.Workplane:
    floor = _box(0.0, 0.0, -TRAY_FLOOR / 2.0, TRAY_SIZE, TRAY_SIZE, TRAY_FLOOR)
    floor = (
        floor.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (MOUNT_HOLE_OFFSET, MOUNT_HOLE_OFFSET),
                (-MOUNT_HOLE_OFFSET, MOUNT_HOLE_OFFSET),
                (MOUNT_HOLE_OFFSET, -MOUNT_HOLE_OFFSET),
                (-MOUNT_HOLE_OFFSET, -MOUNT_HOLE_OFFSET),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
    )

    front_wall = _box(
        0.0,
        (TRAY_SIZE - RIM_THICKNESS) / 2.0,
        RIM_HEIGHT / 2.0,
        TRAY_SIZE,
        RIM_THICKNESS,
        RIM_HEIGHT,
    )
    back_wall = _box(
        0.0,
        -(TRAY_SIZE - RIM_THICKNESS) / 2.0,
        RIM_HEIGHT / 2.0,
        TRAY_SIZE,
        RIM_THICKNESS,
        RIM_HEIGHT,
    )
    left_wall = _box(
        -(TRAY_SIZE - RIM_THICKNESS) / 2.0,
        0.0,
        RIM_HEIGHT / 2.0,
        RIM_THICKNESS,
        TRAY_SIZE - 2.0 * RIM_THICKNESS,
        RIM_HEIGHT,
    )
    right_wall = _box(
        (TRAY_SIZE - RIM_THICKNESS) / 2.0,
        0.0,
        RIM_HEIGHT / 2.0,
        RIM_THICKNESS,
        TRAY_SIZE - 2.0 * RIM_THICKNESS,
        RIM_HEIGHT,
    )

    shell = floor
    for solid in (
        front_wall,
        back_wall,
        left_wall,
        right_wall,
        _tower(TOWER_CENTER_Y),
        _tower(-TOWER_CENTER_Y),
    ):
        shell = shell.union(solid)
    front_pin = (
        cq.Workplane("XZ")
        .circle(OUTER_SHAFT_RADIUS)
        .extrude((TOWER_THICKNESS - 0.002) / 2.0, both=True)
        .translate((0.0, TOWER_CENTER_Y, PIVOT_Z))
    )
    rear_pin = (
        cq.Workplane("XZ")
        .circle(OUTER_SHAFT_RADIUS)
        .extrude((TOWER_THICKNESS - 0.002) / 2.0, both=True)
        .translate((0.0, -TOWER_CENTER_Y, PIVOT_Z))
    )
    shell = shell.union(front_pin).union(rear_pin)
    return shell


def make_seal_collar() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(SEAL_OUTER_RADIUS)
        .circle(SEAL_INNER_RADIUS)
        .extrude(SEAL_HEIGHT)
    )


def make_outer_yoke() -> cq.Workplane:
    rail_width = 0.016
    rail_depth = 0.010
    rail_center_x = OUTER_YOKE_WIDTH / 2.0 - rail_width / 2.0
    rail_center_y = 0.018
    rail_bottom_z = -0.020
    rail_top_z = 0.040
    rail_height = rail_top_z - rail_bottom_z
    rail_center_z = (rail_top_z + rail_bottom_z) / 2.0

    bridge_width = OUTER_YOKE_WIDTH - rail_width
    bridge_depth = 0.008
    bridge_center_y = 0.026
    bridge_height = 0.010
    bridge_center_z = -0.018

    hub_width = 0.018
    hub_depth = 0.012
    hub_height = 0.022
    hub_center_y = 0.040
    hub_center_z = -0.008

    sleeve_outer_radius = 0.0086
    sleeve_length = TOWER_THICKNESS - 0.002
    x_pin_length = 0.014

    yoke = _box(-rail_center_x, rail_center_y, rail_center_z, rail_width, rail_depth, rail_height)
    for solid in (
        _box(-rail_center_x, -rail_center_y, rail_center_z, rail_width, rail_depth, rail_height),
        _box(rail_center_x, rail_center_y, rail_center_z, rail_width, rail_depth, rail_height),
        _box(rail_center_x, -rail_center_y, rail_center_z, rail_width, rail_depth, rail_height),
        _box(0.0, bridge_center_y, bridge_center_z, bridge_width, bridge_depth, bridge_height),
        _box(0.0, -bridge_center_y, bridge_center_z, bridge_width, bridge_depth, bridge_height),
        _box(0.0, hub_center_y, hub_center_z, hub_width, hub_depth, hub_height),
        _box(0.0, -hub_center_y, hub_center_z, hub_width, hub_depth, hub_height),
        cq.Workplane("XZ")
        .circle(sleeve_outer_radius)
        .circle(OUTER_SHAFT_RADIUS)
        .extrude(sleeve_length / 2.0, both=True)
        .translate((0.0, TOWER_CENTER_Y, 0.0)),
        cq.Workplane("XZ")
        .circle(sleeve_outer_radius)
        .circle(OUTER_SHAFT_RADIUS)
        .extrude(sleeve_length / 2.0, both=True)
        .translate((0.0, -TOWER_CENTER_Y, 0.0)),
        cq.Workplane("YZ")
        .circle(INNER_SHAFT_RADIUS)
        .extrude(x_pin_length / 2.0, both=True)
        .translate((-rail_center_x, 0.0, 0.0)),
        cq.Workplane("YZ")
        .circle(INNER_SHAFT_RADIUS)
        .extrude(x_pin_length / 2.0, both=True)
        .translate((rail_center_x, 0.0, 0.0)),
    ):
        yoke = yoke.union(solid)
    return yoke


def make_inner_cradle() -> cq.Workplane:
    sleeve_outer_radius = 0.0074
    sleeve_length = 0.014
    cheek_width = 0.014
    cheek_depth = 0.006
    cheek_center_y = 0.010
    cheek_bottom_z = -0.006
    cheek_top_z = 0.018
    cheek_height = cheek_top_z - cheek_bottom_z
    cheek_center_z = (cheek_top_z + cheek_bottom_z) / 2.0

    top_strap = _box(0.0, 0.0, 0.014, 0.014, 0.020, 0.010)
    lower_saddle = _box(0.0, 0.0, -0.013, 0.020, 0.018, 0.010)
    front_cheek = _box(0.0, cheek_center_y, cheek_center_z, cheek_width, cheek_depth, cheek_height)
    rear_cheek = _box(0.0, -cheek_center_y, cheek_center_z, cheek_width, cheek_depth, cheek_height)
    left_sleeve = (
        cq.Workplane("YZ")
        .circle(sleeve_outer_radius)
        .circle(INNER_SHAFT_RADIUS)
        .extrude(sleeve_length / 2.0, both=True)
        .translate((-OUTER_YOKE_WIDTH / 2.0 + 0.008, 0.0, 0.0))
    )
    right_sleeve = (
        cq.Workplane("YZ")
        .circle(sleeve_outer_radius)
        .circle(INNER_SHAFT_RADIUS)
        .extrude(sleeve_length / 2.0, both=True)
        .translate((OUTER_YOKE_WIDTH / 2.0 - 0.008, 0.0, 0.0))
    )
    socket = cq.Workplane("XY").circle(0.0165).extrude(0.016).translate((0.0, 0.0, -0.016))
    sleeve = cq.Workplane("XY").circle(STICK_SLEEVE_RADIUS).extrude(STICK_SLEEVE_HEIGHT)
    stick = cq.Workplane("XY").circle(STICK_RADIUS).extrude(STICK_HEIGHT)

    cradle = lower_saddle
    for solid in (top_strap, front_cheek, rear_cheek, left_sleeve, right_sleeve, socket, sleeve, stick):
        cradle = cradle.union(solid)
    return cradle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sealed_joystick_core")

    tray_paint = model.material("tray_paint", color=(0.18, 0.19, 0.21, 1.0))
    outer_yoke_finish = model.material("outer_yoke_finish", color=(0.33, 0.35, 0.37, 1.0))
    inner_cradle_finish = model.material("inner_cradle_finish", color=(0.56, 0.58, 0.60, 1.0))
    steel = model.material("steel", color=(0.72, 0.74, 0.77, 1.0))
    seal_black = model.material("seal_black", color=(0.09, 0.09, 0.10, 1.0))

    tray = model.part("mounting_tray")
    tray.visual(
        mesh_from_cadquery(make_mounting_tray_shell(), "mounting_tray_shell"),
        material=tray_paint,
        name="tray_shell",
    )
    tray.visual(
        mesh_from_cadquery(make_seal_collar(), "seal_collar"),
        material=seal_black,
        name="seal_collar",
    )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        mesh_from_cadquery(make_outer_yoke(), "outer_yoke_body"),
        material=outer_yoke_finish,
        name="outer_yoke_body",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(make_inner_cradle(), "inner_cradle_body"),
        material=inner_cradle_finish,
        name="inner_cradle_body",
    )
    inner_cradle.visual(
        Cylinder(radius=STICK_TIP_RADIUS, length=STICK_TIP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, STICK_HEIGHT + STICK_TIP_HEIGHT / 2.0)),
        material=steel,
        name="stick_tip",
    )

    model.articulation(
        "tray_to_outer_yoke",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.52, upper=0.52),
    )
    model.articulation(
        "outer_yoke_to_inner_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_cradle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-0.48, upper=0.48),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    tray = object_model.get_part("mounting_tray")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_cradle = object_model.get_part("inner_cradle")

    ctx.allow_overlap(
        tray,
        outer_yoke,
        elem_a="tray_shell",
        elem_b="outer_yoke_body",
        reason=(
            "The outer yoke's bearing sleeves and trunnion run through the tray's tower bores; "
            "the authored visuals simplify this retained bearing stack into nested solids."
        ),
    )
    ctx.allow_overlap(
        outer_yoke,
        inner_cradle,
        elem_a="outer_yoke_body",
        elem_b="inner_cradle_body",
        reason=(
            "The inner cradle pivots inside the outer yoke on an orthogonal journal set; "
            "the visible meshes intentionally simplify the bearing pockets around the cross shafts."
        ),
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0015)
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

    outer_joint = object_model.get_articulation("tray_to_outer_yoke")
    inner_joint = object_model.get_articulation("outer_yoke_to_inner_cradle")

    ctx.check(
        "outer_yoke_axis_is_crosswise",
        tuple(outer_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected outer yoke axis (0, 1, 0), got {outer_joint.axis}.",
    )
    ctx.check(
        "inner_cradle_axis_is_orthogonal",
        tuple(inner_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected inner cradle axis (1, 0, 0), got {inner_joint.axis}.",
    )
    ctx.check(
        "gimbal_limits_are_bidirectional",
        (
            outer_joint.motion_limits is not None
            and inner_joint.motion_limits is not None
            and outer_joint.motion_limits.lower is not None
            and outer_joint.motion_limits.upper is not None
            and inner_joint.motion_limits.lower is not None
            and inner_joint.motion_limits.upper is not None
            and outer_joint.motion_limits.lower < 0.0 < outer_joint.motion_limits.upper
            and inner_joint.motion_limits.lower < 0.0 < inner_joint.motion_limits.upper
        ),
        "Both gimbal axes should swing to either side of center.",
    )

    ctx.expect_contact(
        outer_yoke,
        tray,
        contact_tol=0.0015,
        name="outer_yoke_is_retained_by_mounting_tray",
    )
    ctx.expect_contact(
        inner_cradle,
        outer_yoke,
        contact_tol=0.0015,
        name="inner_cradle_is_retained_by_outer_yoke",
    )
    ctx.expect_overlap(
        outer_yoke,
        tray,
        axes="xy",
        min_overlap=0.050,
        name="outer_yoke_sits_over_tray_center",
    )
    ctx.expect_overlap(
        inner_cradle,
        outer_yoke,
        axes="xy",
        min_overlap=0.028,
        name="inner_cradle_nests_inside_outer_yoke",
    )
    ctx.expect_gap(
        inner_cradle,
        tray,
        axis="z",
        min_gap=0.003,
        negative_elem="seal_collar",
        name="neutral_cradle_clears_seal_collar",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    with ctx.pose({outer_joint: 0.0, inner_joint: 0.0}):
        neutral_tip = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="stick_tip"))
    with ctx.pose({outer_joint: 0.35, inner_joint: 0.0}):
        outer_tip = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="stick_tip"))
    with ctx.pose({outer_joint: 0.0, inner_joint: 0.35}):
        inner_tip = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="stick_tip"))
    with ctx.pose({outer_joint: 0.35, inner_joint: 0.30}):
        ctx.expect_gap(
            inner_cradle,
            tray,
            axis="z",
            min_gap=0.001,
            negative_elem="seal_collar",
            name="tilted_cradle_stays_above_seal_collar",
        )

    tip_centers_ready = neutral_tip is not None and outer_tip is not None and inner_tip is not None
    ctx.check(
        "stick_tip_probe_visual_is_available",
        tip_centers_ready,
        "Expected named visual 'stick_tip' to have a measurable world AABB.",
    )
    if tip_centers_ready:
        ctx.check(
            "outer_yoke_motion_moves_tip_primarily_in_x",
            outer_tip[0] > neutral_tip[0] + 0.020 and abs(outer_tip[1] - neutral_tip[1]) < 0.010,
            (
                f"Neutral tip center {neutral_tip}, outer-yoke pose tip center {outer_tip}; "
                "expected strong +X movement with little Y drift."
            ),
        )
        ctx.check(
            "inner_cradle_motion_moves_tip_primarily_in_y",
            inner_tip[1] < neutral_tip[1] - 0.020 and abs(inner_tip[0] - neutral_tip[0]) < 0.010,
            (
                f"Neutral tip center {neutral_tip}, inner-cradle pose tip center {inner_tip}; "
                "expected strong -Y movement with little X drift."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
