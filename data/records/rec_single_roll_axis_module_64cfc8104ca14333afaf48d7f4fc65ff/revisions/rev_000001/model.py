from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


PLATE_T = 0.012
PLATE_W = 0.180
PLATE_H = 0.240

REAR_BLOCK_X = 0.022
REAR_BLOCK_L = 0.024
REAR_BLOCK_W = 0.094
REAR_BLOCK_H = 0.082

FRONT_BLOCK_X = 0.064
FRONT_BLOCK_L = 0.020
FRONT_BLOCK_W = 0.082
FRONT_BLOCK_H = 0.064

RIB_T = 0.024
RIB_CENTER_Y = 0.032
BLOCK_CENTER_Y = 0.032

BORE_R = 0.012
SHAFT_R = 0.011
COLLAR_R = 0.020
COLLAR_X = 0.004
COLLAR_L = 0.008

SHAFT_CORE_L = 0.066
HUB_X = SHAFT_CORE_L
HUB_L = 0.014
HUB_R = 0.019
FLANGE_X = 0.074
FLANGE_T = 0.012
FLANGE_R = 0.045
PILOT_X = FLANGE_X + FLANGE_T
PILOT_L = 0.008
PILOT_R = 0.024

PIN_R = 0.004
PIN_L = 0.014
PIN_Y = -0.014
PIN_Z = 0.030


def _box_at(x0: float, dx: float, dy: float, dz: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(dx, dy, dz, centered=(False, True, True))
        .translate((x0, 0.0, 0.0))
    )


def _make_support_shape() -> cq.Workplane:
    backplate = _box_at(0.0, PLATE_T, PLATE_W, PLATE_H).edges("|X").fillet(0.003)

    rail_length = FRONT_BLOCK_X + FRONT_BLOCK_L - REAR_BLOCK_X
    side_rail_blank = (
        _box_at(REAR_BLOCK_X, rail_length, 0.024, 0.050)
        .edges("|X")
        .fillet(0.003)
    )
    left_rail = side_rail_blank.translate((0.0, BLOCK_CENTER_Y, 0.0))
    right_rail = side_rail_blank.translate((0.0, -BLOCK_CENTER_Y, 0.0))

    front_cap_blank = (
        _box_at(FRONT_BLOCK_X, FRONT_BLOCK_L, 0.030, 0.058)
        .edges("|X")
        .fillet(0.003)
    )
    left_front_cap = front_cap_blank.translate((0.0, BLOCK_CENTER_Y, 0.0))
    right_front_cap = front_cap_blank.translate((0.0, -BLOCK_CENTER_Y, 0.0))

    rib_profile = [
        (PLATE_T, -0.082),
        (PLATE_T, 0.082),
        (0.026, 0.072),
        (0.052, 0.044),
        (FRONT_BLOCK_X, 0.026),
        (FRONT_BLOCK_X, -0.026),
        (0.052, -0.044),
        (0.026, -0.072),
    ]
    rib_blank = (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(RIB_T)
    )
    rib_left = rib_blank.translate((0.0, RIB_CENTER_Y - RIB_T / 2.0, 0.0))
    rib_right = rib_blank.translate((0.0, -RIB_CENTER_Y - RIB_T / 2.0, 0.0))

    support = (
        backplate.union(left_rail)
        .union(right_rail)
        .union(left_front_cap)
        .union(right_front_cap)
        .union(rib_left)
        .union(rib_right)
    )

    slot_points = [
        (-0.055, 0.075),
        (0.055, 0.075),
        (-0.055, -0.075),
        (0.055, -0.075),
    ]
    mount_slots = (
        cq.Workplane("YZ")
        .pushPoints(slot_points)
        .slot2D(0.028, 0.010, angle=90)
        .extrude(PLATE_T + 0.003)
    )
    return support.cut(mount_slots)


def _make_rib_shape(y_center: float) -> cq.Workplane:
    rib_profile = [
        (PLATE_T, -0.082),
        (PLATE_T, 0.082),
        (0.026, 0.072),
        (0.052, 0.044),
        (FRONT_BLOCK_X, 0.026),
        (FRONT_BLOCK_X, -0.026),
        (0.052, -0.044),
        (0.026, -0.072),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(rib_profile)
        .close()
        .extrude(RIB_T)
        .translate((0.0, y_center - RIB_T / 2.0, 0.0))
    )

def _make_flange_shape() -> cq.Workplane:
    hub = cq.Workplane("YZ").circle(HUB_R).extrude(HUB_L).translate((HUB_X, 0.0, 0.0))
    flange = (
        cq.Workplane("YZ")
        .circle(FLANGE_R)
        .extrude(FLANGE_T)
        .translate((FLANGE_X, 0.0, 0.0))
    )
    pilot = (
        cq.Workplane("YZ")
        .circle(PILOT_R)
        .extrude(PILOT_L)
        .translate((PILOT_X, 0.0, 0.0))
    )
    stage_face = hub.union(flange).union(pilot)

    bolt_circle_r = 0.028
    bolt_angles = (0.35, 2.38, 4.57)
    hole_points = [
        (bolt_circle_r * cos(angle), bolt_circle_r * sin(angle)) for angle in bolt_angles
    ]
    flange_holes = (
        cq.Workplane("YZ")
        .pushPoints(hole_points)
        .circle(0.0045)
        .extrude(FLANGE_T + 0.010)
        .translate((FLANGE_X - 0.004, 0.0, 0.0))
    )

    return stage_face.cut(flange_holes)


def _make_locator_pin_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(PIN_R)
        .extrude(PIN_L + 0.002)
        .translate((FLANGE_X + FLANGE_T - 0.002, PIN_Y, PIN_Z))
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_roll_spindle_module")

    support_color = model.material("support_color", color=(0.18, 0.19, 0.21, 1.0))
    machined_color = model.material("machined_color", color=(0.74, 0.76, 0.79, 1.0))
    indicator_color = model.material("indicator_color", color=(0.77, 0.16, 0.10, 1.0))

    support = model.part("support")
    rail_length = FRONT_BLOCK_X + FRONT_BLOCK_L - PLATE_T
    support.visual(
        Box((PLATE_T, PLATE_W, PLATE_H)),
        origin=Origin(xyz=(PLATE_T * 0.5, 0.0, 0.0)),
        material=support_color,
        name="backplate",
    )
    support.visual(
        Box((rail_length, 0.024, 0.050)),
        origin=Origin(xyz=(PLATE_T + rail_length * 0.5, BLOCK_CENTER_Y, 0.0)),
        material=support_color,
        name="left_rail",
    )
    support.visual(
        Box((rail_length, 0.024, 0.050)),
        origin=Origin(xyz=(PLATE_T + rail_length * 0.5, -BLOCK_CENTER_Y, 0.0)),
        material=support_color,
        name="right_rail",
    )
    support.visual(
        Box((FRONT_BLOCK_L, 0.030, 0.058)),
        origin=Origin(xyz=(FRONT_BLOCK_X + FRONT_BLOCK_L * 0.5, BLOCK_CENTER_Y, 0.0)),
        material=support_color,
        name="left_front_cap",
    )
    support.visual(
        Box((FRONT_BLOCK_L, 0.030, 0.058)),
        origin=Origin(xyz=(FRONT_BLOCK_X + FRONT_BLOCK_L * 0.5, -BLOCK_CENTER_Y, 0.0)),
        material=support_color,
        name="right_front_cap",
    )
    support.visual(
        Box((0.060, 0.024, 0.160)),
        origin=Origin(xyz=(0.042, 0.044, 0.0)),
        material=support_color,
        name="left_rib",
    )
    support.visual(
        Box((0.060, 0.024, 0.160)),
        origin=Origin(xyz=(0.042, -0.044, 0.0)),
        material=support_color,
        name="right_rib",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_R, length=SHAFT_CORE_L),
        origin=Origin(xyz=(SHAFT_CORE_L * 0.5, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_color,
        name="shaft_journal",
    )
    spindle.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_L),
        origin=Origin(xyz=(COLLAR_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined_color,
        name="thrust_collar",
    )
    spindle.visual(
        mesh_from_cadquery(_make_flange_shape(), "carry_flange"),
        material=machined_color,
        name="carry_flange",
    )
    spindle.visual(
        mesh_from_cadquery(_make_locator_pin_shape(), "locator_pin"),
        material=indicator_color,
        name="locator_pin",
    )

    model.articulation(
        "support_to_spindle",
        ArticulationType.REVOLUTE,
        parent=support,
        child=spindle,
        origin=Origin(xyz=(REAR_BLOCK_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=4.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    spindle = object_model.get_part("spindle")
    roll = object_model.get_articulation("support_to_spindle")

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

    limits = roll.motion_limits
    axis_ok = tuple(roll.axis) == (1.0, 0.0, 0.0)
    limit_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
    )
    ctx.check(
        "roll articulation configured on spindle axis",
        axis_ok and limit_ok,
        f"axis={roll.axis}, limits={limits}",
    )

    ctx.expect_origin_distance(
        spindle,
        support,
        axes="yz",
        max_dist=0.001,
        name="spindle axis stays centered on fixed support",
    )
    ctx.expect_contact(
        spindle,
        support,
        elem_a="thrust_collar",
        contact_tol=1e-4,
        name="thrust collar bears on the fixed support",
    )
    ctx.expect_gap(
        spindle,
        support,
        axis="x",
        min_gap=0.003,
        max_gap=0.006,
        positive_elem="carry_flange",
        name="rotary flange stays proud of front bearing block",
    )
    with ctx.pose({roll: 0.0}):
        pin_center_0 = _aabb_center(ctx.part_element_world_aabb(spindle, elem="locator_pin"))
    with ctx.pose({roll: pi / 2.0}):
        pin_center_90 = _aabb_center(ctx.part_element_world_aabb(spindle, elem="locator_pin"))

    pose_ok = False
    if pin_center_0 is not None and pin_center_90 is not None:
        pose_ok = (
            abs(pin_center_90[0] - pin_center_0[0]) < 0.003
            and abs(pin_center_90[1] + pin_center_0[2]) < 0.003
            and abs(pin_center_90[2] - pin_center_0[1]) < 0.003
        )
    ctx.check(
        "locator pin rolls around positive spindle x-axis",
        pose_ok,
        f"q0={pin_center_0}, q90={pin_center_90}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
