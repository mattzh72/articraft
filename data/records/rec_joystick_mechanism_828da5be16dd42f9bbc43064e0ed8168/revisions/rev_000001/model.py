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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.40
BASE_DEPTH = 0.32
BASE_HEIGHT = 0.12

DECK_LENGTH = 0.30
DECK_DEPTH = 0.22
DECK_HEIGHT = 0.022

PEDESTAL_LENGTH = 0.22
PEDESTAL_DEPTH = 0.18
PEDESTAL_HEIGHT = 0.18

PIVOT_Z = 0.43
TOWER_WIDTH = 0.045
TOWER_DEPTH = 0.18
TOWER_HEIGHT = 0.32
TOWER_Z0 = 0.21
TOWER_X = 0.1125

YOKE_WIDTH = 0.164
YOKE_DEPTH = 0.16
YOKE_HEIGHT = 0.20
YOKE_SIDE_THICKNESS = 0.022
YOKE_BOTTOM_THICKNESS = 0.024

FRAME_WIDTH = 0.10
FRAME_DEPTH = 0.12
FRAME_HEIGHT = 0.11
FRAME_Z_SHIFT = 0.0

LEVER_GRIP_RADIUS = 0.019
LEVER_GRIP_LENGTH = 0.09
LEVER_GRIP_CENTER_Z = 0.375

PITCH_LIMIT = 0.42
ROLL_LIMIT = 0.38


def raised_box(
    length: float,
    depth: float,
    height: float,
    z0: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(length, depth, height).translate((x, y, z0 + height * 0.5))


def make_base_structure() -> cq.Workplane:
    housing = raised_box(BASE_LENGTH, BASE_DEPTH, BASE_HEIGHT, 0.0).edges("|Z").fillet(0.012)
    deck = raised_box(DECK_LENGTH, DECK_DEPTH, DECK_HEIGHT, BASE_HEIGHT - 0.004).edges("|Z").fillet(0.008)
    pedestal = raised_box(
        PEDESTAL_LENGTH,
        PEDESTAL_DEPTH,
        PEDESTAL_HEIGHT,
        BASE_HEIGHT,
    ).edges("|Z").fillet(0.010)
    front_bridge = raised_box(0.285, 0.034, 0.082, 0.188, y=0.073).edges("|Z").fillet(0.004)
    rear_bridge = raised_box(0.285, 0.034, 0.082, 0.188, y=-0.073).edges("|Z").fillet(0.004)
    left_tower = raised_box(
        TOWER_WIDTH,
        TOWER_DEPTH,
        TOWER_HEIGHT,
        TOWER_Z0,
        x=-TOWER_X,
    ).edges("|Z").fillet(0.006)
    right_tower = raised_box(
        TOWER_WIDTH,
        TOWER_DEPTH,
        TOWER_HEIGHT,
        TOWER_Z0,
        x=TOWER_X,
    ).edges("|Z").fillet(0.006)
    return (
        housing.union(deck)
        .union(pedestal)
        .union(front_bridge)
        .union(rear_bridge)
        .union(left_tower)
        .union(right_tower)
    )


def make_outer_yoke() -> cq.Workplane:
    arm_x = YOKE_WIDTH * 0.5 - YOKE_SIDE_THICKNESS * 0.5
    left_arm = cq.Workplane("XY").box(YOKE_SIDE_THICKNESS, YOKE_DEPTH, YOKE_HEIGHT).translate((-arm_x, 0.0, 0.0))
    right_arm = cq.Workplane("XY").box(YOKE_SIDE_THICKNESS, YOKE_DEPTH, YOKE_HEIGHT).translate((arm_x, 0.0, 0.0))
    bottom_bridge = cq.Workplane("XY").box(YOKE_WIDTH - 2.0 * YOKE_SIDE_THICKNESS, 0.052, YOKE_BOTTOM_THICKNESS).translate(
        (0.0, 0.0, -0.088)
    )
    left_pitch_boss = (
        cq.Workplane("YZ")
        .circle(0.027)
        .extrude(0.014)
        .translate((-arm_x - 0.007, 0.0, 0.0))
    )
    right_pitch_boss = (
        cq.Workplane("YZ")
        .circle(0.027)
        .extrude(0.014)
        .translate((arm_x - 0.007, 0.0, 0.0))
    )
    front_bearing = (
        cq.Workplane("XZ")
        .circle(0.018)
        .extrude(0.014)
        .translate((0.0, 0.077, 0.0))
    )
    rear_bearing = (
        cq.Workplane("XZ")
        .circle(0.018)
        .extrude(0.014)
        .translate((0.0, -0.091, 0.0))
    )

    def tie_bar(x: float, y: float, z: float) -> cq.Workplane:
        return cq.Workplane("XY").box(0.046, 0.010, 0.010).translate((x, y, z))

    return (
        left_arm.union(right_arm)
        .union(bottom_bridge)
        .union(left_pitch_boss)
        .union(right_pitch_boss)
        .union(front_bearing)
        .union(rear_bearing)
        .union(tie_bar(-0.029, 0.074, 0.028))
        .union(tie_bar(0.029, 0.074, 0.028))
        .union(tie_bar(-0.029, 0.074, -0.028))
        .union(tie_bar(0.029, 0.074, -0.028))
        .union(tie_bar(-0.029, -0.074, 0.028))
        .union(tie_bar(0.029, -0.074, 0.028))
        .union(tie_bar(-0.029, -0.074, -0.028))
        .union(tie_bar(0.029, -0.074, -0.028))
        .edges("|Z")
        .fillet(0.005)
    )


def make_inner_frame() -> cq.Workplane:
    left_post = cq.Workplane("XY").box(0.014, 0.056, 0.094).translate((-0.026, 0.0, 0.0))
    right_post = cq.Workplane("XY").box(0.014, 0.056, 0.094).translate((0.026, 0.0, 0.0))
    top_bridge = cq.Workplane("XY").box(0.070, 0.020, 0.012).translate((0.0, 0.0, 0.041))
    bottom_bridge = cq.Workplane("XY").box(0.070, 0.020, 0.012).translate((0.0, 0.0, -0.041))
    front_knuckle = cq.Workplane("XY").box(0.046, 0.020, 0.066).translate((0.0, 0.041, 0.0))
    rear_knuckle = cq.Workplane("XY").box(0.046, 0.020, 0.066).translate((0.0, -0.041, 0.0))
    front_trunnion = (
        cq.Workplane("XZ")
        .circle(0.014)
        .extrude(0.018)
        .translate((0.0, 0.052, 0.0))
    )
    rear_trunnion = (
        cq.Workplane("XZ")
        .circle(0.014)
        .extrude(0.018)
        .translate((0.0, -0.070, 0.0))
    )
    hub = cq.Workplane("XY").circle(0.026).extrude(0.028).translate((0.0, 0.0, -0.014))
    return (
        left_post.union(right_post)
        .union(top_bridge)
        .union(bottom_bridge)
        .union(front_knuckle)
        .union(rear_knuckle)
        .union(front_trunnion)
        .union(rear_trunnion)
        .union(hub)
        .edges("|Z")
        .fillet(0.0035)
    )


def make_lever_shaft() -> cq.Workplane:
    shaft = (
        cq.Workplane("XY")
        .circle(0.022)
        .workplane(offset=0.20)
        .circle(0.015)
        .workplane(offset=0.13)
        .circle(0.0105)
        .loft(combine=True)
    )
    collar = cq.Workplane("XY").circle(0.028).extrude(0.012)
    return collar.union(shaft)


def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_joystick")

    housing_gray = model.material("housing_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    coated_black = model.material("coated_black", rgba=(0.10, 0.10, 0.11, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.36, 0.38, 0.41, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_DEPTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT * 0.5)),
        material=housing_gray,
        name="base_structure",
    )
    base.visual(
        Box((DECK_LENGTH, DECK_DEPTH, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.009)),
        material=housing_gray,
        name="top_deck",
    )
    base.visual(
        Box((PEDESTAL_LENGTH, PEDESTAL_DEPTH, 0.162)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.081)),
        material=housing_gray,
        name="pedestal_core",
    )
    base.visual(
        Box((0.285, 0.034, 0.10)),
        origin=Origin(xyz=(0.0, 0.073, 0.250)),
        material=housing_gray,
        name="front_gusset",
    )
    base.visual(
        Box((0.285, 0.034, 0.10)),
        origin=Origin(xyz=(0.0, -0.073, 0.250)),
        material=housing_gray,
        name="rear_gusset",
    )
    base.visual(
        Box((0.035, 0.15, 0.23)),
        origin=Origin(xyz=(-0.1175, 0.0, 0.415)),
        material=housing_gray,
        name="left_tower",
    )
    base.visual(
        Box((0.035, 0.15, 0.23)),
        origin=Origin(xyz=(0.1175, 0.0, 0.415)),
        material=housing_gray,
        name="right_tower",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(-0.093, 0.0, PIVOT_Z), rpy=(0.0, pi * 0.5, 0.0)),
        material=steel,
        name="left_pitch_lug",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.093, 0.0, PIVOT_Z), rpy=(0.0, pi * 0.5, 0.0)),
        material=steel,
        name="right_pitch_lug",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_DEPTH, 0.53)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
    )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        Box((0.016, 0.14, 0.20)),
        origin=Origin(xyz=(-0.074, 0.0, 0.0)),
        material=coated_black,
        name="left_yoke_arm",
    )
    outer_yoke.visual(
        Box((0.016, 0.14, 0.20)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=coated_black,
        name="right_yoke_arm",
    )
    outer_yoke.visual(
        Box((0.132, 0.052, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        material=coated_black,
        name="outer_yoke_structure",
    )
    outer_yoke.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(-0.082, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=coated_black,
        name="left_pitch_pad",
    )
    outer_yoke.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=coated_black,
        name="right_pitch_pad",
    )
    outer_yoke.visual(
        Box((0.052, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, 0.079, 0.0)),
        material=coated_black,
        name="front_roll_lug",
    )
    outer_yoke.visual(
        Box((0.052, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, -0.079, 0.0)),
        material=coated_black,
        name="rear_roll_lug",
    )
    outer_yoke.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(-0.046, 0.068, 0.031)),
        material=coated_black,
        name="front_left_upper_brace",
    )
    outer_yoke.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(0.046, 0.068, 0.031)),
        material=coated_black,
        name="front_right_upper_brace",
    )
    outer_yoke.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(-0.046, 0.068, -0.031)),
        material=coated_black,
        name="front_left_lower_brace",
    )
    outer_yoke.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(0.046, 0.068, -0.031)),
        material=coated_black,
        name="front_right_lower_brace",
    )
    outer_yoke.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(-0.046, -0.068, 0.031)),
        material=coated_black,
        name="rear_left_upper_brace",
    )
    outer_yoke.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(0.046, -0.068, 0.031)),
        material=coated_black,
        name="rear_right_upper_brace",
    )
    outer_yoke.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(-0.046, -0.068, -0.031)),
        material=coated_black,
        name="rear_left_lower_brace",
    )
    outer_yoke.visual(
        Box((0.040, 0.012, 0.018)),
        origin=Origin(xyz=(0.046, -0.068, -0.031)),
        material=coated_black,
        name="rear_right_lower_brace",
    )
    outer_yoke.inertial = Inertial.from_geometry(
        Box((YOKE_WIDTH, YOKE_DEPTH, YOKE_HEIGHT)),
        mass=3.2,
        origin=Origin(),
    )

    inner_frame = model.part("inner_frame")
    inner_frame.visual(
        Box((0.016, 0.052, 0.094)),
        origin=Origin(xyz=(-0.038, 0.0, 0.0)),
        material=frame_gray,
        name="left_inner_post",
    )
    inner_frame.visual(
        Box((0.016, 0.052, 0.094)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=frame_gray,
        name="right_inner_post",
    )
    inner_frame.visual(
        Box((0.092, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=frame_gray,
        name="top_inner_bridge",
    )
    inner_frame.visual(
        Box((0.092, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=frame_gray,
        name="bottom_inner_bridge",
    )
    inner_frame.visual(
        Box((0.016, 0.060, 0.014)),
        origin=Origin(xyz=(-0.038, 0.055, 0.0)),
        material=frame_gray,
        name="front_left_link",
    )
    inner_frame.visual(
        Box((0.016, 0.060, 0.014)),
        origin=Origin(xyz=(0.038, 0.055, 0.0)),
        material=frame_gray,
        name="front_right_link",
    )
    inner_frame.visual(
        Box((0.016, 0.060, 0.014)),
        origin=Origin(xyz=(-0.038, -0.055, 0.0)),
        material=frame_gray,
        name="rear_left_link",
    )
    inner_frame.visual(
        Box((0.016, 0.060, 0.014)),
        origin=Origin(xyz=(0.038, -0.055, 0.0)),
        material=frame_gray,
        name="rear_right_link",
    )
    inner_frame.visual(
        Box((0.068, 0.010, 0.044)),
        origin=Origin(xyz=(0.0, 0.089, 0.0)),
        material=frame_gray,
        name="front_roll_pad",
    )
    inner_frame.visual(
        Box((0.068, 0.010, 0.044)),
        origin=Origin(xyz=(0.0, -0.089, 0.0)),
        material=frame_gray,
        name="rear_roll_pad",
    )
    inner_frame.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_gray,
        name="center_hub",
    )
    inner_frame.visual(
        mesh_from_cadquery(make_lever_shaft(), "control_lever_shaft"),
        material=steel,
        name="lever_shaft",
    )
    inner_frame.visual(
        Cylinder(radius=LEVER_GRIP_RADIUS, length=LEVER_GRIP_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, LEVER_GRIP_CENTER_Z)),
        material=grip_black,
        name="grip",
    )
    inner_frame.visual(
        Sphere(radius=LEVER_GRIP_RADIUS),
        origin=Origin(xyz=(0.0, 0.0, LEVER_GRIP_CENTER_Z + LEVER_GRIP_LENGTH * 0.5)),
        material=grip_black,
        name="grip_cap",
    )
    inner_frame.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.44)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    model.articulation(
        "base_to_outer_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-PITCH_LIMIT,
            upper=PITCH_LIMIT,
        ),
    )
    model.articulation(
        "outer_yoke_to_inner_frame",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_frame,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.8,
            lower=-ROLL_LIMIT,
            upper=ROLL_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_frame = object_model.get_part("inner_frame")
    pitch = object_model.get_articulation("base_to_outer_yoke")
    roll = object_model.get_articulation("outer_yoke_to_inner_frame")

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
        "parts_present",
        all(part is not None for part in (base, outer_yoke, inner_frame)),
        "expected base, outer_yoke, and inner_frame parts",
    )
    ctx.check(
        "pitch_axis_is_transverse",
        tuple(pitch.axis) == (1.0, 0.0, 0.0),
        f"expected pitch axis (1, 0, 0), got {pitch.axis}",
    )
    ctx.check(
        "roll_axis_is_perpendicular",
        tuple(roll.axis) == (0.0, 1.0, 0.0),
        f"expected roll axis (0, 1, 0), got {roll.axis}",
    )

    ctx.expect_contact(
        base,
        outer_yoke,
        elem_a="left_pitch_lug",
        elem_b="left_pitch_pad",
        name="left_pitch_support_contact",
    )
    ctx.expect_contact(
        base,
        outer_yoke,
        elem_a="right_pitch_lug",
        elem_b="right_pitch_pad",
        name="right_pitch_support_contact",
    )
    ctx.expect_contact(
        outer_yoke,
        inner_frame,
        elem_a="front_roll_lug",
        elem_b="front_roll_pad",
        name="front_roll_support_contact",
    )
    ctx.expect_contact(
        outer_yoke,
        inner_frame,
        elem_a="rear_roll_lug",
        elem_b="rear_roll_pad",
        name="rear_roll_support_contact",
    )

    grip_aabb = ctx.part_element_world_aabb(inner_frame, elem="grip")
    grip_center_neutral = aabb_center(grip_aabb)
    grip_top_z = grip_aabb[1][2] if grip_aabb is not None else None
    ctx.check(
        "overall_height_reads_as_tall_pedestal",
        grip_top_z is not None and 0.78 <= grip_top_z <= 0.92,
        f"expected grip top in [0.78, 0.92] m, got {grip_top_z}",
    )

    with ctx.pose({pitch: 0.35}):
        pitched_grip = aabb_center(ctx.part_element_world_aabb(inner_frame, elem="grip"))
    with ctx.pose({roll: 0.30}):
        rolled_grip = aabb_center(ctx.part_element_world_aabb(inner_frame, elem="grip"))

    pitch_motion_ok = (
        grip_center_neutral is not None
        and pitched_grip is not None
        and abs(pitched_grip[0] - grip_center_neutral[0]) < 0.01
        and abs(pitched_grip[1] - grip_center_neutral[1]) > 0.08
        and abs(pitched_grip[2] - grip_center_neutral[2]) > 0.01
    )
    ctx.check(
        "pitch_motion_moves_grip_in_yz_plane",
        pitch_motion_ok,
        f"neutral grip={grip_center_neutral}, pitched grip={pitched_grip}",
    )

    roll_motion_ok = (
        grip_center_neutral is not None
        and rolled_grip is not None
        and abs(rolled_grip[0] - grip_center_neutral[0]) > 0.08
        and abs(rolled_grip[1] - grip_center_neutral[1]) < 0.01
        and abs(rolled_grip[2] - grip_center_neutral[2]) > 0.01
    )
    ctx.check(
        "roll_motion_moves_grip_in_xz_plane",
        roll_motion_ok,
        f"neutral grip={grip_center_neutral}, rolled grip={rolled_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
