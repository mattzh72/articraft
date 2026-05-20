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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

ANGLE_LIMIT = math.radians(18.0)
GIMBAL_Z = 0.018

CUP_RADIUS = 0.036
CUP_FLOOR = 0.003
BASE_WALL_X = 0.058
BASE_WALL_Y = 0.008
BASE_WALL_Z = 0.010
BASE_WALL_CENTER_Y = 0.028
BASE_WALL_CENTER_Z = 0.005
BASE_TOWER_SIZE = (0.008, 0.012, 0.018)
BASE_TOWER_CENTER_X = 0.032
BASE_TOWER_CENTER_Z = 0.009

OUTER_FRAME_THICK = 0.003
OUTER_RING_OUTER_Y = 0.026
OUTER_RING_OUTER_Z = 0.020
OUTER_RING_INNER_Y = 0.018
OUTER_RING_INNER_Z = 0.016
OUTER_ARM_LEN = 0.027
OUTER_ARM_THICK = 0.003
OUTER_TRUNNION_RADIUS = 0.0025
OUTER_TRUNNION_LENGTH = 0.008
OUTER_TRUNNION_CENTER_X = 0.032
OUTER_TAB_X = 0.008
OUTER_TAB_Y = 0.003
OUTER_TAB_Z = 0.005
OUTER_TAB_CENTER_Y = 0.0115

INNER_FRAME_THICK = 0.003
INNER_RING_OUTER_X = 0.012
INNER_RING_OUTER_Z = 0.012
INNER_RING_INNER_X = 0.006
INNER_RING_INNER_Z = 0.006
INNER_ARM_LEN = 0.008
INNER_ARM_THICK = 0.003
INNER_TRUNNION_RADIUS = 0.0025
INNER_TRUNNION_LENGTH = 0.003
INNER_TRUNNION_CENTER_Y = 0.010
LEVER_SEAT_RADIUS = 0.005
LEVER_SEAT_HEIGHT = 0.004
LEVER_SEAT_CENTER_Z = 0.010

LEVER_BASE_Z = LEVER_SEAT_CENTER_Z + LEVER_SEAT_HEIGHT / 2.0
LEVER_COLLAR_HEIGHT = 0.003
LEVER_COLLAR_RADIUS = 0.005
LEVER_BODY_HEIGHT = 0.013
LEVER_BASE_RADIUS = 0.0034
LEVER_TOP_RADIUS = 0.0027

CLAMP_WIDTH = 0.010
CLAMP_DEPTH = 0.007
CLAMP_HEIGHT = 0.006
CLAMP_SLOT_WIDTH = 0.0022


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _base_shape() -> cq.Workplane:
    floor = cq.Workplane("XY").circle(CUP_RADIUS).extrude(CUP_FLOOR)
    front_wall = _box_shape(
        (BASE_WALL_X, BASE_WALL_Y, BASE_WALL_Z),
        (0.0, BASE_WALL_CENTER_Y, BASE_WALL_CENTER_Z),
    )
    rear_wall = _box_shape(
        (BASE_WALL_X, BASE_WALL_Y, BASE_WALL_Z),
        (0.0, -BASE_WALL_CENTER_Y, BASE_WALL_CENTER_Z),
    )
    left_tower = _box_shape(
        BASE_TOWER_SIZE,
        (-BASE_TOWER_CENTER_X, 0.0, BASE_TOWER_CENTER_Z),
    )
    right_tower = _box_shape(
        BASE_TOWER_SIZE,
        (BASE_TOWER_CENTER_X, 0.0, BASE_TOWER_CENTER_Z),
    )
    return floor.union(front_wall).union(rear_wall).union(left_tower).union(right_tower)


def _outer_ring_shape() -> cq.Workplane:
    ring = _box_shape((OUTER_FRAME_THICK, OUTER_RING_OUTER_Y, OUTER_RING_OUTER_Z), (0.0, 0.0, 0.0))
    window = _box_shape((OUTER_FRAME_THICK + 0.001, OUTER_RING_INNER_Y, OUTER_RING_INNER_Z), (0.0, 0.0, 0.0))
    left_arm = _box_shape(
        (OUTER_ARM_LEN, OUTER_ARM_THICK, OUTER_ARM_THICK),
        (-OUTER_FRAME_THICK / 2.0 - OUTER_ARM_LEN / 2.0, 0.0, 0.0),
    )
    right_arm = _box_shape(
        (OUTER_ARM_LEN, OUTER_ARM_THICK, OUTER_ARM_THICK),
        (OUTER_FRAME_THICK / 2.0 + OUTER_ARM_LEN / 2.0, 0.0, 0.0),
    )
    front_tab = _box_shape(
        (OUTER_TAB_X, OUTER_TAB_Y, OUTER_TAB_Z),
        (0.0, OUTER_TAB_CENTER_Y, 0.0),
    )
    rear_tab = _box_shape(
        (OUTER_TAB_X, OUTER_TAB_Y, OUTER_TAB_Z),
        (0.0, -OUTER_TAB_CENTER_Y, 0.0),
    )
    left_trunnion = (
        cq.Workplane("YZ")
        .circle(OUTER_TRUNNION_RADIUS)
        .extrude(OUTER_TRUNNION_LENGTH)
        .translate((-OUTER_TRUNNION_CENTER_X - OUTER_TRUNNION_LENGTH / 2.0, 0.0, 0.0))
    )
    right_trunnion = (
        cq.Workplane("YZ")
        .circle(OUTER_TRUNNION_RADIUS)
        .extrude(OUTER_TRUNNION_LENGTH)
        .translate((OUTER_TRUNNION_CENTER_X - OUTER_TRUNNION_LENGTH / 2.0, 0.0, 0.0))
    )
    return ring.cut(window).union(left_arm).union(right_arm).union(front_tab).union(rear_tab).union(left_trunnion).union(right_trunnion)


def _inner_ring_shape() -> cq.Workplane:
    ring = _box_shape((INNER_RING_OUTER_X, INNER_FRAME_THICK, INNER_RING_OUTER_Z), (0.0, 0.0, 0.0))
    window = _box_shape((INNER_RING_INNER_X, INNER_FRAME_THICK + 0.001, INNER_RING_INNER_Z), (0.0, 0.0, 0.0))
    front_arm = _box_shape(
        (INNER_ARM_THICK, INNER_ARM_LEN, INNER_ARM_THICK),
        (0.0, INNER_FRAME_THICK / 2.0 + INNER_ARM_LEN / 2.0, 0.0),
    )
    rear_arm = _box_shape(
        (INNER_ARM_THICK, INNER_ARM_LEN, INNER_ARM_THICK),
        (0.0, -INNER_FRAME_THICK / 2.0 - INNER_ARM_LEN / 2.0, 0.0),
    )
    front_trunnion = (
        cq.Workplane("XZ")
        .circle(INNER_TRUNNION_RADIUS)
        .extrude(INNER_TRUNNION_LENGTH)
        .translate((0.0, INNER_TRUNNION_CENTER_Y - INNER_TRUNNION_LENGTH / 2.0, 0.0))
    )
    rear_trunnion = (
        cq.Workplane("XZ")
        .circle(INNER_TRUNNION_RADIUS)
        .extrude(INNER_TRUNNION_LENGTH)
        .translate((0.0, -INNER_TRUNNION_CENTER_Y - INNER_TRUNNION_LENGTH / 2.0, 0.0))
    )
    lever_seat = (
        cq.Workplane("XY")
        .circle(LEVER_SEAT_RADIUS)
        .extrude(LEVER_SEAT_HEIGHT)
        .translate((0.0, 0.0, LEVER_SEAT_CENTER_Z - LEVER_SEAT_HEIGHT / 2.0))
    )
    return ring.cut(window).union(front_arm).union(rear_arm).union(front_trunnion).union(rear_trunnion).union(lever_seat)


def _lever_body_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(LEVER_COLLAR_RADIUS).extrude(LEVER_COLLAR_HEIGHT)
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=LEVER_COLLAR_HEIGHT)
        .circle(LEVER_BASE_RADIUS)
        .workplane(offset=LEVER_BODY_HEIGHT - LEVER_COLLAR_HEIGHT)
        .circle(LEVER_TOP_RADIUS)
        .loft(combine=True)
    )
    return collar.union(shaft)


def _clamp_block_shape() -> cq.Workplane:
    block = cq.Workplane("XY").box(CLAMP_WIDTH, CLAMP_DEPTH, CLAMP_HEIGHT, centered=(True, True, False))
    slot = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, CLAMP_HEIGHT * 0.45))
        .box(CLAMP_SLOT_WIDTH, CLAMP_DEPTH + 0.001, CLAMP_HEIGHT * 0.7, centered=(True, True, False))
    )
    return block.cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="console_joystick", assets=ASSETS)

    dark_body = model.material("dark_body", rgba=(0.15, 0.16, 0.18, 1.0))
    anodized = model.material("anodized", rgba=(0.32, 0.34, 0.38, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    blackened = model.material("blackened", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base_cup")
    base.visual(mesh_from_cadquery(_base_shape(), "base_cup.obj", assets=ASSETS), name="cup_shell", material=dark_body)
    base.inertial = Inertial.from_geometry(Box((0.090, 0.072, 0.020)), mass=0.45, origin=Origin(xyz=(0.0, 0.0, 0.010)))

    outer = model.part("outer_ring")
    outer.visual(mesh_from_cadquery(_outer_ring_shape(), "outer_ring.obj", assets=ASSETS), name="outer_frame", material=anodized)
    outer.inertial = Inertial.from_geometry(Box((0.070, 0.030, 0.020)), mass=0.15, origin=Origin())

    inner = model.part("inner_ring")
    inner.visual(mesh_from_cadquery(_inner_ring_shape(), "inner_ring.obj", assets=ASSETS), name="inner_frame", material=steel)
    inner.inertial = Inertial.from_geometry(Box((0.020, 0.022, 0.018)), mass=0.10, origin=Origin(xyz=(0.0, 0.0, 0.004)))

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_lever_body_shape(), "lever_body.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, LEVER_BASE_Z)),
        name="lever_body",
        material=blackened,
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.012, 0.012, LEVER_BODY_HEIGHT + LEVER_COLLAR_HEIGHT)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, LEVER_BASE_Z + (LEVER_BODY_HEIGHT + LEVER_COLLAR_HEIGHT) / 2.0)),
    )

    clamp = model.part("clamp_block")
    clamp.visual(mesh_from_cadquery(_clamp_block_shape(), "clamp_block.obj", assets=ASSETS), name="clamp_shell", material=anodized)
    clamp.inertial = Inertial.from_geometry(Box((CLAMP_WIDTH, CLAMP_DEPTH, CLAMP_HEIGHT)), mass=0.03, origin=Origin(xyz=(0.0, 0.0, CLAMP_HEIGHT / 2.0)))

    model.articulation(
        "base_to_outer_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-ANGLE_LIMIT, upper=ANGLE_LIMIT),
    )
    model.articulation(
        "outer_to_inner_roll",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=inner,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-ANGLE_LIMIT, upper=ANGLE_LIMIT),
    )
    model.articulation("inner_to_lever", ArticulationType.FIXED, parent=inner, child=lever, origin=Origin())
    model.articulation(
        "lever_to_clamp",
        ArticulationType.FIXED,
        parent=lever,
        child=clamp,
        origin=Origin(xyz=(0.0, 0.0, LEVER_BASE_Z + LEVER_BODY_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_cup")
    outer = object_model.get_part("outer_ring")
    inner = object_model.get_part("inner_ring")
    lever = object_model.get_part("lever")
    clamp = object_model.get_part("clamp_block")

    outer_pitch = object_model.get_articulation("base_to_outer_pitch")
    inner_roll = object_model.get_articulation("outer_to_inner_roll")
    lever_body = lever.get_visual("lever_body")
    clamp_shell = clamp.get_visual("clamp_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        base,
        outer,
        reason="Low-profile pitch yoke is seated into the base side towers around the trunnion axis.",
    )
    ctx.allow_overlap(
        outer,
        inner,
        reason="Nested cardan rings intentionally interleave at the roll-axis bearing seats.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_parts_present",
        all(part is not None for part in (base, outer, inner, lever, clamp)),
        "Expected base cup, outer ring, inner ring, lever, and clamp block.",
    )
    ctx.check(
        "pitch_axis_and_limits",
        tuple(outer_pitch.axis) == (1.0, 0.0, 0.0)
        and abs(outer_pitch.motion_limits.lower + ANGLE_LIMIT) < 1e-9
        and abs(outer_pitch.motion_limits.upper - ANGLE_LIMIT) < 1e-9,
        "Outer ring should pitch about X by ±18 degrees.",
    )
    ctx.check(
        "roll_axis_and_limits",
        tuple(inner_roll.axis) == (0.0, 1.0, 0.0)
        and abs(inner_roll.motion_limits.lower + ANGLE_LIMIT) < 1e-9
        and abs(inner_roll.motion_limits.upper - ANGLE_LIMIT) < 1e-9,
        "Inner ring should roll about Y by ±18 degrees.",
    )

    ctx.expect_contact(base, outer, name="outer_ring_contacts_base_towers")
    ctx.expect_contact(outer, inner, name="inner_ring_contacts_outer_tabs")
    ctx.expect_contact(inner, lever, name="lever_mount_contacts_inner_ring")
    ctx.expect_contact(lever, clamp, elem_a=lever_body, elem_b=clamp_shell, name="clamp_contacts_lever")
    ctx.expect_within(outer, base, axes="xy", margin=0.004, name="outer_ring_within_console_footprint")
    ctx.expect_within(inner, outer, axes="xz", margin=0.010, name="inner_ring_within_outer_window")
    ctx.expect_origin_distance(lever, base, axes="xy", max_dist=0.001, name="lever_centered_over_cup")

    with ctx.pose({outer_pitch: math.radians(14.0)}):
        ctx.expect_gap(clamp, base, axis="z", min_gap=0.019, name="clamp_clears_cup_at_positive_pitch")
    with ctx.pose({outer_pitch: math.radians(-14.0), inner_roll: math.radians(12.0)}):
        ctx.expect_gap(clamp, base, axis="z", min_gap=0.017, name="clamp_clears_cup_in_combined_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
