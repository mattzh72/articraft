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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_OUTER_RADIUS = 0.072
BASE_INNER_RADIUS = 0.060
BASE_HEIGHT = 0.060
BASE_FLOOR_THICKNESS = 0.008
GIMBAL_CENTER_Z = 0.040

BASE_PIVOT_BLOCK_X = 0.057
BASE_PIVOT_BLOCK_SIZE = (0.010, 0.026, 0.042)

OUTER_RING_OUTER_RADIUS = 0.046
OUTER_RING_INNER_RADIUS = 0.037
OUTER_RING_THICKNESS = 0.006
OUTER_TRUNNION_RADIUS = 0.0045
OUTER_TRUNNION_LENGTH = 0.007
OUTER_TRUNNION_CENTER_X = 0.0485
OUTER_YOKE_SIZE = (0.020, 0.009, 0.010)
OUTER_YOKE_CENTER_Y = 0.0405

INNER_RING_OUTER_RADIUS = 0.031
INNER_RING_INNER_RADIUS = 0.024
INNER_RING_THICKNESS = 0.006
INNER_TRUNNION_RADIUS = 0.0038
INNER_TRUNNION_LENGTH = 0.005
INNER_TRUNNION_CENTER_Y = 0.0335
INNER_HUB_RADIUS = 0.0105
INNER_HUB_LENGTH = 0.004

STICK_COLLAR_RADIUS = 0.009
STICK_COLLAR_LENGTH = 0.018
STICK_SHAFT_RADIUS = 0.0055
STICK_SHAFT_LENGTH = 0.095
STICK_GRIP_RADIUS = 0.012
STICK_GRIP_LENGTH = 0.026
STICK_CAP_RADIUS = 0.012

OUTER_RING_LIMIT = 0.45
INNER_RING_LIMIT = 0.45


def _cup_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").circle(BASE_OUTER_RADIUS).extrude(BASE_HEIGHT)
    shell = (
        shell.faces(">Z")
        .workplane()
        .circle(BASE_INNER_RADIUS)
        .cutBlind(-(BASE_HEIGHT - BASE_FLOOR_THICKNESS))
    )
    foot = cq.Workplane("XY").circle(BASE_OUTER_RADIUS + 0.006).extrude(0.004)
    return foot.union(shell)


def _outer_ring_frame_shape() -> cq.Workplane:
    outer_disk = cq.Workplane("XY").circle(OUTER_RING_OUTER_RADIUS).extrude(
        OUTER_RING_THICKNESS / 2.0,
        both=True,
    )
    inner_void = cq.Workplane("XY").circle(OUTER_RING_INNER_RADIUS).extrude(
        OUTER_RING_THICKNESS + 0.004,
        both=True,
    )
    ring = outer_disk.cut(inner_void)
    right_lug = cq.Workplane("XY").box(0.016, 0.020, 0.010).translate((0.039, 0.0, 0.0))
    left_lug = cq.Workplane("XY").box(0.016, 0.020, 0.010).translate((-0.039, 0.0, 0.0))
    return ring.union(right_lug).union(left_lug)


def _inner_ring_frame_shape() -> cq.Workplane:
    outer_disk = cq.Workplane("YZ").circle(INNER_RING_OUTER_RADIUS).extrude(
        INNER_RING_THICKNESS / 2.0,
        both=True,
    )
    inner_void = cq.Workplane("YZ").circle(INNER_RING_INNER_RADIUS).extrude(
        INNER_RING_THICKNESS + 0.004,
        both=True,
    )
    stick_clearance_slot = (
        cq.Workplane("YZ")
        .rect(0.022, 0.042)
        .extrude(INNER_RING_THICKNESS + 0.004, both=True)
        .translate((0.0, 0.0, 0.019))
    )
    ring = outer_disk.cut(inner_void).cut(stick_clearance_slot)

    front_lower_brace = (
        cq.Workplane("YZ")
        .moveTo(0.010, -0.003)
        .lineTo(0.018, -0.003)
        .lineTo(0.027, -0.018)
        .lineTo(0.012, -0.018)
        .close()
        .extrude(INNER_RING_THICKNESS / 2.0, both=True)
    )
    rear_lower_brace = (
        cq.Workplane("YZ")
        .moveTo(-0.010, -0.003)
        .lineTo(-0.018, -0.003)
        .lineTo(-0.027, -0.018)
        .lineTo(-0.012, -0.018)
        .close()
        .extrude(INNER_RING_THICKNESS / 2.0, both=True)
    )
    lower_bridge = cq.Workplane("XY").box(INNER_RING_THICKNESS, 0.026, 0.006).translate(
        (0.0, 0.0, -0.018)
    )
    front_lug = cq.Workplane("XY").box(0.010, 0.010, 0.010).translate((0.0, 0.026, 0.0))
    rear_lug = cq.Workplane("XY").box(0.010, 0.010, 0.010).translate((0.0, -0.026, 0.0))
    return (
        ring.union(front_lower_brace)
        .union(rear_lower_brace)
        .union(lower_bridge)
        .union(front_lug)
        .union(rear_lug)
    )


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cardan_ring_joystick")

    model.material("base_polymer", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("dark_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("satin_aluminum", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("gunmetal", rgba=(0.42, 0.45, 0.50, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_cup_shell_shape(), "cup_shell"),
        material="base_polymer",
        name="cup_shell",
    )
    base.visual(
        Box(BASE_PIVOT_BLOCK_SIZE),
        origin=Origin(
            xyz=(
                BASE_PIVOT_BLOCK_X,
                0.0,
                BASE_FLOOR_THICKNESS + BASE_PIVOT_BLOCK_SIZE[2] / 2.0,
            )
        ),
        material="gunmetal",
        name="right_pivot_block",
    )
    base.visual(
        Box(BASE_PIVOT_BLOCK_SIZE),
        origin=Origin(
            xyz=(
                -BASE_PIVOT_BLOCK_X,
                0.0,
                BASE_FLOOR_THICKNESS + BASE_PIVOT_BLOCK_SIZE[2] / 2.0,
            )
        ),
        material="gunmetal",
        name="left_pivot_block",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_OUTER_RADIUS + 0.006, length=BASE_HEIGHT),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(_outer_ring_frame_shape(), "outer_ring_frame"),
        material="satin_aluminum",
        name="outer_ring_frame",
    )
    outer_ring.visual(
        Cylinder(radius=OUTER_TRUNNION_RADIUS, length=OUTER_TRUNNION_LENGTH),
        origin=Origin(xyz=(OUTER_TRUNNION_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="gunmetal",
        name="right_trunnion",
    )
    outer_ring.visual(
        Cylinder(radius=OUTER_TRUNNION_RADIUS, length=OUTER_TRUNNION_LENGTH),
        origin=Origin(xyz=(-OUTER_TRUNNION_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="gunmetal",
        name="left_trunnion",
    )
    outer_ring.visual(
        Box(OUTER_YOKE_SIZE),
        origin=Origin(xyz=(0.0, OUTER_YOKE_CENTER_Y, 0.0)),
        material="gunmetal",
        name="front_yoke",
    )
    outer_ring.visual(
        Box(OUTER_YOKE_SIZE),
        origin=Origin(xyz=(0.0, -OUTER_YOKE_CENTER_Y, 0.0)),
        material="gunmetal",
        name="rear_yoke",
    )
    outer_ring.inertial = Inertial.from_geometry(
        Box((0.104, 0.094, 0.012)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        mesh_from_cadquery(_inner_ring_frame_shape(), "inner_ring_frame"),
        material="satin_aluminum",
        name="inner_ring_frame",
    )
    inner_ring.visual(
        Cylinder(radius=INNER_HUB_RADIUS, length=INNER_HUB_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -INNER_HUB_LENGTH / 2.0)),
        material="gunmetal",
        name="hub_mount",
    )
    inner_ring.visual(
        Cylinder(radius=INNER_TRUNNION_RADIUS, length=INNER_TRUNNION_LENGTH),
        origin=Origin(xyz=(0.0, INNER_TRUNNION_CENTER_Y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="gunmetal",
        name="front_trunnion",
    )
    inner_ring.visual(
        Cylinder(radius=INNER_TRUNNION_RADIUS, length=INNER_TRUNNION_LENGTH),
        origin=Origin(xyz=(0.0, -INNER_TRUNNION_CENTER_Y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="gunmetal",
        name="rear_trunnion",
    )
    inner_ring.inertial = Inertial.from_geometry(
        Box((0.062, 0.072, 0.012)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    stick = model.part("stick")
    stick.visual(
        Cylinder(radius=STICK_COLLAR_RADIUS, length=STICK_COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, STICK_COLLAR_LENGTH / 2.0)),
        material="gunmetal",
        name="stick_collar",
    )
    stick.visual(
        Cylinder(radius=STICK_SHAFT_RADIUS, length=STICK_SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, STICK_COLLAR_LENGTH + STICK_SHAFT_LENGTH / 2.0)),
        material="base_polymer",
        name="stick_shaft",
    )
    stick.visual(
        Cylinder(radius=STICK_GRIP_RADIUS, length=STICK_GRIP_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.102 + STICK_GRIP_LENGTH / 2.0)),
        material="dark_rubber",
        name="stick_grip",
    )
    stick.visual(
        Cylinder(radius=STICK_CAP_RADIUS, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material="dark_rubber",
        name="stick_cap",
    )
    stick.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.138),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
    )

    model.articulation(
        "base_to_outer_ring",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-OUTER_RING_LIMIT,
            upper=OUTER_RING_LIMIT,
            effort=8.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "outer_to_inner_ring",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-INNER_RING_LIMIT,
            upper=INNER_RING_LIMIT,
            effort=8.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "inner_ring_to_stick",
        ArticulationType.FIXED,
        parent=inner_ring,
        child=stick,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    outer_ring = object_model.get_part("outer_ring")
    inner_ring = object_model.get_part("inner_ring")
    stick = object_model.get_part("stick")

    outer_joint = object_model.get_articulation("base_to_outer_ring")
    inner_joint = object_model.get_articulation("outer_to_inner_ring")

    ctx.check("base part exists", base is not None)
    ctx.check("outer ring part exists", outer_ring is not None)
    ctx.check("inner ring part exists", inner_ring is not None)
    ctx.check("stick part exists", stick is not None)

    ctx.expect_origin_distance(
        outer_ring,
        inner_ring,
        axes="xyz",
        max_dist=1e-6,
        name="outer and inner ring joint axes intersect at one center",
    )
    ctx.expect_origin_distance(
        inner_ring,
        stick,
        axes="xyz",
        max_dist=1e-6,
        name="stick root stays centered on the inner ring",
    )
    ctx.expect_contact(
        base,
        outer_ring,
        elem_a="right_pivot_block",
        elem_b="right_trunnion",
        name="outer ring right trunnion seats on the grounded pivot block",
    )
    ctx.expect_contact(
        outer_ring,
        inner_ring,
        elem_a="front_yoke",
        elem_b="front_trunnion",
        name="inner ring front trunnion seats on the outer yoke",
    )
    ctx.expect_contact(
        stick,
        inner_ring,
        elem_a="stick_collar",
        elem_b="hub_mount",
        name="stick collar mounts directly to the inner ring hub",
    )

    neutral_grip_center = _aabb_center(ctx.part_element_world_aabb(stick, elem="stick_grip"))
    with ctx.pose({outer_joint: 0.35}):
        outer_tilt_grip_center = _aabb_center(ctx.part_element_world_aabb(stick, elem="stick_grip"))
    with ctx.pose({inner_joint: 0.35}):
        inner_tilt_grip_center = _aabb_center(ctx.part_element_world_aabb(stick, elem="stick_grip"))

    ctx.check(
        "outer ring revolute joint tilts the stick about the first axis",
        neutral_grip_center is not None
        and outer_tilt_grip_center is not None
        and outer_tilt_grip_center[1] < neutral_grip_center[1] - 0.015,
        details=f"neutral={neutral_grip_center}, outer_tilt={outer_tilt_grip_center}",
    )
    ctx.check(
        "inner ring revolute joint tilts the stick about the orthogonal axis",
        neutral_grip_center is not None
        and inner_tilt_grip_center is not None
        and inner_tilt_grip_center[0] > neutral_grip_center[0] + 0.015,
        details=f"neutral={neutral_grip_center}, inner_tilt={inner_tilt_grip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
