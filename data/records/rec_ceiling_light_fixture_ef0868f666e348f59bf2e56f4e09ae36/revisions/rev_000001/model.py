from __future__ import annotations

import math

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


CANOPY_RADIUS = 0.11
CANOPY_THICKNESS = 0.024
CANOPY_PIVOT_RADIUS = 0.118
CANOPY_PIVOT_Z = -0.020
ARM_DROP = math.radians(32.0)
ARM_TUBE_RADIUS = 0.008
ARM_TUBE_START = 0.020
ARM_TUBE_LENGTH = 0.136
ARM_PIVOT_REACH = 0.172
ARM_BARREL_RADIUS = 0.006
HOLDER_BARREL_RADIUS = 0.005


def _planar_rotate(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        x * cos_yaw - y * sin_yaw,
        x * sin_yaw + y * cos_yaw,
    )


def _yaw_offset(local_xyz: tuple[float, float, float], yaw: float) -> tuple[float, float, float]:
    x, y, z = local_xyz
    rx, ry = _planar_rotate(x, y, yaw)
    return (rx, ry, z)


def _arm_dir() -> tuple[float, float, float]:
    return (math.cos(ARM_DROP), 0.0, -math.sin(ARM_DROP))


def _scale_vec(vec: tuple[float, float, float], scale: float) -> tuple[float, float, float]:
    return (vec[0] * scale, vec[1] * scale, vec[2] * scale)


def _add_vec(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        0.5 * (low[0] + high[0]),
        0.5 * (low[1] + high[1]),
        0.5 * (low[2] + high[2]),
    )


def _build_globe_mesh(name: str):
    outer_radius = 0.073
    wall = 0.004
    globe_center_z = -0.112
    opening_z = -0.040
    cutter = cq.Workplane("XY").box(0.24, 0.24, 0.24).translate((0.0, 0.0, opening_z + 0.12))
    outer = cq.Workplane("XY").sphere(outer_radius).translate((0.0, 0.0, globe_center_z)).cut(cutter)
    inner = (
        cq.Workplane("XY")
        .sphere(outer_radius - wall)
        .translate((0.0, 0.0, globe_center_z))
        .cut(cutter)
    )
    globe = outer.cut(inner)
    return mesh_from_cadquery(globe, name)


def _add_canopy_hinge(canopy, angle: float, index: int) -> None:
    pivot = (
        CANOPY_PIVOT_RADIUS * math.cos(angle),
        CANOPY_PIVOT_RADIUS * math.sin(angle),
        CANOPY_PIVOT_Z,
    )

    boss_center = _add_vec(pivot, _yaw_offset((-0.020, 0.0, 0.0), angle))
    canopy.visual(
        Box((0.027, 0.018, 0.012)),
        origin=Origin(xyz=boss_center, rpy=(0.0, 0.0, angle)),
        material="brass",
        name=f"arm_{index}_boss",
    )

    for side, side_sign in (("a", -1.0), ("b", 1.0)):
        barrel_center = _add_vec(pivot, _yaw_offset((0.0, side_sign * 0.011, 0.0), angle))
        canopy.visual(
            Cylinder(radius=0.0075, length=0.010),
            origin=Origin(xyz=barrel_center, rpy=(-math.pi / 2.0, 0.0, angle)),
            material="brass",
            name=f"arm_{index}_hinge_{side}",
        )


def _add_arm_geometry(arm, index: int) -> None:
    arm.visual(
        Cylinder(radius=ARM_BARREL_RADIUS, length=0.012),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brass",
        name="root_barrel",
    )

    direction = _arm_dir()
    arm.visual(
        Box((0.026, 0.010, 0.010)),
        origin=Origin(xyz=_scale_vec(direction, 0.013), rpy=(0.0, ARM_DROP, 0.0)),
        material="brass",
        name="root_bridge",
    )
    tube_center = _scale_vec(direction, ARM_TUBE_START + 0.5 * ARM_TUBE_LENGTH)
    arm.visual(
        Cylinder(radius=ARM_TUBE_RADIUS, length=ARM_TUBE_LENGTH),
        origin=Origin(xyz=tube_center, rpy=(0.0, math.pi / 2.0 + ARM_DROP, 0.0)),
        material="brass",
        name="arm_tube",
    )

    cheek_center_base = _scale_vec(direction, ARM_PIVOT_REACH - 0.014)
    for side, side_sign in (("a", -1.0), ("b", 1.0)):
        cheek_center = _add_vec(cheek_center_base, (0.0, side_sign * 0.007, 0.0))
        arm.visual(
            Box((0.022, 0.004, 0.010)),
            origin=Origin(xyz=cheek_center, rpy=(0.0, ARM_DROP, 0.0)),
            material="brass",
            name=f"tip_bridge_{side}",
        )
        arm.visual(
            Cylinder(radius=0.0055, length=0.008),
            origin=Origin(
                xyz=_add_vec(_scale_vec(direction, ARM_PIVOT_REACH), (0.0, side_sign * 0.009, 0.0)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material="brass",
            name=f"tip_hinge_{side}",
        )


def _add_holder_geometry(holder, index: int) -> None:
    holder.visual(
        Cylinder(radius=HOLDER_BARREL_RADIUS, length=0.010),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="brass",
        name="tilt_barrel",
    )
    holder.visual(
        Cylinder(radius=0.0045, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material="brass",
        name="stem",
    )
    holder.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material="brass",
        name="swivel_cap",
    )
    holder.visual(
        Cylinder(radius=0.029, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material="brass",
        name="socket_cup",
    )
    holder.visual(
        _build_globe_mesh(f"holder_{index}_globe"),
        material="opal_glass",
        name="globe_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_ceiling_light")

    model.material("brass", rgba=(0.69, 0.58, 0.33, 1.0))
    model.material("opal_glass", rgba=(0.96, 0.95, 0.92, 0.72))

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * CANOPY_THICKNESS)),
        material="brass",
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.046, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material="brass",
        name="center_hub",
    )
    canopy.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material="brass",
        name="hub_cap",
    )

    arm_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(arm_angles):
        _add_canopy_hinge(canopy, angle, index)

        arm = model.part(f"arm_{index}")
        _add_arm_geometry(arm, index)

        holder = model.part(f"holder_{index}")
        _add_holder_geometry(holder, index)

        pivot_xyz = (
            CANOPY_PIVOT_RADIUS * math.cos(angle),
            CANOPY_PIVOT_RADIUS * math.sin(angle),
            CANOPY_PIVOT_Z,
        )
        model.articulation(
            f"canopy_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=arm,
            origin=Origin(xyz=pivot_xyz, rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=-0.35, upper=0.75, effort=16.0, velocity=1.4),
        )

        tip_local = _scale_vec(_arm_dir(), ARM_PIVOT_REACH)
        model.articulation(
            f"arm_{index}_to_holder_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=holder,
            origin=Origin(xyz=tip_local),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=6.0, velocity=1.6),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    arm_0 = object_model.get_part("arm_0")
    holder_0 = object_model.get_part("holder_0")
    arm_joint_0 = object_model.get_articulation("canopy_to_arm_0")
    holder_joint_0 = object_model.get_articulation("arm_0_to_holder_0")

    ctx.expect_contact(
        canopy,
        arm_0,
        contact_tol=0.0005,
        name="arm_0 is mounted at the canopy rim hinge",
    )
    ctx.expect_contact(
        arm_0,
        holder_0,
        contact_tol=0.0005,
        name="holder_0 is mounted at the arm tip hinge",
    )
    ctx.expect_gap(
        canopy,
        holder_0,
        axis="z",
        min_gap=0.08,
        negative_elem="globe_shell",
        name="globe_0 hangs clearly below the canopy",
    )

    rest_tip = ctx.part_world_position(holder_0)
    with ctx.pose({arm_joint_0: 0.50}):
        raised_tip = ctx.part_world_position(holder_0)
    ctx.check(
        "arm_0 joint raises the branch",
        rest_tip is not None and raised_tip is not None and raised_tip[2] > rest_tip[2] + 0.035,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    rest_globe_center = _aabb_center(ctx.part_element_world_aabb(holder_0, elem="globe_shell"))
    with ctx.pose({holder_joint_0: 0.40}):
        tilted_globe_center = _aabb_center(ctx.part_element_world_aabb(holder_0, elem="globe_shell"))
    ctx.check(
        "holder_0 joint tilts the globe outward",
        rest_globe_center is not None
        and tilted_globe_center is not None
        and tilted_globe_center[0] > rest_globe_center[0] + 0.015,
        details=f"rest_center={rest_globe_center}, tilted_center={tilted_globe_center}",
    )

    return ctx.report()


object_model = build_object_model()
