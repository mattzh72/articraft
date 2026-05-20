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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _cylinder_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_cylinder_rpy(start, end)),
        material=material,
        name=name,
    )


def _build_seat_shell_mesh():
    shell = cq.Workplane("XY").circle(0.178).extrude(0.018)
    recess = cq.Workplane("XY").circle(0.122).extrude(0.012)
    return shell.cut(recess).translate((0.0, 0.0, 0.016))


def _build_backrest_ring_mesh():
    return (
        cq.Workplane("XZ")
        .circle(0.112)
        .circle(0.094)
        .extrude(0.016)
        .translate((0.0, -0.208, 0.205))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_swivel_stool")

    black_vinyl = model.material("black_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.27, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    warm_shell = model.material("warm_shell", rgba=(0.41, 0.28, 0.19, 1.0))

    seat_shell_mesh = mesh_from_cadquery(_build_seat_shell_mesh(), "seat_shell")
    backrest_ring_mesh = mesh_from_cadquery(_build_backrest_ring_mesh(), "backrest_ring")

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.225, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.180, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=dark_steel,
        name="base_skirt",
    )
    pedestal.visual(
        Cylinder(radius=0.034, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=brushed_steel,
        name="pedestal_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.052, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=dark_steel,
        name="upper_sleeve",
    )
    pedestal.visual(
        Cylinder(radius=0.066, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=brushed_steel,
        name="bearing_cap",
    )

    seat = model.part("seat")
    seat.visual(seat_shell_mesh, material=warm_shell, name="seat_shell")
    seat.visual(
        Cylinder(radius=0.166, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=black_vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="swivel_housing",
    )
    _add_tube(
        seat,
        start=(-0.068, -0.105, 0.026),
        end=(-0.068, -0.205, 0.125),
        radius=0.0075,
        material=dark_steel,
        name="backrest_support_0",
    )
    _add_tube(
        seat,
        start=(0.068, -0.105, 0.026),
        end=(0.068, -0.205, 0.125),
        radius=0.0075,
        material=dark_steel,
        name="backrest_support_1",
    )
    seat.visual(backrest_ring_mesh, material=dark_steel, name="backrest_ring")
    seat.visual(
        Box((0.028, 0.050, 0.004)),
        origin=Origin(xyz=(0.128, 0.0, 0.016)),
        material=dark_steel,
        name="hook_mount",
    )
    seat.visual(
        Box((0.012, 0.050, 0.020)),
        origin=Origin(xyz=(0.1605, 0.0, 0.006)),
        material=dark_steel,
        name="hook_leaf",
    )

    hook = model.part("hook")
    hook.visual(
        Cylinder(radius=0.0065, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hook_barrel",
    )
    hook.visual(
        Box((0.040, 0.012, 0.010)),
        origin=Origin(xyz=(-0.026, 0.0, 0.0)),
        material=dark_steel,
        name="hook_arm",
    )
    hook.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(xyz=(-0.050, 0.0, -0.016)),
        material=dark_steel,
        name="hook_drop",
    )
    hook.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(-0.042, 0.0, -0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hook_tip",
    )

    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=4.0),
    )
    model.articulation(
        "hook_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=hook,
        origin=Origin(xyz=(0.148, 0.0, 0.008)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    hook = object_model.get_part("hook")
    seat_swivel = object_model.get_articulation("seat_swivel")
    hook_hinge = object_model.get_articulation("hook_hinge")

    ctx.expect_origin_distance(
        seat,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="seat stays centered over pedestal axis",
    )
    ctx.expect_gap(
        seat,
        pedestal,
        axis="z",
        positive_elem="swivel_housing",
        negative_elem="bearing_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel housing seats on the bearing cap",
    )
    ctx.expect_overlap(
        seat,
        pedestal,
        axes="xy",
        elem_a="swivel_housing",
        elem_b="bearing_cap",
        min_overlap=0.120,
        name="swivel housing remains coaxial with the bearing cap",
    )

    with ctx.pose({seat_swivel: 1.30}):
        ctx.expect_origin_distance(
            seat,
            pedestal,
            axes="xy",
            max_dist=0.001,
            name="seat swivel keeps the seat centered",
        )
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            positive_elem="swivel_housing",
            negative_elem="bearing_cap",
            max_gap=0.001,
            max_penetration=0.0,
            name="swivel interface stays seated while rotated",
        )

    seat_shell_aabb = ctx.part_element_world_aabb(seat, elem="seat_shell")
    ring_aabb = ctx.part_element_world_aabb(seat, elem="backrest_ring")
    ctx.check(
        "backrest ring sits behind and above the seat shell",
        seat_shell_aabb is not None
        and ring_aabb is not None
        and ring_aabb[1][1] < seat_shell_aabb[0][1] - 0.010
        and ring_aabb[0][2] > seat_shell_aabb[1][2] + 0.040,
        details=f"seat_shell_aabb={seat_shell_aabb}, ring_aabb={ring_aabb}",
    )

    closed_arm_aabb = ctx.part_element_world_aabb(hook, elem="hook_arm")
    with ctx.pose({hook_hinge: 1.25}):
        open_arm_aabb = ctx.part_element_world_aabb(hook, elem="hook_arm")
        ctx.expect_gap(
            seat,
            hook,
            axis="z",
            positive_elem="seat_shell",
            negative_elem="hook_tip",
            min_gap=0.040,
            name="opened hook hangs well below the seat shell",
        )
    ctx.check(
        "hook swings downward from its stowed position",
        closed_arm_aabb is not None
        and open_arm_aabb is not None
        and open_arm_aabb[0][2] < closed_arm_aabb[0][2] - 0.030,
        details=f"closed_arm_aabb={closed_arm_aabb}, open_arm_aabb={open_arm_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
