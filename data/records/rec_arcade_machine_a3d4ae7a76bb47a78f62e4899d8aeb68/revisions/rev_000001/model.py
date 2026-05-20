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


def _pod_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.80, 0.06),
                (-0.80, 1.08),
                (-0.60, 1.20),
                (-0.30, 1.22),
                (-0.08, 1.04),
                (0.04, 0.78),
                (0.04, 0.44),
                (-0.12, 0.34),
                (-0.46, 0.30),
                (-0.68, 0.26),
                (-0.80, 0.18),
            ]
        )
        .close()
        .extrude(0.82 / 2.0, both=True)
    )
    seat_well = cq.Workplane("XY").box(0.74, 0.60, 0.86).translate((-0.28, 0.0, 0.78))
    leg_opening = cq.Workplane("XY").box(0.30, 0.56, 0.56).translate((0.03, 0.0, 0.34))
    door_opening = cq.Workplane("XY").box(0.12, 0.50, 0.44).translate((-0.79, 0.0, 0.34))
    return outer.cut(seat_well).cut(leg_opening).cut(door_opening)


def _tower_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.42, 0.06),
                (0.84, 0.06),
                (0.90, 0.32),
                (0.90, 1.48),
                (0.66, 1.48),
                (0.54, 1.40),
                (0.38, 1.06),
            ]
        )
        .close()
        .extrude(0.76 / 2.0, both=True)
    )


def _door_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.028, 0.56, 0.50).translate((-0.786, 0.0, 0.34))
    opening = cq.Workplane("XY").box(0.040, 0.48, 0.42).translate((-0.786, 0.0, 0.34))
    return frame.cut(opening)


def _dashboard_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.18, 0.42),
                (0.02, 0.42),
                (0.08, 0.62),
                (0.04, 0.88),
                (-0.08, 0.86),
                (-0.18, 0.60),
            ]
        )
        .close()
        .extrude(0.70 / 2.0, both=True)
    )


def _wheel_rim_shape() -> cq.Workplane:
    return cq.Workplane("YZ").circle(0.19).circle(0.145).extrude(0.028, both=True)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sitdown_arcade_cabinet")

    shell_finish = model.material("shell_finish", rgba=(0.14, 0.17, 0.24, 1.0))
    console_finish = model.material("console_finish", rgba=(0.18, 0.19, 0.22, 1.0))
    seat_finish = model.material("seat_finish", rgba=(0.07, 0.07, 0.08, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.02, 0.02, 0.03, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.72, 0.74, 0.78, 1.0))
    accent_finish = model.material("accent_finish", rgba=(0.76, 0.10, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((1.76, 0.94, 0.08)),
        origin=Origin(xyz=(0.03, 0.0, 0.04)),
        material=shell_finish,
        name="floor_plinth",
    )
    body.visual(
        mesh_from_cadquery(_pod_shell_shape(), "pod_shell"),
        material=shell_finish,
        name="pod_shell",
    )
    body.visual(
        mesh_from_cadquery(_tower_shape(), "tower_shell"),
        material=shell_finish,
        name="tower_shell",
    )
    body.visual(
        mesh_from_cadquery(_door_frame_shape(), "door_frame"),
        material=trim_finish,
        name="door_frame",
    )
    body.visual(
        Box((0.24, 0.46, 0.22)),
        origin=Origin(xyz=(-0.30, 0.0, 0.19)),
        material=shell_finish,
        name="seat_support",
    )
    body.visual(
        Box((0.42, 0.50, 0.12)),
        origin=Origin(xyz=(-0.16, 0.0, 0.34)),
        material=seat_finish,
        name="seat_base",
    )
    body.visual(
        Box((0.20, 0.50, 0.60)),
        origin=Origin(xyz=(-0.36, 0.0, 0.66), rpy=(0.0, -0.14, 0.0)),
        material=seat_finish,
        name="seat_back",
    )
    body.visual(
        Box((0.08, 0.68, 0.48)),
        origin=Origin(xyz=(0.52, 0.0, 1.12), rpy=(0.0, 0.16, 0.0)),
        material=trim_finish,
        name="screen_bezel",
    )
    body.visual(
        Box((0.10, 0.60, 0.40)),
        origin=Origin(xyz=(0.56, 0.0, 1.12), rpy=(0.0, 0.16, 0.0)),
        material=screen_finish,
        name="screen",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.11),
        origin=Origin(xyz=(-0.788, -0.242, 0.485)),
        material=trim_finish,
        name="hinge_jamb_top",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.11),
        origin=Origin(xyz=(-0.788, -0.242, 0.195)),
        material=trim_finish,
        name="hinge_jamb_bottom",
    )

    console = model.part("console")
    console.visual(
        Box((0.18, 0.36, 0.52)),
        origin=Origin(xyz=(0.02, 0.0, 0.26)),
        material=console_finish,
        name="pedestal",
    )
    console.visual(
        mesh_from_cadquery(_dashboard_shape(), "dashboard"),
        material=console_finish,
        name="dashboard",
    )
    console.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(-0.09, 0.0, 0.78), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=console_finish,
        name="column_shroud",
    )
    console.visual(
        Box((0.16, 0.24, 0.10)),
        origin=Origin(xyz=(0.00, 0.0, 0.88)),
        material=trim_finish,
        name="gauge_binnacle",
    )

    steering_wheel = model.part("steering_wheel")
    steering_wheel.visual(
        mesh_from_cadquery(_wheel_rim_shape(), "steering_wheel_rim"),
        material=seat_finish,
        name="rim",
    )
    steering_wheel.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_finish,
        name="hub",
    )
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, -2.0 * math.pi / 3.0)):
        steering_wheel.visual(
            Box((0.038, 0.028, 0.33)),
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=trim_finish,
            name=f"spoke_{idx}",
        )
    steering_wheel.visual(
        Box((0.026, 0.05, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=accent_finish,
        name="grip_mark",
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.018, 0.46, 0.40)),
        origin=Origin(xyz=(0.009, 0.23, 0.0)),
        material=console_finish,
        name="panel",
    )
    service_door.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(
            xyz=(-0.007, 0.34, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_finish,
        name="handle",
    )
    service_door.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        material=trim_finish,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_console",
        ArticulationType.FIXED,
        parent=body,
        child=console,
        origin=Origin(xyz=(0.24, 0.0, 0.08)),
    )
    model.articulation(
        "console_to_steering_wheel",
        ArticulationType.CONTINUOUS,
        parent=console,
        child=steering_wheel,
        origin=Origin(xyz=(-0.22, 0.0, 0.78)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=12.0),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(-0.80, -0.23, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=8.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    console = object_model.get_part("console")
    wheel = object_model.get_part("steering_wheel")
    door = object_model.get_part("service_door")
    door_hinge = object_model.get_articulation("body_to_service_door")
    wheel_spin = object_model.get_articulation("console_to_steering_wheel")

    ctx.expect_contact(
        console,
        body,
        elem_a="pedestal",
        elem_b="floor_plinth",
        name="console pedestal sits on the cabinet plinth",
    )

    pod_aabb = ctx.part_element_world_aabb(body, elem="pod_shell")
    dash_aabb = ctx.part_element_world_aabb(console, elem="dashboard")
    ctx.check(
        "console projects forward of the seat pod",
        pod_aabb is not None
        and dash_aabb is not None
        and dash_aabb[1][0] > pod_aabb[1][0] + 0.18,
        details=f"pod_shell={pod_aabb}, dashboard={dash_aabb}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="panel",
            elem_b="door_frame",
            min_overlap=0.36,
            name="service door covers the rear access opening",
        )
        closed_panel = ctx.part_element_world_aabb(door, elem="panel")

    with ctx.pose({door_hinge: 1.2}):
        open_panel = ctx.part_element_world_aabb(door, elem="panel")

    ctx.check(
        "service door swings outward behind the seat pod",
        closed_panel is not None
        and open_panel is not None
        and open_panel[0][0] < closed_panel[0][0] - 0.16,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    rest_wheel_pos = ctx.part_world_position(wheel)
    rest_mark_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem="grip_mark"))

    with ctx.pose({wheel_spin: 1.2}):
        spun_wheel_pos = ctx.part_world_position(wheel)
        spun_mark_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem="grip_mark"))

    ctx.check(
        "steering wheel spins on its horizontal column axis",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and rest_mark_center is not None
        and spun_mark_center is not None
        and max(abs(spun_wheel_pos[i] - rest_wheel_pos[i]) for i in range(3)) < 1e-6
        and abs(spun_mark_center[1] - rest_mark_center[1]) > 0.10
        and abs(spun_mark_center[2] - rest_mark_center[2]) > 0.04,
        details=(
            f"rest_wheel={rest_wheel_pos}, spun_wheel={spun_wheel_pos}, "
            f"rest_mark={rest_mark_center}, spun_mark={spun_mark_center}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
