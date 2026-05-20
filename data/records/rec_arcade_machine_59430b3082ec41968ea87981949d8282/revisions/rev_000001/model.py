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


CABINET_WIDTH = 0.98
CABINET_LENGTH = 1.80
WALL_THICKNESS = 0.045

DOOR_WIDTH = 0.34
DOOR_HEIGHT = 0.48
DOOR_THICKNESS = 0.026
DOOR_CENTER_Z = 0.56

FLAP_WIDTH = 0.125
FLAP_HEIGHT = 0.055
FLAP_THICKNESS = 0.010
FLAP_CENTER_Z = -0.145


def _cabinet_shell_shape() -> cq.Workplane:
    outer_profile = [
        (0.00, 0.00),
        (CABINET_LENGTH, 0.00),
        (CABINET_LENGTH, 0.68),
        (1.73, 0.92),
        (1.56, 1.16),
        (1.36, 1.40),
        (1.10, 1.62),
        (0.74, 1.73),
        (0.44, 1.68),
        (0.28, 1.50),
        (0.16, 1.10),
        (0.08, 0.90),
        (0.00, 0.72),
    ]

    shell = (
        cq.Workplane("XZ")
        .polyline(outer_profile)
        .close()
        .extrude(CABINET_WIDTH * 0.5, both=True)
        .faces("<Z")
        .shell(-WALL_THICKNESS)
    )

    cockpit_cut = cq.Workplane("XY").box(0.94, 0.70, 1.10).translate((1.20, 0.0, 1.12))
    coin_door_cut = cq.Workplane("XY").box(
        0.16,
        DOOR_WIDTH + 0.002,
        DOOR_HEIGHT + 0.004,
    ).translate((0.07, 0.0, DOOR_CENTER_Z))
    side_entry_cut = cq.Workplane("XY").box(0.46, 0.92, 0.62).translate((1.56, 0.0, 0.63))

    return shell.cut(cockpit_cut).cut(coin_door_cut).cut(side_entry_cut)


def _seat_shape() -> cq.Workplane:
    seat_shell = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.00, 0.22),
                (0.18, 0.22),
                (0.28, 0.26),
                (0.38, 0.40),
                (0.43, 0.82),
                (0.32, 0.98),
                (0.16, 0.94),
                (0.04, 0.76),
                (-0.02, 0.44),
            ]
        )
        .close()
        .extrude(0.26, both=True)
    )
    pedestal = cq.Workplane("XY").circle(0.060).extrude(0.22).translate((0.16, 0.0, 0.0))
    headrest = cq.Workplane("XY").box(0.10, 0.28, 0.12, centered=(True, True, False)).translate((0.34, 0.0, 0.90))
    return seat_shell.union(pedestal).union(headrest)


def _console_pod_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.20, -0.11),
                (0.07, -0.11),
                (0.16, -0.03),
                (0.18, 0.05),
                (0.12, 0.12),
                (-0.02, 0.17),
                (-0.16, 0.10),
            ]
        )
        .close()
        .extrude(0.36, both=True)
    )
    hood = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.11, 0.02),
                (0.06, 0.02),
                (0.13, 0.08),
                (0.12, 0.14),
                (-0.02, 0.18),
                (-0.11, 0.11),
            ]
        )
        .close()
        .extrude(0.20, both=True)
        .translate((0.02, 0.0, 0.01))
    )
    return base.union(hood)


def _steering_wheel_shape() -> cq.Workplane:
    shaft = cq.Workplane("YZ").circle(0.024).extrude(0.16)
    hub = cq.Workplane("YZ").circle(0.055).extrude(0.06).translate((0.10, 0.0, 0.0))
    rim = (
        cq.Workplane("YZ")
        .circle(0.205)
        .circle(0.170)
        .extrude(0.028)
        .translate((0.13, 0.0, 0.0))
    )

    wheel = shaft.union(hub).union(rim)
    for angle_deg in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("YZ")
            .rect(0.032, 0.340)
            .extrude(0.018)
            .translate((0.118, 0.0, 0.0))
            .rotate((0.118, 0.0, 0.0), (1.118, 0.0, 0.0), angle_deg)
        )
        wheel = wheel.union(spoke)
    return wheel


def _coin_door_shape() -> cq.Workplane:
    door = (
        cq.Workplane("YZ")
        .rect(DOOR_WIDTH, DOOR_HEIGHT)
        .extrude(DOOR_THICKNESS)
        .translate((0.0, DOOR_WIDTH * 0.5, 0.0))
    )
    door = door.edges("|X").fillet(0.008)

    service_box = cq.Workplane("XY").box(0.020, 0.11, 0.18).translate((0.010, DOOR_WIDTH * 0.5, 0.10))
    service_slot = cq.Workplane("XY").box(0.040, 0.070, 0.012).translate((0.012, DOOR_WIDTH * 0.5, 0.17))
    flap_opening = cq.Workplane("XY").box(
        DOOR_THICKNESS + 0.020,
        FLAP_WIDTH + 0.001,
        FLAP_HEIGHT + 0.001,
    ).translate((DOOR_THICKNESS * 0.5, DOOR_WIDTH * 0.5, FLAP_CENTER_Z))
    latch_cut = cq.Workplane("XY").box(0.030, 0.050, 0.090).translate((0.012, DOOR_WIDTH * 0.74, 0.02))

    return door.union(service_box).cut(service_slot).cut(flap_opening).cut(latch_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sitdown_driving_arcade_cabinet")

    shell_blue = model.material("shell_blue", rgba=(0.09, 0.21, 0.59, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))
    seat_black = model.material("seat_black", rgba=(0.11, 0.11, 0.12, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.07, 0.07, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.05, 0.06, 0.08, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell_shape(), "arcade_cabinet_shell"),
        material=shell_blue,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((1.10, 1.00, 0.05)),
        origin=Origin(xyz=(1.25, 0.0, 0.215)),
        material=trim_black,
        name="floor_pan",
    )
    cabinet.visual(
        mesh_from_cadquery(_console_pod_shape(), "arcade_console_pod"),
        origin=Origin(xyz=(0.50, 0.0, 0.88), rpy=(0.0, -0.30, 0.0)),
        material=trim_black,
        name="console_pod",
    )
    cabinet.visual(
        Cylinder(radius=0.055, length=0.16),
        origin=Origin(xyz=(0.68, 0.0, 0.88), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim_black,
        name="column_shroud",
    )
    cabinet.visual(
        Box((0.08, 0.68, 0.45)),
        origin=Origin(xyz=(0.39, 0.0, 1.17), rpy=(0.0, -0.56, 0.0)),
        material=screen_glass,
        name="screen_glass",
    )

    seat = model.part("seat")
    seat.visual(
        mesh_from_cadquery(_seat_shape(), "arcade_seat"),
        material=seat_black,
        name="seat_shell",
    )

    steering_wheel = model.part("steering_wheel")
    steering_wheel.visual(
        mesh_from_cadquery(_steering_wheel_shape(), "arcade_steering_wheel"),
        material=wheel_black,
        name="wheel_assembly",
    )

    coin_door = model.part("coin_door")
    coin_door.visual(
        mesh_from_cadquery(_coin_door_shape(), "arcade_coin_door"),
        material=steel,
        name="door_panel",
    )
    coin_door.visual(
        Cylinder(radius=0.006, length=DOOR_HEIGHT * 0.90),
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
        material=dark_steel,
        name="door_hinge_barrel",
    )

    coin_return = model.part("coin_return")
    coin_return.visual(
        Box((FLAP_THICKNESS, FLAP_WIDTH, FLAP_HEIGHT)),
        origin=Origin(xyz=(-FLAP_THICKNESS * 0.5, 0.0, -FLAP_HEIGHT * 0.5)),
        material=dark_steel,
        name="flap_panel",
    )

    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent=cabinet,
        child=seat,
        origin=Origin(xyz=(1.14, 0.0, 0.24)),
    )
    model.articulation(
        "steering_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=steering_wheel,
        origin=Origin(xyz=(0.76, 0.0, 0.88)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=10.0),
    )
    model.articulation(
        "coin_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=coin_door,
        origin=Origin(xyz=(0.0, -DOOR_WIDTH * 0.5, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "coin_return_hinge",
        ArticulationType.REVOLUTE,
        parent=coin_door,
        child=coin_return,
        origin=Origin(xyz=(0.0005, DOOR_WIDTH * 0.5, FLAP_CENTER_Z + FLAP_HEIGHT * 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    coin_door = object_model.get_part("coin_door")
    coin_return = object_model.get_part("coin_return")
    steering_wheel = object_model.get_part("steering_wheel")

    coin_door_hinge = object_model.get_articulation("coin_door_hinge")
    coin_return_hinge = object_model.get_articulation("coin_return_hinge")
    steering_wheel_spin = object_model.get_articulation("steering_wheel_spin")

    ctx.allow_isolated_part(
        coin_return,
        reason="The return flap is intentionally modeled with a small clearance to the coin door opening; the hinge pin is represented by the articulation rather than a separate touching visual.",
    )

    ctx.expect_overlap(
        coin_door,
        cabinet,
        axes="yz",
        min_overlap=0.20,
        name="coin door sits in the cabinet front opening",
    )
    ctx.expect_overlap(
        coin_return,
        coin_door,
        axes="y",
        min_overlap=0.10,
        name="coin return flap spans the door opening width",
    )
    ctx.expect_overlap(
        coin_return,
        coin_door,
        axes="z",
        min_overlap=0.04,
        name="coin return flap remains vertically nested in the door opening",
    )
    ctx.expect_overlap(
        steering_wheel,
        cabinet,
        axes="yz",
        min_overlap=0.10,
        name="steering wheel is centered on the dashboard console",
    )

    closed_door_aabb = ctx.part_element_world_aabb(coin_door, elem="door_panel")
    with ctx.pose({coin_door_hinge: math.radians(78.0)}):
        open_door_aabb = ctx.part_element_world_aabb(coin_door, elem="door_panel")
    ctx.check(
        "coin door opens outward from the cabinet front",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][0] < closed_door_aabb[0][0] - 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(coin_return, elem="flap_panel")
    with ctx.pose({coin_return_hinge: math.radians(72.0)}):
        open_flap_aabb = ctx.part_element_world_aabb(coin_return, elem="flap_panel")
    ctx.check(
        "coin return flap tips outward on its horizontal hinge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][0] < closed_flap_aabb[0][0] - 0.02,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    closed_wheel_aabb = ctx.part_element_world_aabb(steering_wheel, elem="wheel_assembly")
    with ctx.pose({steering_wheel_spin: math.pi * 0.5}):
        spun_wheel_aabb = ctx.part_element_world_aabb(steering_wheel, elem="wheel_assembly")
    ctx.check(
        "steering wheel remains mounted while spinning",
        closed_wheel_aabb is not None
        and spun_wheel_aabb is not None
        and abs(spun_wheel_aabb[0][0] - closed_wheel_aabb[0][0]) < 0.02
        and abs(spun_wheel_aabb[1][0] - closed_wheel_aabb[1][0]) < 0.02,
        details=f"closed={closed_wheel_aabb}, spun={spun_wheel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
