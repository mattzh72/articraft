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


CABINET_WIDTH = 0.84
BASE_DEPTH = 1.78
BASE_HEIGHT = 0.05

DASH_ANGLE = math.radians(22.0)
DASH_ANGLE_DEG = math.degrees(DASH_ANGLE)
DASH_Y = 0.73
DASH_Z = 0.76

BUTTON_XS = (-0.15, -0.05, 0.05, 0.15)
BUTTON_CAP_WIDTH = 0.052
BUTTON_CAP_DEPTH = 0.030
BUTTON_CAP_HEIGHT = 0.010
BUTTON_STEM_WIDTH = 0.024
BUTTON_STEM_DEPTH = 0.012
BUTTON_STEM_HEIGHT = 0.042
BUTTON_TRAVEL = 0.008

COIN_DOOR_WIDTH = 0.29
COIN_DOOR_HEIGHT = 0.41
COIN_DOOR_THICKNESS = 0.018
COIN_OPENING_WIDTH = 0.26
COIN_OPENING_HEIGHT = 0.38
COIN_DOOR_BOTTOM = 0.13

STEERING_TILT = math.radians(40.0)
STEERING_CENTER = (0.0, 1.09, 0.87)


def _build_cabinet_shell() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(CABINET_WIDTH, BASE_DEPTH, BASE_HEIGHT)
        .translate((0.0, BASE_DEPTH / 2.0, BASE_HEIGHT / 2.0))
    )

    front_profile_yz = [
        (0.00, 0.00),
        (0.00, 0.80),
        (0.10, 0.82),
        (0.22, 1.10),
        (0.37, 1.42),
        (0.51, 1.60),
        (0.77, 1.60),
        (0.89, 1.38),
        (0.93, 1.10),
        (0.94, 0.86),
        (0.80, 0.81),
        (0.62, 0.77),
        (0.32, 0.71),
        (0.10, 0.66),
        (0.00, 0.60),
    ]
    front_profile = [(z_pos, y_pos) for y_pos, z_pos in front_profile_yz]
    front_body = (
        cq.Workplane("YZ")
        .polyline(front_profile)
        .close()
        .extrude(CABINET_WIDTH * 0.48, both=True)
    )

    pod_profile_yz = [
        (0.68, 0.00),
        (0.74, 0.16),
        (0.86, 0.40),
        (1.02, 0.52),
        (1.24, 0.58),
        (1.43, 0.70),
        (1.59, 0.92),
        (1.72, 1.00),
        (1.78, 0.90),
        (1.78, 0.08),
    ]
    pod_profile = [(z_pos, y_pos) for y_pos, z_pos in pod_profile_yz]
    seat_pod = (
        cq.Workplane("YZ")
        .polyline(pod_profile)
        .close()
        .extrude(CABINET_WIDTH * 0.5, both=True)
    )

    shell = base.union(front_body).union(seat_pod)

    cockpit_cut = (
        cq.Workplane("XY")
        .box(0.46, 0.74, 0.68)
        .translate((0.0, 1.25, 0.64))
    )
    footwell_cut = (
        cq.Workplane("XY")
        .box(0.34, 0.58, 0.34)
        .translate((0.0, 0.98, 0.40))
    )
    coin_opening = (
        cq.Workplane("XY")
        .box(COIN_OPENING_WIDTH, 0.18, COIN_OPENING_HEIGHT)
        .translate((0.0, 0.09, COIN_DOOR_BOTTOM + COIN_OPENING_HEIGHT / 2.0))
    )
    shell = shell.cut(cockpit_cut).cut(footwell_cut).cut(coin_opening)

    for button_x in BUTTON_XS:
        slot = (
            cq.Workplane("XY")
            .box(0.030, 0.018, 0.090)
            .translate((0.0, 0.0, -0.044))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -DASH_ANGLE_DEG)
            .translate((button_x, DASH_Y, DASH_Z))
        )
        shell = shell.cut(slot)

    return shell


def _build_button_shape() -> cq.Workplane:
    bezel = (
        cq.Workplane("XY")
        .box(0.036, 0.022, 0.002)
        .translate((0.0, 0.0, 0.001))
    )
    cap = (
        cq.Workplane("XY")
        .box(BUTTON_CAP_WIDTH, BUTTON_CAP_DEPTH, BUTTON_CAP_HEIGHT)
        .translate((0.0, 0.0, 0.002 + BUTTON_CAP_HEIGHT / 2.0))
    )
    stem = (
        cq.Workplane("XY")
        .box(BUTTON_STEM_WIDTH, BUTTON_STEM_DEPTH, BUTTON_STEM_HEIGHT)
        .translate((0.0, 0.0, 0.002 - BUTTON_STEM_HEIGHT / 2.0))
    )
    return bezel.union(cap).union(stem)


def _build_dash_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.44, 0.16, 0.14)
        .translate((0.0, 0.0, -0.07))
    )
    for button_x in BUTTON_XS:
        slot = (
            cq.Workplane("XY")
            .box(0.028, 0.018, 0.060)
            .translate((button_x, 0.0, -0.030))
        )
        plate = plate.cut(slot)
    return plate


def _build_coin_door() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(COIN_DOOR_WIDTH, COIN_DOOR_THICKNESS, COIN_DOOR_HEIGHT)
        .translate((COIN_DOOR_WIDTH / 2.0, -COIN_DOOR_THICKNESS / 2.0, COIN_DOOR_HEIGHT / 2.0))
    )
    panel_recess = (
        cq.Workplane("XY")
        .box(COIN_DOOR_WIDTH - 0.050, COIN_DOOR_THICKNESS * 0.48, COIN_DOOR_HEIGHT - 0.070)
        .translate((COIN_DOOR_WIDTH / 2.0, -COIN_DOOR_THICKNESS * 0.72, COIN_DOOR_HEIGHT / 2.0))
    )
    ticket_boss = (
        cq.Workplane("XY")
        .box(0.082, COIN_DOOR_THICKNESS * 0.55, 0.044)
        .translate((COIN_DOOR_WIDTH * 0.35, -COIN_DOOR_THICKNESS * 0.64, COIN_DOOR_HEIGHT * 0.62))
    )
    pull_handle = (
        cq.Workplane("XY")
        .box(0.024, COIN_DOOR_THICKNESS * 0.72, 0.110)
        .translate((COIN_DOOR_WIDTH * 0.82, -COIN_DOOR_THICKNESS * 0.64, COIN_DOOR_HEIGHT * 0.34))
    )
    coin_slot = (
        cq.Workplane("XY")
        .box(0.060, COIN_DOOR_THICKNESS * 0.60, 0.010)
        .translate((COIN_DOOR_WIDTH * 0.34, -COIN_DOOR_THICKNESS * 0.72, COIN_DOOR_HEIGHT * 0.82))
    )
    lock_cylinder = (
        cq.Workplane("XZ")
        .circle(0.008)
        .extrude(COIN_DOOR_THICKNESS * 0.62)
        .translate((COIN_DOOR_WIDTH * 0.78, -COIN_DOOR_THICKNESS * 0.76, COIN_DOOR_HEIGHT * 0.48))
    )
    return panel.cut(panel_recess).cut(coin_slot).union(ticket_boss).union(pull_handle).union(lock_cylinder)


def _build_coin_frame() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(0.33, 0.18, 0.45)
        .translate((0.0, 0.09, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(COIN_OPENING_WIDTH, 0.20, COIN_OPENING_HEIGHT)
        .translate((0.0, 0.10, 0.0))
    )
    return outer.cut(inner)


def _build_steering_wheel() -> cq.Workplane:
    ring = cq.Workplane("XY").circle(0.18).circle(0.145).extrude(0.028, both=True)
    hub = cq.Workplane("XY").circle(0.045).extrude(0.036, both=True)
    wheel = ring.union(hub)
    for angle in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .transformed(rotate=(0.0, 0.0, angle))
            .center(0.092, 0.0)
            .rect(0.118, 0.020)
            .extrude(0.018, both=True)
        )
        wheel = wheel.union(spoke)
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sit_down_arcade_racer")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.10, 0.16, 0.28, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    seat_finish = model.material("seat_finish", rgba=(0.16, 0.16, 0.17, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.06, 0.06, 0.06, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.09, 0.15, 0.18, 0.55))
    metal_finish = model.material("metal_finish", rgba=(0.56, 0.58, 0.62, 1.0))
    button_red = model.material("button_red", rgba=(0.88, 0.14, 0.16, 0.95))
    button_amber = model.material("button_amber", rgba=(0.95, 0.60, 0.10, 0.95))
    button_green = model.material("button_green", rgba=(0.20, 0.82, 0.34, 0.95))
    button_blue = model.material("button_blue", rgba=(0.16, 0.56, 0.96, 0.95))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_shell(), "arcade_cabinet_shell"),
        material=cabinet_finish,
        name="shell",
    )
    cabinet.visual(
        Box((0.37, 0.36, 0.20)),
        origin=Origin(xyz=(0.0, 1.24, 0.31)),
        material=trim_finish,
        name="seat_pedestal",
    )
    cabinet.visual(
        Box((0.43, 0.40, 0.09)),
        origin=Origin(xyz=(0.0, 1.22, 0.45)),
        material=seat_finish,
        name="seat_base",
    )
    cabinet.visual(
        Box((0.41, 0.14, 0.54)),
        origin=Origin(xyz=(0.0, 1.43, 0.72), rpy=(-0.30, 0.0, 0.0)),
        material=seat_finish,
        name="seat_back",
    )
    cabinet.visual(
        Box((0.64, 0.22, 0.52)),
        origin=Origin(xyz=(0.0, 0.56, 1.02), rpy=(-0.20, 0.0, 0.0)),
        material=trim_finish,
        name="screen_shroud",
    )
    cabinet.visual(
        Box((0.50, 0.04, 0.32)),
        origin=Origin(xyz=(0.0, 0.49, 1.06), rpy=(-0.20, 0.0, 0.0)),
        material=glass_finish,
        name="screen",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_dash_plate(), "arcade_dash_plate"),
        origin=Origin(xyz=(0.0, DASH_Y, DASH_Z), rpy=(-DASH_ANGLE, 0.0, 0.0)),
        material=trim_finish,
        name="dash_plate",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_coin_frame(), "arcade_coin_frame"),
        origin=Origin(xyz=(0.0, 0.0, COIN_DOOR_BOTTOM + COIN_DOOR_HEIGHT / 2.0)),
        material=trim_finish,
        name="coin_frame",
    )
    cabinet.visual(
        Cylinder(radius=0.035, length=0.28),
        origin=Origin(xyz=(0.0, 0.99, 0.75), rpy=(-STEERING_TILT, 0.0, 0.0)),
        material=trim_finish,
        name="steering_column",
    )

    steering_wheel = model.part("steering_wheel")
    steering_wheel.visual(
        mesh_from_cadquery(_build_steering_wheel(), "arcade_steering_wheel"),
        material=wheel_finish,
        name="wheel",
    )
    model.articulation(
        "cabinet_to_steering_wheel",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=steering_wheel,
        origin=Origin(xyz=STEERING_CENTER, rpy=(-STEERING_TILT, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=8.0,
        ),
    )

    coin_door = model.part("coin_door")
    coin_door.visual(
        mesh_from_cadquery(_build_coin_door(), "arcade_coin_door"),
        material=metal_finish,
        name="door_panel",
    )
    model.articulation(
        "cabinet_to_coin_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=coin_door,
        origin=Origin(
            xyz=(
                -COIN_DOOR_WIDTH / 2.0,
                0.0,
                COIN_DOOR_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=1.35,
        ),
    )

    button_mesh = mesh_from_cadquery(_build_button_shape(), "arcade_gear_button")
    button_finishes = (button_red, button_amber, button_green, button_blue)
    for index, (button_x, finish) in enumerate(zip(BUTTON_XS, button_finishes)):
        button_part = model.part(f"gear_button_{index}")
        button_part.visual(
            button_mesh,
            material=finish,
            name="button",
        )
        model.articulation(
            f"cabinet_to_gear_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button_part,
            origin=Origin(xyz=(button_x, DASH_Y, DASH_Z), rpy=(-DASH_ANGLE, 0.0, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    coin_door = object_model.get_part("coin_door")
    door_hinge = object_model.get_articulation("cabinet_to_coin_door")
    steering_wheel = object_model.get_part("steering_wheel")

    ctx.allow_overlap(
        cabinet,
        steering_wheel,
        elem_a="steering_column",
        elem_b="wheel",
        reason="The steering column is intentionally simplified as a solid shaft nested into the wheel hub.",
    )

    for index in range(len(BUTTON_XS)):
        ctx.allow_overlap(
            cabinet,
            object_model.get_part(f"gear_button_{index}"),
            elem_a="dash_plate",
            elem_b="button",
            reason="Each gear-select button is simplified as a guided push-button assembly nesting into the dash plate proxy.",
        )

    ctx.expect_gap(
        cabinet,
        coin_door,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        name="coin door sits flush against the cabinet front",
    )

    closed_aabb = ctx.part_element_world_aabb(coin_door, elem="door_panel")
    door_upper = door_hinge.motion_limits.upper if door_hinge.motion_limits is not None else None
    if closed_aabb is not None and door_upper is not None:
        with ctx.pose({door_hinge: door_upper}):
            open_aabb = ctx.part_element_world_aabb(coin_door, elem="door_panel")
        ctx.check(
            "coin door swings outward on its vertical hinge",
            open_aabb is not None and open_aabb[0][1] < closed_aabb[0][1] - 0.10,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    for index in range(len(BUTTON_XS)):
        button = object_model.get_part(f"gear_button_{index}")
        joint = object_model.get_articulation(f"cabinet_to_gear_button_{index}")
        joint_upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        rest_position = ctx.part_world_position(button)
        pressed_position = None
        if joint_upper is not None:
            with ctx.pose({joint: joint_upper}):
                pressed_position = ctx.part_world_position(button)
        ctx.check(
            f"gear button {index} depresses into the dash",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[2] < rest_position[2] - BUTTON_TRAVEL * 0.7
            and pressed_position[1] < rest_position[1] - BUTTON_TRAVEL * 0.2,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    return ctx.report()


object_model = build_object_model()
