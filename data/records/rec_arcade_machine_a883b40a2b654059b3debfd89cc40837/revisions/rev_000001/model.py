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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_WIDTH = 0.79
CABINET_DEPTH = 0.88
CABINET_HEIGHT = 1.74
SIDE_CLEAR = 0.03

DOOR_WIDTH = 0.33
DOOR_HEIGHT = 0.46
DOOR_THICKNESS = 0.024
DOOR_BOTTOM = 0.10

MONITOR_TILT_DEG = 18.0
MONITOR_CENTER_Y = 0.36
MONITOR_CENTER_Z = 1.31

DECK_WIDTH = 0.66
DECK_DEPTH = 0.30
DECK_THICKNESS = 0.055
DECK_MOUNT_Y = 0.075
DECK_MOUNT_Z = 0.99
DECK_TILT_DEG = 12.0

JOYSTICK_X = -0.17
JOYSTICK_Y = -0.13
JOYSTICK_PIVOT_Z = -0.038


def _body_shell_shape():
    outer_profile = [
        (0.06, 0.00),
        (CABINET_DEPTH, 0.00),
        (CABINET_DEPTH, CABINET_HEIGHT),
        (0.56, CABINET_HEIGHT),
        (0.42, 1.60),
        (0.28, 1.08),
        (0.12, 1.00),
        (0.00, 0.90),
        (0.06, 0.82),
        (0.06, 0.00),
    ]
    inner_profile = [
        (0.12, 0.06),
        (CABINET_DEPTH - 0.04, 0.06),
        (CABINET_DEPTH - 0.04, CABINET_HEIGHT - 0.05),
        (0.58, CABINET_HEIGHT - 0.05),
        (0.46, 1.56),
        (0.35, 1.18),
        (0.21, 1.11),
        (0.12, 0.98),
        (0.16, 0.86),
        (0.16, 0.06),
    ]

    shell = (
        cq.Workplane("YZ")
        .polyline(outer_profile)
        .close()
        .extrude(CABINET_WIDTH / 2.0, both=True)
        .cut(
            cq.Workplane("YZ")
            .polyline(inner_profile)
            .close()
            .extrude((CABINET_WIDTH - 2.0 * SIDE_CLEAR) / 2.0, both=True)
        )
    )

    service_opening = cq.Workplane("XY").box(
        DOOR_WIDTH + 0.018,
        0.14,
        DOOR_HEIGHT + 0.024,
    ).translate((0.0, 0.072, DOOR_BOTTOM + DOOR_HEIGHT / 2.0))

    monitor_opening = (
        cq.Workplane("XY")
        .box(0.50, 0.18, 0.38)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -MONITOR_TILT_DEG)
        .translate((0.0, MONITOR_CENTER_Y, MONITOR_CENTER_Z))
    )

    return shell.cut(service_opening).cut(monitor_opening)


def _monitor_bezel_shape():
    frame = (
        cq.Workplane("XY")
        .box(0.58, 0.045, 0.46)
        .cut(cq.Workplane("XY").box(0.48, 0.12, 0.34))
    )
    visor = cq.Workplane("XY").box(0.64, 0.04, 0.05).translate((0.0, -0.02, 0.255))
    return frame.union(visor)


def _control_deck_shape():
    deck = cq.Workplane("XY").box(DECK_WIDTH, DECK_DEPTH, DECK_THICKNESS).translate(
        (0.0, -DECK_DEPTH / 2.0, -DECK_THICKNESS / 2.0)
    )
    front_apron = cq.Workplane("XY").box(DECK_WIDTH, 0.024, 0.075).translate(
        (0.0, -DECK_DEPTH + 0.012, -0.0375)
    )
    joystick_opening = (
        cq.Workplane("XY")
        .circle(0.019)
        .extrude(0.12)
        .translate((JOYSTICK_X, JOYSTICK_Y, -0.12))
    )
    return deck.union(front_apron).cut(joystick_opening)


def _stick_support_shape():
    top_ring = (
        cq.Workplane("XY")
        .circle(0.028)
        .circle(0.0185)
        .extrude(0.006)
        .translate((JOYSTICK_X, JOYSTICK_Y, -0.003))
    )
    lower_collar = (
        cq.Workplane("XY")
        .circle(0.022)
        .circle(0.013)
        .extrude(0.028)
        .translate((JOYSTICK_X, JOYSTICK_Y, -0.030))
    )
    return top_ring.union(lower_collar)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) / 2.0 for i in range(3))


def _roll_yz(y_value: float, z_value: float, angle_deg: float) -> tuple[float, float]:
    angle = math.radians(angle_deg)
    return (
        y_value * math.cos(angle) - z_value * math.sin(angle),
        y_value * math.sin(angle) + z_value * math.cos(angle),
    )


def _deck_world(x_value: float, y_value: float, z_value: float = 0.0) -> tuple[float, float, float]:
    dy, dz = _roll_yz(y_value, z_value, DECK_TILT_DEG)
    return (x_value, DECK_MOUNT_Y + dy, DECK_MOUNT_Z + dz)


def _monitor_world(x_value: float, y_value: float, z_value: float = 0.0) -> tuple[float, float, float]:
    dy, dz = _roll_yz(y_value, z_value, -MONITOR_TILT_DEG)
    return (x_value, MONITOR_CENTER_Y + dy, MONITOR_CENTER_Z + dz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_arcade_cabinet")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.16, 0.02, 0.02, 1.0))
    deck_finish = model.material("deck_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.60, 0.61, 0.64, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.03, 0.04, 0.05, 1.0))
    button_red = model.material("button_red", rgba=(0.76, 0.10, 0.12, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.88, 0.72, 0.12, 1.0))
    button_blue = model.material("button_blue", rgba=(0.18, 0.34, 0.78, 1.0))
    button_green = model.material("button_green", rgba=(0.16, 0.55, 0.22, 1.0))
    ball_finish = model.material("ball_finish", rgba=(0.75, 0.08, 0.10, 1.0))
    coin_finish = model.material("coin_finish", rgba=(0.25, 0.26, 0.28, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "arcade_body"),
        material=cabinet_finish,
        name="shell",
    )
    body.visual(
        Box((0.30, 0.028, 0.18)),
        origin=Origin(xyz=(0.0, 0.074, 0.68)),
        material=coin_finish,
        name="coin_panel",
    )
    body.visual(
        Box((0.055, 0.010, 0.015)),
        origin=Origin(xyz=(-0.075, 0.063, 0.74)),
        material=metal_finish,
        name="coin_slot_0",
    )
    body.visual(
        Box((0.055, 0.010, 0.015)),
        origin=Origin(xyz=(0.075, 0.063, 0.74)),
        material=metal_finish,
        name="coin_slot_1",
    )

    body.visual(
        mesh_from_cadquery(_monitor_bezel_shape(), "monitor_bezel"),
        origin=Origin(
            xyz=_monitor_world(0.0, 0.0, 0.0),
            rpy=(math.radians(-MONITOR_TILT_DEG), 0.0, 0.0),
        ),
        material=trim_finish,
        name="bezel",
    )
    body.visual(
        Box((0.50, 0.018, 0.38)),
        origin=Origin(
            xyz=_monitor_world(0.0, 0.016, 0.0),
            rpy=(math.radians(-MONITOR_TILT_DEG), 0.0, 0.0),
        ),
        material=screen_finish,
        name="screen",
    )

    body.visual(
        mesh_from_cadquery(_control_deck_shape(), "control_deck"),
        origin=Origin(
            xyz=_deck_world(0.0, 0.0, 0.0),
            rpy=(math.radians(DECK_TILT_DEG), 0.0, 0.0),
        ),
        material=deck_finish,
        name="deck_shell",
    )
    body.visual(
        mesh_from_cadquery(_stick_support_shape(), "stick_support"),
        origin=Origin(
            xyz=_deck_world(0.0, 0.0, 0.0),
            rpy=(math.radians(DECK_TILT_DEG), 0.0, 0.0),
        ),
        material=dark_metal,
        name="stick_support",
    )

    button_positions = [
        ((0.11, -0.18), button_red),
        ((0.17, -0.15), button_yellow),
        ((0.23, -0.12), button_blue),
        ((0.29, -0.09), button_green),
    ]
    for index, ((x_pos, y_pos), cap_material) in enumerate(button_positions):
        body.visual(
            Cylinder(radius=0.014, length=0.028),
            origin=Origin(
                xyz=_deck_world(x_pos, y_pos, -0.008),
                rpy=(math.radians(DECK_TILT_DEG), 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"button_barrel_{index}",
        )
        body.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(
                xyz=_deck_world(x_pos, y_pos, 0.006),
                rpy=(math.radians(DECK_TILT_DEG), 0.0, 0.0),
            ),
            material=cap_material,
            name=f"button_cap_{index}",
        )

    door = model.part("door")
    door.visual(
        Box((DOOR_WIDTH + 0.026, 0.006, DOOR_HEIGHT + 0.030)),
        origin=Origin(
            xyz=((DOOR_WIDTH + 0.026) / 2.0, -0.003, (DOOR_HEIGHT + 0.030) / 2.0),
        ),
        material=trim_finish,
        name="door_flange",
    )
    door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0)),
        material=cabinet_finish,
        name="door_panel",
    )
    door.visual(
        Box((0.026, 0.012, 0.10)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.050, -0.006, 0.24)),
        material=metal_finish,
        name="door_pull",
    )
    door.visual(
        Box((0.018, 0.008, 0.018)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.030, -0.004, 0.35)),
        material=metal_finish,
        name="door_lock",
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, 0.061, DOOR_BOTTOM)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=12.0, velocity=1.2),
    )

    stick = model.part("stick")
    stick.visual(
        Sphere(radius=0.010),
        material=dark_metal,
        name="hub",
    )
    stick.visual(
        Cylinder(radius=0.0132, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=dark_metal,
        name="pivot_boss",
    )
    stick.visual(
        Cylinder(radius=0.008, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=metal_finish,
        name="shaft",
    )
    stick.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=ball_finish,
        name="ball_top",
    )
    stick.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=dark_metal,
        name="actuator",
    )
    model.articulation(
        "body_to_stick",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stick,
        origin=Origin(
            xyz=_deck_world(JOYSTICK_X, JOYSTICK_Y, JOYSTICK_PIVOT_Z),
            rpy=(math.radians(DECK_TILT_DEG), 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.35, effort=4.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    stick = object_model.get_part("stick")

    ctx.allow_overlap(
        body,
        door,
        elem_a="shell",
        elem_b="door_flange",
        reason="The service door uses a thin overlay flange that seats over the cabinet opening frame.",
    )
    ctx.allow_isolated_part(
        stick,
        reason="The joystick uses a simplified hidden pivot inside the deck support collar, so the visible stick remains intentionally clearanced from the body geometry.",
    )

    door_hinge = object_model.get_articulation("body_to_door")
    stick_joint = object_model.get_articulation("body_to_stick")

    body_aabb = ctx.part_element_world_aabb(body, elem="shell")
    deck_aabb = ctx.part_element_world_aabb(body, elem="deck_shell")
    ctx.check(
        "control deck projects in front of cabinet",
        body_aabb is not None
        and deck_aabb is not None
        and float(deck_aabb[0][1]) < float(body_aabb[0][1]) - 0.10,
        details=f"body_aabb={body_aabb!r}, deck_aabb={deck_aabb!r}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.25,
            name="service door covers lower opening",
        )
        door_closed_aabb = ctx.part_world_aabb(door)

    with ctx.pose({door_hinge: 1.0}):
        door_open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "service door swings outward on side hinge",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and float(door_open_aabb[0][1]) < float(door_closed_aabb[0][1]) - 0.12,
        details=f"closed={door_closed_aabb!r}, open={door_open_aabb!r}",
    )

    ball_rest = _aabb_center(ctx.part_element_world_aabb(stick, elem="ball_top"))
    with ctx.pose({stick_joint: 0.24}):
        ball_tilt = _aabb_center(ctx.part_element_world_aabb(stick, elem="ball_top"))

    displacement = None
    if ball_rest is not None and ball_tilt is not None:
        displacement = math.dist(ball_rest, ball_tilt)

    ctx.check(
        "joystick tilts off center",
        ball_rest is not None
        and ball_tilt is not None
        and displacement is not None
        and displacement > 0.05
        and ball_tilt[2] < ball_rest[2] - 0.01,
        details=f"rest={ball_rest!r}, tilt={ball_tilt!r}, displacement={displacement!r}",
    )

    return ctx.report()


object_model = build_object_model()
