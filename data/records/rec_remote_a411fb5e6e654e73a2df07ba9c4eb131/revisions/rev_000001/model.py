from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_LENGTH = 0.205
BODY_WIDTH = 0.052
BODY_THICKNESS = 0.017
BODY_FACE_Z = BODY_THICKNESS * 0.5
BODY_BACK_Z = -BODY_FACE_Z

DOOR_LENGTH = 0.098
DOOR_WIDTH = 0.036
DOOR_THICKNESS = 0.0016
DOOR_RECESS_DEPTH = 0.0016
DOOR_TRAVEL = 0.032
DOOR_CENTER_X = -0.018
DOOR_CENTER_Z = BODY_BACK_Z + DOOR_THICKNESS * 0.5

ROCKER_LENGTH = 0.041
ROCKER_WIDTH = 0.0105
ROCKER_THICKNESS = 0.0032
ROCKER_TRAVEL = math.radians(11.0)
ROCKER_HINGE_X = 0.047
ROCKER_BAR_Y = 0.0105

BUTTON_CAP_THICKNESS = 0.0024
BUTTON_TRAVEL = 0.0012
BUTTON_CENTER_Z = BODY_FACE_Z + BUTTON_CAP_THICKNESS * 0.5

BUTTON_LAYOUT = (
    ("power_button", 0.066, 0.000, 0.0052),
    ("back_button", -0.024, -0.011, 0.0047),
    ("home_button", -0.024, 0.011, 0.0047),
    ("menu_button", -0.052, -0.011, 0.0047),
    ("ok_button", -0.052, 0.011, 0.0051),
    ("mute_button", -0.080, -0.011, 0.0047),
    ("play_button", -0.080, 0.011, 0.0047),
)


def _rounded_slab(length: float, width: float, thickness: float, corner_radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, thickness).edges("|Z").fillet(corner_radius)


def _build_body_shape() -> cq.Workplane:
    body = _rounded_slab(BODY_LENGTH, BODY_WIDTH, BODY_THICKNESS, 0.011)
    battery_recess = (
        _rounded_slab(DOOR_LENGTH + 0.004, DOOR_WIDTH + 0.004, DOOR_RECESS_DEPTH, 0.005)
        .translate((DOOR_CENTER_X, 0.0, BODY_BACK_Z + DOOR_RECESS_DEPTH * 0.5))
    )
    return body.cut(battery_recess)


def _build_battery_door_shape() -> cq.Workplane:
    door = _rounded_slab(DOOR_LENGTH, DOOR_WIDTH, DOOR_THICKNESS, 0.0045)
    grip_rib = cq.Workplane("XY").box(0.012, DOOR_WIDTH * 0.66, 0.0007).translate(
        (-DOOR_LENGTH * 0.32, 0.0, -DOOR_THICKNESS * 0.5 - 0.00035)
    )
    return door.union(grip_rib)


def _build_rocker_shape() -> cq.Workplane:
    return _rounded_slab(ROCKER_LENGTH, ROCKER_WIDTH, ROCKER_THICKNESS, 0.0022).translate(
        (-ROCKER_LENGTH * 0.5, 0.0, ROCKER_THICKNESS * 0.5)
    )


def _build_button_shape(cap_radius: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(cap_radius).extrude(BUTTON_CAP_THICKNESS).translate(
        (0.0, 0.0, -BUTTON_CAP_THICKNESS * 0.5)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="television_remote")

    housing = model.material("housing", rgba=(0.12, 0.12, 0.13, 1.0))
    control = model.material("control", rgba=(0.22, 0.22, 0.24, 1.0))
    power = model.material("power", rgba=(0.58, 0.10, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "remote_body"),
        material=housing,
        name="shell",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_cadquery(_build_battery_door_shape(), "battery_door"),
        material=housing,
        name="panel",
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(DOOR_CENTER_X, 0.0, DOOR_CENTER_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.12,
            lower=0.0,
            upper=DOOR_TRAVEL,
        ),
    )

    rocker_mesh = mesh_from_cadquery(_build_rocker_shape(), "remote_rocker")
    for part_name, y_pos in (("volume_bar", -ROCKER_BAR_Y), ("channel_bar", ROCKER_BAR_Y)):
        rocker = model.part(part_name)
        rocker.visual(rocker_mesh, material=control, name="bar")
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=rocker,
            origin=Origin(xyz=(ROCKER_HINGE_X, y_pos, BODY_FACE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.0,
                lower=0.0,
                upper=ROCKER_TRAVEL,
            ),
        )

    button_mesh_cache: dict[float, object] = {}
    for part_name, x_pos, y_pos, cap_radius in BUTTON_LAYOUT:
        button_mesh = button_mesh_cache.get(cap_radius)
        if button_mesh is None:
            mesh_name = f"button_{int(cap_radius * 10000)}"
            button_mesh = mesh_from_cadquery(_build_button_shape(cap_radius), mesh_name)
            button_mesh_cache[cap_radius] = button_mesh

        button = model.part(part_name)
        button.visual(
            button_mesh,
            material=power if part_name == "power_button" else control,
            name="cap",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, BUTTON_CENTER_Z)),
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
    body = object_model.get_part("body")
    battery_door = object_model.get_part("battery_door")
    door_joint = object_model.get_articulation("body_to_battery_door")
    volume_bar = object_model.get_part("volume_bar")
    channel_bar = object_model.get_part("channel_bar")
    volume_joint = object_model.get_articulation("body_to_volume_bar")
    channel_joint = object_model.get_articulation("body_to_channel_bar")
    power_button = object_model.get_part("power_button")
    home_button = object_model.get_part("home_button")
    ok_button = object_model.get_part("ok_button")
    power_joint = object_model.get_articulation("body_to_power_button")
    home_joint = object_model.get_articulation("body_to_home_button")
    ok_joint = object_model.get_articulation("body_to_ok_button")

    for button_name in (
        "power_button",
        "back_button",
        "home_button",
        "menu_button",
        "ok_button",
        "mute_button",
        "play_button",
    ):
        ctx.allow_overlap(
            body,
            object_model.get_part(button_name),
            reason="The front buttons are represented as external plungers over a simplified solid shell and intentionally depress into the shell volume in pressed poses.",
        )

    ctx.expect_within(
        battery_door,
        body,
        axes="yz",
        margin=0.010,
        name="battery door stays aligned within the back shell band",
    )

    closed_pos = ctx.part_world_position(battery_door)
    with ctx.pose({door_joint: DOOR_TRAVEL}):
        ctx.expect_within(
            battery_door,
            body,
            axes="yz",
            margin=0.010,
            name="battery door remains centered on the shell rails when opened",
        )
        open_pos = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door slides toward the tail",
        closed_pos is not None and open_pos is not None and open_pos[0] < closed_pos[0] - 0.02,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    rest_volume_aabb = ctx.part_world_aabb(volume_bar)
    rest_channel_aabb = ctx.part_world_aabb(channel_bar)
    with ctx.pose({volume_joint: ROCKER_TRAVEL}):
        tilted_volume_aabb = ctx.part_world_aabb(volume_bar)
    with ctx.pose({channel_joint: ROCKER_TRAVEL}):
        tilted_channel_aabb = ctx.part_world_aabb(channel_bar)

    ctx.check(
        "volume rocker pivots upward from the face hinge",
        rest_volume_aabb is not None
        and tilted_volume_aabb is not None
        and tilted_volume_aabb[1][2] > rest_volume_aabb[1][2] + 0.0004,
        details=f"rest={rest_volume_aabb}, tilted={tilted_volume_aabb}",
    )
    ctx.check(
        "channel rocker pivots independently on its own hinge",
        rest_channel_aabb is not None
        and tilted_channel_aabb is not None
        and tilted_channel_aabb[1][2] > rest_channel_aabb[1][2] + 0.0004,
        details=f"rest={rest_channel_aabb}, tilted={tilted_channel_aabb}",
    )

    power_rest = ctx.part_world_position(power_button)
    home_rest = ctx.part_world_position(home_button)
    ok_rest = ctx.part_world_position(ok_button)
    with ctx.pose({power_joint: BUTTON_TRAVEL}):
        power_pressed = ctx.part_world_position(power_button)
        home_during_power = ctx.part_world_position(home_button)
    with ctx.pose({ok_joint: BUTTON_TRAVEL, home_joint: BUTTON_TRAVEL}):
        ok_pressed = ctx.part_world_position(ok_button)
        home_pressed = ctx.part_world_position(home_button)

    ctx.check(
        "power button depresses inward without moving neighboring buttons",
        power_rest is not None
        and power_pressed is not None
        and home_rest is not None
        and home_during_power is not None
        and power_pressed[2] < power_rest[2] - 0.001
        and abs(home_during_power[2] - home_rest[2]) < 1e-6,
        details=(
            f"power_rest={power_rest}, power_pressed={power_pressed}, "
            f"home_rest={home_rest}, home_during_power={home_during_power}"
        ),
    )
    ctx.check(
        "small front buttons depress independently",
        ok_rest is not None
        and ok_pressed is not None
        and home_rest is not None
        and home_pressed is not None
        and ok_pressed[2] < ok_rest[2] - 0.001
        and home_pressed[2] < home_rest[2] - 0.001,
        details=f"ok_rest={ok_rest}, ok_pressed={ok_pressed}, home_rest={home_rest}, home_pressed={home_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
