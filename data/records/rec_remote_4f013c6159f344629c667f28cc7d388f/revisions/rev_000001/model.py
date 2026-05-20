from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.145
BODY_WIDTH = 0.050
BODY_HEIGHT = 0.016
CONTROL_DECK_LENGTH = 0.094
CONTROL_DECK_WIDTH = 0.029
CONTROL_DECK_HEIGHT = 0.0015
WHEEL_HEIGHT = 0.0062
BUTTON_HEIGHT = 0.0024
DOOR_THICKNESS = 0.0022


def _body_shell_mesh():
    body_shape = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.0046)
        .edges("<Z")
        .fillet(0.0036)
    )
    return mesh_from_cadquery(body_shape, "bedside_remote_body_shell")


def _control_deck_mesh():
    deck_shape = (
        cq.Workplane("XY")
        .box(CONTROL_DECK_LENGTH, CONTROL_DECK_WIDTH, CONTROL_DECK_HEIGHT)
        .edges("|Z")
        .fillet(0.008)
    )
    return mesh_from_cadquery(deck_shape, "bedside_remote_control_deck")


def _button_cap_mesh():
    button_shape = (
        cq.Workplane("XY")
        .box(0.017, 0.010, BUTTON_HEIGHT)
        .edges("|Z")
        .fillet(0.0038)
        .edges(">Z")
        .fillet(0.0009)
    )
    return mesh_from_cadquery(button_shape, "bedside_remote_button_cap")


def _battery_door_mesh():
    door_shape = (
        cq.Workplane("XY")
        .box(0.056, 0.032, DOOR_THICKNESS)
        .edges("|Z")
        .fillet(0.0035)
        .edges(">Z")
        .fillet(0.0007)
        .edges("<Z")
        .fillet(0.0005)
    )
    return mesh_from_cadquery(door_shape, "bedside_remote_battery_door")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_lighting_remote")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.92, 1.0))
    deck_white = model.material("deck_white", rgba=(0.97, 0.97, 0.95, 1.0))
    wheel_grey = model.material("wheel_grey", rgba=(0.69, 0.71, 0.72, 1.0))
    button_white = model.material("button_white", rgba=(0.98, 0.98, 0.97, 1.0))
    icon_grey = model.material("icon_grey", rgba=(0.58, 0.60, 0.62, 1.0))
    rail_grey = model.material("rail_grey", rgba=(0.82, 0.83, 0.82, 1.0))
    door_grey = model.material("door_grey", rgba=(0.86, 0.86, 0.85, 1.0))

    body_shell_mesh = _body_shell_mesh()
    control_deck_mesh = _control_deck_mesh()
    button_cap_mesh = _button_cap_mesh()
    battery_door_mesh = _battery_door_mesh()
    wheel_mesh = mesh_from_geometry(
        KnobGeometry(
            0.030,
            WHEEL_HEIGHT,
            body_style="tapered",
            top_diameter=0.027,
            base_diameter=0.031,
            edge_radius=0.0012,
            grip=KnobGrip(style="fluted", count=18, depth=0.0008),
        ),
        "bedside_remote_brightness_wheel",
    )

    body = model.part("body")
    body.visual(body_shell_mesh, material=body_white, name="shell")
    body.visual(
        control_deck_mesh,
        origin=Origin(xyz=(-0.006, 0.0, BODY_HEIGHT * 0.5 + CONTROL_DECK_HEIGHT * 0.5)),
        material=deck_white,
        name="control_deck",
    )
    body.visual(
        Box((0.068, 0.003, 0.0022)),
        origin=Origin(xyz=(0.002, 0.0185, -0.0091)),
        material=rail_grey,
        name="door_rail_0",
    )
    body.visual(
        Box((0.068, 0.003, 0.0022)),
        origin=Origin(xyz=(0.002, -0.0185, -0.0091)),
        material=rail_grey,
        name="door_rail_1",
    )
    body.visual(
        Box((0.003, 0.040, 0.0022)),
        origin=Origin(xyz=(-0.0305, 0.0, -0.0091)),
        material=rail_grey,
        name="door_stop",
    )

    brightness_wheel = model.part("brightness_wheel")
    brightness_wheel.visual(
        wheel_mesh,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_HEIGHT * 0.5)),
        material=wheel_grey,
        name="wheel",
    )
    brightness_wheel.visual(
        Box((0.004, 0.004, 0.0008)),
        origin=Origin(xyz=(0.0, 0.011, WHEEL_HEIGHT + 0.0004)),
        material=icon_grey,
        name="wheel_marker",
    )

    button_specs = [
        ("scene_button", -0.002, "scene_icon", (0.006, 0.0016, 0.0006)),
        ("power_button", -0.026, "power_icon", (0.005, 0.005, 0.0006)),
        ("timer_button", -0.050, "timer_icon", (0.007, 0.0016, 0.0006)),
    ]
    button_parts: dict[str, object] = {}
    button_joints: dict[str, object] = {}

    for part_name, x_pos, icon_name, icon_size in button_specs:
        button = model.part(part_name)
        button.visual(
            button_cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, BUTTON_HEIGHT * 0.5)),
            material=button_white,
            name="button_cap",
        )
        button.visual(
            Box(icon_size),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    BUTTON_HEIGHT + icon_size[2] * 0.5,
                )
            ),
            material=icon_grey,
            name=icon_name,
        )
        button_parts[part_name] = button
        button_joints[part_name] = model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(
                xyz=(x_pos, 0.0, BODY_HEIGHT * 0.5 + CONTROL_DECK_HEIGHT),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0016,
            ),
        )

    model.articulation(
        "body_to_brightness_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=brightness_wheel,
        origin=Origin(xyz=(0.040, 0.0, BODY_HEIGHT * 0.5 + CONTROL_DECK_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        battery_door_mesh,
        origin=Origin(xyz=(0.0, 0.0, -DOOR_THICKNESS * 0.5)),
        material=door_grey,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.018, 0.020, 0.0008)),
        origin=Origin(xyz=(0.012, 0.0, -DOOR_THICKNESS - 0.0004)),
        material=icon_grey,
        name="thumb_pad",
    )
    for index, rib_x in enumerate((-0.012, -0.002, 0.008)):
        battery_door.visual(
            Box((0.004, 0.018, 0.0005)),
            origin=Origin(xyz=(rib_x, 0.0, -DOOR_THICKNESS - 0.00025)),
            material=icon_grey,
            name=f"grip_rib_{index}",
        )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, 0.0, -BODY_HEIGHT * 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.020,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    brightness_wheel = object_model.get_part("brightness_wheel")
    battery_door = object_model.get_part("battery_door")
    scene_button = object_model.get_part("scene_button")
    power_button = object_model.get_part("power_button")
    timer_button = object_model.get_part("timer_button")

    wheel_joint = object_model.get_articulation("body_to_brightness_wheel")
    door_joint = object_model.get_articulation("body_to_battery_door")
    power_joint = object_model.get_articulation("body_to_power_button")
    scene_joint = object_model.get_articulation("body_to_scene_button")
    timer_joint = object_model.get_articulation("body_to_timer_button")

    ctx.expect_gap(
        brightness_wheel,
        body,
        axis="z",
        positive_elem="wheel",
        negative_elem="control_deck",
        max_gap=0.0002,
        max_penetration=1e-6,
        name="brightness wheel seats on the control deck",
    )
    ctx.expect_overlap(
        brightness_wheel,
        body,
        axes="xy",
        elem_a="wheel",
        elem_b="control_deck",
        min_overlap=0.015,
        name="brightness wheel sits within the top control area",
    )

    for button in (scene_button, power_button, timer_button):
        ctx.expect_gap(
            button,
            body,
            axis="z",
            positive_elem="button_cap",
            negative_elem="control_deck",
            max_gap=0.0002,
            max_penetration=1e-6,
            name=f"{button.name} sits on the control deck",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="xy",
            elem_a="button_cap",
            elem_b="control_deck",
            min_overlap=0.008,
            name=f"{button.name} stays inside the control deck footprint",
        )

    ctx.expect_gap(
        body,
        battery_door,
        axis="z",
        positive_elem="shell",
        negative_elem="door_panel",
        max_gap=0.0002,
        max_penetration=0.0,
        name="battery door closes flush against the underside",
    )
    ctx.expect_within(
        battery_door,
        body,
        axes="y",
        inner_elem="door_panel",
        outer_elem="shell",
        margin=0.001,
        name="battery door stays centered under the remote width",
    )
    ctx.expect_overlap(
        battery_door,
        body,
        axes="x",
        elem_a="door_panel",
        elem_b="shell",
        min_overlap=0.050,
        name="battery door remains retained in the underside guide at rest",
    )

    wheel_limits = wheel_joint.motion_limits
    ctx.check(
        "brightness wheel uses a continuous joint",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_limits is not None
        and wheel_limits.lower is None
        and wheel_limits.upper is None,
        details=f"type={wheel_joint.articulation_type}, limits={wheel_limits}",
    )

    wheel_rest_pos = ctx.part_world_position(brightness_wheel)
    with ctx.pose({wheel_joint: 1.8}):
        wheel_turned_pos = ctx.part_world_position(brightness_wheel)
    ctx.check(
        "brightness wheel rotates in place",
        wheel_rest_pos is not None
        and wheel_turned_pos is not None
        and all(abs(a - b) < 1e-8 for a, b in zip(wheel_rest_pos, wheel_turned_pos)),
        details=f"rest={wheel_rest_pos}, turned={wheel_turned_pos}",
    )

    button_rest_positions = {
        scene_button.name: ctx.part_world_position(scene_button),
        power_button.name: ctx.part_world_position(power_button),
        timer_button.name: ctx.part_world_position(timer_button),
    }
    with ctx.pose({power_joint: power_joint.motion_limits.upper}):
        power_pressed = ctx.part_world_position(power_button)
        scene_unmoved = ctx.part_world_position(scene_button)
        timer_unmoved = ctx.part_world_position(timer_button)
    ctx.check(
        "power button depresses independently",
        button_rest_positions["power_button"] is not None
        and power_pressed is not None
        and scene_unmoved is not None
        and timer_unmoved is not None
        and power_pressed[2] < button_rest_positions["power_button"][2] - 0.001
        and abs(scene_unmoved[2] - button_rest_positions["scene_button"][2]) < 1e-8
        and abs(timer_unmoved[2] - button_rest_positions["timer_button"][2]) < 1e-8,
        details=(
            f"rest={button_rest_positions}, pressed={power_pressed}, "
            f"scene={scene_unmoved}, timer={timer_unmoved}"
        ),
    )

    with ctx.pose({scene_joint: scene_joint.motion_limits.upper}):
        scene_pressed = ctx.part_world_position(scene_button)
        power_unmoved = ctx.part_world_position(power_button)
    ctx.check(
        "scene button depresses without moving the power button",
        button_rest_positions["scene_button"] is not None
        and scene_pressed is not None
        and power_unmoved is not None
        and scene_pressed[2] < button_rest_positions["scene_button"][2] - 0.001
        and abs(power_unmoved[2] - button_rest_positions["power_button"][2]) < 1e-8,
        details=f"scene={scene_pressed}, power={power_unmoved}",
    )

    door_rest_pos = ctx.part_world_position(battery_door)
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        ctx.expect_within(
            battery_door,
            body,
            axes="y",
            inner_elem="door_panel",
            outer_elem="shell",
            margin=0.001,
            name="battery door stays laterally aligned when opened",
        )
        ctx.expect_overlap(
            battery_door,
            body,
            axes="x",
            elem_a="door_panel",
            elem_b="shell",
            min_overlap=0.028,
            name="battery door keeps retained insertion when opened",
        )
        door_open_pos = ctx.part_world_position(battery_door)

    ctx.check(
        "battery door slides toward the nose of the remote",
        door_rest_pos is not None
        and door_open_pos is not None
        and door_open_pos[0] > door_rest_pos[0] + 0.015,
        details=f"rest={door_rest_pos}, open={door_open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
