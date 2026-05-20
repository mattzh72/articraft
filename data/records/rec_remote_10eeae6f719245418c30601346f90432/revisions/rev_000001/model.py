from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.047
BODY_T = 0.0165
BODY_L = 0.146
FRONT_Y = BODY_T * 0.5
BACK_Y = -BODY_T * 0.5

SLIDER_SLOT_W = 0.0105
SLIDER_SLOT_L = 0.046
SLIDER_TRACK_DEPTH = 0.0078
SLIDER_TRAVEL = 0.028
SLIDER_SLOT_CENTER_Z = 0.020
SLIDER_REST_Z = SLIDER_SLOT_CENTER_Z - SLIDER_TRAVEL * 0.5
SLIDER_TAB_W = 0.013
SLIDER_TAB_T = 0.0042
SLIDER_TAB_L = 0.012
SLIDER_TAB_Y = FRONT_Y - 0.00155
SLIDER_RUNNER_W = 0.0064
SLIDER_RUNNER_T = 0.0046
SLIDER_RUNNER_L = 0.018
SLIDER_RUNNER_Y = -0.0041

BUTTON_CAP_W = 0.015
BUTTON_CAP_T = 0.003
BUTTON_CAP_L = 0.011
BUTTON_CAP_Y = FRONT_Y - 0.0015
BUTTON_STEM_W = 0.0092
BUTTON_STEM_T = 0.0048
BUTTON_STEM_L = 0.0068
BUTTON_STEM_Y = -0.0022
BUTTON_TRAVEL = 0.0018
BUTTON_POSITIONS = (
    (-0.0115, -0.006),
    (0.0115, -0.006),
    (-0.0115, -0.029),
    (0.0115, -0.029),
)

DOOR_W = 0.032
DOOR_T = 0.0018
DOOR_L = 0.062
DOOR_REST_Z = -0.005
DOOR_TRAVEL = 0.032
DOOR_TRACK_DEPTH = 0.0026
DOOR_Y = BACK_Y + DOOR_TRACK_DEPTH - DOOR_T * 0.5


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_W, BODY_T, BODY_L).edges().fillet(0.0032)

    slider_slot = (
        cq.Workplane("XY")
        .box(SLIDER_SLOT_W, BODY_T * 1.25, SLIDER_SLOT_L)
        .translate((0.0, FRONT_Y - BODY_T * 0.18, SLIDER_SLOT_CENTER_Z))
    )
    slider_channel = (
        cq.Workplane("XY")
        .box(SLIDER_SLOT_W + 0.005, SLIDER_TRACK_DEPTH, SLIDER_RUNNER_L + 0.010)
        .translate((0.0, FRONT_Y - SLIDER_TRACK_DEPTH * 0.5, SLIDER_SLOT_CENTER_Z))
    )
    body = body.cut(slider_slot).cut(slider_channel)

    for button_x, button_z in BUTTON_POSITIONS:
        button_recess = (
            cq.Workplane("XY")
            .box(BUTTON_CAP_W + 0.003, 0.0026, BUTTON_CAP_L + 0.003)
            .translate((button_x, FRONT_Y - 0.0013, button_z))
        )
        button_throat = (
            cq.Workplane("XY")
            .box(BUTTON_STEM_W + 0.0012, BODY_T * 0.72, BUTTON_STEM_L + 0.0012)
            .translate((button_x, FRONT_Y - BODY_T * 0.22, button_z))
        )
        body = body.cut(button_recess).cut(button_throat)

    battery_track = (
        cq.Workplane("XY")
        .box(DOOR_W + 0.004, DOOR_TRACK_DEPTH, DOOR_L + DOOR_TRAVEL + 0.008)
        .translate((0.0, BACK_Y + DOOR_TRACK_DEPTH * 0.5, DOOR_REST_Z - DOOR_TRAVEL * 0.5))
    )
    battery_cavity = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.004, 0.009, 0.070)
        .translate((0.0, BACK_Y + 0.0045, -0.001))
    )
    finger_notch = (
        cq.Workplane("XY")
        .box(0.016, DOOR_TRACK_DEPTH + 0.0008, 0.010)
        .translate((0.0, BACK_Y + DOOR_TRACK_DEPTH * 0.5, -0.050))
    )
    body = body.cut(battery_track).cut(battery_cavity).cut(finger_notch)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_fan_remote")

    shell_white = model.material("shell_white", rgba=(0.95, 0.95, 0.93, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.80, 0.82, 0.84, 1.0))
    control_grey = model.material("control_grey", rgba=(0.73, 0.76, 0.79, 1.0))
    slider_grey = model.material("slider_grey", rgba=(0.50, 0.54, 0.58, 1.0))
    smoke = model.material("smoke", rgba=(0.20, 0.24, 0.28, 0.65))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "remote_body"),
        material=shell_white,
        name="shell",
    )
    body.visual(
        Box((0.016, 0.0012, 0.008)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0004, 0.056)),
        material=smoke,
        name="ir_window",
    )

    speed_slider = model.part("speed_slider")
    speed_slider.visual(
        Box((SLIDER_RUNNER_W, SLIDER_RUNNER_T, SLIDER_RUNNER_L)),
        origin=Origin(xyz=(0.0, SLIDER_RUNNER_Y, 0.0)),
        material=slider_grey,
        name="slider_runner",
    )
    speed_slider.visual(
        Box((SLIDER_TAB_W, SLIDER_TAB_T, SLIDER_TAB_L)),
        origin=Origin(),
        material=slider_grey,
        name="slider_tab",
    )
    for ridge_index, ridge_z in enumerate((-0.0032, 0.0, 0.0032)):
        speed_slider.visual(
            Box((0.0085, 0.0010, 0.0011)),
            origin=Origin(xyz=(0.0, 0.0016, ridge_z)),
            material=trim_grey,
            name=f"slider_ridge_{ridge_index}",
        )

    model.articulation(
        "body_to_speed_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=speed_slider,
        origin=Origin(xyz=(0.0, SLIDER_TAB_Y, SLIDER_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.12,
            lower=0.0,
            upper=SLIDER_TRAVEL,
        ),
    )

    for button_index, (button_x, button_z) in enumerate(BUTTON_POSITIONS):
        button = model.part(f"button_{button_index}")
        button.visual(
            Box((BUTTON_STEM_W, BUTTON_STEM_T, BUTTON_STEM_L)),
            origin=Origin(xyz=(0.0, BUTTON_STEM_Y, 0.0)),
            material=control_grey,
            name="button_stem",
        )
        button.visual(
            Box((BUTTON_CAP_W, BUTTON_CAP_T, BUTTON_CAP_L)),
            origin=Origin(),
            material=control_grey,
            name="button_cap",
        )

        model.articulation(
            f"body_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_CAP_Y, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((DOOR_W, DOOR_T, DOOR_L)),
        origin=Origin(),
        material=shell_white,
        name="door_panel",
    )
    battery_door.visual(
        Box((0.014, 0.0024, 0.007)),
        origin=Origin(xyz=(0.0, -0.0002, -0.023)),
        material=trim_grey,
        name="door_grip",
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.PRISMATIC,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.0, DOOR_Y, DOOR_REST_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.10,
            lower=0.0,
            upper=DOOR_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    speed_slider = object_model.get_part("speed_slider")
    slider_joint = object_model.get_articulation("body_to_speed_slider")
    battery_door = object_model.get_part("battery_door")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.expect_overlap(
        speed_slider,
        body,
        axes="x",
        min_overlap=0.007,
        name="speed slider stays centered in the remote width",
    )
    ctx.expect_overlap(
        battery_door,
        body,
        axes="x",
        min_overlap=0.028,
        name="battery door stays aligned to the back shell width",
    )

    slider_rest = ctx.part_world_position(speed_slider)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        ctx.expect_overlap(
            speed_slider,
            body,
            axes="x",
            min_overlap=0.007,
            name="speed slider remains centered when moved to high speed",
        )
        slider_high = ctx.part_world_position(speed_slider)
    ctx.check(
        "speed slider travels upward in its slot",
        slider_rest is not None and slider_high is not None and slider_high[2] > slider_rest[2] + 0.020,
        details=f"rest={slider_rest}, high={slider_high}",
    )

    door_rest = ctx.part_world_position(battery_door)
    with ctx.pose({door_joint: DOOR_TRAVEL}):
        ctx.expect_overlap(
            battery_door,
            body,
            axes="x",
            min_overlap=0.028,
            name="battery door stays centered when opened",
        )
        door_open = ctx.part_world_position(battery_door)
    ctx.check(
        "battery door slides downward on the back shell",
        door_rest is not None and door_open is not None and door_open[2] < door_rest[2] - 0.020,
        details=f"rest={door_rest}, open={door_open}",
    )

    button_parts = [object_model.get_part(f"button_{index}") for index in range(len(BUTTON_POSITIONS))]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(len(BUTTON_POSITIONS))]
    button_rest_positions = [ctx.part_world_position(button) for button in button_parts]

    for button_index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_position = ctx.part_world_position(button)
            ctx.check(
                f"{button.name} depresses inward",
                button_rest_positions[button_index] is not None
                and pressed_position is not None
                and pressed_position[1] < button_rest_positions[button_index][1] - 0.001,
                details=f"rest={button_rest_positions[button_index]}, pressed={pressed_position}",
            )

    with ctx.pose({button_joints[0]: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_position(button_parts[0])
        button_1_rest = ctx.part_world_position(button_parts[1])
    ctx.check(
        "front buttons articulate independently",
        button_rest_positions[0] is not None
        and button_rest_positions[1] is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_0_pressed[1] < button_rest_positions[0][1] - 0.001
        and abs(button_1_rest[1] - button_rest_positions[1][1]) <= 1e-6,
        details=(
            f"button_0_rest={button_rest_positions[0]}, "
            f"button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_rest_positions[1]}, "
            f"button_1_pose={button_1_rest}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
