from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_W = 0.098
BODY_D = 0.046
BODY_H = 0.184
FRONT_Y = BODY_D / 2.0
BACK_Y = -BODY_D / 2.0
BOTTOM_Z = -BODY_H / 2.0

DIAL_CENTER_Z = -0.006
DISPLAY_CENTER_Z = 0.054


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_digital_multimeter")

    holster = model.material("holster", rgba=(0.16, 0.17, 0.18, 1.0))
    shell = model.material("shell", rgba=(0.94, 0.74, 0.13, 1.0))
    panel = model.material("panel", rgba=(0.20, 0.22, 0.24, 1.0))
    glass = model.material("glass", rgba=(0.56, 0.73, 0.74, 0.35))
    lcd = model.material("lcd", rgba=(0.65, 0.80, 0.56, 0.95))
    dial_finish = model.material("dial_finish", rgba=(0.12, 0.12, 0.13, 1.0))
    stand_finish = model.material("stand_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    jack_dark = model.material("jack_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    button_finish = model.material("button_finish", rgba=(0.26, 0.28, 0.30, 1.0))
    rocker_finish = model.material("rocker_finish", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_H)),
        material=holster,
        name="holster_shell",
    )
    body.visual(
        Box((0.110, BODY_D, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=holster,
        name="top_bumper",
    )
    body.visual(
        Box((0.110, BODY_D, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=holster,
        name="bottom_bumper",
    )
    body.visual(
        Box((0.088, 0.038, 0.172)),
        material=shell,
        name="inner_shell",
    )
    body.visual(
        Box((0.082, 0.004, 0.160)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=panel,
        name="front_panel",
    )
    body.visual(
        Box((0.070, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, 0.020, DISPLAY_CENTER_Z)),
        material=panel,
        name="display_bezel",
    )
    body.visual(
        Box((0.058, 0.002, 0.036)),
        origin=Origin(xyz=(0.0, 0.024, DISPLAY_CENTER_Z)),
        material=glass,
        name="display_window",
    )
    body.visual(
        Box((0.052, 0.001, 0.028)),
        origin=Origin(xyz=(0.0, 0.0248, DISPLAY_CENTER_Z)),
        material=lcd,
        name="lcd_panel",
    )
    body.visual(
        Box((0.080, 0.008, 0.038)),
        origin=Origin(xyz=(0.0, 0.020, -0.108)),
        material=panel,
        name="jack_panel",
    )
    body.visual(
        Box((0.062, 0.003, 0.098)),
        origin=Origin(xyz=(0.0, BACK_Y + 0.0015, -0.020)),
        material=panel,
        name="battery_cover",
    )
    body.visual(
        Box((0.064, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.020, -0.058)),
        material=panel,
        name="button_strip",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(0.0, 0.020, DIAL_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=panel,
        name="dial_seat",
    )
    for x_pos in (-0.029, -0.010, 0.010, 0.029):
        body.visual(
            Cylinder(radius=0.0065, length=0.010),
            origin=Origin(xyz=(x_pos, 0.021, -0.108), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=jack_dark,
            name=f"jack_{int((x_pos + 0.03) * 1000):03d}",
        )

    for x_pos in (-0.018, 0.018):
        body.visual(
            Box((0.014, 0.007, 0.012)),
            origin=Origin(xyz=(x_pos, BACK_Y - 0.0015, BOTTOM_Z + 0.006)),
            material=panel,
            name=f"stand_lug_{0 if x_pos < 0.0 else 1}",
        )

    body.visual(
        Box((0.012, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.001, 0.030)),
        material=panel,
        name="dial_index",
    )

    range_dial = model.part("range_dial")
    range_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.020,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.066, 0.006, flare=0.10),
                grip=KnobGrip(style="fluted", count=16, depth=0.0014),
                indicator=KnobIndicator(
                    style="line",
                    mode="engraved",
                    depth=0.0008,
                    angle_deg=0.0,
                ),
                center=False,
            ),
            "multimeter_range_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_finish,
        name="dial_knob",
    )
    range_dial.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=panel,
        name="dial_hub",
    )

    mode_button_origins = (-0.024, 0.0, 0.024)
    for index, x_pos in enumerate(mode_button_origins):
        mode_button = model.part(f"mode_button_{index}")
        mode_button.visual(
            Box((0.018, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        mode_button.visual(
            Box((0.012, 0.0025, 0.004)),
            origin=Origin(xyz=(0.0, 0.005, 0.0)),
            material=glass,
            name="button_face",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=mode_button,
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.0010, -0.058)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.03,
                lower=0.0,
                upper=0.0015,
            ),
        )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Cylinder(radius=0.0022, length=0.014),
        origin=Origin(xyz=(0.0015, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=panel,
        name="rocker_pin",
    )
    power_rocker.visual(
        Box((0.004, 0.014, 0.024)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=rocker_finish,
        name="rocker_cap",
    )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.0035, length=0.042),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_finish,
        name="stand_barrel",
    )
    kickstand.visual(
        Box((0.015, 0.005, 0.114)),
        origin=Origin(xyz=(0.0, -0.0025, 0.057)),
        material=stand_finish,
        name="stand_leg",
    )
    kickstand.visual(
        Box((0.032, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.003, 0.108)),
        material=stand_finish,
        name="stand_foot",
    )

    model.articulation(
        "body_to_range_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=range_dial,
        origin=Origin(xyz=(0.0, FRONT_Y, DIAL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )
    model.articulation(
        "body_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=kickstand,
        origin=Origin(xyz=(0.0, BACK_Y, BOTTOM_Z + 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_rocker,
        origin=Origin(xyz=(BODY_W / 2.0, 0.002, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.32,
            upper=0.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    kickstand = object_model.get_part("kickstand")
    body = object_model.get_part("body")
    power_rocker = object_model.get_part("power_rocker")
    center_button = object_model.get_part("mode_button_1")
    stand_joint = object_model.get_articulation("body_to_kickstand")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    button_joints = [object_model.get_articulation(f"body_to_mode_button_{index}") for index in range(3)]
    buttons = [object_model.get_part(f"mode_button_{index}") for index in range(3)]

    ctx.expect_gap(
        body,
        kickstand,
        axis="y",
        positive_elem="front_panel",
        negative_elem="stand_leg",
        min_gap=0.038,
        name="kickstand stays on the back side of the meter",
    )

    stand_limits = stand_joint.motion_limits
    if stand_limits is not None and stand_limits.upper is not None:
        closed_foot = ctx.part_element_world_aabb(kickstand, elem="stand_foot")
        with ctx.pose({stand_joint: stand_limits.upper}):
            open_foot = ctx.part_element_world_aabb(kickstand, elem="stand_foot")
        ctx.check(
            "kickstand swings outward",
            closed_foot is not None
            and open_foot is not None
            and open_foot[0][1] < closed_foot[0][1] - 0.030,
            details=f"closed={closed_foot}, open={open_foot}",
        )

    center_button_aabb = ctx.part_world_aabb(center_button)
    jack_panel_aabb = ctx.part_element_world_aabb(body, elem="jack_panel")
    ctx.check(
        "buttons sit clearly above the input jacks",
        center_button_aabb is not None
        and jack_panel_aabb is not None
        and center_button_aabb[0][2] > jack_panel_aabb[1][2] + 0.020,
        details=f"button={center_button_aabb}, jack_panel={jack_panel_aabb}",
    )

    right_button_aabb = ctx.part_world_aabb(buttons[2])
    rocker_aabb = ctx.part_world_aabb(power_rocker)
    ctx.check(
        "side rocker stays separate from the front button bank",
        rocker_aabb is not None
        and right_button_aabb is not None
        and rocker_aabb[0][0] > right_button_aabb[1][0] + 0.010,
        details=f"rocker={rocker_aabb}, right_button={right_button_aabb}",
    )

    rest_positions = [ctx.part_world_position(button) for button in buttons]
    for index, joint in enumerate(button_joints):
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.upper}):
            pressed_positions = [ctx.part_world_position(button) for button in buttons]
        own_rest = rest_positions[index]
        own_pressed = pressed_positions[index]
        ctx.check(
            f"mode button {index} depresses inward",
            own_rest is not None
            and own_pressed is not None
            and own_pressed[1] < own_rest[1] - 0.001,
            details=f"rest={own_rest}, pressed={own_pressed}",
        )
        for other_index, other_rest in enumerate(rest_positions):
            if other_index == index:
                continue
            other_pressed = pressed_positions[other_index]
            ctx.check(
                f"mode button {index} does not drive button {other_index}",
                other_rest is not None
                and other_pressed is not None
                and abs(other_pressed[1] - other_rest[1]) < 1e-6,
                details=f"rest={other_rest}, pressed={other_pressed}",
            )

    rocker_limits = rocker_joint.motion_limits
    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        with ctx.pose({rocker_joint: rocker_limits.lower}):
            rocker_low = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            rocker_high = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
        ctx.check(
            "power rocker pivots through a visible angle",
            rocker_low is not None
            and rocker_high is not None
            and abs(rocker_high[0][2] - rocker_low[0][2]) > 0.001,
            details=f"low={rocker_low}, high={rocker_high}",
        )

    return ctx.report()


object_model = build_object_model()
