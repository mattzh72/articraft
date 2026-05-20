from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_FRONT_Y = 0.023
BODY_BACK_Y = -0.023


def _body_shell_mesh():
    upper = cq.Workplane("XY").box(0.088, 0.046, 0.106).translate((0.0, 0.0, 0.023))
    lower = cq.Workplane("XY").box(0.100, 0.046, 0.082).translate((0.0, 0.0, -0.071))
    blend = cq.Workplane("XY").box(0.094, 0.046, 0.020).translate((0.0, 0.0, -0.030))

    shell = upper.union(lower).union(blend)

    front_recess = cq.Workplane("XY").box(0.074, 0.008, 0.144).translate((0.0, 0.019, -0.010))
    back_label_flat = cq.Workplane("XY").box(0.066, 0.004, 0.100).translate((0.0, -0.021, -0.018))
    finger_scallop = cq.Workplane("XY").box(0.044, 0.010, 0.008).translate((0.0, 0.020, -0.052))

    shell = shell.cut(front_recess)
    shell = shell.cut(back_label_flat)
    shell = shell.cut(finger_scallop)
    return shell


def _stand_mesh():
    panel = cq.Workplane("XY").box(0.072, 0.006, 0.100).translate((0.0, -0.003, 0.050))
    foot = cq.Workplane("XY").box(0.056, 0.012, 0.012).translate((0.0, -0.006, 0.010))
    grip_cut = cq.Workplane("XY").box(0.028, 0.010, 0.050).translate((0.0, -0.003, 0.060))

    return panel.union(foot).cut(grip_cut)


def _dial_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.048,
            0.017,
            body_style="skirted",
            top_diameter=0.038,
            skirt=KnobSkirt(0.058, 0.0045, flare=0.06),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "multimeter_range_dial",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_digital_multimeter")

    bumper = model.material("bumper", rgba=(0.84, 0.29, 0.13, 1.0))
    housing = model.material("housing", rgba=(0.13, 0.14, 0.15, 1.0))
    panel = model.material("panel", rgba=(0.07, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.20, 0.33, 0.38, 0.55))
    dial_dark = model.material("dial_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    accent = model.material("accent", rgba=(0.86, 0.73, 0.19, 1.0))
    button_dark = model.material("button_dark", rgba=(0.21, 0.22, 0.23, 1.0))
    stand_dark = model.material("stand_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    jack_black = model.material("jack_black", rgba=(0.05, 0.05, 0.05, 1.0))
    jack_red = model.material("jack_red", rgba=(0.66, 0.08, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_mesh(), "multimeter_body_shell"),
        material=bumper,
        name="shell",
    )
    body.visual(
        Box((0.072, 0.006, 0.142)),
        origin=Origin(xyz=(0.0, 0.020, -0.010)),
        material=housing,
        name="front_panel",
    )
    body.visual(
        Box((0.058, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, 0.021, 0.056)),
        material=panel,
        name="screen_bezel",
    )
    body.visual(
        Box((0.050, 0.002, 0.024)),
        origin=Origin(xyz=(0.0, 0.022, 0.056)),
        material=glass,
        name="screen",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.003),
        origin=Origin(xyz=(0.0, 0.0215, -0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing,
        name="dial_seat",
    )
    body.visual(
        Box((0.070, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.0195, 0.024)),
        material=panel,
        name="button_strip",
    )
    body.visual(
        Box((0.076, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, 0.0195, -0.083)),
        material=panel,
        name="jack_plate",
    )

    jack_x_positions = (-0.031, -0.010, 0.011, 0.032)
    jack_materials = (jack_black, jack_red, jack_red, jack_red)
    for index, (jack_x, jack_material) in enumerate(zip(jack_x_positions, jack_materials)):
        body.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(xyz=(jack_x, 0.021, -0.083), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=jack_material,
            name=f"jack_{index}",
        )

    dial = model.part("dial")
    dial.visual(
        _dial_mesh(),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_dark,
        name="knob",
    )
    dial.visual(
        Box((0.004, 0.0016, 0.020)),
        origin=Origin(xyz=(0.0, 0.0215, 0.010)),
        material=accent,
        name="pointer",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, -0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_mesh(), "multimeter_stand"),
        material=stand_dark,
        name="support_panel",
    )

    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, BODY_BACK_Y, -0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=0.0,
            upper=1.05,
        ),
    )

    button_specs = (
        ("hold_button", -0.028),
        ("range_button", -0.009),
        ("minmax_button", 0.010),
        ("backlight_button", 0.029),
    )
    for button_name, x_pos in button_specs:
        button = model.part(button_name)
        button.visual(
            Box((0.015, 0.0026, 0.009)),
            origin=Origin(xyz=(0.0, 0.0013, 0.0)),
            material=button_dark,
            name="cap",
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0215, 0.024)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0016,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")
    dial_joint = object_model.get_articulation("body_to_dial")
    stand_joint = object_model.get_articulation("body_to_stand")

    button_names = (
        "hold_button",
        "range_button",
        "minmax_button",
        "backlight_button",
    )
    button_parts = [object_model.get_part(name) for name in button_names]
    button_joints = {
        name: object_model.get_articulation(f"body_to_{name}")
        for name in button_names
    }

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        body_size = tuple(body_aabb[1][index] - body_aabb[0][index] for index in range(3))
        ctx.check(
            "body stays handheld tool scale",
            0.08 <= body_size[0] <= 0.12 and 0.04 <= body_size[1] <= 0.06 and 0.17 <= body_size[2] <= 0.21,
            details=f"body_size={body_size}",
        )

    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a="knob",
        elem_b="dial_seat",
        min_overlap=0.055,
        name="dial sits centered on the front seat",
    )
    with ctx.pose({dial_joint: 1.35}):
        ctx.expect_overlap(
            dial,
            body,
            axes="xz",
            elem_a="knob",
            elem_b="dial_seat",
            min_overlap=0.055,
            name="dial remains seated while rotating",
        )

    ctx.expect_gap(
        body,
        stand,
        axis="y",
        positive_elem="shell",
        negative_elem="support_panel",
        max_gap=0.0005,
        max_penetration=0.0,
        name="stand rests against the lower back when closed",
    )
    rest_stand_aabb = ctx.part_world_aabb(stand)
    stand_upper = stand_joint.motion_limits.upper if stand_joint.motion_limits is not None else None
    if rest_stand_aabb is not None and stand_upper is not None:
        with ctx.pose({stand_joint: stand_upper}):
            open_stand_aabb = ctx.part_world_aabb(stand)
        ctx.check(
            "stand swings backward into a support pose",
            open_stand_aabb is not None
            and open_stand_aabb[0][1] < rest_stand_aabb[0][1] - 0.045
            and open_stand_aabb[1][2] < rest_stand_aabb[1][2] - 0.015,
            details=f"rest={rest_stand_aabb}, open={open_stand_aabb}",
        )

    for left_name, right_name in zip(button_names, button_names[1:]):
        ctx.expect_gap(
            right_name,
            left_name,
            axis="x",
            positive_elem="cap",
            negative_elem="cap",
            min_gap=0.003,
            name=f"{left_name} and {right_name} stay visually separate",
        )

    for button_name, button_part in zip(button_names, button_parts):
        ctx.expect_gap(
            button_part,
            body,
            axis="y",
            positive_elem="cap",
            negative_elem="button_strip",
            max_gap=0.003,
            max_penetration=0.0,
            name=f"{button_name} sits on the front button strip",
        )
        ctx.expect_gap(
            body,
            button_part,
            axis="z",
            positive_elem="screen",
            negative_elem="cap",
            min_gap=0.015,
            name=f"{button_name} stays below the screen",
        )
        rest_position = ctx.part_world_position(button_part)
        joint = button_joints[button_name]
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if rest_position is not None and upper is not None:
            with ctx.pose({joint: upper}):
                pressed_position = ctx.part_world_position(button_part)
            ctx.check(
                f"{button_name} presses inward",
                pressed_position is not None and pressed_position[1] < rest_position[1] - 0.001,
                details=f"rest={rest_position}, pressed={pressed_position}",
            )

    return ctx.report()


object_model = build_object_model()
