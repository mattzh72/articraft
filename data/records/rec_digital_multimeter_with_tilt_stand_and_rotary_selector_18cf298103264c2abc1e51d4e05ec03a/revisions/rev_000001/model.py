from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_HEIGHT = 0.196
BODY_DEPTH = 0.040
BUMPER_DEPTH = 0.046
FRONT_FACE_Y = BODY_DEPTH * 0.5
BACK_FACE_Y = -BODY_DEPTH * 0.5
BACK_BUMPER_Y = -BUMPER_DEPTH * 0.5

DIAL_CENTER_Z = 0.104
DIAL_FRONT_Y = FRONT_FACE_Y + 0.002

DISPLAY_CENTER_Z = 0.158
DISPLAY_FRONT_Y = FRONT_FACE_Y + 0.002

JACK_PANEL_CENTER_Z = 0.026
JACK_PANEL_FRONT_Y = FRONT_FACE_Y + 0.003

BUTTON_FRAME_CENTER_Z = 0.051
BUTTON_FRAME_CENTER_Y = FRONT_FACE_Y + 0.005
BUTTON_FRAME_FRONT_Y = BUTTON_FRAME_CENTER_Y + 0.003
BUTTON_STEM_BACK_Y = BUTTON_FRAME_FRONT_Y - 0.006

ROCKER_CENTER_Z = 0.141
ROCKER_MOUNT_X = 0.056
ROCKER_CENTER_Y = 0.004

STAND_HEIGHT = 0.114
STAND_THICKNESS = 0.005


def _housing_profile() -> list[tuple[float, float]]:
    return [
        (-0.041, BODY_HEIGHT),
        (-0.048, 0.191),
        (-0.051, 0.178),
        (-0.052, 0.140),
        (-0.053, 0.093),
        (-0.057, 0.052),
        (-0.057, 0.019),
        (-0.054, 0.004),
        (-0.047, 0.000),
        (0.047, 0.000),
        (0.054, 0.004),
        (0.057, 0.019),
        (0.057, 0.052),
        (0.053, 0.093),
        (0.052, 0.140),
        (0.051, 0.178),
        (0.048, 0.191),
        (0.041, BODY_HEIGHT),
    ]


def _bumper_outer_profile() -> list[tuple[float, float]]:
    return [
        (-0.045, BODY_HEIGHT + 0.001),
        (-0.053, 0.192),
        (-0.0565, 0.178),
        (-0.058, 0.138),
        (-0.060, 0.090),
        (-0.062, 0.048),
        (-0.062, 0.015),
        (-0.059, 0.000),
        (-0.051, -0.001),
        (0.051, -0.001),
        (0.059, 0.000),
        (0.062, 0.015),
        (0.062, 0.048),
        (0.060, 0.090),
        (0.058, 0.138),
        (0.0565, 0.178),
        (0.053, 0.192),
        (0.045, BODY_HEIGHT + 0.001),
    ]


def _bumper_inner_profile() -> list[tuple[float, float]]:
    return [
        (-0.034, BODY_HEIGHT - 0.010),
        (-0.040, 0.180),
        (-0.043, 0.167),
        (-0.044, 0.134),
        (-0.045, 0.093),
        (-0.048, 0.055),
        (-0.048, 0.026),
        (-0.046, 0.012),
        (-0.041, 0.008),
        (0.041, 0.008),
        (0.046, 0.012),
        (0.048, 0.026),
        (0.048, 0.055),
        (0.045, 0.093),
        (0.044, 0.134),
        (0.043, 0.167),
        (0.040, 0.180),
        (0.034, BODY_HEIGHT - 0.010),
    ]


def _body_housing_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(_housing_profile())
        .close()
        .extrude(BODY_DEPTH)
        .translate((0.0, FRONT_FACE_Y, 0.0))
        .edges("|Y")
        .fillet(0.0045)
    )


def _bumper_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .polyline(_bumper_outer_profile())
        .close()
        .extrude(BUMPER_DEPTH)
        .translate((0.0, BUMPER_DEPTH * 0.5, 0.0))
    )
    inner = (
        cq.Workplane("XZ")
        .polyline(_bumper_inner_profile())
        .close()
        .extrude(BUMPER_DEPTH + 0.010)
        .translate((0.0, (BUMPER_DEPTH + 0.010) * 0.5, 0.0))
    )
    return outer.cut(inner)


def _stand_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.040, 0.000),
                (-0.040, 0.012),
                (-0.033, STAND_HEIGHT - 0.010),
                (-0.028, STAND_HEIGHT),
                (0.028, STAND_HEIGHT),
                (0.033, STAND_HEIGHT - 0.010),
                (0.040, 0.012),
                (0.040, 0.000),
            ]
        )
        .close()
        .extrude(STAND_THICKNESS)
        .translate((0.0, -STAND_THICKNESS, 0.0))
    )
    foot = cq.Workplane("XY").box(0.082, 0.009, 0.007).translate((0.0, -0.0045, 0.0035))
    rib = cq.Workplane("XY").box(0.015, 0.009, 0.074).translate((0.0, -0.0045, 0.046))
    relief = cq.Workplane("XY").box(0.026, 0.012, 0.050).translate((0.0, -0.006, 0.053))
    return panel.union(foot).union(rib).cut(relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_digital_multimeter")

    body_polymer = model.material("body_polymer", rgba=(0.18, 0.19, 0.20, 1.0))
    bumper_rubber = model.material("bumper_rubber", rgba=(0.92, 0.53, 0.11, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    stand_dark = model.material("stand_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    screen_tint = model.material("screen_tint", rgba=(0.34, 0.54, 0.62, 0.40))
    dial_dark = model.material("dial_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    jack_black = model.material("jack_black", rgba=(0.08, 0.08, 0.09, 1.0))
    jack_red = model.material("jack_red", rgba=(0.78, 0.10, 0.08, 1.0))
    jack_yellow = model.material("jack_yellow", rgba=(0.84, 0.69, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_housing_shape(), "multimeter_body"),
        material=body_polymer,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(_bumper_shape(), "multimeter_bumper"),
        material=bumper_rubber,
        name="bumper",
    )
    body.visual(
        Box((0.086, 0.004, 0.122)),
        origin=Origin(xyz=(0.000, FRONT_FACE_Y + 0.002, 0.124)),
        material=panel_dark,
        name="front_panel",
    )
    body.visual(
        Box((0.060, 0.004, 0.031)),
        origin=Origin(xyz=(0.000, DISPLAY_FRONT_Y, DISPLAY_CENTER_Z)),
        material=panel_dark,
        name="display_bezel",
    )
    body.visual(
        Box((0.054, 0.002, 0.024)),
        origin=Origin(xyz=(0.000, DISPLAY_FRONT_Y + 0.001, DISPLAY_CENTER_Z)),
        material=screen_tint,
        name="display_glass",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(
            xyz=(0.000, DIAL_FRONT_Y, DIAL_CENTER_Z),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=panel_dark,
        name="dial_well",
    )
    body.visual(
        Box((0.080, 0.006, 0.030)),
        origin=Origin(xyz=(0.000, JACK_PANEL_FRONT_Y, JACK_PANEL_CENTER_Z)),
        material=panel_dark,
        name="jack_panel",
    )
    body.visual(
        Box((0.086, 0.003, 0.118)),
        origin=Origin(xyz=(0.000, BACK_BUMPER_Y + 0.0015, 0.069)),
        material=panel_dark,
        name="rear_pad",
    )

    jack_offsets = (-0.034, -0.011, 0.012, 0.035)
    jack_materials = (jack_black, jack_black, jack_red, jack_yellow)
    for index, (x_pos, material) in enumerate(zip(jack_offsets, jack_materials)):
        body.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(
                xyz=(x_pos, JACK_PANEL_FRONT_Y + 0.001, 0.015),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=material,
            name=f"jack_{index}",
        )
        body.visual(
            Cylinder(radius=0.0048, length=0.003),
            origin=Origin(
                xyz=(x_pos, JACK_PANEL_FRONT_Y + 0.003, 0.015),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=panel_dark,
            name=f"jack_inner_{index}",
        )

    frame_y = BUTTON_FRAME_CENTER_Y
    body.visual(
        Box((0.074, 0.006, 0.003)),
        origin=Origin(xyz=(0.000, frame_y, BUTTON_FRAME_CENTER_Z + 0.009)),
        material=panel_dark,
        name="button_frame_top",
    )
    body.visual(
        Box((0.074, 0.006, 0.003)),
        origin=Origin(xyz=(0.000, frame_y, BUTTON_FRAME_CENTER_Z - 0.009)),
        material=panel_dark,
        name="button_frame_bottom",
    )
    for name, x_pos, width in (
        ("button_frame_left", -0.036, 0.004),
        ("button_frame_sep_0", -0.012, 0.003),
        ("button_frame_sep_1", 0.012, 0.003),
        ("button_frame_right", 0.036, 0.004),
    ):
        body.visual(
            Box((width, 0.006, 0.018)),
            origin=Origin(xyz=(x_pos, frame_y, BUTTON_FRAME_CENTER_Z)),
            material=panel_dark,
            name=name,
        )

    body.visual(
        Box((0.004, 0.020, 0.030)),
        origin=Origin(xyz=(ROCKER_MOUNT_X, ROCKER_CENTER_Y, ROCKER_CENTER_Z)),
        material=panel_dark,
        name="rocker_mount",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.120, 0.050, BODY_HEIGHT + 0.002)),
        mass=0.65,
        origin=Origin(xyz=(0.000, 0.000, BODY_HEIGHT * 0.5)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.029, length=0.004),
        origin=Origin(xyz=(0.000, 0.002, 0.000), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dial_dark,
        name="skirt",
    )
    dial.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.000, 0.008, 0.000), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dial_dark,
        name="cap",
    )
    dial.visual(
        Box((0.004, 0.003, 0.014)),
        origin=Origin(xyz=(0.000, 0.0165, 0.010)),
        material=bumper_rubber,
        name="pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.029, length=0.020),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.010, 0.000), rpy=(-math.pi * 0.5, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.000, FRONT_FACE_Y + 0.004, DIAL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shape(), "multimeter_stand"),
        material=stand_dark,
        name="panel",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.082, 0.009, STAND_HEIGHT)),
        mass=0.08,
        origin=Origin(xyz=(0.000, -0.0045, STAND_HEIGHT * 0.5)),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.000, BACK_BUMPER_Y, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    for index, x_pos in enumerate((-0.024, 0.000, 0.024)):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.014, 0.006, 0.008)),
            origin=Origin(xyz=(0.000, 0.003, 0.000)),
            material=dial_dark,
            name="stem",
        )
        button.visual(
            Box((0.022, 0.0025, 0.012)),
            origin=Origin(xyz=(0.000, 0.00725, 0.000)),
            material=body_polymer,
            name="cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.022, 0.0085, 0.012)),
            mass=0.01,
            origin=Origin(xyz=(0.000, 0.00425, 0.000)),
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, BUTTON_STEM_BACK_Y, BUTTON_FRAME_CENTER_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0018,
            ),
        )

    rocker = model.part("power_rocker")
    rocker.visual(
        Cylinder(radius=0.0018, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dial_dark,
        name="axle",
    )
    rocker.visual(
        Box((0.006, 0.016, 0.024)),
        origin=Origin(xyz=(0.003, 0.000, 0.000)),
        material=body_polymer,
        name="paddle",
    )
    rocker.inertial = Inertial.from_geometry(
        Box((0.006, 0.018, 0.024)),
        mass=0.015,
        origin=Origin(xyz=(0.003, 0.000, 0.000)),
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(ROCKER_MOUNT_X + 0.002, ROCKER_CENTER_Y, ROCKER_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=2.0,
            lower=-0.22,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")
    rocker = object_model.get_part("power_rocker")
    mode_buttons = [object_model.get_part(f"mode_button_{index}") for index in range(3)]

    dial_joint = object_model.get_articulation("body_to_dial")
    stand_joint = object_model.get_articulation("body_to_stand")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")
    button_joints = [
        object_model.get_articulation(f"body_to_mode_button_{index}") for index in range(3)
    ]

    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a="cap",
        elem_b="dial_well",
        min_overlap=0.040,
        name="range dial stays centered on the front panel",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="y",
        positive_elem="skirt",
        negative_elem="dial_well",
        max_gap=0.0005,
        max_penetration=0.0,
        name="range dial seats against the dial well",
    )
    with ctx.pose({dial_joint: 1.3}):
        ctx.expect_gap(
            dial,
            body,
            axis="y",
            positive_elem="skirt",
            negative_elem="dial_well",
            max_gap=0.0005,
            max_penetration=0.0,
            name="range dial stays seated while rotated",
        )

    ctx.expect_gap(
        body,
        stand,
        axis="y",
        positive_elem="rear_pad",
        negative_elem="panel",
        max_gap=0.0005,
        max_penetration=0.0,
        name="rear stand folds flat onto the back bumper",
    )

    closed_aabb = ctx.part_world_aabb(stand)
    upper_limit = stand_joint.motion_limits.upper if stand_joint.motion_limits is not None else None
    with ctx.pose({stand_joint: upper_limit if upper_limit is not None else 0.0}):
        opened_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "rear stand swings backward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.040,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.expect_gap(
        rocker,
        body,
        axis="x",
        positive_elem="paddle",
        negative_elem="rocker_mount",
        max_gap=0.0005,
        max_penetration=0.0,
        name="side power rocker stays seated on its side mount",
    )

    rocker_closed_aabb = ctx.part_world_aabb(rocker)
    rocker_upper = rocker_joint.motion_limits.upper if rocker_joint.motion_limits is not None else None
    with ctx.pose({rocker_joint: rocker_upper if rocker_upper is not None else 0.0}):
        rocker_open_aabb = ctx.part_world_aabb(rocker)
    ctx.check(
        "side power rocker tips upward at its outer edge",
        rocker_closed_aabb is not None
        and rocker_open_aabb is not None
        and rocker_open_aabb[1][2] > rocker_closed_aabb[1][2] + 0.001,
        details=f"closed={rocker_closed_aabb}, open={rocker_open_aabb}",
    )

    rest_button_positions = [ctx.part_world_position(button) for button in mode_buttons]
    for index, (button, joint, rest_pos) in enumerate(
        zip(mode_buttons, button_joints, rest_button_positions)
    ):
        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="cap",
            max_gap=0.0006,
            max_penetration=1e-6,
            name=f"mode button {index} sits proud of the front frame",
        )
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper if upper is not None else 0.0}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"mode button {index} depresses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    middle_rest = rest_button_positions[1]
    left_rest = rest_button_positions[0]
    right_rest = rest_button_positions[2]
    middle_upper = button_joints[1].motion_limits.upper if button_joints[1].motion_limits is not None else None
    with ctx.pose({button_joints[1]: middle_upper if middle_upper is not None else 0.0}):
        left_during = ctx.part_world_position(mode_buttons[0])
        middle_during = ctx.part_world_position(mode_buttons[1])
        right_during = ctx.part_world_position(mode_buttons[2])
    ctx.check(
        "mode buttons actuate independently",
        middle_rest is not None
        and middle_during is not None
        and left_rest is not None
        and left_during is not None
        and right_rest is not None
        and right_during is not None
        and middle_during[1] < middle_rest[1] - 0.0015
        and abs(left_during[1] - left_rest[1]) < 1e-6
        and abs(right_during[1] - right_rest[1]) < 1e-6,
        details=(
            f"left_rest={left_rest}, left_during={left_during}, "
            f"middle_rest={middle_rest}, middle_during={middle_during}, "
            f"right_rest={right_rest}, right_during={right_during}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
