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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.236
BODY_D = 0.186
BODY_H = 0.342
DECK_T = 0.010

GRILLE_W = 0.192
GRILLE_H = 0.236

RAIL_X = 0.064
RAIL_Y = -0.102
SLEEVE_BOTTOM = 0.180
SLEEVE_LEN = 0.125
SLEEVE_TOP = SLEEVE_BOTTOM + SLEEVE_LEN
SLEEVE_OUTER_R = 0.0105
SLEEVE_INNER_R = 0.0082
HANDLE_TRAVEL = 0.210
INNER_RAIL_R = 0.0072
INNER_RAIL_LEN = 0.440
INNER_RAIL_CENTER_Z = -0.044
GRIP_Z = 0.184

WHEEL_CENTER_Y = -0.132
WHEEL_CENTER_Z = 0.043
WHEEL_CENTER_X = 0.130
WHEEL_RADIUS = 0.039
WHEEL_WIDTH = 0.022

DECK_TOP = BODY_H + DECK_T

KNOB_POS = (-0.068, 0.010, DECK_TOP)
MODE_BUTTON_POSITIONS = (
    (-0.008, 0.026, DECK_TOP),
    (-0.008, -0.006, DECK_TOP),
)
ROCKER_POS = (0.060, 0.010, DECK_TOP + 0.008)
MENU_BUTTON_POSITIONS = (
    (0.094, 0.026, DECK_TOP),
    (0.094, -0.006, DECK_TOP),
)


def _speaker_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    shell = shell.edges("|Z").fillet(0.018)
    shell = shell.edges("<Z").fillet(0.006)
    shell = shell.edges(">Z").fillet(0.012)

    front_recess = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, BODY_D * 0.5 - 0.0075, 0.172))
        .box(0.202, 0.016, 0.252, centered=(True, True, True))
    )
    top_recess = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BODY_H - 0.003))
        .box(0.206, 0.142, 0.012, centered=(True, True, True))
    )
    return shell.cut(front_recess).cut(top_recess)


def _tube_shape(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="karaoke_speaker")

    shell_black = model.material("shell_black", rgba=(0.15, 0.16, 0.18, 1.0))
    deck_black = model.material("deck_black", rgba=(0.08, 0.09, 0.10, 1.0))
    grille_black = model.material("grille_black", rgba=(0.11, 0.11, 0.12, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.24, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    rocker_red = model.material("rocker_red", rgba=(0.46, 0.09, 0.10, 1.0))
    button_grey = model.material("button_grey", rgba=(0.34, 0.35, 0.37, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.22, 0.23, 0.24, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_speaker_shell_shape(), "speaker_shell"),
        material=shell_black,
        name="shell",
    )
    body.visual(
        Box((0.206, 0.142, DECK_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H + DECK_T * 0.5)),
        material=deck_black,
        name="control_deck",
    )
    body.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (GRILLE_W, GRILLE_H),
                0.0035,
                hole_diameter=0.0045,
                pitch=(0.009, 0.009),
                frame=0.010,
                corner_radius=0.010,
                stagger=True,
                center=False,
            ),
            "speaker_grille",
        ),
        origin=Origin(
            xyz=(0.0, BODY_D * 0.5 - 0.0155, 0.172),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=grille_black,
        name="front_grille",
    )
    sleeve_mesh = mesh_from_cadquery(
        _tube_shape(SLEEVE_OUTER_R, SLEEVE_INNER_R, SLEEVE_LEN),
        "trolley_sleeve",
    )
    collar_mesh = mesh_from_cadquery(
        _tube_shape(0.013, 0.0085, 0.010),
        "trolley_sleeve_collar",
    )
    for index, x_pos in enumerate((-RAIL_X, RAIL_X)):
        body.visual(
            sleeve_mesh,
            origin=Origin(xyz=(x_pos, RAIL_Y, SLEEVE_BOTTOM)),
            material=dark_grey,
            name=f"sleeve_{index}",
        )
        body.visual(
            collar_mesh,
            origin=Origin(xyz=(x_pos, RAIL_Y, SLEEVE_TOP - 0.005)),
            material=trim_grey,
            name=f"sleeve_collar_{index}",
        )
    for index, x_pos in enumerate((-WHEEL_CENTER_X, WHEEL_CENTER_X)):
        body.visual(
            Box((0.024, 0.030, 0.064)),
            origin=Origin(xyz=(x_pos * 0.86, -0.106, 0.058)),
            material=trim_grey,
            name=f"wheel_bracket_{index}",
        )
    body.visual(
        Box((0.062, 0.016, 0.010)),
        origin=Origin(xyz=(-0.008, 0.010, DECK_TOP + 0.005)),
        material=trim_grey,
        name="mode_button_pad",
    )
    body.visual(
        Box((0.038, 0.028, 0.004)),
        origin=Origin(xyz=(ROCKER_POS[0], ROCKER_POS[1], DECK_TOP + 0.002)),
        material=trim_grey,
        name="rocker_bezel",
    )
    handle = model.part("trolley_handle")
    for index, x_pos in enumerate((-RAIL_X, RAIL_X)):
        handle.visual(
            Cylinder(radius=INNER_RAIL_R, length=INNER_RAIL_LEN),
            origin=Origin(xyz=(x_pos, 0.0, INNER_RAIL_CENTER_Z)),
            material=steel,
            name=f"rail_{index}",
        )
    handle.visual(
        Box((0.158, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, GRIP_Z)),
        material=aluminum,
        name="grip_bar",
    )
    handle.visual(
        Box((0.154, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=trim_grey,
        name="lower_carriage",
    )
    handle.visual(
        Box((0.112, 0.026, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, GRIP_Z)),
        material=rubber,
        name="grip_sleeve",
    )
    model.articulation(
        "body_to_trolley_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, RAIL_Y, SLEEVE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.20,
            lower=0.0,
            upper=HANDLE_TRAVEL,
        ),
    )

    for index, x_pos in enumerate((-WHEEL_CENTER_X, WHEEL_CENTER_X)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.031, length=WHEEL_WIDTH - 0.004),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=dark_grey,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.013, length=WHEEL_WIDTH + 0.004),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=trim_grey,
            name="hub",
        )
        model.articulation(
            f"body_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x_pos, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=18.0),
        )

    main_knob = model.part("main_knob")
    main_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.028,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.062, 0.007, flare=0.05),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "speaker_main_knob",
        ),
        material=dark_grey,
        name="knob",
    )
    model.articulation(
        "body_to_main_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=main_knob,
        origin=Origin(xyz=KNOB_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=8.0),
    )

    for index, (x_pos, y_pos, z_pos) in enumerate(MODE_BUTTON_POSITIONS):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.034, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=button_grey,
            name="cap",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=0.004,
            ),
        )

    rocker = model.part("power_rocker")
    rocker.visual(
        Box((0.030, 0.020, 0.012)),
        material=rocker_red,
        name="rocker_cap",
    )
    rocker.visual(
        Cylinder(radius=0.0026, length=0.034),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim_grey,
        name="pivot_pin",
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=ROCKER_POS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.0,
            lower=-0.24,
            upper=0.24,
        ),
    )

    for index, (x_pos, y_pos, z_pos) in enumerate(MENU_BUTTON_POSITIONS):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Cylinder(radius=0.0075, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=button_grey,
            name="button",
        )
        model.articulation(
            f"body_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0035,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("trolley_handle")
    handle_joint = object_model.get_articulation("body_to_trolley_handle")
    handle_limits = handle_joint.motion_limits

    for rail_index in range(2):
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            inner_elem=f"rail_{rail_index}",
            outer_elem=f"sleeve_{rail_index}",
            margin=0.0015,
            name=f"rail_{rail_index} stays centered in sleeve at rest",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a=f"rail_{rail_index}",
            elem_b=f"sleeve_{rail_index}",
            min_overlap=0.115,
            name=f"rail_{rail_index} remains inserted when collapsed",
        )

    handle_rest = ctx.part_world_position(handle)
    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_joint: handle_limits.upper}):
            for rail_index in range(2):
                ctx.expect_within(
                    handle,
                    body,
                    axes="xy",
                    inner_elem=f"rail_{rail_index}",
                    outer_elem=f"sleeve_{rail_index}",
                    margin=0.0015,
                    name=f"rail_{rail_index} stays centered in sleeve when extended",
                )
                ctx.expect_overlap(
                    handle,
                    body,
                    axes="z",
                    elem_a=f"rail_{rail_index}",
                    elem_b=f"sleeve_{rail_index}",
                    min_overlap=0.040,
                    name=f"rail_{rail_index} still retains insertion when extended",
                )
            handle_extended = ctx.part_world_position(handle)
        ctx.check(
            "trolley handle extends upward",
            handle_rest is not None
            and handle_extended is not None
            and handle_extended[2] > handle_rest[2] + 0.18,
            details=f"rest={handle_rest}, extended={handle_extended}",
        )

    rocker = object_model.get_part("power_rocker")
    for index in range(2):
        menu_button = object_model.get_part(f"menu_button_{index}")
        ctx.expect_gap(
            menu_button,
            rocker,
            axis="x",
            min_gap=0.009,
            name=f"menu button {index} stays distinct from power rocker",
        )

    mode_button_0 = object_model.get_part("mode_button_0")
    mode_button_1 = object_model.get_part("mode_button_1")
    mode_joint_0 = object_model.get_articulation("body_to_mode_button_0")
    mode_rest_0 = ctx.part_world_position(mode_button_0)
    mode_rest_1 = ctx.part_world_position(mode_button_1)
    mode_limits_0 = mode_joint_0.motion_limits
    if mode_limits_0 is not None and mode_limits_0.upper is not None:
        with ctx.pose({mode_joint_0: mode_limits_0.upper}):
            mode_pressed_0 = ctx.part_world_position(mode_button_0)
            mode_other_1 = ctx.part_world_position(mode_button_1)
        ctx.check(
            "mode buttons articulate independently",
            mode_rest_0 is not None
            and mode_pressed_0 is not None
            and mode_rest_1 is not None
            and mode_other_1 is not None
            and mode_pressed_0[2] < mode_rest_0[2] - 0.0025
            and abs(mode_other_1[2] - mode_rest_1[2]) < 0.0005,
            details=(
                f"rest0={mode_rest_0}, pressed0={mode_pressed_0}, "
                f"rest1={mode_rest_1}, other1={mode_other_1}"
            ),
        )

    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    menu_joint_0 = object_model.get_articulation("body_to_menu_button_0")
    menu_rest_0 = ctx.part_world_position(menu_button_0)
    menu_rest_1 = ctx.part_world_position(menu_button_1)
    menu_limits_0 = menu_joint_0.motion_limits
    if menu_limits_0 is not None and menu_limits_0.upper is not None:
        with ctx.pose({menu_joint_0: menu_limits_0.upper}):
            menu_pressed_0 = ctx.part_world_position(menu_button_0)
            menu_other_1 = ctx.part_world_position(menu_button_1)
        ctx.check(
            "menu buttons articulate independently",
            menu_rest_0 is not None
            and menu_pressed_0 is not None
            and menu_rest_1 is not None
            and menu_other_1 is not None
            and menu_pressed_0[2] < menu_rest_0[2] - 0.002
            and abs(menu_other_1[2] - menu_rest_1[2]) < 0.0005,
            details=(
                f"rest0={menu_rest_0}, pressed0={menu_pressed_0}, "
                f"rest1={menu_rest_1}, other1={menu_other_1}"
            ),
        )

    for joint_name in ("body_to_main_knob", "body_to_wheel_0", "body_to_wheel_1"):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
