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
    place_on_face,
)


SHELL_W = 0.235
SHELL_D = 0.185
SHELL_H = 0.325

GRILLE_W = 0.202
GRILLE_H = 0.225
GRILLE_T = 0.004
GRILLE_CENTER_Z = 0.122

STRIP_W = 0.205
STRIP_H = 0.058
STRIP_T = 0.012
STRIP_CENTER_Z = 0.272

KICK_HINGE_Z = 0.055
KICK_HINGE_Y = (SHELL_D * 0.5) + 0.0085


def _build_shell_body() -> object:
    return (
        cq.Workplane("XY")
        .box(SHELL_W, SHELL_D, SHELL_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
    )


def _build_control_strip_plate() -> object:
    rocker_x = 0.034
    button_0_x = 0.070
    button_1_x = 0.092

    plate = (
        cq.Workplane("XY")
        .rect(STRIP_W, STRIP_H)
        .extrude(STRIP_T)
        .edges("|Z")
        .fillet(0.006)
    )
    plate = (
        plate.faces(">Z")
        .workplane()
        .center(rocker_x, 0.0)
        .rect(0.024, 0.036)
        .cutBlind(-STRIP_T)
    )
    plate = (
        plate.faces(">Z")
        .workplane()
        .center(button_0_x, 0.0)
        .circle(0.0068)
        .cutBlind(-STRIP_T)
    )
    plate = (
        plate.faces(">Z")
        .workplane()
        .center(button_1_x, 0.0)
        .circle(0.0068)
        .cutBlind(-STRIP_T)
    )
    plate = (
        plate.faces(">Z")
        .workplane()
        .center(-0.054, 0.0)
        .circle(0.0042)
        .cutBlind(-STRIP_T)
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pa_speaker")

    shell_finish = model.material("shell_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    shell_trim = model.material("shell_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    grille_finish = model.material("grille_finish", rgba=(0.07, 0.07, 0.08, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    control_dark = model.material("control_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    control_mid = model.material("control_mid", rgba=(0.18, 0.18, 0.19, 1.0))
    button_finish = model.material("button_finish", rgba=(0.24, 0.24, 0.25, 1.0))
    rubber_finish = model.material("rubber_finish", rgba=(0.05, 0.05, 0.05, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_build_shell_body(), "speaker_shell"),
        material=shell_finish,
        name="cabinet",
    )
    shell.visual(
        Box((0.026, 0.004, 0.030)),
        origin=Origin(xyz=(-0.060, (SHELL_D * 0.5) + 0.002, KICK_HINGE_Z)),
        material=shell_trim,
        name="kick_pad_0",
    )
    shell.visual(
        Box((0.026, 0.004, 0.030)),
        origin=Origin(xyz=(0.060, (SHELL_D * 0.5) + 0.002, KICK_HINGE_Z)),
        material=shell_trim,
        name="kick_pad_1",
    )
    shell.visual(
        Box((0.038, 0.016, 0.006)),
        origin=Origin(xyz=(-0.060, 0.040, 0.003)),
        material=rubber_finish,
        name="foot_0",
    )
    shell.visual(
        Box((0.038, 0.016, 0.006)),
        origin=Origin(xyz=(0.060, 0.040, 0.003)),
        material=rubber_finish,
        name="foot_1",
    )
    shell.visual(
        Box((0.210, 0.012, 0.010)),
        origin=Origin(xyz=(0.000, -(SHELL_D * 0.5) + 0.006, 0.015)),
        material=shell_trim,
        name="lower_front_trim",
    )

    grille = model.part("grille")
    grille.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (GRILLE_W, GRILLE_H),
                GRILLE_T,
                hole_diameter=0.0055,
                pitch=(0.011, 0.011),
                frame=0.010,
                corner_radius=0.010,
                stagger=True,
                center=False,
            ),
            "speaker_grille",
        ),
        material=grille_finish,
        name="panel",
    )
    model.articulation(
        "shell_to_grille",
        ArticulationType.FIXED,
        parent=shell,
        child=grille,
        origin=place_on_face(
            shell,
            "-y",
            face_pos=(0.0, GRILLE_CENTER_Z - (SHELL_H * 0.5)),
            proud=0.0,
        ),
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        mesh_from_cadquery(_build_control_strip_plate(), "control_strip_plate"),
        material=panel_finish,
        name="plate",
    )
    control_strip.visual(
        Box((0.048, 0.010, 0.003)),
        origin=Origin(xyz=(-0.002, 0.017, STRIP_T + 0.0015)),
        material=control_mid,
        name="display_window",
    )
    model.articulation(
        "shell_to_control_strip",
        ArticulationType.FIXED,
        parent=shell,
        child=control_strip,
        origin=place_on_face(
            shell,
            "-y",
            face_pos=(0.0, STRIP_CENTER_Z - (SHELL_H * 0.5)),
            proud=0.0,
        ),
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.022, 0.028, 0.010)),
        origin=Origin(xyz=(-0.060, 0.000, 0.005)),
        material=handle_finish,
        name="foot_0",
    )
    handle.visual(
        Box((0.022, 0.028, 0.010)),
        origin=Origin(xyz=(0.060, 0.000, 0.005)),
        material=handle_finish,
        name="foot_1",
    )
    handle.visual(
        Box((0.016, 0.022, 0.034)),
        origin=Origin(xyz=(-0.060, 0.000, 0.022)),
        material=handle_finish,
        name="upright_0",
    )
    handle.visual(
        Box((0.016, 0.022, 0.034)),
        origin=Origin(xyz=(0.060, 0.000, 0.022)),
        material=handle_finish,
        name="upright_1",
    )
    handle.visual(
        Box((0.142, 0.024, 0.015)),
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
        material=handle_finish,
        name="grip",
    )
    model.articulation(
        "shell_to_handle",
        ArticulationType.FIXED,
        parent=shell,
        child=handle,
        origin=place_on_face(shell, "+z", face_pos=(0.0, 0.000), proud=0.0),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.020,
                body_style="skirted",
                top_diameter=0.031,
                skirt=KnobSkirt(0.044, 0.0045, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "selector_knob",
        ),
        material=control_dark,
        name="knob",
    )
    model.articulation(
        "control_strip_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=control_strip,
        child=selector_knob,
        origin=Origin(xyz=(-0.054, 0.000, STRIP_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.020, 0.030, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=control_dark,
        name="cap",
    )
    power_rocker.visual(
        Cylinder(radius=0.0023, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, 0.0023), rpy=(0.000, math.pi * 0.5, 0.000)),
        material=control_mid,
        name="pivot_axle",
    )
    power_rocker.visual(
        Box((0.015, 0.021, 0.0015)),
        origin=Origin(xyz=(0.000, 0.0025, 0.00875)),
        material=control_mid,
        name="face_pad",
    )
    model.articulation(
        "control_strip_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=control_strip,
        child=power_rocker,
        origin=Origin(xyz=(0.034, 0.000, STRIP_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.12,
            velocity=4.0,
            lower=-0.26,
            upper=0.26,
        ),
    )

    for index, x_pos in enumerate((0.070, 0.092)):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Cylinder(radius=0.0055, length=0.0042),
            origin=Origin(xyz=(0.000, 0.000, 0.0021)),
            material=button_finish,
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.0036, length=0.0035),
            origin=Origin(xyz=(0.000, 0.000, 0.00595)),
            material=control_mid,
            name="center_dot",
        )
        model.articulation(
            f"control_strip_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button,
            origin=Origin(xyz=(x_pos, 0.000, STRIP_T)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0022,
            ),
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.0045, length=0.150),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.000, math.pi * 0.5, 0.000)),
        material=shell_trim,
        name="hinge_rod",
    )
    kickstand.visual(
        Box((0.012, 0.008, 0.090)),
        origin=Origin(xyz=(-0.056, 0.0065, 0.045)),
        material=shell_trim,
        name="leg_0",
    )
    kickstand.visual(
        Box((0.012, 0.008, 0.090)),
        origin=Origin(xyz=(0.056, 0.0065, 0.045)),
        material=shell_trim,
        name="leg_1",
    )
    kickstand.visual(
        Box((0.118, 0.008, 0.012)),
        origin=Origin(xyz=(0.000, 0.0065, 0.084)),
        material=shell_trim,
        name="brace",
    )
    kickstand.visual(
        Box((0.026, 0.006, 0.010)),
        origin=Origin(xyz=(-0.056, 0.0075, 0.088)),
        material=rubber_finish,
        name="foot_0",
    )
    kickstand.visual(
        Box((0.026, 0.006, 0.010)),
        origin=Origin(xyz=(0.056, 0.0075, 0.088)),
        material=rubber_finish,
        name="foot_1",
    )
    model.articulation(
        "shell_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=kickstand,
        origin=Origin(xyz=(0.000, KICK_HINGE_Y, KICK_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=1.2,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    grille = object_model.get_part("grille")
    control_strip = object_model.get_part("control_strip")
    handle = object_model.get_part("handle")
    selector_knob = object_model.get_part("selector_knob")
    power_rocker = object_model.get_part("power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    kickstand = object_model.get_part("kickstand")

    knob_joint = object_model.get_articulation("control_strip_to_selector_knob")
    rocker_joint = object_model.get_articulation("control_strip_to_power_rocker")
    button_joint_0 = object_model.get_articulation("control_strip_to_menu_button_0")
    button_joint_1 = object_model.get_articulation("control_strip_to_menu_button_1")
    kick_joint = object_model.get_articulation("shell_to_kickstand")

    ctx.expect_contact(grille, shell, elem_a="panel", elem_b="cabinet", name="grille seats on cabinet face")
    ctx.expect_contact(control_strip, shell, elem_a="plate", elem_b="cabinet", name="control strip mounts on cabinet face")
    ctx.expect_contact(handle, shell, elem_a="foot_0", elem_b="cabinet", name="handle foot_0 lands on shell")
    ctx.expect_contact(handle, shell, elem_a="foot_1", elem_b="cabinet", name="handle foot_1 lands on shell")

    ctx.expect_gap(
        control_strip,
        grille,
        axis="z",
        min_gap=0.006,
        max_gap=0.014,
        positive_elem="plate",
        negative_elem="panel",
        name="control strip sits above grille with a narrow gap",
    )

    ctx.expect_overlap(
        selector_knob,
        control_strip,
        axes="xz",
        min_overlap=0.020,
        elem_a="knob",
        elem_b="plate",
        name="selector knob stays centered on control strip",
    )
    ctx.expect_gap(
        menu_button_0,
        power_rocker,
        axis="x",
        min_gap=0.010,
        elem_a="cap",
        elem_b="cap",
        name="menu button_0 stays distinct from rocker",
    )
    ctx.expect_gap(
        menu_button_1,
        menu_button_0,
        axis="x",
        min_gap=0.008,
        elem_a="cap",
        elem_b="cap",
        name="menu buttons remain visually separate",
    )

    ctx.check(
        "selector knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"type={knob_joint.articulation_type}, limits={knob_joint.motion_limits!r}",
    )

    rocker_limits = rocker_joint.motion_limits
    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        rocker_rest = ctx.part_element_world_aabb(power_rocker, elem="cap")
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            rocker_up = ctx.part_element_world_aabb(power_rocker, elem="cap")
        ctx.check(
            "power rocker pivots outward at its top edge",
            rocker_rest is not None
            and rocker_up is not None
            and rocker_up[0][1] < rocker_rest[0][1] - 0.001,
            details=f"rest={rocker_rest}, opened={rocker_up}",
        )

    button_0_rest = ctx.part_world_position(menu_button_0)
    button_1_rest = ctx.part_world_position(menu_button_1)
    if button_0_rest is not None and button_1_rest is not None:
        with ctx.pose({button_joint_0: 0.0022}):
            button_0_pressed = ctx.part_world_position(menu_button_0)
            button_1_still = ctx.part_world_position(menu_button_1)
        ctx.check(
            "menu button_0 depresses independently",
            button_0_pressed is not None
            and button_1_still is not None
            and button_0_pressed[1] > button_0_rest[1] + 0.0015
            and abs(button_1_still[1] - button_1_rest[1]) <= 1e-6,
            details=f"rest0={button_0_rest}, pressed0={button_0_pressed}, rest1={button_1_rest}, still1={button_1_still}",
        )
        with ctx.pose({button_joint_1: 0.0022}):
            button_1_pressed = ctx.part_world_position(menu_button_1)
            button_0_still = ctx.part_world_position(menu_button_0)
        ctx.check(
            "menu button_1 depresses independently",
            button_1_pressed is not None
            and button_0_still is not None
            and button_1_pressed[1] > button_1_rest[1] + 0.0015
            and abs(button_0_still[1] - button_0_rest[1]) <= 1e-6,
            details=f"rest1={button_1_rest}, pressed1={button_1_pressed}, rest0={button_0_rest}, still0={button_0_still}",
        )

    kick_limits = kick_joint.motion_limits
    if kick_limits is not None and kick_limits.upper is not None:
        kick_rest = ctx.part_world_aabb(kickstand)
        with ctx.pose({kick_joint: kick_limits.upper}):
            kick_open = ctx.part_world_aabb(kickstand)
        ctx.check(
            "kickstand swings rearward",
            kick_rest is not None
            and kick_open is not None
            and kick_open[1][1] > kick_rest[1][1] + 0.035,
            details=f"rest={kick_rest}, open={kick_open}",
        )
        ctx.expect_gap(
            kickstand,
            shell,
            axis="y",
            min_gap=0.002,
            max_gap=0.018,
            positive_elem="brace",
            negative_elem="cabinet",
            name="stowed kickstand remains close to rear shell",
        )

    return ctx.report()


object_model = build_object_model()
