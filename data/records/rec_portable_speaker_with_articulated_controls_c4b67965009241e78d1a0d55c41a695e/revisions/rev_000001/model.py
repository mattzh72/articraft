from __future__ import annotations

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
    mesh_from_geometry,
)


SHELL_D = 0.220
SHELL_W = 0.290
SHELL_H = 0.400
WALL = 0.015

FRONT_X = SHELL_D * 0.5
REAR_X = -SHELL_D * 0.5
TOP_Z = SHELL_H
INNER_W = SHELL_W - 2.0 * WALL

CONTROL_X = FRONT_X + 0.009
CONTROL_T = 0.018
CONTROL_FRONT_X = CONTROL_X + CONTROL_T * 0.5
CONTROL_Z = 0.322
CONTROL_H = 0.078

BUTTON_DEPTH = 0.008
BUTTON_WIDTH = 0.022
BUTTON_HEIGHT = 0.016
BUTTON_Y = (0.030, 0.065, 0.100)

GRILLE_H = 0.268
GRILLE_Z = 0.149


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pa_speaker")

    body = model.material("body", rgba=(0.16, 0.17, 0.18, 1.0))
    trim = model.material("trim", rgba=(0.21, 0.22, 0.24, 1.0))
    grille_mat = model.material("grille", rgba=(0.07, 0.07, 0.08, 1.0))
    handle_mat = model.material("handle", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_mat = model.material("knob", rgba=(0.13, 0.13, 0.14, 1.0))
    button_mat = model.material("button", rgba=(0.58, 0.60, 0.63, 1.0))
    stand_mat = model.material("stand", rgba=(0.12, 0.12, 0.13, 1.0))

    shell = model.part("shell")
    shell.visual(
        Box((WALL, SHELL_W, SHELL_H)),
        origin=Origin(xyz=(REAR_X + WALL * 0.5, 0.0, SHELL_H * 0.5)),
        material=body,
        name="rear_wall",
    )
    shell.visual(
        Box((SHELL_D - WALL, WALL, SHELL_H)),
        origin=Origin(xyz=(0.5 * WALL, -(SHELL_W - WALL) * 0.5, SHELL_H * 0.5)),
        material=body,
        name="side_wall_0",
    )
    shell.visual(
        Box((SHELL_D - WALL, WALL, SHELL_H)),
        origin=Origin(xyz=(0.5 * WALL, (SHELL_W - WALL) * 0.5, SHELL_H * 0.5)),
        material=body,
        name="side_wall_1",
    )
    shell.visual(
        Box((SHELL_D - WALL, INNER_W, WALL)),
        origin=Origin(xyz=(0.5 * WALL, 0.0, WALL * 0.5)),
        material=body,
        name="bottom_wall",
    )
    shell.visual(
        Box((SHELL_D - WALL, INNER_W, WALL)),
        origin=Origin(xyz=(0.5 * WALL, 0.0, TOP_Z - WALL * 0.5)),
        material=body,
        name="top_wall",
    )
    shell.visual(
        Box((CONTROL_T, INNER_W, 0.031)),
        origin=Origin(xyz=(CONTROL_X, 0.0, 0.3455)),
        material=trim,
        name="control_top",
    )
    shell.visual(
        Box((CONTROL_T, INNER_W, 0.031)),
        origin=Origin(xyz=(CONTROL_X, 0.0, 0.2985)),
        material=trim,
        name="control_bottom",
    )
    shell.visual(
        Box((CONTROL_T, 0.149, 0.016)),
        origin=Origin(xyz=(CONTROL_X, -0.0555, CONTROL_Z)),
        material=trim,
        name="control_left",
    )
    shell.visual(
        Box((CONTROL_T, 0.013, 0.016)),
        origin=Origin(xyz=(CONTROL_X, 0.0475, CONTROL_Z)),
        material=trim,
        name="control_web_0",
    )
    shell.visual(
        Box((CONTROL_T, 0.013, 0.016)),
        origin=Origin(xyz=(CONTROL_X, 0.0825, CONTROL_Z)),
        material=trim,
        name="control_web_1",
    )
    shell.visual(
        Box((CONTROL_T, 0.019, 0.016)),
        origin=Origin(xyz=(CONTROL_X, 0.1205, CONTROL_Z)),
        material=trim,
        name="control_right",
    )
    grille = model.part("grille")
    grille_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (INNER_W, GRILLE_H),
            0.004,
            hole_diameter=0.006,
            pitch=(0.012, 0.012),
            frame=0.010,
            corner_radius=0.006,
            stagger=True,
            center=False,
        ),
        "speaker_grille",
    )
    grille.visual(
        grille_mesh,
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=grille_mat,
        name="grille_face",
    )
    grille.visual(
        Box((0.016, INNER_W - 0.020, GRILLE_H - 0.020)),
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
        material=grille_mat,
        name="grille_shadow",
    )
    model.articulation(
        "shell_to_grille",
        ArticulationType.FIXED,
        parent=shell,
        child=grille,
        origin=Origin(xyz=(FRONT_X, 0.0, GRILLE_Z)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.172),
        origin=Origin(xyz=(0.0, 0.0, 0.036), rpy=(1.57079632679, 0.0, 0.0)),
        material=handle_mat,
        name="handle_bar",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(xyz=(0.0, -0.080, 0.018)),
        material=handle_mat,
        name="handle_leg_0",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(xyz=(0.0, 0.080, 0.018)),
        material=handle_mat,
        name="handle_leg_1",
    )
    model.articulation(
        "shell_to_handle",
        ArticulationType.FIXED,
        parent=shell,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.024,
                body_style="skirted",
                top_diameter=0.036,
                skirt=KnobSkirt(0.052, 0.005, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speaker_selector_knob",
        ),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=knob_mat,
        name="knob_cap",
    )
    selector_knob.visual(
        Cylinder(radius=0.006, length=0.007),
        origin=Origin(xyz=(0.0035, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=knob_mat,
        name="knob_spindle",
    )
    model.articulation(
        "shell_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=shell,
        child=selector_knob,
        origin=Origin(xyz=(CONTROL_FRONT_X, -0.074, CONTROL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.12, velocity=5.0),
    )

    for index, y_pos in enumerate(BUTTON_Y):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_DEPTH, BUTTON_WIDTH, BUTTON_HEIGHT)),
            origin=Origin(xyz=(BUTTON_DEPTH * 0.5, 0.0, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"shell_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=shell,
            child=button,
            origin=Origin(xyz=(CONTROL_FRONT_X, y_pos, CONTROL_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=0.003,
            ),
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=stand_mat,
        name="kickstand_barrel",
    )
    kickstand.visual(
        Box((0.010, 0.016, 0.138)),
        origin=Origin(xyz=(0.001, -0.042, 0.075)),
        material=stand_mat,
        name="kickstand_arm_0",
    )
    kickstand.visual(
        Box((0.010, 0.016, 0.138)),
        origin=Origin(xyz=(0.001, 0.042, 0.075)),
        material=stand_mat,
        name="kickstand_arm_1",
    )
    kickstand.visual(
        Box((0.014, 0.110, 0.018)),
        origin=Origin(xyz=(-0.002, 0.0, 0.150)),
        material=stand_mat,
        name="kickstand_foot",
    )
    model.articulation(
        "shell_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=kickstand,
        origin=Origin(xyz=(REAR_X - 0.006, 0.0, 0.036)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=0.78,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    grille = object_model.get_part("grille")
    selector_knob = object_model.get_part("selector_knob")
    kickstand = object_model.get_part("kickstand")

    knob_joint = object_model.get_articulation("shell_to_selector_knob")
    kickstand_joint = object_model.get_articulation("shell_to_kickstand")
    button_joints = [
        object_model.get_articulation("shell_to_button_0"),
        object_model.get_articulation("shell_to_button_1"),
        object_model.get_articulation("shell_to_button_2"),
    ]
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]

    ctx.check(
        "selector knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={knob_joint.articulation_type}",
    )

    ctx.expect_gap(
        selector_knob,
        grille,
        axis="z",
        min_gap=0.015,
        positive_elem="knob_cap",
        negative_elem="grille_face",
        name="selector knob sits above the grille",
    )

    for index, button in enumerate(buttons):
        ctx.expect_gap(
            button,
            grille,
            axis="z",
            min_gap=0.020,
            positive_elem="button_cap",
            negative_elem="grille_face",
            name=f"button_{index} sits above the grille",
        )

    button_rest_positions = [ctx.part_world_position(button) for button in buttons]
    for index, (button, joint) in enumerate(zip(buttons, button_joints)):
        upper = 0.003
        if joint.motion_limits is not None and joint.motion_limits.upper is not None:
            upper = joint.motion_limits.upper
        with ctx.pose({joint: upper}):
            pressed_position = ctx.part_world_position(button)
            other_position = ctx.part_world_position(buttons[(index + 1) % 3])
        rest_position = button_rest_positions[index]
        other_rest = button_rest_positions[(index + 1) % 3]
        ctx.check(
            f"button_{index} depresses inward",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[0] < rest_position[0] - 0.002,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )
        ctx.check(
            f"button_{index} moves independently",
            other_rest is not None
            and other_position is not None
            and abs(other_position[0] - other_rest[0]) < 1e-6,
            details=f"other_rest={other_rest}, other_pressed={other_position}",
        )

    kickstand_upper = 0.78
    if kickstand_joint.motion_limits is not None and kickstand_joint.motion_limits.upper is not None:
        kickstand_upper = kickstand_joint.motion_limits.upper

    rest_foot = ctx.part_element_world_aabb(kickstand, elem="kickstand_foot")
    with ctx.pose({kickstand_joint: kickstand_upper}):
        open_foot = ctx.part_element_world_aabb(kickstand, elem="kickstand_foot")
    ctx.check(
        "kickstand swings rearward from the lower hinge",
        rest_foot is not None
        and open_foot is not None
        and open_foot[0][0] < rest_foot[0][0] - 0.050,
        details=f"rest_foot={rest_foot}, open_foot={open_foot}",
    )

    ctx.expect_gap(
        shell,
        kickstand,
        axis="x",
        max_gap=0.008,
        max_penetration=1e-5,
        positive_elem="rear_wall",
        negative_elem="kickstand_arm_0",
        name="closed kickstand stays close to the back shell",
    )

    return ctx.report()


object_model = build_object_model()
