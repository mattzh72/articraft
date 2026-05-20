from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_WIDTH = 0.315
BODY_DEPTH = 0.285
BODY_HEIGHT = 0.620
BODY_FRONT_Y = -BODY_DEPTH / 2.0
SIDE_WALL_THICKNESS = 0.025
INNER_WIDTH = BODY_WIDTH - 2.0 * SIDE_WALL_THICKNESS

HANDLE_POCKET_WIDTH = 0.246
HANDLE_POCKET_DEPTH = 0.114
HANDLE_POCKET_HEIGHT = 0.034
HANDLE_POCKET_CENTER_Y = 0.006
HANDLE_FINGER_RELIEF_WIDTH = 0.200
HANDLE_FINGER_RELIEF_DEPTH = 0.062
HANDLE_FINGER_RELIEF_HEIGHT = 0.018
HANDLE_HINGE_Y = -0.038
HANDLE_HINGE_Z = BODY_HEIGHT - 0.026

GRILLE_WIDTH = INNER_WIDTH
GRILLE_HEIGHT = 0.402
GRILLE_THICKNESS = 0.008
GRILLE_CENTER_Z = 0.289

PANEL_WIDTH = 0.252
PANEL_HEIGHT = 0.076
PANEL_THICKNESS = 0.008
PANEL_CENTER_Z = 0.556

KNOB_X = -0.074
KNOB_Z = 0.012
ROCKER_X = 0.020
ROCKER_Z = 0.000
MENU_BUTTON_DEPTH = 0.004
MENU_BUTTON_0_X = 0.074
MENU_BUTTON_1_X = 0.108
MENU_BUTTON_Z = 0.000
MENU_TRAVEL = 0.0014


def _build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(
        BODY_WIDTH,
        BODY_DEPTH,
        BODY_HEIGHT,
        centered=(True, True, False),
    )

    handle_pocket = (
        cq.Workplane("XY")
        .box(
            HANDLE_POCKET_WIDTH,
            HANDLE_POCKET_DEPTH,
            HANDLE_POCKET_HEIGHT,
            centered=(True, True, False),
        )
        .translate(
            (
                0.0,
                HANDLE_POCKET_CENTER_Y,
                BODY_HEIGHT - HANDLE_POCKET_HEIGHT,
            )
        )
    )
    finger_relief = cq.Workplane("XY").box(
        HANDLE_FINGER_RELIEF_WIDTH,
        HANDLE_FINGER_RELIEF_DEPTH,
        HANDLE_FINGER_RELIEF_HEIGHT,
        centered=(True, True, False),
    ).translate(
        (
            0.0,
            HANDLE_POCKET_CENTER_Y + 0.006,
            BODY_HEIGHT - HANDLE_POCKET_HEIGHT - HANDLE_FINGER_RELIEF_HEIGHT,
        )
    )
    grille_recess = (
        cq.Workplane("XY")
        .box(GRILLE_WIDTH + 0.018, 0.014, GRILLE_HEIGHT + 0.020, centered=(True, True, False))
        .translate((0.0, BODY_FRONT_Y + 0.006, GRILLE_CENTER_Z - (GRILLE_HEIGHT + 0.020) / 2.0))
    )
    panel_recess = (
        cq.Workplane("XY")
        .box(PANEL_WIDTH + 0.010, PANEL_THICKNESS + 0.002, PANEL_HEIGHT + 0.012, centered=(True, True, False))
        .translate((0.0, BODY_FRONT_Y + PANEL_THICKNESS / 2.0, PANEL_CENTER_Z - (PANEL_HEIGHT + 0.012) / 2.0))
    )

    return shell.cut(handle_pocket).cut(finger_relief).cut(grille_recess).cut(panel_recess)


def _build_handle_mesh() -> cq.Workplane:
    grip_length = 0.220
    grip_depth = 0.020
    grip_height = 0.014
    grip_center_y = 0.034
    grip_center_z = 0.013
    arm_width = 0.016
    arm_length = 0.040
    arm_height = 0.012
    arm_center_y = 0.018
    arm_center_z = 0.009
    barrel_radius = 0.006
    half_span = grip_length / 2.0 - arm_width / 2.0

    grip = cq.Workplane("XY").box(grip_length, grip_depth, grip_height).translate(
        (0.0, grip_center_y, grip_center_z)
    )
    left_arm = cq.Workplane("XY").box(arm_width, arm_length, arm_height).translate(
        (-half_span, arm_center_y, arm_center_z)
    )
    right_arm = cq.Workplane("XY").box(arm_width, arm_length, arm_height).translate(
        (half_span, arm_center_y, arm_center_z)
    )
    left_barrel = (
        cq.Workplane("YZ")
        .circle(barrel_radius)
        .extrude(arm_width, both=True)
        .translate((-half_span, 0.0, 0.0))
    )
    right_barrel = (
        cq.Workplane("YZ")
        .circle(barrel_radius)
        .extrude(arm_width, both=True)
        .translate((half_span, 0.0, 0.0))
    )

    return grip.union(left_arm).union(right_arm).union(left_barrel).union(right_barrel)


def _build_rocker_mesh() -> cq.Workplane:
    rocker = (
        cq.Workplane("XZ")
        .moveTo(-0.016, -0.010)
        .lineTo(0.016, -0.010)
        .lineTo(0.014, 0.010)
        .lineTo(-0.014, 0.010)
        .close()
        .extrude(0.0025, both=True)
    )
    return rocker.edges("|Y").fillet(0.001)


def _build_menu_button_mesh() -> cq.Workplane:
    return cq.Workplane("XY").box(0.016, MENU_BUTTON_DEPTH, 0.010).edges("|Y").fillet(0.0012)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="party_speaker")

    body_mat = model.material("body_mat", rgba=(0.15, 0.16, 0.17, 1.0))
    trim_mat = model.material("trim_mat", rgba=(0.07, 0.07, 0.08, 1.0))
    grille_mat = model.material("grille_mat", rgba=(0.09, 0.09, 0.10, 1.0))
    handle_mat = model.material("handle_mat", rgba=(0.08, 0.08, 0.09, 1.0))
    control_mat = model.material("control_mat", rgba=(0.11, 0.11, 0.12, 1.0))
    button_mat = model.material("button_mat", rgba=(0.17, 0.18, 0.19, 1.0))
    rocker_mat = model.material("rocker_mat", rgba=(0.18, 0.18, 0.19, 1.0))
    foot_mat = model.material("foot_mat", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        Box((SIDE_WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_WIDTH / 2.0 - SIDE_WALL_THICKNESS / 2.0), 0.0, BODY_HEIGHT / 2.0)),
        material=body_mat,
        name="side_0",
    )
    body.visual(
        Box((SIDE_WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=((BODY_WIDTH / 2.0 - SIDE_WALL_THICKNESS / 2.0), 0.0, BODY_HEIGHT / 2.0)),
        material=body_mat,
        name="side_1",
    )
    body.visual(
        Box((INNER_WIDTH, 0.012, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - 0.006, BODY_HEIGHT / 2.0)),
        material=body_mat,
        name="rear_wall",
    )
    body.visual(
        Box((INNER_WIDTH, BODY_DEPTH, 0.088)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=body_mat,
        name="bottom_band",
    )
    body.visual(
        Box((INNER_WIDTH, BODY_DEPTH, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.504)),
        material=body_mat,
        name="panel_bridge",
    )
    body.visual(
        Box((INNER_WIDTH, 0.060, 0.026)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.030, BODY_HEIGHT - 0.013)),
        material=body_mat,
        name="front_header",
    )
    body.visual(
        Box((INNER_WIDTH, 0.090, 0.022)),
        origin=Origin(xyz=(0.0, 0.0975, BODY_HEIGHT - 0.011)),
        material=body_mat,
        name="rear_top",
    )
    cheek_width = (INNER_WIDTH - PANEL_WIDTH) / 2.0
    cheek_x = PANEL_WIDTH / 2.0 + cheek_width / 2.0
    for index, x_pos in enumerate((-cheek_x, cheek_x)):
        body.visual(
            Box((cheek_width, BODY_DEPTH, PANEL_HEIGHT + 0.010)),
            origin=Origin(xyz=(x_pos, 0.0, PANEL_CENTER_Z)),
            material=body_mat,
            name=f"panel_cheek_{index}",
        )
    for index, x_pos in enumerate((-0.1215, 0.1215)):
        body.visual(
            Box((0.023, 0.016, 0.020)),
            origin=Origin(xyz=(x_pos, HANDLE_HINGE_Y, HANDLE_HINGE_Z)),
            material=trim_mat,
            name=f"hinge_mount_{index}",
        )
    for index, x_pos in enumerate((-0.100, 0.100)):
        body.visual(
            Box((0.060, 0.020, 0.010)),
            origin=Origin(xyz=(x_pos, 0.000, 0.005)),
            material=foot_mat,
            name=f"foot_{index}",
        )

    grille = model.part("grille")
    grille.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (GRILLE_WIDTH, GRILLE_HEIGHT),
                GRILLE_THICKNESS,
                hole_diameter=0.006,
                pitch=(0.012, 0.012),
                frame=0.010,
                corner_radius=0.010,
                stagger=True,
                center=False,
            ),
            "party_speaker_grille",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="grille_face",
    )
    model.articulation(
        "body_to_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.010, GRILLE_CENTER_Z)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS / 2.0, 0.0)),
        material=control_mat,
        name="panel_shell",
    )
    control_panel.visual(
        Box((0.082, 0.002, 0.016)),
        origin=Origin(xyz=(KNOB_X + 0.006, -0.001, 0.024)),
        material=trim_mat,
        name="knob_strip",
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, PANEL_CENTER_Z)),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_mesh(), "party_speaker_handle"),
        material=handle_mat,
        name="handle_grip",
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_HINGE_Y, HANDLE_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.20),
    )

    volume_knob = model.part("volume_knob")
    volume_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.024,
                body_style="skirted",
                top_diameter=0.034,
                edge_radius=0.0015,
                center=False,
            ),
            "party_speaker_volume_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_mat,
        name="knob",
    )
    model.articulation(
        "panel_to_volume_knob",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=volume_knob,
        origin=Origin(xyz=(KNOB_X, 0.0, KNOB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=12.0),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        mesh_from_cadquery(_build_rocker_mesh(), "party_speaker_power_rocker"),
        material=rocker_mat,
        name="rocker",
    )
    model.articulation(
        "panel_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=control_panel,
        child=power_rocker,
        origin=Origin(xyz=(ROCKER_X, -0.0025, ROCKER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-0.20, upper=0.20),
    )

    for index, x_pos in enumerate((MENU_BUTTON_0_X, MENU_BUTTON_1_X)):
        button = model.part(f"menu_button_{index}")
        button.visual(
            mesh_from_cadquery(_build_menu_button_mesh(), "party_speaker_menu_button"),
            material=button_mat,
            name="button",
        )
        model.articulation(
            f"panel_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(x_pos, -MENU_BUTTON_DEPTH / 2.0, MENU_BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.04,
                lower=-MENU_TRAVEL,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "body",
        "handle",
        elem_a="hinge_mount_0",
        elem_b="handle_grip",
        reason="The recessed carry handle uses captured side hinge blocks, so the simplified hinge barrel sits partially inside the support block.",
    )
    ctx.allow_overlap(
        "body",
        "handle",
        elem_a="hinge_mount_1",
        elem_b="handle_grip",
        reason="The recessed carry handle uses captured side hinge blocks, so the simplified hinge barrel sits partially inside the support block.",
    )

    handle = object_model.get_part("handle")
    grille = object_model.get_part("grille")
    control_panel = object_model.get_part("control_panel")
    volume_joint = object_model.get_articulation("panel_to_volume_knob")
    rocker_joint = object_model.get_articulation("panel_to_power_rocker")
    menu_0 = object_model.get_part("menu_button_0")
    menu_1 = object_model.get_part("menu_button_1")
    menu_0_joint = object_model.get_articulation("panel_to_menu_button_0")
    menu_1_joint = object_model.get_articulation("panel_to_menu_button_1")
    handle_joint = object_model.get_articulation("body_to_handle")

    ctx.expect_gap(
        control_panel,
        grille,
        axis="z",
        min_gap=0.020,
        max_gap=0.050,
        name="control row sits above the main grille",
    )
    ctx.expect_origin_gap(
        menu_0,
        "power_rocker",
        axis="x",
        min_gap=0.030,
        name="power rocker stays distinct from the first menu button",
    )
    ctx.expect_origin_gap(
        menu_1,
        menu_0,
        axis="x",
        min_gap=0.022,
        name="menu buttons remain separate controls",
    )
    ctx.check(
        "volume knob uses continuous rotation",
        volume_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={volume_joint.articulation_type}",
    )
    ctx.check(
        "power switch is a rocker pivot",
        rocker_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={rocker_joint.articulation_type}",
    )

    menu_0_rest = ctx.part_world_position(menu_0)
    with ctx.pose({menu_0_joint: MENU_TRAVEL * -1.0}):
        menu_0_pressed = ctx.part_world_position(menu_0)
    ctx.check(
        "menu button 0 depresses inward",
        menu_0_rest is not None
        and menu_0_pressed is not None
        and menu_0_pressed[1] > menu_0_rest[1] + 0.0008,
        details=f"rest={menu_0_rest}, pressed={menu_0_pressed}",
    )

    menu_1_rest = ctx.part_world_position(menu_1)
    with ctx.pose({menu_1_joint: MENU_TRAVEL * -1.0}):
        menu_1_pressed = ctx.part_world_position(menu_1)
    ctx.check(
        "menu button 1 depresses inward",
        menu_1_rest is not None
        and menu_1_pressed is not None
        and menu_1_pressed[1] > menu_1_rest[1] + 0.0008,
        details=f"rest={menu_1_rest}, pressed={menu_1_pressed}",
    )

    handle_closed = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_joint: 1.10}):
        handle_open = ctx.part_element_world_aabb(handle, elem="handle_grip")
    ctx.check(
        "stowed handle stays within the top recess",
        handle_closed is not None and handle_closed[1][2] <= BODY_HEIGHT + 0.002,
        details=f"closed_aabb={handle_closed}",
    )
    ctx.check(
        "handle lifts above the cabinet when raised",
        handle_open is not None
        and handle_open[1][2] >= BODY_HEIGHT + 0.015
        and handle_closed is not None
        and handle_open[1][2] > handle_closed[1][2] + 0.020,
        details=f"closed_aabb={handle_closed}, open_aabb={handle_open}",
    )

    return ctx.report()


object_model = build_object_model()
