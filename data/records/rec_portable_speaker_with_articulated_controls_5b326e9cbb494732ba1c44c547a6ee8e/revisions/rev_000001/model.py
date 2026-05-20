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
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.225
BODY_D = 0.092
BODY_H = 0.142

PANEL_W = 0.128
PANEL_D = 0.040
PANEL_T = 0.008


def _rounded_block(width: float, depth: float, height: float, radius: float):
    return cq.Workplane("XY").box(width, depth, height).edges().fillet(radius)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="battery_speaker")

    shell_rubber = model.material("shell_rubber", rgba=(0.17, 0.18, 0.20, 1.0))
    panel_black = model.material("panel_black", rgba=(0.09, 0.10, 0.11, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.25, 0.27, 0.29, 1.0))
    dial_dark = model.material("dial_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    control_gray = model.material("control_gray", rgba=(0.33, 0.35, 0.37, 1.0))
    icon_gray = model.material("icon_gray", rgba=(0.73, 0.75, 0.77, 1.0))
    strap_black = model.material("strap_black", rgba=(0.08, 0.08, 0.09, 1.0))

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(_rounded_block(BODY_W, BODY_D, BODY_H, 0.016), "speaker_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
        material=shell_rubber,
        name="shell_body",
    )
    shell.visual(
        Box((0.014, 0.016, 0.003)),
        origin=Origin(xyz=(-0.082, -0.008, BODY_H + 0.0015)),
        material=shell_rubber,
        name="strap_pad_0",
    )
    shell.visual(
        Box((0.014, 0.016, 0.003)),
        origin=Origin(xyz=(0.082, -0.008, BODY_H + 0.0015)),
        material=shell_rubber,
        name="strap_pad_1",
    )

    grille = model.part("grille")
    grille.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.166, 0.094),
                0.0025,
                hole_diameter=0.0038,
                pitch=(0.0075, 0.0075),
                frame=0.007,
                corner_radius=0.010,
                stagger=True,
                center=False,
            ),
            "speaker_grille",
        ),
        material=grille_dark,
        name="grille_face",
    )
    model.articulation(
        "shell_to_grille",
        ArticulationType.FIXED,
        parent=shell,
        child=grille,
        origin=Origin(
            xyz=(0.0, BODY_D * 0.5, 0.068),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        mesh_from_cadquery(_rounded_block(PANEL_W, PANEL_D, PANEL_T, 0.0045), "speaker_control_panel"),
        origin=Origin(xyz=(0.0, 0.0, PANEL_T * 0.5)),
        material=panel_black,
        name="panel_plate",
    )
    model.articulation(
        "shell_to_control_panel",
        ArticulationType.FIXED,
        parent=shell,
        child=control_panel,
        origin=Origin(xyz=(0.0, 0.004, BODY_H - 0.0002)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="cylindrical",
                edge_radius=0.0014,
                grip=KnobGrip(style="knurled", count=28, depth=0.0007, helix_angle_deg=18.0),
                center=False,
            ),
            "speaker_dial",
        ),
        material=dial_dark,
        name="dial_body",
    )
    dial.visual(
        Box((0.003, 0.010, 0.0012)),
        origin=Origin(xyz=(0.012, 0.0, 0.0186)),
        material=icon_gray,
        name="indicator",
    )
    model.articulation(
        "panel_to_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, PANEL_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        Cylinder(radius=0.0055, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0016)),
        material=control_gray,
        name="button_cap",
    )
    model.articulation(
        "panel_to_menu_button_0",
        ArticulationType.PRISMATIC,
        parent=control_panel,
        child=menu_button_0,
        origin=Origin(xyz=(0.035, -0.010, PANEL_T)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.10, lower=0.0, upper=0.0022),
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        Cylinder(radius=0.0055, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0016)),
        material=control_gray,
        name="button_cap",
    )
    model.articulation(
        "panel_to_menu_button_1",
        ArticulationType.PRISMATIC,
        parent=control_panel,
        child=menu_button_1,
        origin=Origin(xyz=(0.051, -0.010, PANEL_T)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.10, lower=0.0, upper=0.0022),
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Cylinder(radius=0.0018, length=0.018),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=control_gray,
        name="rocker_pivot",
    )
    power_rocker.visual(
        Box((0.019, 0.012, 0.0042)),
        origin=Origin(xyz=(0.0, 0.0, 0.0034)),
        material=control_gray,
        name="rocker_body",
    )
    power_rocker.visual(
        Box((0.010, 0.006, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0009)),
        material=control_gray,
        name="rocker_base",
    )
    power_rocker.visual(
        Box((0.006, 0.003, 0.001)),
        origin=Origin(xyz=(0.0, 0.0045, 0.0060)),
        material=icon_gray,
        name="rocker_mark",
    )
    model.articulation(
        "panel_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=control_panel,
        child=power_rocker,
        origin=Origin(xyz=(0.046, 0.008, PANEL_T + 0.0018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.24, upper=0.24),
    )

    strap = model.part("strap")
    strap.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(-0.083, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=strap_black,
        name="pivot_0",
    )
    strap.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(0.083, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=strap_black,
        name="pivot_1",
    )
    strap.visual(
        Box((0.012, 0.050, 0.008)),
        origin=Origin(xyz=(-0.076, -0.024, 0.006)),
        material=strap_black,
        name="arm_0",
    )
    strap.visual(
        Box((0.012, 0.050, 0.008)),
        origin=Origin(xyz=(0.076, -0.024, 0.006)),
        material=strap_black,
        name="arm_1",
    )
    strap.visual(
        Box((0.140, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -0.040, 0.014)),
        material=strap_black,
        name="strap_grip",
    )
    model.articulation(
        "shell_to_strap",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=strap,
        origin=Origin(xyz=(0.0, -0.008, BODY_H + 0.0048)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.4, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    grille = object_model.get_part("grille")
    control_panel = object_model.get_part("control_panel")
    dial = object_model.get_part("dial")
    power_rocker = object_model.get_part("power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    strap = object_model.get_part("strap")

    dial_spin = object_model.get_articulation("panel_to_dial")
    rocker_joint = object_model.get_articulation("panel_to_power_rocker")
    button_0_joint = object_model.get_articulation("panel_to_menu_button_0")
    button_1_joint = object_model.get_articulation("panel_to_menu_button_1")
    strap_joint = object_model.get_articulation("shell_to_strap")

    ctx.expect_gap(
        control_panel,
        grille,
        axis="z",
        min_gap=0.018,
        name="control_panel_clearly_above_grille",
    )
    ctx.expect_overlap(
        dial,
        control_panel,
        axes="xy",
        min_overlap=0.026,
        elem_b="panel_plate",
        name="dial_stays_centered_on_panel",
    )
    ctx.expect_gap(
        power_rocker,
        menu_button_0,
        axis="y",
        min_gap=0.004,
        name="rocker_stays_distinct_from_menu_button_0",
    )
    ctx.expect_gap(
        power_rocker,
        menu_button_1,
        axis="y",
        min_gap=0.004,
        name="rocker_stays_distinct_from_menu_button_1",
    )
    ctx.expect_gap(
        menu_button_1,
        menu_button_0,
        axis="x",
        min_gap=0.004,
        name="menu_buttons_stay_separate",
    )

    dial_indicator_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator"))
    with ctx.pose({dial_spin: math.pi * 0.5}):
        dial_indicator_turned = _aabb_center(ctx.part_element_world_aabb(dial, elem="indicator"))
    ctx.check(
        "dial_rotates_continuously_about_local_axis",
        dial_indicator_rest is not None
        and dial_indicator_turned is not None
        and math.hypot(
            dial_indicator_turned[0] - dial_indicator_rest[0],
            dial_indicator_turned[1] - dial_indicator_rest[1],
        )
        > 0.010,
        details=f"rest={dial_indicator_rest}, turned={dial_indicator_turned}",
    )

    button_0_rest = ctx.part_world_position(menu_button_0)
    button_1_rest = ctx.part_world_position(menu_button_1)
    with ctx.pose({button_0_joint: 0.0022}):
        button_0_pressed = ctx.part_world_position(menu_button_0)
        button_1_static_during_0 = ctx.part_world_position(menu_button_1)
    with ctx.pose({button_1_joint: 0.0022}):
        button_1_pressed = ctx.part_world_position(menu_button_1)
        button_0_static_during_1 = ctx.part_world_position(menu_button_0)
    ctx.check(
        "menu_button_0_depresses_independently",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_static_during_0 is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.0015
        and abs(button_1_static_during_0[2] - button_1_rest[2]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_static={button_1_static_during_0}"
        ),
    )
    ctx.check(
        "menu_button_1_depresses_independently",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_0_rest is not None
        and button_0_static_during_1 is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.0015
        and abs(button_0_static_during_1[2] - button_0_rest[2]) < 1e-6,
        details=(
            f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}, "
            f"button_0_rest={button_0_rest}, button_0_static={button_0_static_during_1}"
        ),
    )

    with ctx.pose({rocker_joint: -0.24}):
        rocker_mark_low = _aabb_center(ctx.part_element_world_aabb(power_rocker, elem="rocker_mark"))
    with ctx.pose({rocker_joint: 0.24}):
        rocker_mark_high = _aabb_center(ctx.part_element_world_aabb(power_rocker, elem="rocker_mark"))
    ctx.check(
        "power_rocker_tilts_about_short_pivot",
        rocker_mark_low is not None
        and rocker_mark_high is not None
        and abs(rocker_mark_high[2] - rocker_mark_low[2]) > 0.002,
        details=f"low={rocker_mark_low}, high={rocker_mark_high}",
    )

    strap_rest = _aabb_center(ctx.part_element_world_aabb(strap, elem="strap_grip"))
    with ctx.pose({strap_joint: 1.35}):
        strap_raised = _aabb_center(ctx.part_element_world_aabb(strap, elem="strap_grip"))
    ctx.check(
        "strap_swings_up_from_top_pivots",
        strap_rest is not None
        and strap_raised is not None
        and strap_raised[2] > strap_rest[2] + 0.020,
        details=f"rest={strap_rest}, raised={strap_raised}",
    )

    return ctx.report()


object_model = build_object_model()
