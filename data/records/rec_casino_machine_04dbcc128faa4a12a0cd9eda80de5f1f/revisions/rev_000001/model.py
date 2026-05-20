from __future__ import annotations

import math

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


CABINET_WIDTH = 0.385
CABINET_HEIGHT = 0.575
SHELF_Z = 0.214
SCREEN_TILT_DEG = -7.5
SCREEN_TILT_RAD = math.radians(SCREEN_TILT_DEG)
BUTTON_CENTERS = (-0.112, -0.056, 0.0, 0.056, 0.112)


def _build_cabinet_shape() -> cq.Workplane:
    profile = [
        (-0.146, 0.000),
        (0.122, 0.000),
        (0.108, CABINET_HEIGHT),
        (0.006, CABINET_HEIGHT),
        (-0.054, 0.522),
        (-0.090, 0.243),
        (-0.160, SHELF_Z),
        (-0.160, 0.176),
        (-0.145, 0.156),
        (-0.146, 0.000),
    ]
    inner_profile = [
        (-0.130, 0.014),
        (0.104, 0.014),
        (0.097, 0.552),
        (0.010, 0.552),
        (-0.040, 0.509),
        (-0.071, 0.252),
        (-0.110, 0.232),
        (-0.128, 0.205),
        (-0.130, 0.014),
    ]

    outer = cq.Workplane("YZ").polyline(profile).close().extrude(CABINET_WIDTH / 2.0, both=True)
    inner = cq.Workplane("YZ").polyline(inner_profile).close().extrude((CABINET_WIDTH - 0.030) / 2.0, both=True)
    body = outer.cut(inner)

    screen_cutter = (
        cq.Workplane("XY")
        .box(0.278, 0.140, 0.210)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), SCREEN_TILT_DEG)
        .translate((0.0, -0.072, 0.392))
    )
    tray_cutter = cq.Workplane("XY").box(0.212, 0.132, 0.078).translate((0.0, -0.100, 0.106))
    for button_x in BUTTON_CENTERS:
        pocket_cutter = cq.Workplane("XY").box(0.052, 0.024, 0.020).translate((button_x, -0.138, SHELF_Z + 0.012))
        button_cutter = cq.Workplane("XY").box(0.020, 0.014, 0.050).translate((button_x, -0.138, SHELF_Z + 0.010))
        body = body.cut(pocket_cutter)
        body = body.cut(button_cutter)

    return body.cut(screen_cutter).cut(tray_cutter)
def _build_tray_flap_shape() -> cq.Workplane:
    door = cq.Workplane("XY").box(0.182, 0.009, 0.050)
    lip = cq.Workplane("XY").box(0.134, 0.010, 0.010).translate((0.0, -0.001, 0.020))
    hinge_barrel = (
        cq.Workplane("XY")
        .cylinder(0.170, 0.005)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.0, 0.004, -0.025))
    )
    hinge_leaf = cq.Workplane("XY").box(0.168, 0.012, 0.004).translate((0.0, 0.004, -0.023))
    return door.union(lip).union(hinge_barrel).union(hinge_leaf)


def _build_tray_hinge_support_shape() -> cq.Workplane:
    bar = cq.Workplane("XY").box(0.188, 0.014, 0.008)
    left_tab = cq.Workplane("XY").box(0.010, 0.014, 0.150).translate((-0.094, 0.0, 0.071))
    right_tab = cq.Workplane("XY").box(0.010, 0.014, 0.150).translate((0.094, 0.0, 0.071))
    return bar.union(left_tab).union(right_tab)


def _build_control_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.314, 0.076, 0.010)
    for button_x in BUTTON_CENTERS:
        pocket = cq.Workplane("XY").box(0.050, 0.024, 0.004).translate((button_x, 0.0, 0.003))
        hole = cq.Workplane("XY").box(0.020, 0.014, 0.016).translate((button_x, 0.0, 0.0))
        panel = panel.cut(pocket).cut(hole)
    return panel


def _build_button_cap_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(0.046, 0.020, 0.008).edges("|Z").fillet(0.0025)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bartop_video_poker_terminal")

    cabinet_color = model.material("cabinet_color", rgba=(0.16, 0.16, 0.18, 1.0))
    trim_color = model.material("trim_color", rgba=(0.08, 0.08, 0.09, 1.0))
    screen_color = model.material("screen_color", rgba=(0.10, 0.22, 0.19, 0.92))
    button_color = model.material("button_color", rgba=(0.76, 0.12, 0.11, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_build_cabinet_shape(), "terminal_cabinet"),
        material=cabinet_color,
        name="shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_control_panel_shape(), "video_poker_button_panel"),
        origin=Origin(xyz=(0.0, -0.138, SHELF_Z + 0.005)),
        material=trim_color,
        name="control_shelf",
    )
    cabinet.visual(
        mesh_from_cadquery(_build_tray_hinge_support_shape(), "tray_hinge_support"),
        origin=Origin(xyz=(0.0, -0.115, 0.079)),
        material=trim_color,
        name="tray_hinge_support",
    )

    cabinet.visual(
        Box((0.260, 0.020, 0.190)),
        origin=Origin(xyz=(0.0, -0.071, 0.392), rpy=(SCREEN_TILT_RAD, 0.0, 0.0)),
        material=screen_color,
        name="screen_panel",
    )
    cabinet.visual(
        Box((0.284, 0.024, 0.214)),
        origin=Origin(xyz=(0.0, -0.061, 0.392), rpy=(SCREEN_TILT_RAD, 0.0, 0.0)),
        material=trim_color,
        name="screen_mount",
    )

    tray_flap = model.part("tray_flap")
    tray_flap.visual(
        mesh_from_cadquery(_build_tray_flap_shape(), "cash_tray_flap"),
        origin=Origin(xyz=(0.0, -0.003, 0.025)),
        material=trim_color,
        name="door",
    )
    model.articulation(
        "cabinet_to_tray_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=tray_flap,
        origin=Origin(xyz=(0.0, -0.129, 0.079)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    button_cap = mesh_from_cadquery(_build_button_cap_shape(), "video_poker_button_cap")
    for index, button_x in enumerate(BUTTON_CENTERS):
        button = model.part(f"play_button_{index}")
        button.visual(
            button_cap,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=button_color,
            name="cap",
        )
        button.visual(
            Box((0.016, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=button_color,
            name="stem",
        )
        button.visual(
            Box((0.030, 0.016, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, -0.0115)),
            material=button_color,
            name="retainer",
        )
        model.articulation(
            f"cabinet_to_play_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(button_x, -0.138, SHELF_Z + 0.010)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.0035),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tray_flap = object_model.get_part("tray_flap")
    tray_hinge = object_model.get_articulation("cabinet_to_tray_flap")

    closed_aabb = ctx.part_element_world_aabb(tray_flap, elem="door")
    with ctx.pose({tray_hinge: tray_hinge.motion_limits.upper}):
        open_aabb = ctx.part_element_world_aabb(tray_flap, elem="door")

    tray_closed_location_ok = (
        closed_aabb is not None
        and closed_aabb[0][0] < -0.085
        and closed_aabb[1][0] > 0.085
        and -0.145 < closed_aabb[0][1] < -0.130
        and -0.132 < closed_aabb[1][1] < -0.115
        and 0.070 < closed_aabb[0][2] < 0.090
        and 0.120 < closed_aabb[1][2] < 0.135
    )
    ctx.check(
        "tray flap closes over the recessed cash tray opening",
        tray_closed_location_ok,
        details=f"closed={closed_aabb}",
    )

    tray_swings_forward = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.020
        and open_aabb[1][2] < closed_aabb[1][2] - 0.006
    )
    ctx.check(
        "tray flap opens outward from the recess",
        tray_swings_forward,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    button_joints = [object_model.get_articulation(f"cabinet_to_play_button_{index}") for index in range(5)]
    button_parts = [object_model.get_part(f"play_button_{index}") for index in range(5)]

    for index, (button_joint, button_part) in enumerate(zip(button_joints, button_parts)):
        rest_aabb = ctx.part_element_world_aabb(button_part, elem="cap")
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed_aabb = ctx.part_element_world_aabb(button_part, elem="cap")
        ctx.check(
            f"play button {index} depresses downward",
            rest_aabb is not None
            and pressed_aabb is not None
            and pressed_aabb[1][2] < rest_aabb[1][2] - 0.0025,
            details=f"rest={rest_aabb}, pressed={pressed_aabb}",
        )

    button_0_rest = ctx.part_element_world_aabb(button_parts[0], elem="cap")
    button_1_rest = ctx.part_element_world_aabb(button_parts[1], elem="cap")
    with ctx.pose({button_joints[0]: button_joints[0].motion_limits.upper}):
        button_0_pressed = ctx.part_element_world_aabb(button_parts[0], elem="cap")
        button_1_while_button_0_pressed = ctx.part_element_world_aabb(button_parts[1], elem="cap")

    independent_buttons = (
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_while_button_0_pressed is not None
        and button_0_pressed[1][2] < button_0_rest[1][2] - 0.0025
        and abs(button_1_rest[1][2] - button_1_while_button_0_pressed[1][2]) < 1e-6
    )
    ctx.check(
        "play buttons articulate independently",
        independent_buttons,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_during_other_press={button_1_while_button_0_pressed}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
