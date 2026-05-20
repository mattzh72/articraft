from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Y")
        .fillet(min(radius, width * 0.45, height * 0.45))
    )


def _build_base_plate() -> cq.Workplane:
    base = _rounded_box(0.290, 0.220, 0.012, 0.020)
    center_pad = _rounded_box(0.120, 0.090, 0.006, 0.010).translate((0.0, 0.0, 0.009))
    return base.union(center_pad)


def _build_stand() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(0.040).extrude(0.020)
    column = _rounded_box(0.088, 0.038, 0.285, 0.014).translate((0.0, 0.014, 0.1625))
    neck = _rounded_box(0.116, 0.046, 0.060, 0.012).translate((0.0, 0.010, 0.285))
    head = _rounded_box(0.160, 0.060, 0.106, 0.014).translate((0.0, 0.012, 0.338))
    front_pad = _rounded_box(0.104, 0.028, 0.084, 0.008).translate((0.0, -0.026, 0.338))
    return collar.union(column).union(neck).union(head).union(front_pad)


def _build_display_shell() -> cq.Workplane:
    outer_width = 0.618
    outer_height = 0.378
    edge_depth = 0.014
    pocket_width = 0.592
    pocket_height = 0.330
    pocket_depth = 0.006
    pocket_z = 0.007

    shell = _rounded_box(outer_width, edge_depth, outer_height, 0.022)
    screen_pocket = _rounded_box(pocket_width, pocket_depth, pocket_height, 0.010).translate(
        (0.0, -0.004, pocket_z)
    )
    rear_bulge = _rounded_box(0.250, 0.032, 0.188, 0.012).translate((0.0, 0.023, 0.005))
    lower_mount = _rounded_box(0.120, 0.022, 0.060, 0.008).translate((0.0, 0.014, -0.055))
    return shell.cut(screen_pocket).union(rear_bulge).union(lower_mount)


def _aabb_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        upper[0] - lower[0],
        upper[1] - lower[1],
        upper[2] - lower[2],
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_monitor")

    dark_graphite = model.material("dark_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    shell_black = model.material("shell_black", rgba=(0.09, 0.10, 0.11, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.04, 0.05, 1.0))
    screen_tint = model.material("screen_tint", rgba=(0.08, 0.13, 0.18, 0.35))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_build_base_plate(), "monitor_base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_graphite,
        name="base",
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_build_stand(), "monitor_stand"),
        material=dark_graphite,
        name="stand_body",
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Box((0.090, 0.024, 0.086)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material=satin_black,
        name="knuckle",
    )
    tilt_head.visual(
        Box((0.120, 0.030, 0.108)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=satin_black,
        name="arm",
    )
    tilt_head.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="pivot_ring",
    )
    tilt_head.visual(
        Box((0.090, 0.014, 0.090)),
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        material=satin_black,
        name="pivot_plate",
    )

    display_shell = model.part("display_shell")
    display_shell.visual(
        mesh_from_cadquery(_build_display_shell(), "monitor_display_shell"),
        origin=Origin(xyz=(0.0, -0.039, 0.0)),
        material=shell_black,
        name="shell",
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.588, 0.002, 0.326)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=screen_black,
        name="screen_panel",
    )
    screen.visual(
        Box((0.588, 0.0008, 0.326)),
        origin=Origin(xyz=(0.0, -0.0014, 0.0)),
        material=screen_tint,
        name="screen_glass",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.022, 0.011, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=control_black,
        name="rocker",
    )

    menu_button_0 = model.part("menu_button_0")
    menu_button_0.visual(
        Cylinder(radius=0.0042, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, -0.0016)),
        material=control_black,
        name="button",
    )

    menu_button_1 = model.part("menu_button_1")
    menu_button_1.visual(
        Cylinder(radius=0.0042, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, -0.0016)),
        material=control_black,
        name="button",
    )

    menu_button_2 = model.part("menu_button_2")
    menu_button_2.visual(
        Cylinder(radius=0.0042, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, -0.0016)),
        material=control_black,
        name="button",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5),
    )

    model.articulation(
        "stand_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tilt_head,
        origin=Origin(xyz=(0.0, -0.044, 0.338)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-18.0),
            upper=math.radians(18.0),
        ),
    )

    model.articulation(
        "display_pivot",
        ArticulationType.CONTINUOUS,
        parent=tilt_head,
        child=display_shell,
        origin=Origin(xyz=(0.0, -0.042, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0),
    )

    model.articulation(
        "screen_mount",
        ArticulationType.FIXED,
        parent=display_shell,
        child=screen,
        origin=Origin(xyz=(0.0, -0.0414, 0.007)),
    )

    model.articulation(
        "power_switch",
        ArticulationType.REVOLUTE,
        parent=display_shell,
        child=power_rocker,
        origin=Origin(xyz=(0.192, -0.040, -0.189)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=-0.30,
            upper=0.30,
        ),
    )

    button_x = (0.162, 0.140, 0.118)
    for part, name, x in (
        (menu_button_0, "menu_button_0_press", button_x[0]),
        (menu_button_1, "menu_button_1_press", button_x[1]),
        (menu_button_2, "menu_button_2_press", button_x[2]),
    ):
        model.articulation(
            name,
            ArticulationType.PRISMATIC,
            parent=display_shell,
            child=part,
            origin=Origin(xyz=(x, -0.040, -0.189)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=0.0008,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    display_shell = object_model.get_part("display_shell")
    screen = object_model.get_part("screen")
    power_rocker = object_model.get_part("power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    menu_button_2 = object_model.get_part("menu_button_2")

    swivel = object_model.get_articulation("base_swivel")
    tilt = object_model.get_articulation("stand_tilt")
    pivot = object_model.get_articulation("display_pivot")
    rocker_joint = object_model.get_articulation("power_switch")
    button_joint_0 = object_model.get_articulation("menu_button_0_press")
    button_joint_1 = object_model.get_articulation("menu_button_1_press")
    button_joint_2 = object_model.get_articulation("menu_button_2_press")

    ctx.expect_within(
        screen,
        display_shell,
        axes="xz",
        inner_elem="screen_panel",
        outer_elem="shell",
        margin=0.003,
        name="screen stays within the display shell opening",
    )
    ctx.expect_origin_gap(
        power_rocker,
        menu_button_0,
        axis="x",
        min_gap=0.020,
        name="power rocker stays distinct from the first menu button",
    )
    ctx.expect_origin_gap(
        menu_button_0,
        menu_button_1,
        axis="x",
        min_gap=0.020,
        name="first and second menu buttons stay distinct",
    )
    ctx.expect_origin_gap(
        menu_button_1,
        menu_button_2,
        axis="x",
        min_gap=0.020,
        name="second and third menu buttons stay distinct",
    )

    tilt_limits = tilt.motion_limits
    neutral_display_aabb = ctx.part_world_aabb(display_shell)
    neutral_stand_aabb = ctx.part_world_aabb(stand)
    neutral_clearance = None
    if neutral_display_aabb is not None and neutral_stand_aabb is not None:
        neutral_clearance = neutral_stand_aabb[0][1] - neutral_display_aabb[1][1]
    ctx.check(
        "display clears the stand housing at neutral",
        neutral_clearance is not None and neutral_clearance > 0.001,
        details=f"neutral_clearance={neutral_clearance}",
    )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.lower}):
            lower_aabb = ctx.part_world_aabb(display_shell)
            lower_pos = ctx.part_world_position(display_shell)
        with ctx.pose({tilt: tilt_limits.upper}):
            upper_aabb = ctx.part_world_aabb(display_shell)
            upper_pos = ctx.part_world_position(display_shell)
        ctx.check(
            "positive tilt raises the display",
            lower_pos is not None and upper_pos is not None and upper_pos[2] > lower_pos[2] + 0.02,
            details=f"lower_pos={lower_pos}, upper_pos={upper_pos}",
        )

    rest_aabb = ctx.part_world_aabb(display_shell)
    with ctx.pose({pivot: math.pi / 2.0}):
        portrait_aabb = ctx.part_world_aabb(display_shell)
    rest_dims = _aabb_dims(rest_aabb)
    portrait_dims = _aabb_dims(portrait_aabb)
    ctx.check(
        "display pivots to portrait orientation",
        rest_dims is not None
        and portrait_dims is not None
        and rest_dims[0] > rest_dims[2] + 0.18
        and portrait_dims[2] > portrait_dims[0] + 0.18,
        details=f"rest_dims={rest_dims}, portrait_dims={portrait_dims}",
    )

    rest_pos = ctx.part_world_position(display_shell)
    with ctx.pose({swivel: math.pi / 2.0}):
        swiveled_pos = ctx.part_world_position(display_shell)
    ctx.check(
        "stand swivels around the base",
        rest_pos is not None and swiveled_pos is not None and swiveled_pos[0] > rest_pos[0] + 0.04,
        details=f"rest_pos={rest_pos}, swiveled_pos={swiveled_pos}",
    )

    rocker_limits = rocker_joint.motion_limits
    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        with ctx.pose({rocker_joint: rocker_limits.lower}):
            rocker_low_center = _aabb_center(ctx.part_world_aabb(power_rocker))
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            rocker_high_center = _aabb_center(ctx.part_world_aabb(power_rocker))
        ctx.check(
            "power rocker tips on its local pivot",
            rocker_low_center is not None
            and rocker_high_center is not None
            and rocker_high_center[1] > rocker_low_center[1] + 0.0012,
            details=f"rocker_low_center={rocker_low_center}, rocker_high_center={rocker_high_center}",
        )

    button_limits = button_joint_1.motion_limits
    rest_button_0 = ctx.part_world_position(menu_button_0)
    rest_button_1 = ctx.part_world_position(menu_button_1)
    rest_button_2 = ctx.part_world_position(menu_button_2)
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_joint_1: button_limits.upper}):
            pressed_button_0 = ctx.part_world_position(menu_button_0)
            pressed_button_1 = ctx.part_world_position(menu_button_1)
            pressed_button_2 = ctx.part_world_position(menu_button_2)
        ctx.check(
            "menu buttons depress independently",
            rest_button_0 is not None
            and rest_button_1 is not None
            and rest_button_2 is not None
            and pressed_button_0 is not None
            and pressed_button_1 is not None
            and pressed_button_2 is not None
            and pressed_button_1[2] > rest_button_1[2] + 0.0006
            and abs(pressed_button_0[2] - rest_button_0[2]) < 1e-6
            and abs(pressed_button_2[2] - rest_button_2[2]) < 1e-6,
            details=(
                f"rest={(rest_button_0, rest_button_1, rest_button_2)}, "
                f"pressed={(pressed_button_0, pressed_button_1, pressed_button_2)}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
