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


DISPLAY_WIDTH = 0.82
DISPLAY_HEIGHT = 0.40
SCREEN_WIDTH = 0.762
SCREEN_HEIGHT = 0.338
SCREEN_RADIUS = 1.25
DISPLAY_PIVOT_Z = 0.105
TRUNNION_Y = 0.287
DISPLAY_OFFSET_X = 0.080
CONTROL_X = DISPLAY_OFFSET_X + 0.006
CONTROL_Z = -0.199


def _beam_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.sqrt(dx * dx + dy * dy))
    mid = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _add_beam(
    part,
    name: str,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    width: float,
    thickness: float,
    material,
) -> None:
    origin, length = _beam_origin(start, end)
    part.visual(
        Box((length, width, thickness)),
        origin=origin,
        material=material,
        name=name,
    )


def _curved_panel(
    *,
    width: float,
    height: float,
    radius: float,
    front_x: float,
    depth: float,
) -> cq.Workplane:
    half_width = width * 0.5
    center_x = front_x - math.sqrt(max(radius * radius - half_width * half_width, 1e-9))
    cylinder = cq.Workplane("XY").circle(radius).extrude(height).translate((center_x, 0.0, -height * 0.5))
    clip = cq.Workplane("XY").box(depth, width, height + 0.010).translate((front_x + depth * 0.5, 0.0, 0.0))
    return cylinder.intersect(clip)


def _curved_frame(
    *,
    outer_width: float,
    outer_height: float,
    inner_width: float,
    inner_height: float,
    radius: float,
    front_x: float,
    depth: float,
) -> cq.Workplane:
    outer = _curved_panel(
        width=outer_width,
        height=outer_height,
        radius=radius,
        front_x=front_x,
        depth=depth,
    )
    inner = _curved_panel(
        width=inner_width,
        height=inner_height,
        radius=radius,
        front_x=front_x - 0.004,
        depth=depth + 0.020,
    )
    return outer.cut(inner)


def _display_back_cover() -> cq.Workplane:
    cover = (
        cq.Workplane("XY")
        .box(0.064, 0.770, 0.360)
        .translate((-0.016, 0.0, 0.012))
        .edges("|Z")
        .fillet(0.018)
    )
    notch = cq.Workplane("XY").box(0.120, 0.290, 0.190).translate((-0.010, 0.0, -0.108))
    return cover.cut(notch)


def _display_pod() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.086, 0.126, 0.106)
        .translate((-0.048, 0.0, 0.072))
        .edges("|Z")
        .fillet(0.016)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_gaming_monitor")

    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    trim_black = model.material("trim_black", rgba=(0.18, 0.18, 0.19, 1.0))
    stand_black = model.material("stand_black", rgba=(0.16, 0.16, 0.17, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.24, 0.24, 0.25, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.10, 0.16, 0.18, 0.72))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.078, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=stand_black,
        name="hub",
    )
    base.visual(
        Cylinder(radius=0.041, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=metal_dark,
        name="swivel_plate",
    )
    base.visual(
        Cylinder(radius=0.027, length=0.168),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=stand_black,
        name="column",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.204)),
        material=metal_dark,
        name="top_collar",
    )

    leg_specs = (
        ("leg_0", math.radians(32.0), 0.170, 0.255),
        ("leg_1", math.radians(148.0), 0.170, 0.255),
        ("leg_2", math.radians(-90.0), 0.160, 0.245),
    )
    for index, (name, angle, center_radius, leg_length) in enumerate(leg_specs):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            Box((leg_length, 0.048, 0.014)),
            origin=Origin(
                xyz=(center_radius * c, center_radius * s, 0.007),
                rpy=(0.0, 0.0, angle),
            ),
            material=stand_black,
            name=name,
        )
        tip_radius = center_radius + leg_length * 0.5 - 0.014
        base.visual(
            Box((0.046, 0.062, 0.010)),
            origin=Origin(
                xyz=(tip_radius * c, tip_radius * s, 0.005),
                rpy=(0.0, 0.0, angle),
            ),
            material=trim_black,
            name=f"foot_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.039, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=metal_dark,
        name="lower_collar",
    )
    yoke.visual(
        Cylinder(radius=0.024, length=0.120),
        origin=Origin(xyz=(-0.044, 0.0, 0.072)),
        material=stand_black,
        name="neck",
    )
    yoke.visual(
        Box((0.064, 0.126, 0.026)),
        origin=Origin(xyz=(-0.028, 0.0, DISPLAY_PIVOT_Z)),
        material=stand_black,
        name="fork_block",
    )
    _add_beam(
        yoke,
        "arm_0",
        start=(-0.048, -0.058, DISPLAY_PIVOT_Z),
        end=(-0.058, -0.278, DISPLAY_PIVOT_Z),
        width=0.028,
        thickness=0.026,
        material=stand_black,
    )
    _add_beam(
        yoke,
        "arm_1",
        start=(-0.048, 0.058, DISPLAY_PIVOT_Z),
        end=(-0.058, 0.278, DISPLAY_PIVOT_Z),
        width=0.028,
        thickness=0.026,
        material=stand_black,
    )
    yoke.visual(
        Box((0.028, 0.030, 0.026)),
        origin=Origin(xyz=(-0.040, -0.292, DISPLAY_PIVOT_Z)),
        material=stand_black,
        name="ear_0",
    )
    yoke.visual(
        Box((0.028, 0.030, 0.026)),
        origin=Origin(xyz=(-0.040, 0.292, DISPLAY_PIVOT_Z)),
        material=stand_black,
        name="ear_1",
    )
    yoke.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, -0.304, DISPLAY_PIVOT_Z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=metal_dark,
        name="pivot_0",
    )
    yoke.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.304, DISPLAY_PIVOT_Z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=metal_dark,
        name="pivot_1",
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(
            _curved_frame(
                outer_width=DISPLAY_WIDTH,
                outer_height=DISPLAY_HEIGHT,
                inner_width=SCREEN_WIDTH,
                inner_height=SCREEN_HEIGHT,
                radius=SCREEN_RADIUS,
                front_x=0.004,
                depth=0.018,
            ),
            "monitor_bezel",
        ),
        origin=Origin(xyz=(DISPLAY_OFFSET_X, 0.0, 0.0)),
        material=shell_black,
        name="bezel",
    )
    display.visual(
        mesh_from_cadquery(
            _curved_panel(
                width=SCREEN_WIDTH + 0.004,
                height=SCREEN_HEIGHT + 0.004,
                radius=SCREEN_RADIUS,
                front_x=0.010,
                depth=0.004,
            ),
            "monitor_glass",
        ),
        origin=Origin(xyz=(DISPLAY_OFFSET_X, 0.0, 0.0)),
        material=glass_dark,
        name="screen",
    )
    display.visual(
        mesh_from_cadquery(_display_back_cover(), "monitor_back_cover"),
        origin=Origin(xyz=(DISPLAY_OFFSET_X, 0.0, 0.0)),
        material=trim_black,
        name="back_cover",
    )
    display.visual(
        mesh_from_cadquery(_display_pod(), "monitor_pod"),
        origin=Origin(xyz=(DISPLAY_OFFSET_X, 0.0, 0.0)),
        material=shell_black,
        name="rear_pod",
    )
    display.visual(
        Box((0.040, 0.220, 0.018)),
        origin=Origin(xyz=(DISPLAY_OFFSET_X - 0.004, 0.0, -0.188)),
        material=trim_black,
        name="chin",
    )
    display.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, -TRUNNION_Y, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=metal_dark,
        name="boss_0",
    )
    display.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, TRUNNION_Y, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=metal_dark,
        name="boss_1",
    )
    display.visual(
        Box((0.028, 0.024, 0.060)),
        origin=Origin(xyz=(0.020, -0.270, 0.0)),
        material=trim_black,
        name="boss_mount_0",
    )
    display.visual(
        Box((0.028, 0.024, 0.060)),
        origin=Origin(xyz=(0.020, 0.270, 0.0)),
        material=trim_black,
        name="boss_mount_1",
    )

    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Box((0.018, 0.034, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, -0.008)),
        material=shell_black,
        name="rocker_cap",
    )
    power_rocker.visual(
        Cylinder(radius=0.0035, length=0.022),
        origin=Origin(xyz=(-0.001, 0.0, -0.0035), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=metal_dark,
        name="rocker_stub",
    )

    button_positions = (-0.030, 0.002, 0.034)
    button_parts = []
    for index, y_pos in enumerate(button_positions):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Cylinder(radius=0.0052, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, -0.0075)),
            material=trim_black,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0035, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, -0.0035)),
            material=metal_dark,
            name="button_stem",
        )
        button_parts.append((button, y_pos))

    model.articulation(
        "base_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0),
    )
    model.articulation(
        "yoke_to_display",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=display,
        origin=Origin(xyz=(0.0, 0.0, DISPLAY_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=math.radians(-7.0),
            upper=math.radians(20.0),
        ),
    )
    model.articulation(
        "display_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=display,
        child=power_rocker,
        origin=Origin(xyz=(CONTROL_X, -0.078, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=1.2,
            lower=math.radians(-12.0),
            upper=math.radians(12.0),
        ),
    )
    for index, (button, y_pos) in enumerate(button_parts):
        model.articulation(
            f"display_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(CONTROL_X, y_pos, CONTROL_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=0.002,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    display = object_model.get_part("display")
    power_rocker = object_model.get_part("power_rocker")
    menu_button_0 = object_model.get_part("menu_button_0")
    menu_button_1 = object_model.get_part("menu_button_1")
    menu_button_2 = object_model.get_part("menu_button_2")
    swivel = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_display")
    rocker_joint = object_model.get_articulation("display_to_power_rocker")
    button_joint_0 = object_model.get_articulation("display_to_menu_button_0")
    button_joint_1 = object_model.get_articulation("display_to_menu_button_1")
    button_joint_2 = object_model.get_articulation("display_to_menu_button_2")

    ctx.expect_contact(display, yoke, elem_a="boss_0", elem_b="pivot_0", name="left trunnion contacts yoke")
    ctx.expect_contact(display, yoke, elem_a="boss_1", elem_b="pivot_1", name="right trunnion contacts yoke")
    ctx.expect_contact(yoke, base, elem_a="lower_collar", elem_b="top_collar", name="swivel collar seats on base")
    ctx.expect_overlap(display, yoke, axes="z", elem_a="boss_0", elem_b="pivot_0", min_overlap=0.030, name="tilt pivots share hinge height")
    ctx.expect_origin_distance(power_rocker, menu_button_0, axes="y", min_dist=0.035, name="power rocker stays distinct from first menu button")
    ctx.expect_origin_distance(menu_button_0, menu_button_1, axes="y", min_dist=0.020, name="first two menu buttons stay distinct")
    ctx.expect_origin_distance(menu_button_1, menu_button_2, axes="y", min_dist=0.020, name="last two menu buttons stay distinct")

    for button in (menu_button_0, menu_button_1, menu_button_2):
        ctx.expect_gap(
            display,
            button,
            axis="z",
            positive_elem="chin",
            negative_elem="button_stem",
            min_gap=0.0015,
            max_gap=0.0035,
            name=f"{button.name} hangs just below the underside bezel",
        )

    rest_screen_aabb = ctx.part_element_world_aabb(display, elem="screen")
    rest_arm_aabb = ctx.part_element_world_aabb(yoke, elem="arm_0")
    with ctx.pose({swivel: math.radians(35.0)}):
        swiveled_screen_aabb = ctx.part_element_world_aabb(display, elem="screen")
        swiveled_arm_aabb = ctx.part_element_world_aabb(yoke, elem="arm_0")
    ctx.check(
        "stand swivels the whole screen assembly",
        rest_screen_aabb is not None
        and rest_arm_aabb is not None
        and swiveled_screen_aabb is not None
        and swiveled_arm_aabb is not None
        and abs((swiveled_screen_aabb[1][0] - swiveled_screen_aabb[0][0]) - (rest_screen_aabb[1][0] - rest_screen_aabb[0][0])) > 0.05
        and abs((swiveled_arm_aabb[1][1] - swiveled_arm_aabb[0][1]) - (rest_arm_aabb[1][1] - rest_arm_aabb[0][1])) > 0.01,
        details=f"rest_screen_aabb={rest_screen_aabb}, swiveled_screen_aabb={swiveled_screen_aabb}, rest_arm_aabb={rest_arm_aabb}, swiveled_arm_aabb={swiveled_arm_aabb}",
    )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        rest_aabb = ctx.part_element_world_aabb(display, elem="screen")
        with ctx.pose({tilt: tilt_limits.upper}):
            upper_aabb = ctx.part_element_world_aabb(display, elem="screen")
        ctx.check(
            "display tilts upward at upper limit",
            rest_aabb is not None
            and upper_aabb is not None
            and upper_aabb[1][0] > rest_aabb[1][0] + 0.03
            and upper_aabb[0][0] < rest_aabb[0][0] - 0.01,
            details=f"rest_aabb={rest_aabb}, upper_aabb={upper_aabb}",
        )

    button_limits = button_joint_1.motion_limits
    if button_limits is not None and button_limits.upper is not None:
        rest_button_0 = ctx.part_world_position(menu_button_0)
        rest_button_1 = ctx.part_world_position(menu_button_1)
        with ctx.pose({button_joint_1: button_limits.upper}):
            pressed_button_0 = ctx.part_world_position(menu_button_0)
            pressed_button_1 = ctx.part_world_position(menu_button_1)
            ctx.expect_gap(
                display,
                menu_button_1,
                axis="z",
                positive_elem="chin",
                negative_elem="button_stem",
                max_gap=0.001,
                max_penetration=0.0,
                name="center menu button presses flush into the underside bezel",
            )
        ctx.check(
            "menu buttons depress independently",
            rest_button_0 is not None
            and rest_button_1 is not None
            and pressed_button_0 is not None
            and pressed_button_1 is not None
            and abs(pressed_button_0[2] - rest_button_0[2]) < 1e-6
            and pressed_button_1[2] > rest_button_1[2] + 0.0015,
            details=f"rest_button_0={rest_button_0}, pressed_button_0={pressed_button_0}, rest_button_1={rest_button_1}, pressed_button_1={pressed_button_1}",
        )

    rocker_limits = rocker_joint.motion_limits
    if rocker_limits is not None and rocker_limits.lower is not None and rocker_limits.upper is not None:
        with ctx.pose({rocker_joint: rocker_limits.lower}):
            rocker_lower = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
        with ctx.pose({rocker_joint: rocker_limits.upper}):
            rocker_upper = ctx.part_element_world_aabb(power_rocker, elem="rocker_cap")
            ctx.expect_gap(
                display,
                power_rocker,
                axis="z",
                positive_elem="chin",
                negative_elem="rocker_cap",
                min_gap=0.001,
                max_gap=0.010,
                name="power rocker stays tucked under the chin",
            )
        ctx.check(
            "power rocker rotates on its local pivot",
            rocker_lower is not None
            and rocker_upper is not None
            and abs(
                ((rocker_upper[0][0] + rocker_upper[1][0]) * 0.5)
                - ((rocker_lower[0][0] + rocker_lower[1][0]) * 0.5)
            )
            > 0.002,
            details=f"rocker_lower={rocker_lower}, rocker_upper={rocker_upper}",
        )

    return ctx.report()


object_model = build_object_model()
