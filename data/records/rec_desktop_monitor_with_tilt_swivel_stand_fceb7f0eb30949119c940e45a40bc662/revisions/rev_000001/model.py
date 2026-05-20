from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _curved_band_points(
    *,
    half_width: float,
    front_radius: float,
    back_radius: float,
    center_y: float,
    y_shift: float = 0.0,
    samples: int = 33,
) -> list[tuple[float, float]]:
    xs = [
        -half_width + (2.0 * half_width * i) / (samples - 1)
        for i in range(samples)
    ]
    front = [
        (
            x,
            center_y - math.sqrt(max(front_radius**2 - x**2, 0.0)) + y_shift,
        )
        for x in xs
    ]
    back = [
        (
            x,
            center_y - math.sqrt(max(back_radius**2 - x**2, 0.0)) + y_shift,
        )
        for x in reversed(xs)
    ]
    return front + back


def _curved_band_solid(
    *,
    half_width: float,
    front_radius: float,
    back_radius: float,
    center_y: float,
    height: float,
    z0: float,
    y_shift: float = 0.0,
) -> cq.Workplane:
    points = _curved_band_points(
        half_width=half_width,
        front_radius=front_radius,
        back_radius=back_radius,
        center_y=center_y,
        y_shift=y_shift,
    )
    return cq.Workplane("XY").polyline(points).close().extrude(height).translate(
        (0.0, 0.0, z0)
    )


def _build_base_shape() -> cq.Workplane:
    leg_length = 0.27
    leg_width = 0.054
    leg_height = 0.012
    foot_radius = leg_width / 2.0

    hub = cq.Workplane("XY").circle(0.057).extrude(0.028)
    neck = cq.Workplane("XY").circle(0.023).extrude(0.090).translate((0.0, 0.0, 0.028))
    swivel_pedestal = (
        cq.Workplane("XY").circle(0.036).extrude(0.028).translate((0.0, 0.0, 0.090))
    )

    base = hub.union(neck).union(swivel_pedestal)

    for angle_deg in (0.0, 120.0, 240.0):
        leg = (
            cq.Workplane("XY")
            .box(leg_length, leg_width, leg_height)
            .translate((0.5 * leg_length - 0.020, 0.0, 0.5 * leg_height))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        foot = (
            cq.Workplane("XY")
            .circle(foot_radius)
            .extrude(leg_height)
            .translate((leg_length - 0.020, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        base = base.union(leg).union(foot)

    return base


def _build_yoke_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(0.032).extrude(0.024)
    stem = cq.Workplane("XY").box(0.074, 0.020, 0.168).translate((0.0, 0.016, 0.108))

    right_arm = (
        cq.Workplane("XY")
        .box(0.300, 0.018, 0.026)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -14.0)
        .translate((0.170, 0.032, 0.197))
    )
    left_arm = (
        cq.Workplane("XY")
        .box(0.300, 0.018, 0.026)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 14.0)
        .translate((-0.170, 0.032, 0.197))
    )

    right_ear = (
        cq.Workplane("XZ")
        .circle(0.034)
        .extrude(0.020)
        .translate((0.334, -0.006, 0.245))
    )
    left_ear = (
        cq.Workplane("XZ")
        .circle(0.034)
        .extrude(0.020)
        .translate((-0.334, -0.006, 0.245))
    )
    right_bridge = cq.Workplane("XY").box(0.044, 0.040, 0.032).translate(
        (0.306, 0.012, 0.236)
    )
    left_bridge = cq.Workplane("XY").box(0.044, 0.040, 0.032).translate(
        (-0.306, 0.012, 0.236)
    )

    return (
        collar.union(stem)
        .union(right_arm)
        .union(left_arm)
        .union(right_bridge)
        .union(left_bridge)
        .union(right_ear)
        .union(left_ear)
    )


def _build_screen_shell_shape() -> cq.Workplane:
    outer_half_width = 0.405
    outer_front_radius = 2.000
    outer_back_radius = 1.930
    arc_center_y = 1.950
    shell_shift_y = -0.049
    shell_height = 0.370
    shell_bottom = -0.155

    outer = _curved_band_solid(
        half_width=outer_half_width,
        front_radius=outer_front_radius,
        back_radius=outer_back_radius,
        center_y=arc_center_y,
        height=shell_height,
        z0=shell_bottom,
        y_shift=shell_shift_y,
    )
    inner = _curved_band_solid(
        half_width=0.394,
        front_radius=1.996,
        back_radius=1.926,
        center_y=arc_center_y,
        height=0.354,
        z0=shell_bottom + 0.008,
        y_shift=shell_shift_y,
    )

    view_opening = cq.Workplane("XY").box(0.756, 0.120, 0.320).translate(
        (0.0, -0.024, 0.035)
    )
    right_pad = cq.Workplane("XY").box(0.024, 0.013, 0.055).translate(
        (0.384, -0.012, 0.0)
    )
    left_pad = cq.Workplane("XY").box(0.024, 0.013, 0.055).translate(
        (-0.384, -0.012, 0.0)
    )
    shell = outer.cut(inner).cut(view_opening).union(right_pad).union(left_pad)

    for x in (-0.042, -0.014, 0.014, 0.042):
        button_slot = cq.Workplane("XY").box(0.012, 0.018, 0.012).translate(
            (x, -0.081, -0.149)
        )
        shell = shell.cut(button_slot)

    return shell


def _build_glass_shape() -> cq.Workplane:
    glass = _curved_band_solid(
        half_width=0.381,
        front_radius=1.995,
        back_radius=1.993,
        center_y=1.950,
        height=0.336,
        z0=-0.138,
        y_shift=-0.060,
    )
    left_tab = cq.Workplane("XY").box(0.020, 0.020, 0.250).translate(
        (-0.386, -0.072, 0.030)
    )
    right_tab = cq.Workplane("XY").box(0.020, 0.020, 0.250).translate(
        (0.386, -0.072, 0.030)
    )
    return glass.union(left_tab).union(right_tab)


def _build_button_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.010, 0.006, 0.004).translate((0.0, -0.003, -0.002))
    stem = cq.Workplane("XY").box(0.006, 0.010, 0.010).translate((0.0, 0.001, 0.005))
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_gaming_monitor")

    base_mat = model.material("base_mat", rgba=(0.12, 0.12, 0.13, 1.0))
    stand_mat = model.material("stand_mat", rgba=(0.16, 0.16, 0.17, 1.0))
    shell_mat = model.material("shell_mat", rgba=(0.10, 0.10, 0.11, 1.0))
    glass_mat = model.material("glass_mat", rgba=(0.05, 0.06, 0.07, 1.0))
    button_mat = model.material("button_mat", rgba=(0.18, 0.18, 0.19, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "monitor_base"),
        material=base_mat,
        name="base_shell",
    )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_cadquery(_build_yoke_shape(), "monitor_yoke"),
        material=stand_mat,
        name="yoke_frame",
    )

    screen = model.part("screen")
    screen.visual(
        mesh_from_cadquery(_build_screen_shell_shape(), "monitor_shell"),
        material=shell_mat,
        name="shell",
    )
    screen.visual(
        mesh_from_cadquery(_build_glass_shape(), "monitor_glass"),
        material=glass_mat,
        name="glass",
    )

    button_offsets = (-0.042, -0.014, 0.014, 0.042)
    for index, x in enumerate(button_offsets):
        button = model.part(f"button_{index}")
        button.visual(
            mesh_from_cadquery(_build_button_shape(), f"monitor_button_{index}"),
            material=button_mat,
            name="button",
        )
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=screen,
            child=button,
            origin=Origin(xyz=(x, -0.081, -0.155)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.10,
                lower=0.0,
                upper=0.004,
            ),
        )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5),
    )
    model.articulation(
        "screen_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=screen,
        origin=Origin(xyz=(0.0, 0.032, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    screen = object_model.get_part("screen")
    buttons = [object_model.get_part(f"button_{i}") for i in range(4)]

    swivel = object_model.get_articulation("base_swivel")
    tilt = object_model.get_articulation("screen_tilt")
    tilt_limits = tilt.motion_limits
    button_joints = [object_model.get_articulation(f"button_{i}_press") for i in range(4)]

    ctx.allow_isolated_part(
        screen,
        reason="The monitor shell rides on visible yoke clearances at the tilt sockets; the articulated hinge support is represented with a small non-contact rest gap.",
    )
    for button in buttons:
        ctx.allow_isolated_part(
            button,
            reason="The menu button is mounted to the screen shell, and the shell-to-yoke hinge is intentionally modeled with a small visible clearance gap.",
        )

    ctx.expect_origin_gap(
        screen,
        base,
        axis="z",
        min_gap=0.240,
        name="screen sits well above the tripod base",
    )
    ctx.expect_overlap(
        screen,
        yoke,
        axes="x",
        min_overlap=0.600,
        name="yoke arms span the supported screen width",
    )

    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: 0.0}):
            rest_glass = ctx.part_element_world_aabb(screen, elem="glass")
        with ctx.pose({tilt: tilt_limits.upper}):
            tilted_glass = ctx.part_element_world_aabb(screen, elem="glass")
        ctx.check(
            "screen tilts rearward at the top",
            rest_glass is not None
            and tilted_glass is not None
            and tilted_glass[1][1] > rest_glass[1][1] + 0.030,
            details=f"rest={rest_glass}, tilted={tilted_glass}",
        )

    with ctx.pose({swivel: 0.0}):
        rest_button_pos = ctx.part_world_position(buttons[0])
    with ctx.pose({swivel: 0.75}):
        swiveled_button_pos = ctx.part_world_position(buttons[0])
    ctx.check(
        "stand swivels the screen about the vertical hub",
        rest_button_pos is not None
        and swiveled_button_pos is not None
        and math.hypot(
            swiveled_button_pos[0] - rest_button_pos[0],
            swiveled_button_pos[1] - rest_button_pos[1],
        )
        > 0.035,
        details=f"rest={rest_button_pos}, swiveled={swiveled_button_pos}",
    )

    for button, joint in zip(buttons, button_joints):
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue
        with ctx.pose({joint: 0.0}):
            rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{button.name} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
