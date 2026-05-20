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


def _make_base_mesh():
    return (
        cq.Workplane("XY")
        .box(0.240, 0.190, 0.014, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )


def _make_housing_mesh():
    outer_width = 0.546
    outer_depth = 0.046
    outer_height = 0.344
    outer_center_z = 0.052
    screen_center_z = 0.0565

    housing = (
        cq.Workplane("XY")
        .box(outer_width, outer_depth, outer_height)
        .translate((0.0, outer_depth * 0.5, outer_center_z))
        .edges("|Z")
        .fillet(0.012)
    )

    screen_recess = (
        cq.Workplane("XY")
        .box(0.530, 0.030, 0.303)
        .translate((0.0, outer_depth - 0.015, screen_center_z))
    )
    control_recess = (
        cq.Workplane("XY")
        .box(0.086, 0.014, 0.018)
        .translate((0.001, outer_depth - 0.007, -0.108))
    )

    return housing.cut(screen_recess).cut(control_recess)


def _make_neck_mesh():
    lower_spine = (
        cq.Workplane("XY")
        .box(0.050, 0.028, 0.150, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.007)
        .translate((0.0, -0.015, 0.012))
        .val()
    )
    upper_spine = (
        cq.Workplane("XY")
        .box(0.040, 0.024, 0.064, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, -0.012, 0.156))
        .val()
    )
    tilt_head = (
        cq.Workplane("XY")
        .box(0.086, 0.020, 0.034, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
        .translate((0.0, -0.010, 0.212))
        .val()
    )
    return cq.Workplane("XY").newObject([lower_spine.fuse(upper_spine).fuse(tilt_head)])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_monitor")

    base_dark = model.material("base_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.05, 0.08, 0.10, 1.0))
    button_dark = model.material("button_dark", rgba=(0.17, 0.18, 0.19, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_mesh(), "monitor_base"),
        material=base_dark,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=base_dark,
        name="swivel_plinth",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=base_dark,
        name="swivel_hub",
    )
    neck.visual(
        mesh_from_cadquery(_make_neck_mesh(), "monitor_neck"),
        origin=Origin(),
        material=base_dark,
        name="shroud",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_mesh(), "monitor_housing"),
        material=shell_dark,
        name="shell",
    )
    housing.visual(
        Box((0.490, 0.006, 0.284)),
        origin=Origin(xyz=(0.0, 0.018, 0.0565)),
        material=trim_dark,
        name="inner_frame",
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.530, 0.003, 0.299)),
        material=screen_dark,
        name="panel",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.094, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.0078, 0.0)),
        material=trim_dark,
        name="carrier",
    )
    control_panel.visual(
        Box((0.036, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.004, 0.005)),
        material=trim_dark,
        name="mount_tab",
    )

    button_x_positions = (-0.027, -0.009, 0.009, 0.027)
    for index, local_x in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.012, 0.003, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=button_dark,
            name="cap",
        )
        button.visual(
            Box((0.006, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_dark,
            name="stem",
        )
        model.articulation(
            f"control_panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(local_x, 0.0042, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.03,
                lower=0.0,
                upper=0.0016,
            ),
        )

    model.articulation(
        "base_to_neck",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5),
    )
    model.articulation(
        "neck_to_housing",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.252)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.20,
            upper=0.45,
        ),
    )
    model.articulation(
        "housing_to_screen",
        ArticulationType.FIXED,
        parent=housing,
        child=screen,
        origin=Origin(xyz=(0.0, 0.0425, 0.0565)),
    )
    model.articulation(
        "housing_to_control_panel",
        ArticulationType.FIXED,
        parent=housing,
        child=control_panel,
        origin=Origin(xyz=(0.0, 0.038, -0.128)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    screen = object_model.get_part("screen")
    button_0 = object_model.get_part("button_0")
    swivel = object_model.get_articulation("base_to_neck")
    tilt = object_model.get_articulation("neck_to_housing")
    button_joint = object_model.get_articulation("control_panel_to_button_0")

    ctx.expect_within(
        screen,
        housing,
        axes="xz",
        margin=0.0,
        name="screen stays within housing face",
    )
    ctx.expect_gap(
        screen,
        base,
        axis="z",
        min_gap=0.14,
        name="screen clears desk base vertically",
    )
    ctx.expect_origin_gap(
        screen,
        button_0,
        axis="z",
        min_gap=0.12,
        name="button bank sits below the screen center",
    )

    button_positions = [
        ctx.part_world_position(object_model.get_part(f"button_{index}"))
        for index in range(4)
    ]
    centered_bank = all(position is not None for position in button_positions)
    if centered_bank:
        x_positions = [position[0] for position in button_positions if position is not None]
        z_positions = [position[2] for position in button_positions if position is not None]
        spacings = [x_positions[index + 1] - x_positions[index] for index in range(3)]
        centered_bank = (
            max(abs(z - z_positions[0]) for z in z_positions) < 0.001
            and all(0.015 <= spacing <= 0.021 for spacing in spacings)
        )
    ctx.check(
        "menu buttons form a small aligned bank",
        centered_bank,
        details=f"button_positions={button_positions}",
    )

    rest_screen = ctx.part_world_position(screen)
    with ctx.pose({tilt: 0.45}):
        tilted_screen = ctx.part_world_position(screen)
    ctx.check(
        "screen tilts backward at positive angle",
        rest_screen is not None
        and tilted_screen is not None
        and tilted_screen[1] < rest_screen[1] - 0.015
        and tilted_screen[2] > rest_screen[2] + 0.008,
        details=f"rest={rest_screen}, tilted={tilted_screen}",
    )

    with ctx.pose({swivel: math.pi / 4.0}):
        swiveled_screen = ctx.part_world_position(screen)
    ctx.check(
        "stand swivels around the vertical column",
        rest_screen is not None
        and swiveled_screen is not None
        and abs(swiveled_screen[0]) > 0.02
        and abs(swiveled_screen[2] - rest_screen[2]) < 0.002,
        details=f"rest={rest_screen}, swiveled={swiveled_screen}",
    )

    rest_button = ctx.part_world_position(button_0)
    with ctx.pose({button_joint: 0.0016}):
        pressed_button = ctx.part_world_position(button_0)
    ctx.check(
        "menu button presses inward",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1] < rest_button[1] - 0.001,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
