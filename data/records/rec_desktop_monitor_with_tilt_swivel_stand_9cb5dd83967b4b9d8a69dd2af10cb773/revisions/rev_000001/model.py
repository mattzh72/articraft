from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_monitor_stand")

    matte_black = Material("matte_black", rgba=(0.006, 0.007, 0.009, 1.0))
    satin_black = Material("satin_black", rgba=(0.035, 0.037, 0.040, 1.0))
    dark_metal = Material("dark_metal", rgba=(0.11, 0.115, 0.12, 1.0))
    rubber = Material("rubber_black", rgba=(0.002, 0.002, 0.002, 1.0))
    screen_glass = Material("deep_blue_glass", rgba=(0.015, 0.030, 0.055, 1.0))
    red_accent = Material("gaming_red", rgba=(0.85, 0.02, 0.01, 1.0))
    button_mat = Material("button_black", rgba=(0.018, 0.020, 0.022, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.095, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_metal,
        name="swivel_plinth",
    )
    base.visual(
        Box((0.42, 0.070, 0.035)),
        origin=Origin(xyz=(0.145, -0.150, 0.0175), rpy=(0.0, 0.0, math.radians(-45.0))),
        material=dark_metal,
        name="foot_0",
    )
    base.visual(
        Box((0.42, 0.070, 0.035)),
        origin=Origin(xyz=(-0.145, -0.150, 0.0175), rpy=(0.0, 0.0, math.radians(-135.0))),
        material=dark_metal,
        name="foot_1",
    )
    base.visual(
        Box((0.25, 0.010, 0.006)),
        origin=Origin(xyz=(0.180, -0.180, 0.036), rpy=(0.0, 0.0, math.radians(-45.0))),
        material=red_accent,
        name="accent_0",
    )
    base.visual(
        Box((0.25, 0.010, 0.006)),
        origin=Origin(xyz=(-0.180, -0.180, 0.036), rpy=(0.0, 0.0, math.radians(-135.0))),
        material=red_accent,
        name="accent_1",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=rubber,
        name="turntable_pad",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.078, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=satin_black,
        name="turntable",
    )
    swivel.visual(
        Box((0.026, 0.110, 0.035)),
        origin=Origin(xyz=(0.067, 0.0, 0.043)),
        material=satin_black,
        name="lower_collar_0",
    )
    swivel.visual(
        Box((0.026, 0.110, 0.035)),
        origin=Origin(xyz=(-0.067, 0.0, 0.043)),
        material=satin_black,
        name="lower_collar_1",
    )
    swivel.visual(
        Box((0.140, 0.020, 0.035)),
        origin=Origin(xyz=(0.0, 0.055, 0.043)),
        material=satin_black,
        name="lower_collar_2",
    )
    swivel.visual(
        Box((0.140, 0.020, 0.035)),
        origin=Origin(xyz=(0.0, -0.055, 0.043)),
        material=satin_black,
        name="lower_collar_3",
    )
    swivel.visual(
        Box((0.018, 0.090, 0.310)),
        origin=Origin(xyz=(0.055, 0.0, 0.185)),
        material=matte_black,
        name="sleeve_side_0",
    )
    swivel.visual(
        Box((0.018, 0.090, 0.310)),
        origin=Origin(xyz=(-0.055, 0.0, 0.185)),
        material=matte_black,
        name="sleeve_side_1",
    )
    swivel.visual(
        Box((0.110, 0.014, 0.310)),
        origin=Origin(xyz=(0.0, 0.041, 0.185)),
        material=matte_black,
        name="sleeve_rear_wall",
    )
    swivel.visual(
        Box((0.110, 0.014, 0.310)),
        origin=Origin(xyz=(0.0, -0.041, 0.185)),
        material=matte_black,
        name="sleeve_front_wall",
    )
    swivel.visual(
        Box((0.024, 0.100, 0.018)),
        origin=Origin(xyz=(0.064, 0.0, 0.340)),
        material=satin_black,
        name="sleeve_lip_0",
    )
    swivel.visual(
        Box((0.024, 0.100, 0.018)),
        origin=Origin(xyz=(-0.064, 0.0, 0.340)),
        material=satin_black,
        name="sleeve_lip_1",
    )
    swivel.visual(
        Box((0.130, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.050, 0.340)),
        material=satin_black,
        name="sleeve_lip_2",
    )
    swivel.visual(
        Box((0.130, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.050, 0.340)),
        material=satin_black,
        name="sleeve_lip_3",
    )

    column = model.part("column")
    column.visual(
        Box((0.064, 0.046, 0.460)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=dark_metal,
        name="inner_mast",
    )
    column.visual(
        Box((0.018, 0.006, 0.300)),
        origin=Origin(xyz=(0.0, -0.026, -0.060)),
        material=red_accent,
        name="height_mark",
    )
    column.visual(
        Box((0.014, 0.030, 0.070)),
        origin=Origin(xyz=(0.039, 0.0, -0.120)),
        material=rubber,
        name="guide_pad_0",
    )
    column.visual(
        Box((0.014, 0.030, 0.070)),
        origin=Origin(xyz=(-0.039, 0.0, -0.120)),
        material=rubber,
        name="guide_pad_1",
    )
    column.visual(
        Box((0.030, 0.011, 0.070)),
        origin=Origin(xyz=(0.0, 0.0285, -0.120)),
        material=rubber,
        name="guide_pad_2",
    )
    column.visual(
        Box((0.030, 0.011, 0.070)),
        origin=Origin(xyz=(0.0, -0.0285, -0.120)),
        material=rubber,
        name="guide_pad_3",
    )
    column.visual(
        Box((0.160, 0.050, 0.120)),
        origin=Origin(xyz=(0.0, -0.010, 0.200)),
        material=satin_black,
        name="stand_head",
    )
    column.visual(
        Box((0.026, 0.055, 0.090)),
        origin=Origin(xyz=(0.078, -0.055, 0.230)),
        material=satin_black,
        name="yoke_0",
    )
    column.visual(
        Box((0.026, 0.055, 0.090)),
        origin=Origin(xyz=(-0.078, -0.055, 0.230)),
        material=satin_black,
        name="yoke_1",
    )
    column.visual(
        Box((0.210, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.095, 0.280)),
        material=satin_black,
        name="yoke_bridge",
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Cylinder(radius=0.025, length=0.130),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tilt_barrel",
    )
    tilt_frame.visual(
        Box((0.095, 0.110, 0.060)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=satin_black,
        name="pivot_arm",
    )
    tilt_frame.visual(
        Cylinder(radius=0.056, length=0.024),
        origin=Origin(xyz=(0.0, -0.118, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="portrait_cup",
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.840, 0.050, 0.480)),
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=matte_black,
        name="screen_shell",
    )
    screen.visual(
        Box((0.760, 0.008, 0.390)),
        origin=Origin(xyz=(0.0, -0.073, 0.018)),
        material=screen_glass,
        name="display_glass",
    )
    screen.visual(
        Box((0.500, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.071, -0.226)),
        material=red_accent,
        name="front_accent",
    )
    screen.visual(
        Box((0.320, 0.026, 0.220)),
        origin=Origin(xyz=(0.0, -0.016, 0.0)),
        material=satin_black,
        name="rear_hump",
    )
    screen.visual(
        Cylinder(radius=0.065, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="portrait_boss",
    )
    for i, (x, z) in enumerate(((-0.055, -0.045), (0.055, -0.045), (-0.055, 0.045), (0.055, 0.045))):
        screen.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, -0.004, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"vesa_screw_{i}",
        )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_mat,
        name="button_cap",
    )
    power_button.visual(
        Cylinder(radius=0.005, length=0.0015),
        origin=Origin(xyz=(0.0, -0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red_accent,
        name="power_mark",
    )

    model.articulation(
        "base_to_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "swivel_to_column",
        ArticulationType.PRISMATIC,
        parent=swivel,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.140),
    )
    model.articulation(
        "column_to_tilt_frame",
        ArticulationType.REVOLUTE,
        parent=column,
        child=tilt_frame,
        origin=Origin(xyz=(0.0, -0.065, 0.230)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "tilt_frame_to_screen",
        ArticulationType.REVOLUTE,
        parent=tilt_frame,
        child=screen,
        origin=Origin(xyz=(0.0, -0.130, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-math.pi / 2.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "screen_to_power_button",
        ArticulationType.PRISMATIC,
        parent=screen,
        child=power_button,
        origin=Origin(xyz=(0.340, -0.070, -0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    swivel = object_model.get_part("swivel")
    column = object_model.get_part("column")
    tilt_frame = object_model.get_part("tilt_frame")
    screen = object_model.get_part("screen")
    power_button = object_model.get_part("power_button")

    height_joint = object_model.get_articulation("swivel_to_column")
    swivel_joint = object_model.get_articulation("base_to_swivel")
    tilt_joint = object_model.get_articulation("column_to_tilt_frame")
    portrait_joint = object_model.get_articulation("tilt_frame_to_screen")
    button_joint = object_model.get_articulation("screen_to_power_button")

    ctx.expect_contact(
        base,
        swivel,
        elem_a="turntable_pad",
        elem_b="turntable",
        name="swivel turntable sits on the V base",
    )
    ctx.expect_within(
        column,
        swivel,
        axes="xy",
        inner_elem="inner_mast",
        margin=0.010,
        name="height mast is centered inside the sleeve",
    )
    ctx.expect_overlap(
        column,
        swivel,
        axes="z",
        elem_a="inner_mast",
        elem_b="sleeve_side_0",
        min_overlap=0.120,
        name="collapsed height mast remains inserted",
    )
    ctx.expect_gap(
        screen,
        power_button,
        axis="y",
        positive_elem="screen_shell",
        negative_elem="button_cap",
        min_gap=0.0,
        max_gap=0.004,
        name="power button protrudes from the front edge",
    )

    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({height_joint: 0.140}):
        raised_column_pos = ctx.part_world_position(column)
        ctx.expect_overlap(
            column,
            swivel,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_side_0",
            min_overlap=0.080,
            name="raised height mast stays retained",
        )
    ctx.check(
        "height slider raises the monitor",
        rest_column_pos is not None
        and raised_column_pos is not None
        and raised_column_pos[2] > rest_column_pos[2] + 0.120,
        details=f"rest={rest_column_pos}, raised={raised_column_pos}",
    )

    rest_screen_pos = ctx.part_world_position(screen)
    with ctx.pose({swivel_joint: 0.60}):
        swivel_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "base swivel yaws the screen about the stand",
        rest_screen_pos is not None
        and swivel_screen_pos is not None
        and abs(swivel_screen_pos[0] - rest_screen_pos[0]) > 0.060,
        details=f"rest={rest_screen_pos}, swiveled={swivel_screen_pos}",
    )

    with ctx.pose({tilt_joint: 0.25}):
        tilted_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "tilt hinge moves the panel around a horizontal axis",
        rest_screen_pos is not None
        and tilted_screen_pos is not None
        and abs(tilted_screen_pos[2] - rest_screen_pos[2]) > 0.020,
        details=f"rest={rest_screen_pos}, tilted={tilted_screen_pos}",
    )

    rest_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({portrait_joint: math.pi / 2.0}):
        portrait_aabb = ctx.part_world_aabb(screen)
    if rest_aabb is not None and portrait_aabb is not None:
        rest_size = tuple(rest_aabb[1][i] - rest_aabb[0][i] for i in range(3))
        portrait_size = tuple(portrait_aabb[1][i] - portrait_aabb[0][i] for i in range(3))
    else:
        rest_size = portrait_size = None
    ctx.check(
        "portrait pivot turns the broad panel upright",
        rest_size is not None
        and portrait_size is not None
        and rest_size[0] > rest_size[2]
        and portrait_size[2] > portrait_size[0],
        details=f"rest_size={rest_size}, portrait_size={portrait_size}",
    )

    rest_button_pos = ctx.part_world_position(power_button)
    with ctx.pose({button_joint: 0.008}):
        pressed_button_pos = ctx.part_world_position(power_button)
    ctx.check(
        "power button travels inward on a short guide",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.006,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
