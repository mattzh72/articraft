from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="design_monitor")

    shell_dark = model.material("shell_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.23, 0.24, 0.26, 1.0))
    stand_metal = model.material("stand_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.04, 0.05, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.24, 0.012)),
        origin=Origin(xyz=(0.0, -0.015, 0.006)),
        material=stand_metal,
        name="base_plate",
    )
    base.visual(
        Box((0.24, 0.16, 0.014)),
        origin=Origin(xyz=(0.0, -0.020, 0.018)),
        material=shell_mid,
        name="base_puck",
    )
    base.visual(
        Box((0.082, 0.060, 0.045)),
        origin=Origin(xyz=(0.0, -0.055, 0.048)),
        material=stand_metal,
        name="column_pedestal",
    )
    base.visual(
        Box((0.055, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, -0.040, 0.032)),
        material=stand_metal,
        name="base_neck",
    )
    base.visual(
        Box((0.009, 0.032, 0.400)),
        origin=Origin(xyz=(-0.0255, -0.055, 0.270)),
        material=stand_metal,
        name="sleeve_left",
    )
    base.visual(
        Box((0.009, 0.032, 0.400)),
        origin=Origin(xyz=(0.0255, -0.055, 0.270)),
        material=stand_metal,
        name="sleeve_right",
    )
    base.visual(
        Box((0.042, 0.004, 0.400)),
        origin=Origin(xyz=(0.0, -0.041, 0.270)),
        material=stand_metal,
        name="sleeve_front",
    )
    base.visual(
        Box((0.042, 0.004, 0.400)),
        origin=Origin(xyz=(0.0, -0.069, 0.270)),
        material=stand_metal,
        name="sleeve_back",
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((0.038, 0.020, 0.600)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=shell_mid,
        name="inner_mast",
    )
    inner_column.visual(
        Box((0.004, 0.016, 0.220)),
        origin=Origin(xyz=(-0.019, 0.0, -0.030)),
        material=trim_dark,
        name="guide_pad_0",
    )
    inner_column.visual(
        Box((0.004, 0.016, 0.220)),
        origin=Origin(xyz=(0.019, 0.0, -0.030)),
        material=trim_dark,
        name="guide_pad_1",
    )
    inner_column.visual(
        Box((0.094, 0.020, 0.048)),
        origin=Origin(xyz=(0.0, -0.010, 0.381)),
        material=shell_mid,
        name="head_block",
    )
    inner_column.visual(
        Box((0.052, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.012, 0.403)),
        material=stand_metal,
        name="hinge_cap",
    )
    inner_column.visual(
        Box((0.016, 0.024, 0.032)),
        origin=Origin(xyz=(-0.033, -0.008, 0.389)),
        material=stand_metal,
        name="hinge_left_cheek",
    )
    inner_column.visual(
        Box((0.016, 0.024, 0.032)),
        origin=Origin(xyz=(0.033, -0.008, 0.389)),
        material=stand_metal,
        name="hinge_right_cheek",
    )

    model.articulation(
        "base_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_column,
        origin=Origin(xyz=(0.0, -0.055, 0.470)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=0.140,
        ),
    )

    tilt_carrier = model.part("tilt_carrier")
    tilt_carrier.visual(
        Box((0.072, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material=stand_metal,
        name="bridge_arm",
    )
    tilt_carrier.visual(
        Box((0.024, 0.004, 0.034)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material=stand_metal,
        name="pivot_stem",
    )
    tilt_carrier.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.0, 0.027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_metal,
        name="pivot_plate",
    )

    model.articulation(
        "inner_column_to_tilt_carrier",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=tilt_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-0.35,
            upper=0.28,
        ),
    )

    display = model.part("display")
    display.visual(
        Box((0.680, 0.020, 0.420)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=shell_mid,
        name="rear_cover",
    )
    display.visual(
        Box((0.680, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, 0.010, 0.190)),
        material=shell_dark,
        name="top_bezel",
    )
    display.visual(
        Box((0.680, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.010, -0.185)),
        material=shell_dark,
        name="bottom_bezel",
    )
    display.visual(
        Box((0.034, 0.012, 0.348)),
        origin=Origin(xyz=(-0.323, 0.010, 0.004)),
        material=shell_dark,
        name="side_bezel_0",
    )
    display.visual(
        Box((0.034, 0.012, 0.348)),
        origin=Origin(xyz=(0.323, 0.010, 0.004)),
        material=shell_dark,
        name="side_bezel_1",
    )
    display.visual(
        Box((0.632, 0.006, 0.352)),
        origin=Origin(xyz=(0.0, 0.007, 0.018)),
        material=screen_black,
        name="screen",
    )
    display.visual(
        Box((0.240, 0.028, 0.160)),
        origin=Origin(xyz=(0.0, -0.022, -0.010)),
        material=shell_dark,
        name="rear_bulge",
    )
    display.visual(
        Cylinder(radius=0.044, length=0.012),
        origin=Origin(xyz=(0.0, -0.032, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="mount_boss",
    )
    display.visual(
        Box((0.220, 0.010, 0.016)),
        origin=Origin(xyz=(0.110, 0.013, -0.184)),
        material=shell_mid,
        name="control_shelf",
    )

    model.articulation(
        "tilt_carrier_to_display",
        ArticulationType.CONTINUOUS,
        parent=tilt_carrier,
        child=display,
        origin=Origin(xyz=(0.0, 0.068, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=8.0),
    )

    button_x_positions = (0.192, 0.162, 0.132, 0.102, 0.066)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"button_{index}")
        if index < 4:
            button.visual(
                Box((0.013, 0.006, 0.010)),
                origin=Origin(xyz=(0.0, 0.025, 0.0)),
                material=trim_dark,
                name="button_cap",
            )
            button.visual(
                Box((0.007, 0.006, 0.008)),
                origin=Origin(xyz=(0.0, 0.021, 0.0)),
                material=shell_mid,
                name="guide_stem",
            )
        else:
            button.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=trim_dark,
                name="button_cap",
            )
            button.visual(
                Cylinder(radius=0.004, length=0.006),
                origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=shell_mid,
                name="guide_stem",
            )

        model.articulation(
            f"display_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, -0.184)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.04,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    inner_column = object_model.get_part("inner_column")
    display = object_model.get_part("display")

    column_slide = object_model.get_articulation("base_to_inner_column")
    tilt = object_model.get_articulation("inner_column_to_tilt_carrier")
    spin = object_model.get_articulation("tilt_carrier_to_display")
    button_names = [f"button_{index}" for index in range(5)]
    buttons = [object_model.get_part(name) for name in button_names]
    button_joints = [object_model.get_articulation(f"display_to_button_{index}") for index in range(5)]

    slide_limits = column_slide.motion_limits
    tilt_limits = tilt.motion_limits

    if slide_limits is not None and slide_limits.upper is not None:
        ctx.expect_within(
            inner_column,
            base,
            axes="y",
            inner_elem="inner_mast",
            outer_elem="sleeve_left",
            margin=0.006,
            name="inner mast stays aligned with the stand sleeve depth",
        )
        ctx.expect_gap(
            inner_column,
            base,
            axis="x",
            positive_elem="guide_pad_0",
            negative_elem="sleeve_left",
            min_gap=0.0,
            max_gap=0.001,
            name="left guide pad rides the left sleeve wall",
        )
        ctx.expect_gap(
            base,
            inner_column,
            axis="x",
            positive_elem="sleeve_right",
            negative_elem="guide_pad_1",
            min_gap=0.0,
            max_gap=0.001,
            name="right guide pad rides the right sleeve wall",
        )
        ctx.expect_overlap(
            inner_column,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_front",
            min_overlap=0.18,
            name="collapsed inner mast remains deeply inserted in the sleeve",
        )
        rest_pos = ctx.part_world_position(display)
        with ctx.pose({column_slide: slide_limits.upper}):
            ctx.expect_gap(
                inner_column,
                base,
                axis="x",
                positive_elem="guide_pad_0",
                negative_elem="sleeve_left",
                min_gap=0.0,
                max_gap=0.001,
                name="left guide pad stays on the sleeve wall at full height",
            )
            ctx.expect_gap(
                base,
                inner_column,
                axis="x",
                positive_elem="sleeve_right",
                negative_elem="guide_pad_1",
                min_gap=0.0,
                max_gap=0.001,
                name="right guide pad stays on the sleeve wall at full height",
            )
            ctx.expect_overlap(
                inner_column,
                base,
                axes="z",
                elem_a="inner_mast",
                elem_b="sleeve_front",
                min_overlap=0.035,
                name="extended inner mast still retains insertion in the sleeve",
            )
            extended_pos = ctx.part_world_position(display)
        ctx.check(
            "display raises when the stand extends",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.10,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        rest_display_pos = ctx.part_world_position(display)
        with ctx.pose({tilt: tilt_limits.upper}):
            upper_display_pos = ctx.part_world_position(display)
        with ctx.pose({tilt: tilt_limits.lower}):
            lower_display_pos = ctx.part_world_position(display)
        ctx.check(
            "positive tilt lifts the display center",
            rest_display_pos is not None
            and upper_display_pos is not None
            and upper_display_pos[2] > rest_display_pos[2] + 0.008,
            details=f"rest={rest_display_pos}, upper={upper_display_pos}",
        )
        ctx.check(
            "negative tilt lowers the display center",
            rest_display_pos is not None
            and lower_display_pos is not None
            and lower_display_pos[2] < rest_display_pos[2] - 0.010,
            details=f"rest={rest_display_pos}, lower={lower_display_pos}",
        )

    rest_aabb = ctx.part_world_aabb(display)
    with ctx.pose({spin: math.pi / 2.0}):
        portrait_aabb = ctx.part_world_aabb(display)
    if rest_aabb is not None and portrait_aabb is not None:
        rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
        rest_dz = rest_aabb[1][2] - rest_aabb[0][2]
        portrait_dx = portrait_aabb[1][0] - portrait_aabb[0][0]
        portrait_dz = portrait_aabb[1][2] - portrait_aabb[0][2]
        ctx.check(
            "display rotates into portrait orientation",
            rest_dx > rest_dz + 0.18 and portrait_dz > portrait_dx + 0.18,
            details=(
                f"rest spans=({rest_dx:.3f}, {rest_dz:.3f}), "
                f"portrait spans=({portrait_dx:.3f}, {portrait_dz:.3f})"
            ),
        )

    rest_button_positions = {name: ctx.part_world_position(part) for name, part in zip(button_names, buttons)}
    for name, button, joint in zip(button_names, buttons, button_joints):
        limits = joint.motion_limits
        ctx.expect_gap(
            button,
            display,
            axis="y",
            positive_elem="guide_stem",
            negative_elem="control_shelf",
            min_gap=0.0,
            max_gap=0.001,
            name=f"{name} guide stem seats against the control shelf",
        )
        if limits is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.upper}):
            pressed_positions = {
                other_name: ctx.part_world_position(other_button)
                for other_name, other_button in zip(button_names, buttons)
            }
        rest_pos = rest_button_positions[name]
        pressed_pos = pressed_positions[name]
        moved_ok = (
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.003
        )
        others_ok = True
        unchanged_details = []
        for other_name in button_names:
            if other_name == name:
                continue
            other_rest = rest_button_positions[other_name]
            other_pressed = pressed_positions[other_name]
            same_y = (
                other_rest is not None
                and other_pressed is not None
                and abs(other_pressed[1] - other_rest[1]) <= 1e-6
            )
            others_ok = others_ok and same_y
            unchanged_details.append(f"{other_name}:{other_rest}->{other_pressed}")
        ctx.check(
            f"{name} presses independently",
            moved_ok and others_ok,
            details=(
                f"driven {name}: rest={rest_pos}, pressed={pressed_pos}; "
                + ", ".join(unchanged_details)
            ),
        )

    return ctx.report()


object_model = build_object_model()
