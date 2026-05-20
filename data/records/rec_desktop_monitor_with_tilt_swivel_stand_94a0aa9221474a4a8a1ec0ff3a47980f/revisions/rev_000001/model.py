from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        0.5 * (low[0] + high[0]),
        0.5 * (low[1] + high[1]),
        0.5 * (low[2] + high[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_monitor_24")

    shell_black = model.material("shell_black", rgba=(0.10, 0.10, 0.11, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.05, 0.05, 0.06, 1.0))
    screen_black = model.material("screen_black", rgba=(0.03, 0.04, 0.05, 1.0))
    stand_gray = model.material("stand_gray", rgba=(0.27, 0.29, 0.31, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.36, 0.38, 0.40, 1.0))
    control_gray = model.material("control_gray", rgba=(0.18, 0.18, 0.19, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.220, 0.200, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=stand_gray,
        name="base_core",
    )
    base.visual(
        Box((0.280, 0.140, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=stand_gray,
        name="base_span",
    )
    corner_radius = 0.030
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.110, -0.070),
            (-0.110, 0.070),
            (0.110, -0.070),
            (0.110, 0.070),
        )
    ):
        base.visual(
            Cylinder(radius=corner_radius, length=0.014),
            origin=Origin(xyz=(x_pos, y_pos, 0.007)),
            material=stand_gray,
            name=f"base_corner_{index}",
        )
    base.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=trim_gray,
        name="swivel_boss",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_gray,
        name="swivel_shaft",
    )
    column.visual(
        Box((0.040, 0.004, 0.215)),
        origin=Origin(xyz=(0.0, 0.012, 0.1125)),
        material=stand_gray,
        name="sleeve_front",
    )
    column.visual(
        Box((0.040, 0.004, 0.215)),
        origin=Origin(xyz=(0.0, -0.012, 0.1125)),
        material=stand_gray,
        name="sleeve_back",
    )
    column.visual(
        Box((0.004, 0.020, 0.215)),
        origin=Origin(xyz=(0.018, 0.0, 0.1125)),
        material=stand_gray,
        name="sleeve_right",
    )
    column.visual(
        Box((0.004, 0.020, 0.215)),
        origin=Origin(xyz=(-0.018, 0.0, 0.1125)),
        material=stand_gray,
        name="sleeve_left",
    )
    column.visual(
        Box((0.046, 0.005, 0.028)),
        origin=Origin(xyz=(0.0, 0.0145, 0.218)),
        material=trim_gray,
        name="collar_front",
    )
    column.visual(
        Box((0.046, 0.005, 0.028)),
        origin=Origin(xyz=(0.0, -0.0145, 0.218)),
        material=trim_gray,
        name="collar_back",
    )
    column.visual(
        Box((0.005, 0.024, 0.028)),
        origin=Origin(xyz=(0.0205, 0.0, 0.218)),
        material=trim_gray,
        name="collar_right",
    )
    column.visual(
        Box((0.005, 0.024, 0.028)),
        origin=Origin(xyz=(-0.0205, 0.0, 0.218)),
        material=trim_gray,
        name="collar_left",
    )
    column.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.022, 0.0, 0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_gray,
        name="height_lock",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.032, 0.020, 0.400)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=trim_gray,
        name="mast_bar",
    )
    mast.visual(
        Box((0.038, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=trim_gray,
        name="head_block",
    )
    mast.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.0, -0.010, 0.273), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="tilt_barrel",
    )

    display = model.part("display")
    display.visual(
        Box((0.552, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, 0.243)),
        material=shell_black,
        name="top_shell",
    )
    display.visual(
        Box((0.552, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, -0.082)),
        material=shell_black,
        name="bottom_shell",
    )
    display.visual(
        Box((0.012, 0.040, 0.336)),
        origin=Origin(xyz=(0.270, 0.020, 0.080)),
        material=shell_black,
        name="right_shell",
    )
    display.visual(
        Box((0.012, 0.040, 0.336)),
        origin=Origin(xyz=(-0.270, 0.020, 0.080)),
        material=shell_black,
        name="left_shell",
    )
    display.visual(
        Box((0.536, 0.004, 0.320)),
        origin=Origin(xyz=(0.0, 0.004, 0.080)),
        material=shell_black,
        name="rear_plate",
    )
    display.visual(
        Box((0.192, 0.016, 0.110)),
        origin=Origin(xyz=(0.0, 0.012, 0.070)),
        material=shell_black,
        name="rear_bulge",
    )
    display.visual(
        Box((0.552, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.036, 0.243)),
        material=bezel_black,
        name="top_bezel",
    )
    display.visual(
        Box((0.552, 0.008, 0.027)),
        origin=Origin(xyz=(0.0, 0.036, -0.0745)),
        material=bezel_black,
        name="bottom_bezel",
    )
    display.visual(
        Box((0.011, 0.008, 0.299)),
        origin=Origin(xyz=(0.2705, 0.036, 0.0885)),
        material=bezel_black,
        name="right_bezel",
    )
    display.visual(
        Box((0.011, 0.008, 0.299)),
        origin=Origin(xyz=(-0.2705, 0.036, 0.0885)),
        material=bezel_black,
        name="left_bezel",
    )
    display.visual(
        Box((0.530, 0.003, 0.299)),
        origin=Origin(xyz=(0.0, 0.0385, 0.0885)),
        material=screen_black,
        name="screen_panel",
    )
    for index, x_pos in enumerate((0.165, 0.180, 0.195, 0.214)):
        button_size = (0.008, 0.002, 0.007) if index < 3 else (0.012, 0.002, 0.009)
        display.visual(
            Box(button_size),
            origin=Origin(xyz=(x_pos, 0.0405, -0.074)),
            material=control_gray,
            name=f"button_{index}",
        )
    display.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(0.146, 0.018, -0.093)),
        material=control_gray,
        name="joystick_mount",
    )
    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=control_gray,
        name="joystick_stem",
    )
    joystick.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=control_gray,
        name="joystick_cap",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
    )
    model.articulation(
        "column_to_mast",
        ArticulationType.PRISMATIC,
        parent=column,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.16, lower=0.0, upper=0.120),
    )
    model.articulation(
        "mast_to_display",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=display,
        origin=Origin(xyz=(0.0, 0.010, 0.273)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=math.radians(-10.0),
            upper=math.radians(23.0),
        ),
    )
    model.articulation(
        "display_to_joystick",
        ArticulationType.REVOLUTE,
        parent=display,
        child=joystick,
        origin=Origin(xyz=(0.146, 0.018, -0.098)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=math.radians(-14.0),
            upper=math.radians(14.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("column")
    mast = object_model.get_part("mast")
    display = object_model.get_part("display")
    joystick = object_model.get_part("joystick")
    lift = object_model.get_articulation("column_to_mast")
    tilt = object_model.get_articulation("mast_to_display")
    swivel = object_model.get_articulation("base_to_column")
    joystick_pivot = object_model.get_articulation("display_to_joystick")

    ctx.check(
        "lower bezel has visible front controls",
        all(display.get_visual(f"button_{index}") is not None for index in range(4)),
        details="Expected a visible cluster of front-panel controls in the lower bezel.",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    lift_upper = lift.motion_limits.upper if lift.motion_limits is not None else None
    if rest_mast_pos is not None and lift_upper is not None:
        with ctx.pose({lift: lift_upper}):
            raised_mast_pos = ctx.part_world_position(mast)
            mast_box = ctx.part_element_world_aabb(mast, elem="mast_bar")
            column_box = ctx.part_world_aabb(column)
            within_xy = False
            retained_overlap = False
            if mast_box is not None and column_box is not None:
                mast_min, mast_max = mast_box
                column_min, column_max = column_box
                within_xy = (
                    mast_min[0] >= column_min[0] - 0.001
                    and mast_max[0] <= column_max[0] + 0.001
                    and mast_min[1] >= column_min[1] - 0.001
                    and mast_max[1] <= column_max[1] + 0.001
                )
                retained_overlap = (
                    min(mast_max[2], column_max[2]) - max(mast_min[2], column_min[2])
                ) >= 0.070

            ctx.check(
                "mast raises upward",
                raised_mast_pos is not None and raised_mast_pos[2] > rest_mast_pos[2] + 0.08,
                details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
            )
            ctx.check(
                "mast stays centered inside the sleeve",
                within_xy,
                details=f"mast_bar={mast_box}, column={column_box}",
            )
            ctx.check(
                "mast remains inserted at max height",
                retained_overlap,
                details=f"mast_bar={mast_box}, column={column_box}",
            )

    screen_rest = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_panel"))
    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    if screen_rest is not None and tilt_upper is not None:
        with ctx.pose({tilt: tilt_upper}):
            screen_tilted = _aabb_center(ctx.part_element_world_aabb(display, elem="screen_panel"))
        ctx.check(
            "display tilts backward at the hinge",
            screen_tilted is not None and screen_tilted[1] < screen_rest[1] - 0.02,
            details=f"rest={screen_rest}, tilted={screen_tilted}",
        )

    button_rest = _aabb_center(ctx.part_element_world_aabb(display, elem="button_3"))
    if button_rest is not None:
        with ctx.pose({swivel: 0.65}):
            button_swiveled = _aabb_center(ctx.part_element_world_aabb(display, elem="button_3"))
        ctx.check(
            "stand swivels about the pedestal",
            button_swiveled is not None and button_swiveled[1] > button_rest[1] + 0.06,
            details=f"rest={button_rest}, swiveled={button_swiveled}",
        )

    joystick_rest = _aabb_center(ctx.part_element_world_aabb(joystick, elem="joystick_cap"))
    joystick_upper = joystick_pivot.motion_limits.upper if joystick_pivot.motion_limits is not None else None
    if joystick_rest is not None and joystick_upper is not None:
        display_box = ctx.part_world_aabb(display)
        ctx.check(
            "joystick hangs below the lower bezel",
            display_box is not None and joystick_rest[2] < display_box[0][2] - 0.004,
            details=f"display={display_box}, joystick={joystick_rest}",
        )
        with ctx.pose({joystick_pivot: joystick_upper}):
            joystick_tilted = _aabb_center(ctx.part_element_world_aabb(joystick, elem="joystick_cap"))
        ctx.check(
            "joystick pivots on a short local hinge",
            joystick_tilted is not None and joystick_tilted[1] > joystick_rest[1] + 0.003,
            details=f"rest={joystick_rest}, tilted={joystick_tilted}",
        )

    return ctx.report()


object_model = build_object_model()
