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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_DEPTH = 0.300
BASE_WIDTH = 0.235
BASE_THICK = 0.014

COLUMN_OUTER_X = 0.054
COLUMN_OUTER_Y = 0.092
COLUMN_INNER_X = 0.038
COLUMN_INNER_Y = 0.060
COLUMN_HEIGHT = 0.280
COLUMN_WALL_X = (COLUMN_OUTER_X - COLUMN_INNER_X) / 2.0
COLUMN_WALL_Y = (COLUMN_OUTER_Y - COLUMN_INNER_Y) / 2.0

MAST_WIDTH_X = 0.032
MAST_WIDTH_Y = 0.052
MAST_LENGTH = 0.500
MAST_TRAVEL = 0.120

DISPLAY_DEPTH = 0.014
DISPLAY_WIDTH = 0.620
DISPLAY_HEIGHT = 0.368


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    sx, sy, sz = size
    shape = cq.Workplane("XY").box(sx, sy, sz).edges("|X").fillet(radius)
    return mesh_from_cadquery(shape, name)


def _rounded_plate_mesh(size: tuple[float, float, float], radius: float, name: str):
    sx, sy, sz = size
    shape = cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)
    return mesh_from_cadquery(shape, name)


def _aabb_size(aabb):
    mins, maxs = aabb
    return tuple(float(maxs[i] - mins[i]) for i in range(3))


def _aabb_center(aabb):
    mins, maxs = aabb
    return tuple(float((mins[i] + maxs[i]) * 0.5) for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_monitor")

    base_dark = model.material("base_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.11, 0.16, 0.20, 0.78))
    trim = model.material("trim", rgba=(0.22, 0.23, 0.24, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        _rounded_plate_mesh((BASE_DEPTH, BASE_WIDTH, BASE_THICK), 0.030, "monitor_base_plate"),
        origin=Origin(xyz=(0.020, 0.0, BASE_THICK * 0.5)),
        material=base_dark,
        name="base",
    )

    swivel_column = model.part("swivel_column")
    swivel_column.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim,
        name="turntable",
    )
    swivel_column.visual(
        Box((0.064, 0.100, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shell_dark,
        name="column_base",
    )
    swivel_column.visual(
        Box((COLUMN_OUTER_X, COLUMN_WALL_Y, COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, (COLUMN_INNER_Y * 0.5) + (COLUMN_WALL_Y * 0.5), COLUMN_HEIGHT * 0.5)),
        material=shell_dark,
        name="side_wall_0",
    )
    swivel_column.visual(
        Box((COLUMN_OUTER_X, COLUMN_WALL_Y, COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, -((COLUMN_INNER_Y * 0.5) + (COLUMN_WALL_Y * 0.5)), COLUMN_HEIGHT * 0.5)),
        material=shell_dark,
        name="side_wall_1",
    )
    swivel_column.visual(
        Box((COLUMN_WALL_X, COLUMN_OUTER_Y, COLUMN_HEIGHT)),
        origin=Origin(xyz=((COLUMN_INNER_X * 0.5) + (COLUMN_WALL_X * 0.5), 0.0, COLUMN_HEIGHT * 0.5)),
        material=shell_dark,
        name="front_wall",
    )
    swivel_column.visual(
        Box((COLUMN_WALL_X, COLUMN_OUTER_Y, COLUMN_HEIGHT)),
        origin=Origin(xyz=(-((COLUMN_INNER_X * 0.5) + (COLUMN_WALL_X * 0.5)), 0.0, COLUMN_HEIGHT * 0.5)),
        material=shell_dark,
        name="rear_wall",
    )

    mast = model.part("mast")
    mast.visual(
        Box((MAST_WIDTH_X, MAST_WIDTH_Y, MAST_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=trim,
        name="mast_body",
    )
    mast.visual(
        Box((0.003, 0.040, 0.120)),
        origin=Origin(xyz=(0.0175, 0.0, -0.095)),
        material=trim,
        name="guide_front",
    )
    mast.visual(
        Box((0.003, 0.040, 0.120)),
        origin=Origin(xyz=(-0.0175, 0.0, -0.095)),
        material=trim,
        name="guide_rear",
    )
    mast.visual(
        Box((0.020, 0.002, 0.120)),
        origin=Origin(xyz=(0.0, 0.0265, -0.095)),
        material=trim,
        name="guide_side_0",
    )
    mast.visual(
        Box((0.020, 0.002, 0.120)),
        origin=Origin(xyz=(0.0, -0.0265, -0.095)),
        material=trim,
        name="guide_side_1",
    )
    mast.visual(
        Box((0.024, 0.072, 0.055)),
        origin=Origin(xyz=(-0.032, 0.0, 0.312)),
        material=shell_dark,
        name="mast_head",
    )
    mast.visual(
        Box((0.048, 0.090, 0.018)),
        origin=Origin(xyz=(-0.018, 0.0, 0.278)),
        material=shell_dark,
        name="head_cap",
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(-0.010, 0.0, 0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="tilt_barrel",
    )
    tilt_head.visual(
        Box((0.022, 0.090, 0.050)),
        origin=Origin(xyz=(-0.008, 0.0, 0.028)),
        material=shell_dark,
        name="tilt_block",
    )
    tilt_head.visual(
        Cylinder(radius=0.033, length=0.016),
        origin=Origin(xyz=(-0.020, 0.0, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="pivot_housing",
    )

    display = model.part("display")
    display.visual(
        _rounded_box_mesh((DISPLAY_DEPTH, DISPLAY_WIDTH, DISPLAY_HEIGHT), 0.016, "monitor_display_shell"),
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
        material=shell_dark,
        name="shell",
    )
    display.visual(
        Box((0.010, 0.596, 0.328)),
        origin=Origin(xyz=(0.056, 0.0, 0.012)),
        material=satin_black,
        name="panel_backer",
    )
    display.visual(
        Box((0.003, 0.592, 0.324)),
        origin=Origin(xyz=(0.0615, 0.0, 0.012)),
        material=glass,
        name="panel",
    )
    display.visual(
        Box((0.036, 0.176, 0.124)),
        origin=Origin(xyz=(0.036, 0.0, 0.008)),
        material=trim,
        name="rear_bulge",
    )
    display.visual(
        Box((0.012, 0.120, 0.090)),
        origin=Origin(xyz=(0.032, 0.0, 0.006)),
        material=trim,
        name="mount_block",
    )
    display.visual(
        Box((0.016, 0.070, 0.048)),
        origin=Origin(xyz=(0.013, 0.0, 0.028)),
        material=trim,
        name="mount_spigot",
    )
    display.visual(
        Box((0.018, 0.034, 0.012)),
        origin=Origin(xyz=(0.049, 0.0, -0.190)),
        material=shell_dark,
        name="joystick_socket",
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0042, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=satin_black,
        name="joystick_collar",
    )
    joystick.visual(
        Cylinder(radius=0.0028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=satin_black,
        name="joystick_stem",
    )
    joystick.visual(
        Sphere(radius=0.0048),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=satin_black,
        name="joystick_cap",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=swivel_column,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.4),
    )
    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=swivel_column,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=0.0,
            upper=MAST_TRAVEL,
        ),
    )
    model.articulation(
        "screen_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=math.radians(-20.0),
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "screen_pivot",
        ArticulationType.CONTINUOUS,
        parent=tilt_head,
        child=display,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.6),
    )
    model.articulation(
        "joystick_pivot",
        ArticulationType.REVOLUTE,
        parent=display,
        child=joystick,
        origin=Origin(xyz=(0.049, 0.0, -0.196)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=5.0,
            lower=-0.28,
            upper=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    swivel = object_model.get_articulation("base_swivel")
    lift = object_model.get_articulation("column_lift")
    tilt = object_model.get_articulation("screen_tilt")
    pivot = object_model.get_articulation("screen_pivot")
    joystick_pivot = object_model.get_articulation("joystick_pivot")

    column = object_model.get_part("swivel_column")
    mast = object_model.get_part("mast")
    display = object_model.get_part("display")
    joystick = object_model.get_part("joystick")

    lift_limits = lift.motion_limits
    if lift_limits is not None and lift_limits.upper is not None:
        ctx.expect_within(
            mast,
            column,
            axes="xy",
            inner_elem="mast_body",
            margin=0.003,
            name="mast stays centered in the column sleeve",
        )
        ctx.expect_overlap(
            mast,
            column,
            axes="z",
            elem_a="mast_body",
            min_overlap=0.020,
            name="mast remains inserted at rest",
        )
        rest_mast_pos = ctx.part_world_position(mast)
        with ctx.pose({lift: lift_limits.upper}):
            ctx.expect_within(
                mast,
                column,
                axes="xy",
                inner_elem="mast_body",
                margin=0.003,
                name="mast stays centered at full height",
            )
            ctx.expect_overlap(
                mast,
                column,
                axes="z",
                elem_a="mast_body",
                min_overlap=0.018,
                name="mast retains insertion at full height",
            )
            raised_mast_pos = ctx.part_world_position(mast)
        ctx.check(
            "mast raises the monitor upward",
            rest_mast_pos is not None
            and raised_mast_pos is not None
            and raised_mast_pos[2] > rest_mast_pos[2] + 0.08,
            details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
        )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        rest_socket = ctx.part_element_world_aabb(display, elem="joystick_socket")
        with ctx.pose({tilt: tilt_limits.upper}):
            up_socket = ctx.part_element_world_aabb(display, elem="joystick_socket")
        with ctx.pose({tilt: tilt_limits.lower}):
            down_socket = ctx.part_element_world_aabb(display, elem="joystick_socket")
        ctx.check(
            "positive tilt pushes the lower edge forward",
            rest_socket is not None
            and up_socket is not None
            and _aabb_center(up_socket)[0] > _aabb_center(rest_socket)[0] + 0.040,
            details=f"rest={rest_socket}, up={up_socket}",
        )
        ctx.check(
            "negative tilt pulls the lower edge rearward",
            rest_socket is not None
            and down_socket is not None
            and _aabb_center(down_socket)[0] < _aabb_center(rest_socket)[0] - 0.030,
            details=f"rest={rest_socket}, down={down_socket}",
        )

    rest_display_box = ctx.part_world_aabb(display)
    with ctx.pose({pivot: math.pi / 2.0}):
        portrait_box = ctx.part_world_aabb(display)
    ctx.check(
        "screen rotates into portrait orientation",
        rest_display_box is not None
        and portrait_box is not None
        and _aabb_size(rest_display_box)[1] > _aabb_size(rest_display_box)[2]
        and _aabb_size(portrait_box)[2] > _aabb_size(portrait_box)[1]
        and abs(_aabb_size(portrait_box)[2] - _aabb_size(rest_display_box)[1]) < 0.05,
        details=f"rest_size={_aabb_size(rest_display_box) if rest_display_box else None}, portrait_size={_aabb_size(portrait_box) if portrait_box else None}",
    )

    rest_center = _aabb_center(rest_display_box) if rest_display_box is not None else None
    with ctx.pose({swivel: math.pi / 2.0}):
        swivel_box = ctx.part_world_aabb(display)
    swivel_center = _aabb_center(swivel_box) if swivel_box is not None else None
    ctx.check(
        "stand swivels about the base axis",
        rest_center is not None
        and swivel_center is not None
        and abs(swivel_center[1]) > abs(rest_center[1]) + 0.006
        and abs(swivel_center[0]) < abs(rest_center[0]),
        details=f"rest_center={rest_center}, swivel_center={swivel_center}",
    )

    ctx.expect_gap(
        display,
        joystick,
        axis="z",
        positive_elem="joystick_socket",
        negative_elem="joystick_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="joystick collar seats against the socket",
    )
    ctx.expect_within(
        joystick,
        display,
        axes="xy",
        inner_elem="joystick_collar",
        outer_elem="joystick_socket",
        margin=0.010,
        name="joystick stays within the supported socket footprint",
    )

    joystick_limits = joystick_pivot.motion_limits
    if joystick_limits is not None and joystick_limits.lower is not None and joystick_limits.upper is not None:
        with ctx.pose({joystick_pivot: joystick_limits.lower}):
            low_cap = ctx.part_element_world_aabb(joystick, elem="joystick_cap")
        with ctx.pose({joystick_pivot: joystick_limits.upper}):
            high_cap = ctx.part_element_world_aabb(joystick, elem="joystick_cap")
        ctx.check(
            "joystick has short pivot travel",
            low_cap is not None
            and high_cap is not None
            and abs(_aabb_center(high_cap)[0] - _aabb_center(low_cap)[0]) > 0.003,
            details=f"low_cap={low_cap}, high_cap={high_cap}",
        )

    return ctx.report()


object_model = build_object_model()
