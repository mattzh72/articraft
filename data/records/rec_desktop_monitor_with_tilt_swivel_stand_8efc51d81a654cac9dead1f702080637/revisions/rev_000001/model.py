from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _aabb_size(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float(maxs[i] - mins[i]) for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_monitor")

    stand_metal = model.material("stand_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    shell_black = model.material("shell_black", rgba=(0.13, 0.14, 0.15, 1.0))
    deep_black = model.material("deep_black", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = model.material("glass", rgba=(0.06, 0.10, 0.12, 0.95))
    hinge_dark = model.material("hinge_dark", rgba=(0.20, 0.21, 0.23, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.300, 0.235, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=stand_metal,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.066, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=stand_metal,
        name="pedestal_disc",
    )
    base.visual(
        Cylinder(radius=0.042, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=hinge_dark,
        name="swivel_bearing",
    )

    lower_column = model.part("lower_column")
    lower_column.visual(
        Cylinder(radius=0.037, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=hinge_dark,
        name="swivel_drum",
    )
    lower_column.visual(
        Box((0.024, 0.052, 0.036)),
        origin=Origin(xyz=(0.003, 0.0, 0.044)),
        material=stand_metal,
        name="sleeve_root",
    )
    lower_column.visual(
        Box((0.010, 0.072, 0.270)),
        origin=Origin(xyz=(0.013, 0.0, 0.173)),
        material=stand_metal,
        name="sleeve_front",
    )
    lower_column.visual(
        Box((0.030, 0.010, 0.270)),
        origin=Origin(xyz=(-0.002, -0.031, 0.173)),
        material=stand_metal,
        name="sleeve_side_0",
    )
    lower_column.visual(
        Box((0.030, 0.010, 0.270)),
        origin=Origin(xyz=(-0.002, 0.031, 0.173)),
        material=stand_metal,
        name="sleeve_side_1",
    )
    lower_column.visual(
        Box((0.006, 0.052, 0.040)),
        origin=Origin(xyz=(-0.015, 0.0, 0.086)),
        material=stand_metal,
        name="sleeve_bottom_bridge",
    )
    lower_column.visual(
        Box((0.006, 0.052, 0.040)),
        origin=Origin(xyz=(-0.015, 0.0, 0.260)),
        material=stand_metal,
        name="sleeve_top_bridge",
    )
    lower_column.visual(
        Box((0.004, 0.010, 0.168)),
        origin=Origin(xyz=(-0.008, -0.031, 0.173)),
        material=hinge_dark,
        name="door_jamb",
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Box((0.014, 0.036, 0.340)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=hinge_dark,
        name="inner_mast",
    )
    upper_column.visual(
        Box((0.020, 0.082, 0.070)),
        origin=Origin(xyz=(0.012, 0.0, 0.200)),
        material=stand_metal,
        name="head_block",
    )
    upper_column.visual(
        Box((0.020, 0.096, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, 0.266)),
        material=stand_metal,
        name="yoke_bar",
    )
    upper_column.visual(
        Box((0.020, 0.014, 0.054)),
        origin=Origin(xyz=(0.018, -0.041, 0.242)),
        material=stand_metal,
        name="yoke_ear_0",
    )
    upper_column.visual(
        Box((0.020, 0.014, 0.054)),
        origin=Origin(xyz=(0.018, 0.041, 0.242)),
        material=stand_metal,
        name="yoke_ear_1",
    )
    upper_column.visual(
        Box((0.002, 0.044, 0.120)),
        origin=Origin(xyz=(0.008, 0.0, -0.100)),
        material=hinge_dark,
        name="guide_shoe",
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Cylinder(radius=0.012, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="trunnion",
    )
    tilt_frame.visual(
        Box((0.090, 0.050, 0.018)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=hinge_dark,
        name="tilt_arm",
    )
    tilt_frame.visual(
        Cylinder(radius=0.033, length=0.016),
        origin=Origin(xyz=(0.098, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="roll_collar",
    )

    screen = model.part("screen")
    screen.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.688, 0.388),
                (0.710, 0.410),
                0.010,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.010,
                outer_corner_radius=0.018,
            ),
            "monitor_bezel",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)),
        material=shell_black,
        name="bezel",
    )
    screen.visual(
        Box((0.003, 0.692, 0.392)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=glass,
        name="glass_panel",
    )
    screen.visual(
        Box((0.018, 0.688, 0.090)),
        origin=Origin(xyz=(-0.010, 0.0, 0.151)),
        material=shell_black,
        name="rear_top",
    )
    screen.visual(
        Box((0.018, 0.688, 0.090)),
        origin=Origin(xyz=(-0.010, 0.0, -0.151)),
        material=shell_black,
        name="rear_bottom",
    )
    screen.visual(
        Box((0.018, 0.090, 0.212)),
        origin=Origin(xyz=(-0.010, -0.299, 0.0)),
        material=shell_black,
        name="rear_side_0",
    )
    screen.visual(
        Box((0.018, 0.090, 0.212)),
        origin=Origin(xyz=(-0.010, 0.299, 0.0)),
        material=shell_black,
        name="rear_side_1",
    )

    cable_door = model.part("cable_door")
    cable_door.visual(
        Cylinder(radius=0.0032, length=0.156),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=hinge_dark,
        name="hinge_barrel",
    )
    cable_door.visual(
        Box((0.004, 0.052, 0.156)),
        origin=Origin(xyz=(0.004, 0.028, 0.078)),
        material=shell_black,
        name="door_panel",
    )
    cable_door.visual(
        Box((0.003, 0.012, 0.024)),
        origin=Origin(xyz=(0.0025, 0.046, 0.078)),
        material=deep_black,
        name="finger_notch",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=lower_column,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0),
    )
    model.articulation(
        "column_height",
        ArticulationType.PRISMATIC,
        parent=lower_column,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.16,
            lower=0.0,
            upper=0.140,
        ),
    )
    model.articulation(
        "screen_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=tilt_frame,
        origin=Origin(xyz=(0.018, 0.0, 0.248)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.4,
            lower=math.radians(-18.0),
            upper=math.radians(24.0),
        ),
    )
    model.articulation(
        "screen_roll",
        ArticulationType.CONTINUOUS,
        parent=tilt_frame,
        child=screen,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )
    model.articulation(
        "cable_door_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_column,
        child=cable_door,
        origin=Origin(xyz=(-0.024, -0.028, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_column = object_model.get_part("lower_column")
    upper_column = object_model.get_part("upper_column")
    screen = object_model.get_part("screen")
    cable_door = object_model.get_part("cable_door")

    base_swivel = object_model.get_articulation("base_swivel")
    column_height = object_model.get_articulation("column_height")
    screen_tilt = object_model.get_articulation("screen_tilt")
    screen_roll = object_model.get_articulation("screen_roll")
    cable_door_hinge = object_model.get_articulation("cable_door_hinge")

    column_limits = column_height.motion_limits
    tilt_limits = screen_tilt.motion_limits
    door_limits = cable_door_hinge.motion_limits

    ctx.expect_within(
        upper_column,
        lower_column,
        axes="xy",
        inner_elem="inner_mast",
        margin=0.006,
        name="inner mast stays centered in the sleeve",
    )
    if column_limits is not None and column_limits.upper is not None:
        with ctx.pose({column_height: column_limits.upper}):
            ctx.expect_within(
                upper_column,
                lower_column,
                axes="xy",
                inner_elem="inner_mast",
                margin=0.006,
                name="extended mast stays centered in the sleeve",
            )
            ctx.expect_overlap(
                upper_column,
                lower_column,
                axes="z",
                elem_a="inner_mast",
                min_overlap=0.018,
                name="extended mast retains insertion in the sleeve",
            )

    rest_screen_pos = ctx.part_world_position(screen)
    if column_limits is not None and column_limits.upper is not None:
        with ctx.pose({column_height: column_limits.upper}):
            raised_screen_pos = ctx.part_world_position(screen)
        ctx.check(
            "height adjustment raises the screen",
            rest_screen_pos is not None
            and raised_screen_pos is not None
            and raised_screen_pos[2] > rest_screen_pos[2] + 0.10,
            details=f"rest={rest_screen_pos}, raised={raised_screen_pos}",
        )

    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({screen_tilt: tilt_limits.upper}):
            tilted_screen_pos = ctx.part_world_position(screen)
        ctx.check(
            "positive tilt lifts the screen center",
            rest_screen_pos is not None
            and tilted_screen_pos is not None
            and tilted_screen_pos[2] > rest_screen_pos[2] + 0.015,
            details=f"rest={rest_screen_pos}, tilted={tilted_screen_pos}",
        )

    screen_landscape = _aabb_size(ctx.part_world_aabb(screen))
    with ctx.pose({screen_roll: math.pi / 2.0}):
        screen_portrait = _aabb_size(ctx.part_world_aabb(screen))
    ctx.check(
        "roll swaps the screen between landscape and portrait",
        screen_landscape is not None
        and screen_portrait is not None
        and screen_landscape[1] > screen_landscape[2]
        and screen_portrait[2] > screen_portrait[1],
        details=f"landscape={screen_landscape}, portrait={screen_portrait}",
    )

    with ctx.pose({base_swivel: math.pi / 2.0}):
        swivel_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "base swivel moves the display around the vertical axis",
        rest_screen_pos is not None
        and swivel_screen_pos is not None
        and swivel_screen_pos[1] > 0.05
        and abs(swivel_screen_pos[0]) < rest_screen_pos[0],
        details=f"rest={rest_screen_pos}, swivel={swivel_screen_pos}",
    )

    door_closed = _aabb_size(ctx.part_world_aabb(cable_door))
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({cable_door_hinge: door_limits.upper}):
            door_open = _aabb_size(ctx.part_world_aabb(cable_door))
        ctx.check(
            "cable door swings away from the spine",
            door_closed is not None
            and door_open is not None
            and door_open[0] > door_closed[0] + 0.030
            and door_open[1] < door_closed[1] * 0.60,
            details=f"closed={door_closed}, open={door_open}",
        )

    return ctx.report()


object_model = build_object_model()
