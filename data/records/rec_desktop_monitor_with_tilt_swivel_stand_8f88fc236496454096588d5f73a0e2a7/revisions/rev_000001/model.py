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


def _make_base_plate(width: float, depth: float, thickness: float) -> object:
    return (
        cq.Workplane("XY")
        .box(width, depth, thickness)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, 0.0, thickness / 2.0))
    )


def _make_stand_shell() -> object:
    column_width = 0.112
    column_depth = 0.038
    column_height = 0.285
    column_base_z = 0.003
    opening_width = 0.070
    opening_height = 0.180
    opening_bottom_z = 0.055
    opening_depth = 0.020

    head_width = 0.128
    head_depth = 0.060
    head_height = 0.072
    head_center_z = 0.299

    column = (
        cq.Workplane("XY")
        .box(column_width, column_depth, column_height)
        .edges("|Z")
        .fillet(0.015)
        .translate((0.0, 0.0, column_base_z + column_height / 2.0))
    )

    rear_channel = (
        cq.Workplane("XY")
        .box(opening_width, opening_depth, opening_height)
        .edges("|Z")
        .fillet(0.004)
        .translate(
            (
                0.0,
                column_depth / 2.0 - opening_depth / 2.0 + 0.0005,
                opening_bottom_z + opening_height / 2.0,
            )
        )
    )
    column = column.cut(rear_channel)

    head = (
        cq.Workplane("XY")
        .box(head_width, head_depth, head_height)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.004, head_center_z))
    )

    stand = column.union(head)

    front_clearance = (
        cq.Workplane("XY")
        .box(0.086, 0.040, 0.080)
        .translate((0.0, -0.014, 0.296))
    )

    return stand.cut(front_clearance)


def _make_display_shell() -> object:
    outer_width = 0.624
    outer_height = 0.372
    outer_depth = 0.020
    wall = 0.004
    outer_center_y = -0.008
    outer_corner = 0.007

    inner_depth = 0.018
    inner_center_y = -0.011
    inner_corner = 0.005

    outer = (
        cq.Workplane("XY")
        .box(outer_width, outer_depth, outer_height)
        .edges("|Y")
        .fillet(outer_corner)
        .translate((0.0, outer_center_y, 0.0))
    )

    inner = (
        cq.Workplane("XY")
        .box(outer_width - 2.0 * wall, inner_depth, outer_height - 2.0 * wall)
        .edges("|Y")
        .fillet(inner_corner)
        .translate((0.0, inner_center_y, 0.0))
    )

    shell = outer.cut(inner)

    rear_bulge = (
        cq.Workplane("XY")
        .box(0.240, 0.016, 0.164)
        .edges("|Y")
        .fillet(0.005)
        .translate((0.0, -0.006, 0.0))
    )

    lower_bulge = (
        cq.Workplane("XY")
        .box(0.290, 0.012, 0.090)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.0, -0.007, -0.108))
    )

    return shell.union(rear_bulge).union(lower_bulge)


def _make_cable_door(width: float, thickness: float, height: float) -> object:
    door = (
        cq.Workplane("XY")
        .box(width, thickness, height)
        .edges("|Z")
        .fillet(0.0015)
        .translate((width / 2.0, thickness / 2.0, height / 2.0))
    )

    barrel_radius = 0.004
    barrel_length = 0.040
    barrel_centers = (0.024, height / 2.0, height - 0.024)

    for center_z in barrel_centers:
        barrel = (
            cq.Workplane("XY")
            .circle(barrel_radius)
            .extrude(barrel_length)
            .translate((0.0, 0.002, center_z - barrel_length / 2.0))
        )
        door = door.union(barrel)

    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_monitor")

    dark_shell = model.material("dark_shell", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.58, 0.59, 0.61, 1.0))
    deep_black = model.material("deep_black", rgba=(0.03, 0.03, 0.04, 1.0))
    pivot_black = model.material("pivot_black", rgba=(0.09, 0.09, 0.10, 1.0))
    screen_black = model.material("screen_black", rgba=(0.04, 0.05, 0.06, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_make_base_plate(0.300, 0.230, 0.014), "base_plate"),
        material=satin_metal,
        name="plate",
    )
    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_make_stand_shell(), "stand"),
        material=satin_metal,
        name="spine",
    )
    stand.visual(
        Cylinder(radius=0.036, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=satin_metal,
        name="swivel_collar",
    )
    stand.visual(
        Box((0.004, 0.004, 0.170)),
        origin=Origin(xyz=(-0.0315, 0.019, 0.145)),
        material=satin_metal,
        name="door_hinge_rail",
    )

    tilt_carrier = model.part("tilt_carrier")
    tilt_carrier.visual(
        Cylinder(radius=0.007, length=0.080),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_black,
        name="tilt_axle",
    )
    tilt_carrier.visual(
        Box((0.040, 0.030, 0.048)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=pivot_black,
        name="tilt_neck",
    )
    tilt_carrier.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=Origin(xyz=(0.0, -0.027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pivot_black,
        name="pivot_plate",
    )
    tilt_carrier.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pivot_black,
        name="pivot_spigot",
    )

    display = model.part("display")
    display.visual(
        mesh_from_cadquery(_make_display_shell(), "display_shell"),
        material=dark_shell,
        name="rear_housing",
    )
    display.visual(
        Box((0.070, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=dark_shell,
        name="mount_block",
    )

    screen_panel = model.part("screen_panel")
    screen_panel.visual(
        Box((0.596, 0.004, 0.336)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=screen_black,
        name="screen_glass",
    )
    screen_panel.visual(
        Box((0.520, 0.0015, 0.290)),
        origin=Origin(xyz=(0.0, -0.0122, 0.0)),
        material=deep_black,
        name="screen_dark",
    )

    cable_door = model.part("cable_door")
    cable_door.visual(
        mesh_from_cadquery(_make_cable_door(0.060, 0.004, 0.170), "cable_door"),
        material=dark_shell,
        name="door_panel",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=3.0),
    )
    model.articulation(
        "stand_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tilt_carrier,
        origin=Origin(xyz=(0.0, -0.004, 0.299)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.14,
            upper=0.44,
        ),
    )
    model.articulation(
        "display_pivot",
        ArticulationType.CONTINUOUS,
        parent=tilt_carrier,
        child=display,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "screen_mount",
        ArticulationType.FIXED,
        parent=display,
        child=screen_panel,
        origin=Origin(),
    )
    model.articulation(
        "cable_door_hinge",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=cable_door,
        origin=Origin(xyz=(-0.030, 0.021, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_plate = object_model.get_part("base_plate")
    stand = object_model.get_part("stand")
    display = object_model.get_part("display")
    screen_panel = object_model.get_part("screen_panel")
    cable_door = object_model.get_part("cable_door")

    swivel = object_model.get_articulation("base_swivel")
    tilt = object_model.get_articulation("stand_tilt")
    pivot = object_model.get_articulation("display_pivot")
    door_hinge = object_model.get_articulation("cable_door_hinge")

    ctx.expect_gap(
        display,
        base_plate,
        axis="z",
        min_gap=0.10,
        name="screen clears the base with desk-monitor height",
    )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt: tilt_limits.lower, pivot: 0.0, swivel: 0.0}):
            low_pos = ctx.part_world_position(display)
        with ctx.pose({tilt: tilt_limits.upper, pivot: 0.0, swivel: 0.0}):
            high_pos = ctx.part_world_position(display)

        ctx.check(
            "display rises at positive tilt",
            low_pos is not None
            and high_pos is not None
            and high_pos[2] > low_pos[2] + 0.01,
            details=f"low={low_pos}, high={high_pos}",
        )

    with ctx.pose({tilt: 0.0, pivot: 0.0, swivel: 0.0}):
        landscape_aabb = ctx.part_world_aabb(screen_panel)
        rest_pos = ctx.part_world_position(display)

    with ctx.pose({tilt: 0.0, pivot: math.pi / 2.0, swivel: 0.0}):
        portrait_aabb = ctx.part_world_aabb(screen_panel)

    landscape_width = None
    landscape_height = None
    portrait_width = None
    portrait_height = None
    if landscape_aabb is not None:
        landscape_width = landscape_aabb[1][0] - landscape_aabb[0][0]
        landscape_height = landscape_aabb[1][2] - landscape_aabb[0][2]
    if portrait_aabb is not None:
        portrait_width = portrait_aabb[1][0] - portrait_aabb[0][0]
        portrait_height = portrait_aabb[1][2] - portrait_aabb[0][2]

    ctx.check(
        "display pivots to portrait orientation",
        landscape_width is not None
        and landscape_height is not None
        and portrait_width is not None
        and portrait_height is not None
        and landscape_width > landscape_height
        and portrait_height > portrait_width
        and portrait_height > landscape_height * 0.95,
        details=(
            f"landscape=({landscape_width}, {landscape_height}), "
            f"portrait=({portrait_width}, {portrait_height})"
        ),
    )

    with ctx.pose({tilt: 0.0, pivot: 0.0, swivel: math.pi / 2.0}):
        swivel_pos = ctx.part_world_position(display)

    ctx.check(
        "stand swivels the screen around the base",
        rest_pos is not None
        and swivel_pos is not None
        and abs(rest_pos[0]) < 0.01
        and abs(swivel_pos[0]) > 0.02
        and abs(swivel_pos[1]) < abs(rest_pos[1]),
        details=f"rest={rest_pos}, swivel={swivel_pos}",
    )

    with ctx.pose({door_hinge: 0.0}):
        closed_door = ctx.part_world_aabb(cable_door)
    with ctx.pose({door_hinge: 1.2}):
        open_door = ctx.part_world_aabb(cable_door)

    door_closed_rear = None
    door_open_rear = None
    if closed_door is not None:
        door_closed_rear = closed_door[1][1]
    if open_door is not None:
        door_open_rear = open_door[1][1]

    ctx.check(
        "cable door swings outward from the stand spine",
        door_closed_rear is not None
        and door_open_rear is not None
        and door_open_rear > door_closed_rear + 0.03,
        details=f"closed={door_closed_rear}, open={door_open_rear}",
    )

    ctx.expect_overlap(
        cable_door,
        stand,
        axes="z",
        min_overlap=0.15,
        name="cable door spans the rear routing opening height",
    )

    return ctx.report()


object_model = build_object_model()
