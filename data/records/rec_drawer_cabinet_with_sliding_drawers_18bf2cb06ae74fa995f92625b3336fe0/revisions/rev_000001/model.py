from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_DEPTH = 0.46
CABINET_WIDTH = 0.62
CABINET_HEIGHT = 0.78
SHELL_THICKNESS = 0.018
FRONT_FRAME_THICKNESS = 0.012

DRAWER_COUNT = 6
FRAME_MARGIN_Z = 0.03
DRAWER_SEPARATOR = 0.008
DRAWER_OPENING_HEIGHT = (
    CABINET_HEIGHT - 2.0 * FRAME_MARGIN_Z - (DRAWER_COUNT - 1) * DRAWER_SEPARATOR
) / DRAWER_COUNT
DRAWER_PITCH = DRAWER_OPENING_HEIGHT + DRAWER_SEPARATOR
DRAWER_OPENING_WIDTH = CABINET_WIDTH - 2.0 * SHELL_THICKNESS

DRAWER_FRONT_HEIGHT = DRAWER_OPENING_HEIGHT - 0.004
DRAWER_FRONT_WIDTH = DRAWER_OPENING_WIDTH - 0.004
DRAWER_FRONT_THICKNESS = 0.016
DRAWER_TRAY_DEPTH = 0.39
DRAWER_TRAY_WIDTH = 0.54
DRAWER_TRAY_HEIGHT = DRAWER_FRONT_HEIGHT - 0.018
DRAWER_WALL_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.012
DRAWER_TRAVEL = 0.22
DRAWER_SLIDE_WIDTH = 0.01
DRAWER_SLIDE_HEIGHT = 0.01
DRAWER_SLIDE_DEPTH = 0.34

RUNNER_DEPTH = 0.3
RUNNER_WIDTH = 0.014
RUNNER_HEIGHT = 0.01
RUNNER_X = -0.18

HANDLE_TUBE_RADIUS = 0.009
HANDLE_SPREAD = 0.28
HANDLE_DROP = 0.13
HANDLE_PIVOT_LENGTH = 0.03
HANDLE_PIVOT_Z = 0.235
HANDLE_SWING = 1.18

BRACKET_SIZE_X = 0.032
BRACKET_SIZE_Y = 0.018
BRACKET_SIZE_Z = 0.05
HANDLE_PIVOT_Y = CABINET_WIDTH / 2.0 + BRACKET_SIZE_Y + HANDLE_TUBE_RADIUS - 0.001


def _drawer_center_z(index: int) -> float:
    bottom_center = -CABINET_HEIGHT / 2.0 + FRAME_MARGIN_Z + DRAWER_OPENING_HEIGHT / 2.0
    return bottom_center + index * DRAWER_PITCH


def _x_cylinder(radius: float, length: float, center_x: float, center_y: float, center_z: float):
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x - length / 2.0, center_y, center_z))
    )


def _z_cylinder(radius: float, length: float, center_x: float, center_y: float, base_z: float):
    return cq.Workplane("XY").circle(radius).extrude(length).translate((center_x, center_y, base_z))


def _build_cabinet_shape():
    outer = cq.Workplane("XY").box(CABINET_DEPTH, CABINET_WIDTH, CABINET_HEIGHT)
    cavity = (
        cq.Workplane("XY")
        .box(
            CABINET_DEPTH - SHELL_THICKNESS,
            CABINET_WIDTH - 2.0 * SHELL_THICKNESS,
            CABINET_HEIGHT - 2.0 * SHELL_THICKNESS,
        )
        .translate((SHELL_THICKNESS / 2.0, 0.0, 0.0))
    )
    cabinet = outer.cut(cavity)

    front_frame = (
        cq.Workplane("XY")
        .box(FRONT_FRAME_THICKNESS, CABINET_WIDTH, CABINET_HEIGHT)
        .translate((CABINET_DEPTH / 2.0 - FRONT_FRAME_THICKNESS / 2.0, 0.0, 0.0))
    )
    for index in range(DRAWER_COUNT):
        opening = (
            cq.Workplane("XY")
            .box(
                FRONT_FRAME_THICKNESS * 2.0,
                DRAWER_OPENING_WIDTH,
                DRAWER_OPENING_HEIGHT,
            )
            .translate(
                (
                    CABINET_DEPTH / 2.0 - FRONT_FRAME_THICKNESS / 2.0,
                    0.0,
                    _drawer_center_z(index),
                )
            )
        )
        front_frame = front_frame.cut(opening)
    cabinet = cabinet.union(front_frame)

    runner_y = CABINET_WIDTH / 2.0 - SHELL_THICKNESS - RUNNER_WIDTH / 2.0
    for index in range(DRAWER_COUNT):
        z_center = _drawer_center_z(index)
        for y_sign in (-1.0, 1.0):
            runner = (
                cq.Workplane("XY")
                .box(RUNNER_DEPTH, RUNNER_WIDTH, RUNNER_HEIGHT)
                .translate((RUNNER_X, y_sign * runner_y, z_center))
            )
            cabinet = cabinet.union(runner)

    bracket_y = CABINET_WIDTH / 2.0 + BRACKET_SIZE_Y / 2.0
    pivot_x = HANDLE_SPREAD / 2.0
    for y_sign in (-1.0, 1.0):
        for x_sign in (-1.0, 1.0):
            bracket = (
                cq.Workplane("XY")
                .box(BRACKET_SIZE_X, BRACKET_SIZE_Y, BRACKET_SIZE_Z)
                .translate((x_sign * pivot_x, y_sign * bracket_y, HANDLE_PIVOT_Z))
            )
            cabinet = cabinet.union(bracket)

    return cabinet


def _build_drawer_shape():
    front_panel = (
        cq.Workplane("XY")
        .box(DRAWER_FRONT_THICKNESS, DRAWER_FRONT_WIDTH, DRAWER_FRONT_HEIGHT)
        .translate((-DRAWER_FRONT_THICKNESS / 2.0, 0.0, 0.0))
    )
    grip_recess = (
        cq.Workplane("XY")
        .box(
            DRAWER_FRONT_THICKNESS * 0.7,
            DRAWER_FRONT_WIDTH * 0.72,
            0.022,
        )
        .translate((-DRAWER_FRONT_THICKNESS * 0.18, 0.0, DRAWER_FRONT_HEIGHT * 0.18))
    )
    front_panel = front_panel.cut(grip_recess)

    tray_center_x = -(DRAWER_FRONT_THICKNESS + DRAWER_TRAY_DEPTH / 2.0)
    tray_center_z = -0.004
    tray_outer = (
        cq.Workplane("XY")
        .box(DRAWER_TRAY_DEPTH, DRAWER_TRAY_WIDTH, DRAWER_TRAY_HEIGHT)
        .translate((tray_center_x, 0.0, tray_center_z))
    )
    tray_inner = (
        cq.Workplane("XY")
        .box(
            DRAWER_TRAY_DEPTH - 2.0 * DRAWER_WALL_THICKNESS,
            DRAWER_TRAY_WIDTH - 2.0 * DRAWER_WALL_THICKNESS,
            DRAWER_TRAY_HEIGHT - DRAWER_BOTTOM_THICKNESS,
        )
        .translate((tray_center_x, 0.0, tray_center_z + DRAWER_BOTTOM_THICKNESS / 2.0))
    )
    drawer_shell = tray_outer.cut(tray_inner)

    slide_center_x = -0.23
    slide_center_y = DRAWER_TRAY_WIDTH / 2.0 + DRAWER_SLIDE_WIDTH / 2.0 - 0.002
    left_slide = (
        cq.Workplane("XY")
        .box(DRAWER_SLIDE_DEPTH, DRAWER_SLIDE_WIDTH, DRAWER_SLIDE_HEIGHT)
        .translate((slide_center_x, -slide_center_y, 0.0))
    )
    right_slide = (
        cq.Workplane("XY")
        .box(DRAWER_SLIDE_DEPTH, DRAWER_SLIDE_WIDTH, DRAWER_SLIDE_HEIGHT)
        .translate((slide_center_x, slide_center_y, 0.0))
    )

    return front_panel.union(drawer_shell).union(left_slide).union(right_slide)


def _build_handle_shape():
    grip = _x_cylinder(HANDLE_TUBE_RADIUS, HANDLE_SPREAD, 0.0, 0.0, -HANDLE_DROP)
    front_leg = _z_cylinder(HANDLE_TUBE_RADIUS, HANDLE_DROP, -HANDLE_SPREAD / 2.0, 0.0, -HANDLE_DROP)
    rear_leg = _z_cylinder(HANDLE_TUBE_RADIUS, HANDLE_DROP, HANDLE_SPREAD / 2.0, 0.0, -HANDLE_DROP)
    front_pivot = _x_cylinder(
        HANDLE_TUBE_RADIUS * 0.95,
        HANDLE_PIVOT_LENGTH,
        -HANDLE_SPREAD / 2.0,
        0.0,
        0.0,
    )
    rear_pivot = _x_cylinder(
        HANDLE_TUBE_RADIUS * 0.95,
        HANDLE_PIVOT_LENGTH,
        HANDLE_SPREAD / 2.0,
        0.0,
        0.0,
    )
    return grip.union(front_leg).union(rear_leg).union(front_pivot).union(rear_pivot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_drawer_cabinet")
    model.material("cabinet_steel", rgba=(0.31, 0.35, 0.40, 1.0))
    model.material("drawer_steel", rgba=(0.39, 0.43, 0.48, 1.0))
    model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))

    cabinet = model.part("cabinet")
    inner_width = CABINET_WIDTH - 2.0 * SHELL_THICKNESS
    front_x = CABINET_DEPTH / 2.0 - FRONT_FRAME_THICKNESS / 2.0
    top_bottom_extra = FRAME_MARGIN_Z - SHELL_THICKNESS

    cabinet.visual(
        Box((SHELL_THICKNESS, CABINET_WIDTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_DEPTH / 2.0 + SHELL_THICKNESS / 2.0, 0.0, 0.0)),
        material="cabinet_steel",
        name="back_panel",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, SHELL_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.0, -CABINET_WIDTH / 2.0 + SHELL_THICKNESS / 2.0, 0.0)),
        material="cabinet_steel",
        name="left_side",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, SHELL_THICKNESS, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.0, CABINET_WIDTH / 2.0 - SHELL_THICKNESS / 2.0, 0.0)),
        material="cabinet_steel",
        name="right_side",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0 - SHELL_THICKNESS / 2.0)),
        material="cabinet_steel",
        name="top_panel",
    )
    cabinet.visual(
        Box((CABINET_DEPTH, CABINET_WIDTH, SHELL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -CABINET_HEIGHT / 2.0 + SHELL_THICKNESS / 2.0)),
        material="cabinet_steel",
        name="bottom_panel",
    )
    cabinet.visual(
        Box((FRONT_FRAME_THICKNESS, inner_width, top_bottom_extra)),
        origin=Origin(
            xyz=(front_x, 0.0, CABINET_HEIGHT / 2.0 - SHELL_THICKNESS - top_bottom_extra / 2.0)
        ),
        material="cabinet_steel",
        name="top_frame",
    )
    cabinet.visual(
        Box((FRONT_FRAME_THICKNESS, inner_width, top_bottom_extra)),
        origin=Origin(
            xyz=(front_x, 0.0, -CABINET_HEIGHT / 2.0 + SHELL_THICKNESS + top_bottom_extra / 2.0)
        ),
        material="cabinet_steel",
        name="bottom_frame",
    )
    for index in range(DRAWER_COUNT - 1):
        separator_z = (_drawer_center_z(index) + _drawer_center_z(index + 1)) / 2.0
        cabinet.visual(
            Box((FRONT_FRAME_THICKNESS, inner_width, DRAWER_SEPARATOR)),
            origin=Origin(xyz=(front_x, 0.0, separator_z)),
            material="cabinet_steel",
            name=f"separator_{index}",
        )
    runner_y = CABINET_WIDTH / 2.0 - SHELL_THICKNESS - RUNNER_WIDTH / 2.0
    for index in range(DRAWER_COUNT):
        z_center = _drawer_center_z(index)
        cabinet.visual(
            Box((RUNNER_DEPTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(RUNNER_X, -runner_y, z_center)),
            material="cabinet_steel",
            name=f"runner_{index}_0",
        )
        cabinet.visual(
            Box((RUNNER_DEPTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(RUNNER_X, runner_y, z_center)),
            material="cabinet_steel",
            name=f"runner_{index}_1",
        )
    bracket_y = CABINET_WIDTH / 2.0 + BRACKET_SIZE_Y / 2.0
    pivot_x = HANDLE_SPREAD / 2.0
    for y_sign, side_index in ((-1.0, 0), (1.0, 1)):
        for x_sign, mount_index in ((-1.0, 0), (1.0, 1)):
            cabinet.visual(
                Box((BRACKET_SIZE_X, BRACKET_SIZE_Y, BRACKET_SIZE_Z)),
                origin=Origin(
                    xyz=(x_sign * pivot_x, y_sign * bracket_y, HANDLE_PIVOT_Z)
                ),
                material="cabinet_steel",
                name=f"handle_mount_{side_index}_{mount_index}",
            )

    drawer_mesh = _build_drawer_shape()
    for index in range(DRAWER_COUNT):
        drawer = model.part(f"drawer_{index}")
        drawer.visual(
            mesh_from_cadquery(drawer_mesh, f"drawer_{index}"),
            material="drawer_steel",
            name="drawer_shell",
        )
        model.articulation(
            f"cabinet_to_drawer_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=drawer,
            origin=Origin(xyz=(CABINET_DEPTH / 2.0, 0.0, _drawer_center_z(index))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=140.0,
                velocity=0.35,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    handle_mesh = _build_handle_shape()
    left_handle = model.part("left_handle")
    left_handle.visual(
        mesh_from_cadquery(handle_mesh, "left_handle"),
        material="handle_finish",
        name="handle_bar",
    )
    model.articulation(
        "cabinet_to_left_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_handle,
        origin=Origin(xyz=(0.0, -HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=0.0,
            upper=HANDLE_SWING,
        ),
    )

    right_handle = model.part("right_handle")
    right_handle.visual(
        mesh_from_cadquery(handle_mesh, "right_handle"),
        material="handle_finish",
        name="handle_bar",
    )
    model.articulation(
        "cabinet_to_right_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_handle,
        origin=Origin(xyz=(0.0, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=0.0,
            upper=HANDLE_SWING,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")

    drawers = [object_model.get_part(f"drawer_{index}") for index in range(DRAWER_COUNT)]
    drawer_joints = [
        object_model.get_articulation(f"cabinet_to_drawer_{index}") for index in range(DRAWER_COUNT)
    ]

    drawer_positions = [ctx.part_world_position(drawer) for drawer in drawers]
    stacked_evenly = (
        all(position is not None for position in drawer_positions)
        and all(
            drawer_positions[index + 1][2] > drawer_positions[index][2] + 0.08
            for index in range(DRAWER_COUNT - 1)
        )
        and all(
            abs(
                (drawer_positions[index + 1][2] - drawer_positions[index][2]) - DRAWER_PITCH
            )
            < 0.003
            for index in range(DRAWER_COUNT - 1)
        )
    )
    ctx.check(
        "six drawers are evenly stacked",
        stacked_evenly,
        details=f"drawer_positions={drawer_positions}",
    )

    for index, drawer in enumerate(drawers):
        ctx.allow_overlap(
            cabinet,
            drawer,
            elem_a=f"runner_{index}_0",
            elem_b="drawer_shell",
            reason="The fixed runner is simplified as a solid guide that intentionally captures the drawer-side slide shoe.",
        )
        ctx.allow_overlap(
            cabinet,
            drawer,
            elem_a=f"runner_{index}_1",
            elem_b="drawer_shell",
            reason="The fixed runner is simplified as a solid guide that intentionally captures the drawer-side slide shoe.",
        )

    for index, (drawer, joint) in enumerate(zip(drawers, drawer_joints)):
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="yz",
            min_overlap=0.10,
            name=f"drawer_{index} sits within cabinet width and height at rest",
        )
        rest_pos = ctx.part_world_position(drawer)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper}):
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="yz",
                min_overlap=0.10,
                name=f"drawer_{index} stays aligned on its runners",
            )
            extended_pos = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer_{index} extends forward",
            rest_pos is not None
            and extended_pos is not None
            and upper is not None
            and extended_pos[0] > rest_pos[0] + 0.18,
            details=f"rest={rest_pos}, extended={extended_pos}, upper={upper}",
        )

    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_joint = object_model.get_articulation("cabinet_to_left_handle")
    right_joint = object_model.get_articulation("cabinet_to_right_handle")

    ctx.allow_overlap(
        cabinet,
        left_handle,
        reason="The folding side handle pivots are intentionally captured inside the side mounting blocks.",
    )
    ctx.allow_overlap(
        cabinet,
        right_handle,
        reason="The folding side handle pivots are intentionally captured inside the side mounting blocks.",
    )

    ctx.expect_gap(
        cabinet,
        left_handle,
        axis="y",
        min_gap=-0.002,
        max_gap=0.04,
        name="left handle stows close to cabinet side",
    )
    ctx.expect_gap(
        right_handle,
        cabinet,
        axis="y",
        min_gap=-0.002,
        max_gap=0.04,
        name="right handle stows close to cabinet side",
    )

    left_closed = ctx.part_world_aabb(left_handle)
    with ctx.pose({left_joint: left_joint.motion_limits.upper}):
        left_open = ctx.part_world_aabb(left_handle)
    ctx.check(
        "left handle swings upward and outward",
        left_closed is not None
        and left_open is not None
        and left_open[0][2] > left_closed[0][2] + 0.05
        and left_open[0][1] < left_closed[0][1] - 0.05,
        details=f"closed={left_closed}, open={left_open}",
    )

    right_closed = ctx.part_world_aabb(right_handle)
    with ctx.pose({right_joint: right_joint.motion_limits.upper}):
        right_open = ctx.part_world_aabb(right_handle)
    ctx.check(
        "right handle swings upward and outward",
        right_closed is not None
        and right_open is not None
        and right_open[0][2] > right_closed[0][2] + 0.05
        and right_open[1][1] > right_closed[1][1] + 0.05,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()


object_model = build_object_model()
