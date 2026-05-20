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


TRAY_LENGTH_Y = 0.78
TRAY_REACH_X = 0.395
TRAY_BACKSET_X = 0.025
TRAY_TOTAL_X = TRAY_REACH_X + TRAY_BACKSET_X
TRAY_HEIGHT = 0.03


def _make_sleeve_mesh() -> object:
    width = 0.10
    depth = 0.064
    height = 0.47
    wall = 0.006

    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height, centered=(True, True, False))
        .translate((0.0, 0.0, wall))
    )
    collar = cq.Workplane("XY").box(width + 0.018, depth + 0.018, 0.028, centered=(True, True, False))
    return outer.cut(inner).union(collar)


def _make_tray_mesh() -> object:
    wall = 0.008
    bottom = 0.006

    outer = (
        cq.Workplane("XY")
        .box(TRAY_TOTAL_X, TRAY_LENGTH_Y, TRAY_HEIGHT, centered=(False, True, False))
        .translate((-TRAY_BACKSET_X, 0.0, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(
            TRAY_TOTAL_X - 2.0 * wall,
            TRAY_LENGTH_Y - 2.0 * wall,
            TRAY_HEIGHT,
            centered=(False, True, False),
        )
        .translate((-TRAY_BACKSET_X + wall, 0.0, bottom))
    )
    return outer.cut(inner)


def _add_caster(
    model: ArticulatedObject,
    base,
    *,
    index: int,
    x: float,
    y: float,
    frame_material,
    wheel_material,
    hub_material,
) -> None:
    base.visual(
        Box((0.05, 0.04, 0.012)),
        origin=Origin(xyz=(x, y, 0.067)),
        material=frame_material,
        name=f"caster_plate_{index}",
    )
    for side in (-1.0, 1.0):
        base.visual(
            Box((0.04, 0.006, 0.042)),
            origin=Origin(xyz=(x, y + side * 0.012, 0.045)),
            material=frame_material,
            name=f"caster_cheek_{index}_{int(side > 0)}",
        )

    caster = model.part(f"caster_{index}")
    caster.visual(
        Cylinder(radius=0.03, length=0.018),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wheel_material,
        name="tire",
    )
    caster.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )

    model.articulation(
        f"base_to_caster_{index}",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=caster,
        origin=Origin(xyz=(x, y, 0.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=15.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pneumatic_overbed_table")

    frame = model.material("frame", rgba=(0.73, 0.75, 0.77, 1.0))
    sleeve_finish = model.material("sleeve_finish", rgba=(0.86, 0.87, 0.88, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    tray_surface = model.material("tray_surface", rgba=(0.80, 0.73, 0.60, 1.0))
    trim = model.material("trim", rgba=(0.30, 0.32, 0.34, 1.0))
    paddle_finish = model.material("paddle_finish", rgba=(0.28, 0.30, 0.33, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    sleeve_mesh = mesh_from_cadquery(_make_sleeve_mesh(), "overbed_table_sleeve")
    tray_mesh = mesh_from_cadquery(_make_tray_mesh(), "overbed_table_tray")

    base = model.part("base")
    base.visual(
        Box((0.10, 0.40, 0.03)),
        origin=Origin(xyz=(0.02, 0.0, 0.075)),
        material=frame,
        name="side_spine",
    )
    base.visual(
        Box((0.56, 0.07, 0.03)),
        origin=Origin(xyz=(0.28, -0.17, 0.075)),
        material=frame,
        name="leg_0",
    )
    base.visual(
        Box((0.56, 0.07, 0.03)),
        origin=Origin(xyz=(0.28, 0.17, 0.075)),
        material=frame,
        name="leg_1",
    )
    base.visual(
        Box((0.10, 0.12, 0.08)),
        origin=Origin(xyz=(0.00, 0.0, 0.130)),
        material=frame,
        name="pedestal",
    )
    base.visual(
        Box((0.08, 0.10, 0.016)),
        origin=Origin(xyz=(0.00, 0.0, 0.162)),
        material=trim,
        name="pedestal_cap",
    )

    caster_positions = (
        (0.05, -0.17),
        (0.05, 0.17),
        (0.51, -0.17),
        (0.51, 0.17),
    )
    for caster_index, (caster_x, caster_y) in enumerate(caster_positions):
        _add_caster(
            model,
            base,
            index=caster_index,
            x=caster_x,
            y=caster_y,
            frame_material=frame,
            wheel_material=wheel_rubber,
            hub_material=steel,
        )

    sleeve = model.part("sleeve")
    sleeve.visual(sleeve_mesh, material=sleeve_finish, name="sleeve_shell")

    column = model.part("column")
    column.visual(
        Box((0.056, 0.036, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=steel,
        name="lift_tube",
    )
    column.visual(
        Box((0.090, 0.054, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=trim,
        name="guide_collar",
    )
    column.visual(
        Box((0.082, 0.060, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        material=trim,
        name="head_block",
    )
    column.visual(
        Box((0.22, 0.055, 0.028)),
        origin=Origin(xyz=(0.11, 0.0, 0.178)),
        material=trim,
        name="head_arm",
    )
    column.visual(
        Cylinder(radius=0.008, length=0.12),
        origin=Origin(xyz=(0.22, 0.0, 0.178), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_pivot",
    )
    column.visual(
        Box((0.03, 0.055, 0.034)),
        origin=Origin(xyz=(0.188, 0.0, 0.177)),
        material=trim,
        name="arm_socket",
    )

    tray = model.part("tray")
    tray.visual(
        tray_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=tray_surface,
        name="tray_shell",
    )
    tray.visual(
        Box((0.06, 0.14, 0.014)),
        origin=Origin(xyz=(0.042, 0.0, 0.005)),
        material=trim,
        name="tilt_block",
    )
    tray.visual(
        Box((0.055, 0.020, 0.008)),
        origin=Origin(xyz=(0.360, 0.0, 0.036)),
        material=trim,
        name="clip_rim_plate",
    )
    tray.visual(
        Box((0.024, 0.032, 0.016)),
        origin=Origin(xyz=(0.09, 0.355, 0.008)),
        material=trim,
        name="paddle_mount",
    )
    tray.visual(
        Box((0.012, 0.030, 0.012)),
        origin=Origin(xyz=(TRAY_REACH_X - 0.006, -0.18, 0.038)),
        material=trim,
        name="clip_mount_0",
    )
    tray.visual(
        Box((0.012, 0.030, 0.012)),
        origin=Origin(xyz=(TRAY_REACH_X - 0.006, 0.18, 0.038)),
        material=trim,
        name="clip_mount_1",
    )

    paddle = model.part("paddle")
    paddle.visual(
        Box((0.020, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.032, -0.010)),
        material=paddle_finish,
        name="lever",
    )
    paddle.visual(
        Box((0.020, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, -0.005)),
        material=paddle_finish,
        name="pivot_block",
    )

    clip_bar = model.part("clip_bar")
    clip_bar.visual(
        Cylinder(radius=0.0055, length=0.58),
        origin=Origin(xyz=(0.040, 0.0, 0.016), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bar",
    )
    clip_bar.visual(
        Box((0.038, 0.014, 0.010)),
        origin=Origin(xyz=(0.020, -0.18, 0.010)),
        material=steel,
        name="support_0",
    )
    clip_bar.visual(
        Box((0.038, 0.014, 0.010)),
        origin=Origin(xyz=(0.020, 0.18, 0.010)),
        material=steel,
        name="support_1",
    )
    clip_bar.visual(
        Box((0.012, 0.030, 0.012)),
        origin=Origin(xyz=(0.006, -0.18, 0.0)),
        material=trim,
        name="hinge_0",
    )
    clip_bar.visual(
        Box((0.012, 0.030, 0.012)),
        origin=Origin(xyz=(0.006, 0.18, 0.0)),
        material=trim,
        name="hinge_1",
    )

    model.articulation(
        "base_to_sleeve",
        ArticulationType.FIXED,
        parent=base,
        child=sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )
    model.articulation(
        "sleeve_to_column",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=0.24),
    )
    model.articulation(
        "column_to_tray",
        ArticulationType.REVOLUTE,
        parent=column,
        child=tray,
        origin=Origin(xyz=(0.22, 0.0, 0.178)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.2, lower=0.0, upper=0.72),
    )
    model.articulation(
        "tray_to_paddle",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=paddle,
        origin=Origin(xyz=(0.09, 0.355, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "tray_to_clip_bar",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=clip_bar,
        origin=Origin(xyz=(TRAY_REACH_X, 0.0, 0.038)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    sleeve = object_model.get_part("sleeve")
    column = object_model.get_part("column")
    tray = object_model.get_part("tray")
    paddle = object_model.get_part("paddle")
    clip_bar = object_model.get_part("clip_bar")

    slide = object_model.get_articulation("sleeve_to_column")
    tilt = object_model.get_articulation("column_to_tray")
    paddle_joint = object_model.get_articulation("tray_to_paddle")
    clip_joint = object_model.get_articulation("tray_to_clip_bar")

    slide_limits = slide.motion_limits
    tilt_limits = tilt.motion_limits
    paddle_limits = paddle_joint.motion_limits
    clip_limits = clip_joint.motion_limits

    with ctx.pose({slide: 0.0, tilt: 0.0, paddle_joint: 0.0, clip_joint: 0.0}):
        ctx.expect_within(
            column,
            sleeve,
            axes="xy",
            inner_elem="lift_tube",
            outer_elem="sleeve_shell",
            margin=0.0,
            name="lift tube stays nested in the sleeve footprint",
        )
        ctx.expect_overlap(
            column,
            sleeve,
            axes="z",
            elem_a="lift_tube",
            elem_b="sleeve_shell",
            min_overlap=0.14,
            name="collapsed lift tube remains engaged in the sleeve",
        )
        ctx.expect_gap(
            tray,
            paddle,
            axis="z",
            positive_elem="tray_shell",
            negative_elem="lever",
            min_gap=0.004,
            max_gap=0.05,
            name="release paddle hangs just below the tray",
        )
        ctx.expect_overlap(
            tray,
            paddle,
            axes="x",
            elem_a="tray_shell",
            elem_b="lever",
            min_overlap=0.018,
            name="release paddle stays beneath the tray side",
        )
        ctx.expect_overlap(
            clip_bar,
            tray,
            axes="y",
            elem_a="bar",
            elem_b="tray_shell",
            min_overlap=0.50,
            name="clip bar spans the tray edge",
        )
        ctx.expect_gap(
            clip_bar,
            tray,
            axis="x",
            positive_elem="bar",
            negative_elem="tray_shell",
            min_gap=0.0,
            max_gap=0.08,
            name="clip bar parks right at the tray rim",
        )

    if slide_limits is not None and slide_limits.upper is not None:
        rest_column_pos = ctx.part_world_position(column)
        with ctx.pose({slide: slide_limits.upper}):
            ctx.expect_within(
                column,
                sleeve,
                axes="xy",
                inner_elem="lift_tube",
                outer_elem="sleeve_shell",
                margin=0.0,
                name="extended lift tube stays centered in the sleeve",
            )
            ctx.expect_overlap(
                column,
                sleeve,
                axes="z",
                elem_a="lift_tube",
                elem_b="sleeve_shell",
                min_overlap=0.14,
                name="extended lift tube keeps retained insertion",
            )
            extended_column_pos = ctx.part_world_position(column)
        ctx.check(
            "column extends upward",
            rest_column_pos is not None
            and extended_column_pos is not None
            and extended_column_pos[2] > rest_column_pos[2] + 0.18,
            details=f"rest={rest_column_pos}, extended={extended_column_pos}",
        )

    with ctx.pose({slide: 0.0, tilt: 0.0}):
        rest_tray_aabb = ctx.part_world_aabb(tray)
    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({slide: 0.0, tilt: tilt_limits.upper}):
            tilted_tray_aabb = ctx.part_world_aabb(tray)
        ctx.check(
            "tray tilts upward at the free edge",
            rest_tray_aabb is not None
            and tilted_tray_aabb is not None
            and tilted_tray_aabb[1][2] > rest_tray_aabb[1][2] + 0.12,
            details=f"rest={rest_tray_aabb}, tilted={tilted_tray_aabb}",
        )

    with ctx.pose({paddle_joint: 0.0}):
        rest_paddle_aabb = ctx.part_world_aabb(paddle)
    if paddle_limits is not None and paddle_limits.upper is not None:
        with ctx.pose({paddle_joint: paddle_limits.upper}):
            raised_paddle_aabb = ctx.part_world_aabb(paddle)
        ctx.check(
            "paddle can be pulled upward",
            rest_paddle_aabb is not None
            and raised_paddle_aabb is not None
            and raised_paddle_aabb[1][2] > rest_paddle_aabb[1][2] + 0.02,
            details=f"rest={rest_paddle_aabb}, raised={raised_paddle_aabb}",
        )

    with ctx.pose({clip_joint: 0.0}):
        rest_clip_aabb = ctx.part_world_aabb(clip_bar)
    if clip_limits is not None and clip_limits.upper is not None:
        with ctx.pose({clip_joint: clip_limits.upper}):
            raised_clip_aabb = ctx.part_world_aabb(clip_bar)
        ctx.check(
            "clip bar flips upward from the tray edge",
            rest_clip_aabb is not None
            and raised_clip_aabb is not None
            and raised_clip_aabb[1][2] > rest_clip_aabb[1][2] + 0.02,
            details=f"rest={rest_clip_aabb}, raised={raised_clip_aabb}",
        )

    ctx.check(
        "four caster spin joints exist",
        all(
            object_model.get_articulation(f"base_to_caster_{index}") is not None
            for index in range(4)
        ),
        details="Expected four continuously rotating caster joints.",
    )

    return ctx.report()


object_model = build_object_model()
