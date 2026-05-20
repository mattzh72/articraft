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


def _pedestal_base_mesh() -> object:
    """Low, heavy cast base with softened edges."""
    return (
        cq.Workplane("XY")
        .box(0.72, 0.54, 0.075)
        .edges("|Z")
        .fillet(0.035)
        .edges(">Z")
        .fillet(0.012)
        .edges("<Z")
        .fillet(0.006)
    )


def _square_sleeve_mesh() -> object:
    """Open square telescoping sleeve, hollow through the middle."""
    outer = cq.Workplane("XY").box(0.18, 0.18, 0.57)
    inner = cq.Workplane("XY").box(0.116, 0.116, 0.62)
    return outer.cut(inner).edges("|Z").fillet(0.010)


def _drafting_board_mesh() -> object:
    """Thin rectangular board slab with lightly rounded manufactured edges."""
    return (
        cq.Workplane("XY")
        .box(1.20, 0.75, 0.040)
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.006)
    )


def _tray_mesh() -> object:
    """Shallow U-channel pencil tray drawer with slide lips as one connected body."""
    bottom = cq.Workplane("XY").box(0.78, 0.16, 0.014)
    front_wall = cq.Workplane("XY").box(0.78, 0.016, 0.045).translate((0.0, -0.080, 0.016))
    rear_wall = cq.Workplane("XY").box(0.78, 0.012, 0.032).translate((0.0, 0.080, 0.010))
    side_wall_a = cq.Workplane("XY").box(0.018, 0.16, 0.038).translate((-0.390, 0.0, 0.012))
    side_wall_b = cq.Workplane("XY").box(0.018, 0.16, 0.038).translate((0.390, 0.0, 0.012))
    slide_lip_a = cq.Workplane("XY").box(0.040, 0.34, 0.018).translate((-0.408, 0.0, 0.023))
    slide_lip_b = cq.Workplane("XY").box(0.040, 0.34, 0.018).translate((0.408, 0.0, 0.023))
    return (
        bottom.union(front_wall)
        .union(rear_wall)
        .union(side_wall_a)
        .union(side_wall_b)
        .union(slide_lip_a)
        .union(slide_lip_b)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_drafting_table")

    cast_metal = model.material("dark_cast_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    black_metal = model.material("black_enamel", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    board_green = model.material("drafting_green", rgba=(0.58, 0.66, 0.55, 1.0))
    wood_edge = model.material("pale_wood_edge", rgba=(0.72, 0.55, 0.35, 1.0))
    tray_plastic = model.material("charcoal_tray", rgba=(0.025, 0.027, 0.030, 1.0))
    pencil_yellow = model.material("pencil_yellow", rgba=(0.95, 0.72, 0.12, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_base_mesh(), "pedestal_base"),
        material=cast_metal,
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        name="heavy_base",
    )
    pedestal.visual(
        mesh_from_cadquery(_square_sleeve_mesh(), "pedestal_sleeve"),
        material=black_metal,
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        name="sleeve_tube",
    )
    pedestal.visual(
        Box((0.28, 0.28, 0.018)),
        material=cast_metal,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        name="sleeve_foot_plate",
    )
    # Four visible hold-down bolts on the plate, all seated on the plate.
    for ix, x in enumerate((-0.105, 0.105)):
        for iy, y in enumerate((-0.105, 0.105)):
            pedestal.visual(
                Cylinder(radius=0.014, length=0.010),
                material=satin_steel,
                origin=Origin(xyz=(x, y, 0.098)),
                name=f"bolt_{ix}_{iy}",
            )
    column = model.part("column")
    column.visual(
        Cylinder(radius=0.046, length=1.00),
        material=satin_steel,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        name="inner_column",
    )
    column.visual(
        Cylinder(radius=0.060, length=0.060),
        material=black_metal,
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        name="top_collar",
    )
    column.visual(
        Box((1.38, 0.080, 0.050)),
        material=black_metal,
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        name="trunnion_crosshead",
    )
    column.visual(
        Box((0.116, 0.116, 0.050)),
        material=black_metal,
        origin=Origin(xyz=(0.0, 0.0, -0.410)),
        name="glide_block_0",
    )
    column.visual(
        Box((0.116, 0.116, 0.050)),
        material=black_metal,
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        name="glide_block_1",
    )
    for i, x in enumerate((-0.680, 0.680)):
        column.visual(
            Box((0.050, 0.130, 0.230)),
            material=black_metal,
            origin=Origin(xyz=(x, 0.0, 0.610)),
            name=f"trunnion_cheek_{i}",
        )
        column.visual(
            Cylinder(radius=0.048, length=0.014),
            material=satin_steel,
            origin=Origin(xyz=(0.648 if x > 0.0 else -0.648, 0.0, 0.585), rpy=(0.0, math.pi / 2.0, 0.0)),
            name=f"bearing_face_{i}",
        )

    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_drafting_board_mesh(), "drafting_board_slab"),
        material=board_green,
        origin=Origin(xyz=(0.0, -0.050, 0.040)),
        name="drafting_surface",
    )
    # Wood edge banding makes the panel read as a real drafting board.
    board.visual(
        Box((1.22, 0.020, 0.052)),
        material=wood_edge,
        origin=Origin(xyz=(0.0, -0.435, 0.040)),
        name="front_edge_band",
    )
    board.visual(
        Box((1.22, 0.020, 0.052)),
        material=wood_edge,
        origin=Origin(xyz=(0.0, 0.335, 0.040)),
        name="rear_edge_band",
    )
    board.visual(
        Box((0.020, 0.75, 0.052)),
        material=wood_edge,
        origin=Origin(xyz=(-0.610, -0.050, 0.040)),
        name="side_edge_band_0",
    )
    board.visual(
        Box((0.020, 0.75, 0.052)),
        material=wood_edge,
        origin=Origin(xyz=(0.610, -0.050, 0.040)),
        name="side_edge_band_1",
    )
    board.visual(
        Cylinder(radius=0.032, length=1.282),
        material=satin_steel,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="trunnion_axle",
    )
    board.visual(
        Box((1.10, 0.052, 0.030)),
        material=black_metal,
        origin=Origin(xyz=(0.0, 0.140, 0.008)),
        name="underside_rear_rail",
    )
    board.visual(
        Box((0.050, 0.62, 0.028)),
        material=black_metal,
        origin=Origin(xyz=(-0.460, -0.090, 0.006)),
        name="underside_side_rail_0",
    )
    board.visual(
        Box((0.050, 0.62, 0.028)),
        material=black_metal,
        origin=Origin(xyz=(0.460, -0.090, 0.006)),
        name="underside_side_rail_1",
    )
    board.visual(
        Box((0.044, 0.500, 0.025)),
        material=satin_steel,
        origin=Origin(xyz=(-0.420, -0.200, 0.0075)),
        name="drawer_runner_0",
    )
    board.visual(
        Box((0.044, 0.500, 0.025)),
        material=satin_steel,
        origin=Origin(xyz=(0.420, -0.200, 0.0075)),
        name="drawer_runner_1",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_mesh(), "pencil_tray_drawer"),
        material=tray_plastic,
        origin=Origin(),
        name="tray_body",
    )
    tray.visual(
        Cylinder(radius=0.006, length=0.650),
        material=pencil_yellow,
        origin=Origin(xyz=(0.0, -0.015, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        name="pencil",
    )

    model.articulation(
        "pedestal_to_column",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=0.0, upper=0.250),
    )
    model.articulation(
        "column_to_board",
        ArticulationType.REVOLUTE,
        parent=column,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.20, upper=0.90),
    )
    model.articulation(
        "board_to_tray",
        ArticulationType.PRISMATIC,
        parent=board,
        child=tray,
        origin=Origin(xyz=(0.0, -0.255, -0.037)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.280),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    column = object_model.get_part("column")
    board = object_model.get_part("board")
    tray = object_model.get_part("tray")
    height_slide = object_model.get_articulation("pedestal_to_column")
    board_tilt = object_model.get_articulation("column_to_board")
    tray_slide = object_model.get_articulation("board_to_tray")

    ctx.allow_overlap(
        column,
        pedestal,
        elem_a="glide_block_1",
        elem_b="sleeve_tube",
        reason="The upper glide block is a hidden sliding shoe seated inside the pedestal sleeve proxy.",
    )
    ctx.allow_overlap(
        column,
        pedestal,
        elem_a="glide_block_0",
        elem_b="sleeve_tube",
        reason="The lower glide block is a hidden sliding shoe seated inside the pedestal sleeve proxy.",
    )
    ctx.allow_overlap(
        board,
        tray,
        elem_a="drawer_runner_0",
        elem_b="tray_body",
        reason="The pencil tray lip is intentionally captured by the straight runner under the board.",
    )
    ctx.allow_overlap(
        board,
        tray,
        elem_a="drawer_runner_1",
        elem_b="tray_body",
        reason="The opposite pencil tray lip is intentionally captured by the paired straight runner.",
    )

    ctx.expect_within(
        column,
        pedestal,
        axes="xy",
        inner_elem="inner_column",
        outer_elem="sleeve_tube",
        margin=0.004,
        name="telescoping column is centered in sleeve",
    )
    ctx.expect_overlap(
        column,
        pedestal,
        axes="z",
        elem_a="inner_column",
        elem_b="sleeve_tube",
        min_overlap=0.19,
        name="column remains inserted when lowered",
    )
    ctx.expect_within(
        column,
        pedestal,
        axes="xy",
        inner_elem="glide_block_1",
        outer_elem="sleeve_tube",
        margin=0.001,
        name="upper glide block stays inside sleeve",
    )
    ctx.expect_overlap(
        column,
        pedestal,
        axes="z",
        elem_a="glide_block_1",
        elem_b="sleeve_tube",
        min_overlap=0.04,
        name="upper glide block remains in sleeve",
    )
    ctx.expect_within(
        column,
        pedestal,
        axes="xy",
        inner_elem="glide_block_0",
        outer_elem="sleeve_tube",
        margin=0.001,
        name="lower glide block stays inside sleeve",
    )
    ctx.expect_overlap(
        column,
        pedestal,
        axes="z",
        elem_a="glide_block_0",
        elem_b="sleeve_tube",
        min_overlap=0.04,
        name="lower glide block remains in sleeve",
    )
    with ctx.pose({height_slide: 0.250}):
        ctx.expect_within(
            column,
            pedestal,
            axes="xy",
            inner_elem="inner_column",
            outer_elem="sleeve_tube",
            margin=0.004,
            name="extended column stays centered in sleeve",
        )
        ctx.expect_overlap(
            column,
            pedestal,
            axes="z",
            elem_a="inner_column",
            elem_b="sleeve_tube",
            min_overlap=0.19,
            name="extended column retains insertion",
        )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.280}):
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "pencil tray slides toward the front",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] < rest_tray_pos[1] - 0.25,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    with ctx.pose({board_tilt: 0.75}):
        tilted_board_aabb = ctx.part_element_world_aabb(board, elem="drafting_surface")
    with ctx.pose({board_tilt: 0.0}):
        flat_board_aabb = ctx.part_element_world_aabb(board, elem="drafting_surface")
    ctx.check(
        "board tilt raises one edge about the trunnion",
        flat_board_aabb is not None
        and tilted_board_aabb is not None
        and (tilted_board_aabb[1][2] - flat_board_aabb[1][2]) > 0.20,
        details=f"flat={flat_board_aabb}, tilted={tilted_board_aabb}",
    )

    ctx.expect_contact(
        board,
        tray,
        elem_a="drawer_runner_0",
        elem_b="tray_body",
        contact_tol=0.001,
        name="tray rides on straight runner",
    )
    ctx.expect_contact(
        board,
        tray,
        elem_a="drawer_runner_1",
        elem_b="tray_body",
        contact_tol=0.001,
        name="tray is retained by paired runner",
    )

    return ctx.report()


object_model = build_object_model()
