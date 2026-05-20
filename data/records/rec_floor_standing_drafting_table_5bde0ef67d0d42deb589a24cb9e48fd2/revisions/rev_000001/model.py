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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_drafting_table")

    steel = model.material("satin_black_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed_metal = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    board_mat = model.material("sealed_beech_board", rgba=(0.78, 0.58, 0.34, 1.0))
    paper_mat = model.material("warm_drawing_paper", rgba=(0.93, 0.91, 0.84, 1.0))
    drawer_mat = model.material("painted_drawer", rgba=(0.50, 0.52, 0.50, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.305, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="rubber_foot_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.285, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=steel,
        name="round_base",
    )
    pedestal.visual(
        Cylinder(radius=0.115, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=dark_metal,
        name="pedestal_boss",
    )

    outer_sleeve_shape = (
        cq.Workplane("XY")
        .circle(0.058)
        .circle(0.041)
        .extrude(0.55)
        .translate((0.0, 0.0, 0.115))
    )
    pedestal.visual(
        mesh_from_cadquery(outer_sleeve_shape, "outer_sleeve"),
        material=dark_metal,
        name="outer_sleeve",
    )
    top_collar_shape = (
        cq.Workplane("XY")
        .circle(0.082)
        .circle(0.041)
        .extrude(0.045)
        .translate((0.0, 0.0, 0.635))
    )
    pedestal.visual(
        mesh_from_cadquery(top_collar_shape, "height_collar"),
        material=brushed_metal,
        name="height_collar",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.034, length=0.77),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=brushed_metal,
        name="inner_mast",
    )
    column.visual(
        Cylinder(radius=0.036, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.245)),
        material=dark_metal,
        name="lower_guide_bushing",
    )
    column.visual(
        Cylinder(radius=0.036, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_metal,
        name="upper_guide_bushing",
    )
    column.visual(
        Cylinder(radius=0.076, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=dark_metal,
        name="stop_collar",
    )
    column.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=dark_metal,
        name="mast_cap",
    )
    column.visual(
        Box((1.070, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.402)),
        material=dark_metal,
        name="yoke_bridge",
    )
    column.visual(
        Box((0.045, 0.100, 0.210)),
        origin=Origin(xyz=(-0.535, 0.0, 0.500)),
        material=dark_metal,
        name="yoke_cheek_0",
    )
    column.visual(
        Box((0.045, 0.100, 0.210)),
        origin=Origin(xyz=(0.535, 0.0, 0.500)),
        material=dark_metal,
        name="yoke_cheek_1",
    )

    board = model.part("board")
    board.visual(
        Box((0.92, 0.36, 0.026)),
        origin=Origin(xyz=(0.0, -0.190, -0.010)),
        material=board_mat,
        name="drawing_board",
    )
    board.visual(
        Box((0.84, 0.285, 0.002)),
        origin=Origin(xyz=(0.0, -0.205, 0.004)),
        material=paper_mat,
        name="drawing_sheet",
    )
    board.visual(
        Box((0.92, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.372, -0.001)),
        material=brushed_metal,
        name="lower_rule_strip",
    )
    board.visual(
        Box((0.026, 0.34, 0.014)),
        origin=Origin(xyz=(-0.445, -0.190, -0.002)),
        material=brushed_metal,
        name="side_rule_strip",
    )
    board.visual(
        Cylinder(radius=0.018, length=1.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="trunnion_rod",
    )
    board.visual(
        Box((0.82, 0.300, 0.015)),
        origin=Origin(xyz=(0.0, -0.255, -0.028)),
        material=dark_metal,
        name="shelf_mount_plate",
    )
    board.visual(
        Box((0.030, 0.300, 0.096)),
        origin=Origin(xyz=(-0.405, -0.255, -0.076)),
        material=dark_metal,
        name="shelf_side_0",
    )
    board.visual(
        Box((0.030, 0.300, 0.096)),
        origin=Origin(xyz=(0.405, -0.255, -0.076)),
        material=dark_metal,
        name="shelf_side_1",
    )
    board.visual(
        Box((0.82, 0.026, 0.086)),
        origin=Origin(xyz=(0.0, -0.120, -0.081)),
        material=dark_metal,
        name="shelf_back_rail",
    )
    board.visual(
        Box((0.022, 0.270, 0.012)),
        origin=Origin(xyz=(-0.365, -0.245, -0.130)),
        material=brushed_metal,
        name="drawer_runner_0",
    )
    board.visual(
        Box((0.022, 0.270, 0.012)),
        origin=Origin(xyz=(0.365, -0.245, -0.130)),
        material=brushed_metal,
        name="drawer_runner_1",
    )

    shelf_drawer = model.part("shelf_drawer")
    shelf_drawer.visual(
        Box((0.690, 0.245, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=drawer_mat,
        name="drawer_tray",
    )
    shelf_drawer.visual(
        Box((0.740, 0.020, 0.065)),
        origin=Origin(xyz=(0.0, -0.130, 0.002)),
        material=drawer_mat,
        name="drawer_front",
    )
    shelf_drawer.visual(
        Box((0.250, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.144, 0.010)),
        material=brushed_metal,
        name="drawer_pull",
    )
    shelf_drawer.visual(
        Box((0.050, 0.235, 0.010)),
        origin=Origin(xyz=(-0.355, 0.0, -0.009)),
        material=brushed_metal,
        name="drawer_glide_0",
    )
    shelf_drawer.visual(
        Box((0.050, 0.235, 0.010)),
        origin=Origin(xyz=(0.355, 0.0, -0.009)),
        material=brushed_metal,
        name="drawer_glide_1",
    )

    model.articulation(
        "pedestal_to_column",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.22),
    )
    model.articulation(
        "column_to_board",
        ArticulationType.REVOLUTE,
        parent=column,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=-0.12, upper=1.05),
    )
    model.articulation(
        "board_to_shelf_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=shelf_drawer,
        origin=Origin(xyz=(0.0, -0.265, -0.110)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.30, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    pedestal = object_model.get_part("pedestal")
    column = object_model.get_part("column")
    board = object_model.get_part("board")
    drawer = object_model.get_part("shelf_drawer")
    height_slide = object_model.get_articulation("pedestal_to_column")
    board_tilt = object_model.get_articulation("column_to_board")
    drawer_slide = object_model.get_articulation("board_to_shelf_drawer")

    ctx.expect_within(
        column,
        pedestal,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="telescoping column stays centered in pedestal sleeve",
    )
    ctx.expect_overlap(
        column,
        pedestal,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.10,
        name="collapsed column retains insertion in sleeve",
    )
    with ctx.pose({height_slide: 0.22}):
        ctx.expect_within(
            column,
            pedestal,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.004,
            name="raised column stays centered in pedestal sleeve",
        )
        ctx.expect_overlap(
            column,
            pedestal,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.055,
            name="raised column remains captured in sleeve",
        )

    ctx.expect_within(
        drawer,
        board,
        axes="x",
        inner_elem="drawer_tray",
        outer_elem="shelf_mount_plate",
        margin=0.02,
        name="drawer tray fits between fixed shelf sides",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="y",
        elem_a="drawer_tray",
        elem_b="drawer_runner_0",
        min_overlap=0.12,
        name="drawer rests on straight runners when stowed",
    )
    with ctx.pose({drawer_slide: 0.20}):
        ctx.expect_overlap(
            drawer,
            board,
            axes="y",
            elem_a="drawer_tray",
            elem_b="drawer_runner_0",
            min_overlap=0.025,
            name="extended drawer still has runner engagement",
        )

    rest_aabb = ctx.part_world_aabb(board)
    with ctx.pose({board_tilt: 0.90}):
        tilted_aabb = ctx.part_world_aabb(board)
    ctx.check(
        "board trunnion raises the front edge when tilted",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > rest_aabb[1][2] + 0.15,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
