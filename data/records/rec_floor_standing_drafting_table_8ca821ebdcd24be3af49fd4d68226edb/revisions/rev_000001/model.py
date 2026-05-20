from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hollow_tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    bottom_z: float,
    top_z: float,
):
    """Return a closed annular tube mesh in the part's local coordinates."""
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, bottom_z), (outer_radius, top_z)],
            [(inner_radius, bottom_z), (inner_radius, top_z)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_drafting_stand")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    warm_board = model.material("warm_board", rgba=(0.78, 0.70, 0.58, 1.0))
    board_edge = model.material("board_edge", rgba=(0.50, 0.43, 0.34, 1.0))
    drawer_plastic = model.material("drawer_plastic", rgba=(0.34, 0.36, 0.38, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.075, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
        name="cross_foot_x",
    )
    base.visual(
        Box((0.075, 0.28, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=graphite,
        name="cross_foot_y",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_steel,
        name="pedestal_plate",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=graphite,
        name="lower_hub",
    )
    base.visual(
        _hollow_tube_mesh(
            "drafting_outer_sleeve",
            outer_radius=0.032,
            inner_radius=0.024,
            bottom_z=0.060,
            top_z=0.320,
        ),
        material=dark_steel,
        name="outer_sleeve",
    )
    base.visual(
        _hollow_tube_mesh(
            "drafting_top_collar",
            outer_radius=0.039,
            inner_radius=0.024,
            bottom_z=0.300,
            top_z=0.340,
        ),
        material=graphite,
        name="top_collar",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(0.044, 0.0, 0.302), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="height_clamp_stem",
    )
    base.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.073, 0.0, 0.302)),
        material=rubber,
        name="height_clamp_knob",
    )
    for index, (x, y) in enumerate(
        ((0.175, 0.0), (-0.175, 0.0), (0.0, 0.135), (0.0, -0.135))
    ):
        base.visual(
            Sphere(radius=0.014),
            origin=Origin(xyz=(x, y, 0.018)),
            material=rubber,
            name=f"foot_pad_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.38, 0.30, 0.34)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    center_post = model.part("center_post")
    center_post.visual(
        Cylinder(radius=0.017, length=0.530),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=brushed_steel,
        name="inner_tube",
    )
    center_post.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=graphite,
        name="upper_collar",
    )
    center_post.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=graphite,
        name="slide_stop_collar",
    )
    center_post.visual(
        Box((0.064, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=graphite,
        name="hinge_block",
    )
    center_post.visual(
        Box((0.040, 0.790, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=graphite,
        name="yoke_crossmember",
    )
    center_post.visual(
        Box((0.050, 0.026, 0.080)),
        origin=Origin(xyz=(0.0, 0.390, 0.336)),
        material=graphite,
        name="yoke_plate_0",
    )
    center_post.visual(
        Box((0.050, 0.026, 0.080)),
        origin=Origin(xyz=(0.0, -0.390, 0.336)),
        material=graphite,
        name="yoke_plate_1",
    )
    center_post.visual(
        Cylinder(radius=0.010, length=0.830),
        origin=Origin(xyz=(0.0, 0.0, 0.354), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_pin",
    )
    center_post.inertial = Inertial.from_geometry(
        Box((0.08, 0.66, 0.64)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
    )

    board = model.part("board")
    board.visual(
        Cylinder(radius=0.018, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    board.visual(
        Box((0.080, 0.560, 0.020)),
        origin=Origin(xyz=(0.020, 0.0, 0.028)),
        material=board_edge,
        name="barrel_saddle",
    )
    board.visual(
        Box((0.500, 0.680, 0.022)),
        origin=Origin(xyz=(0.210, 0.0, 0.029)),
        material=warm_board,
        name="board_panel",
    )
    board.visual(
        Box((0.510, 0.028, 0.032)),
        origin=Origin(xyz=(0.210, 0.340, 0.045)),
        material=board_edge,
        name="side_frame_0",
    )
    board.visual(
        Box((0.510, 0.028, 0.032)),
        origin=Origin(xyz=(0.210, -0.340, 0.045)),
        material=board_edge,
        name="side_frame_1",
    )
    board.visual(
        Box((0.030, 0.680, 0.032)),
        origin=Origin(xyz=(-0.040, 0.0, 0.045)),
        material=board_edge,
        name="hinge_edge_frame",
    )
    board.visual(
        Box((0.052, 0.640, 0.016)),
        origin=Origin(xyz=(0.460, 0.0, 0.026)),
        material=board_edge,
        name="lower_lip_shelf",
    )
    board.visual(
        Box((0.026, 0.650, 0.044)),
        origin=Origin(xyz=(0.480, 0.0, 0.054)),
        material=board_edge,
        name="lower_lip_fence",
    )
    board.visual(
        Box((0.350, 0.030, 0.020)),
        origin=Origin(xyz=(0.270, 0.250, -0.040)),
        material=dark_steel,
        name="guide_runner_0",
    )
    board.visual(
        Box((0.350, 0.030, 0.020)),
        origin=Origin(xyz=(0.270, -0.250, -0.040)),
        material=dark_steel,
        name="guide_runner_1",
    )
    for index, (x, y) in enumerate(
        ((0.120, 0.250), (0.400, 0.250), (0.120, -0.250), (0.400, -0.250))
    ):
        board.visual(
            Box((0.026, 0.032, 0.052)),
            origin=Origin(xyz=(x, y, -0.005)),
            material=dark_steel,
            name=f"runner_standoff_{index}",
        )
    board.inertial = Inertial.from_geometry(
        Box((0.54, 0.72, 0.11)),
        mass=2.2,
        origin=Origin(xyz=(0.22, 0.0, 0.02)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.320, 0.400, 0.012)),
        origin=Origin(xyz=(0.260, 0.0, -0.091)),
        material=drawer_plastic,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.320, 0.018, 0.035)),
        origin=Origin(xyz=(0.260, 0.200, -0.078)),
        material=drawer_plastic,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.320, 0.018, 0.035)),
        origin=Origin(xyz=(0.260, -0.200, -0.078)),
        material=drawer_plastic,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.018, 0.400, 0.040)),
        origin=Origin(xyz=(0.429, 0.0, -0.077)),
        material=drawer_plastic,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.014, 0.400, 0.034)),
        origin=Origin(xyz=(0.093, 0.0, -0.078)),
        material=drawer_plastic,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.285, 0.075, 0.012)),
        origin=Origin(xyz=(0.260, 0.228, -0.056)),
        material=brushed_steel,
        name="slide_flange_0",
    )
    drawer.visual(
        Box((0.285, 0.075, 0.012)),
        origin=Origin(xyz=(0.260, -0.228, -0.056)),
        material=brushed_steel,
        name="slide_flange_1",
    )
    drawer.visual(
        Box((0.018, 0.130, 0.014)),
        origin=Origin(xyz=(0.441, 0.0, -0.060)),
        material=brushed_steel,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.35, 0.47, 0.07)),
        mass=0.45,
        origin=Origin(xyz=(0.22, 0.0, -0.07)),
    )

    model.articulation(
        "post_height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=center_post,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=0.160),
    )
    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=center_post,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.354), rpy=(0.0, 0.28, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=-0.22, upper=0.55),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.28, lower=0.0, upper=0.160),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    center_post = object_model.get_part("center_post")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")
    height_slide = object_model.get_articulation("post_height_slide")
    board_tilt = object_model.get_articulation("board_tilt")
    drawer_slide = object_model.get_articulation("drawer_slide")

    ctx.allow_overlap(
        center_post,
        board,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The steel hinge pin is intentionally captured inside the board hinge barrel.",
    )
    ctx.expect_contact(
        center_post,
        board,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        contact_tol=0.010,
        name="tilting board is carried by the top hinge pin",
    )

    ctx.expect_within(
        center_post,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="telescoping post stays centered in outer sleeve",
    )
    ctx.expect_overlap(
        center_post,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.080,
        name="telescoping post visibly remains nested at rest",
    )

    ctx.expect_contact(
        drawer,
        board,
        elem_a="slide_flange_0",
        elem_b="guide_runner_0",
        contact_tol=0.001,
        name="drawer flange rides on first guide runner",
    )
    ctx.expect_contact(
        drawer,
        board,
        elem_a="slide_flange_1",
        elem_b="guide_runner_1",
        contact_tol=0.001,
        name="drawer flange rides on second guide runner",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="x",
        elem_a="slide_flange_0",
        elem_b="guide_runner_0",
        min_overlap=0.120,
        name="closed drawer remains carried by runners",
    )

    rest_post_z = ctx.part_world_position(center_post)[2]
    with ctx.pose({height_slide: 0.160}):
        extended_post_z = ctx.part_world_position(center_post)[2]
        ctx.expect_overlap(
            center_post,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.055,
            name="raised post retains nested insertion",
        )

    rest_drawer_x = ctx.part_world_position(drawer)[0]
    with ctx.pose({drawer_slide: 0.160}):
        extended_drawer_x = ctx.part_world_position(drawer)[0]
        ctx.expect_overlap(
            drawer,
            board,
            axes="x",
            elem_a="slide_flange_0",
            elem_b="guide_runner_0",
            min_overlap=0.090,
            name="extended drawer remains on guide runners",
        )

    rest_board_aabb = ctx.part_element_world_aabb(board, elem="lower_lip_fence")
    with ctx.pose({board_tilt: 0.45}):
        tilted_board_aabb = ctx.part_element_world_aabb(board, elem="lower_lip_fence")

    ctx.check(
        "center post slides upward",
        extended_post_z > rest_post_z + 0.12,
        details=f"rest_z={rest_post_z}, extended_z={extended_post_z}",
    )
    ctx.check(
        "drawer slides out from below board",
        extended_drawer_x > rest_drawer_x + 0.12,
        details=f"rest_x={rest_drawer_x}, extended_x={extended_drawer_x}",
    )
    ctx.check(
        "board tilt changes lip height",
        rest_board_aabb is not None
        and tilted_board_aabb is not None
        and abs(tilted_board_aabb[0][2] - rest_board_aabb[0][2]) > 0.035,
        details=f"rest={rest_board_aabb}, tilted={tilted_board_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
