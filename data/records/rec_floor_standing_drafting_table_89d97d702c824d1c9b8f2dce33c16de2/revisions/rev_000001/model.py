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
    model = ArticulatedObject(name="compact_drafting_stand")

    dark_metal = model.material("satin_black_metal", color=(0.05, 0.055, 0.06, 1.0))
    brushed = model.material("brushed_aluminum", color=(0.62, 0.64, 0.62, 1.0))
    board_face = model.material("warm_gray_drafting_board", color=(0.77, 0.75, 0.68, 1.0))
    board_edge = model.material("dark_board_edge", color=(0.14, 0.13, 0.12, 1.0))
    rubber = model.material("black_knurled_rubber", color=(0.015, 0.014, 0.013, 1.0))
    steel = model.material("threaded_steel", color=(0.45, 0.46, 0.47, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.175, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_metal,
        name="pedestal_disk",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brushed,
        name="center_boss",
    )
    base.visual(
        Box((0.38, 0.055, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="cross_foot_x",
    )
    base.visual(
        Box((0.055, 0.38, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="cross_foot_y",
    )

    # Four walls and two thicker collars make a visibly hollow square sleeve.
    sleeve_height = 0.36
    sleeve_center_z = 0.245
    base.visual(
        Box((0.070, 0.011, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.0295, sleeve_center_z)),
        material=dark_metal,
        name="front_sleeve_wall",
    )
    base.visual(
        Box((0.070, 0.011, sleeve_height)),
        origin=Origin(xyz=(0.0, 0.0295, sleeve_center_z)),
        material=dark_metal,
        name="rear_sleeve_wall",
    )
    base.visual(
        Box((0.011, 0.048, sleeve_height)),
        origin=Origin(xyz=(-0.0295, 0.0, sleeve_center_z)),
        material=dark_metal,
        name="side_sleeve_wall_0",
    )
    base.visual(
        Box((0.011, 0.048, sleeve_height)),
        origin=Origin(xyz=(0.0295, 0.0, sleeve_center_z)),
        material=dark_metal,
        name="side_sleeve_wall_1",
    )
    base.visual(
        Box((0.092, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, -0.040, 0.095)),
        material=brushed,
        name="lower_collar_front",
    )
    base.visual(
        Box((0.092, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, 0.040, 0.095)),
        material=brushed,
        name="lower_collar_rear",
    )
    base.visual(
        Box((0.012, 0.068, 0.045)),
        origin=Origin(xyz=(-0.040, 0.0, 0.095)),
        material=brushed,
        name="lower_collar_side_0",
    )
    base.visual(
        Box((0.012, 0.068, 0.045)),
        origin=Origin(xyz=(0.040, 0.0, 0.095)),
        material=brushed,
        name="lower_collar_side_1",
    )
    base.visual(
        Box((0.092, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, -0.040, 0.415)),
        material=brushed,
        name="upper_collar_front",
    )
    base.visual(
        Box((0.092, 0.012, 0.045)),
        origin=Origin(xyz=(0.0, 0.040, 0.415)),
        material=brushed,
        name="upper_collar_rear",
    )
    base.visual(
        Box((0.012, 0.068, 0.045)),
        origin=Origin(xyz=(-0.040, 0.0, 0.415)),
        material=brushed,
        name="upper_collar_side_0",
    )
    base.visual(
        Box((0.012, 0.068, 0.045)),
        origin=Origin(xyz=(0.040, 0.0, 0.415)),
        material=brushed,
        name="upper_collar_side_1",
    )

    post = model.part("post")
    post.visual(
        Box((0.048, 0.048, 0.630)),
        # The post frame is at the sleeve mouth; the bar extends far below it so
        # it remains visibly nested in the outer tube through the height travel.
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=brushed,
        name="inner_mast",
    )
    post.visual(
        Box((0.048, 0.054, 0.048)),
        origin=Origin(xyz=(0.0, -0.018, 0.340)),
        material=dark_metal,
        name="tilt_head_neck",
    )
    post.visual(
        Box((0.048, 0.060, 0.017)),
        origin=Origin(xyz=(0.0, -0.075, 0.3725)),
        material=dark_metal,
        name="upper_neck_strap",
    )
    post.visual(
        Box((0.048, 0.060, 0.017)),
        origin=Origin(xyz=(0.0, -0.075, 0.3075)),
        material=dark_metal,
        name="lower_neck_strap",
    )
    post.visual(
        Box((0.145, 0.060, 0.014)),
        origin=Origin(xyz=(0.0, -0.075, 0.388)),
        material=dark_metal,
        name="upper_yoke_bridge",
    )
    post.visual(
        Box((0.145, 0.060, 0.014)),
        origin=Origin(xyz=(0.0, -0.075, 0.292)),
        material=dark_metal,
        name="lower_yoke_bridge",
    )
    post.visual(
        Box((0.018, 0.060, 0.085)),
        origin=Origin(xyz=(-0.060, -0.075, 0.340)),
        material=dark_metal,
        name="yoke_plate_0",
    )
    post.visual(
        Box((0.018, 0.060, 0.085)),
        origin=Origin(xyz=(0.060, -0.075, 0.340)),
        material=dark_metal,
        name="yoke_plate_1",
    )
    post.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.074, -0.075, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="threaded_boss",
    )

    board = model.part("board")
    board.visual(
        Cylinder(radius=0.023, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="hinge_hub",
    )
    board.visual(
        Box((0.110, 0.060, 0.034)),
        origin=Origin(xyz=(0.0, -0.040, -0.010)),
        material=board_edge,
        name="hub_web",
    )
    board.visual(
        Box((0.700, 0.024, 0.455)),
        origin=Origin(xyz=(0.0, -0.082, -0.075)),
        material=board_face,
        name="panel",
    )
    board.visual(
        Box((0.705, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, -0.081, 0.165)),
        material=board_edge,
        name="top_edge",
    )
    board.visual(
        Box((0.705, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, -0.081, -0.315)),
        material=board_edge,
        name="bottom_edge",
    )
    board.visual(
        Box((0.026, 0.026, 0.455)),
        origin=Origin(xyz=(-0.365, -0.081, -0.075)),
        material=board_edge,
        name="side_edge_0",
    )
    board.visual(
        Box((0.026, 0.026, 0.455)),
        origin=Origin(xyz=(0.365, -0.081, -0.075)),
        material=board_edge,
        name="side_edge_1",
    )
    board.visual(
        Box((0.660, 0.064, 0.018)),
        origin=Origin(xyz=(0.0, -0.112, -0.292)),
        material=brushed,
        name="lower_lip_tray",
    )
    board.visual(
        Box((0.660, 0.015, 0.044)),
        origin=Origin(xyz=(0.0, -0.144, -0.274)),
        material=brushed,
        name="lower_lip_rail",
    )

    knob = model.part("clamp_knob")
    knob.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="threaded_stem",
    )
    knob.visual(
        Cylinder(radius=0.043, length=0.028),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="knob_cap",
    )
    for i in range(10):
        angle = 2.0 * math.pi * i / 10.0
        knob.visual(
            Box((0.030, 0.010, 0.018)),
            origin=Origin(
                xyz=(0.058, 0.043 * math.cos(angle), 0.043 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=rubber,
            name=f"grip_rib_{i}",
        )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.180),
    )
    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=post,
        child=board,
        origin=Origin(xyz=(0.0, -0.075, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=knob,
        origin=Origin(xyz=(0.074, -0.075, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    post = object_model.get_part("post")
    board = object_model.get_part("board")
    knob = object_model.get_part("clamp_knob")
    height_slide = object_model.get_articulation("height_slide")
    board_tilt = object_model.get_articulation("board_tilt")
    knob_spin = object_model.get_articulation("knob_spin")

    ctx.expect_origin_distance(
        post,
        base,
        axes="xy",
        max_dist=0.001,
        name="telescoping mast remains centered in the sleeve footprint",
    )
    ctx.expect_overlap(
        post,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="front_sleeve_wall",
        min_overlap=0.20,
        name="collapsed post is visibly retained inside the outer sleeve",
    )
    with ctx.pose({height_slide: 0.180}):
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="front_sleeve_wall",
            min_overlap=0.09,
            name="extended post remains nested in the sleeve",
        )
        ctx.expect_origin_distance(
            post,
            base,
            axes="xy",
            max_dist=0.001,
            name="extended mast remains coaxial with sleeve",
        )

    ctx.expect_gap(
        knob,
        post,
        axis="x",
        positive_elem="threaded_stem",
        negative_elem="threaded_boss",
        min_gap=0.0,
        max_gap=0.003,
        name="knob stem seats on the separate clamp boss",
    )
    ctx.expect_overlap(
        knob,
        post,
        axes="yz",
        elem_a="threaded_stem",
        elem_b="threaded_boss",
        min_overlap=0.010,
        name="knob threaded axis is aligned with the head boss",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(board, elem="lower_lip_tray")
    with ctx.pose({board_tilt: 0.50}):
        tilted_lip_aabb = ctx.part_element_world_aabb(board, elem="lower_lip_tray")
    ctx.check(
        "board tilt moves the lower lip about the horizontal hinge",
        rest_lip_aabb is not None
        and tilted_lip_aabb is not None
        and abs(tilted_lip_aabb[0][1] - rest_lip_aabb[0][1]) > 0.030,
        details=f"rest={rest_lip_aabb}, tilted={tilted_lip_aabb}",
    )
    ctx.check(
        "clamp knob is a continuous threaded-axis control",
        getattr(knob_spin, "articulation_type", None) == ArticulationType.CONTINUOUS
        and tuple(getattr(knob_spin, "axis", ())) == (1.0, 0.0, 0.0),
        details=f"type={getattr(knob_spin, 'articulation_type', None)}, axis={getattr(knob_spin, 'axis', None)}",
    )

    return ctx.report()


object_model = build_object_model()
