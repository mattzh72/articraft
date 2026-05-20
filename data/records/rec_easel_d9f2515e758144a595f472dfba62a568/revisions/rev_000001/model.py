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


def _box(part, size, xyz, *, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _cylinder(part, radius, length, xyz, *, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _rotate_x(xyz, angle):
    x, y, z = xyz
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="childrens_art_easel")

    wood = model.material("wood", rgba=(0.79, 0.63, 0.41, 1.0))
    birch = model.material("birch", rgba=(0.90, 0.82, 0.66, 1.0))
    whiteboard = model.material("whiteboard", rgba=(0.97, 0.97, 0.95, 1.0))
    chalkboard = model.material("chalkboard", rgba=(0.19, 0.38, 0.24, 1.0))
    clamp_red = model.material("clamp_red", rgba=(0.86, 0.24, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))

    width = 0.68
    front_depth = 0.04
    rear_depth = 0.036
    front_top_z = 1.08
    front_leg_height = 1.10
    rear_leg_height = 1.16
    rear_rest_angle = -0.42
    spreader_rest_angle = -1.05

    front_board = model.part("front_board")
    _box(
        front_board,
        (0.052, front_depth, front_leg_height),
        (-0.305, 0.0, front_leg_height / 2.0),
        material=wood,
        name="leg_0",
    )
    _box(
        front_board,
        (0.052, front_depth, front_leg_height),
        (0.305, 0.0, front_leg_height / 2.0),
        material=wood,
        name="leg_1",
    )
    front_board.visual(
        Box((width, front_depth, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 1.0475)),
        material=wood,
        name="top_rail",
    )
    front_board.visual(
        Box((0.58, front_depth, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=wood,
        name="bottom_rail",
    )
    _box(
        front_board,
        (0.55, 0.006, 0.72),
        (0.0, 0.019, 0.62),
        material=whiteboard,
        name="drawing_surface",
    )
    _box(
        front_board,
        (0.60, 0.022, 0.052),
        (0.0, 0.026, 0.255),
        material=wood,
        name="tray_back",
    )
    _box(
        front_board,
        (0.60, 0.110, 0.020),
        (0.0, 0.078, 0.236),
        material=birch,
        name="tray_shelf",
    )
    _box(
        front_board,
        (0.022, 0.088, 0.046),
        (-0.289, 0.067, 0.247),
        material=wood,
        name="tray_cap_0",
    )
    _box(
        front_board,
        (0.022, 0.088, 0.046),
        (0.289, 0.067, 0.247),
        material=wood,
        name="tray_cap_1",
    )
    _box(
        front_board,
        (0.070, 0.016, 0.028),
        (-0.245, -0.028, 1.114),
        material=wood,
        name="rear_hinge_block_0",
    )
    _box(
        front_board,
        (0.070, 0.016, 0.028),
        (0.245, -0.028, 1.114),
        material=wood,
        name="rear_hinge_block_1",
    )
    _box(
        front_board,
        (0.064, 0.019, 0.020),
        (-0.272, -0.0295, 0.489),
        material=wood,
        name="spreader_hinge_block_0",
    )
    _box(
        front_board,
        (0.064, 0.019, 0.020),
        (0.272, -0.0295, 0.489),
        material=wood,
        name="spreader_hinge_block_1",
    )

    rear_board = model.part("rear_board")
    rear_rpy = (rear_rest_angle, 0.0, 0.0)
    _box(
        rear_board,
        (0.048, rear_depth, rear_leg_height),
        _rotate_x((-0.255, -0.002, -rear_leg_height / 2.0), rear_rest_angle),
        material=wood,
        name="rear_leg_0",
        rpy=rear_rpy,
    )
    _box(
        rear_board,
        (0.048, rear_depth, rear_leg_height),
        _rotate_x((0.255, -0.002, -rear_leg_height / 2.0), rear_rest_angle),
        material=wood,
        name="rear_leg_1",
        rpy=rear_rpy,
    )
    _box(
        rear_board,
        (0.64, rear_depth, 0.058),
        _rotate_x((0.0, -0.002, -0.029), rear_rest_angle),
        material=wood,
        name="rear_top_rail",
        rpy=rear_rpy,
    )
    rear_board.visual(
        Box((0.54, rear_depth, 0.050)),
        origin=Origin(xyz=_rotate_x((0.0, -0.002, -0.905), rear_rest_angle), rpy=rear_rpy),
        material=wood,
        name="rear_bottom_rail",
    )
    rear_board.visual(
        Box((0.46, 0.020, 0.014)),
        origin=Origin(xyz=_rotate_x((0.0, 0.024, -0.886), rear_rest_angle), rpy=rear_rpy),
        material=birch,
        name="spreader_stop",
    )
    _box(
        rear_board,
        (0.52, 0.006, 0.68),
        _rotate_x((0.0, -0.020, -0.58), rear_rest_angle),
        material=chalkboard,
        name="rear_surface",
        rpy=rear_rpy,
    )
    _box(
        rear_board,
        (0.056, 0.020, 0.040),
        _rotate_x((-0.236, -0.010, -0.025), rear_rest_angle),
        material=steel,
        name="rear_hinge_ear_0",
        rpy=rear_rpy,
    )
    _box(
        rear_board,
        (0.056, 0.020, 0.040),
        _rotate_x((0.236, -0.010, -0.025), rear_rest_angle),
        material=steel,
        name="rear_hinge_ear_1",
        rpy=rear_rpy,
    )

    paper_clamp = model.part("paper_clamp")
    _cylinder(
        paper_clamp,
        radius=0.010,
        length=0.58,
        xyz=(0.0, 0.010, 0.0),
        material=steel,
        name="clamp_hinge",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    paper_clamp.visual(
        Box((0.56, 0.024, 0.082)),
        origin=Origin(xyz=(0.0, 0.010, -0.050)),
        material=clamp_red,
        name="clamp_bar",
    )
    paper_clamp.visual(
        Box((0.56, 0.038, 0.022)),
        origin=Origin(xyz=(0.0, 0.020, -0.090)),
        material=clamp_red,
        name="clamp_lip",
    )

    rear_spreader = model.part("rear_spreader")
    spreader_rpy = (spreader_rest_angle, 0.0, 0.0)
    _box(
        rear_spreader,
        (0.44, 0.018, 0.39),
        _rotate_x((0.0, -0.004, -0.195), spreader_rest_angle),
        material=birch,
        name="spreader_plank",
        rpy=spreader_rpy,
    )
    rear_spreader.visual(
        Box((0.44, 0.030, 0.022)),
        origin=Origin(xyz=_rotate_x((0.0, -0.002, -0.390), spreader_rest_angle), rpy=spreader_rpy),
        material=wood,
        name="rear_shoe",
    )
    _cylinder(
        rear_spreader,
        radius=0.009,
        length=0.055,
        xyz=(-0.225, 0.0, 0.0),
        material=steel,
        name="spreader_hinge_0",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    _cylinder(
        rear_spreader,
        radius=0.009,
        length=0.055,
        xyz=(0.225, 0.0, 0.0),
        material=steel,
        name="spreader_hinge_1",
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    model.articulation(
        "rear_board_hinge",
        ArticulationType.REVOLUTE,
        parent=front_board,
        child=rear_board,
        origin=Origin(xyz=(0.0, -0.040, front_top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.24,
            upper=0.30,
        ),
    )
    model.articulation(
        "paper_clamp_hinge",
        ArticulationType.REVOLUTE,
        parent=front_board,
        child=paper_clamp,
        origin=Origin(xyz=(0.0, 0.024, 1.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=1.32,
        ),
    )
    model.articulation(
        "rear_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_board,
        child=rear_spreader,
        origin=Origin(xyz=(0.0, -0.028, 0.470)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.34,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_board = object_model.get_part("front_board")
    rear_board = object_model.get_part("rear_board")
    paper_clamp = object_model.get_part("paper_clamp")
    rear_spreader = object_model.get_part("rear_spreader")

    rear_hinge = object_model.get_articulation("rear_board_hinge")
    clamp_hinge = object_model.get_articulation("paper_clamp_hinge")
    spreader_hinge = object_model.get_articulation("rear_spreader_hinge")

    ctx.expect_overlap(
        front_board,
        paper_clamp,
        axes="x",
        elem_a="top_rail",
        elem_b="clamp_bar",
        min_overlap=0.50,
        name="paper clamp spans the front board width",
    )
    ctx.expect_gap(
        paper_clamp,
        front_board,
        axis="y",
        positive_elem="clamp_bar",
        negative_elem="top_rail",
        min_gap=0.0,
        max_gap=0.018,
        name="paper clamp rests close to the top edge",
    )
    ctx.expect_gap(
        front_board,
        rear_board,
        axis="y",
        positive_elem="bottom_rail",
        negative_elem="rear_bottom_rail",
        min_gap=0.14,
        name="rear board stands behind the front board",
    )
    ctx.allow_overlap(
        rear_spreader,
        rear_board,
        elem_a="rear_shoe",
        elem_b="spreader_stop",
        reason="The rear spreader is simplified as a seated shoe bearing into a stop pad instead of a more detailed bracketed latch.",
    )
    ctx.expect_gap(
        rear_spreader,
        rear_board,
        axis="y",
        positive_elem="rear_shoe",
        negative_elem="spreader_stop",
        max_gap=0.050,
        max_penetration=0.030,
        name="rear spreader reaches the rear board stop",
    )
    ctx.expect_overlap(
        rear_spreader,
        rear_board,
        axes="x",
        elem_a="rear_shoe",
        elem_b="spreader_stop",
        min_overlap=0.42,
        name="rear spreader spans the easel width",
    )

    clamp_closed = ctx.part_element_world_aabb(paper_clamp, elem="clamp_lip")
    with ctx.pose({clamp_hinge: clamp_hinge.motion_limits.upper}):
        clamp_open = ctx.part_element_world_aabb(paper_clamp, elem="clamp_lip")
    ctx.check(
        "paper clamp flips upward",
        clamp_closed is not None
        and clamp_open is not None
        and clamp_open[1][1] > clamp_closed[1][1] + 0.05
        and clamp_open[1][2] > clamp_closed[1][2] + 0.05,
        details=f"closed={clamp_closed}, open={clamp_open}",
    )

    with ctx.pose(
        {
            rear_hinge: rear_hinge.motion_limits.lower,
            spreader_hinge: spreader_hinge.motion_limits.lower,
        }
    ):
        rear_folded = ctx.part_element_world_aabb(rear_board, elem="rear_bottom_rail")
        spreader_folded = ctx.part_element_world_aabb(rear_spreader, elem="rear_shoe")
    with ctx.pose(
        {
            rear_hinge: rear_hinge.motion_limits.upper,
            spreader_hinge: spreader_hinge.motion_limits.upper,
        }
    ):
        rear_open = ctx.part_element_world_aabb(rear_board, elem="rear_bottom_rail")
        spreader_open = ctx.part_element_world_aabb(rear_spreader, elem="rear_shoe")

    ctx.check(
        "rear board swings farther back",
        rear_folded is not None
        and rear_open is not None
        and rear_open[1][1] < rear_folded[1][1] - 0.14,
        details=f"folded={rear_folded}, open={rear_open}",
    )
    ctx.check(
        "rear spreader deploys toward the rear board",
        spreader_folded is not None
        and spreader_open is not None
        and spreader_open[1][1] < spreader_folded[1][1] - 0.10,
        details=f"folded={spreader_folded}, open={spreader_open}",
    )

    return ctx.report()


object_model = build_object_model()
