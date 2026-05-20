from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="film_clapperboard")

    board_black = model.material("board_black", rgba=(0.11, 0.11, 0.12, 1.0))
    stripe_white = model.material("stripe_white", rgba=(0.95, 0.95, 0.94, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.72, 0.74, 0.77, 1.0))

    board_width = 0.280
    board_height = 0.220
    board_thickness = 0.008

    arm_width = 0.292
    arm_height = 0.032
    arm_thickness = 0.010

    hinge_radius = 0.0035
    hinge_axis_y = board_thickness / 2.0 + hinge_radius
    hinge_z = board_height

    board = model.part("board")
    board.visual(
        Box((board_width, board_thickness, board_height)),
        origin=Origin(xyz=(0.0, 0.0, board_height / 2.0)),
        material=board_black,
        name="board_panel",
    )

    trim_thickness = 0.0012
    trim_embed = 0.0002
    trim_y = board_thickness / 2.0 + trim_thickness / 2.0 - trim_embed
    board.visual(
        Box((0.014, trim_thickness, 0.190)),
        origin=Origin(xyz=(-board_width / 2.0 + 0.007, trim_y, 0.095)),
        material=stripe_white,
        name="left_trim",
    )
    board.visual(
        Box((0.014, trim_thickness, 0.190)),
        origin=Origin(xyz=(board_width / 2.0 - 0.007, trim_y, 0.095)),
        material=stripe_white,
        name="right_trim",
    )
    board.visual(
        Box((0.246, trim_thickness, 0.006)),
        origin=Origin(xyz=(0.0, trim_y, 0.012)),
        material=stripe_white,
        name="bottom_trim",
    )
    board.visual(
        Box((0.185, trim_thickness, 0.004)),
        origin=Origin(xyz=(0.033, trim_y, 0.062)),
        material=stripe_white,
        name="writing_line_1",
    )
    board.visual(
        Box((0.185, trim_thickness, 0.004)),
        origin=Origin(xyz=(0.033, trim_y, 0.098)),
        material=stripe_white,
        name="writing_line_2",
    )
    board.visual(
        Box((0.185, trim_thickness, 0.004)),
        origin=Origin(xyz=(0.033, trim_y, 0.134)),
        material=stripe_white,
        name="writing_line_3",
    )
    board.visual(
        Box((0.060, trim_thickness, 0.020)),
        origin=Origin(xyz=(-0.085, trim_y, 0.152)),
        material=stripe_white,
        name="scene_box",
    )

    hinge_leaf_thickness = hinge_axis_y - board_thickness / 2.0
    leaf_z = hinge_z - 0.006
    leaf_height = 0.012
    leaf_length = 0.074
    left_leaf_x = -0.080
    right_leaf_x = 0.080
    leaf_y = board_thickness / 2.0 + hinge_leaf_thickness / 2.0
    board.visual(
        Box((leaf_length, hinge_leaf_thickness, leaf_height)),
        origin=Origin(xyz=(left_leaf_x, leaf_y, leaf_z)),
        material=hinge_metal,
        name="left_hinge_leaf",
    )
    board.visual(
        Box((leaf_length, hinge_leaf_thickness, leaf_height)),
        origin=Origin(xyz=(right_leaf_x, leaf_y, leaf_z)),
        material=hinge_metal,
        name="right_hinge_leaf",
    )

    knuckle_length = 0.074
    knuckle_origin_rpy = Origin(rpy=(0.0, pi / 2.0, 0.0))
    board.visual(
        Cylinder(radius=hinge_radius, length=knuckle_length),
        origin=Origin(xyz=(left_leaf_x, hinge_axis_y, hinge_z), rpy=knuckle_origin_rpy.rpy),
        material=hinge_metal,
        name="left_board_knuckle",
    )
    board.visual(
        Cylinder(radius=hinge_radius, length=knuckle_length),
        origin=Origin(xyz=(right_leaf_x, hinge_axis_y, hinge_z), rpy=knuckle_origin_rpy.rpy),
        material=hinge_metal,
        name="right_board_knuckle",
    )
    board.inertial = Inertial.from_geometry(
        Box((board_width, board_thickness, board_height)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, board_height / 2.0)),
    )

    clapper = model.part("clapper_arm")
    clapper.visual(
        Box((arm_width, arm_thickness, arm_height)),
        origin=Origin(xyz=(0.0, arm_thickness / 2.0, -arm_height / 2.0)),
        material=board_black,
        name="clapper_bar",
    )
    clapper.visual(
        Cylinder(radius=hinge_radius, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_metal,
        name="center_arm_knuckle",
    )

    stripe_y = arm_thickness - 0.0011
    stripe_size = (0.036, 0.0018, 0.007)
    stripe_pitch = 0.38
    stripe_xs = (-0.108, -0.054, 0.0, 0.054, 0.108)
    for index, sx in enumerate(stripe_xs, start=1):
        clapper.visual(
            Box(stripe_size),
            origin=Origin(xyz=(sx, stripe_y, -0.015), rpy=(0.0, stripe_pitch, 0.0)),
            material=stripe_white,
            name=f"stripe_{index}",
        )

    clapper.inertial = Inertial.from_geometry(
        Box((arm_width, arm_thickness, arm_height)),
        mass=0.28,
        origin=Origin(xyz=(0.0, arm_thickness / 2.0, -arm_height / 2.0)),
    )

    model.articulation(
        "board_to_clapper",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapper,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    clapper = object_model.get_part("clapper_arm")
    hinge = object_model.get_articulation("board_to_clapper")

    ctx.expect_gap(
        clapper,
        board,
        axis="y",
        positive_elem="clapper_bar",
        negative_elem="board_panel",
        min_gap=0.003,
        max_gap=0.0045,
        name="closed clapper hangs just ahead of the board face",
    )
    ctx.expect_overlap(
        clapper,
        board,
        axes="x",
        elem_a="clapper_bar",
        elem_b="board_panel",
        min_overlap=0.270,
        name="clapper spans nearly the full board width",
    )
    ctx.expect_overlap(
        clapper,
        board,
        axes="z",
        elem_a="clapper_bar",
        elem_b="board_panel",
        min_overlap=0.028,
        name="closed clapper covers the board's top band",
    )

    limits = hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.lower}):
            closed_aabb = ctx.part_element_world_aabb(clapper, elem="clapper_bar")
        with ctx.pose({hinge: limits.upper}):
            open_aabb = ctx.part_element_world_aabb(clapper, elem="clapper_bar")

        ctx.check(
            "clapper free edge lifts when opened",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] > closed_aabb[0][2] + 0.012,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )
        ctx.check(
            "clapper swings outward from the board",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.018,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
