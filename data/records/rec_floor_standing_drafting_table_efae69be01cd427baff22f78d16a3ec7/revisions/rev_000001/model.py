from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_room_drafting_table")

    painted_steel = Material("painted_steel", color=(0.18, 0.20, 0.22, 1.0))
    dark_steel = Material("dark_steel", color=(0.05, 0.055, 0.06, 1.0))
    aluminum = Material("brushed_aluminum", color=(0.62, 0.64, 0.62, 1.0))
    board_green = Material("green_drafting_board", color=(0.28, 0.43, 0.32, 1.0))
    paper = Material("pale_drawing_sheet", color=(0.93, 0.90, 0.80, 1.0))
    tray_gray = Material("tray_gray", color=(0.45, 0.47, 0.48, 1.0))
    drawer_mat = Material("drawer_off_white", color=(0.82, 0.80, 0.72, 1.0))
    rubber = Material("black_rubber", color=(0.01, 0.01, 0.012, 1.0))

    # Root: a broad, classroom-scale floor pedestal with a hollow square sleeve
    # for the height-adjusting center column.
    floor_base = model.part("floor_base")
    floor_base.visual(
        Box((1.05, 0.75, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=painted_steel,
        name="wide_floor_plate",
    )
    floor_base.visual(
        Box((0.88, 0.09, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=painted_steel,
        name="raised_center_spine",
    )
    for i, y in enumerate((-0.32, 0.32)):
        floor_base.visual(
            Box((0.18, 0.07, 0.018)),
            origin=Origin(xyz=(-0.42, y, 0.009)),
            material=rubber,
            name=f"rubber_foot_{i}_0",
        )
        floor_base.visual(
            Box((0.18, 0.07, 0.018)),
            origin=Origin(xyz=(0.42, y, 0.009)),
            material=rubber,
            name=f"rubber_foot_{i}_1",
        )

    sleeve_height = 0.58
    sleeve_center_z = 0.055 + sleeve_height / 2.0
    # Four separate sleeve walls leave a real central clearance for the
    # prismatic column instead of hiding it inside a solid block.
    for x in (-0.0825, 0.0825):
        floor_base.visual(
            Box((0.035, 0.20, sleeve_height)),
            origin=Origin(xyz=(x, 0.0, sleeve_center_z)),
            material=dark_steel,
            name=f"sleeve_x_{0 if x < 0 else 1}",
        )
    for y in (-0.0825, 0.0825):
        floor_base.visual(
            Box((0.13, 0.035, sleeve_height)),
            origin=Origin(xyz=(0.0, y, sleeve_center_z)),
            material=dark_steel,
            name=f"sleeve_y_{0 if y < 0 else 1}",
        )

    height_column = model.part("height_column")
    height_column.visual(
        Box((0.13, 0.13, 0.66)),
        # The lower portion remains engaged inside the open sleeve at all
        # heights; the upper end carries the distinct support head.
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=aluminum,
        name="inner_post",
    )
    height_column.visual(
        Box((0.012, 0.104, 0.20)),
        origin=Origin(xyz=(-0.071, 0.0, 0.12)),
        material=dark_steel,
        name="height_scale_strip",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=floor_base,
        child=height_column,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.18, lower=0.0, upper=0.32),
    )

    # A separate support head/yoke sits on top of the sliding column.  Its
    # different color and fixed seam keep it visually distinct from the
    # height-adjustment post.
    board_support = model.part("board_support")
    board_support.visual(
        Box((0.20, 0.20, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="support_collar",
    )
    board_support.visual(
        Box((0.10, 0.12, 0.12)),
        origin=Origin(xyz=(0.055, 0.0, 0.10)),
        material=dark_steel,
        name="support_riser",
    )
    board_support.visual(
        Box((0.10, 0.42, 0.03)),
        origin=Origin(xyz=(0.12, 0.0, 0.13)),
        material=dark_steel,
        name="hinge_head_block",
    )
    board_support.visual(
        Cylinder(radius=0.012, length=0.46),
        origin=Origin(xyz=(0.16, 0.0, 0.19), rpy=(pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_pin",
    )
    for i, y in enumerate((-0.21, 0.21)):
        board_support.visual(
            Box((0.04, 0.035, 0.12)),
            origin=Origin(xyz=(0.16, y, 0.19)),
            material=dark_steel,
            name=f"yoke_cheek_{i}",
        )

    model.articulation(
        "column_to_support",
        ArticulationType.FIXED,
        parent=height_column,
        child=board_support,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    board = model.part("board")
    board.visual(
        Box((0.80, 1.20, 0.035)),
        origin=Origin(xyz=(-0.44, 0.0, 0.025)),
        material=board_green,
        name="drawing_board",
    )
    board.visual(
        Box((0.70, 1.05, 0.004)),
        origin=Origin(xyz=(-0.44, 0.0, 0.0445)),
        material=paper,
        name="drawing_sheet",
    )
    board.visual(
        Box((0.045, 1.24, 0.04)),
        origin=Origin(xyz=(-0.055, 0.0, 0.0)),
        material=aluminum,
        name="rear_frame",
    )
    board.visual(
        Box((0.045, 1.24, 0.04)),
        origin=Origin(xyz=(-0.825, 0.0, 0.0)),
        material=aluminum,
        name="front_frame",
    )
    for i, y in enumerate((-0.62, 0.62)):
        board.visual(
            Box((0.80, 0.035, 0.04)),
            origin=Origin(xyz=(-0.44, y, 0.0)),
            material=aluminum,
            name=f"side_frame_{i}",
        )

    board.visual(
        Cylinder(radius=0.022, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle",
    )
    board.visual(
        Box((0.11, 0.26, 0.012)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_leaf",
    )

    # Bottom tool tray on the student/user edge of the tilting board.
    board.visual(
        Box((0.11, 1.10, 0.025)),
        origin=Origin(xyz=(-0.885, 0.0, -0.055)),
        material=tray_gray,
        name="tool_tray_shelf",
    )
    board.visual(
        Box((0.025, 1.10, 0.09)),
        origin=Origin(xyz=(-0.827, 0.0, -0.012)),
        material=tray_gray,
        name="tool_tray_back",
    )
    board.visual(
        Box((0.025, 1.10, 0.07)),
        origin=Origin(xyz=(-0.94, 0.0, -0.025)),
        material=tray_gray,
        name="tool_tray_lip",
    )

    # Short guide runners under the board frame carry the shallow accessory
    # drawer; hanger blocks make their support path visible.
    board.visual(
        Box((0.62, 0.035, 0.035)),
        origin=Origin(xyz=(-0.50, -0.28, -0.085)),
        material=dark_steel,
        name="runner_0",
    )
    board.visual(
        Box((0.62, 0.035, 0.035)),
        origin=Origin(xyz=(-0.50, 0.28, -0.085)),
        material=dark_steel,
        name="runner_1",
    )
    for i, y in enumerate((-0.28, 0.28)):
        for j, x in enumerate((-0.22, -0.62)):
            board.visual(
                Box((0.035, 0.035, 0.08)),
                origin=Origin(xyz=(x, y, -0.032)),
                material=dark_steel,
                name=f"runner_hanger_{i}_{j}",
            )

    model.articulation(
        "support_to_board",
        ArticulationType.REVOLUTE,
        parent=board_support,
        child=board,
        origin=Origin(xyz=(0.16, 0.0, 0.19)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.7, lower=0.0, upper=0.95),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.42, 0.42, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=drawer_mat,
        name="drawer_bottom",
    )
    for i, y in enumerate((-0.211, 0.211)):
        drawer.visual(
            Box((0.42, 0.018, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=drawer_mat,
            name=f"drawer_side_{i}",
        )
    drawer.visual(
        Box((0.024, 0.46, 0.090)),
        origin=Origin(xyz=(-0.220, 0.0, 0.005)),
        material=drawer_mat,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.022, 0.42, 0.060)),
        origin=Origin(xyz=(0.205, 0.0, -0.005)),
        material=drawer_mat,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.32, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, -0.262, 0.0455)),
        material=dark_steel,
        name="glide_0",
    )
    drawer.visual(
        Box((0.32, 0.030, 0.025)),
        origin=Origin(xyz=(0.0, -0.2335, 0.0425)),
        material=dark_steel,
        name="glide_web_0",
    )
    drawer.visual(
        Box((0.32, 0.030, 0.014)),
        origin=Origin(xyz=(0.0, 0.262, 0.0455)),
        material=dark_steel,
        name="glide_1",
    )
    drawer.visual(
        Box((0.32, 0.030, 0.025)),
        origin=Origin(xyz=(0.0, 0.2335, 0.0425)),
        material=dark_steel,
        name="glide_web_1",
    )
    for i, y in enumerate((-0.08, 0.08)):
        drawer.visual(
            Box((0.045, 0.015, 0.018)),
            origin=Origin(xyz=(-0.250, y, 0.015)),
            material=dark_steel,
            name=f"handle_stem_{i}",
        )
    drawer.visual(
        Cylinder(radius=0.011, length=0.22),
        origin=Origin(xyz=(-0.278, 0.0, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="drawer_pull",
    )

    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(-0.50, 0.0, -0.155)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    floor_base = object_model.get_part("floor_base")
    height_column = object_model.get_part("height_column")
    board_support = object_model.get_part("board_support")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")

    height_joint = object_model.get_articulation("base_to_column")
    tilt_joint = object_model.get_articulation("support_to_board")
    drawer_joint = object_model.get_articulation("board_to_drawer")

    ctx.allow_overlap(
        board_support,
        board,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        reason="The visible hinge pin is intentionally captured inside the board hinge knuckle proxy.",
    )
    ctx.expect_within(
        board_support,
        board,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_knuckle",
        name="hinge pin nests inside board knuckle",
    )
    ctx.expect_overlap(
        board_support,
        board,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        min_overlap=0.25,
        name="hinge pin spans the board knuckle",
    )

    ctx.check(
        "classroom scale wide base",
        floor_base.get_visual("wide_floor_plate").geometry.size[0] >= 1.0
        and floor_base.get_visual("wide_floor_plate").geometry.size[1] >= 0.70,
    )
    ctx.check(
        "three required user mechanisms",
        height_joint.articulation_type == ArticulationType.PRISMATIC
        and tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and drawer_joint.articulation_type == ArticulationType.PRISMATIC,
    )

    rest_column = ctx.part_world_position(height_column)
    with ctx.pose({height_joint: 0.32}):
        raised_column = ctx.part_world_position(height_column)
    ctx.check(
        "height column slides upward",
        rest_column is not None
        and raised_column is not None
        and raised_column[2] > rest_column[2] + 0.30,
        details=f"rest={rest_column}, raised={raised_column}",
    )

    rest_tray = ctx.part_element_world_aabb(board, elem="tool_tray_lip")
    with ctx.pose({tilt_joint: 0.70}):
        tilted_tray = ctx.part_element_world_aabb(board, elem="tool_tray_lip")
    ctx.check(
        "tilting board raises the tray edge",
        rest_tray is not None
        and tilted_tray is not None
        and tilted_tray[0][2] > rest_tray[0][2] + 0.30,
        details=f"rest={rest_tray}, tilted={tilted_tray}",
    )

    rest_drawer = ctx.part_world_position(drawer)
    ctx.expect_overlap(
        drawer,
        board,
        axes="x",
        elem_a="glide_0",
        elem_b="runner_0",
        min_overlap=0.25,
        name="closed drawer is carried by runner",
    )
    with ctx.pose({drawer_joint: 0.25}):
        extended_drawer = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            board,
            axes="x",
            elem_a="glide_0",
            elem_b="runner_0",
            min_overlap=0.10,
            name="extended drawer remains on runner",
        )
    ctx.check(
        "drawer slides out toward the student side",
        rest_drawer is not None
        and extended_drawer is not None
        and extended_drawer[0] < rest_drawer[0] - 0.20,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )

    return ctx.report()


object_model = build_object_model()
