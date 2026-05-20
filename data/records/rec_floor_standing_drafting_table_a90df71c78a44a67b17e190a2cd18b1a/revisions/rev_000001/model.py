from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_room_drafting_table")

    steel = model.material("steel_grey", rgba=(0.30, 0.32, 0.35, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    wood = model.material("maple_board", rgba=(0.73, 0.63, 0.48, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.57, 0.60, 0.63, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.82, 0.14, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_runner",
    )
    base.visual(
        Box((0.22, 0.46, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_crossbar",
    )
    base.visual(
        Box((0.22, 0.22, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=steel,
        name="pedestal_cap",
    )

    sleeve_outer = 0.15
    sleeve_wall = 0.015
    sleeve_inner = sleeve_outer - 2.0 * sleeve_wall
    sleeve_height = 0.40
    sleeve_center_z = 0.10 + sleeve_height / 2.0
    sleeve_offset = sleeve_outer / 2.0 - sleeve_wall / 2.0
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, sleeve_offset, sleeve_center_z)),
        material=steel,
        name="sleeve_front",
    )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, -sleeve_offset, sleeve_center_z)),
        material=steel,
        name="sleeve_rear",
    )
    base.visual(
        Box((sleeve_wall, sleeve_inner, sleeve_height)),
        origin=Origin(xyz=(sleeve_offset, 0.0, sleeve_center_z)),
        material=steel,
        name="sleeve_side_0",
    )
    base.visual(
        Box((sleeve_wall, sleeve_inner, sleeve_height)),
        origin=Origin(xyz=(-sleeve_offset, 0.0, sleeve_center_z)),
        material=steel,
        name="sleeve_side_1",
    )

    column_post = model.part("column_post")
    column_post.visual(
        Box((0.12, 0.12, 0.70)),
        origin=Origin(),
        material=steel,
        name="column_post",
    )
    column_post.visual(
        Box((0.12, 0.12, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.336)),
        material=dark_steel,
        name="column_cap",
    )

    head_support = model.part("head_support")
    head_support.visual(
        Box((0.10, 0.055, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel,
        name="support_neck",
    )
    head_support.visual(
        Box((0.056, 0.072, 0.084)),
        origin=Origin(xyz=(0.062, 0.0, 0.122)),
        material=dark_steel,
        name="support_cheek_0",
    )
    head_support.visual(
        Box((0.056, 0.072, 0.084)),
        origin=Origin(xyz=(-0.062, 0.0, 0.122)),
        material=dark_steel,
        name="support_cheek_1",
    )
    head_support.visual(
        Box((0.18, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, 0.045, 0.110)),
        material=dark_steel,
        name="support_bridge",
    )
    head_support.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(
            xyz=(0.096, 0.0, 0.136),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="knob_boss",
    )

    board = model.part("board")
    board.visual(
        Box((0.94, 0.64, 0.024)),
        origin=Origin(xyz=(0.0, -0.375, 0.0)),
        material=wood,
        name="board_panel",
    )
    board.visual(
        Box((0.058, 0.09, 0.05)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=dark_steel,
        name="hinge_block",
    )
    board.visual(
        Cylinder(radius=0.009, length=0.068),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="trunnion_barrel",
    )
    board.visual(
        Box((0.84, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, -0.670, 0.018)),
        material=tray_metal,
        name="tray_floor",
    )
    board.visual(
        Box((0.84, 0.008, 0.036)),
        origin=Origin(xyz=(0.0, -0.691, 0.030)),
        material=tray_metal,
        name="front_lip",
    )
    board.visual(
        Box((0.008, 0.050, 0.030)),
        origin=Origin(xyz=(0.416, -0.670, 0.027)),
        material=tray_metal,
        name="tray_side_0",
    )
    board.visual(
        Box((0.008, 0.050, 0.030)),
        origin=Origin(xyz=(-0.416, -0.670, 0.027)),
        material=tray_metal,
        name="tray_side_1",
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(
            xyz=(0.009, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="knob_shaft",
    )
    clamp_knob.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(
            xyz=(0.020, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="knob_washer",
    )
    clamp_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.022,
                body_style="mushroom",
                top_diameter=0.042,
                base_diameter=0.048,
                center=False,
            ),
            "drafting_table_clamp_knob",
        ),
        origin=Origin(
            xyz=(0.022, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=knob_black,
        name="knob_body",
    )

    model.articulation(
        "base_to_column_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column_post,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "column_post_to_head_support",
        ArticulationType.FIXED,
        parent=column_post,
        child=head_support,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
    )
    model.articulation(
        "head_support_to_board",
        ArticulationType.REVOLUTE,
        parent=head_support,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "head_support_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=head_support,
        child=clamp_knob,
        origin=Origin(xyz=(0.102, 0.0, 0.136)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column_post = object_model.get_part("column_post")
    head_support = object_model.get_part("head_support")
    board = object_model.get_part("board")
    clamp_knob = object_model.get_part("clamp_knob")

    column_slide = object_model.get_articulation("base_to_column_post")
    board_tilt = object_model.get_articulation("head_support_to_board")
    knob_spin = object_model.get_articulation("head_support_to_clamp_knob")

    ctx.expect_origin_gap(
        head_support,
        column_post,
        axis="z",
        min_gap=0.34,
        max_gap=0.36,
        name="support head stays visibly above the sliding column",
    )
    ctx.expect_gap(
        clamp_knob,
        head_support,
        axis="x",
        positive_elem="knob_shaft",
        negative_elem="knob_boss",
        max_gap=0.001,
        max_penetration=1e-6,
        name="clamp knob remains a separate mounted part",
    )

    board_panel_aabb = ctx.part_element_world_aabb(board, elem="board_panel")
    if board_panel_aabb is not None:
        mins, maxs = board_panel_aabb
        size = (
            float(maxs[0] - mins[0]),
            float(maxs[1] - mins[1]),
            float(maxs[2] - mins[2]),
        )
        ctx.check(
            "board matches classroom drafting table scale",
            0.90 <= size[0] <= 1.00 and 0.60 <= size[1] <= 0.68,
            details=f"board_size={size!r}",
        )
    else:
        ctx.fail("board matches classroom drafting table scale", "Missing board_panel AABB.")

    slide_limits = column_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        rest_column_pos = ctx.part_world_position(column_post)
        rest_board_pos = ctx.part_world_position(board)
        with ctx.pose({column_slide: slide_limits.upper}):
            raised_column_pos = ctx.part_world_position(column_post)
            raised_board_pos = ctx.part_world_position(board)
        ctx.check(
            "height adjustment raises the center column",
            rest_column_pos is not None
            and raised_column_pos is not None
            and raised_column_pos[2] > rest_column_pos[2] + 0.15,
            details=f"rest={rest_column_pos!r}, raised={raised_column_pos!r}",
        )
        ctx.check(
            "height adjustment lifts the drafting board",
            rest_board_pos is not None
            and raised_board_pos is not None
            and raised_board_pos[2] > rest_board_pos[2] + 0.15,
            details=f"rest={rest_board_pos!r}, raised={raised_board_pos!r}",
        )

    tilt_limits = board_tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        low_front_lip = ctx.part_element_world_aabb(board, elem="front_lip")
        with ctx.pose({board_tilt: tilt_limits.upper}):
            high_front_lip = ctx.part_element_world_aabb(board, elem="front_lip")
        ctx.check(
            "board front edge rises when tilted",
            low_front_lip is not None
            and high_front_lip is not None
            and high_front_lip[1][2] > low_front_lip[1][2] + 0.30,
            details=f"low={low_front_lip!r}, high={high_front_lip!r}",
        )

    with ctx.pose({knob_spin: math.pi / 2.0}):
        ctx.expect_origin_gap(
            clamp_knob,
            head_support,
            axis="x",
            min_gap=0.10,
            max_gap=0.11,
            name="clamp knob keeps its side-mounted position while rotating",
        )

    ctx.expect_origin_distance(
        column_post,
        base,
        axes="xy",
        max_dist=0.001,
        name="column stays centered over the wide floor base",
    )

    return ctx.report()


object_model = build_object_model()
