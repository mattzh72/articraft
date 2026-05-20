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
    model = ArticulatedObject(name="professional_drafting_table")

    model.material("steel", rgba=(0.34, 0.37, 0.40, 1.0))
    model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("board_frame", rgba=(0.58, 0.46, 0.32, 1.0))
    model.material("board_surface", rgba=(0.93, 0.92, 0.87, 1.0))
    model.material("wing_surface", rgba=(0.90, 0.89, 0.84, 1.0))
    model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.08, 0.74, 0.04)),
        origin=Origin(xyz=(-0.42, 0.0, 0.02)),
        material="steel",
        name="left_runner",
    )
    base.visual(
        Box((0.08, 0.74, 0.04)),
        origin=Origin(xyz=(0.42, 0.0, 0.02)),
        material="steel",
        name="right_runner",
    )
    base.visual(
        Box((0.76, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, -0.33, 0.02)),
        material="steel",
        name="front_crossmember",
    )
    base.visual(
        Box((0.76, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.33, 0.02)),
        material="steel",
        name="rear_crossmember",
    )
    base.visual(
        Box((0.34, 0.26, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material="dark_steel",
        name="pedestal",
    )
    base.visual(
        Box((0.18, 0.66, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="dark_steel",
        name="center_spine",
    )
    base.visual(
        Box((0.08, 0.08, 0.90)),
        origin=Origin(xyz=(-0.12, -0.02, 0.56)),
        material="steel",
        name="left_column",
    )
    base.visual(
        Box((0.08, 0.08, 0.90)),
        origin=Origin(xyz=(0.12, -0.02, 0.56)),
        material="steel",
        name="right_column",
    )
    base.visual(
        Box((0.32, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, -0.02, 1.0)),
        material="steel",
        name="top_bridge",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.10, 0.08, 0.26)),
        origin=Origin(xyz=(-0.11, 0.06, 0.0)),
        material="dark_steel",
        name="left_bearing",
    )
    carriage.visual(
        Box((0.10, 0.08, 0.26)),
        origin=Origin(xyz=(0.11, 0.06, 0.0)),
        material="dark_steel",
        name="right_bearing",
    )
    carriage.visual(
        Box((0.38, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
        material="steel",
        name="crosshead",
    )
    carriage.visual(
        Box((0.06, 0.08, 0.12)),
        origin=Origin(xyz=(-0.09, 0.13, -0.02)),
        material="steel",
        name="head_block_left",
    )
    carriage.visual(
        Box((0.06, 0.08, 0.12)),
        origin=Origin(xyz=(0.09, 0.13, -0.02)),
        material="steel",
        name="head_block_right",
    )
    carriage.visual(
        Box((0.03, 0.16, 0.20)),
        origin=Origin(xyz=(-0.12, 0.19, 0.02)),
        material="steel",
        name="left_cheek",
    )
    carriage.visual(
        Box((0.03, 0.16, 0.20)),
        origin=Origin(xyz=(0.12, 0.19, 0.02)),
        material="steel",
        name="right_cheek",
    )
    carriage.visual(
        Box((0.24, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.11, -0.07)),
        material="steel",
        name="head_cap",
    )
    carriage.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(
            xyz=(0.141, 0.19, -0.04),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="dark_steel",
        name="knob_boss",
    )

    board = model.part("board")
    board.visual(
        Box((0.92, 0.62, 0.02)),
        origin=Origin(xyz=(0.0, 0.41, 0.012)),
        material="board_surface",
        name="main_panel",
    )
    board.visual(
        Box((0.38, 0.05, 0.04)),
        origin=Origin(xyz=(-0.3275, 0.025, 0.02)),
        material="board_frame",
        name="lower_frame_left",
    )
    board.visual(
        Box((0.38, 0.05, 0.04)),
        origin=Origin(xyz=(0.3275, 0.025, 0.02)),
        material="board_frame",
        name="lower_frame_right",
    )
    board.visual(
        Box((1.06, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.735, 0.02)),
        material="board_frame",
        name="upper_frame",
    )
    board.visual(
        Box((0.05, 0.76, 0.04)),
        origin=Origin(xyz=(-0.505, 0.38, 0.02)),
        material="board_frame",
        name="left_frame",
    )
    board.visual(
        Box((0.07, 0.76, 0.04)),
        origin=Origin(xyz=(0.495, 0.38, 0.02)),
        material="board_frame",
        name="right_frame",
    )
    board.visual(
        Box((0.20, 0.10, 0.14)),
        origin=Origin(xyz=(0.0, 0.05, -0.05)),
        material="board_frame",
        name="tilt_head",
    )
    board.visual(
        Cylinder(radius=0.022, length=0.10),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="dark_steel",
        name="tilt_axle",
    )
    board.visual(
        Box((0.012, 0.22, 0.035)),
        origin=Origin(xyz=(0.536, 0.12, 0.018)),
        material="board_frame",
        name="wing_standoff_lower",
    )
    board.visual(
        Box((0.012, 0.22, 0.035)),
        origin=Origin(xyz=(0.536, 0.64, 0.018)),
        material="board_frame",
        name="wing_standoff_upper",
    )
    board.visual(
        Box((0.012, 0.22, 0.035)),
        origin=Origin(xyz=(0.548, 0.12, 0.018)),
        material="board_frame",
        name="wing_mount_lower",
    )
    board.visual(
        Box((0.012, 0.22, 0.035)),
        origin=Origin(xyz=(0.548, 0.64, 0.018)),
        material="board_frame",
        name="wing_mount_upper",
    )

    wing = model.part("wing")
    wing.visual(
        Box((0.15, 0.62, 0.018)),
        origin=Origin(xyz=(0.087, 0.41, 0.011)),
        material="wing_surface",
        name="wing_panel",
    )
    wing.visual(
        Box((0.17, 0.05, 0.03)),
        origin=Origin(xyz=(0.097, 0.025, 0.015)),
        material="board_frame",
        name="wing_lower_rail",
    )
    wing.visual(
        Box((0.17, 0.05, 0.03)),
        origin=Origin(xyz=(0.097, 0.735, 0.015)),
        material="board_frame",
        name="wing_upper_rail",
    )
    wing.visual(
        Box((0.04, 0.76, 0.03)),
        origin=Origin(xyz=(0.16, 0.38, 0.015)),
        material="board_frame",
        name="wing_outer_rail",
    )
    wing.visual(
        Box((0.024, 0.76, 0.03)),
        origin=Origin(xyz=(0.036, 0.38, 0.015)),
        material="board_frame",
        name="wing_inner_rail",
    )
    wing.visual(
        Box((0.012, 0.22, 0.035)),
        origin=Origin(xyz=(0.018, 0.12, 0.018)),
        material="board_frame",
        name="wing_leaf_lower",
    )
    wing.visual(
        Box((0.012, 0.22, 0.035)),
        origin=Origin(xyz=(0.018, 0.64, 0.018)),
        material="board_frame",
        name="wing_leaf_upper",
    )

    knob = model.part("knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.028,
            body_style="mushroom",
            top_diameter=0.056,
            center=False,
        ),
        "tilt_clamp_knob",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(
            xyz=(0.006, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="dark_steel",
        name="shaft",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(
            xyz=(0.012, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="knob_black",
        name="handwheel",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.18,
            lower=0.0,
            upper=0.28,
        ),
    )
    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(
            xyz=(0.0, 0.18, 0.02),
            rpy=(0.35, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-0.25,
            upper=0.95,
        ),
    )
    model.articulation(
        "board_to_wing",
        ArticulationType.REVOLUTE,
        parent=board,
        child=wing,
        origin=Origin(xyz=(0.542, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.5,
        ),
    )
    model.articulation(
        "carriage_to_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=knob,
        origin=Origin(xyz=(0.147, 0.19, -0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    wing = object_model.get_part("wing")
    knob = object_model.get_part("knob")

    slide = object_model.get_articulation("base_to_carriage")
    tilt = object_model.get_articulation("carriage_to_board")
    wing_hinge = object_model.get_articulation("board_to_wing")
    knob_joint = object_model.get_articulation("carriage_to_knob")

    slide_limits = slide.motion_limits
    tilt_limits = tilt.motion_limits
    wing_limits = wing_hinge.motion_limits

    ctx.check(
        "knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    ctx.expect_contact(
        carriage,
        knob,
        elem_a="knob_boss",
        elem_b="shaft",
        name="knob mounts on the separate head boss",
    )
    ctx.expect_contact(
        board,
        wing,
        elem_a="wing_mount_lower",
        elem_b="wing_leaf_lower",
        name="wing is carried by the board frame",
    )
    ctx.expect_overlap(
        wing,
        board,
        axes="y",
        elem_a="wing_panel",
        elem_b="main_panel",
        min_overlap=0.58,
        name="wing stays aligned with the drawing surface footprint",
    )

    if slide_limits is not None and slide_limits.upper is not None:
        rest_pos = ctx.part_world_position(carriage)
        with ctx.pose({slide: slide_limits.upper}):
            raised_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage slides upward on the support",
            rest_pos is not None
            and raised_pos is not None
            and raised_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        low_top = None
        high_top = None
        with ctx.pose({tilt: tilt_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(board, elem="main_panel")
            if low_aabb is not None:
                low_top = low_aabb[1][2]
        with ctx.pose({tilt: tilt_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(board, elem="main_panel")
            if high_aabb is not None:
                high_top = high_aabb[1][2]
        ctx.check(
            "main board tilts upward about the head hinge",
            low_top is not None and high_top is not None and high_top > low_top + 0.22,
            details=f"low_top={low_top}, high_top={high_top}",
        )

    if wing_limits is not None and wing_limits.upper is not None:
        rest_top = None
        open_top = None
        with ctx.pose({wing_hinge: 0.0}):
            rest_aabb = ctx.part_element_world_aabb(wing, elem="wing_panel")
            if rest_aabb is not None:
                rest_top = rest_aabb[1][2]
        with ctx.pose({wing_hinge: wing_limits.upper}):
            open_aabb = ctx.part_element_world_aabb(wing, elem="wing_panel")
            if open_aabb is not None:
                open_top = open_aabb[1][2]
        ctx.check(
            "reference wing folds upward on its own hinge",
            rest_top is not None and open_top is not None and open_top > rest_top + 0.10,
            details=f"rest_top={rest_top}, open_top={open_top}",
        )

    ctx.expect_origin_gap(
        knob,
        base,
        axis="z",
        min_gap=0.50,
        name="tilt knob sits well above the welded base",
    )

    return ctx.report()


object_model = build_object_model()
