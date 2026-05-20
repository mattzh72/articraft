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


BOARD_REST_TILT = 0.45


def _carriage_shape():
    body = (
        cq.Workplane("XY")
        .box(0.22, 0.11, 0.18)
        .edges("|Z")
        .fillet(0.010)
    )
    pocket = cq.Workplane("XY").box(0.14, 0.10, 0.16).translate((0.0, -0.030, 0.0))
    cable_relief = cq.Workplane("XY").box(0.10, 0.040, 0.070).translate((0.0, 0.032, -0.040))
    return body.cut(pocket).cut(cable_relief)


def _board_shell_shape():
    shell = cq.Workplane("XY").box(1.10, 0.05, 0.045).translate((0.0, 0.050, 0.565))
    shell = shell.union(cq.Workplane("XY").box(1.02, 0.07, 0.05).translate((0.0, 0.075, -0.245)))
    shell = shell.union(cq.Workplane("XY").box(0.05, 0.06, 0.82).translate((-0.525, 0.050, 0.155)))
    shell = shell.union(cq.Workplane("XY").box(0.05, 0.06, 0.82).translate((0.525, 0.050, 0.155)))
    shell = shell.union(cq.Workplane("XY").box(0.16, 0.05, 0.50).translate((0.0, 0.100, 0.0)))
    return shell


def _tray_shape():
    tray = cq.Workplane("XY").box(0.94, 0.14, 0.024).translate((0.0, 0.110, -0.238))
    tray = tray.union(cq.Workplane("XY").box(0.94, 0.020, 0.038).translate((0.0, 0.182, -0.214)))
    tray = tray.union(cq.Workplane("XY").box(0.016, 0.100, 0.050).translate((-0.462, 0.120, -0.223)))
    tray = tray.union(cq.Workplane("XY").box(0.016, 0.100, 0.050).translate((0.462, 0.120, -0.223)))
    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_workstation")

    steel = model.material("steel", rgba=(0.26, 0.28, 0.30, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    guide_gray = model.material("guide_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    laminate = model.material("laminate", rgba=(0.86, 0.84, 0.78, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.58, 0.61, 0.64, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    glide_black = model.material("glide_black", rgba=(0.08, 0.08, 0.09, 1.0))

    floor_frame = model.part("floor_frame")
    floor_frame.visual(
        Box((0.92, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.30, 0.03)),
        material=steel,
        name="front_bar",
    )
    floor_frame.visual(
        Box((0.92, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.30, 0.03)),
        material=steel,
        name="rear_bar",
    )
    for index, x_pos in enumerate((-0.43, 0.43)):
        floor_frame.visual(
            Box((0.06, 0.60, 0.06)),
            origin=Origin(xyz=(x_pos, 0.0, 0.03)),
            material=steel,
            name=f"side_bar_{index}",
        )
    floor_frame.visual(
        Box((0.80, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.14, 0.03)),
        material=steel,
        name="rear_bridge",
    )
    floor_frame.visual(
        Box((0.22, 0.12, 0.08)),
        origin=Origin(xyz=(0.0, -0.14, 0.07)),
        material=dark_steel,
        name="rail_socket",
    )
    for index, (x_pos, y_pos) in enumerate(((-0.43, -0.30), (0.43, -0.30), (-0.43, 0.30), (0.43, 0.30))):
        floor_frame.visual(
            Cylinder(radius=0.022, length=0.008),
            origin=Origin(xyz=(x_pos, y_pos, 0.004)),
            material=glide_black,
            name=f"glide_{index}",
        )

    support_rail = model.part("support_rail")
    support_rail.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, -0.01, 0.05)),
        material=dark_steel,
        name="lower_housing",
    )
    support_rail.visual(
        Box((0.12, 0.03, 1.08)),
        origin=Origin(xyz=(0.0, 0.015, 0.64)),
        material=guide_gray,
        name="guide_plate",
    )
    support_rail.visual(
        Box((0.07, 0.06, 1.08)),
        origin=Origin(xyz=(0.0, -0.03, 0.64)),
        material=steel,
        name="rear_spine",
    )
    support_rail.visual(
        Box((0.015, 0.02, 1.04)),
        origin=Origin(xyz=(-0.0525, 0.04, 0.64)),
        material=guide_gray,
        name="guide_strip_0",
    )
    support_rail.visual(
        Box((0.015, 0.02, 1.04)),
        origin=Origin(xyz=(0.0525, 0.04, 0.64)),
        material=guide_gray,
        name="guide_strip_1",
    )
    support_rail.visual(
        Box((0.16, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, -0.015, 1.21)),
        material=dark_steel,
        name="top_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.025, 0.18)),
        origin=Origin(xyz=(0.0, 0.0425, 0.0)),
        material=dark_steel,
        name="front_plate",
    )
    carriage.visual(
        Box((0.04, 0.055, 0.18)),
        origin=Origin(xyz=(-0.08, 0.0025, 0.0)),
        material=dark_steel,
        name="shoe_0",
    )
    carriage.visual(
        Box((0.04, 0.055, 0.18)),
        origin=Origin(xyz=(0.08, 0.0025, 0.0)),
        material=dark_steel,
        name="shoe_1",
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Box((0.14, 0.04, 0.12)),
        origin=Origin(xyz=(0.0, 0.02, 0.0)),
        material=dark_steel,
        name="mount_block",
    )
    tilt_head.visual(
        Box((0.08, 0.16, 0.07)),
        origin=Origin(xyz=(0.0, 0.12, 0.0)),
        material=steel,
        name="arm_block",
    )
    tilt_head.visual(
        Cylinder(radius=0.028, length=0.34),
        origin=Origin(xyz=(0.0, 0.20, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_gray,
        name="pivot_tube",
    )
    for index, x_pos in enumerate((-0.11, 0.11)):
        tilt_head.visual(
            Box((0.024, 0.05, 0.12)),
            origin=Origin(xyz=(x_pos, 0.195, 0.060)),
            material=dark_steel,
            name=f"cheek_{index}",
        )
    tilt_head.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(0.186, 0.20, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_gray,
        name="knob_boss",
    )

    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_board_shell_shape(), "drafting_board_shell"),
        origin=Origin(rpy=(BOARD_REST_TILT, 0.0, 0.0)),
        material=steel,
        name="shell",
    )
    board.visual(
        Box((1.02, 0.018, 0.78)),
        origin=Origin(xyz=(0.0, 0.070, 0.155), rpy=(BOARD_REST_TILT, 0.0, 0.0)),
        material=laminate,
        name="panel",
    )
    board.visual(
        mesh_from_cadquery(_tray_shape(), "drafting_board_tray"),
        origin=Origin(rpy=(BOARD_REST_TILT, 0.0, 0.0)),
        material=tray_gray,
        name="storage_ledge",
    )
    board.visual(
        Box((0.196, 0.10, 0.12)),
        origin=Origin(xyz=(0.0, 0.05, 0.03), rpy=(BOARD_REST_TILT, 0.0, 0.0)),
        material=steel,
        name="hinge_saddle",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guide_gray,
        name="stem",
    )
    knob.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="handwheel",
    )
    knob.visual(
        Cylinder(radius=0.006, length=0.048),
        origin=Origin(xyz=(0.039, 0.0, 0.0)),
        material=guide_gray,
        name="grip_bar",
    )

    model.articulation(
        "frame_to_rail",
        ArticulationType.FIXED,
        parent=floor_frame,
        child=support_rail,
        origin=Origin(xyz=(0.0, -0.14, 0.11)),
    )
    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=support_rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.075, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.34),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.FIXED,
        parent=carriage,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
    )
    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=tilt_head,
        child=board,
        origin=Origin(xyz=(0.0, 0.20, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.20, upper=0.35),
    )
    model.articulation(
        "knob_turn",
        ArticulationType.CONTINUOUS,
        parent=tilt_head,
        child=knob,
        origin=Origin(xyz=(0.204, 0.20, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_rail = object_model.get_part("support_rail")
    carriage = object_model.get_part("carriage")
    tilt_head = object_model.get_part("tilt_head")
    board = object_model.get_part("board")
    knob = object_model.get_part("knob")

    rail_slide = object_model.get_articulation("rail_slide")
    board_tilt = object_model.get_articulation("board_tilt")
    knob_turn = object_model.get_articulation("knob_turn")

    ctx.expect_overlap(
        carriage,
        support_rail,
        axes="x",
        min_overlap=0.11,
        name="carriage stays centered on the rail",
    )
    ctx.expect_origin_gap(
        carriage,
        support_rail,
        axis="y",
        min_gap=0.03,
        max_gap=0.08,
        name="carriage sits in front of the support rail",
    )
    ctx.expect_origin_gap(
        knob,
        tilt_head,
        axis="x",
        min_gap=0.18,
        max_gap=0.25,
        name="knob sits outboard of the tilt head",
    )
    ctx.allow_overlap(
        board,
        tilt_head,
        elem_a="hinge_saddle",
        elem_b="pivot_tube",
        reason="The board hinge is simplified as a solid saddle clamping around the tilt-head pivot tube.",
    )

    slide_limits = rail_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        board_rest_pos = ctx.part_world_position(board)
        with ctx.pose({rail_slide: slide_limits.upper}):
            ctx.expect_overlap(
                carriage,
                support_rail,
                axes="x",
                min_overlap=0.11,
                name="raised carriage remains aligned on the rail",
            )
            board_raised_pos = ctx.part_world_position(board)

        ctx.check(
            "rail slide lifts the board assembly",
            board_rest_pos is not None
            and board_raised_pos is not None
            and board_raised_pos[2] > board_rest_pos[2] + 0.30,
            details=f"rest={board_rest_pos}, raised={board_raised_pos}",
        )

    tilt_limits = board_tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({board_tilt: tilt_limits.lower}):
            lower_panel_aabb = ctx.part_element_world_aabb(board, elem="panel")
        with ctx.pose({board_tilt: tilt_limits.upper}):
            upper_panel_aabb = ctx.part_element_world_aabb(board, elem="panel")

        lower_center_y = None
        upper_center_y = None
        if lower_panel_aabb is not None:
            lower_center_y = (lower_panel_aabb[0][1] + lower_panel_aabb[1][1]) / 2.0
        if upper_panel_aabb is not None:
            upper_center_y = (upper_panel_aabb[0][1] + upper_panel_aabb[1][1]) / 2.0

        ctx.check(
            "board tilt reclines the drawing surface backward",
            lower_center_y is not None
            and upper_center_y is not None
            and upper_center_y < lower_center_y - 0.06,
            details=f"lower_center_y={lower_center_y}, upper_center_y={upper_center_y}",
        )

    grip_rest = ctx.part_element_world_aabb(knob, elem="grip_bar")
    with ctx.pose({knob_turn: math.pi / 2.0}):
        grip_quarter = ctx.part_element_world_aabb(knob, elem="grip_bar")

    rest_y_span = None
    rest_z_span = None
    quarter_y_span = None
    quarter_z_span = None
    if grip_rest is not None:
        rest_y_span = grip_rest[1][1] - grip_rest[0][1]
        rest_z_span = grip_rest[1][2] - grip_rest[0][2]
    if grip_quarter is not None:
        quarter_y_span = grip_quarter[1][1] - grip_quarter[0][1]
        quarter_z_span = grip_quarter[1][2] - grip_quarter[0][2]

    ctx.check(
        "knob grip rotates about the threaded axis",
        rest_y_span is not None
        and rest_z_span is not None
        and quarter_y_span is not None
        and quarter_z_span is not None
        and quarter_y_span > rest_y_span + 0.015
        and rest_z_span > quarter_z_span + 0.015,
        details=(
            f"rest_y_span={rest_y_span}, rest_z_span={rest_z_span}, "
            f"quarter_y_span={quarter_y_span}, quarter_z_span={quarter_z_span}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
