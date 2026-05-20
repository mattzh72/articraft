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


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[idx] + high[idx]) * 0.5 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_drafting_table")

    powder_coat = model.material("powder_coat", rgba=(0.19, 0.20, 0.22, 1.0))
    rail_gray = model.material("rail_gray", rgba=(0.57, 0.60, 0.64, 1.0))
    board_frame = model.material("board_frame", rgba=(0.66, 0.54, 0.41, 1.0))
    board_surface = model.material("board_surface", rgba=(0.90, 0.86, 0.78, 1.0))
    hardware = model.material("hardware", rgba=(0.16, 0.16, 0.17, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.09, 0.68, 0.028)),
        origin=Origin(xyz=(-0.30, 0.0, 0.014)),
        material=powder_coat,
        name="left_runner",
    )
    frame.visual(
        Box((0.09, 0.68, 0.028)),
        origin=Origin(xyz=(0.30, 0.0, 0.014)),
        material=powder_coat,
        name="right_runner",
    )
    frame.visual(
        Box((0.66, 0.10, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=powder_coat,
        name="cross_tie",
    )
    frame.visual(
        Box((0.18, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=powder_coat,
        name="pedestal",
    )
    frame.visual(
        Box((0.18, 0.10, 1.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.718)),
        material=rail_gray,
        name="rail_body",
    )
    frame.visual(
        Box((0.10, 0.03, 1.10)),
        origin=Origin(xyz=(0.0, 0.065, 0.720)),
        material=rail_gray,
        name="rail_track",
    )
    frame.visual(
        Box((0.22, 0.14, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.330)),
        material=powder_coat,
        name="top_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.20, 0.03, 0.30)),
        origin=Origin(xyz=(0.0, 0.015, 0.0)),
        material=powder_coat,
        name="carriage_back",
    )
    carriage.visual(
        Box((0.035, 0.045, 0.24)),
        origin=Origin(xyz=(-0.0825, 0.0375, 0.0)),
        material=powder_coat,
        name="left_guide",
    )
    carriage.visual(
        Box((0.035, 0.045, 0.24)),
        origin=Origin(xyz=(0.0825, 0.0375, 0.0)),
        material=powder_coat,
        name="right_guide",
    )
    carriage.visual(
        Box((0.12, 0.05, 0.12)),
        origin=Origin(xyz=(0.0, 0.045, 0.140)),
        material=powder_coat,
        name="head_mast",
    )
    carriage.visual(
        Box((0.035, 0.14, 0.18)),
        origin=Origin(xyz=(-0.110, 0.100, 0.240)),
        material=powder_coat,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.035, 0.14, 0.18)),
        origin=Origin(xyz=(0.110, 0.100, 0.240)),
        material=powder_coat,
        name="right_cheek",
    )

    board = model.part("board")
    board.visual(
        Box((0.176, 0.05, 0.10)),
        origin=Origin(xyz=(0.0, -0.010, -0.050)),
        material=hardware,
        name="pivot_block",
    )
    board.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.0875, -0.010, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="left_trunnion",
    )
    board.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0875, -0.010, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="right_trunnion",
    )
    board.visual(
        Box((0.176, 0.12, 0.20)),
        origin=Origin(xyz=(0.0, 0.030, -0.140)),
        material=hardware,
        name="rear_brace",
    )
    board.visual(
        Box((0.98, 0.032, 0.76)),
        origin=Origin(xyz=(0.0, 0.078, -0.380)),
        material=board_frame,
        name="panel",
    )
    board.visual(
        Box((0.90, 0.004, 0.68)),
        origin=Origin(xyz=(0.0, 0.096, -0.395)),
        material=board_surface,
        name="surface_sheet",
    )
    board.visual(
        Box((1.00, 0.036, 0.050)),
        origin=Origin(xyz=(0.0, 0.078, -0.025)),
        material=board_frame,
        name="top_rail",
    )
    board.visual(
        Box((0.98, 0.036, 0.050)),
        origin=Origin(xyz=(0.0, 0.078, -0.735)),
        material=board_frame,
        name="bottom_rail",
    )
    board.visual(
        Box((0.040, 0.036, 0.700)),
        origin=Origin(xyz=(-0.470, 0.078, -0.380)),
        material=board_frame,
        name="left_rail",
    )
    board.visual(
        Box((0.040, 0.036, 0.700)),
        origin=Origin(xyz=(0.470, 0.078, -0.380)),
        material=board_frame,
        name="right_rail",
    )
    board.visual(
        Box((0.82, 0.070, 0.030)),
        origin=Origin(xyz=(0.0, 0.103, -0.704)),
        material=board_frame,
        name="ledge",
    )
    board.visual(
        Box((0.80, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.138, -0.692)),
        material=board_frame,
        name="ledge_lip",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="hub",
    )
    knob.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="wheel",
    )
    knob.visual(
        Box((0.018, 0.012, 0.024)),
        origin=Origin(xyz=(0.045, 0.0, 0.028)),
        material=hardware,
        name="handle",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.080, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.20, lower=0.0, upper=0.32),
    )
    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(xyz=(0.0, 0.115, 0.250)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )
    model.articulation(
        "carriage_to_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=knob,
        origin=Origin(xyz=(0.1275, 0.100, 0.240)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    knob = object_model.get_part("knob")

    slide = object_model.get_articulation("frame_to_carriage")
    tilt = object_model.get_articulation("carriage_to_board")
    spin = object_model.get_articulation("carriage_to_knob")

    ctx.expect_gap(
        carriage,
        frame,
        axis="y",
        positive_elem="carriage_back",
        negative_elem="rail_track",
        max_gap=0.0005,
        max_penetration=0.0,
        name="carriage back rides flush on the rail track",
    )
    ctx.expect_gap(
        board,
        carriage,
        axis="y",
        positive_elem="panel",
        negative_elem="right_cheek",
        min_gap=0.005,
        name="drawing board sits forward of the tilt cheeks",
    )
    ctx.expect_gap(
        knob,
        carriage,
        axis="x",
        positive_elem="shaft",
        negative_elem="right_cheek",
        max_gap=0.0005,
        max_penetration=0.0,
        name="clamp knob mounts on the outside face of the right cheek",
    )

    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    if slide_upper is not None:
        rest_pos = ctx.part_world_position(carriage)
        with ctx.pose({slide: slide_upper}):
            high_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage slides upward on the rail",
            rest_pos is not None and high_pos is not None and high_pos[2] > rest_pos[2] + 0.25,
            details=f"rest={rest_pos}, high={high_pos}",
        )

    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    if tilt_upper is not None:
        rest_ledge_center = _aabb_center(ctx.part_element_world_aabb(board, elem="ledge"))
        with ctx.pose({tilt: tilt_upper}):
            ctx.expect_gap(
                board,
                frame,
                axis="z",
                positive_elem="ledge",
                negative_elem="cross_tie",
                min_gap=0.20,
                name="tilted board ledge stays well above the H-base",
            )
            tilted_ledge_center = _aabb_center(ctx.part_element_world_aabb(board, elem="ledge"))
        ctx.check(
            "board tilts forward about the head axis",
            rest_ledge_center is not None
            and tilted_ledge_center is not None
            and tilted_ledge_center[1] > rest_ledge_center[1] + 0.20,
            details=f"rest={rest_ledge_center}, tilted={tilted_ledge_center}",
        )

    rest_handle_center = _aabb_center(ctx.part_element_world_aabb(knob, elem="handle"))
    with ctx.pose({spin: math.pi / 2.0}):
        turned_handle_center = _aabb_center(ctx.part_element_world_aabb(knob, elem="handle"))
    ctx.check(
        "tilt clamp knob rotates about its threaded axis",
        rest_handle_center is not None
        and turned_handle_center is not None
        and abs(turned_handle_center[2] - rest_handle_center[2]) > 0.015,
        details=f"rest={rest_handle_center}, turned={turned_handle_center}",
    )

    return ctx.report()


object_model = build_object_model()
