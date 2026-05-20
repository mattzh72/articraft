from __future__ import annotations

import math

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
    model = ArticulatedObject(name="studio_drafting_table")

    steel = Material("blackened_steel", color=(0.05, 0.055, 0.06, 1.0))
    dark_steel = Material("dark_knurled_steel", color=(0.015, 0.016, 0.018, 1.0))
    worn_wood = Material("varnished_maple_board", color=(0.74, 0.56, 0.34, 1.0))
    edge_wood = Material("darker_wood_edges", color=(0.34, 0.21, 0.11, 1.0))
    brass = Material("aged_brass_bushings", color=(0.70, 0.51, 0.20, 1.0))

    base = model.part("base")
    # Low, broad sled runners keep the drafting table stable without separate legs.
    for x in (-0.42, 0.42):
        base.visual(
            Cylinder(radius=0.030, length=1.20),
            origin=Origin(xyz=(x, 0.0, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"sled_runner_{0 if x < 0 else 1}",
        )
    for y in (-0.46, 0.46):
        base.visual(
            Cylinder(radius=0.026, length=0.90),
            origin=Origin(xyz=(0.0, y, 0.075), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"cross_tube_{0 if y < 0 else 1}",
        )
    base.visual(
        Box((0.25, 0.20, 0.080)),
        origin=Origin(xyz=(0.0, -0.25, 0.115)),
        material=steel,
        name="standard_foot",
    )
    base.visual(
        Box((0.080, 0.92, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=steel,
        name="center_sled_keel",
    )
    base.visual(
        Box((0.11, 0.11, 1.16)),
        origin=Origin(xyz=(0.0, -0.25, 0.66)),
        material=steel,
        name="vertical_standard",
    )
    # Welded gussets from foot to standard, kept as stout rectangular braces.
    for x in (-0.095, 0.095):
        base.visual(
            Box((0.035, 0.22, 0.24)),
            origin=Origin(xyz=(x, -0.25, 0.235), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name=f"standard_gusset_{0 if x < 0 else 1}",
        )

    upper_frame = model.part("upper_frame")
    # Sliding sleeve: four plates leave a real clearance window around the standard.
    for x in (-0.0825, 0.0825):
        upper_frame.visual(
            Box((0.025, 0.190, 0.240)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=steel,
            name=f"sleeve_side_{0 if x < 0 else 1}",
        )
    for y in (-0.0825, 0.0825):
        upper_frame.visual(
            Box((0.170, 0.025, 0.240)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=steel,
            name=f"sleeve_plate_{0 if y < 0 else 1}",
        )
    upper_frame.visual(
        Box((0.015, 0.060, 0.115)),
        origin=Origin(xyz=(-0.0625, 0.0, 0.0)),
        material=brass,
        name="guide_shoe_0",
    )
    upper_frame.visual(
        Box((0.015, 0.060, 0.115)),
        origin=Origin(xyz=(0.0625, 0.0, 0.0)),
        material=brass,
        name="guide_shoe_1",
    )
    upper_frame.visual(
        Box((0.11, 0.38, 0.070)),
        origin=Origin(xyz=(0.0, 0.265, 0.075)),
        material=steel,
        name="forward_boom",
    )
    upper_frame.visual(
        Box((1.30, 0.070, 0.080)),
        origin=Origin(xyz=(0.0, 0.470, 0.075)),
        material=steel,
        name="cheek_crossbar",
    )
    upper_frame.visual(
        Box((0.045, 0.86, 0.20)),
        origin=Origin(xyz=(-0.62, 0.50, 0.18)),
        material=steel,
        name="side_cheek_0",
    )
    upper_frame.visual(
        Box((0.045, 0.86, 0.20)),
        origin=Origin(xyz=(0.62, 0.50, 0.18)),
        material=steel,
        name="side_cheek_1",
    )
    for x in (-0.645, 0.645):
        upper_frame.visual(
            Cylinder(radius=0.047, length=0.065),
            origin=Origin(xyz=(x, 0.50, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=f"tilt_bearing_{0 if x < 0 else 1}",
        )
    upper_frame.visual(
        Box((1.30, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, 0.905, 0.095)),
        material=steel,
        name="front_tie_bar",
    )

    board = model.part("board")
    board.visual(
        Box((1.08, 0.80, 0.040)),
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
        material=worn_wood,
        name="drawing_surface",
    )
    board.visual(
        Box((1.13, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, -0.315, 0.0)),
        material=edge_wood,
        name="rear_edge",
    )
    board.visual(
        Box((1.13, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, 0.515, 0.0)),
        material=edge_wood,
        name="front_edge",
    )
    for x in (-0.565, 0.565):
        board.visual(
            Box((0.035, 0.80, 0.055)),
            origin=Origin(xyz=(x, 0.10, 0.0)),
            material=edge_wood,
            name=f"side_edge_{0 if x < 0 else 1}",
        )
    board.visual(
        Cylinder(radius=0.024, length=1.195),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )

    crank = model.part("side_crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.12),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="crank_shaft",
    )
    crank.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="crank_hub",
    )
    crank.visual(
        Box((0.026, 0.036, 0.230)),
        origin=Origin(xyz=(0.125, 0.0, -0.115)),
        material=dark_steel,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.027, length=0.12),
        origin=Origin(xyz=(0.145, 0.0, -0.245), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hand_grip",
    )

    model.articulation(
        "standard_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_frame,
        origin=Origin(xyz=(0.0, -0.25, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.20, lower=0.0, upper=0.34),
    )
    model.articulation(
        "board_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_frame,
        child=board,
        origin=Origin(xyz=(0.0, 0.50, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.8, lower=-0.75, upper=0.95),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=upper_frame,
        child=crank,
        origin=Origin(xyz=(0.6775, 0.50, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_frame = object_model.get_part("upper_frame")
    board = object_model.get_part("board")
    crank = object_model.get_part("side_crank")
    slide = object_model.get_articulation("standard_slide")
    tilt = object_model.get_articulation("board_tilt")
    crank_spin = object_model.get_articulation("crank_spin")

    # The carriage is guided by bronze shoes touching the standard, not by a
    # solid overlapping proxy sleeve.
    ctx.expect_gap(
        upper_frame,
        base,
        axis="x",
        positive_elem="guide_shoe_1",
        negative_elem="vertical_standard",
        max_gap=0.0005,
        max_penetration=0.0,
        name="right guide shoe bears on standard",
    )
    ctx.expect_gap(
        base,
        upper_frame,
        axis="x",
        positive_elem="vertical_standard",
        negative_elem="guide_shoe_0",
        max_gap=0.0005,
        max_penetration=0.0,
        name="left guide shoe bears on standard",
    )

    # The board pivot pin is captured between the side cheeks at rest.
    ctx.expect_gap(
        upper_frame,
        board,
        axis="x",
        positive_elem="side_cheek_1",
        negative_elem="hinge_pin",
        max_gap=0.001,
        max_penetration=0.0,
        name="right cheek captures board pin",
    )
    ctx.expect_gap(
        board,
        upper_frame,
        axis="x",
        positive_elem="hinge_pin",
        negative_elem="side_cheek_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="left cheek captures board pin",
    )

    rest_upper_pos = ctx.part_world_position(upper_frame)
    with ctx.pose({slide: 0.34}):
        raised_upper_pos = ctx.part_world_position(upper_frame)
        ctx.expect_gap(
            upper_frame,
            base,
            axis="x",
            positive_elem="guide_shoe_1",
            negative_elem="vertical_standard",
            max_gap=0.0005,
            max_penetration=0.0,
            name="raised carriage remains guided",
        )
    ctx.check(
        "upper frame slides upward on standard",
        rest_upper_pos is not None
        and raised_upper_pos is not None
        and raised_upper_pos[2] > rest_upper_pos[2] + 0.30,
        details=f"rest={rest_upper_pos}, raised={raised_upper_pos}",
    )

    rest_board_aabb = ctx.part_element_world_aabb(board, elem="drawing_surface")
    with ctx.pose({tilt: 0.95}):
        tilted_board_aabb = ctx.part_element_world_aabb(board, elem="drawing_surface")
    rest_board_z = None if rest_board_aabb is None else (rest_board_aabb[0][2] + rest_board_aabb[1][2]) * 0.5
    tilted_board_z = None if tilted_board_aabb is None else (tilted_board_aabb[0][2] + tilted_board_aabb[1][2]) * 0.5
    ctx.check(
        "board tilts upward about cheek axis",
        rest_board_z is not None and tilted_board_z is not None and tilted_board_z > rest_board_z + 0.06,
        details=f"rest_z={rest_board_z}, tilted_z={tilted_board_z}",
    )

    rest_grip_aabb = ctx.part_element_world_aabb(crank, elem="hand_grip")
    with ctx.pose({crank_spin: math.pi / 2.0}):
        spun_grip_aabb = ctx.part_element_world_aabb(crank, elem="hand_grip")
    rest_grip_y = None if rest_grip_aabb is None else (rest_grip_aabb[0][1] + rest_grip_aabb[1][1]) * 0.5
    spun_grip_y = None if spun_grip_aabb is None else (spun_grip_aabb[0][1] + spun_grip_aabb[1][1]) * 0.5
    ctx.check(
        "side crank rotates continuously about its shaft",
        rest_grip_y is not None and spun_grip_y is not None and spun_grip_y > rest_grip_y + 0.18,
        details=f"rest_y={rest_grip_y}, spun_y={spun_grip_y}",
    )

    return ctx.report()


object_model = build_object_model()
