from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _ironing_board_profile() -> list[tuple[float, float]]:
    """Plan-view outline: blunt rear, broad shoulders, and a rounded tapered nose."""
    rear_x = -0.62
    shoulder_x = 0.18
    nose_x = 0.70
    half_width = 0.19

    lower: list[tuple[float, float]] = [(rear_x, -0.16), (rear_x + 0.03, -half_width)]
    lower.append((shoulder_x, -half_width))
    for i in range(1, 18):
        t = i / 17.0
        # Smoothly pinch the sides into the rounded front point.
        x = shoulder_x + (nose_x - shoulder_x) * t
        y = -half_width * (1.0 - t**1.55) ** 0.55
        lower.append((x, y))

    upper = [(x, -y) for x, y in reversed(lower)]
    return lower + upper


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    fabric = Material("blue_heat_resistant_cover", rgba=(0.08, 0.23, 0.48, 1.0))
    edge = Material("pale_plastic_edge", rgba=(0.78, 0.80, 0.78, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    dark = Material("dark_rubber", rgba=(0.035, 0.035, 0.032, 1.0))

    board = model.part("board")

    board_top = ExtrudeGeometry(_ironing_board_profile(), 0.035, center=True)
    board.visual(
        mesh_from_geometry(board_top, "board_top"),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=fabric,
        name="board_top",
    )
    # A slightly smaller pale under-layer gives the thin top a visible molded rim.
    board_edge = ExtrudeGeometry(_ironing_board_profile(), 0.018, center=True)
    board.visual(
        mesh_from_geometry(board_edge, "underside_edge"),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=edge,
        name="underside_edge",
    )

    # Fixed base bracket under the middle of the board.  The revolute leg pivots
    # between the two side lugs on a horizontal cross-board axis.
    board.visual(
        Box((0.18, 0.34, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=steel,
        name="base_bracket",
    )
    board.visual(
        Box((0.085, 0.030, 0.100)),
        origin=Origin(xyz=(0.0, -0.185, 0.015)),
        material=steel,
        name="hinge_lug_0",
    )
    board.visual(
        Box((0.085, 0.030, 0.100)),
        origin=Origin(xyz=(0.0, 0.185, 0.015)),
        material=steel,
        name="hinge_lug_1",
    )
    board.visual(
        Cylinder(radius=0.008, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )

    # Small fixed iron rest/tray at the blunt rear end.  It is part of the fixed
    # board assembly, not a second moving loop.
    board.visual(
        Box((0.245, 0.275, 0.016)),
        origin=Origin(xyz=(-0.735, 0.0, 0.122)),
        material=steel,
        name="rest_tray",
    )
    board.visual(
        Box((0.035, 0.275, 0.045)),
        origin=Origin(xyz=(-0.850, 0.0, 0.149)),
        material=steel,
        name="rest_back_rim",
    )
    board.visual(
        Box((0.245, 0.025, 0.040)),
        origin=Origin(xyz=(-0.735, -0.137, 0.147)),
        material=steel,
        name="rest_side_rim_0",
    )
    board.visual(
        Box((0.245, 0.025, 0.040)),
        origin=Origin(xyz=(-0.735, 0.137, 0.147)),
        material=steel,
        name="rest_side_rim_1",
    )
    board.visual(
        Box((0.135, 0.035, 0.014)),
        origin=Origin(xyz=(-0.620, -0.075, 0.112)),
        material=steel,
        name="rest_tab_0",
    )
    board.visual(
        Box((0.135, 0.035, 0.014)),
        origin=Origin(xyz=(-0.620, 0.075, 0.112)),
        material=steel,
        name="rest_tab_1",
    )

    leg = model.part("leg_frame")
    leg_tube = wire_from_points(
        [
            (0.0, -0.155, 0.0),
            (0.0, 0.155, 0.0),
            (-0.220, 0.180, -0.720),
            (-0.220, -0.180, -0.720),
        ],
        radius=0.013,
        radial_segments=18,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.045,
    )
    leg.visual(
        mesh_from_geometry(leg_tube, "leg_tube"),
        material=steel,
        name="leg_tube",
    )
    leg.visual(
        Box((0.135, 0.070, 0.028)),
        origin=Origin(xyz=(-0.220, -0.145, -0.742)),
        material=dark,
        name="foot_pad_0",
    )
    leg.visual(
        Box((0.135, 0.070, 0.028)),
        origin=Origin(xyz=(-0.220, 0.145, -0.742)),
        material=dark,
        name="foot_pad_1",
    )

    model.articulation(
        "board_to_leg",
        ArticulationType.REVOLUTE,
        parent=board,
        child=leg,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    leg = object_model.get_part("leg_frame")
    hinge = object_model.get_articulation("board_to_leg")

    ctx.allow_overlap(
        board,
        leg,
        elem_a="pivot_pin",
        elem_b="leg_tube",
        reason="A steel hinge pin is intentionally captured through the folding leg sleeve.",
    )
    ctx.expect_overlap(
        board,
        leg,
        axes="y",
        elem_a="pivot_pin",
        elem_b="leg_tube",
        min_overlap=0.25,
        name="pivot pin spans the leg sleeve",
    )

    ctx.expect_gap(
        board,
        leg,
        axis="z",
        positive_elem="board_top",
        negative_elem="leg_tube",
        min_gap=0.04,
        name="leg frame is below the board top",
    )

    rest_aabb = ctx.part_element_world_aabb(board, elem="rest_tray")
    board_aabb = ctx.part_element_world_aabb(board, elem="board_top")
    ctx.check(
        "fixed rest is mounted at rear end",
        rest_aabb is not None
        and board_aabb is not None
        and rest_aabb[1][0] <= board_aabb[0][0] + 0.02
        and rest_aabb[0][2] > board_aabb[1][2] - 0.01,
        details=f"rest_aabb={rest_aabb}, board_aabb={board_aabb}",
    )

    deployed_aabb = ctx.part_world_aabb(leg)
    with ctx.pose({hinge: 1.32}):
        folded_aabb = ctx.part_world_aabb(leg)

    ctx.check(
        "leg frame folds upward on hinge",
        deployed_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > deployed_aabb[0][2] + 0.55,
        details=f"deployed={deployed_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
