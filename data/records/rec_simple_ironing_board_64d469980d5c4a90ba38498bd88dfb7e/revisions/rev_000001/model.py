from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _ironing_board_outline(samples: int = 36) -> list[tuple[float, float]]:
    """Planform of a real ironing board: broad square tail and tapered nose."""
    x_tail = -0.62
    x_nose = 0.72
    taper_start = 0.16
    upper: list[tuple[float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        x = x_tail + (x_nose - x_tail) * t
        if x <= taper_start:
            # Slight waist so the wide end does not read as a plain rectangle.
            u = (x - x_tail) / (taper_start - x_tail)
            half_width = 0.185 - 0.012 * u
        else:
            u = min(1.0, (x - taper_start) / (x_nose - taper_start))
            half_width = 0.173 * (1.0 - u**1.85) ** 0.55
        upper.append((x, half_width))
    lower = [(x, -y) for x, y in reversed(upper)]
    return upper + lower


def _build_board_top():
    outline = _ironing_board_outline()
    profiles = []
    for z, sx, sy in (
        (0.035, 0.982, 0.925),
        (0.057, 1.000, 1.000),
        (0.079, 0.992, 0.970),
    ):
        profiles.append([(x * sx, y * sy, z) for x, y in outline])
    return LoftGeometry(profiles, cap=True, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    fabric = model.material("blue_cotton_cover", rgba=(0.38, 0.57, 0.70, 1.0))
    dark_edge = model.material("dark_edge_binding", rgba=(0.10, 0.12, 0.14, 1.0))
    steel = model.material("powder_coated_steel", rgba=(0.72, 0.74, 0.75, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.025, 0.025, 0.025, 1.0))

    board = model.part("board")
    board.visual(
        mesh_from_geometry(_build_board_top(), "ironing_board_top"),
        material=fabric,
        name="board_top",
    )
    board.visual(
        Box((0.88, 0.026, 0.026)),
        origin=Origin(xyz=(0.02, 0.0, 0.030)),
        material=dark_edge,
        name="center_spine",
    )
    board.visual(
        Box((0.58, 0.460, 0.022)),
        origin=Origin(xyz=(0.01, 0.0, 0.024)),
        material=steel,
        name="base_bracket",
    )
    board.visual(
        Box((0.080, 0.036, 0.080)),
        origin=Origin(xyz=(0.0, -0.205, 0.000)),
        material=steel,
        name="pivot_hanger_0",
    )
    board.visual(
        Box((0.080, 0.036, 0.080)),
        origin=Origin(xyz=(0.0, 0.205, 0.000)),
        material=steel,
        name="pivot_hanger_1",
    )

    # A small non-folding U rest at the broad end gives the trestle a second
    # support while keeping the only moving mechanism as the main leg frame.
    fixed_rest_tube = wire_from_points(
        [
            (-0.50, -0.140, 0.030),
            (-0.58, -0.165, -0.720),
            (-0.58, 0.165, -0.720),
            (-0.50, 0.140, 0.030),
        ],
        radius=0.012,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.030,
        corner_segments=10,
    )
    board.visual(
        mesh_from_geometry(fixed_rest_tube, "fixed_rear_rest"),
        material=steel,
        name="fixed_rest",
    )
    for y, name in ((-0.140, "rest_mount_0"), (0.140, "rest_mount_1")):
        board.visual(
            Box((0.085, 0.045, 0.026)),
            origin=Origin(xyz=(-0.50, y, 0.030)),
            material=steel,
            name=name,
        )
    board.visual(
        Box((0.110, 0.030, 0.025)),
        origin=Origin(xyz=(-0.58, -0.185, -0.720)),
        material=rubber,
        name="fixed_foot_0",
    )
    board.visual(
        Box((0.110, 0.030, 0.025)),
        origin=Origin(xyz=(-0.58, 0.185, -0.720)),
        material=rubber,
        name="fixed_foot_1",
    )

    leg_frame = model.part("leg_frame")
    leg_frame.visual(
        Cylinder(radius=0.012, length=0.450),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    main_leg_tube = wire_from_points(
        [
            (0.0, -0.152, -0.026),
            (0.33, -0.185, -0.720),
            (0.33, 0.185, -0.720),
            (0.0, 0.152, -0.026),
        ],
        radius=0.015,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.040,
        corner_segments=12,
    )
    leg_frame.visual(
        mesh_from_geometry(main_leg_tube, "main_u_leg_frame"),
        material=steel,
        name="u_leg_tube",
    )
    for y, name in ((-0.152, "pin_collar_0"), (0.152, "pin_collar_1")):
        leg_frame.visual(
            Box((0.040, 0.038, 0.032)),
            origin=Origin(xyz=(0.0, y, -0.012)),
            material=steel,
            name=name,
        )
    leg_frame.visual(
        Cylinder(radius=0.014, length=0.410),
        origin=Origin(xyz=(0.33, 0.0, -0.720), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="floor_crossbar",
    )
    leg_frame.visual(
        Box((0.120, 0.035, 0.026)),
        origin=Origin(xyz=(0.33, -0.205, -0.720)),
        material=rubber,
        name="leg_foot_0",
    )
    leg_frame.visual(
        Box((0.120, 0.035, 0.026)),
        origin=Origin(xyz=(0.33, 0.205, -0.720)),
        material=rubber,
        name="leg_foot_1",
    )

    model.articulation(
        "board_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=board,
        child=leg_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-1.05, upper=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    leg_frame = object_model.get_part("leg_frame")
    leg_joint = object_model.get_articulation("board_to_leg_frame")

    ctx.allow_overlap(
        board,
        leg_frame,
        elem_a="pivot_hanger_0",
        elem_b="hinge_pin",
        reason="The hinge pin is intentionally captured in the left bracket lug.",
    )
    ctx.allow_overlap(
        board,
        leg_frame,
        elem_a="pivot_hanger_1",
        elem_b="hinge_pin",
        reason="The hinge pin is intentionally captured in the right bracket lug.",
    )

    top_aabb = ctx.part_element_world_aabb(board, elem="board_top")
    ctx.check("board_top_aabb_present", top_aabb is not None, "Expected shaped board top geometry.")
    if top_aabb is not None:
        mins, maxs = top_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("board_top_real_scale", 1.25 <= size[0] <= 1.38 and 0.32 <= size[1] <= 0.40, f"size={size!r}")

    ctx.expect_overlap(
        board,
        leg_frame,
        axes="y",
        elem_a="pivot_hanger_0",
        elem_b="hinge_pin",
        min_overlap=0.018,
        name="left pivot captures pin",
    )
    ctx.expect_overlap(
        board,
        leg_frame,
        axes="y",
        elem_a="pivot_hanger_1",
        elem_b="hinge_pin",
        min_overlap=0.018,
        name="right pivot captures pin",
    )
    ctx.expect_overlap(
        board,
        leg_frame,
        axes="y",
        elem_a="base_bracket",
        elem_b="u_leg_tube",
        min_overlap=0.20,
        name="leg frame spans under board width",
    )

    deployed_aabb = ctx.part_world_aabb(leg_frame)
    with ctx.pose({leg_joint: -1.05}):
        folded_aabb = ctx.part_world_aabb(leg_frame)
    ctx.check(
        "leg frame folds upward",
        deployed_aabb is not None
        and folded_aabb is not None
        and float(folded_aabb[0][2]) > float(deployed_aabb[0][2]) + 0.30,
        details=f"deployed={deployed_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
