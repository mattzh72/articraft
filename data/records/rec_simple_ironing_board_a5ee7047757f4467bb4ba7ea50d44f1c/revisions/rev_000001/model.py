from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


BOARD_Z = 0.775
BOARD_THICKNESS = 0.032
HINGE_X = 0.040
HINGE_Z = 0.700


def _board_outline(scale_y: float = 1.0) -> list[tuple[float, float]]:
    """Ironing-board planform: broad square tail and rounded tapered nose."""
    upper = [
        (-0.640, 0.150 * scale_y),
        (-0.615, 0.178 * scale_y),
        (-0.540, 0.190 * scale_y),
        (-0.220, 0.190 * scale_y),
        (0.140, 0.178 * scale_y),
        (0.360, 0.132 * scale_y),
        (0.535, 0.062 * scale_y),
        (0.650, 0.000),
    ]
    return upper + [(x, -y) for x, y in reversed(upper[:-1])]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    fabric = model.material("blue_cotton_cover", rgba=(0.10, 0.32, 0.55, 1.0))
    fabric_stripe = model.material("subtle_cover_stripe", rgba=(0.78, 0.87, 0.92, 1.0))
    board_edge = model.material("cream_board_edge", rgba=(0.86, 0.84, 0.76, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.65, 1.0))
    dark_joint = model.material("dark_pivot_bushing", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.035, 0.035, 0.035, 1.0))

    board = model.part("board")
    board.visual(
        mesh_from_geometry(
            ExtrudeGeometry(_board_outline(), BOARD_THICKNESS, center=True),
            "ironing_board_tapered_core",
        ),
        origin=Origin(xyz=(0.0, 0.0, BOARD_Z)),
        material=board_edge,
        name="board_core",
    )
    board.visual(
        mesh_from_geometry(
            ExtrudeGeometry(_board_outline(scale_y=0.985), 0.006, center=True),
            "ironing_board_fabric_cover",
        ),
        origin=Origin(xyz=(0.0, 0.0, BOARD_Z + BOARD_THICKNESS * 0.5 + 0.002)),
        material=fabric,
        name="fabric_cover",
    )
    for index, y in enumerate((-0.088, 0.0, 0.088)):
        board.visual(
            Box((0.720, 0.006, 0.0012)),
            origin=Origin(xyz=(-0.150, y, 0.7965)),
            material=fabric_stripe,
            name=f"cover_stripe_{index}",
        )

    # Short metal base bracket fixed directly under the board.  The hinge pin is
    # intentionally very close to this top bracket so the folding leg member has
    # a compact pivot cluster rather than a long dangling linkage.
    board.visual(
        Box((0.400, 0.400, 0.023)),
        origin=Origin(xyz=(0.030, 0.000, 0.7485)),
        material=galvanized,
        name="base_bracket",
    )
    for side, y in enumerate((-0.185, 0.185)):
        board.visual(
            Box((0.110, 0.014, 0.076)),
            origin=Origin(xyz=(HINGE_X, y, 0.700)),
            material=galvanized,
            name=f"hinge_cheek_{side}",
        )
    board.visual(
        Cylinder(radius=0.011, length=0.405),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_joint,
        name="hinge_pin",
    )

    # Small fixed end rest at the broad tail.  It is a real U-shaped tube whose
    # upper ends are seated into the underside of the board.
    end_rest = wire_from_points(
        [
            (-0.555, -0.130, 0.780),
            (-0.555, -0.130, 0.640),
            (-0.555, 0.130, 0.640),
            (-0.555, 0.130, 0.780),
        ],
        radius=0.010,
        radial_segments=18,
        corner_mode="fillet",
        corner_radius=0.035,
        corner_segments=10,
        cap_ends=True,
    )
    board.visual(
        mesh_from_geometry(end_rest, "fixed_end_rest_tube"),
        material=galvanized,
        name="fixed_rest",
    )

    leg_frame = model.part("leg_frame")
    leg_frame.visual(
        Cylinder(radius=0.017, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_joint,
        name="top_barrel",
    )
    u_frame = wire_from_points(
        [
            (0.020, -0.145, -0.010),
            (0.170, -0.145, -0.350),
            (0.310, -0.145, -0.705),
            (0.310, 0.145, -0.705),
            (0.170, 0.145, -0.350),
            (0.020, 0.145, -0.010),
        ],
        radius=0.012,
        radial_segments=18,
        corner_mode="fillet",
        corner_radius=0.040,
        corner_segments=10,
        cap_ends=True,
    )
    leg_frame.visual(
        mesh_from_geometry(u_frame, "folding_u_leg_frame"),
        material=galvanized,
        name="u_tube",
    )
    for side, y in enumerate((-0.145, 0.145)):
        leg_frame.visual(
            Box((0.024, 0.030, 0.024)),
            origin=Origin(xyz=(0.024, y, -0.017)),
            material=galvanized,
            name=f"barrel_weld_{side}",
        )
    for side, y in enumerate((-0.128, 0.128)):
        leg_frame.visual(
            Box((0.070, 0.035, 0.014)),
            origin=Origin(xyz=(0.310, y, -0.718)),
            material=rubber,
            name=f"rubber_foot_{side}",
        )

    model.articulation(
        "bracket_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=board,
        child=leg_frame,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    leg_frame = object_model.get_part("leg_frame")
    hinge = object_model.get_articulation("bracket_to_leg_frame")

    ctx.check("board_part_present", board is not None, "Expected a fixed board assembly.")
    ctx.check("leg_frame_present", leg_frame is not None, "Expected a folding U-shaped leg frame.")
    ctx.check("main_leg_hinge_present", hinge is not None, "Expected the main folding revolute joint.")
    if board is None or leg_frame is None or hinge is None:
        return ctx.report()

    ctx.allow_overlap(
        board,
        leg_frame,
        elem_a="hinge_pin",
        elem_b="top_barrel",
        reason="The leg-frame barrel is intentionally captured around the fixed hinge pin.",
    )
    ctx.expect_overlap(
        board,
        leg_frame,
        axes="xyz",
        elem_a="hinge_pin",
        elem_b="top_barrel",
        min_overlap=0.018,
        name="hinge pin captured in leg barrel",
    )
    ctx.expect_gap(
        board,
        leg_frame,
        axis="z",
        positive_elem="board_core",
        negative_elem="u_tube",
        min_gap=0.030,
        name="unfolded leg clears the board underside",
    )

    board_aabb = ctx.part_element_world_aabb(board, elem="board_core")
    if board_aabb is not None:
        mins, maxs = board_aabb
        length = float(maxs[0] - mins[0])
        width = float(maxs[1] - mins[1])
        ctx.check("board has ironing-board proportions", 1.20 <= length <= 1.35 and 0.34 <= width <= 0.41, f"length={length}, width={width}")

    rest_aabb = ctx.part_world_aabb(leg_frame)
    with ctx.pose({hinge: 0.75}):
        folded_aabb = ctx.part_world_aabb(leg_frame)
        ctx.expect_gap(
            board,
            leg_frame,
            axis="z",
            positive_elem="board_core",
            negative_elem="u_tube",
            min_gap=0.005,
            name="folded leg remains tucked below the board",
        )
    if rest_aabb is not None and folded_aabb is not None:
        rest_min_z = float(rest_aabb[0][2])
        folded_min_z = float(folded_aabb[0][2])
        ctx.check(
            "leg frame folds upward",
            folded_min_z > rest_min_z + 0.18,
            details=f"rest_min_z={rest_min_z:.3f}, folded_min_z={folded_min_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
