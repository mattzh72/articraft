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
import cadquery as cq


def _ironing_board_profile() -> list[tuple[float, float]]:
    """Plan-view outline for a real-size tapered ironing-board top."""
    upper: list[tuple[float, float]] = []

    # Rounded, wider iron-rest end.
    rear_cx = -0.56
    rear_rx = 0.09
    rear_ry = 0.185
    for i in range(9):
        theta = math.pi - (math.pi / 2.0) * (i / 8.0)
        upper.append((rear_cx + rear_rx * math.cos(theta), rear_ry * math.sin(theta)))

    # Long nearly parallel sides, then a graceful taper to the nose.
    upper.extend(
        [
            (-0.20, 0.190),
            (0.18, 0.180),
            (0.38, 0.135),
        ]
    )
    for i in range(1, 12):
        t = i / 11.0
        x = 0.38 + 0.30 * t
        y = 0.135 * (1.0 - t) ** 0.72
        upper.append((x, y))

    lower = [(x, -y) for x, y in reversed(upper[1:-1])]
    return upper + lower


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    fabric = model.material("blue_fabric", rgba=(0.47, 0.64, 0.78, 1.0))
    pale_thread = model.material("pale_thread", rgba=(0.82, 0.90, 0.96, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.25, 0.27, 0.28, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    heat_pad = model.material("heat_rest_pad", rgba=(0.17, 0.18, 0.19, 1.0))

    board = model.part("board")

    top_geom = ExtrudeGeometry.from_z0(_ironing_board_profile(), 0.040)
    board.visual(
        mesh_from_geometry(top_geom, "tapered_padded_top"),
        material=fabric,
        name="tapered_padded_top",
    )

    # Subtle stitched bands on the cloth cover, kept short so they stay inside
    # the tapered planform.
    for idx, (x, width) in enumerate(
        [(-0.38, 0.26), (-0.13, 0.30), (0.12, 0.28), (0.34, 0.20)]
    ):
        board.visual(
            Box((0.010, width, 0.002)),
            origin=Origin(xyz=(x, 0.0, 0.041), rpy=(0.0, 0.0, math.radians(15.0))),
            material=pale_thread,
            name=f"cloth_band_{idx}",
        )

    # Metal underside plate and hinge clevis carried by the board.
    board.visual(
        Box((0.44, 0.082, 0.018)),
        origin=Origin(xyz=(-0.03, 0.0, -0.006)),
        material=satin_metal,
        name="mount_plate",
    )
    board.visual(
        Box((0.065, 0.450, 0.016)),
        origin=Origin(xyz=(0.030, 0.0, -0.020)),
        material=dark_metal,
        name="clevis_crossrail",
    )
    board.visual(
        Box((0.105, 0.026, 0.082)),
        origin=Origin(xyz=(0.030, -0.220, -0.045)),
        material=dark_metal,
        name="clevis_strap_0",
    )
    board.visual(
        Cylinder(radius=0.025, length=0.034),
        origin=Origin(xyz=(0.030, -0.220, -0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_boss_0",
    )
    board.visual(
        Box((0.105, 0.026, 0.082)),
        origin=Origin(xyz=(0.030, 0.220, -0.045)),
        material=dark_metal,
        name="clevis_strap_1",
    )
    board.visual(
        Cylinder(radius=0.025, length=0.034),
        origin=Origin(xyz=(0.030, 0.220, -0.075), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_boss_1",
    )

    # A small fixed iron rest at the wider rear end: a heat pad plus a raised
    # chrome wire cradle with its feet embedded in the board cover.
    board.visual(
        Box((0.210, 0.250, 0.006)),
        origin=Origin(xyz=(-0.545, 0.0, 0.043)),
        material=heat_pad,
        name="rest_pad",
    )
    rest_wire = wire_from_points(
        [
            (-0.470, -0.135, 0.035),
            (-0.555, -0.145, 0.075),
            (-0.630, -0.070, 0.090),
            (-0.630, 0.070, 0.090),
            (-0.555, 0.145, 0.075),
            (-0.470, 0.135, 0.035),
        ],
        radius=0.0075,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.028,
        corner_segments=10,
    )
    board.visual(
        mesh_from_geometry(rest_wire, "fixed_wire_rest"),
        material=satin_metal,
        name="fixed_wire_rest",
    )

    leg = model.part("leg_frame")
    u_frame = wire_from_points(
        [
            (0.0, -0.165, 0.0),
            (0.035, -0.165, -0.230),
            (0.155, -0.165, -0.720),
            (0.155, 0.165, -0.720),
            (0.035, 0.165, -0.230),
            (0.0, 0.165, 0.0),
        ],
        radius=0.014,
        radial_segments=20,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.045,
        corner_segments=12,
    )
    leg.visual(
        mesh_from_geometry(u_frame, "bent_u_tube"),
        material=satin_metal,
        name="bent_u_tube",
    )
    leg.visual(
        Cylinder(radius=0.018, length=0.410),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_crossbar",
    )
    for side, y in enumerate((-0.165, 0.165)):
        leg.visual(
            Box((0.105, 0.070, 0.030)),
            origin=Origin(xyz=(0.155, y, -0.727)),
            material=black_rubber,
            name=f"rubber_foot_{side}",
        )

    model.articulation(
        "board_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=board,
        child=leg,
        origin=Origin(xyz=(0.030, 0.0, -0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    leg = object_model.get_part("leg_frame")
    hinge = object_model.get_articulation("board_to_leg_frame")

    ctx.allow_overlap(
        board,
        leg,
        elem_a="hinge_boss_0",
        elem_b="pivot_crossbar",
        reason="The leg pivot tube is intentionally captured a few millimeters inside the fixed hinge boss.",
    )
    ctx.allow_overlap(
        board,
        leg,
        elem_a="hinge_boss_1",
        elem_b="pivot_crossbar",
        reason="The leg pivot tube is intentionally captured a few millimeters inside the fixed hinge boss.",
    )

    ctx.check(
        "main leg hinge is horizontal",
        hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    ctx.expect_overlap(
        leg,
        board,
        axes="x",
        elem_a="pivot_crossbar",
        elem_b="mount_plate",
        min_overlap=0.030,
        name="leg hinge sits under middle bracket",
    )
    ctx.expect_gap(
        board,
        leg,
        axis="z",
        positive_elem="mount_plate",
        negative_elem="pivot_crossbar",
        min_gap=0.010,
        max_gap=0.050,
        name="pivot crossbar is just below board bracket",
    )
    ctx.expect_gap(
        leg,
        board,
        axis="y",
        positive_elem="pivot_crossbar",
        negative_elem="hinge_boss_0",
        max_penetration=0.004,
        max_gap=0.004,
        name="negative hinge boss captures pivot end",
    )
    ctx.expect_gap(
        board,
        leg,
        axis="y",
        positive_elem="hinge_boss_1",
        negative_elem="pivot_crossbar",
        max_penetration=0.004,
        max_gap=0.004,
        name="positive hinge boss captures pivot end",
    )

    open_aabb = ctx.part_world_aabb(leg)
    with ctx.pose({hinge: 1.20}):
        folded_aabb = ctx.part_world_aabb(leg)
    ctx.check(
        "leg frame folds upward",
        open_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > open_aabb[0][2] + 0.35,
        details=f"open={open_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
