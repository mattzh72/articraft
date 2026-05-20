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
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_square_lazy_susan")

    wood = model.material("warm_bamboo_board", rgba=(0.74, 0.52, 0.30, 1.0))
    end_grain = model.material("darker_end_grain", rgba=(0.49, 0.30, 0.15, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.63, 0.61, 1.0))
    shadow_steel = model.material("shadowed_steel", rgba=(0.30, 0.31, 0.31, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("turntable_base")
    base.visual(
        Cylinder(radius=0.205, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=shadow_steel,
        name="base_disk",
    )
    for index, (x, y) in enumerate(((0.135, 0.135), (-0.135, 0.135), (-0.135, -0.135), (0.135, -0.135))):
        base.visual(
            Cylinder(radius=0.024, length=0.004),
            origin=Origin(xyz=(x, y, 0.002)),
            material=black_rubber,
            name=f"rubber_foot_{index}",
        )

    lower_race_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.155, tube=0.004, radial_segments=16, tubular_segments=96),
        "lower_bearing_race",
    )
    base.visual(
        lower_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=brushed_steel,
        name="lower_race",
    )

    top = model.part("top_board")
    board_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.520, 0.520, 0.014, corner_segments=8),
            0.022,
            center=True,
        ),
        "rounded_square_board",
    )
    top.visual(
        board_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=wood,
        name="square_board",
    )
    top.visual(
        Box((0.518, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.262, 0.016)),
        material=end_grain,
        name="front_edge_band",
    )
    top.visual(
        Box((0.518, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, -0.262, 0.016)),
        material=end_grain,
        name="rear_edge_band",
    )
    top.visual(
        Box((0.018, 0.518, 0.003)),
        origin=Origin(xyz=(0.262, 0.0, 0.016)),
        material=end_grain,
        name="side_edge_band_0",
    )
    top.visual(
        Box((0.018, 0.518, 0.003)),
        origin=Origin(xyz=(-0.262, 0.0, 0.016)),
        material=end_grain,
        name="side_edge_band_1",
    )
    for index, y in enumerate((-0.145, -0.072, 0.000, 0.081, 0.158)):
        top.visual(
            Box((0.455, 0.003, 0.0007)),
            origin=Origin(xyz=(0.0, y, 0.02735)),
            material=end_grain,
            name=f"grain_line_{index}",
        )

    top.visual(
        Cylinder(radius=0.170, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shadow_steel,
        name="rotating_plate",
    )
    upper_race_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.155, tube=0.004, radial_segments=16, tubular_segments=96),
        "upper_bearing_race",
    )
    top.visual(
        upper_race_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=brushed_steel,
        name="upper_race",
    )

    model.articulation(
        "base_to_top",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("turntable_base")
    top = object_model.get_part("top_board")
    swivel = object_model.get_articulation("base_to_top")

    ctx.check(
        "top uses continuous vertical rotation",
        swivel.articulation_type == ArticulationType.CONTINUOUS and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            top,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            name="upper and lower bearing races meet at the running track",
        )
        ctx.expect_within(
            base,
            top,
            axes="xy",
            inner_elem="base_disk",
            outer_elem="square_board",
            margin=0.0,
            name="round turntable base hides under square board footprint",
        )
        ctx.expect_overlap(
            top,
            base,
            axes="xy",
            elem_a="square_board",
            elem_b="base_disk",
            min_overlap=0.35,
            name="square top is centered over circular base",
        )

    rest_pos = ctx.part_world_position(top)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(top)
        ctx.expect_within(
            base,
            top,
            axes="xy",
            inner_elem="base_disk",
            outer_elem="square_board",
            margin=0.0,
            name="base remains covered after quarter turn",
        )
    ctx.check(
        "rotation axis stays centered through the bearing",
        rest_pos is not None and turned_pos is not None and abs(rest_pos[0] - turned_pos[0]) < 1e-9 and abs(rest_pos[1] - turned_pos[1]) < 1e-9,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
