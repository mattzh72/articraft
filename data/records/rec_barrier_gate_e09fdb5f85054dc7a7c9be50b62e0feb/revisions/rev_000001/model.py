from __future__ import annotations

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
    model = ArticulatedObject(name="vertical_beam_road_blocker")

    galvanized = Material("galvanized_steel", rgba=(0.55, 0.58, 0.58, 1.0))
    dark_steel = Material("dark_guide_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    safety_yellow = Material("safety_yellow_paint", rgba=(1.0, 0.73, 0.05, 1.0))
    black = Material("black_reflective_stripe", rgba=(0.01, 0.01, 0.01, 1.0))

    housing = model.part("frame")
    housing.visual(
        Box((3.10, 0.55, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_steel,
        name="base_plate",
    )
    housing.visual(
        Box((3.10, 0.07, 1.17)),
        origin=Origin(xyz=(0.0, 0.245, 0.675)),
        material=galvanized,
        name="rear_backplate",
    )
    for suffix, x in (("0", -1.43), ("1", 1.43)):
        housing.visual(
            Box((0.24, 0.46, 1.12)),
            origin=Origin(xyz=(x, 0.0, 0.65)),
            material=galvanized,
            name=f"side_column_{suffix}",
        )
    housing.visual(
        Box((3.10, 0.46, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 1.275)),
        material=galvanized,
        name="top_header",
    )

    # Four proud guide rails make the fixed sliding channels visible while
    # leaving a real clearance envelope for the lifting beam and its end shoes.
    housing.visual(
        Box((0.05, 0.05, 1.115)),
        origin=Origin(xyz=(-1.255, -0.125, 0.6475)),
        material=dark_steel,
        name="guide_rail_0_front",
    )
    housing.visual(
        Box((0.05, 0.05, 1.115)),
        origin=Origin(xyz=(-1.255, 0.125, 0.6475)),
        material=dark_steel,
        name="guide_rail_0_rear",
    )
    housing.visual(
        Box((0.05, 0.05, 1.115)),
        origin=Origin(xyz=(1.255, -0.125, 0.6475)),
        material=dark_steel,
        name="guide_rail_1_front",
    )
    housing.visual(
        Box((0.05, 0.05, 1.115)),
        origin=Origin(xyz=(1.255, 0.125, 0.6475)),
        material=dark_steel,
        name="guide_rail_1_rear",
    )

    for suffix, x in (("0", -1.33), ("1", 1.33)):
        for y in (-0.18, 0.18):
            housing.visual(
                Cylinder(radius=0.035, length=0.012),
                origin=Origin(xyz=(x, y, 0.106)),
                material=dark_steel,
                name=f"anchor_bolt_{suffix}_{'front' if y < 0.0 else 'rear'}",
            )

    beam = model.part("beam")
    beam.visual(
        Box((2.22, 0.15, 0.20)),
        origin=Origin(),
        material=safety_yellow,
        name="beam_bar",
    )
    beam.visual(
        Box((0.18, 0.20, 0.26)),
        origin=Origin(xyz=(-1.14, 0.0, 0.0)),
        material=dark_steel,
        name="end_shoe_0",
    )
    beam.visual(
        Box((0.18, 0.20, 0.26)),
        origin=Origin(xyz=(1.14, 0.0, 0.0)),
        material=dark_steel,
        name="end_shoe_1",
    )

    # Thin diagonal hazard plates are attached proud of both faces of the flat
    # steel beam so the road blocker reads correctly from either traffic side.
    stripe_angle = 0.78
    for face_name, y, angle in (("front", -0.081, stripe_angle), ("rear", 0.081, stripe_angle)):
        for index, x in enumerate((-0.86, -0.43, 0.0, 0.43, 0.86)):
            beam.visual(
                Box((0.08, 0.012, 0.22)),
                origin=Origin(xyz=(x, y, 0.0), rpy=(0.0, angle, 0.0)),
                material=black,
                name=f"stripe_{face_name}_{index}",
            )

    model.articulation(
        "frame_to_beam",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.18, lower=0.0, upper=0.68),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    beam = object_model.get_part("beam")
    lift = object_model.get_articulation("frame_to_beam")

    ctx.check(
        "beam uses vertical prismatic lift",
        lift.articulation_type == ArticulationType.PRISMATIC and tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    ctx.expect_gap(
        beam,
        frame,
        axis="z",
        positive_elem="end_shoe_0",
        negative_elem="base_plate",
        min_gap=0.06,
        max_gap=0.09,
        name="lowered beam clears base sill",
    )
    ctx.expect_contact(
        beam,
        frame,
        elem_a="end_shoe_1",
        elem_b="guide_rail_1_rear",
        name="rear guide shoe contact at rest",
    )
    ctx.expect_contact(
        beam,
        frame,
        elem_a="end_shoe_1",
        elem_b="guide_rail_1_front",
        name="front guide shoe contact at rest",
    )
    ctx.expect_contact(
        beam,
        frame,
        elem_a="end_shoe_1",
        elem_b="guide_rail_1_front",
        name="side guide shoe contact at rest",
    )

    rest_pos = ctx.part_world_position(beam)
    with ctx.pose({lift: 0.68}):
        ctx.expect_gap(
            frame,
            beam,
            axis="z",
            positive_elem="top_header",
            negative_elem="end_shoe_0",
            min_gap=0.06,
            max_gap=0.09,
            name="raised beam remains below header",
        )
        ctx.expect_contact(
            beam,
            frame,
            elem_a="end_shoe_1",
            elem_b="guide_rail_1_rear",
            name="rear guide shoe contact raised",
        )
        ctx.expect_contact(
            beam,
            frame,
            elem_a="end_shoe_1",
            elem_b="guide_rail_1_front",
            name="side guide shoe contact raised",
        )
        raised_pos = ctx.part_world_position(beam)

    ctx.check(
        "beam translates upward on lift joint",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.65,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
