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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_industrial_lift_carriage")

    steel = model.material("painted_steel", rgba=(0.18, 0.22, 0.25, 1.0))
    rail_steel = model.material("bright_wear_steel", rgba=(0.70, 0.74, 0.74, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    dark_rubber = model.material("dark_guide_rollers", rgba=(0.025, 0.025, 0.022, 1.0))
    bolt_dark = model.material("blackened_bolts", rgba=(0.04, 0.04, 0.04, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.12, 0.10, 2.05)),
        origin=Origin(xyz=(-0.46, 0.0, 1.025)),
        material=steel,
        name="upright_0",
    )
    frame.visual(
        Box((0.12, 0.10, 2.05)),
        origin=Origin(xyz=(0.46, 0.0, 1.025)),
        material=steel,
        name="upright_1",
    )
    frame.visual(
        Box((1.08, 0.14, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 2.08)),
        material=steel,
        name="top_tie",
    )
    frame.visual(
        Box((1.08, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=steel,
        name="bottom_tie",
    )
    frame.visual(
        Box((0.08, 0.018, 1.70)),
        origin=Origin(xyz=(-0.46, -0.057, 0.98)),
        material=rail_steel,
        name="rail_0",
    )
    frame.visual(
        Box((0.08, 0.018, 1.70)),
        origin=Origin(xyz=(0.46, -0.057, 0.98)),
        material=rail_steel,
        name="rail_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.90, 0.055, 0.58)),
        origin=Origin(xyz=(0.0, -0.300, 0.0)),
        material=safety_orange,
        name="tooling_plate",
    )
    carriage.visual(
        Box((0.82, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, -0.335, 0.235)),
        material=steel,
        name="top_plate_rib",
    )
    carriage.visual(
        Box((0.82, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, -0.335, -0.235)),
        material=steel,
        name="bottom_plate_rib",
    )

    for side_index, x in enumerate((-0.46, 0.46)):
        rail_name = f"guide_side_{side_index}"
        carriage.visual(
            Box((0.13, 0.18, 0.55)),
            origin=Origin(xyz=(x, -0.215, 0.0)),
            material=steel,
            name=rail_name,
        )
        for level_index, z in enumerate((-0.22, 0.22)):
            suffix = f"{side_index}_{level_index}"
            carriage.visual(
                Box((0.18, 0.08, 0.13)),
                origin=Origin(xyz=(x, -0.120, z)),
                material=steel,
                name=f"guide_block_{suffix}",
            )
            carriage.visual(
                Cylinder(radius=0.035, length=0.115),
                origin=Origin(xyz=(x, -0.101, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_rubber,
                name=f"guide_roller_{suffix}",
            )

    for x in (-0.26, 0.0, 0.26):
        for z in (-0.16, 0.16):
            carriage.visual(
                Cylinder(radius=0.022, length=0.014),
                origin=Origin(xyz=(x, -0.332, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=bolt_dark,
                name=f"plate_bolt_{x:.2f}_{z:.2f}",
            )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=0.45, lower=0.0, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")

    ctx.check(
        "single lift axis is prismatic",
        len(object_model.articulations) == 1
        and lift.articulation_type == ArticulationType.PRISMATIC
        and lift.axis == (0.0, 0.0, 1.0),
        details=f"articulations={object_model.articulations}",
    )

    for side_index, rail in enumerate(("rail_0", "rail_1")):
        for level_index in range(2):
            roller = f"guide_roller_{side_index}_{level_index}"
            ctx.expect_overlap(
                frame,
                carriage,
                axes="xz",
                elem_a=rail,
                elem_b=roller,
                min_overlap=0.03,
                name=f"{roller} lines up with {rail}",
            )
            ctx.expect_gap(
                frame,
                carriage,
                axis="y",
                positive_elem=rail,
                negative_elem=roller,
                min_gap=0.0,
                max_gap=0.006,
                name=f"{roller} closely follows {rail}",
            )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.80}):
        raised_position = ctx.part_world_position(carriage)
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="top_tie",
            negative_elem="tooling_plate",
            min_gap=0.10,
            name="raised plate clears top tie",
        )
        for side_index, rail in enumerate(("rail_0", "rail_1")):
            for level_index in range(2):
                ctx.expect_overlap(
                    frame,
                    carriage,
                    axes="z",
                    elem_a=rail,
                    elem_b=f"guide_roller_{side_index}_{level_index}",
                    min_overlap=0.03,
                    name=f"raised guide {side_index}_{level_index} remains on rail",
                )

    ctx.check(
        "carriage raises along uprights",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.75,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
