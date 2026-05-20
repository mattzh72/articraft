from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_rail_module")

    cast_iron = model.material("dark_cast_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = model.material("brushed_rail_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    carriage_gray = model.material("machined_carriage_gray", rgba=(0.38, 0.40, 0.42, 1.0))
    black = model.material("black_oxide", rgba=(0.015, 0.016, 0.017, 1.0))
    spindle_blue = model.material("blued_spindle_body", rgba=(0.08, 0.15, 0.24, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.90, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="base_plate",
    )
    for y, pad_name, cap_name in (
        (-0.065, "rail_pad_0", "rail_cap_0"),
        (0.065, "rail_pad_1", "rail_cap_1"),
    ):
        base.visual(
            Box((0.78, 0.035, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.0475)),
            material=rail_steel,
            name=pad_name,
        )
        base.visual(
            Box((0.76, 0.012, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.063)),
            material=rail_steel,
            name=cap_name,
        )

    base.visual(
        Box((0.72, 0.028, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0385)),
        material=black,
        name="center_cover",
    )
    for idx, y in enumerate((-0.104, 0.104)):
        base.visual(
            Box((0.82, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.040)),
            material=black,
            name=f"cover_strip_{idx}",
        )
        for screw_idx, x in enumerate((-0.31, -0.12, 0.12, 0.31)):
            base.visual(
                Cylinder(radius=0.006, length=0.007),
                origin=Origin(xyz=(x, y, 0.0455)),
                material=rail_steel,
                name=f"cover_screw_{idx}_{screw_idx}",
            )

    for idx, (x, stop_name) in enumerate(((-0.43, "stop_0"), (0.43, "stop_1"))):
        base.visual(
            Box((0.035, 0.20, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=cast_iron,
            name=stop_name,
        )
        bumper_x = x + (0.023 if x < 0.0 else -0.023)
        base.visual(
            Box((0.012, 0.070, 0.030)),
            origin=Origin(xyz=(bumper_x, 0.0, 0.078)),
            material=rubber,
            name=f"bumper_{idx}",
        )

    carriage = model.part("carriage")
    for idx, (y, shoe_name) in enumerate(((-0.065, "bearing_shoe_0"), (0.065, "bearing_shoe_1"))):
        carriage.visual(
            Box((0.188, 0.022, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.009)),
            material=black,
            name=shoe_name,
        )
        for block_idx, x in enumerate((-0.060, 0.060)):
            carriage.visual(
                Box((0.055, 0.034, 0.030)),
                origin=Origin(xyz=(x, y, 0.027)),
                material=carriage_gray,
                name=f"bearing_block_{idx}_{block_idx}",
            )

    carriage.visual(
        Box((0.220, 0.155, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0455)),
        material=carriage_gray,
        name="saddle_block",
    )
    for idx, x in enumerate((-0.116, 0.116)):
        carriage.visual(
            Box((0.014, 0.172, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.038)),
            material=black,
            name=f"end_wiper_{idx}",
        )
    carriage.visual(
        Box((0.190, 0.115, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0765)),
        material=rail_steel,
        name="top_cover",
    )
    for idx, x in enumerate((-0.065, 0.065)):
        for screw_idx, y in enumerate((-0.040, 0.040)):
            carriage.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(xyz=(x, y, 0.0833)),
                material=black,
                name=f"cap_screw_{idx}_{screw_idx}",
            )
    carriage.visual(
        Box((0.118, 0.095, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=carriage_gray,
        name="spindle_pedestal",
    )
    bearing_ring_shape = cq.Workplane("XY").circle(0.054).circle(0.026).extrude(0.016)
    carriage.visual(
        mesh_from_cadquery(bearing_ring_shape, "bearing_ring", tolerance=0.0006),
        origin=Origin(xyz=(0.0, 0.0, 0.1195)),
        material=rail_steel,
        name="bearing_ring",
    )
    for idx, y in enumerate((-0.052, 0.052)):
        carriage.visual(
            Box((0.020, 0.016, 0.046)),
            origin=Origin(xyz=(0.0, y, 0.111)),
            material=carriage_gray,
            name=f"side_support_{idx}",
        )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=rail_steel,
        name="pilot_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rail_steel,
        name="lower_cap",
    )
    spindle.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=spindle_blue,
        name="spindle_body",
    )
    spindle.visual(
        Cylinder(radius=0.041, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=rail_steel,
        name="top_cap",
    )
    spindle.visual(
        Box((0.044, 0.007, 0.004)),
        origin=Origin(xyz=(0.008, 0.0, 0.084)),
        material=black,
        name="index_mark",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.200, 0.0, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.360),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.1355)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=5.0, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("base_to_carriage")
    rotate = object_model.get_articulation("carriage_to_spindle")

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="bearing_shoe_0",
        negative_elem="rail_cap_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="bearing shoe rides on rail cap",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xy",
        elem_a="bearing_shoe_1",
        elem_b="rail_cap_1",
        min_overlap=0.010,
        name="second bearing shoe remains seated on rail",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="x",
        positive_elem="saddle_block",
        negative_elem="stop_0",
        min_gap=0.045,
        name="carriage clears lower travel stop",
    )
    ctx.expect_gap(
        base,
        carriage,
        axis="x",
        positive_elem="stop_1",
        negative_elem="saddle_block",
        min_gap=0.080,
        name="carriage clears upper travel stop at rest",
    )
    ctx.expect_gap(
        spindle,
        carriage,
        axis="z",
        positive_elem="lower_cap",
        negative_elem="bearing_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="spindle lower cap seats on bearing ring",
    )
    ctx.expect_overlap(
        spindle,
        carriage,
        axes="z",
        elem_a="pilot_shaft",
        elem_b="bearing_ring",
        min_overlap=0.010,
        name="pilot shaft is retained inside bearing ring",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.360, rotate: pi / 2.0}):
        extended_position = ctx.part_world_position(carriage)
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem="stop_1",
            negative_elem="saddle_block",
            min_gap=0.080,
            name="extended carriage still clears upper stop",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem="bearing_shoe_0",
            negative_elem="rail_cap_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended bearing shoe stays on rail cap",
        )
        ctx.expect_gap(
            spindle,
            carriage,
            axis="z",
            positive_elem="lower_cap",
            negative_elem="bearing_ring",
            max_gap=0.001,
            max_penetration=0.0,
            name="rotated spindle remains seated above bearing",
        )
    ctx.check(
        "carriage translates along rail axis",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.30
        and abs(extended_position[1] - rest_position[1]) < 0.001,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
