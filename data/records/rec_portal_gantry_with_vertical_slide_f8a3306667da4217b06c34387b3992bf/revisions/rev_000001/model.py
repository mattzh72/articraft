from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_bed_portal_gantry")

    cast_iron = model.material("dark_cast_iron", rgba=(0.10, 0.12, 0.13, 1.0))
    machine_blue = model.material("machine_blue", rgba=(0.08, 0.22, 0.38, 1.0))
    painted_gray = model.material("painted_gray", rgba=(0.46, 0.49, 0.50, 1.0))
    hardened_steel = model.material("hardened_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.03, 0.035, 0.04, 1.0))

    bed = model.part("machine_bed")
    bed.visual(
        Box((2.20, 1.60, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=cast_iron,
        name="base_slab",
    )
    for idx, x in enumerate((-0.88, 0.88)):
        bed.visual(
            Box((0.24, 1.48, 0.085)),
            origin=Origin(xyz=(x, 0.0, 0.200)),
            material=painted_gray,
            name=f"side_plinth_{idx}",
        )
        bed.visual(
            Box((0.10, 1.38, 0.037)),
            origin=Origin(xyz=(x, 0.0, 0.2565)),
            material=hardened_steel,
            name=("linear_rail_0", "linear_rail_1")[idx],
        )
        for end_idx, y in enumerate((-0.715, 0.715)):
            bed.visual(
                Box((0.15, 0.07, 0.10)),
                origin=Origin(xyz=(x, y, 0.324)),
                material=blackened_steel,
                name=f"end_stop_{idx}_{end_idx}",
            )
    for idx, x in enumerate((-0.45, 0.0, 0.45)):
        bed.visual(
            Box((0.045, 1.12, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.163)),
            material=blackened_steel,
            name=f"table_slot_{idx}",
        )

    beam = model.part("beam")
    for idx, x in enumerate((-0.88, 0.88)):
        beam.visual(
            Box((0.26, 0.30, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.06)),
            material=machine_blue,
            name=("truck_0", "truck_1")[idx],
        )
        beam.visual(
            Box((0.20, 0.22, 0.96)),
            origin=Origin(xyz=(x, 0.0, 0.59)),
            material=machine_blue,
            name=f"upright_{idx}",
        )
    beam.visual(
        Box((1.78, 0.24, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        material=machine_blue,
        name="crossbeam",
    )
    beam.visual(
        Box((1.45, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, -0.135, 1.06)),
        material=hardened_steel,
        name="beam_rail_upper",
    )
    beam.visual(
        Box((1.45, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, -0.135, 0.92)),
        material=hardened_steel,
        name="beam_rail_lower",
    )
    beam.visual(
        Box((1.54, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.126, 0.99)),
        material=blackened_steel,
        name="rack_strip",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.34, 0.13, 0.46)),
        origin=Origin(xyz=(0.0, -0.070, 0.0)),
        material=painted_gray,
        name="saddle_plate",
    )
    carriage.visual(
        Box((0.30, 0.029, 0.055)),
        origin=Origin(xyz=(0.0, 0.0055, 0.07)),
        material=blackened_steel,
        name="bearing_upper",
    )
    carriage.visual(
        Box((0.30, 0.029, 0.055)),
        origin=Origin(xyz=(0.0, 0.0055, -0.07)),
        material=blackened_steel,
        name="bearing_lower",
    )
    for idx, x in enumerate((-0.10, 0.10)):
        carriage.visual(
            Box((0.035, 0.026, 0.56)),
            origin=Origin(xyz=(x, -0.148, 0.02)),
            material=hardened_steel,
            name=("vertical_rail_0", "vertical_rail_1")[idx],
        )
    carriage.visual(
        Box((0.25, 0.030, 0.08)),
        origin=Origin(xyz=(0.0, -0.151, 0.28)),
        material=blackened_steel,
        name="top_bumper",
    )

    head = model.part("vertical_head")
    head.visual(
        Box((0.22, 0.12, 0.50)),
        origin=Origin(xyz=(0.0, -0.065, -0.18)),
        material=painted_gray,
        name="ram_body",
    )
    for idx, x in enumerate((-0.10, 0.10)):
        head.visual(
            Box((0.052, 0.030, 0.38)),
            origin=Origin(xyz=(x, 0.009, -0.10)),
            material=blackened_steel,
            name=("head_bearing_0", "head_bearing_1")[idx],
        )
    head.visual(
        Box((0.26, 0.035, 0.24)),
        origin=Origin(xyz=(0.0, -0.1425, -0.22)),
        material=cast_iron,
        name="faceplate",
    )

    model.articulation(
        "bed_to_beam",
        ArticulationType.PRISMATIC,
        parent=bed,
        child=beam,
        origin=Origin(xyz=(0.0, -0.45, 0.275)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.60, lower=0.0, upper=0.90),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.170, 0.99)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.70, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.0, -0.185, 0.14)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.30, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("machine_bed")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("vertical_head")
    bed_slide = object_model.get_articulation("bed_to_beam")
    carriage_slide = object_model.get_articulation("beam_to_carriage")
    head_slide = object_model.get_articulation("carriage_to_head")

    for joint, axis in (
        (bed_slide, (0.0, 1.0, 0.0)),
        (carriage_slide, (1.0, 0.0, 0.0)),
        (head_slide, (0.0, 0.0, -1.0)),
    ):
        ctx.check(
            f"{joint.name} is prismatic on its supported axis",
            joint.articulation_type == ArticulationType.PRISMATIC and tuple(joint.axis) == axis,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_gap(
        beam,
        bed,
        axis="z",
        positive_elem="truck_0",
        negative_elem="linear_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="beam truck sits on bed rail",
    )
    ctx.expect_overlap(
        beam,
        bed,
        axes="xy",
        elem_a="truck_0",
        elem_b="linear_rail_0",
        min_overlap=0.08,
        name="beam truck footprint stays over bed rail",
    )
    ctx.expect_gap(
        beam,
        carriage,
        axis="y",
        positive_elem="beam_rail_upper",
        negative_elem="bearing_upper",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage bearing rides beam rail",
    )
    ctx.expect_gap(
        carriage,
        head,
        axis="y",
        positive_elem="vertical_rail_0",
        negative_elem="head_bearing_0",
        max_gap=0.001,
        max_penetration=0.000001,
        name="head bearing rides vertical way",
    )

    rest_beam_pos = ctx.part_world_position(beam)
    with ctx.pose({bed_slide: 0.90}):
        ctx.expect_within(
            beam,
            bed,
            axes="y",
            inner_elem="truck_0",
            outer_elem="linear_rail_0",
            margin=0.0,
            name="advanced beam truck remains on rail",
        )
        advanced_beam_pos = ctx.part_world_position(beam)
    ctx.check(
        "beam advances along bed",
        rest_beam_pos is not None
        and advanced_beam_pos is not None
        and advanced_beam_pos[1] > rest_beam_pos[1] + 0.85,
        details=f"rest={rest_beam_pos}, advanced={advanced_beam_pos}",
    )

    with ctx.pose({carriage_slide: 0.55}):
        ctx.expect_within(
            carriage,
            beam,
            axes="x",
            inner_elem="saddle_plate",
            outer_elem="beam_rail_upper",
            margin=0.0,
            name="carriage remains on positive beam rail travel",
        )
    with ctx.pose({carriage_slide: -0.55}):
        ctx.expect_within(
            carriage,
            beam,
            axes="x",
            inner_elem="saddle_plate",
            outer_elem="beam_rail_upper",
            margin=0.0,
            name="carriage remains on negative beam rail travel",
        )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({head_slide: 0.22}):
        ctx.expect_overlap(
            head,
            carriage,
            axes="z",
            elem_a="head_bearing_0",
            elem_b="vertical_rail_0",
            min_overlap=0.12,
            name="lowered head keeps guide engagement",
        )
        lowered_head_pos = ctx.part_world_position(head)
    ctx.check(
        "head feeds downward",
        rest_head_pos is not None
        and lowered_head_pos is not None
        and lowered_head_pos[2] < rest_head_pos[2] - 0.20,
        details=f"rest={rest_head_pos}, lowered={lowered_head_pos}",
    )

    return ctx.report()


object_model = build_object_model()
