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
    model = ArticulatedObject(name="offset_indexing_head")

    cast_dark = Material("dark_cast_iron", rgba=(0.08, 0.095, 0.105, 1.0))
    cast_blue = Material("blue_gray_casting", rgba=(0.18, 0.24, 0.29, 1.0))
    machined = Material("machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_slot = Material("blackened_slots", rgba=(0.015, 0.015, 0.016, 1.0))
    brass = Material("brass_index_marks", rgba=(0.90, 0.68, 0.28, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.34, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_dark,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.205, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=machined,
        name="bearing_ring",
    )
    for i, (x, y) in enumerate(
        ((0.23, 0.23), (-0.23, 0.23), (0.23, -0.23), (-0.23, -0.23))
    ):
        pedestal.visual(
            Cylinder(radius=0.045, length=0.025),
            origin=Origin(xyz=(x, y, -0.0125)),
            material=rubber,
            name=f"foot_{i}",
        )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.26, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=cast_blue,
        name="rotary_table",
    )
    lower_stage.visual(
        Cylinder(radius=0.275, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=machined,
        name="bright_outer_rim",
    )
    lower_stage.visual(
        Cylinder(radius=0.082, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=machined,
        name="center_boss",
    )
    for i in range(12):
        angle = i * (2.0 * math.pi / 12.0)
        lower_stage.visual(
            Box((0.145, 0.008, 0.006)),
            origin=Origin(
                xyz=(0.145 * math.cos(angle), 0.145 * math.sin(angle), 0.089),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_slot,
            name=f"table_slot_{i}",
        )
    for i in range(24):
        angle = i * (2.0 * math.pi / 24.0)
        lower_stage.visual(
            Box((0.012, 0.003, 0.009)),
            origin=Origin(
                xyz=(0.273 * math.cos(angle), 0.273 * math.sin(angle), 0.089),
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"index_tick_{i}",
        )

    # The raised side support is part of the rotating lower stage: a low plinth
    # grows out of the table rim, a thick upright carries the offset head axis,
    # and a shelf under the head makes the load path visible.
    lower_stage.visual(
        Box((0.32, 0.19, 0.075)),
        origin=Origin(xyz=(0.34, 0.0, 0.1225)),
        material=cast_blue,
        name="side_plinth",
    )
    lower_stage.visual(
        Box((0.13, 0.18, 0.32)),
        origin=Origin(xyz=(0.42, 0.0, 0.32)),
        material=cast_blue,
        name="support_arm",
    )
    lower_stage.visual(
        Box((0.18, 0.12, 0.24)),
        origin=Origin(xyz=(0.34, 0.0, 0.30), rpy=(0.0, 0.0, -0.23)),
        material=cast_blue,
        name="diagonal_web",
    )
    lower_stage.visual(
        Box((0.24, 0.22, 0.040)),
        origin=Origin(xyz=(0.42, 0.0, 0.460)),
        material=cast_blue,
        name="head_shelf",
    )

    spin_head = model.part("spin_head")
    spin_head.visual(
        Cylinder(radius=0.130, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=machined,
        name="head_drum",
    )
    spin_head.visual(
        Cylinder(radius=0.112, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=cast_dark,
        name="index_face",
    )
    spin_head.visual(
        Cylinder(radius=0.045, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=machined,
        name="work_spindle",
    )
    for i in range(6):
        angle = i * (2.0 * math.pi / 6.0)
        spin_head.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(
                xyz=(0.075 * math.cos(angle), 0.075 * math.sin(angle), 0.089)
            ),
            material=dark_slot,
            name=f"face_hole_{i}",
        )
    for i in range(8):
        angle = i * (2.0 * math.pi / 8.0)
        spin_head.visual(
            Box((0.058, 0.006, 0.007)),
            origin=Origin(
                xyz=(0.066 * math.cos(angle), 0.066 * math.sin(angle), 0.090),
                rpy=(0.0, 0.0, angle),
            ),
            material=brass,
            name=f"head_tick_{i}",
        )
    spin_head.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(xyz=(0.160, 0.0, 0.041), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="knob_stem",
    )
    spin_head.visual(
        Cylinder(radius=0.030, length=0.035),
        origin=Origin(xyz=(0.222, 0.0, 0.041), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_slot,
        name="knob_grip",
    )

    model.articulation(
        "pedestal_to_lower_stage",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
    )
    model.articulation(
        "lower_stage_to_spin_head",
        ArticulationType.CONTINUOUS,
        parent=lower_stage,
        child=spin_head,
        origin=Origin(xyz=(0.42, 0.0, 0.480)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_stage = object_model.get_part("lower_stage")
    spin_head = object_model.get_part("spin_head")
    main_axis = object_model.get_articulation("pedestal_to_lower_stage")
    offset_axis = object_model.get_articulation("lower_stage_to_spin_head")

    ctx.check(
        "main stage is a vertical rotary axis",
        main_axis.articulation_type == ArticulationType.CONTINUOUS
        and tuple(main_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_axis.articulation_type}, axis={main_axis.axis}",
    )
    ctx.check(
        "upper head is a parallel vertical rotary axis",
        offset_axis.articulation_type == ArticulationType.CONTINUOUS
        and tuple(offset_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={offset_axis.articulation_type}, axis={offset_axis.axis}",
    )
    offset = math.hypot(offset_axis.origin.xyz[0], offset_axis.origin.xyz[1])
    ctx.check(
        "upper rotary axis is visibly offset from the main axis",
        offset > 0.35,
        details=f"offset={offset:.3f} m",
    )

    ctx.expect_gap(
        lower_stage,
        "pedestal",
        axis="z",
        positive_elem="rotary_table",
        negative_elem="bearing_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="large lower stage seats on the low bearing ring",
    )
    ctx.expect_overlap(
        lower_stage,
        "pedestal",
        axes="xy",
        elem_a="rotary_table",
        elem_b="bearing_ring",
        min_overlap=0.35,
        name="large table is centered over the main bearing",
    )
    ctx.expect_gap(
        spin_head,
        lower_stage,
        axis="z",
        positive_elem="head_drum",
        negative_elem="head_shelf",
        max_gap=0.001,
        max_penetration=0.0,
        name="small upper head sits on the raised support shelf",
    )
    ctx.expect_overlap(
        spin_head,
        lower_stage,
        axes="xy",
        elem_a="head_drum",
        elem_b="head_shelf",
        min_overlap=0.18,
        name="head footprint is carried by the side support shelf",
    )

    rest_head_position = ctx.part_world_position(spin_head)
    with ctx.pose({main_axis: 0.8}):
        swung_head_position = ctx.part_world_position(spin_head)
    ctx.check(
        "main rotation carries the offset head around the low base",
        rest_head_position is not None
        and swung_head_position is not None
        and swung_head_position[1] > 0.25
        and abs(
            math.hypot(swung_head_position[0], swung_head_position[1])
            - math.hypot(rest_head_position[0], rest_head_position[1])
        )
        < 0.005,
        details=f"rest={rest_head_position}, swung={swung_head_position}",
    )

    rest_knob = ctx.part_element_world_aabb(spin_head, elem="knob_grip")
    with ctx.pose({offset_axis: 1.0}):
        turned_knob = ctx.part_element_world_aabb(spin_head, elem="knob_grip")
    rest_knob_y = None if rest_knob is None else (rest_knob[0][1] + rest_knob[1][1]) / 2.0
    turned_knob_y = (
        None if turned_knob is None else (turned_knob[0][1] + turned_knob[1][1]) / 2.0
    )
    ctx.check(
        "upper head rotates about its own offset upright axis",
        rest_knob_y is not None
        and turned_knob_y is not None
        and abs(rest_knob_y) < 0.01
        and turned_knob_y > 0.12,
        details=f"rest_knob_y={rest_knob_y}, turned_knob_y={turned_knob_y}",
    )

    return ctx.report()


object_model = build_object_model()
