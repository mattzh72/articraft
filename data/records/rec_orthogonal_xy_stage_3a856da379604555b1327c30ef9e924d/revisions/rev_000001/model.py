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
    model = ArticulatedObject(name="compact_pick_and_place_stage")

    cast_aluminum = Material("mat_cast_aluminum", color=(0.58, 0.61, 0.62, 1.0))
    machined_aluminum = Material("mat_machined_aluminum", color=(0.72, 0.75, 0.76, 1.0))
    bright_steel = Material("mat_bright_steel", color=(0.86, 0.86, 0.82, 1.0))
    dark_steel = Material("mat_dark_steel", color=(0.06, 0.065, 0.07, 1.0))
    anodized_plate = Material("mat_anodized_plate", color=(0.18, 0.27, 0.34, 1.0))
    rubber = Material("mat_rubber", color=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.58, 0.42, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_aluminum,
        name="base_plate",
    )
    for x in (-0.230, 0.230):
        for y in (-0.155, 0.155):
            base.visual(
                Box((0.080, 0.055, 0.010)),
                origin=Origin(xyz=(x, y, -0.004)),
                material=rubber,
                name=f"foot_{x:+.2f}_{y:+.2f}",
            )
    for y, name in ((-0.130, "x_rail_0"), (0.130, "x_rail_1")):
        base.visual(
            Box((0.500, 0.032, 0.036)),
            origin=Origin(xyz=(0.0, y, 0.058)),
            material=bright_steel,
            name=name,
        )
        base.visual(
            Box((0.500, 0.006, 0.005)),
            origin=Origin(xyz=(0.0, y - 0.019, 0.043)),
            material=dark_steel,
            name=f"{name}_shadow_a",
        )
        base.visual(
            Box((0.500, 0.006, 0.005)),
            origin=Origin(xyz=(0.0, y + 0.019, 0.043)),
            material=dark_steel,
            name=f"{name}_shadow_b",
        )
    for x, name in ((-0.270, "x_stop_0"), (0.270, "x_stop_1")):
        base.visual(
            Box((0.026, 0.305, 0.066)),
            origin=Origin(xyz=(x, 0.0, 0.072)),
            material=machined_aluminum,
            name=name,
        )

    first_carriage = model.part("first_carriage")
    for y, name in ((-0.130, "x_bearing_0"), (0.130, "x_bearing_1")):
        first_carriage.visual(
            Box((0.200, 0.052, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.012)),
            material=machined_aluminum,
            name=name,
        )
    first_carriage.visual(
        Box((0.280, 0.235, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=machined_aluminum,
        name="first_block",
    )
    first_carriage.visual(
        Box((0.018, 0.210, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=dark_steel,
        name="y_center_groove",
    )
    for x, name in ((-0.080, "y_rail_0"), (0.080, "y_rail_1")):
        first_carriage.visual(
            Box((0.030, 0.215, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.088)),
            material=bright_steel,
            name=name,
        )
        first_carriage.visual(
            Box((0.006, 0.215, 0.004)),
            origin=Origin(xyz=(x - 0.018, 0.0, 0.078)),
            material=dark_steel,
            name=f"{name}_shadow_a",
        )
        first_carriage.visual(
            Box((0.006, 0.215, 0.004)),
            origin=Origin(xyz=(x + 0.018, 0.0, 0.078)),
            material=dark_steel,
            name=f"{name}_shadow_b",
        )

    second_carriage = model.part("second_carriage")
    for x, name in ((-0.080, "y_bearing_0"), (0.080, "y_bearing_1")):
        second_carriage.visual(
            Box((0.052, 0.140, 0.022)),
            origin=Origin(xyz=(x, 0.0, 0.011)),
            material=machined_aluminum,
            name=name,
        )
    second_carriage.visual(
        Box((0.225, 0.180, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=machined_aluminum,
        name="cross_block",
    )
    second_carriage.visual(
        Box((0.165, 0.165, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=anodized_plate,
        name="tooling_deck",
    )
    second_carriage.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=dark_steel,
        name="tool_mount_boss",
    )
    for x in (-0.055, 0.055):
        for y in (-0.055, 0.055):
            second_carriage.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(x, y, 0.092)),
                material=dark_steel,
                name=f"fixture_hole_{x:+.2f}_{y:+.2f}",
            )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.45, lower=-0.080, upper=0.080),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.40, lower=-0.060, upper=0.060),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_carriage")
    x_slide = object_model.get_articulation("x_slide")
    y_slide = object_model.get_articulation("y_slide")

    prismatic = ArticulationType.PRISMATIC
    dot = sum(a * b for a, b in zip(x_slide.axis, y_slide.axis))
    ctx.check(
        "two orthogonal horizontal prismatic joints",
        len(object_model.articulations) == 2
        and x_slide.articulation_type == prismatic
        and y_slide.articulation_type == prismatic
        and abs(dot) < 1e-6
        and abs(x_slide.axis[2]) < 1e-6
        and abs(y_slide.axis[2]) < 1e-6,
        details=f"joints={len(object_model.articulations)}, axes={x_slide.axis},{y_slide.axis}",
    )

    ctx.expect_gap(
        first,
        base,
        axis="z",
        positive_elem="x_bearing_0",
        negative_elem="x_rail_0",
        min_gap=0.0,
        max_gap=0.001,
        name="lower carriage rides on first guide rail",
    )
    ctx.expect_gap(
        first,
        base,
        axis="z",
        positive_elem="x_bearing_1",
        negative_elem="x_rail_1",
        min_gap=0.0,
        max_gap=0.001,
        name="lower carriage rides on second guide rail",
    )
    ctx.expect_overlap(
        first,
        base,
        axes="xy",
        elem_a="x_bearing_0",
        elem_b="x_rail_0",
        min_overlap=0.020,
        name="x bearing footprint follows rail",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        positive_elem="y_bearing_0",
        negative_elem="y_rail_0",
        min_gap=0.0,
        max_gap=0.001,
        name="upper carriage rides on first cross rail",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        positive_elem="y_bearing_1",
        negative_elem="y_rail_1",
        min_gap=0.0,
        max_gap=0.001,
        name="upper carriage rides on second cross rail",
    )
    ctx.expect_overlap(
        second,
        first,
        axes="xy",
        elem_a="y_bearing_0",
        elem_b="y_rail_0",
        min_overlap=0.020,
        name="y bearing footprint follows cross rail",
    )

    deck_box = ctx.part_element_world_aabb(second, elem="tooling_deck")
    body_box = ctx.part_element_world_aabb(second, elem="cross_block")
    if deck_box is not None and body_box is not None:
        deck_min, deck_max = deck_box
        body_min, body_max = body_box
        deck_dx = deck_max[0] - deck_min[0]
        deck_dy = deck_max[1] - deck_min[1]
        deck_above = deck_min[2] >= body_max[2] - 0.001
    else:
        deck_dx = deck_dy = 0.0
        deck_above = False
    ctx.check(
        "square tooling deck sits above upper carriage",
        abs(deck_dx - deck_dy) < 0.002 and deck_dx > 0.150 and deck_above,
        details=f"deck_dx={deck_dx}, deck_dy={deck_dy}, deck_above={deck_above}",
    )

    rest_first = ctx.part_world_position(first)
    with ctx.pose({x_slide: x_slide.motion_limits.upper}):
        moved_first = ctx.part_world_position(first)
    ctx.check(
        "x slide moves the lower carriage along x only",
        rest_first is not None
        and moved_first is not None
        and moved_first[0] > rest_first[0] + 0.070
        and abs(moved_first[1] - rest_first[1]) < 1e-6
        and abs(moved_first[2] - rest_first[2]) < 1e-6,
        details=f"rest={rest_first}, moved={moved_first}",
    )

    rest_second = ctx.part_world_position(second)
    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        moved_second = ctx.part_world_position(second)
    ctx.check(
        "y slide moves the upper carriage along y only",
        rest_second is not None
        and moved_second is not None
        and moved_second[1] > rest_second[1] + 0.050
        and abs(moved_second[0] - rest_second[0]) < 1e-6
        and abs(moved_second[2] - rest_second[2]) < 1e-6,
        details=f"rest={rest_second}, moved={moved_second}",
    )

    return ctx.report()


object_model = build_object_model()
