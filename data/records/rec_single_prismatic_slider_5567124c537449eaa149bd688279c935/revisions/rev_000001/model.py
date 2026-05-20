from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="drawer_instrument_slide")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.095, 1.0))
    bright = model.material("machined_bright_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    wear = model.material("oiled_bronze_wear_strip", rgba=(0.78, 0.55, 0.24, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    fastener = model.material("black_oxide_fastener", rgba=(0.015, 0.014, 0.012, 1.0))

    base = model.part("base_channel")
    base.visual(
        Box((0.720, 0.260, 0.018)),
        origin=Origin(xyz=(-0.320, 0.0, 0.009)),
        material=anodized,
        name="base_floor",
    )
    for y, name in ((0.115, "side_wall_pos"), (-0.115, "side_wall_neg")):
        base.visual(
            Box((0.720, 0.035, 0.067)),
            origin=Origin(xyz=(-0.320, y, 0.0515)),
            material=anodized,
            name=name,
        )
    for y, name in ((0.075, "top_lip_pos"), (-0.075, "top_lip_neg")):
        base.visual(
            Box((0.650, 0.045, 0.018)),
            origin=Origin(xyz=(-0.345, y, 0.087)),
            material=anodized,
            name=name,
        )
    base.visual(
        Box((0.590, 0.007, 0.035)),
        origin=Origin(xyz=(-0.360, 0.0945, 0.050)),
        material=wear,
        name="side_wear_pos",
    )
    base.visual(
        Box((0.590, 0.007, 0.035)),
        origin=Origin(xyz=(-0.360, -0.0945, 0.050)),
        material=wear,
        name="side_wear_neg",
    )
    base.visual(
        Box((0.610, 0.026, 0.007)),
        origin=Origin(xyz=(-0.350, 0.052, 0.0215)),
        material=wear,
        name="bottom_wear_pos",
    )
    base.visual(
        Box((0.610, 0.026, 0.007)),
        origin=Origin(xyz=(-0.350, -0.052, 0.0215)),
        material=wear,
        name="bottom_wear_neg",
    )
    base.visual(
        Box((0.055, 0.037, 0.024)),
        origin=Origin(xyz=(0.012, 0.115, 0.097)),
        material=bright,
        name="front_lug_pos",
    )
    base.visual(
        Box((0.055, 0.037, 0.024)),
        origin=Origin(xyz=(0.012, -0.115, 0.097)),
        material=bright,
        name="front_lug_neg",
    )
    for x, y, name in (
        (-0.580, 0.115, "base_screw_0"),
        (-0.200, 0.115, "base_screw_1"),
        (-0.580, -0.115, "base_screw_2"),
        (-0.200, -0.115, "base_screw_3"),
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, y, 0.087), rpy=(0.0, 0.0, 0.0)),
            material=fastener,
            name=name,
        )

    carriage = model.part("carriage_tray")
    carriage.visual(
        Box((0.686, 0.150, 0.014)),
        origin=Origin(xyz=(-0.277, 0.0, 0.047)),
        material=bright,
        name="tray_plate",
    )
    carriage.visual(
        Box((0.675, 0.024, 0.017)),
        origin=Origin(xyz=(-0.2825, 0.052, 0.0335)),
        material=bright,
        name="runner_pos",
    )
    carriage.visual(
        Box((0.675, 0.024, 0.017)),
        origin=Origin(xyz=(-0.2825, -0.052, 0.0335)),
        material=bright,
        name="runner_neg",
    )
    for y, name in ((0.082, "tray_flange_pos"), (-0.082, "tray_flange_neg")):
        carriage.visual(
            Box((0.615, 0.014, 0.026)),
            origin=Origin(xyz=(-0.2525, y, 0.060)),
            material=bright,
            name=name,
        )
    carriage.visual(
        Box((0.505, 0.004, 0.022)),
        origin=Origin(xyz=(-0.300, 0.087, 0.058)),
        material=wear,
        name="side_bearing_pos",
    )
    carriage.visual(
        Box((0.505, 0.004, 0.022)),
        origin=Origin(xyz=(-0.300, -0.087, 0.058)),
        material=wear,
        name="side_bearing_neg",
    )
    carriage.visual(
        Box((0.050, 0.180, 0.072)),
        origin=Origin(xyz=(0.087, 0.0, 0.076)),
        material=anodized,
        name="front_stop",
    )
    carriage.visual(
        Box((0.010, 0.130, 0.040)),
        origin=Origin(xyz=(0.117, 0.0, 0.076)),
        material=rubber,
        name="bumper_pad",
    )
    for y, name in ((0.044, "stop_screw_pos"), (-0.044, "stop_screw_neg")):
        carriage.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.122, y, 0.083), rpy=(0.0, pi / 2.0, 0.0)),
            material=fastener,
            name=name,
        )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.260),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_channel")
    carriage = object_model.get_part("carriage_tray")
    slide = object_model.get_articulation("base_to_carriage")

    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="runner_pos",
            elem_b="bottom_wear_pos",
            min_overlap=0.56,
            name="collapsed carriage has long retained runner engagement",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem="runner_pos",
            negative_elem="bottom_wear_pos",
            max_gap=0.0005,
            max_penetration=0.0,
            name="runner bears on bottom wear strip without clipping",
        )
        ctx.expect_gap(
            base,
            carriage,
            axis="y",
            positive_elem="side_wear_pos",
            negative_elem="side_bearing_pos",
            min_gap=0.001,
            max_gap=0.006,
            name="positive side guide clearance is machined",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="y",
            positive_elem="side_bearing_neg",
            negative_elem="side_wear_neg",
            min_gap=0.001,
            max_gap=0.006,
            name="negative side guide clearance is machined",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="x",
            positive_elem="front_stop",
            negative_elem="front_lug_pos",
            min_gap=0.015,
            max_gap=0.040,
            name="front stop clears fixed nose lug at rest",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.260}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="runner_pos",
            elem_b="bottom_wear_pos",
            min_overlap=0.30,
            name="extended carriage remains deeply supported",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem="runner_pos",
            negative_elem="bottom_wear_pos",
            max_gap=0.0005,
            max_penetration=0.0,
            name="extended runner stays seated on bottom strip",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates outward along channel",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.24,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
