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
    model = ArticulatedObject(name="bench_positioning_axis")

    dark = model.material("black_anodized", rgba=(0.02, 0.025, 0.028, 1.0))
    rail_steel = model.material("polished_steel", rgba=(0.74, 0.76, 0.75, 1.0))
    blue = model.material("blue_carriage", rgba=(0.08, 0.22, 0.52, 1.0))
    aluminum = model.material("machined_aluminum", rgba=(0.62, 0.64, 0.61, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.01, 0.01, 0.01, 1.0))

    rail_bed = model.part("rail_bed")
    rail_bed.visual(
        Box((1.28, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark,
        name="base_plate",
    )
    for i, x in enumerate((-0.56, 0.56)):
        rail_bed.visual(
            Box((0.09, 0.24, 0.095)),
            origin=Origin(xyz=(x, 0.0, 0.075)),
            material=dark,
            name=f"end_support_{i}",
        )
    for i, y in enumerate((-0.085, 0.085)):
        rail_bed.visual(
            Cylinder(radius=0.016, length=1.18),
            origin=Origin(xyz=(0.0, y, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
            material=rail_steel,
            name=f"rail_{i}",
        )
    rail_bed.visual(
        Cylinder(radius=0.007, length=1.08),
        origin=Origin(xyz=(0.0, 0.0, 0.068), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_steel,
        name="drive_screw",
    )
    for i, x in enumerate((-0.36, 0.36)):
        rail_bed.visual(
            Box((0.08, 0.22, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.041)),
            material=aluminum,
            name=f"tie_bar_{i}",
        )
    for i, (x, y) in enumerate(
        ((-0.54, -0.11), (-0.54, 0.11), (0.54, -0.11), (0.54, 0.11))
    ):
        rail_bed.visual(
            Box((0.08, 0.035, 0.012)),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"foot_{i}",
        )

    base_carriage = model.part("base_carriage")
    base_carriage.visual(
        Box((0.24, 0.21, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=blue,
        name="saddle_plate",
    )
    for i, (x, y) in enumerate(
        ((-0.065, -0.085), (0.065, -0.085), (-0.065, 0.085), (0.065, 0.085))
    ):
        base_carriage.visual(
            Box((0.075, 0.04, 0.036)),
            origin=Origin(xyz=(x, y, 0.018)),
            material=aluminum,
            name=f"bearing_shoe_{i}",
        )
    base_carriage.visual(
        Box((0.15, 0.13, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=blue,
        name="guide_foot",
    )
    base_carriage.visual(
        Box((0.052, 0.05, 0.41)),
        origin=Origin(xyz=(0.0, 0.028, 0.295)),
        material=dark,
        name="upright_spine",
    )
    for i, x in enumerate((-0.032, 0.032)):
        base_carriage.visual(
            Cylinder(radius=0.006, length=0.39),
            origin=Origin(xyz=(x, -0.012, 0.295)),
            material=rail_steel,
            name=f"front_guide_rod_{i}",
        )
    base_carriage.visual(
        Box((0.11, 0.065, 0.025)),
        origin=Origin(xyz=(0.0, 0.008, 0.095)),
        material=dark,
        name="lower_guide_cap",
    )
    base_carriage.visual(
        Box((0.11, 0.065, 0.025)),
        origin=Origin(xyz=(0.0, 0.008, 0.495)),
        material=dark,
        name="upper_guide_cap",
    )
    for i, x in enumerate((-0.053, 0.053)):
        base_carriage.visual(
            Box((0.024, 0.055, 0.18)),
            origin=Origin(xyz=(0.035 if x > 0 else -0.035, 0.012, 0.235)),
            material=dark,
            name=f"side_gusset_{i}",
        )

    upright_carriage = model.part("upright_carriage")
    upright_carriage.visual(
        Box((0.115, 0.024, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=aluminum,
        name="slide_block",
    )
    upright_carriage.visual(
        Box((0.06, 0.05, 0.045)),
        origin=Origin(xyz=(0.0, -0.012, 0.145)),
        material=aluminum,
        name="neck_block",
    )
    upright_carriage.visual(
        Box((0.18, 0.10, 0.018)),
        origin=Origin(xyz=(0.0, -0.045, 0.176)),
        material=blue,
        name="top_plate",
    )
    upright_carriage.visual(
        Box((0.14, 0.015, 0.045)),
        origin=Origin(xyz=(0.0, -0.075, 0.152)),
        material=blue,
        name="front_lip",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_bed,
        child=base_carriage,
        origin=Origin(xyz=(-0.32, 0.0, 0.131)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.64),
    )
    model.articulation(
        "carriage_to_lift",
        ArticulationType.PRISMATIC,
        parent=base_carriage,
        child=upright_carriage,
        origin=Origin(xyz=(0.0, -0.035, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.24),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_bed = object_model.get_part("rail_bed")
    base_carriage = object_model.get_part("base_carriage")
    upright_carriage = object_model.get_part("upright_carriage")
    horizontal = object_model.get_articulation("rail_to_carriage")
    vertical = object_model.get_articulation("carriage_to_lift")

    ctx.check(
        "base carriage uses horizontal prismatic travel",
        horizontal.articulation_type == ArticulationType.PRISMATIC
        and tuple(horizontal.axis) == (1.0, 0.0, 0.0)
        and horizontal.motion_limits.lower == 0.0
        and horizontal.motion_limits.upper >= 0.6,
        details=f"type={horizontal.articulation_type}, axis={horizontal.axis}, limits={horizontal.motion_limits}",
    )
    ctx.check(
        "upright carriage uses vertical prismatic travel",
        vertical.articulation_type == ArticulationType.PRISMATIC
        and tuple(vertical.axis) == (0.0, 0.0, 1.0)
        and vertical.motion_limits.lower == 0.0
        and vertical.motion_limits.upper >= 0.22,
        details=f"type={vertical.articulation_type}, axis={vertical.axis}, limits={vertical.motion_limits}",
    )

    ctx.expect_gap(
        base_carriage,
        rail_bed,
        axis="z",
        positive_elem="bearing_shoe_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=1e-5,
        name="bearing shoe sits on low rail",
    )
    ctx.expect_overlap(
        base_carriage,
        rail_bed,
        axes="xy",
        elem_a="saddle_plate",
        elem_b="base_plate",
        min_overlap=0.18,
        name="base carriage footprint stays over rail bed",
    )
    ctx.expect_gap(
        base_carriage,
        upright_carriage,
        axis="y",
        positive_elem="front_guide_rod_0",
        negative_elem="slide_block",
        min_gap=0.001,
        max_gap=0.01,
        name="lift slide is clearanced in front of guide",
    )
    ctx.expect_overlap(
        upright_carriage,
        base_carriage,
        axes="z",
        elem_a="slide_block",
        elem_b="front_guide_rod_0",
        min_overlap=0.11,
        name="lift block remains engaged in guide at rest",
    )

    rest_base = ctx.part_world_position(base_carriage)
    rest_lift = ctx.part_world_position(upright_carriage)
    with ctx.pose({horizontal: horizontal.motion_limits.upper, vertical: vertical.motion_limits.upper}):
        ctx.expect_overlap(
            upright_carriage,
            base_carriage,
            axes="z",
            elem_a="slide_block",
            elem_b="front_guide_rod_0",
            min_overlap=0.11,
            name="lift block remains engaged at full lift",
        )
        extended_base = ctx.part_world_position(base_carriage)
        extended_lift = ctx.part_world_position(upright_carriage)

    ctx.check(
        "horizontal carriage advances along rail",
        rest_base is not None
        and extended_base is not None
        and extended_base[0] > rest_base[0] + 0.6,
        details=f"rest={rest_base}, extended={extended_base}",
    )
    ctx.check(
        "upright carriage lifts vertically",
        rest_lift is not None
        and extended_lift is not None
        and extended_lift[2] > rest_lift[2] + 0.22,
        details=f"rest={rest_lift}, extended={extended_lift}",
    )

    return ctx.report()


object_model = build_object_model()
