from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="side_mounted_yz_stage")

    dark_anodized = Material("dark_anodized", color=(0.05, 0.055, 0.060, 1.0))
    blue_anodized = Material("blue_carriage", color=(0.10, 0.20, 0.34, 1.0))
    satin_steel = Material("satin_steel", color=(0.72, 0.72, 0.68, 1.0))
    black = Material("black_oxide", color=(0.015, 0.015, 0.018, 1.0))
    brass = Material("brass_bearing", color=(0.72, 0.52, 0.22, 1.0))
    red = Material("red_index", color=(0.85, 0.05, 0.04, 1.0))

    fixed = model.part("fixed_frame")
    fixed.visual(
        Box((0.240, 0.030, 0.620)),
        origin=Origin(xyz=(0.0, -0.050, 0.310)),
        material=dark_anodized,
        name="mount_plate",
    )
    fixed.visual(
        Box((0.260, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, -0.008, 0.025)),
        material=dark_anodized,
        name="lower_cap",
    )
    fixed.visual(
        Box((0.260, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, -0.008, 0.595)),
        material=dark_anodized,
        name="upper_cap",
    )
    fixed.visual(
        Cylinder(radius=0.011, length=0.540),
        origin=Origin(xyz=(-0.075, 0.0, 0.310)),
        material=satin_steel,
        name="rail_0",
    )
    fixed.visual(
        Cylinder(radius=0.011, length=0.540),
        origin=Origin(xyz=(0.075, 0.0, 0.310)),
        material=satin_steel,
        name="rail_1",
    )
    fixed.visual(
        Cylinder(radius=0.006, length=0.550),
        origin=Origin(xyz=(0.0, 0.001, 0.310)),
        material=satin_steel,
        name="lead_screw",
    )
    for x in (-0.095, 0.095):
        for z in (0.115, 0.505):
            fixed.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, -0.034, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=black,
                name=f"mount_screw_{x}_{z}",
            )

    carriage = model.part("vertical_carriage")
    carriage.visual(
        Box((0.180, 0.035, 0.140)),
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
        material=blue_anodized,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.180, 0.018, 0.130)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=blue_anodized,
        name="bearing_web",
    )
    carriage.visual(
        Box((0.045, 0.052, 0.130)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=brass,
        name="bearing_0",
    )
    carriage.visual(
        Box((0.045, 0.052, 0.130)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=brass,
        name="bearing_1",
    )
    carriage.visual(
        Box((0.150, 0.090, 0.030)),
        origin=Origin(xyz=(0.0, 0.095, 0.065)),
        material=blue_anodized,
        name="side_saddle",
    )
    carriage.visual(
        Box((0.032, 0.130, 0.040)),
        origin=Origin(xyz=(-0.035, 0.150, 0.095)),
        material=brass,
        name="y_bearing_0",
    )
    carriage.visual(
        Box((0.032, 0.130, 0.040)),
        origin=Origin(xyz=(0.035, 0.150, 0.095)),
        material=brass,
        name="y_bearing_1",
    )
    carriage.visual(
        Box((0.160, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.064, -0.050)),
        material=red,
        name="height_index",
    )

    side_slide = model.part("side_slide")
    side_slide.visual(
        Cylinder(radius=0.0075, length=0.350),
        origin=Origin(xyz=(-0.035, 0.085, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="y_rail_0",
    )
    side_slide.visual(
        Cylinder(radius=0.0075, length=0.350),
        origin=Origin(xyz=(0.035, 0.085, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="y_rail_1",
    )
    side_slide.visual(
        Box((0.110, 0.038, 0.060)),
        origin=Origin(xyz=(0.0, 0.262, -0.006)),
        material=blue_anodized,
        name="end_block",
    )
    side_slide.visual(
        Box((0.140, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.320, -0.045)),
        material=dark_anodized,
        name="end_platform",
    )
    for x in (-0.045, 0.045):
        for y in (0.295, 0.345):
            side_slide.visual(
                Cylinder(radius=0.007, length=0.004),
                origin=Origin(xyz=(x, y, -0.034)),
                material=black,
                name=f"platform_screw_{x}_{y}",
            )

    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.300),
    )
    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=side_slide,
        origin=Origin(xyz=(0.0, 0.150, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_frame")
    carriage = object_model.get_part("vertical_carriage")
    side_slide = object_model.get_part("side_slide")
    z_slide = object_model.get_articulation("z_slide")
    y_slide = object_model.get_articulation("y_slide")

    ctx.allow_overlap(
        fixed,
        carriage,
        elem_a="rail_0",
        elem_b="bearing_0",
        reason="The vertical bearing block is a solid visual proxy for a captured linear bearing around the fixed guide rail.",
    )
    ctx.allow_overlap(
        fixed,
        carriage,
        elem_a="rail_1",
        elem_b="bearing_1",
        reason="The vertical bearing block is a solid visual proxy for a captured linear bearing around the fixed guide rail.",
    )
    ctx.allow_overlap(
        carriage,
        side_slide,
        elem_a="y_bearing_0",
        elem_b="y_rail_0",
        reason="The side bearing is a solid visual proxy for a sliding sleeve around the horizontal Y rail.",
    )
    ctx.allow_overlap(
        carriage,
        side_slide,
        elem_a="y_bearing_1",
        elem_b="y_rail_1",
        reason="The side bearing is a solid visual proxy for a sliding sleeve around the horizontal Y rail.",
    )

    ctx.check(
        "two prismatic stages",
        z_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"z={z_slide.articulation_type}, y={y_slide.articulation_type}",
    )
    ctx.check(
        "z stage axis is vertical",
        tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={z_slide.axis}",
    )
    ctx.check(
        "y stage axis is sideways",
        tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"axis={y_slide.axis}",
    )

    for rail, bearing in (("rail_0", "bearing_0"), ("rail_1", "bearing_1")):
        ctx.expect_within(
            fixed,
            carriage,
            axes="xy",
            inner_elem=rail,
            outer_elem=bearing,
            margin=0.0,
            name=f"{bearing} captures {rail} laterally",
        )
        ctx.expect_overlap(
            fixed,
            carriage,
            axes="z",
            elem_a=rail,
            elem_b=bearing,
            min_overlap=0.10,
            name=f"{bearing} remains engaged on {rail}",
        )

    for rail, bearing in (("y_rail_0", "y_bearing_0"), ("y_rail_1", "y_bearing_1")):
        ctx.expect_within(
            side_slide,
            carriage,
            axes="xz",
            inner_elem=rail,
            outer_elem=bearing,
            margin=0.0,
            name=f"{bearing} captures {rail} laterally",
        )
        ctx.expect_overlap(
            side_slide,
            carriage,
            axes="y",
            elem_a=rail,
            elem_b=bearing,
            min_overlap=0.09,
            name=f"{rail} is retained in {bearing}",
        )

    rest_carriage = ctx.part_world_position(carriage)
    rest_side = ctx.part_world_position(side_slide)
    with ctx.pose({z_slide: 0.300, y_slide: 0.120}):
        lifted_carriage = ctx.part_world_position(carriage)
        extended_side = ctx.part_world_position(side_slide)
        for rail, bearing in (("rail_0", "bearing_0"), ("rail_1", "bearing_1")):
            ctx.expect_overlap(
                fixed,
                carriage,
                axes="z",
                elem_a=rail,
                elem_b=bearing,
                min_overlap=0.10,
                name=f"{bearing} retains {rail} at full height",
            )
        for rail, bearing in (("y_rail_0", "y_bearing_0"), ("y_rail_1", "y_bearing_1")):
            ctx.expect_overlap(
                side_slide,
                carriage,
                axes="y",
                elem_a=rail,
                elem_b=bearing,
                min_overlap=0.025,
                name=f"{rail} remains inserted at full side travel",
            )

    ctx.check(
        "carriage moves upward on z_slide",
        rest_carriage is not None
        and lifted_carriage is not None
        and lifted_carriage[2] > rest_carriage[2] + 0.25,
        details=f"rest={rest_carriage}, lifted={lifted_carriage}",
    )
    ctx.check(
        "platform slide moves along +Y",
        rest_side is not None
        and extended_side is not None
        and extended_side[1] > rest_side[1] + 0.10,
        details=f"rest={rest_side}, extended={extended_side}",
    )

    return ctx.report()


object_model = build_object_model()
