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
    model = ArticulatedObject(name="compact_service_xyz_stage")

    cast = model.material("charcoal_cast_iron", rgba=(0.12, 0.15, 0.17, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.66, 0.70, 0.72, 1.0))
    rail_steel = model.material("hardened_rail_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    screw_steel = model.material("polished_lead_screw", rgba=(0.62, 0.64, 0.64, 1.0))
    bronze = model.material("bronze_gibs", rgba=(0.70, 0.46, 0.18, 1.0))
    dark = model.material("blackened_fasteners", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.70, 0.42, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=cast,
        name="base_plate",
    )
    for x in (-0.275, 0.275):
        for y in (-0.155, 0.155):
            base.visual(
                Box((0.105, 0.070, 0.020)),
                origin=Origin(xyz=(x, y, 0.010)),
                material=dark,
                name=f"leveling_foot_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    for i, y in enumerate((-0.120, 0.120)):
        base.visual(
            Box((0.600, 0.034, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.086)),
            material=rail_steel,
            name=f"x_rail_{i}",
        )

    base.visual(
        Cylinder(radius=0.009, length=0.650),
        origin=Origin(xyz=(0.0, 0.0, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_steel,
        name="x_lead_screw",
    )
    for i, x in enumerate((-0.325, 0.325)):
        base.visual(
            Box((0.030, 0.070, 0.044)),
            origin=Origin(xyz=(x, 0.0, 0.092)),
            material=cast,
            name=f"x_bearing_block_{i}",
        )

    lower = model.part("lower_carriage")
    for i, y in enumerate((-0.120, 0.120)):
        lower.visual(
            Box((0.300, 0.052, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.010)),
            material=rail_steel,
            name=f"x_bearing_{i}",
        )
    lower.visual(
        Box((0.380, 0.290, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=aluminum,
        name="lower_plate",
    )
    for i, x in enumerate((-0.105, 0.105)):
        lower.visual(
            Box((0.032, 0.255, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.069)),
            material=rail_steel,
            name=f"y_rail_{i}",
        )
    lower.visual(
        Cylinder(radius=0.007, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.069), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=screw_steel,
        name="y_lead_screw",
    )
    for i, y in enumerate((-0.135, 0.135)):
        lower.visual(
            Box((0.060, 0.024, 0.038)),
            origin=Origin(xyz=(0.0, y, 0.075)),
            material=aluminum,
            name=f"y_bearing_block_{i}",
        )

    cross = model.part("cross_slide")
    for i, x in enumerate((-0.105, 0.105)):
        cross.visual(
            Box((0.052, 0.120, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.009)),
            material=rail_steel,
            name=f"y_bearing_{i}",
        )
    cross.visual(
        Box((0.300, 0.220, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=aluminum,
        name="cross_plate",
    )
    for i, x in enumerate((-0.062, 0.062)):
        cross.visual(
            Box((0.025, 0.130, 0.220)),
            origin=Origin(xyz=(x, -0.015, 0.160)),
            material=aluminum,
            name=f"guide_column_{i}",
        )
    cross.visual(
        Box((0.155, 0.026, 0.025)),
        origin=Origin(xyz=(0.0, 0.040, 0.2825)),
        material=aluminum,
        name="rear_bridge",
    )
    for i, x in enumerate((-0.0465, 0.0465)):
        cross.visual(
            Box((0.006, 0.050, 0.190)),
            origin=Origin(xyz=(x, -0.020, 0.155)),
            material=bronze,
            name=f"gib_strip_{i}",
        )

    ram = model.part("ram")
    ram.visual(
        Box((0.080, 0.050, 0.270)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=rail_steel,
        name="ram_bar",
    )
    ram.visual(
        Box((0.100, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, -0.034, 0.050)),
        material=dark,
        name="front_tool_plate",
    )
    ram.visual(
        Box((0.090, 0.060, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.2825)),
        material=dark,
        name="ram_cap",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.18, lower=-0.10, upper=0.10),
    )
    model.articulation(
        "lower_to_cross",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=cross,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.16, lower=-0.06, upper=0.06),
    )
    model.articulation(
        "cross_to_ram",
        ArticulationType.PRISMATIC,
        parent=cross,
        child=ram,
        origin=Origin(xyz=(0.0, -0.025, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.12, lower=0.0, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_carriage")
    cross = object_model.get_part("cross_slide")
    ram = object_model.get_part("ram")
    x_slide = object_model.get_articulation("base_to_lower")
    y_slide = object_model.get_articulation("lower_to_cross")
    z_slide = object_model.get_articulation("cross_to_ram")

    for joint in (x_slide, y_slide, z_slide):
        ctx.check(
            f"{joint.name} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"{joint.name} type={joint.articulation_type}",
        )

    for i in (0, 1):
        ctx.expect_contact(
            lower,
            base,
            elem_a=f"x_bearing_{i}",
            elem_b=f"x_rail_{i}",
            name=f"lower bearing {i} seats on base rail",
        )
        ctx.expect_contact(
            cross,
            lower,
            elem_a=f"y_bearing_{i}",
            elem_b=f"y_rail_{i}",
            name=f"cross bearing {i} seats on lower rail",
        )

    rest_lower = ctx.part_world_position(lower)
    with ctx.pose({x_slide: x_slide.motion_limits.upper}):
        extended_lower = ctx.part_world_position(lower)
        for i in (0, 1):
            ctx.expect_within(
                lower,
                base,
                axes="x",
                inner_elem=f"x_bearing_{i}",
                outer_elem=f"x_rail_{i}",
                margin=0.001,
                name=f"x bearing {i} remains on rail at full travel",
            )
    ctx.check(
        "lower carriage travels along +X",
        rest_lower is not None
        and extended_lower is not None
        and extended_lower[0] > rest_lower[0] + 0.08,
        details=f"rest={rest_lower}, extended={extended_lower}",
    )

    rest_cross = ctx.part_world_position(cross)
    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        extended_cross = ctx.part_world_position(cross)
        for i in (0, 1):
            ctx.expect_within(
                cross,
                lower,
                axes="y",
                inner_elem=f"y_bearing_{i}",
                outer_elem=f"y_rail_{i}",
                margin=0.001,
                name=f"y bearing {i} remains on rail at full travel",
            )
    ctx.check(
        "cross slide travels along +Y",
        rest_cross is not None
        and extended_cross is not None
        and extended_cross[1] > rest_cross[1] + 0.05,
        details=f"rest={rest_cross}, extended={extended_cross}",
    )

    rest_ram = ctx.part_world_position(ram)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        extended_ram = ctx.part_world_position(ram)
        ctx.expect_overlap(
            ram,
            cross,
            axes="z",
            elem_a="ram_bar",
            elem_b="guide_column_0",
            min_overlap=0.080,
            name="ram remains inserted in vertical guide at full lift",
        )
    ctx.check(
        "ram travels along +Z",
        rest_ram is not None
        and extended_ram is not None
        and extended_ram[2] > rest_ram[2] + 0.08,
        details=f"rest={rest_ram}, extended={extended_ram}",
    )

    return ctx.report()


object_model = build_object_model()
