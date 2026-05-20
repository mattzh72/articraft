from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_yz_positioner")

    aluminum = model.material("clear_anodized_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    rail_steel = model.material("polished_hardened_steel", rgba=(0.86, 0.88, 0.86, 1.0))
    blue = model.material("blue_anodized_fixture", rgba=(0.05, 0.18, 0.48, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.54, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    for x, pad_name, rail_name in (
        (-0.13, "y_rail_pad_0", "y_rail_0"),
        (0.13, "y_rail_pad_1", "y_rail_1"),
    ):
        base.visual(
            Box((0.034, 0.46, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.041)),
            material=aluminum,
            name=pad_name,
        )
        base.visual(
            Cylinder(radius=0.009, length=0.46),
            origin=Origin(xyz=(x, 0.0, 0.056), rpy=(pi / 2, 0.0, 0.0)),
            material=rail_steel,
            name=rail_name,
        )

    base.visual(
        Cylinder(radius=0.005, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.059), rpy=(pi / 2, 0.0, 0.0)),
        material=rail_steel,
        name="y_lead_screw",
    )
    for y in (-0.225, 0.225):
        base.visual(
            Box((0.056, 0.030, 0.038)),
            origin=Origin(xyz=(0.0, y, 0.054)),
            material=aluminum,
            name=f"y_screw_support_{0 if y < 0 else 1}",
        )
    base.visual(
        Box((0.090, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, -0.245, 0.065)),
        material=rubber,
        name="stepper_motor",
    )
    base.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(0.0, -0.207, 0.059), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="motor_coupler",
    )
    for x in (-0.19, 0.19):
        for y in (-0.20, 0.20):
            base.visual(
                Cylinder(radius=0.007, length=0.004),
                origin=Origin(xyz=(x, y, 0.037), rpy=(0.0, 0.0, 0.0)),
                material=aluminum,
                name=f"base_bolt_{'n' if y > 0 else 's'}_{'p' if x > 0 else 'm'}",
            )

    y_stage = model.part("y_stage")
    for x, bearing_name in ((-0.13, "y_bearing_0"), (0.13, "y_bearing_1")):
        y_stage.visual(
            Box((0.052, 0.120, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.077)),
            material=aluminum,
            name=bearing_name,
        )
    y_stage.visual(
        Box((0.340, 0.160, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=aluminum,
        name="saddle_plate",
    )
    y_stage.visual(
        Box((0.058, 0.070, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0765)),
        material=aluminum,
        name="y_drive_nut",
    )
    y_stage.visual(
        Box((0.260, 0.035, 0.420)),
        origin=Origin(xyz=(0.0, 0.050, 0.317)),
        material=aluminum,
        name="upright_plate",
    )
    y_stage.visual(
        Box((0.300, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, 0.075, 0.125)),
        material=aluminum,
        name="lower_z_cap",
    )
    y_stage.visual(
        Box((0.300, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, 0.075, 0.509)),
        material=aluminum,
        name="upper_z_cap",
    )
    for x, rail_name in ((-0.080, "z_rail_0"), (0.080, "z_rail_1")):
        y_stage.visual(
            Cylinder(radius=0.007, length=0.420),
            origin=Origin(xyz=(x, 0.0735, 0.317), rpy=(0.0, 0.0, 0.0)),
            material=rail_steel,
            name=rail_name,
        )
    y_stage.visual(
        Cylinder(radius=0.0045, length=0.390),
        origin=Origin(xyz=(0.0, 0.0735, 0.317), rpy=(0.0, 0.0, 0.0)),
        material=rail_steel,
        name="z_lead_screw",
    )
    for x in (-0.135, 0.135):
        for y in (-0.055, 0.055):
            y_stage.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, 0.109), rpy=(0.0, 0.0, 0.0)),
                material=dark_steel,
                name=f"saddle_bolt_{'p' if x > 0 else 'm'}_{'p' if y > 0 else 'm'}",
            )

    z_carriage = model.part("z_carriage")
    for x, bearing_pairs in (
        (-0.080, ((0.060, "z_bearing_0_low"), (0.150, "z_bearing_0_high"))),
        (0.080, ((0.060, "z_bearing_1_low"), (0.150, "z_bearing_1_high"))),
    ):
        for z, bearing_name in bearing_pairs:
            z_carriage.visual(
                Box((0.042, 0.024, 0.055)),
                origin=Origin(xyz=(x, 0.0925, z)),
                material=aluminum,
                name=bearing_name,
            )
    z_carriage.visual(
        Box((0.230, 0.012, 0.190)),
        origin=Origin(xyz=(0.0, 0.1105, 0.105)),
        material=blue,
        name="tool_backplate",
    )
    z_carriage.visual(
        Box((0.125, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, 0.1255, 0.052)),
        material=blue,
        name="platform_neck",
    )
    z_carriage.visual(
        Box((0.165, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, 0.1665, 0.040)),
        material=blue,
        name="tool_platform",
    )
    z_carriage.visual(
        Box((0.150, 0.010, 0.035)),
        origin=Origin(xyz=(0.0, 0.2145, 0.058)),
        material=blue,
        name="platform_lip",
    )
    for x in (-0.070, 0.070):
        for z in (0.045, 0.165):
            z_carriage.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, 0.1185, z), rpy=(pi / 2, 0.0, 0.0)),
                material=dark_steel,
                name=f"tool_bolt_{'p' if x > 0 else 'm'}_{'h' if z > 0.1 else 'l'}",
            )

    model.articulation(
        "y_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=-0.09, upper=0.13),
        motion_properties=MotionProperties(damping=4.0, friction=0.4),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=y_stage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.25, lower=0.0, upper=0.18),
        motion_properties=MotionProperties(damping=5.0, friction=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    y_slide = object_model.get_articulation("y_slide")
    z_slide = object_model.get_articulation("z_slide")
    base = object_model.get_part("base")
    y_stage = object_model.get_part("y_stage")
    z_carriage = object_model.get_part("z_carriage")

    ctx.check(
        "stacked prismatic y and z axes",
        y_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.axis == (0.0, 1.0, 0.0)
        and z_slide.axis == (0.0, 0.0, 1.0),
        details=f"y={y_slide.articulation_type}/{y_slide.axis}, z={z_slide.articulation_type}/{z_slide.axis}",
    )
    ctx.expect_gap(
        y_stage,
        base,
        axis="z",
        positive_elem="y_bearing_0",
        negative_elem="y_rail_0",
        max_gap=0.002,
        max_penetration=0.0005,
        name="y bearing rides on rail",
    )
    ctx.expect_gap(
        z_carriage,
        y_stage,
        axis="y",
        positive_elem="z_bearing_0_low",
        negative_elem="z_rail_0",
        max_gap=0.002,
        max_penetration=0.0005,
        name="z bearing rides on rail",
    )

    y_rest = ctx.part_world_position(y_stage)
    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        ctx.expect_overlap(
            y_stage,
            base,
            axes="y",
            elem_a="y_bearing_0",
            elem_b="y_rail_0",
            min_overlap=0.09,
            name="y slide remains captured at travel",
        )
        y_extended = ctx.part_world_position(y_stage)
    ctx.check(
        "y stage moves along y",
        y_rest is not None and y_extended is not None and y_extended[1] > y_rest[1] + 0.12,
        details=f"rest={y_rest}, extended={y_extended}",
    )

    z_rest = ctx.part_world_position(z_carriage)
    with ctx.pose({z_slide: z_slide.motion_limits.upper}):
        ctx.expect_overlap(
            z_carriage,
            y_stage,
            axes="z",
            elem_a="z_bearing_0_high",
            elem_b="z_rail_0",
            min_overlap=0.045,
            name="z slide remains captured at travel",
        )
        z_extended = ctx.part_world_position(z_carriage)
    ctx.check(
        "tool platform moves upward",
        z_rest is not None and z_extended is not None and z_extended[2] > z_rest[2] + 0.16,
        details=f"rest={z_rest}, extended={z_extended}",
    )

    return ctx.report()


object_model = build_object_model()
