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
    model = ArticulatedObject(name="compact_service_yz_stage")

    dark_anodized = Material("dark_anodized", color=(0.08, 0.085, 0.09, 1.0))
    black = Material("black_hardcoat", color=(0.015, 0.015, 0.018, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.72, 0.72, 0.68, 1.0))
    red_carriage = Material("red_carriage", color=(0.78, 0.08, 0.04, 1.0))
    blue_carriage = Material("blue_carriage", color=(0.06, 0.18, 0.72, 1.0))
    rubber = Material("rubber", color=(0.01, 0.01, 0.01, 1.0))

    fixed_back = model.part("fixed_back")
    fixed_back.visual(
        Box((0.035, 0.46, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=dark_anodized,
        name="back_plate",
    )
    fixed_back.visual(
        Box((0.080, 0.49, 0.052)),
        origin=Origin(xyz=(0.022, 0.0, 0.026)),
        material=black,
        name="base_foot",
    )
    fixed_back.visual(
        Box((0.060, 0.49, 0.034)),
        origin=Origin(xyz=(0.012, 0.0, 0.545)),
        material=black,
        name="top_cap",
    )
    # Two fixed guide rods define the short lateral Y slide on the front face.
    fixed_back.visual(
        Cylinder(radius=0.012, length=0.365),
        origin=Origin(xyz=(0.025, 0.0, 0.300), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="y_guide_rod_0",
    )
    fixed_back.visual(
        Cylinder(radius=0.012, length=0.365),
        origin=Origin(xyz=(0.025, 0.0, 0.380), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="y_guide_rod_1",
    )
    for i, y in enumerate((-0.205, 0.205)):
        fixed_back.visual(
            Box((0.042, 0.030, 0.130)),
            origin=Origin(xyz=(0.024, y, 0.340)),
            material=black,
            name=f"y_end_stop_{i}",
        )
    fixed_back.visual(
        Cylinder(radius=0.006, length=0.345),
        origin=Origin(xyz=(0.032, 0.0, 0.445), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="y_lead_screw",
    )
    for i, y in enumerate((-0.165, 0.165)):
        fixed_back.visual(
            Box((0.030, 0.026, 0.030)),
            origin=Origin(xyz=(0.022, y, 0.445)),
            material=black,
            name=f"screw_bearing_{i}",
        )
    fixed_back.visual(
        Box((0.070, 0.055, 0.070)),
        origin=Origin(xyz=(0.030, -0.242, 0.445)),
        material=black,
        name="side_motor",
    )

    lateral_slide = model.part("lateral_slide")
    lateral_slide.visual(
        Box((0.026, 0.155, 0.145)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=red_carriage,
        name="saddle_plate",
    )
    lateral_slide.visual(
        Box((0.018, 0.112, 0.026)),
        origin=Origin(xyz=(0.003, 0.0, -0.040)),
        material=brushed_steel,
        name="y_bearing_shoe_0",
    )
    lateral_slide.visual(
        Box((0.018, 0.112, 0.026)),
        origin=Origin(xyz=(0.003, 0.0, 0.040)),
        material=brushed_steel,
        name="y_bearing_shoe_1",
    )
    lateral_slide.visual(
        Box((0.035, 0.058, 0.085)),
        origin=Origin(xyz=(0.043, 0.0, -0.040)),
        material=red_carriage,
        name="front_bridge",
    )
    lateral_slide.visual(
        Box((0.018, 0.084, 0.255)),
        origin=Origin(xyz=(0.056, 0.0, -0.155)),
        material=black,
        name="z_rail_backbone",
    )
    lateral_slide.visual(
        Box((0.012, 0.010, 0.235)),
        origin=Origin(xyz=(0.070, -0.027, -0.155)),
        material=brushed_steel,
        name="z_guide_rail_0",
    )
    lateral_slide.visual(
        Box((0.012, 0.010, 0.235)),
        origin=Origin(xyz=(0.070, 0.027, -0.155)),
        material=brushed_steel,
        name="z_guide_rail_1",
    )
    lateral_slide.visual(
        Box((0.035, 0.104, 0.025)),
        origin=Origin(xyz=(0.058, 0.0, -0.030)),
        material=black,
        name="z_top_stop",
    )
    lateral_slide.visual(
        Box((0.035, 0.104, 0.025)),
        origin=Origin(xyz=(0.058, 0.0, -0.278)),
        material=black,
        name="z_bottom_stop",
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.028, 0.095, 0.120)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=blue_carriage,
        name="carriage_block",
    )
    z_carriage.visual(
        Box((0.035, 0.056, 0.090)),
        origin=Origin(xyz=(0.025, 0.0, -0.095)),
        material=blue_carriage,
        name="tool_plate",
    )
    for i, (y, z) in enumerate(((-0.030, 0.033), (0.030, 0.033), (-0.030, -0.033), (0.030, -0.033))):
        z_carriage.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.021, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"front_screw_{i}",
        )

    model.articulation(
        "fixed_to_lateral",
        ArticulationType.PRISMATIC,
        parent=fixed_back,
        child=lateral_slide,
        origin=Origin(xyz=(0.043, -0.060, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    model.articulation(
        "lateral_to_z",
        ArticulationType.PRISMATIC,
        parent=lateral_slide,
        child=z_carriage,
        origin=Origin(xyz=(0.085, 0.0, -0.150)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.20, lower=0.0, upper=0.140),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_back = object_model.get_part("fixed_back")
    lateral_slide = object_model.get_part("lateral_slide")
    z_carriage = object_model.get_part("z_carriage")
    y_joint = object_model.get_articulation("fixed_to_lateral")
    z_joint = object_model.get_articulation("lateral_to_z")

    ctx.expect_contact(
        lateral_slide,
        fixed_back,
        elem_a="y_bearing_shoe_0",
        elem_b="y_guide_rod_0",
        contact_tol=0.0005,
        name="lower Y bearing shoe rides on fixed guide rod",
    )
    ctx.expect_contact(
        lateral_slide,
        fixed_back,
        elem_a="y_bearing_shoe_1",
        elem_b="y_guide_rod_1",
        contact_tol=0.0005,
        name="upper Y bearing shoe rides on fixed guide rod",
    )
    ctx.expect_contact(
        z_carriage,
        lateral_slide,
        elem_a="carriage_block",
        elem_b="z_guide_rail_0",
        contact_tol=0.0005,
        name="Z carriage bears on first vertical guide",
    )
    ctx.expect_contact(
        z_carriage,
        lateral_slide,
        elem_a="carriage_block",
        elem_b="z_guide_rail_1",
        contact_tol=0.0005,
        name="Z carriage bears on second vertical guide",
    )

    lateral_rest = ctx.part_world_position(lateral_slide)
    with ctx.pose({y_joint: 0.120}):
        lateral_extended = ctx.part_world_position(lateral_slide)
        ctx.expect_contact(
            lateral_slide,
            fixed_back,
            elem_a="y_bearing_shoe_0",
            elem_b="y_guide_rod_0",
            contact_tol=0.0005,
            name="Y slide remains captured at lateral travel limit",
        )
    ctx.check(
        "lateral joint moves only along Y",
        lateral_rest is not None
        and lateral_extended is not None
        and lateral_extended[1] > lateral_rest[1] + 0.110
        and abs(lateral_extended[0] - lateral_rest[0]) < 1e-6
        and abs(lateral_extended[2] - lateral_rest[2]) < 1e-6,
        details=f"rest={lateral_rest}, extended={lateral_extended}",
    )

    z_rest = ctx.part_world_position(z_carriage)
    with ctx.pose({z_joint: 0.140}):
        z_lowered = ctx.part_world_position(z_carriage)
        ctx.expect_overlap(
            z_carriage,
            lateral_slide,
            axes="z",
            elem_a="carriage_block",
            elem_b="z_guide_rail_0",
            min_overlap=0.030,
            name="lowered Z carriage remains engaged on vertical guide",
        )
    ctx.check(
        "vertical joint moves only along Z",
        z_rest is not None
        and z_lowered is not None
        and z_lowered[2] < z_rest[2] - 0.130
        and abs(z_lowered[0] - z_rest[0]) < 1e-6
        and abs(z_lowered[1] - z_rest[1]) < 1e-6,
        details=f"rest={z_rest}, lowered={z_lowered}",
    )

    return ctx.report()


object_model = build_object_model()
