from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_positioning_module")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = model.material("ground_steel_rail", rgba=(0.62, 0.65, 0.66, 1.0))
    carriage_blue = model.material("blue_carriage_casting", rgba=(0.05, 0.20, 0.45, 1.0))
    pivot_steel = model.material("brushed_pivot_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    tool_steel = model.material("tool_steel", rgba=(0.80, 0.78, 0.70, 1.0))
    rubber = model.material("black_rubber_stops", rgba=(0.01, 0.01, 0.012, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((1.45, 0.24, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=anodized,
        name="base_bed",
    )
    base_rail.visual(
        Box((1.18, 0.10, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=rail_steel,
        name="guide_rail",
    )
    base_rail.visual(
        Box((1.22, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, 0.055, 0.095)),
        material=rail_steel,
        name="upper_way_0",
    )
    base_rail.visual(
        Box((1.22, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.055, 0.095)),
        material=rail_steel,
        name="upper_way_1",
    )
    base_rail.visual(
        Box((0.07, 0.30, 0.17)),
        origin=Origin(xyz=(-0.70, 0.0, 0.085)),
        material=rubber,
        name="end_stop_0",
    )
    base_rail.visual(
        Box((0.07, 0.30, 0.17)),
        origin=Origin(xyz=(0.70, 0.0, 0.085)),
        material=rubber,
        name="end_stop_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.28, 0.25, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=carriage_blue,
        name="saddle_plate",
    )
    carriage.visual(
        Box((0.24, 0.030, 0.075)),
        origin=Origin(xyz=(0.0, 0.081, -0.014)),
        material=carriage_blue,
        name="side_clamp_0",
    )
    carriage.visual(
        Box((0.24, 0.030, 0.075)),
        origin=Origin(xyz=(0.0, -0.081, -0.014)),
        material=carriage_blue,
        name="side_clamp_1",
    )
    carriage.visual(
        Box((0.14, 0.038, 0.270)),
        origin=Origin(xyz=(0.0, 0.098, 0.180)),
        material=carriage_blue,
        name="yoke_cheek_0",
    )
    carriage.visual(
        Box((0.14, 0.038, 0.270)),
        origin=Origin(xyz=(0.0, -0.098, 0.180)),
        material=carriage_blue,
        name="yoke_cheek_1",
    )
    carriage.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(0.0, 0.124, 0.220), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="bearing_cap_0",
    )
    carriage.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(0.0, -0.124, 0.220), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="bearing_cap_1",
    )

    elbow_bracket = model.part("elbow_bracket")
    elbow_bracket.visual(
        Cylinder(radius=0.055, length=0.158),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pivot_steel,
        name="pivot_hub",
    )
    elbow_bracket.visual(
        Box((0.385, 0.018, 0.120)),
        origin=Origin(xyz=(0.2375, 0.056, 0.0)),
        material=carriage_blue,
        name="sleeve_side_0",
    )
    elbow_bracket.visual(
        Box((0.385, 0.018, 0.120)),
        origin=Origin(xyz=(0.2375, -0.056, 0.0)),
        material=carriage_blue,
        name="sleeve_side_1",
    )
    elbow_bracket.visual(
        Box((0.385, 0.130, 0.018)),
        origin=Origin(xyz=(0.2375, 0.0, 0.056)),
        material=carriage_blue,
        name="sleeve_top",
    )
    elbow_bracket.visual(
        Box((0.385, 0.130, 0.018)),
        origin=Origin(xyz=(0.2375, 0.0, -0.056)),
        material=carriage_blue,
        name="sleeve_bottom",
    )
    elbow_bracket.visual(
        Box((0.220, 0.045, 0.085)),
        origin=Origin(xyz=(0.125, 0.0, 0.107)),
        material=carriage_blue,
        name="elbow_web",
    )

    tool_nose = model.part("tool_nose")
    tool_nose.visual(
        Cylinder(radius=0.026, length=0.620),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=tool_steel,
        name="inner_ram",
    )
    tool_nose.visual(
        Box((0.240, 0.036, 0.042)),
        origin=Origin(xyz=(-0.180, 0.0, 0.026)),
        material=pivot_steel,
        name="guide_shoe",
    )
    tool_nose.visual(
        Cylinder(radius=0.036, length=0.050),
        origin=Origin(xyz=(0.355, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=pivot_steel,
        name="front_collar",
    )
    tool_nose.visual(
        Cylinder(radius=0.012, length=0.115),
        origin=Origin(xyz=(0.437, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=tool_steel,
        name="tool_bit",
    )
    tool_nose.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        material=tool_steel,
        name="rounded_tip",
    )

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=carriage,
        origin=Origin(xyz=(-0.42, 0.0, 0.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.70),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=elbow_bracket,
        child=tool_nose,
        origin=Origin(xyz=(0.430, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.18),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_rail = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    elbow_bracket = object_model.get_part("elbow_bracket")
    tool_nose = object_model.get_part("tool_nose")
    rail_slide = object_model.get_articulation("rail_slide")
    elbow_pivot = object_model.get_articulation("elbow_pivot")
    nose_slide = object_model.get_articulation("nose_slide")

    ctx.expect_contact(
        carriage,
        base_rail,
        elem_a="saddle_plate",
        elem_b="guide_rail",
        contact_tol=0.001,
        name="carriage saddle rides on the fixed rail",
    )
    ctx.expect_within(
        elbow_bracket,
        carriage,
        axes="y",
        inner_elem="pivot_hub",
        margin=0.0,
        name="pivot hub is captured between yoke cheeks",
    )
    ctx.expect_within(
        tool_nose,
        elbow_bracket,
        axes="yz",
        inner_elem="inner_ram",
        margin=0.0,
        name="tool ram stays inside the square sleeve bore",
    )
    ctx.expect_overlap(
        tool_nose,
        elbow_bracket,
        axes="x",
        elem_a="inner_ram",
        elem_b="sleeve_top",
        min_overlap=0.20,
        name="collapsed tool nose remains deeply telescoped",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({rail_slide: 0.70}):
        carriage_extended = ctx.part_world_position(carriage)
    ctx.check(
        "root carriage slides along the rail",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.60,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    nose_rest = ctx.part_world_position(tool_nose)
    with ctx.pose({nose_slide: 0.18}):
        ctx.expect_overlap(
            tool_nose,
            elbow_bracket,
            axes="x",
            elem_a="inner_ram",
            elem_b="sleeve_top",
            min_overlap=0.09,
            name="extended tool nose retains sleeve insertion",
        )
        nose_extended = ctx.part_world_position(tool_nose)
    ctx.check(
        "tool nose extends along its local axis",
        nose_rest is not None and nose_extended is not None and nose_extended[0] > nose_rest[0] + 0.15,
        details=f"rest={nose_rest}, extended={nose_extended}",
    )

    elbow_rest = ctx.part_world_position(tool_nose)
    with ctx.pose({elbow_pivot: 0.85}):
        elbow_lifted = ctx.part_world_position(tool_nose)
    ctx.check(
        "rotary elbow lifts the nose about the cross axis",
        elbow_rest is not None and elbow_lifted is not None and elbow_lifted[2] > elbow_rest[2] + 0.25,
        details=f"rest={elbow_rest}, lifted={elbow_lifted}",
    )

    return ctx.report()


object_model = build_object_model()
