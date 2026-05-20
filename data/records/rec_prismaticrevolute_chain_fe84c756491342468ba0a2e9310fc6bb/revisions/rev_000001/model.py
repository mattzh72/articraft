from __future__ import annotations

from math import isclose, pi

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
    model = ArticulatedObject(name="linear_slide_hinge_arm")

    dark_steel = model.material("dark_steel", color=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.55, 0.58, 0.60, 1.0))
    blue_anodized = model.material("blue_anodized", color=(0.05, 0.22, 0.52, 1.0))
    black_rubber = model.material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))
    safety_orange = model.material("safety_orange", color=(0.95, 0.34, 0.05, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((0.75, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="base_plate",
    )
    rail.visual(
        Box((0.68, 0.018, 0.032)),
        # A tiny embed into the base plate makes the rail read as one bolted extrusion.
        origin=Origin(xyz=(0.0, -0.050, 0.040)),
        material=brushed_steel,
        name="guide_rail_0",
    )
    rail.visual(
        Box((0.68, 0.018, 0.032)),
        # A tiny embed into the base plate makes the rail read as one bolted extrusion.
        origin=Origin(xyz=(0.0, 0.050, 0.040)),
        material=brushed_steel,
        name="guide_rail_1",
    )
    for x, name in ((-0.365, "end_stop_0"), (0.365, "end_stop_1")):
        rail.visual(
            Box((0.018, 0.16, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0525)),
            material=dark_steel,
            name=name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.130, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.050, 0.003)),
        material=black_rubber,
        name="bearing_pad_0",
    )
    carriage.visual(
        Box((0.130, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.050, 0.003)),
        material=black_rubber,
        name="bearing_pad_1",
    )
    carriage.visual(
        Box((0.170, 0.145, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=blue_anodized,
        name="saddle",
    )
    for y, name in ((-0.0795, "side_skirt_0"), (0.0795, "side_skirt_1")):
        carriage.visual(
            Box((0.170, 0.014, 0.034)),
            origin=Origin(xyz=(0.0, y, -0.011)),
            material=blue_anodized,
            name=name,
        )
    carriage.visual(
        Box((0.095, 0.088, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=blue_anodized,
        name="bracket_base",
    )
    carriage.visual(
        Box((0.055, 0.014, 0.085)),
        origin=Origin(xyz=(0.0, -0.032, 0.0855)),
        material=blue_anodized,
        name="cheek_0",
    )
    carriage.visual(
        Box((0.055, 0.014, 0.085)),
        origin=Origin(xyz=(0.0, 0.032, 0.0855)),
        material=blue_anodized,
        name="cheek_1",
    )
    for y, name in ((-0.040, "pivot_bore_0"), (0.040, "pivot_bore_1")):
        carriage.visual(
            Cylinder(radius=0.011, length=0.002),
            origin=Origin(xyz=(0.0, y, 0.095), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=name,
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.016, length=0.044),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_hub",
    )
    arm.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_pin",
    )
    arm.visual(
        Box((0.160, 0.024, 0.018)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=brushed_steel,
        name="arm_bar",
    )
    arm.visual(
        Box((0.046, 0.060, 0.060)),
        origin=Origin(xyz=(0.192, 0.0, 0.0)),
        material=safety_orange,
        name="square_pad",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(-0.200, 0.0, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.320),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        # With the arm geometry extending along local +X, -Y makes positive
        # joint motion lift the output pad upward from the carriage.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    slide = object_model.get_articulation("rail_to_carriage")
    hinge = object_model.get_articulation("carriage_to_arm")

    ctx.check(
        "carriage uses prismatic slide on rail",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.parent == "rail"
        and slide.child == "carriage"
        and slide.axis == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, parent={slide.parent}, child={slide.child}, axis={slide.axis}",
    )
    ctx.check(
        "arm hinge is rooted on carriage",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.parent == "carriage"
        and hinge.child == "arm"
        and hinge.axis == (0.0, -1.0, 0.0),
        details=f"type={hinge.articulation_type}, parent={hinge.parent}, child={hinge.child}, axis={hinge.axis}",
    )

    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        positive_elem="bearing_pad_0",
        negative_elem="guide_rail_0",
        max_gap=0.0005,
        max_penetration=0.0,
        name="carriage pad sits on first guide rail",
    )
    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        positive_elem="bearing_pad_1",
        negative_elem="guide_rail_1",
        max_gap=0.0005,
        max_penetration=0.0,
        name="carriage pad sits on second guide rail",
    )
    ctx.expect_gap(
        carriage,
        arm,
        axis="y",
        positive_elem="cheek_1",
        negative_elem="pivot_hub",
        min_gap=0.001,
        max_gap=0.006,
        name="positive cheek clears arm hub",
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="y",
        positive_elem="pivot_hub",
        negative_elem="cheek_0",
        min_gap=0.001,
        max_gap=0.006,
        name="negative cheek clears arm hub",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_arm = ctx.part_world_position(arm)
    rest_pad_aabb = ctx.part_element_world_aabb(arm, elem="square_pad")
    with ctx.pose({slide: 0.320}):
        extended_carriage = ctx.part_world_position(carriage)
        extended_arm = ctx.part_world_position(arm)
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            positive_elem="bearing_pad_0",
            negative_elem="guide_rail_0",
            max_gap=0.0005,
            max_penetration=0.0,
            name="extended carriage remains seated on rail",
        )
    with ctx.pose({hinge: 1.0}):
        lifted_pad_aabb = ctx.part_element_world_aabb(arm, elem="square_pad")

    if rest_carriage is not None and rest_arm is not None and extended_carriage is not None and extended_arm is not None:
        carriage_dx = extended_carriage[0] - rest_carriage[0]
        arm_dx = extended_arm[0] - rest_arm[0]
        ctx.check(
            "arm pivot travels with carriage",
            carriage_dx > 0.30 and isclose(carriage_dx, arm_dx, abs_tol=1e-6),
            details=f"carriage_dx={carriage_dx}, arm_dx={arm_dx}",
        )
    else:
        ctx.fail("arm pivot travels with carriage", "missing world positions")

    if rest_pad_aabb is not None and lifted_pad_aabb is not None:
        rest_pad_z = (rest_pad_aabb[0][2] + rest_pad_aabb[1][2]) / 2.0
        lifted_pad_z = (lifted_pad_aabb[0][2] + lifted_pad_aabb[1][2]) / 2.0
        ctx.check(
            "positive hinge motion lifts square pad",
            lifted_pad_z > rest_pad_z + 0.12,
            details=f"rest_pad_z={rest_pad_z}, lifted_pad_z={lifted_pad_z}",
        )
    else:
        ctx.fail("positive hinge motion lifts square pad", "missing square pad AABB")

    return ctx.report()


object_model = build_object_model()
