from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TURNTABLE_Z = 0.080
ROTARY_PLATE_THICKNESS = 0.032
GUIDE_TRAVEL = 0.285


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_lift_module")

    powder_black = model.material("powder_black", rgba=(0.07, 0.075, 0.08, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.65, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.77, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.08, 0.25, 0.48, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.42, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.52, 0.52, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=powder_black,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.185, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=dark_graphite,
        name="bearing_housing",
    )
    base.visual(
        Cylinder(radius=0.120, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=satin_steel,
        name="bearing_race",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.52, 0.52, TURNTABLE_Z)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_Z * 0.5)),
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=0.170, length=ROTARY_PLATE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, ROTARY_PLATE_THICKNESS * 0.5)),
        material=brushed_aluminum,
        name="rotary_plate",
    )
    for index, (x, y) in enumerate(
        ((0.130, 0.0), (0.0, 0.130), (-0.130, 0.0), (0.0, -0.130))
    ):
        rotary_stage.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(x, y, ROTARY_PLATE_THICKNESS + 0.003)),
            material=satin_steel,
            name=f"bolt_{index}",
        )
    rotary_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=ROTARY_PLATE_THICKNESS),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, ROTARY_PLATE_THICKNESS * 0.5)),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.190, 0.100, 0.040)),
        origin=Origin(xyz=(0.0, 0.040, 0.020)),
        material=dark_graphite,
        name="base_saddle",
    )
    guide.visual(
        Box((0.035, 0.020, 0.505)),
        origin=Origin(xyz=(0.0, 0.085, 0.2925)),
        material=dark_graphite,
        name="rear_spine",
    )
    guide.visual(
        Cylinder(radius=0.012, length=0.510),
        origin=Origin(xyz=(-0.055, 0.020, 0.291)),
        material=satin_steel,
        name="rail_0",
    )
    guide.visual(
        Cylinder(radius=0.012, length=0.510),
        origin=Origin(xyz=(0.055, 0.020, 0.291)),
        material=satin_steel,
        name="rail_1",
    )
    guide.visual(
        Cylinder(radius=0.006, length=0.500),
        origin=Origin(xyz=(0.0, 0.020, 0.290)),
        material=satin_steel,
        name="lead_screw",
    )
    guide.visual(
        Box((0.180, 0.075, 0.038)),
        origin=Origin(xyz=(0.0, 0.040, 0.561)),
        material=dark_graphite,
        name="top_bridge",
    )
    guide.visual(
        Box((0.018, 0.006, 0.440)),
        origin=Origin(xyz=(-0.0265, 0.078, 0.300)),
        material=safety_orange,
        name="travel_scale",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.20, 0.12, 0.60)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.040, 0.30)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.180, 0.040, 0.130)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=carriage_blue,
        name="front_plate",
    )
    carriage.visual(
        Box((0.120, 0.033, 0.026)),
        origin=Origin(xyz=(0.0, -0.0905, -0.035)),
        material=dark_graphite,
        name="tool_shelf",
    )
    carriage.visual(
        Box((0.060, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=brushed_aluminum,
        name="lift_nut",
    )
    carriage.visual(
        Box((0.0085, 0.054, 0.110)),
        origin=Origin(xyz=(-0.07125, -0.008, 0.0)),
        material=brushed_aluminum,
        name="shoe_0_0",
    )
    carriage.visual(
        Box((0.0085, 0.054, 0.110)),
        origin=Origin(xyz=(-0.03875, -0.008, 0.0)),
        material=brushed_aluminum,
        name="shoe_0_1",
    )
    carriage.visual(
        Box((0.0085, 0.054, 0.110)),
        origin=Origin(xyz=(0.03875, -0.008, 0.0)),
        material=brushed_aluminum,
        name="shoe_1_0",
    )
    carriage.visual(
        Box((0.0085, 0.054, 0.110)),
        origin=Origin(xyz=(0.07125, -0.008, 0.0)),
        material=brushed_aluminum,
        name="shoe_1_1",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.19, 0.10, 0.15)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
    )

    model.articulation(
        "base_to_rotary_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "rotary_stage_to_guide",
        ArticulationType.FIXED,
        parent=rotary_stage,
        child=guide,
        origin=Origin(xyz=(0.0, 0.0, ROTARY_PLATE_THICKNESS)),
    )
    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.020, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=GUIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    rotary_stage = object_model.get_part("rotary_stage")
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    turn_joint = object_model.get_articulation("base_to_rotary_stage")
    lift_joint = object_model.get_articulation("guide_to_carriage")

    ctx.check(
        "lower stage is revolute",
        turn_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(turn_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={turn_joint.articulation_type}, axis={turn_joint.axis}",
    )
    ctx.check(
        "carriage is vertical prismatic",
        lift_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift_joint.articulation_type}, axis={lift_joint.axis}",
    )
    ctx.expect_gap(
        rotary_stage,
        base,
        axis="z",
        positive_elem="rotary_plate",
        negative_elem="bearing_housing",
        min_gap=0.0,
        max_gap=0.001,
        name="rotary plate seats on bearing",
    )
    ctx.expect_overlap(
        rotary_stage,
        base,
        axes="xy",
        elem_a="rotary_plate",
        elem_b="bearing_housing",
        min_overlap=0.20,
        name="turntable centered over bearing",
    )
    ctx.expect_gap(
        guide,
        rotary_stage,
        axis="z",
        positive_elem="base_saddle",
        negative_elem="rotary_plate",
        min_gap=0.0,
        max_gap=0.001,
        name="guide saddle bolted to rotating plate",
    )
    ctx.expect_overlap(
        guide,
        rotary_stage,
        axes="xy",
        elem_a="base_saddle",
        elem_b="rotary_plate",
        min_overlap=0.09,
        name="guide footprint sits on turntable",
    )
    ctx.expect_gap(
        carriage,
        guide,
        axis="z",
        positive_elem="front_plate",
        negative_elem="base_saddle",
        min_gap=0.06,
        max_gap=0.09,
        name="carriage clears lower saddle at bottom",
    )
    ctx.expect_gap(
        guide,
        carriage,
        axis="x",
        positive_elem="rail_1",
        negative_elem="shoe_1_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="carriage shoe clears rail inner side",
    )
    ctx.expect_gap(
        carriage,
        guide,
        axis="x",
        positive_elem="shoe_1_1",
        negative_elem="rail_1",
        max_gap=0.001,
        max_penetration=0.00001,
        name="carriage shoe clears rail outer side",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift_joint: GUIDE_TRAVEL}):
        ctx.expect_gap(
            guide,
            carriage,
            axis="z",
            positive_elem="top_bridge",
            negative_elem="front_plate",
            min_gap=0.005,
            max_gap=0.025,
            name="raised carriage stops below top bridge",
        )
        raised_pos = ctx.part_world_position(carriage)
    ctx.check(
        "lift joint raises carriage",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.25,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
