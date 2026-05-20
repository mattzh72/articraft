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
    model = ArticulatedObject(name="turntable_lift_module")

    cast_iron = Material("dark_cast_iron", color=(0.08, 0.09, 0.10, 1.0))
    satin_metal = Material("satin_metal", color=(0.62, 0.65, 0.66, 1.0))
    black_oxide = Material("black_oxide", color=(0.015, 0.016, 0.018, 1.0))
    blue_anodized = Material("blue_anodized", color=(0.05, 0.22, 0.52, 1.0))
    rail_steel = Material("polished_guide_steel", color=(0.82, 0.84, 0.82, 1.0))
    rubber = Material("dark_rubber", color=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.40, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast_iron,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.270, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=satin_metal,
        name="bearing_race",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=black_oxide,
        name="center_spigot",
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        Cylinder(radius=0.320, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black_oxide,
        name="turntable_disk",
    )
    rotary_stage.visual(
        Cylinder(radius=0.240, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=satin_metal,
        name="top_index_plate",
    )
    rotary_stage.visual(
        Box((0.060, 0.240, 0.600)),
        origin=Origin(xyz=(-0.070, 0.0, 0.362)),
        material=blue_anodized,
        name="guide_mast",
    )
    rotary_stage.visual(
        Box((0.120, 0.250, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, 0.092)),
        material=blue_anodized,
        name="lower_rail_block",
    )
    rotary_stage.visual(
        Box((0.120, 0.250, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, 0.632)),
        material=blue_anodized,
        name="upper_rail_block",
    )
    rotary_stage.visual(
        Cylinder(radius=0.012, length=0.508),
        origin=Origin(xyz=(0.035, -0.085, 0.362)),
        material=rail_steel,
        name="guide_rail_0",
    )
    rotary_stage.visual(
        Cylinder(radius=0.012, length=0.508),
        origin=Origin(xyz=(0.035, 0.085, 0.362)),
        material=rail_steel,
        name="guide_rail_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.096, 0.250, 0.120)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=blue_anodized,
        name="carriage_plate",
    )
    for index, y in enumerate((-0.085, 0.085)):
        carriage.visual(
            Box((0.030, 0.065, 0.145)),
            origin=Origin(xyz=(0.067, y, 0.0)),
            material=satin_metal,
            name=f"linear_bearing_{index}",
        )
    carriage.visual(
        Box((0.030, 0.140, 0.090)),
        origin=Origin(xyz=(0.156, 0.0, 0.0)),
        material=black_oxide,
        name="tooling_face",
    )
    for index, (y, z) in enumerate(
        ((-0.045, -0.030), (0.045, -0.030), (-0.045, 0.030), (0.045, 0.030))
    ):
        carriage.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(
                xyz=(0.174, y, z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rubber,
            name=f"face_screw_{index}",
        )

    model.articulation(
        "base_to_rotary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "rotary_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rotary_stage,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.280),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rotary_stage = object_model.get_part("rotary_stage")
    carriage = object_model.get_part("carriage")
    spin = object_model.get_articulation("base_to_rotary")
    lift = object_model.get_articulation("rotary_to_carriage")

    ctx.check(
        "lower stage is a vertical revolute joint",
        spin.articulation_type == ArticulationType.REVOLUTE and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "carriage lift is a vertical prismatic joint",
        lift.articulation_type == ArticulationType.PRISMATIC and tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )

    ctx.expect_gap(
        rotary_stage,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_race",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable disk seats on bearing race",
    )
    ctx.expect_overlap(
        rotary_stage,
        base,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="bearing_race",
        min_overlap=0.20,
        name="turntable is centered over fixed bearing",
    )
    for rail_name in ("guide_rail_0", "guide_rail_1"):
        ctx.expect_gap(
            carriage,
            rotary_stage,
            axis="x",
            positive_elem="carriage_plate",
            negative_elem=rail_name,
            max_gap=0.001,
            max_penetration=0.0,
            name=f"carriage rides against {rail_name}",
        )
        ctx.expect_overlap(
            carriage,
            rotary_stage,
            axes="z",
            elem_a="carriage_plate",
            elem_b=rail_name,
            min_overlap=0.10,
            name=f"carriage remains engaged with {rail_name}",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.280}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            rotary_stage,
            axes="z",
            elem_a="carriage_plate",
            elem_b="guide_rail_0",
            min_overlap=0.10,
            name="raised carriage still overlaps the guide rail length",
        )
    ctx.check(
        "prismatic carriage raises upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.25,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
