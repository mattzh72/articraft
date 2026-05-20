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


FIRST_TRAVEL = 0.32
SECOND_TRAVEL = 0.22
TIP_TRAVEL = 0.12


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_triple_carriage")

    painted_steel = model.material("painted_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.015, 0.017, 0.020, 1.0))
    rail_steel = model.material("linear_rail_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.08, 0.20, 0.55, 1.0))
    orange_stage = model.material("orange_stage", rgba=(0.95, 0.42, 0.08, 1.0))
    bolt_metal = model.material("bolt_metal", rgba=(0.35, 0.36, 0.36, 1.0))

    # Grounded vertical side plate.  The front face is +Y; all three carriages
    # translate along +X while being stacked outward from the side wall.
    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((1.25, 0.050, 0.65)),
        origin=Origin(xyz=(0.625, 0.000, 0.325)),
        material=painted_steel,
        name="grounded_plate",
    )
    side_plate.visual(
        Box((1.05, 0.035, 0.035)),
        origin=Origin(xyz=(0.625, 0.0425, 0.200)),
        material=rail_steel,
        name="base_lower_rail",
    )
    side_plate.visual(
        Box((1.05, 0.035, 0.035)),
        origin=Origin(xyz=(0.625, 0.0425, 0.450)),
        material=rail_steel,
        name="base_upper_rail",
    )
    for x in (0.085, 1.165):
        for z in (0.085, 0.565):
            side_plate.visual(
                Cylinder(radius=0.020, length=0.014),
                origin=Origin(xyz=(x, 0.032, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=bolt_metal,
                name=f"plate_bolt_{x:.3f}_{z:.3f}",
            )
    for x, name in ((0.080, "base_stop_0"), (1.170, "base_stop_1")):
        side_plate.visual(
            Box((0.035, 0.044, 0.090)),
            origin=Origin(xyz=(x, 0.047, 0.325)),
            material=dark_steel,
            name=name,
        )

    first_carriage = model.part("first_carriage")
    first_carriage.visual(
        Box((0.660, 0.035, 0.340)),
        origin=Origin(xyz=(0.000, 0.0225, 0.000)),
        material=dark_steel,
        name="first_backbone",
    )
    first_carriage.visual(
        Box((0.240, 0.040, 0.065)),
        origin=Origin(xyz=(-0.040, 0.000, -0.125)),
        material=bearing_black,
        name="first_lower_shoe",
    )
    first_carriage.visual(
        Box((0.240, 0.040, 0.065)),
        origin=Origin(xyz=(-0.040, 0.000, 0.125)),
        material=bearing_black,
        name="first_upper_shoe",
    )
    first_carriage.visual(
        Box((0.560, 0.025, 0.030)),
        origin=Origin(xyz=(0.040, 0.0510, -0.095)),
        material=rail_steel,
        name="first_lower_rail",
    )
    first_carriage.visual(
        Box((0.560, 0.025, 0.030)),
        origin=Origin(xyz=(0.040, 0.0510, 0.095)),
        material=rail_steel,
        name="first_upper_rail",
    )
    for x, name in ((-0.245, "first_stop_0"), (0.335, "first_stop_1")):
        first_carriage.visual(
            Box((0.035, 0.038, 0.075)),
            origin=Origin(xyz=(x, 0.052, 0.000)),
            material=blue_anodized,
            name=name,
        )

    second_carriage = model.part("second_carriage")
    second_carriage.visual(
        Box((0.460, 0.030, 0.270)),
        origin=Origin(xyz=(0.020, 0.0190, 0.000)),
        material=blue_anodized,
        name="second_backbone",
    )
    second_carriage.visual(
        Box((0.180, 0.035, 0.052)),
        origin=Origin(xyz=(0.000, 0.000, -0.095)),
        material=bearing_black,
        name="second_lower_shoe",
    )
    second_carriage.visual(
        Box((0.180, 0.035, 0.052)),
        origin=Origin(xyz=(0.000, 0.000, 0.095)),
        material=bearing_black,
        name="second_upper_shoe",
    )
    second_carriage.visual(
        Box((0.340, 0.020, 0.026)),
        origin=Origin(xyz=(0.040, 0.0430, -0.073)),
        material=rail_steel,
        name="second_lower_rail",
    )
    second_carriage.visual(
        Box((0.340, 0.020, 0.026)),
        origin=Origin(xyz=(0.040, 0.0430, 0.073)),
        material=rail_steel,
        name="second_upper_rail",
    )

    tip_stage = model.part("tip_stage")
    tip_stage.visual(
        Box((0.120, 0.028, 0.045)),
        origin=Origin(xyz=(0.000, 0.000, -0.073)),
        material=bearing_black,
        name="tip_lower_shoe",
    )
    tip_stage.visual(
        Box((0.120, 0.028, 0.045)),
        origin=Origin(xyz=(0.000, 0.000, 0.073)),
        material=bearing_black,
        name="tip_upper_shoe",
    )
    tip_stage.visual(
        Box((0.220, 0.040, 0.220)),
        origin=Origin(xyz=(0.080, 0.027, 0.000)),
        material=orange_stage,
        name="tip_carrier_plate",
    )
    tip_stage.visual(
        Box((0.180, 0.012, 0.160)),
        origin=Origin(xyz=(0.100, 0.052, 0.000)),
        material=painted_steel,
        name="tip_mounting_face",
    )

    model.articulation(
        "side_plate_to_first",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=first_carriage,
        origin=Origin(xyz=(0.360, 0.080, 0.325)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=0.0, upper=FIRST_TRAVEL),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_carriage,
        origin=Origin(xyz=(-0.080, 0.081, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.38, lower=0.0, upper=SECOND_TRAVEL),
    )
    model.articulation(
        "second_to_tip",
        ArticulationType.PRISMATIC,
        parent=second_carriage,
        child=tip_stage,
        origin=Origin(xyz=(-0.035, 0.067, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.30, lower=0.0, upper=TIP_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    first_carriage = object_model.get_part("first_carriage")
    second_carriage = object_model.get_part("second_carriage")
    tip_stage = object_model.get_part("tip_stage")
    side_plate_to_first = object_model.get_articulation("side_plate_to_first")
    first_to_second = object_model.get_articulation("first_to_second")
    second_to_tip = object_model.get_articulation("second_to_tip")

    ctx.check(
        "three serial prismatic joints",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (side_plate_to_first, first_to_second, second_to_tip)
        ),
    )

    ctx.expect_gap(
        first_carriage,
        side_plate,
        axis="y",
        positive_elem="first_upper_shoe",
        negative_elem="base_upper_rail",
        max_gap=0.001,
        max_penetration=0.00001,
        name="first carriage seated on base rail",
    )
    ctx.expect_overlap(
        first_carriage,
        side_plate,
        axes="xz",
        elem_a="first_upper_shoe",
        elem_b="base_upper_rail",
        min_overlap=0.020,
        name="first carriage retained on base rail",
    )
    ctx.expect_gap(
        second_carriage,
        first_carriage,
        axis="y",
        positive_elem="second_upper_shoe",
        negative_elem="first_upper_rail",
        max_gap=0.001,
        max_penetration=0.00001,
        name="second carriage seated on first rail",
    )
    ctx.expect_overlap(
        second_carriage,
        first_carriage,
        axes="xz",
        elem_a="second_upper_shoe",
        elem_b="first_upper_rail",
        min_overlap=0.020,
        name="second carriage retained on first rail",
    )
    ctx.expect_gap(
        tip_stage,
        second_carriage,
        axis="y",
        positive_elem="tip_upper_shoe",
        negative_elem="second_upper_rail",
        max_gap=0.001,
        max_penetration=0.00001,
        name="tip stage seated on second rail",
    )
    ctx.expect_overlap(
        tip_stage,
        second_carriage,
        axes="xz",
        elem_a="tip_upper_shoe",
        elem_b="second_upper_rail",
        min_overlap=0.018,
        name="tip stage retained on second rail",
    )

    rest_tip = ctx.part_world_position(tip_stage)
    with ctx.pose(
        {
            side_plate_to_first: FIRST_TRAVEL,
            first_to_second: SECOND_TRAVEL,
            second_to_tip: TIP_TRAVEL,
        }
    ):
        extended_tip = ctx.part_world_position(tip_stage)
        ctx.expect_overlap(
            first_carriage,
            side_plate,
            axes="xz",
            elem_a="first_upper_shoe",
            elem_b="base_upper_rail",
            min_overlap=0.020,
            name="first carriage retained when extended",
        )
        ctx.expect_overlap(
            second_carriage,
            first_carriage,
            axes="xz",
            elem_a="second_upper_shoe",
            elem_b="first_upper_rail",
            min_overlap=0.020,
            name="second carriage retained when extended",
        )
        ctx.expect_overlap(
            tip_stage,
            second_carriage,
            axes="xz",
            elem_a="tip_upper_shoe",
            elem_b="second_upper_rail",
            min_overlap=0.018,
            name="tip stage retained when extended",
        )

    ctx.check(
        "serial stages extend along x",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[0] > rest_tip[0] + FIRST_TRAVEL + SECOND_TRAVEL + TIP_TRAVEL - 0.010,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
