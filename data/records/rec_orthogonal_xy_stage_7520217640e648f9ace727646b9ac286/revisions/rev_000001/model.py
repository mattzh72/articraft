from __future__ import annotations

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

import math


LOWER_TRAVEL = 0.10
UPPER_TRAVEL = 0.045

X_RAIL_Y = 0.070
X_RAIL_TOP_Z = 0.050
Y_RAIL_X = 0.055
Y_RAIL_TOP_Z = 0.064


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_xy_table")

    model.material("painted_cast_iron", rgba=(0.16, 0.19, 0.21, 1.0))
    model.material("dark_cast_iron", rgba=(0.10, 0.12, 0.13, 1.0))
    model.material("ground_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    model.material("black_slot", rgba=(0.015, 0.015, 0.018, 1.0))
    model.material("lead_screw", rgba=(0.53, 0.45, 0.34, 1.0))

    lower_slide = model.part("lower_slide")
    lower_slide.visual(
        Box((0.480, 0.250, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="painted_cast_iron",
        name="base_casting",
    )
    for index, y in enumerate((-X_RAIL_Y, X_RAIL_Y)):
        lower_slide.visual(
            Box((0.420, 0.032, 0.026)),
            # The rail is very slightly seated into the casting so the root
            # reads as one bolted assembly rather than separate bars.
            origin=Origin(xyz=(0.0, y, 0.0370)),
            material="ground_steel",
            name=f"x_rail_{index}",
        )
    for index, x in enumerate((-0.225, 0.225)):
        lower_slide.visual(
            Box((0.030, 0.205, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material="dark_cast_iron",
            name=f"x_end_bearing_{index}",
        )
    lower_slide.visual(
        Cylinder(radius=0.005, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="lead_screw",
        name="x_lead_screw",
    )

    upper_slide = model.part("upper_slide")
    for index, y in enumerate((-X_RAIL_Y, X_RAIL_Y)):
        upper_slide.visual(
            Box((0.180, 0.046, 0.016)),
            origin=Origin(xyz=(0.0, y, 0.008)),
            material="ground_steel",
            name=f"x_shoe_{index}",
        )
    upper_slide.visual(
        Box((0.205, 0.196, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material="painted_cast_iron",
        name="cross_saddle",
    )
    for index, x in enumerate((-Y_RAIL_X, Y_RAIL_X)):
        upper_slide.visual(
            Box((0.030, 0.215, 0.023)),
            origin=Origin(xyz=(x, 0.0, 0.0525)),
            material="ground_steel",
            name=f"y_rail_{index}",
        )
    for index, y in enumerate((-0.112, 0.112)):
        upper_slide.visual(
            Box((0.176, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.051)),
            material="dark_cast_iron",
            name=f"y_end_bearing_{index}",
        )
    upper_slide.visual(
        Cylinder(radius=0.004, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.054), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="lead_screw",
        name="y_lead_screw",
    )

    top_plate = model.part("top_plate")
    for index, x in enumerate((-Y_RAIL_X, Y_RAIL_X)):
        top_plate.visual(
            Box((0.044, 0.120, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.007)),
            material="ground_steel",
            name=f"y_shoe_{index}",
        )
    top_plate.visual(
        Box((0.180, 0.140, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material="dark_cast_iron",
        name="table_carrier",
    )
    top_plate.visual(
        Box((0.215, 0.165, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material="painted_cast_iron",
        name="work_plate",
    )
    for index, x in enumerate((-0.055, 0.0, 0.055)):
        top_plate.visual(
            Box((0.012, 0.138, 0.002)),
            origin=Origin(xyz=(x, 0.0, 0.0445)),
            material="black_slot",
            name=f"t_slot_{index}",
        )

    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_slide,
        child=upper_slide,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_TRAVEL,
            upper=LOWER_TRAVEL,
            effort=550.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "upper_to_plate",
        ArticulationType.PRISMATIC,
        parent=upper_slide,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, Y_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-UPPER_TRAVEL,
            upper=UPPER_TRAVEL,
            effort=420.0,
            velocity=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_slide = object_model.get_part("lower_slide")
    upper_slide = object_model.get_part("upper_slide")
    top_plate = object_model.get_part("top_plate")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    upper_to_plate = object_model.get_articulation("upper_to_plate")

    lower_axis = tuple(lower_to_upper.axis)
    upper_axis = tuple(upper_to_plate.axis)
    dot = sum(a * b for a, b in zip(lower_axis, upper_axis))
    ctx.check(
        "orthogonal prismatic axes",
        lower_to_upper.articulation_type == ArticulationType.PRISMATIC
        and upper_to_plate.articulation_type == ArticulationType.PRISMATIC
        and abs(dot) < 1.0e-6,
        details=f"lower_axis={lower_axis}, upper_axis={upper_axis}, dot={dot}",
    )

    ctx.expect_gap(
        upper_slide,
        lower_slide,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="x_shoe_0",
        negative_elem="x_rail_0",
        name="upper slide sits on lower rail",
    )
    ctx.expect_overlap(
        upper_slide,
        lower_slide,
        axes="xy",
        min_overlap=0.030,
        elem_a="x_shoe_0",
        elem_b="x_rail_0",
        name="upper shoe remains on lower rail footprint",
    )
    ctx.expect_gap(
        top_plate,
        upper_slide,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="y_shoe_0",
        negative_elem="y_rail_0",
        name="top plate sits on upper rail",
    )
    ctx.expect_overlap(
        top_plate,
        upper_slide,
        axes="xy",
        min_overlap=0.025,
        elem_a="y_shoe_0",
        elem_b="y_rail_0",
        name="top shoe remains on upper rail footprint",
    )

    rest_upper = ctx.part_world_position(upper_slide)
    rest_plate = ctx.part_world_position(top_plate)
    with ctx.pose({lower_to_upper: LOWER_TRAVEL}):
        ctx.expect_overlap(
            upper_slide,
            lower_slide,
            axes="x",
            min_overlap=0.070,
            elem_a="x_shoe_0",
            elem_b="x_rail_0",
            name="lower slide retains support at x travel",
        )
        upper_at_limit = ctx.part_world_position(upper_slide)
    with ctx.pose({upper_to_plate: UPPER_TRAVEL}):
        ctx.expect_overlap(
            top_plate,
            upper_slide,
            axes="y",
            min_overlap=0.055,
            elem_a="y_shoe_0",
            elem_b="y_rail_0",
            name="upper slide retains support at y travel",
        )
        plate_at_limit = ctx.part_world_position(top_plate)

    ctx.check(
        "lower prismatic moves along x",
        rest_upper is not None
        and upper_at_limit is not None
        and upper_at_limit[0] > rest_upper[0] + LOWER_TRAVEL * 0.9
        and abs(upper_at_limit[1] - rest_upper[1]) < 1.0e-6,
        details=f"rest={rest_upper}, limit={upper_at_limit}",
    )
    ctx.check(
        "upper prismatic moves along y",
        rest_plate is not None
        and plate_at_limit is not None
        and plate_at_limit[1] > rest_plate[1] + UPPER_TRAVEL * 0.9
        and abs(plate_at_limit[0] - rest_plate[0]) < 1.0e-6,
        details=f"rest={rest_plate}, limit={plate_at_limit}",
    )

    return ctx.report()


object_model = build_object_model()
