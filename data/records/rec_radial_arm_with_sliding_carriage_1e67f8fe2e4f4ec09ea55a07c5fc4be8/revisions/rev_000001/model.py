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
    model = ArticulatedObject(name="shop_radial_arm_module")

    cast_iron = Material("dark_cast_iron", color=(0.08, 0.09, 0.09, 1.0))
    machine_yellow = Material("machine_yellow", color=(0.95, 0.66, 0.10, 1.0))
    black = Material("blackened_hardware", color=(0.015, 0.015, 0.014, 1.0))
    rail_steel = Material("polished_guide_steel", color=(0.72, 0.74, 0.72, 1.0))
    bearing_gray = Material("bearing_gray", color=(0.26, 0.28, 0.29, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.38, 0.30, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.324),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=cast_iron,
        name="short_column",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3775)),
        material=bearing_gray,
        name="lower_bearing",
    )
    for index, (x, y) in enumerate(
        ((-0.145, -0.105), (-0.145, 0.105), (0.145, -0.105), (0.145, 0.105))
    ):
        pedestal.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, 0.045)),
            material=black,
            name=f"anchor_bolt_{index}",
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.115, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=bearing_gray,
        name="rotary_plate",
    )
    arm.visual(
        Cylinder(radius=0.056, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=machine_yellow,
        name="shoulder_hub",
    )
    arm.visual(
        Box((0.140, 0.120, 0.095)),
        origin=Origin(xyz=(-0.070, 0.0, 0.152)),
        material=machine_yellow,
        name="rear_counterweight",
    )
    arm.visual(
        Box((0.550, 0.110, 0.075)),
        origin=Origin(xyz=(0.295, 0.0, 0.165)),
        material=machine_yellow,
        name="main_beam",
    )
    for index, y in enumerate((-0.035, 0.035)):
        arm.visual(
            Cylinder(radius=0.008, length=0.450),
            origin=Origin(xyz=(0.325, y, 0.2105), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rail_steel,
            name=f"guide_rail_{index}",
        )
    arm.visual(
        Box((0.026, 0.142, 0.054)),
        origin=Origin(xyz=(0.092, 0.0, 0.226)),
        material=black,
        name="inner_stop",
    )
    arm.visual(
        Box((0.026, 0.142, 0.054)),
        origin=Origin(xyz=(0.558, 0.0, 0.226)),
        material=black,
        name="outer_stop",
    )

    carriage = model.part("carriage")
    for index, y in enumerate((-0.035, 0.035)):
        carriage.visual(
            Box((0.105, 0.028, 0.028)),
            origin=Origin(xyz=(0.0, y, 0.014)),
            material=bearing_gray,
            name=f"linear_bearing_{index}",
        )
    carriage.visual(
        Box((0.130, 0.150, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=black,
        name="saddle_plate",
    )
    carriage.visual(
        Box((0.075, 0.022, 0.120)),
        origin=Origin(xyz=(0.0, -0.076, -0.018)),
        material=black,
        name="tool_side_plate",
    )
    carriage.visual(
        Cylinder(radius=0.022, length=0.065),
        origin=Origin(xyz=(0.0, -0.105, -0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="tool_socket",
    )

    model.articulation(
        "pedestal_to_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.9, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.190, 0.0, 0.2185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.150),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    shoulder = object_model.get_articulation("pedestal_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    ctx.check(
        "shoulder joint is vertical",
        tuple(round(v, 6) for v in shoulder.axis) == (0.0, 0.0, 1.0),
        details=f"axis={shoulder.axis}",
    )
    ctx.check(
        "carriage travel is 150 mm",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and abs(slide.motion_limits.upper - 0.150) < 1.0e-6,
        details=f"limits={slide.motion_limits}",
    )

    ctx.expect_contact(
        arm,
        pedestal,
        elem_a="rotary_plate",
        elem_b="lower_bearing",
        name="rotary shoulder sits on the pedestal bearing",
    )
    ctx.expect_overlap(
        arm,
        pedestal,
        axes="xy",
        elem_a="rotary_plate",
        elem_b="lower_bearing",
        min_overlap=0.18,
        name="rotary plates remain coaxially nested in plan",
    )

    for index in (0, 1):
        ctx.expect_gap(
            carriage,
            arm,
            axis="z",
            positive_elem=f"linear_bearing_{index}",
            negative_elem=f"guide_rail_{index}",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"carriage bearing {index} rides on guide rail {index}",
        )
        ctx.expect_overlap(
            carriage,
            arm,
            axes="xy",
            elem_a=f"linear_bearing_{index}",
            elem_b=f"guide_rail_{index}",
            min_overlap=0.012,
            name=f"carriage bearing {index} stays over guide rail {index}",
        )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.150}):
        for index in (0, 1):
            ctx.expect_gap(
                carriage,
                arm,
                axis="z",
                positive_elem=f"linear_bearing_{index}",
                negative_elem=f"guide_rail_{index}",
                max_gap=0.001,
                max_penetration=0.0,
                name=f"extended bearing {index} remains on its rail",
            )
            ctx.expect_overlap(
                carriage,
                arm,
                axes="xy",
                elem_a=f"linear_bearing_{index}",
                elem_b=f"guide_rail_{index}",
                min_overlap=0.012,
                name=f"extended bearing {index} remains captured by its rail",
            )
        extended_position = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along the beam",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.145,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    rest_beam_aabb = ctx.part_element_world_aabb(arm, elem="main_beam")
    with ctx.pose({shoulder: 0.70}):
        swung_beam_aabb = ctx.part_element_world_aabb(arm, elem="main_beam")
    ctx.check(
        "shoulder swing rotates the beam around the pedestal",
        rest_beam_aabb is not None
        and swung_beam_aabb is not None
        and swung_beam_aabb[1][1] > rest_beam_aabb[1][1] + 0.20,
        details=f"rest={rest_beam_aabb}, swung={swung_beam_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
