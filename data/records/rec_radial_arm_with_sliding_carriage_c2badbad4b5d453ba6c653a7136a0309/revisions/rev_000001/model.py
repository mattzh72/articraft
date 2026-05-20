from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="pedestal_radial_beam_slide")

    cast_iron = model.material("dark_cast_iron", rgba=(0.16, 0.17, 0.18, 1.0))
    machined_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    oiled_rail = model.material("oiled_rail", rgba=(0.07, 0.08, 0.08, 1.0))
    safety_blue = model.material("safety_blue", rgba=(0.08, 0.23, 0.55, 1.0))
    face_metal = model.material("plain_output_metal", rgba=(0.74, 0.75, 0.72, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.28, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=cast_iron,
        name="floor_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=cast_iron,
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=0.19, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        material=machined_steel,
        name="top_bearing",
    )

    arm = model.part("radial_arm")
    arm.visual(
        Cylinder(radius=0.165, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=machined_steel,
        name="turntable_hub",
    )
    arm.visual(
        Box((1.25, 0.16, 0.08)),
        origin=Origin(xyz=(0.65, 0.0, 0.08)),
        material=cast_iron,
        name="beam_web",
    )
    arm.visual(
        Box((1.08, 0.035, 0.035)),
        origin=Origin(xyz=(0.70, 0.067, 0.1375)),
        material=oiled_rail,
        name="guide_rail_0",
    )
    arm.visual(
        Box((1.08, 0.035, 0.035)),
        origin=Origin(xyz=(0.70, -0.067, 0.1375)),
        material=oiled_rail,
        name="guide_rail_1",
    )
    arm.visual(
        Box((1.02, 0.055, 0.006)),
        origin=Origin(xyz=(0.72, 0.0, 0.123)),
        material=oiled_rail,
        name="center_oil_slot",
    )
    arm.visual(
        Box((0.055, 0.24, 0.14)),
        origin=Origin(xyz=(0.145, 0.0, 0.105)),
        material=cast_iron,
        name="inner_end_stop",
    )
    arm.visual(
        Box((0.055, 0.24, 0.14)),
        origin=Origin(xyz=(1.275, 0.0, 0.105)),
        material=cast_iron,
        name="outer_end_stop",
    )

    truck = model.part("sliding_truck")
    truck.visual(
        Box((0.25, 0.32, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=safety_blue,
        name="top_saddle",
    )
    truck.visual(
        Box((0.24, 0.045, 0.15)),
        origin=Origin(xyz=(0.0, 0.1825, 0.115)),
        material=safety_blue,
        name="side_cheek_0",
    )
    truck.visual(
        Box((0.24, 0.045, 0.15)),
        origin=Origin(xyz=(0.0, -0.1825, 0.115)),
        material=safety_blue,
        name="side_cheek_1",
    )
    truck.visual(
        Box((0.035, 0.30, 0.17)),
        origin=Origin(xyz=(0.140, 0.0, 0.2425)),
        material=face_metal,
        name="output_face",
    )
    truck.visual(
        Box((0.22, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.067, 0.165)),
        material=machined_steel,
        name="rail_shoe_0",
    )
    truck.visual(
        Box((0.22, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.067, 0.165)),
        material=machined_steel,
        name="rail_shoe_1",
    )

    model.articulation(
        "pedestal_to_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "arm_to_truck",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=truck,
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    arm = object_model.get_part("radial_arm")
    truck = object_model.get_part("sliding_truck")
    rotary = object_model.get_articulation("pedestal_to_arm")
    slide = object_model.get_articulation("arm_to_truck")

    ctx.expect_gap(
        arm,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem="turntable_hub",
        negative_elem="top_bearing",
        name="rotating hub sits on pedestal bearing",
    )
    ctx.expect_gap(
        truck,
        arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem="top_saddle",
        negative_elem="guide_rail_0",
        name="truck saddle rides on guide rail",
    )
    ctx.expect_within(
        truck,
        arm,
        axes="y",
        margin=0.002,
        inner_elem="rail_shoe_0",
        outer_elem="guide_rail_0",
        name="truck shoe remains centered over rail",
    )

    rest_pos = ctx.part_world_position(truck)
    with ctx.pose({slide: 0.65}):
        ctx.expect_overlap(
            truck,
            arm,
            axes="x",
            min_overlap=0.20,
            elem_a="top_saddle",
            elem_b="guide_rail_0",
            name="extended truck still engaged with rail",
        )
        extended_pos = ctx.part_world_position(truck)
    ctx.check(
        "truck slides outward along the arm",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.60,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({rotary: pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(arm, elem="beam_web")
    ctx.check(
        "arm rotates around vertical pedestal axis",
        turned_aabb is not None and turned_aabb[1][1] - turned_aabb[0][1] > 1.0,
        details=f"turned beam aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
