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
    model = ArticulatedObject(name="slide_mounted_service_arm")

    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black = Material("black_bearing", rgba=(0.015, 0.016, 0.018, 1.0))
    rail_steel = Material("polished_rail", rgba=(0.72, 0.74, 0.76, 1.0))
    safety_orange = Material("safety_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    arm_blue = Material("arm_blue", rgba=(0.08, 0.25, 0.55, 1.0))
    pin_grey = Material("pin_grey", rgba=(0.45, 0.47, 0.50, 1.0))

    guideway = model.part("guideway")
    guideway.visual(
        Box((1.35, 0.34, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="floor_plate",
    )
    guideway.visual(
        Box((1.22, 0.18, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=dark_steel,
        name="raised_bed",
    )
    guideway.visual(
        Cylinder(radius=0.018, length=1.15),
        origin=Origin(xyz=(0.0, -0.085, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="rail_0",
    )
    guideway.visual(
        Cylinder(radius=0.018, length=1.15),
        origin=Origin(xyz=(0.0, 0.085, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="rail_1",
    )
    for idx, x in enumerate((-0.625, 0.625)):
        guideway.visual(
            Box((0.055, 0.27, 0.16)),
            origin=Origin(xyz=(x, 0.0, 0.12)),
            material=dark_steel,
            name=f"end_stop_{idx}",
        )
    for idx, x in enumerate((-0.44, -0.15, 0.15, 0.44)):
        for idy, y in enumerate((-0.135, 0.135)):
            guideway.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(x, y, 0.041)),
                material=pin_grey,
                name=f"anchor_bolt_{idx}_{idy}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.27, 0.23, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=safety_orange,
        name="saddle_body",
    )
    carriage.visual(
        Box((0.22, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, -0.085, -0.122)),
        material=black,
        name="bearing_shoe_0",
    )
    carriage.visual(
        Box((0.22, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, 0.085, -0.122)),
        material=black,
        name="bearing_shoe_1",
    )
    carriage.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=pin_grey,
        name="turntable_bearing",
    )
    carriage.visual(
        Box((0.11, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, -0.132, -0.055)),
        material=black,
        name="front_wiper",
    )
    carriage.visual(
        Box((0.11, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, 0.132, -0.055)),
        material=black,
        name="rear_wiper",
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        Cylinder(radius=0.070, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=pin_grey,
        name="shoulder_hub",
    )
    proximal_link.visual(
        Box((0.50, 0.070, 0.050)),
        origin=Origin(xyz=(0.25, 0.0, 0.0275)),
        material=arm_blue,
        name="main_beam",
    )
    proximal_link.visual(
        Box((0.32, 0.020, 0.045)),
        origin=Origin(xyz=(0.26, 0.0, 0.0300)),
        material=arm_blue,
        name="center_web",
    )
    proximal_link.visual(
        Cylinder(radius=0.055, length=0.055),
        origin=Origin(xyz=(0.50, 0.0, 0.0275)),
        material=pin_grey,
        name="elbow_lower_hub",
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=pin_grey,
        name="elbow_upper_hub",
    )
    distal_link.visual(
        Box((0.32, 0.055, 0.045)),
        origin=Origin(xyz=(0.16, 0.0, 0.0225)),
        material=arm_blue,
        name="short_beam",
    )
    distal_link.visual(
        Cylinder(radius=0.045, length=0.045),
        origin=Origin(xyz=(0.32, 0.0, 0.0225)),
        material=pin_grey,
        name="tool_flange",
    )
    distal_link.visual(
        Box((0.045, 0.095, 0.035)),
        origin=Origin(xyz=(0.355, 0.0, 0.0225)),
        material=pin_grey,
        name="service_plate",
    )

    model.articulation(
        "guideway_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guideway,
        child=carriage,
        origin=Origin(xyz=(-0.36, 0.0, 0.265)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.45, lower=0.0, upper=0.72),
    )
    model.articulation(
        "carriage_to_proximal_link",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=proximal_link,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-2.1, upper=2.1),
    )
    model.articulation(
        "proximal_link_to_distal_link",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(0.50, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-2.4, upper=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    guideway = object_model.get_part("guideway")
    carriage = object_model.get_part("carriage")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")
    slide = object_model.get_articulation("guideway_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_proximal_link")
    elbow = object_model.get_articulation("proximal_link_to_distal_link")

    ctx.check(
        "prismatic root with two serial revolutes",
        len(object_model.articulations) == 3
        and slide.articulation_type == ArticulationType.PRISMATIC
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and slide.parent == "guideway"
        and slide.child == "carriage"
        and shoulder.parent == "carriage"
        and shoulder.child == "proximal_link"
        and elbow.parent == "proximal_link"
        and elbow.child == "distal_link",
        details="The mechanism should be guideway -> carriage -> proximal_link -> distal_link.",
    )

    ctx.expect_contact(
        carriage,
        guideway,
        elem_a="bearing_shoe_0",
        elem_b="rail_0",
        contact_tol=0.001,
        name="carriage shoe rides on first rail",
    )
    ctx.expect_contact(
        carriage,
        guideway,
        elem_a="bearing_shoe_1",
        elem_b="rail_1",
        contact_tol=0.001,
        name="carriage shoe rides on second rail",
    )
    ctx.expect_contact(
        proximal_link,
        carriage,
        elem_a="shoulder_hub",
        elem_b="turntable_bearing",
        contact_tol=0.001,
        name="proximal link is carried by turntable",
    )
    ctx.expect_contact(
        distal_link,
        proximal_link,
        elem_a="elbow_upper_hub",
        elem_b="elbow_lower_hub",
        contact_tol=0.001,
        name="distal link is stacked on elbow hub",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.72}):
        ctx.expect_contact(
            carriage,
            guideway,
            elem_a="bearing_shoe_0",
            elem_b="rail_0",
            contact_tol=0.001,
            name="first shoe stays on rail at full travel",
        )
        ctx.expect_contact(
            carriage,
            guideway,
            elem_a="bearing_shoe_1",
            elem_b="rail_1",
            contact_tol=0.001,
            name="second shoe stays on rail at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along guideway",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.70,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
