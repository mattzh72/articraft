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
    model = ArticulatedObject(name="side_wall_transfer_slide")

    painted_steel = Material("dark_painted_steel", rgba=(0.08, 0.10, 0.12, 1.0))
    zinc = Material("zinc_plated_guide", rgba=(0.62, 0.64, 0.62, 1.0))
    carriage_finish = Material("carriage_orange", rgba=(0.92, 0.42, 0.08, 1.0))
    arm_finish = Material("flap_arm_yellow", rgba=(0.95, 0.76, 0.18, 1.0))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    bolt_dark = Material("dark_fasteners", rgba=(0.025, 0.025, 0.025, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.74, 0.028, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=painted_steel,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.78, 0.12, 0.022)),
        origin=Origin(xyz=(0.0, 0.038, 0.011)),
        material=painted_steel,
        name="ground_foot",
    )

    for i, x in enumerate((-0.28, 0.0, 0.28)):
        side_plate.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, 0.018, 0.45), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_dark,
            name=f"top_bolt_{i}",
        )
        side_plate.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, 0.018, 0.08), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_dark,
            name=f"lower_bolt_{i}",
        )

    side_plate.visual(
        Box((0.60, 0.035, 0.060)),
        origin=Origin(xyz=(0.0, 0.035, 0.30)),
        material=zinc,
        name="guide_bar",
    )
    side_plate.visual(
        Box((0.54, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.0545, 0.30)),
        material=bolt_dark,
        name="guide_slot",
    )
    for i, x in enumerate((-0.24, 0.24)):
        side_plate.visual(
            Box((0.065, 0.030, 0.095)),
            origin=Origin(xyz=(x, 0.021, 0.30)),
            material=zinc,
            name=f"guide_standoff_{i}",
        )
    for i, x in enumerate((-0.285, 0.285)):
        side_plate.visual(
            Cylinder(radius=0.014, length=0.009),
            origin=Origin(xyz=(x, 0.057, 0.335), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bolt_dark,
            name=f"rail_bolt_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.145, 0.040, 0.160)),
        origin=Origin(xyz=(-0.0025, 0.0, 0.0)),
        material=carriage_finish,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.128, 0.006, 0.130)),
        origin=Origin(xyz=(-0.006, -0.027, 0.0)),
        material=rubber,
        name="rear_wear_pad",
    )
    for i, x in enumerate((-0.040, 0.035)):
        for j, z in enumerate((-0.050, 0.050)):
            carriage.visual(
                Cylinder(radius=0.017, length=0.012),
                origin=Origin(xyz=(x, -0.023, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=bolt_dark,
                name=f"roller_cap_{i}_{j}",
            )
    carriage.visual(
        Box((0.070, 0.025, 0.060)),
        origin=Origin(xyz=(0.065, 0.010, 0.015)),
        material=carriage_finish,
        name="front_yoke_base",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.095, 0.012, 0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="pivot_barrel",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.095, 0.012, 0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt_dark,
        name="pivot_pin_stub",
    )

    flap_arm = model.part("flap_arm")
    flap_arm.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="hinge_hub",
    )
    flap_arm.visual(
        Box((0.080, 0.024, 0.034)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=arm_finish,
        name="root_web",
    )
    flap_arm.visual(
        Box((0.270, 0.026, 0.030)),
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        material=arm_finish,
        name="arm_bar",
    )
    flap_arm.visual(
        Box((0.105, 0.036, 0.078)),
        origin=Origin(xyz=(0.365, 0.0, 0.0)),
        material=arm_finish,
        name="flap_pad",
    )
    flap_arm.visual(
        Box((0.112, 0.040, 0.014)),
        origin=Origin(xyz=(0.373, 0.0, -0.046)),
        material=rubber,
        name="rubber_edge",
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=carriage,
        origin=Origin(xyz=(-0.18, 0.085, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.26),
    )
    model.articulation(
        "arm_pivot",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=flap_arm,
        origin=Origin(xyz=(0.095, 0.037, 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    side_plate = object_model.get_part("side_plate")
    carriage = object_model.get_part("carriage")
    flap_arm = object_model.get_part("flap_arm")
    guide_slide = object_model.get_articulation("guide_slide")
    arm_pivot = object_model.get_articulation("arm_pivot")

    ctx.check(
        "guide stage is prismatic",
        guide_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"found {guide_slide.articulation_type}",
    )
    ctx.check(
        "flap arm is revolute",
        arm_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"found {arm_pivot.articulation_type}",
    )

    ctx.expect_gap(
        carriage,
        side_plate,
        axis="y",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="rear_wear_pad",
        negative_elem="guide_bar",
        name="carriage pad runs proud of guide",
    )
    ctx.expect_overlap(
        carriage,
        side_plate,
        axes="xz",
        min_overlap=0.050,
        elem_a="rear_wear_pad",
        elem_b="guide_bar",
        name="carriage remains registered on guide",
    )
    ctx.expect_contact(
        flap_arm,
        carriage,
        contact_tol=0.0015,
        elem_a="hinge_hub",
        elem_b="pivot_barrel",
        name="arm hub bears on fixed pivot barrel",
    )

    rest_pos = ctx.part_world_position(carriage)
    closed_pad = ctx.part_element_world_aabb(flap_arm, elem="flap_pad")

    with ctx.pose({guide_slide: 0.26}):
        ctx.expect_overlap(
            carriage,
            side_plate,
            axes="xz",
            min_overlap=0.050,
            elem_a="rear_wear_pad",
            elem_b="guide_bar",
            name="extended carriage still overlaps guide length",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage advances forward on straight guide",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.24,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({arm_pivot: 1.20}):
        raised_pad = ctx.part_element_world_aabb(flap_arm, elem="flap_pad")

    ctx.check(
        "flap arm lifts clear upward",
        closed_pad is not None
        and raised_pad is not None
        and raised_pad[1][2] > closed_pad[1][2] + 0.22,
        details=f"closed_pad={closed_pad}, raised_pad={raised_pad}",
    )

    return ctx.report()


object_model = build_object_model()
