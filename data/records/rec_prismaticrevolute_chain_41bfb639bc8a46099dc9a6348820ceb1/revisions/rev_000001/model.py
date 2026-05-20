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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_chain")

    dark_steel = model.material("dark_steel", color=(0.08, 0.085, 0.09, 1.0))
    ground_black = model.material("ground_black", color=(0.02, 0.022, 0.025, 1.0))
    polished_rail = model.material("polished_rail", color=(0.63, 0.66, 0.68, 1.0))
    carriage_orange = model.material("carriage_orange", color=(0.90, 0.31, 0.09, 1.0))
    wear_pad = model.material("graphite_wear_pad", color=(0.015, 0.017, 0.018, 1.0))
    arm_blue = model.material("arm_blue", color=(0.05, 0.20, 0.58, 1.0))
    pin_metal = model.material("pin_metal", color=(0.78, 0.76, 0.70, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((0.88, 0.20, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="ground_plate",
    )
    base_rail.visual(
        Box((0.82, 0.032, 0.035)),
        origin=Origin(xyz=(0.0, -0.055, 0.0475)),
        material=polished_rail,
        name="rail_bar_0",
    )
    base_rail.visual(
        Box((0.82, 0.032, 0.035)),
        origin=Origin(xyz=(0.0, 0.055, 0.0475)),
        material=polished_rail,
        name="rail_bar_1",
    )
    base_rail.visual(
        Box((0.76, 0.045, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=ground_black,
        name="center_shadow_slot",
    )
    for x, stop_name in ((-0.405, "end_stop_0"), (0.405, "end_stop_1")):
        base_rail.visual(
            Box((0.030, 0.18, 0.095)),
            origin=Origin(xyz=(x, 0.0, 0.0775)),
            material=dark_steel,
            name=stop_name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.155, 0.150, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_orange,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.145, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.055, -0.027)),
        material=wear_pad,
        name="shoe_0",
    )
    carriage.visual(
        Box((0.145, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.055, -0.027)),
        material=wear_pad,
        name="shoe_1",
    )
    carriage.visual(
        Box((0.150, 0.012, 0.035)),
        origin=Origin(xyz=(0.0, -0.081, -0.004)),
        material=carriage_orange,
        name="side_skirt_0",
    )
    carriage.visual(
        Box((0.150, 0.012, 0.035)),
        origin=Origin(xyz=(0.0, 0.081, -0.004)),
        material=carriage_orange,
        name="side_skirt_1",
    )

    # A side clevis rigidly carried by the sliding carriage.  The two cheeks
    # are tied back to the carriage by a lower bridge, leaving the central
    # gap open for the arm knuckle.
    carriage.visual(
        Box((0.076, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, 0.081, 0.035)),
        material=carriage_orange,
        name="clevis_cheek_0",
    )
    carriage.visual(
        Box((0.076, 0.012, 0.090)),
        origin=Origin(xyz=(0.0, 0.139, 0.035)),
        material=carriage_orange,
        name="clevis_cheek_1",
    )
    carriage.visual(
        Box((0.076, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.110, -0.002)),
        material=carriage_orange,
        name="clevis_bridge",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(xyz=(0.0, 0.071, 0.035), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="pivot_cap_0",
    )
    carriage.visual(
        Cylinder(radius=0.025, length=0.008),
        origin=Origin(xyz=(0.0, 0.149, 0.035), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="pivot_cap_1",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.024, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=arm_blue,
        name="pivot_knuckle",
    )
    arm.visual(
        Box((0.240, 0.026, 0.026)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=arm_blue,
        name="arm_bar",
    )
    arm.visual(
        Cylinder(radius=0.023, length=0.030),
        origin=Origin(xyz=(0.250, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=arm_blue,
        name="rounded_tip",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=carriage,
        origin=Origin(xyz=(-0.250, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.500),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, 0.110, 0.035)),
        # The closed arm points along local +X; -Y makes positive rotation lift it.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base_rail = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    slide = object_model.get_articulation("rail_to_carriage")
    hinge = object_model.get_articulation("carriage_to_arm")

    ctx.check(
        "root joint is prismatic along the rail",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "carried arm is revolute about the clevis pivot",
        hinge.articulation_type == ArticulationType.REVOLUTE and tuple(hinge.axis) == (0.0, -1.0, 0.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    ctx.expect_gap(
        carriage,
        base_rail,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="shoe_0",
        negative_elem="rail_bar_0",
        name="near shoe rides on rail bar",
    )
    ctx.expect_gap(
        carriage,
        base_rail,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="shoe_1",
        negative_elem="rail_bar_1",
        name="far shoe rides on rail bar",
    )
    ctx.expect_within(
        arm,
        carriage,
        axes="y",
        margin=0.0,
        inner_elem="pivot_knuckle",
        outer_elem="clevis_bridge",
        name="arm knuckle sits between clevis cheeks",
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_tip = ctx.part_element_world_aabb(arm, elem="rounded_tip")
    with ctx.pose({slide: 0.50, hinge: 1.20}):
        extended_carriage = ctx.part_world_position(carriage)
        lifted_tip = ctx.part_element_world_aabb(arm, elem="rounded_tip")

    ctx.check(
        "carriage translates forward on the rail",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.45
        and abs(extended_carriage[1] - rest_carriage[1]) < 0.001
        and abs(extended_carriage[2] - rest_carriage[2]) < 0.001,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )
    ctx.check(
        "positive hinge rotation lifts the arm tip",
        rest_tip is not None
        and lifted_tip is not None
        and lifted_tip[0][2] > rest_tip[1][2] + 0.10,
        details=f"rest_tip={rest_tip}, lifted_tip={lifted_tip}",
    )

    return ctx.report()


object_model = build_object_model()
