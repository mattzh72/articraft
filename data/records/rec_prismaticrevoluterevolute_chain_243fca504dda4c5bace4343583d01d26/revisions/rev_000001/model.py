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
    model = ArticulatedObject(name="bench_positioning_arm")

    dark_steel = Material("dark_steel", color=(0.08, 0.09, 0.10, 1.0))
    satin_rail = Material("satin_rail", color=(0.58, 0.62, 0.66, 1.0))
    carriage_orange = Material("carriage_orange", color=(0.95, 0.42, 0.10, 1.0))
    link_blue = Material("link_blue", color=(0.12, 0.30, 0.72, 1.0))
    joint_gray = Material("joint_gray", color=(0.35, 0.37, 0.40, 1.0))
    rubber = Material("rubber", color=(0.015, 0.015, 0.013, 1.0))

    base = model.part("base_rail")
    base.visual(
        Box((1.20, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    for y, name in ((-0.058, "guide_rail_0"), (0.058, "guide_rail_1")):
        base.visual(
            Box((1.12, 0.026, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.055)),
            material=satin_rail,
            name=name,
        )
    base.visual(
        Box((1.04, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=joint_gray,
        name="center_scale",
    )
    for x, name in ((-0.5825, "end_stop_0"), (0.5825, "end_stop_1")):
        base.visual(
            Box((0.035, 0.18, 0.075)),
            origin=Origin(xyz=(x, 0.0, 0.0725)),
            material=dark_steel,
            name=name,
        )
    for x in (-0.42, -0.14, 0.14, 0.42):
        for y in (-0.075, 0.075):
            base.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, y, 0.038)),
                material=joint_gray,
                name=f"rail_screw_{x:+.2f}_{y:+.2f}",
            )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.36, 0.145, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=carriage_orange,
        name="carriage_deck",
    )
    carriage.visual(
        Box((0.31, 0.060, 0.025)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0575)),
        material=carriage_orange,
        name="carriage_rib",
    )
    for y, name in ((-0.077, "side_shoe_0"), (0.077, "side_shoe_1")):
        carriage.visual(
            Box((0.325, 0.018, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.013)),
            material=joint_gray,
            name=name,
        )

    slide = model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.30, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.50),
    )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.060, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=joint_gray,
        name="turntable",
    )
    shoulder.visual(
        Box((0.105, 0.105, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=joint_gray,
        name="shoulder_block",
    )
    shoulder.visual(
        Cylinder(radius=0.034, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_rail,
        name="shoulder_pin",
    )
    shoulder.visual(
        Box((0.34, 0.050, 0.035)),
        origin=Origin(xyz=(0.17, 0.0, 0.085)),
        material=link_blue,
        name="upper_link",
    )
    shoulder.visual(
        Cylinder(radius=0.047, length=0.040),
        origin=Origin(xyz=(0.36, 0.0, 0.085)),
        material=joint_gray,
        name="elbow_stack",
    )

    shoulder_joint = model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.10, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-1.75, upper=1.75),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.040, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=joint_gray,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.32, 0.045, 0.030)),
        origin=Origin(xyz=(0.17, 0.0, 0.020)),
        material=link_blue,
        name="forearm_link",
    )
    forearm.visual(
        Box((0.045, 0.034, 0.034)),
        origin=Origin(xyz=(0.345, 0.0, 0.020)),
        material=joint_gray,
        name="pad_stem",
    )
    forearm.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.376, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="end_pad",
    )

    elbow = model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forearm,
        origin=Origin(xyz=(0.36, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.5, lower=-2.20, upper=2.20),
    )

    # Keep local names live for linters while making the intended mechanisms obvious.
    _ = (slide, shoulder_joint, elbow)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    shoulder = object_model.get_part("shoulder")
    forearm = object_model.get_part("forearm")
    slide = object_model.get_articulation("base_to_carriage")
    shoulder_joint = object_model.get_articulation("carriage_to_shoulder")
    elbow = object_model.get_articulation("shoulder_to_forearm")

    ctx.expect_contact(
        carriage,
        base,
        elem_a="carriage_deck",
        elem_b="guide_rail_0",
        name="carriage deck rides on the first rail",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="carriage_deck",
        elem_b="guide_rail_1",
        name="carriage deck rides on the second rail",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="y",
        inner_elem="carriage_deck",
        outer_elem="base_plate",
        margin=0.0,
        name="carriage stays within rail width",
    )
    ctx.expect_contact(
        shoulder,
        carriage,
        elem_a="turntable",
        elem_b="carriage_rib",
        name="shoulder turntable is seated on carriage",
    )
    ctx.expect_contact(
        forearm,
        shoulder,
        elem_a="elbow_hub",
        elem_b="elbow_stack",
        name="forearm elbow hub is seated on the upper link",
    )

    deck_aabb = ctx.part_element_world_aabb(carriage, elem="carriage_deck")
    shoulder_pos = ctx.part_world_position(shoulder)
    ctx.check(
        "carriage reads as a long stage before the arm",
        deck_aabb is not None
        and shoulder_pos is not None
        and shoulder_pos[0] - deck_aabb[0][0] > 0.24,
        details=f"deck_aabb={deck_aabb}, shoulder_pos={shoulder_pos}",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.50}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a="carriage_deck",
            elem_b="guide_rail_0",
            name="extended carriage remains supported by first rail",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.30,
            elem_a="carriage_deck",
            elem_b="base_plate",
            name="extended carriage remains on the base rail",
        )
        extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic carriage advances along the base",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.45,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    pad_rest = ctx.part_element_world_aabb(forearm, elem="end_pad")
    with ctx.pose({shoulder_joint: 1.0}):
        pad_swept = ctx.part_element_world_aabb(forearm, elem="end_pad")
    ctx.check(
        "shoulder revolute joint swings the arm pad",
        pad_rest is not None
        and pad_swept is not None
        and abs(((pad_swept[0][1] + pad_swept[1][1]) * 0.5) - ((pad_rest[0][1] + pad_rest[1][1]) * 0.5)) > 0.12,
        details=f"pad_rest={pad_rest}, pad_swept={pad_swept}",
    )

    pad_before_elbow = ctx.part_element_world_aabb(forearm, elem="end_pad")
    with ctx.pose({elbow: 1.1}):
        pad_after_elbow = ctx.part_element_world_aabb(forearm, elem="end_pad")
    ctx.check(
        "elbow revolute joint bends the second link",
        pad_before_elbow is not None
        and pad_after_elbow is not None
        and abs(
            ((pad_after_elbow[0][1] + pad_after_elbow[1][1]) * 0.5)
            - ((pad_before_elbow[0][1] + pad_before_elbow[1][1]) * 0.5)
        )
        > 0.08,
        details=f"pad_before={pad_before_elbow}, pad_after={pad_after_elbow}",
    )

    return ctx.report()


object_model = build_object_model()
