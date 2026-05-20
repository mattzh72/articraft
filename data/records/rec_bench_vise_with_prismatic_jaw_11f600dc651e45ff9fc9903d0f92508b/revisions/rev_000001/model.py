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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jaw_in_jaw_drill_vise")

    cast_iron = model.material("blued_cast_iron", rgba=(0.10, 0.13, 0.15, 1.0))
    dark_iron = model.material("darkened_cast_iron", rgba=(0.045, 0.052, 0.055, 1.0))
    rail_steel = model.material("ground_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    jaw_steel = model.material("hardened_jaw_steel", rgba=(0.42, 0.44, 0.43, 1.0))
    bright_steel = model.material("polished_leadscrew", rgba=(0.78, 0.78, 0.74, 1.0))
    black = model.material("black_recess", rgba=(0.01, 0.011, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.48, 0.22, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.260, 0.024, 0.014)),
        origin=Origin(xyz=(-0.075, -0.075, 0.037)),
        material=rail_steel,
        name="guide_rail_neg",
    )
    base.visual(
        Box((0.260, 0.024, 0.014)),
        origin=Origin(xyz=(-0.075, 0.075, 0.037)),
        material=rail_steel,
        name="guide_rail_pos",
    )
    for y in (-0.075, 0.075):
        base.visual(
            Box((0.130, 0.030, 0.003)),
            origin=Origin(xyz=(0.135, y, 0.0295)),
            material=black,
            name=f"mount_slot_{'neg' if y < 0 else 'pos'}_rear",
        )
        base.visual(
            Box((0.120, 0.030, 0.003)),
            origin=Origin(xyz=(-0.150, y, 0.0295)),
            material=black,
            name=f"mount_slot_{'neg' if y < 0 else 'pos'}_front",
        )
    base.visual(
        Box((0.065, 0.030, 0.010)),
        origin=Origin(xyz=(-0.225, 0.0, 0.035)),
        material=dark_iron,
        name="front_lip",
    )

    rear_jaw = model.part("rear_jaw")
    rear_jaw.visual(
        Box((0.090, 0.045, 0.105)),
        origin=Origin(xyz=(0.0, -0.0625, 0.0525)),
        material=cast_iron,
        name="rear_side_neg",
    )
    rear_jaw.visual(
        Box((0.090, 0.045, 0.105)),
        origin=Origin(xyz=(0.0, 0.0625, 0.0525)),
        material=cast_iron,
        name="rear_side_pos",
    )
    rear_jaw.visual(
        Box((0.090, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=cast_iron,
        name="rear_top_web",
    )
    rear_jaw.visual(
        Box((0.090, 0.080, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_iron,
        name="rear_bottom_web",
    )
    rear_jaw.visual(
        Box((0.112, 0.188, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, 0.009)),
        material=dark_iron,
        name="rear_foot",
    )
    rear_jaw.visual(
        Box((0.010, 0.140, 0.055)),
        origin=Origin(xyz=(-0.050, 0.0, 0.075)),
        material=jaw_steel,
        name="rear_jaw_plate",
    )
    for i, z in enumerate((0.052, 0.061, 0.070, 0.079, 0.088, 0.097)):
        rear_jaw.visual(
            Box((0.006, 0.132, 0.0025)),
            origin=Origin(xyz=(-0.057, 0.0, z)),
            material=bright_steel,
            name=f"rear_tooth_{i}",
        )
    rear_jaw.visual(
        Box((0.007, 0.010, 0.055)),
        origin=Origin(xyz=(-0.0585, -0.014, 0.075), rpy=(0.0, 0.0, math.radians(28))),
        material=black,
        name="rear_v_groove_a",
    )
    rear_jaw.visual(
        Box((0.007, 0.010, 0.055)),
        origin=Origin(xyz=(-0.0585, 0.014, 0.075), rpy=(0.0, 0.0, math.radians(-28))),
        material=black,
        name="rear_v_groove_b",
    )
    rear_jaw.visual(
        Cylinder(radius=0.018, length=0.046),
        origin=Origin(xyz=(-0.055, 0.0, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="rear_nut_boss",
    )
    rear_jaw.visual(
        Box((0.046, 0.026, 0.012)),
        origin=Origin(xyz=(-0.055, -0.027, 0.038)),
        material=dark_iron,
        name="rear_nut_tab_neg",
    )
    rear_jaw.visual(
        Box((0.046, 0.026, 0.012)),
        origin=Origin(xyz=(-0.055, 0.027, 0.038)),
        material=dark_iron,
        name="rear_nut_tab_pos",
    )
    for y in (-0.064, 0.064):
        rear_jaw.visual(
            Cylinder(radius=0.009, length=0.006),
            origin=Origin(xyz=(0.025, y, 0.021)),
            material=bright_steel,
            name=f"rear_cap_screw_{'neg' if y < 0 else 'pos'}",
        )
    model.articulation(
        "base_to_rear_jaw",
        ArticulationType.FIXED,
        parent=base,
        child=rear_jaw,
        origin=Origin(xyz=(0.130, 0.0, 0.030)),
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        Box((0.100, 0.172, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, 0.009)),
        material=dark_iron,
        name="front_saddle",
    )
    front_jaw.visual(
        Box((0.075, 0.158, 0.075)),
        origin=Origin(xyz=(0.032, 0.0, 0.0775)),
        material=cast_iron,
        name="front_block",
    )
    front_jaw.visual(
        Box((0.010, 0.132, 0.055)),
        origin=Origin(xyz=(0.0745, 0.0, 0.061)),
        material=jaw_steel,
        name="front_jaw_plate",
    )
    for i, z in enumerate((0.038, 0.047, 0.056, 0.065, 0.074, 0.083)):
        front_jaw.visual(
            Box((0.006, 0.124, 0.0025)),
            origin=Origin(xyz=(0.082, 0.0, z)),
            material=bright_steel,
            name=f"front_tooth_{i}",
        )
    front_jaw.visual(
        Box((0.007, 0.010, 0.052)),
        origin=Origin(xyz=(0.083, -0.014, 0.061), rpy=(0.0, 0.0, math.radians(-28))),
        material=black,
        name="front_v_groove_a",
    )
    front_jaw.visual(
        Box((0.007, 0.010, 0.052)),
        origin=Origin(xyz=(0.083, 0.014, 0.061), rpy=(0.0, 0.0, math.radians(28))),
        material=black,
        name="front_v_groove_b",
    )
    front_jaw.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(-0.055, 0.0, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="screw_boss",
    )
    front_jaw.visual(
        Box((0.095, 0.018, 0.006)),
        origin=Origin(xyz=(0.000, -0.058, 0.003)),
        material=jaw_steel,
        name="slide_wear_strip_neg",
    )
    front_jaw.visual(
        Box((0.060, 0.012, 0.018)),
        origin=Origin(xyz=(0.030, -0.058, 0.124)),
        material=rail_steel,
        name="top_slide_rail_neg",
    )
    front_jaw.visual(
        Box((0.095, 0.018, 0.006)),
        origin=Origin(xyz=(0.000, 0.058, 0.003)),
        material=jaw_steel,
        name="slide_wear_strip_pos",
    )
    front_jaw.visual(
        Box((0.060, 0.012, 0.018)),
        origin=Origin(xyz=(0.030, 0.058, 0.124)),
        material=rail_steel,
        name="top_slide_rail_pos",
    )
    for y, name in ((-0.073, "side_cheek_neg"), (0.073, "side_cheek_pos")):
        front_jaw.visual(
            Box((0.060, 0.012, 0.022)),
            origin=Origin(xyz=(0.032, y, 0.029)),
            material=cast_iron,
            name=name,
        )
    model.articulation(
        "base_to_front_jaw",
        ArticulationType.PRISMATIC,
        parent=base,
        child=front_jaw,
        origin=Origin(xyz=(-0.145, 0.0, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.120, effort=450.0, velocity=0.08),
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.007, length=0.330),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="leadscrew",
    )
    for i in range(10):
        screw_handle.visual(
            Cylinder(radius=0.009, length=0.0035),
            origin=Origin(
                xyz=(0.025 + i * 0.025, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rail_steel,
            name=f"thread_crest_{i}",
        )
    screw_handle.visual(
        Cylinder(radius=0.013, length=0.028),
        origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="handle_hub",
    )
    screw_handle.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(-0.027, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="t_bar",
    )
    for y in (-0.064, 0.064):
        screw_handle.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(-0.027, y, 0.0)),
            material=bright_steel,
            name=f"bar_end_{'neg' if y < 0 else 'pos'}",
        )
    model.articulation(
        "front_jaw_to_screw",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=screw_handle,
        origin=Origin(xyz=(-0.095, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=8.0),
    )

    top_jaw = model.part("top_jaw")
    top_jaw.visual(
        Box((0.060, 0.104, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, 0.006)),
        material=dark_iron,
        name="top_slide_shoe",
    )
    top_jaw.visual(
        Box((0.046, 0.084, 0.034)),
        origin=Origin(xyz=(0.019, 0.0, 0.029)),
        material=cast_iron,
        name="top_block",
    )
    top_jaw.visual(
        Box((0.008, 0.070, 0.028)),
        origin=Origin(xyz=(0.046, 0.0, 0.029)),
        material=jaw_steel,
        name="top_jaw_plate",
    )
    top_jaw.visual(
        Box((0.006, 0.009, 0.028)),
        origin=Origin(xyz=(0.052, -0.012, 0.029), rpy=(0.0, 0.0, math.radians(-30))),
        material=black,
        name="top_v_groove_a",
    )
    top_jaw.visual(
        Box((0.006, 0.009, 0.028)),
        origin=Origin(xyz=(0.052, 0.012, 0.029), rpy=(0.0, 0.0, math.radians(30))),
        material=black,
        name="top_v_groove_b",
    )
    for y in (-0.053, 0.053):
        top_jaw.visual(
            Box((0.045, 0.010, 0.018)),
            origin=Origin(xyz=(0.005, y, 0.021)),
            material=rail_steel,
            name=f"top_gib_{'neg' if y < 0 else 'pos'}",
        )
    model.articulation(
        "front_jaw_to_top_jaw",
        ArticulationType.PRISMATIC,
        parent=front_jaw,
        child=top_jaw,
        origin=Origin(xyz=(0.010, 0.0, 0.133)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.040, effort=120.0, velocity=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_slide = object_model.get_articulation("base_to_front_jaw")
    top_slide = object_model.get_articulation("front_jaw_to_top_jaw")
    screw_spin = object_model.get_articulation("front_jaw_to_screw")

    ctx.allow_overlap(
        "front_jaw",
        "screw_handle",
        elem_a="screw_boss",
        elem_b="leadscrew",
        reason="The leadscrew is intentionally captured through the moving jaw's screw boss.",
    )
    ctx.allow_overlap(
        "rear_jaw",
        "screw_handle",
        elem_a="rear_nut_boss",
        elem_b="leadscrew",
        reason="The leadscrew intentionally enters the fixed rear nut boss as the drive screw.",
    )

    ctx.expect_contact(
        "rear_jaw",
        "base",
        elem_a="rear_foot",
        elem_b="base_plate",
        name="fixed rear jaw foot sits on the base plate",
    )
    ctx.expect_contact(
        "front_jaw",
        "base",
        elem_a="slide_wear_strip_neg",
        elem_b="guide_rail_neg",
        contact_tol=0.001,
        name="front jaw rides on one flat guide rail",
    )
    ctx.expect_contact(
        "front_jaw",
        "base",
        elem_a="slide_wear_strip_pos",
        elem_b="guide_rail_pos",
        contact_tol=0.001,
        name="front jaw rides on the other flat guide rail",
    )
    ctx.expect_contact(
        "top_jaw",
        "front_jaw",
        elem_a="top_slide_shoe",
        elem_b="top_slide_rail_neg",
        contact_tol=0.001,
        name="auxiliary top jaw rides on the front jaw slide",
    )
    ctx.expect_within(
        "screw_handle",
        "front_jaw",
        axes="yz",
        inner_elem="leadscrew",
        outer_elem="screw_boss",
        margin=0.002,
        name="leadscrew centered in moving jaw boss",
    )
    ctx.expect_overlap(
        "screw_handle",
        "front_jaw",
        axes="x",
        elem_a="leadscrew",
        elem_b="screw_boss",
        min_overlap=0.030,
        name="leadscrew retained through moving jaw boss",
    )
    ctx.expect_overlap(
        "screw_handle",
        "rear_jaw",
        axes="x",
        elem_a="leadscrew",
        elem_b="rear_nut_boss",
        min_overlap=0.006,
        name="leadscrew reaches fixed rear nut",
    )

    rest_front = ctx.part_world_position("front_jaw")
    with ctx.pose({front_slide: 0.10}):
        closed_front = ctx.part_world_position("front_jaw")
        ctx.expect_gap(
            "rear_jaw",
            "front_jaw",
            axis="x",
            positive_elem="rear_jaw_plate",
            negative_elem="front_jaw_plate",
            min_gap=0.015,
            max_gap=0.070,
            name="primary jaw closes toward fixed rear jaw without collision",
        )
    ctx.check(
        "primary prismatic motion advances the jaw",
        rest_front is not None and closed_front is not None and closed_front[0] > rest_front[0] + 0.08,
        details=f"rest={rest_front}, closed={closed_front}",
    )

    rest_top = ctx.part_world_position("top_jaw")
    with ctx.pose({top_slide: 0.030}):
        advanced_top = ctx.part_world_position("top_jaw")
    ctx.check(
        "auxiliary top jaw advances on secondary slide",
        rest_top is not None and advanced_top is not None and advanced_top[0] > rest_top[0] + 0.020,
        details=f"rest={rest_top}, advanced={advanced_top}",
    )
    ctx.check(
        "t-bar handle spins about the leadscrew axis",
        tuple(round(v, 3) for v in screw_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={screw_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
