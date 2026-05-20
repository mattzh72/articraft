from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _annular_collar(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Centered hollow cylindrical bearing/collar, with its bore along local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_inspection_fixture_head")

    cast_iron = model.material("dark_cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    parkerized = model.material("parkerized_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    blue_steel = model.material("blue_anodized_arms", rgba=(0.05, 0.17, 0.33, 1.0))
    satin_steel = model.material("satin_steel_faces", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("black_rubber_inserts", rgba=(0.01, 0.01, 0.012, 1.0))
    brass = model.material("brass_index_marks", rgba=(0.83, 0.64, 0.25, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.080, 0.070, 0.450)),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=cast_iron,
        name="rectangular_spine",
    )
    spine.visual(
        Box((0.096, 0.086, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=cast_iron,
        name="bottom_spine_cap",
    )
    spine.visual(
        Box((0.096, 0.086, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.493)),
        material=cast_iron,
        name="top_spine_cap",
    )

    # Low transverse trunnion housing: two cheek plates on a separate side collar.
    spine.visual(
        Box((0.045, 0.145, 0.070)),
        origin=Origin(xyz=(0.055, 0.0, 0.155)),
        material=cast_iron,
        name="low_trunnion_neck",
    )
    for suffix, y in (("0", 0.057), ("1", -0.057)):
        spine.visual(
            Box((0.075, 0.025, 0.085)),
            origin=Origin(xyz=(0.108, y, 0.155)),
            material=cast_iron,
            name=f"low_trunnion_cheek_{suffix}",
        )
        spine.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(xyz=(0.112, y + (0.015 if y > 0 else -0.015), 0.155), rpy=(math.pi / 2, 0.0, 0.0)),
            material=parkerized,
            name=f"low_bore_boss_{suffix}",
        )

    # Forward vertical swivel collar: a real annular bearing on its own nose.
    spine.visual(
        Box((0.090, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, -0.052, 0.285)),
        material=cast_iron,
        name="forward_collar_neck",
    )
    spine.visual(
        mesh_from_cadquery(_annular_collar(0.038, 0.018, 0.032), "forward_vertical_collar"),
        origin=Origin(xyz=(0.0, -0.105, 0.285)),
        material=cast_iron,
        name="forward_vertical_collar",
    )

    # High longitudinal bearing: a separate horizontal collar, not a shared hub.
    spine.visual(
        Box((0.035, 0.080, 0.070)),
        origin=Origin(xyz=(0.054, 0.0, 0.415)),
        material=cast_iron,
        name="high_collar_neck",
    )
    spine.visual(
        mesh_from_cadquery(_annular_collar(0.035, 0.017, 0.052), "high_longitudinal_collar"),
        origin=Origin(xyz=(0.083, 0.0, 0.415), rpy=(0.0, math.pi / 2, 0.0)),
        material=cast_iron,
        name="high_longitudinal_collar",
    )

    low_arm = model.part("low_arm")
    low_arm.visual(
        Cylinder(radius=0.017, length=0.089),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=parkerized,
        name="low_pivot_barrel",
    )
    low_arm.visual(
        Box((0.295, 0.032, 0.024)),
        origin=Origin(xyz=(0.1595, 0.0, 0.0)),
        material=blue_steel,
        name="low_side_arm_tube",
    )
    low_arm.visual(
        Box((0.030, 0.115, 0.065)),
        origin=Origin(xyz=(0.322, 0.0, 0.0)),
        material=satin_steel,
        name="low_slotted_pad",
    )
    low_arm.visual(
        Box((0.008, 0.088, 0.014)),
        origin=Origin(xyz=(0.341, 0.0, 0.0)),
        material=rubber,
        name="low_slot_strip",
    )
    for suffix, y in (("0", 0.035), ("1", -0.035)):
        low_arm.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(0.347, y, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            material=rubber,
            name=f"low_slot_end_{suffix}",
        )

    forward_arm = model.part("forward_arm")
    forward_arm.visual(
        Cylinder(radius=0.0125, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=parkerized,
        name="forward_swivel_pin",
    )
    forward_arm.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=blue_steel,
        name="forward_turntable_cap",
    )
    forward_arm.visual(
        Box((0.035, 0.276, 0.025)),
        origin=Origin(xyz=(0.0, -0.158, 0.040)),
        material=blue_steel,
        name="forward_arm_tube",
    )
    forward_arm.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, -0.306, 0.040), rpy=(math.pi / 2, 0.0, 0.0)),
        material=satin_steel,
        name="forward_round_pad",
    )
    for i, (x, z) in enumerate(((0.0, 0.076), (0.030, 0.022), (-0.030, 0.022))):
        forward_arm.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, -0.318, z), rpy=(math.pi / 2, 0.0, 0.0)),
            material=rubber,
            name=f"forward_pad_bolt_{i}",
        )

    high_arm = model.part("high_arm")
    high_arm.visual(
        Cylinder(radius=0.013, length=0.090),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=parkerized,
        name="high_shaft",
    )
    high_arm.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=parkerized,
        name="high_bearing_shoulder",
    )
    high_arm.visual(
        Box((0.260, 0.026, 0.034)),
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        material=blue_steel,
        name="high_arm_tube",
    )
    high_arm.visual(
        Box((0.060, 0.018, 0.012)),
        origin=Origin(xyz=(0.275, 0.0, 0.022)),
        material=brass,
        name="high_orientation_mark",
    )
    high_arm.visual(
        Box((0.038, 0.110, 0.075)),
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        material=satin_steel,
        name="high_clamp_pad",
    )
    for suffix, y in (("0", 0.030), ("1", -0.030)):
        high_arm.visual(
            Box((0.008, 0.014, 0.060)),
            origin=Origin(xyz=(0.363, y, 0.0)),
            material=rubber,
            name=f"high_clamp_jaw_{suffix}",
        )

    model.articulation(
        "spine_to_low_arm",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=low_arm,
        origin=Origin(xyz=(0.112, 0.0, 0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.70, upper=0.90),
    )
    model.articulation(
        "spine_to_forward_arm",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=forward_arm,
        origin=Origin(xyz=(0.0, -0.105, 0.285)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "spine_to_high_arm",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=high_arm,
        origin=Origin(xyz=(0.083, 0.0, 0.415)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.60, upper=1.60),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    spine = object_model.get_part("spine")
    low_arm = object_model.get_part("low_arm")
    forward_arm = object_model.get_part("forward_arm")
    high_arm = object_model.get_part("high_arm")
    low_joint = object_model.get_articulation("spine_to_low_arm")
    forward_joint = object_model.get_articulation("spine_to_forward_arm")
    high_joint = object_model.get_articulation("spine_to_high_arm")

    ctx.expect_within(
        low_arm,
        spine,
        axes="yz",
        margin=0.004,
        inner_elem="low_pivot_barrel",
        name="low barrel sits between transverse trunnion cheeks",
    )
    ctx.expect_overlap(
        forward_arm,
        spine,
        axes="z",
        min_overlap=0.025,
        elem_a="forward_swivel_pin",
        elem_b="forward_vertical_collar",
        name="forward vertical pin passes through its own collar",
    )
    ctx.expect_within(
        forward_arm,
        spine,
        axes="xy",
        margin=0.004,
        inner_elem="forward_swivel_pin",
        outer_elem="forward_vertical_collar",
        name="forward pin is centered in the vertical collar footprint",
    )
    ctx.expect_overlap(
        high_arm,
        spine,
        axes="x",
        min_overlap=0.030,
        elem_a="high_shaft",
        elem_b="high_longitudinal_collar",
        name="high shaft is retained through longitudinal collar",
    )
    ctx.expect_within(
        high_arm,
        spine,
        axes="yz",
        margin=0.004,
        inner_elem="high_shaft",
        outer_elem="high_longitudinal_collar",
        name="high shaft is centered in longitudinal collar bore",
    )

    low_pad_rest = ctx.part_element_world_aabb(low_arm, elem="low_slotted_pad")
    with ctx.pose({low_joint: -0.45}):
        low_pad_raised = ctx.part_element_world_aabb(low_arm, elem="low_slotted_pad")
    ctx.check(
        "low arm pitches about transverse axis",
        low_pad_rest is not None
        and low_pad_raised is not None
        and (low_pad_raised[0][2] + low_pad_raised[1][2]) * 0.5 > (low_pad_rest[0][2] + low_pad_rest[1][2]) * 0.5 + 0.055,
        details=f"rest={low_pad_rest}, raised={low_pad_raised}",
    )

    forward_pad_rest = ctx.part_element_world_aabb(forward_arm, elem="forward_round_pad")
    with ctx.pose({forward_joint: 0.80}):
        forward_pad_swung = ctx.part_element_world_aabb(forward_arm, elem="forward_round_pad")
    ctx.check(
        "forward arm yaws about vertical stub",
        forward_pad_rest is not None
        and forward_pad_swung is not None
        and (forward_pad_swung[0][0] + forward_pad_swung[1][0]) * 0.5 > (forward_pad_rest[0][0] + forward_pad_rest[1][0]) * 0.5 + 0.16,
        details=f"rest={forward_pad_rest}, swung={forward_pad_swung}",
    )

    high_mark_rest = ctx.part_element_world_aabb(high_arm, elem="high_orientation_mark")
    with ctx.pose({high_joint: 1.00}):
        high_mark_rolled = ctx.part_element_world_aabb(high_arm, elem="high_orientation_mark")
    ctx.check(
        "high arm rolls about longitudinal shaft",
        high_mark_rest is not None
        and high_mark_rolled is not None
        and (high_mark_rolled[0][1] + high_mark_rolled[1][1]) * 0.5 < (high_mark_rest[0][1] + high_mark_rest[1][1]) * 0.5 - 0.012,
        details=f"rest={high_mark_rest}, rolled={high_mark_rolled}",
    )

    return ctx.report()


object_model = build_object_model()
