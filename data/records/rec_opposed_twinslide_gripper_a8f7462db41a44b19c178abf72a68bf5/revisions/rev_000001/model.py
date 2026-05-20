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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="opposed_twin_slide_gripper")

    aluminum = Material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black_oxide = Material("black_oxide", rgba=(0.015, 0.015, 0.018, 1.0))
    rubber = Material("matte_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    accent = Material("pickup_zone_mark", rgba=(0.95, 0.55, 0.08, 1.0))

    center_body = model.part("center_body")

    # A low grounded central body with a rear drive cover.  The front/center is
    # intentionally left low and open so the two jaw fingers share a clear
    # pickup zone as they close along X.
    center_body.visual(
        Box((0.26, 0.22, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=aluminum,
        name="base_plate",
    )
    center_body.visual(
        Box((0.14, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, 0.115, 0.0725)),
        material=aluminum,
        name="drive_cover",
    )
    center_body.visual(
        Box((0.11, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.050, 0.048)),
        material=accent,
        name="pickup_line",
    )

    # Opposed guideways, one to each side of the grounded body.
    for guidebed_name, rail_0_name, rail_1_name, bolt_0_name, bolt_1_name, x in (
        ("left_guidebed", "left_rail_0", "left_rail_1", "left_bolt_0", "left_bolt_1", -0.205),
        ("right_guidebed", "right_rail_0", "right_rail_1", "right_bolt_0", "right_bolt_1", 0.205),
    ):
        center_body.visual(
            Box((0.240, 0.120, 0.030)),
            origin=Origin(xyz=(x, 0.0, 0.059)),
            material=aluminum,
            name=guidebed_name,
        )
        center_body.visual(
            Box((0.185, 0.018, 0.021)),
            origin=Origin(xyz=(x - 0.010 if x < 0.0 else x + 0.010, 0.040, 0.0835)),
            material=dark_steel,
            name=rail_0_name,
        )
        center_body.visual(
            Box((0.185, 0.018, 0.021)),
            origin=Origin(xyz=(x - 0.010 if x < 0.0 else x + 0.010, -0.040, 0.0835)),
            material=dark_steel,
            name=rail_1_name,
        )
        center_body.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x - 0.070, 0.0, 0.075)),
            material=black_oxide,
            name=bolt_0_name,
        )
        center_body.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x + 0.070, 0.0, 0.075)),
            material=black_oxide,
            name=bolt_1_name,
        )

    center_body.visual(
        Box((0.026, 0.130, 0.070)),
        origin=Origin(xyz=(-0.338, 0.0, 0.080)),
        material=dark_steel,
        name="left_end_stop",
    )
    center_body.visual(
        Box((0.026, 0.130, 0.070)),
        origin=Origin(xyz=(0.338, 0.0, 0.080)),
        material=dark_steel,
        name="right_end_stop",
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        Box((0.075, 0.110, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.1215)),
        material=black_oxide,
        name="carriage_block",
    )
    left_jaw.visual(
        Box((0.095, 0.036, 0.040)),
        origin=Origin(xyz=(0.078, 0.0, 0.125)),
        material=black_oxide,
        name="jaw_arm",
    )
    left_jaw.visual(
        Box((0.022, 0.100, 0.120)),
        origin=Origin(xyz=(0.136, 0.0, 0.137)),
        material=black_oxide,
        name="jaw_finger",
    )
    left_jaw.visual(
        Box((0.007, 0.082, 0.084)),
        origin=Origin(xyz=(0.1495, 0.0, 0.137)),
        material=rubber,
        name="rubber_pad",
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        Box((0.075, 0.110, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.1215)),
        material=black_oxide,
        name="carriage_block",
    )
    right_jaw.visual(
        Box((0.095, 0.036, 0.040)),
        origin=Origin(xyz=(-0.078, 0.0, 0.125)),
        material=black_oxide,
        name="jaw_arm",
    )
    right_jaw.visual(
        Box((0.022, 0.100, 0.120)),
        origin=Origin(xyz=(-0.136, 0.0, 0.137)),
        material=black_oxide,
        name="jaw_finger",
    )
    right_jaw.visual(
        Box((0.007, 0.082, 0.084)),
        origin=Origin(xyz=(-0.1495, 0.0, 0.137)),
        material=rubber,
        name="rubber_pad",
    )

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=center_body,
        child=left_jaw,
        origin=Origin(xyz=(-0.235, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.065),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=center_body,
        child=right_jaw,
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.065),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_body = object_model.get_part("center_body")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

    ctx.check(
        "two separate prismatic jaw slides",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and right_slide.articulation_type == ArticulationType.PRISMATIC
        and left_slide.mimic is None
        and right_slide.mimic is None,
        details=f"left={left_slide.articulation_type}, right={right_slide.articulation_type}",
    )
    ctx.check(
        "slides share the same closing axis",
        tuple(left_slide.axis) == (1.0, 0.0, 0.0) and tuple(right_slide.axis) == (-1.0, 0.0, 0.0),
        details=f"left_axis={left_slide.axis}, right_axis={right_slide.axis}",
    )

    ctx.expect_gap(
        left_jaw,
        center_body,
        axis="z",
        positive_elem="carriage_block",
        negative_elem="left_rail_0",
        min_gap=0.0,
        max_gap=0.004,
        name="left carriage rides on guide rail",
    )
    ctx.expect_gap(
        right_jaw,
        center_body,
        axis="z",
        positive_elem="carriage_block",
        negative_elem="right_rail_0",
        min_gap=0.0,
        max_gap=0.004,
        name="right carriage rides on guide rail",
    )
    ctx.expect_overlap(
        right_jaw,
        left_jaw,
        axes="y",
        elem_a="rubber_pad",
        elem_b="rubber_pad",
        min_overlap=0.06,
        name="jaw pads face the same pickup zone",
    )
    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        positive_elem="rubber_pad",
        negative_elem="rubber_pad",
        min_gap=0.14,
        max_gap=0.18,
        name="open jaws straddle pickup zone",
    )

    left_rest = ctx.part_world_position(left_jaw)
    right_rest = ctx.part_world_position(right_jaw)
    with ctx.pose({left_slide: 0.065, right_slide: 0.065}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            positive_elem="rubber_pad",
            negative_elem="rubber_pad",
            min_gap=0.025,
            max_gap=0.050,
            name="closed jaws leave narrow pickup clearance",
        )
        left_closed = ctx.part_world_position(left_jaw)
        right_closed = ctx.part_world_position(right_jaw)

    ctx.check(
        "left jaw closes inward",
        left_rest is not None and left_closed is not None and left_closed[0] > left_rest[0] + 0.055,
        details=f"left_rest={left_rest}, left_closed={left_closed}",
    )
    ctx.check(
        "right jaw closes inward",
        right_rest is not None and right_closed is not None and right_closed[0] < right_rest[0] - 0.055,
        details=f"right_rest={right_rest}, right_closed={right_closed}",
    )

    return ctx.report()


object_model = build_object_model()
