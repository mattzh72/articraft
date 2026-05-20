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
    model = ArticulatedObject(name="under_slung_hinge_slide_unit")

    plated_steel = Material("zinc_plated_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = Material("black_oxide_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    bearing_steel = Material("polished_bearing_steel", rgba=(0.78, 0.80, 0.78, 1.0))
    safety_orange = Material("orange_slider_carriage", rgba=(0.95, 0.32, 0.08, 1.0))

    support = model.part("top_support")
    support.visual(
        Box((0.36, 0.24, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=plated_steel,
        name="top_plate",
    )
    support.visual(
        Box((0.070, 0.030, 0.155)),
        origin=Origin(xyz=(0.0, 0.100, -0.0575)),
        material=plated_steel,
        name="hinge_ear_0",
    )
    support.visual(
        Box((0.070, 0.030, 0.155)),
        origin=Origin(xyz=(0.0, -0.100, -0.0575)),
        material=plated_steel,
        name="hinge_ear_1",
    )
    support.visual(
        Cylinder(radius=0.044, length=0.026),
        origin=Origin(xyz=(0.0, 0.100, -0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=plated_steel,
        name="outer_boss_0",
    )
    support.visual(
        Cylinder(radius=0.044, length=0.026),
        origin=Origin(xyz=(0.0, -0.100, -0.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plated_steel,
        name="outer_boss_1",
    )
    support.visual(
        Cylinder(radius=0.014, length=0.224),
        origin=Origin(xyz=(0.0, 0.0, -0.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    for i, (x, y) in enumerate(((-0.125, -0.075), (-0.125, 0.075), (0.125, -0.075), (0.125, 0.075))):
        support.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, y, 0.040)),
            material=dark_steel,
            name=f"bolt_head_{i}",
        )

    arm = model.part("hanging_arm")
    arm.visual(
        Cylinder(radius=0.032, length=0.134),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle",
    )
    arm.visual(
        Box((0.046, 0.054, 0.360)),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=dark_steel,
        name="drop_link",
    )
    arm.visual(
        Box((0.078, 0.076, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.372)),
        material=dark_steel,
        name="tip_saddle",
    )
    arm.visual(
        Box((0.240, 0.018, 0.074)),
        origin=Origin(xyz=(0.0, 0.059, -0.430)),
        material=bearing_steel,
        name="guide_rail_0",
    )
    arm.visual(
        Box((0.240, 0.018, 0.074)),
        origin=Origin(xyz=(0.0, -0.059, -0.430)),
        material=bearing_steel,
        name="guide_rail_1",
    )
    arm.visual(
        Box((0.240, 0.118, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.387)),
        material=dark_steel,
        name="guide_top",
    )
    arm.visual(
        Box((0.240, 0.118, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.473)),
        material=dark_steel,
        name="guide_bottom",
    )
    arm.visual(
        Box((0.018, 0.118, 0.090)),
        origin=Origin(xyz=(-0.129, 0.0, -0.430)),
        material=dark_steel,
        name="rear_stop",
    )

    slider = model.part("tip_slider")
    slider.visual(
        Box((0.180, 0.070, 0.035)),
        origin=Origin(xyz=(0.030, 0.0, 0.0165)),
        material=safety_orange,
        name="slide_bar",
    )
    slider.visual(
        Box((0.040, 0.086, 0.056)),
        origin=Origin(xyz=(0.136, 0.0, 0.0)),
        material=safety_orange,
        name="end_carriage",
    )
    slider.visual(
        Cylinder(radius=0.014, length=0.094),
        origin=Origin(xyz=(0.143, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cross_pin",
    )

    model.articulation(
        "support_to_arm",
        ArticulationType.REVOLUTE,
        parent=support,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, -0.430)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.100),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    arm = object_model.get_part("hanging_arm")
    slider = object_model.get_part("tip_slider")
    hinge = object_model.get_articulation("support_to_arm")
    slide = object_model.get_articulation("arm_to_slider")

    ctx.allow_overlap(
        support,
        arm,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        reason="The support pin is intentionally captured inside the arm hinge knuckle.",
    )

    ctx.check(
        "arm uses revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "tip slider uses prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.expect_gap(
        support,
        arm,
        axis="z",
        min_gap=0.050,
        positive_elem="top_plate",
        negative_elem="hinge_knuckle",
        name="hinge knuckle hangs below top plate",
    )
    ctx.expect_within(
        arm,
        support,
        axes="y",
        margin=0.004,
        inner_elem="hinge_knuckle",
        outer_elem="top_plate",
        name="hinge knuckle sits between support ears",
    )
    ctx.expect_within(
        support,
        arm,
        axes="xz",
        margin=0.001,
        inner_elem="hinge_pin",
        outer_elem="hinge_knuckle",
        name="captured hinge pin fits inside knuckle bore",
    )
    ctx.expect_overlap(
        support,
        arm,
        axes="y",
        min_overlap=0.120,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        name="hinge pin spans the knuckle",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            slider,
            arm,
            axes="x",
            min_overlap=0.160,
            elem_a="slide_bar",
            elem_b="guide_top",
            name="retracted slider is carried by guide",
        )
        ctx.expect_within(
            slider,
            arm,
            axes="y",
            margin=0.004,
            inner_elem="slide_bar",
            outer_elem="guide_top",
            name="slider is centered between side rails",
        )
        ctx.expect_contact(
            arm,
            slider,
            elem_a="guide_top",
            elem_b="slide_bar",
            contact_tol=0.001,
            name="top bearing rail carries sliding bar",
        )
        ctx.expect_gap(
            slider,
            arm,
            axis="z",
            min_gap=0.030,
            max_gap=0.040,
            positive_elem="slide_bar",
            negative_elem="guide_bottom",
            name="lower rail leaves slide clearance",
        )
        rest_slider_position = ctx.part_world_position(slider)

    with ctx.pose({slide: 0.100}):
        ctx.expect_overlap(
            slider,
            arm,
            axes="x",
            min_overlap=0.075,
            elem_a="slide_bar",
            elem_b="guide_top",
            name="extended slider remains retained in guide",
        )
        extended_slider_position = ctx.part_world_position(slider)

    ctx.check(
        "slider extends along arm tip",
        rest_slider_position is not None
        and extended_slider_position is not None
        and extended_slider_position[0] > rest_slider_position[0] + 0.080,
        details=f"rest={rest_slider_position}, extended={extended_slider_position}",
    )

    with ctx.pose({hinge: 0.75, slide: 0.0}):
        swung_slider_aabb = ctx.part_world_aabb(slider)
    with ctx.pose({hinge: 0.0, slide: 0.0}):
        vertical_slider_aabb = ctx.part_world_aabb(slider)
    swung_center_x = None
    vertical_center_x = None
    if swung_slider_aabb is not None and vertical_slider_aabb is not None:
        swung_center_x = (swung_slider_aabb[0][0] + swung_slider_aabb[1][0]) / 2.0
        vertical_center_x = (vertical_slider_aabb[0][0] + vertical_slider_aabb[1][0]) / 2.0
    ctx.check(
        "revolute arm swings under support",
        swung_center_x is not None
        and vertical_center_x is not None
        and swung_center_x < vertical_center_x - 0.20,
        details=f"vertical_x={vertical_center_x}, swung_x={swung_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
