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
    model = ArticulatedObject(name="hinge_slide_hinge_chain")

    model.material("base_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("rail_blue", rgba=(0.12, 0.26, 0.45, 1.0))
    model.material("slide_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("pin_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    model.material("tab_orange", rgba=(0.93, 0.50, 0.14, 1.0))
    model.material("rubber_black", rgba=(0.035, 0.035, 0.04, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.24, 0.16, 0.024)),
        origin=Origin(xyz=(0.035, 0.0, 0.012)),
        material="base_dark",
        name="base_plate",
    )
    for y in (-0.049, 0.049):
        base.visual(
            Box((0.050, 0.014, 0.112)),
            origin=Origin(xyz=(0.0, y, 0.078)),
            material="base_dark",
            name=f"hinge_cheek_{'neg' if y < 0 else 'pos'}",
        )
    base.visual(
        Cylinder(radius=0.007, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.120), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="root_pin",
    )
    for x in (-0.060, 0.120):
        for y in (-0.055, 0.055):
            base.visual(
                Cylinder(radius=0.007, length=0.004),
                origin=Origin(xyz=(x, y, 0.026)),
                material="pin_steel",
                name=f"bolt_{x}_{y}",
            )

    pivot_link = model.part("pivot_link")
    pivot_link.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="rail_blue",
        name="root_barrel",
    )
    pivot_link.visual(
        Box((0.320, 0.055, 0.030)),
        origin=Origin(xyz=(0.170, 0.0, -0.015)),
        material="rail_blue",
        name="rail_body",
    )
    pivot_link.visual(
        Box((0.250, 0.010, 0.008)),
        origin=Origin(xyz=(0.190, 0.030, 0.004)),
        material="pin_steel",
        name="guide_rail_0",
    )
    pivot_link.visual(
        Box((0.250, 0.010, 0.008)),
        origin=Origin(xyz=(0.190, -0.030, 0.004)),
        material="pin_steel",
        name="guide_rail_1",
    )
    for y in (-0.034, 0.034):
        pivot_link.visual(
            Box((0.034, 0.014, 0.014)),
            origin=Origin(xyz=(0.315, y, 0.002)),
            material="rail_blue",
            name=f"end_stop_{'neg' if y < 0 else 'pos'}",
        )

    slide_member = model.part("slide_member")
    slide_member.visual(
        Box((0.260, 0.038, 0.022)),
        origin=Origin(xyz=(0.130, 0.0, 0.011)),
        material="slide_aluminum",
        name="slide_bar",
    )
    for y in (-0.024, 0.024):
        slide_member.visual(
            Box((0.044, 0.012, 0.044)),
            origin=Origin(xyz=(0.276, y, 0.011)),
            material="slide_aluminum",
            name=f"tip_clevis_{'neg' if y < 0 else 'pos'}",
        )
    slide_member.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(0.276, 0.0, 0.011), rpy=(pi / 2.0, 0.0, 0.0)),
        material="pin_steel",
        name="tip_pin",
    )
    slide_member.visual(
        Box((0.030, 0.043, 0.006)),
        origin=Origin(xyz=(0.032, 0.0, 0.025)),
        material="rubber_black",
        name="slide_grip_pad",
    )

    tip_tab = model.part("tip_tab")
    tip_tab.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="tab_orange",
        name="tip_hub",
    )
    tip_tab.visual(
        Box((0.092, 0.022, 0.010)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material="tab_orange",
        name="tab_neck",
    )
    for y in (-0.015, 0.015):
        tip_tab.visual(
            Box((0.045, 0.012, 0.010)),
            origin=Origin(xyz=(0.116, y, 0.0)),
            material="tab_orange",
            name=f"fork_tine_{'neg' if y < 0 else 'pos'}",
        )

    model.articulation(
        "root_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pivot_link,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-0.55, upper=1.05),
    )
    model.articulation(
        "middle_slide",
        ArticulationType.PRISMATIC,
        parent=pivot_link,
        child=slide_member,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.30, lower=0.0, upper=0.16),
    )
    model.articulation(
        "tip_hinge",
        ArticulationType.REVOLUTE,
        parent=slide_member,
        child=tip_tab,
        origin=Origin(xyz=(0.276, 0.0, 0.011)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.75, upper=1.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_articulation("root_hinge")
    slide = object_model.get_articulation("middle_slide")
    tip = object_model.get_articulation("tip_hinge")
    base = object_model.get_part("base")
    pivot_link = object_model.get_part("pivot_link")
    slide_member = object_model.get_part("slide_member")
    tip_tab = object_model.get_part("tip_tab")

    ctx.check(
        "hinge-slide-hinge topology",
        root.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC
        and tip.articulation_type == ArticulationType.REVOLUTE
        and root.parent == "base"
        and root.child == "pivot_link"
        and slide.parent == "pivot_link"
        and slide.child == "slide_member"
        and tip.parent == "slide_member"
        and tip.child == "tip_tab",
        details="Expected base -> revolute -> pivot_link -> prismatic -> slide_member -> revolute -> tip_tab.",
    )

    ctx.allow_overlap(
        base,
        pivot_link,
        elem_a="root_pin",
        elem_b="root_barrel",
        reason="The visible root hinge pin is intentionally captured inside the rotating barrel.",
    )
    ctx.expect_within(
        base,
        pivot_link,
        axes="xz",
        inner_elem="root_pin",
        outer_elem="root_barrel",
        margin=0.001,
        name="root pin sits inside barrel cross-section",
    )
    ctx.expect_overlap(
        base,
        pivot_link,
        axes="y",
        elem_a="root_pin",
        elem_b="root_barrel",
        min_overlap=0.030,
        name="root pin passes through hinge barrel",
    )

    ctx.allow_overlap(
        slide_member,
        tip_tab,
        elem_a="tip_pin",
        elem_b="tip_hub",
        reason="The distal fork hinge pin is intentionally captured inside the tab hub.",
    )
    ctx.expect_within(
        slide_member,
        tip_tab,
        axes="xz",
        inner_elem="tip_pin",
        outer_elem="tip_hub",
        margin=0.001,
        name="tip pin sits inside tab hub cross-section",
    )
    ctx.expect_overlap(
        slide_member,
        tip_tab,
        axes="y",
        elem_a="tip_pin",
        elem_b="tip_hub",
        min_overlap=0.025,
        name="tip pin passes through tab hub",
    )

    ctx.expect_gap(
        slide_member,
        pivot_link,
        axis="z",
        positive_elem="slide_bar",
        negative_elem="rail_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="middle slide rides on pivot rail",
    )
    ctx.expect_overlap(
        slide_member,
        pivot_link,
        axes="x",
        elem_a="slide_bar",
        elem_b="rail_body",
        min_overlap=0.18,
        name="middle slide retained on rail at rest",
    )

    rest_slide_pos = ctx.part_world_position(slide_member)
    with ctx.pose({slide: 0.16}):
        ctx.expect_gap(
            slide_member,
            pivot_link,
            axis="z",
            positive_elem="slide_bar",
            negative_elem="rail_body",
            max_gap=0.001,
            max_penetration=0.0,
            name="extended slide still rides on rail",
        )
        ctx.expect_overlap(
            slide_member,
            pivot_link,
            axes="x",
            elem_a="slide_bar",
            elem_b="rail_body",
            min_overlap=0.09,
            name="extended slide keeps retained insertion",
        )
        extended_slide_pos = ctx.part_world_position(slide_member)
    ctx.check(
        "prismatic joint extends middle member",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[0] > rest_slide_pos[0] + 0.14,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    rest_tip_aabb = ctx.part_world_aabb(tip_tab)
    with ctx.pose({tip: 1.0}):
        raised_tip_aabb = ctx.part_world_aabb(tip_tab)
    ctx.check(
        "tip hinge raises fork with positive rotation",
        rest_tip_aabb is not None
        and raised_tip_aabb is not None
        and raised_tip_aabb[1][2] > rest_tip_aabb[1][2] + 0.05,
        details=f"rest={rest_tip_aabb}, raised={raised_tip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
