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
    model = ArticulatedObject(name="adjustable_multi_hole_punch")

    base_dark = model.material("base_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    shell_steel = model.material("shell_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_red = model.material("accent_red", rgba=(0.73, 0.17, 0.14, 1.0))
    guide_gray = model.material("guide_gray", rgba=(0.62, 0.64, 0.67, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.34, 0.116, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=base_dark,
        name="base_plate",
    )
    body.visual(
        Box((0.30, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.050, 0.014)),
        material=shell_steel,
        name="front_fence",
    )
    body.visual(
        Box((0.246, 0.020, 0.056)),
        origin=Origin(xyz=(0.0, 0.024, 0.036)),
        material=shell_steel,
        name="rear_wall",
    )
    for x_pos, name in ((-0.118, "end_cheek_0"), (0.118, "end_cheek_1")):
        body.visual(
            Box((0.018, 0.056, 0.044)),
            origin=Origin(xyz=(x_pos, 0.004, 0.030)),
            material=shell_steel,
            name=name,
        )
    body.visual(
        Box((0.254, 0.056, 0.016)),
        origin=Origin(xyz=(0.0, 0.004, 0.060)),
        material=shell_steel,
        name="housing_top",
    )
    body.visual(
        Box((0.254, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, -0.016, 0.039)),
        material=shell_steel,
        name="front_lip",
    )
    body.visual(
        Box((0.210, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.036, 0.046)),
        material=shell_steel,
        name="selector_track",
    )
    for x_pos, name in ((-0.084, "hinge_cradle_0"), (0.084, "hinge_cradle_1")):
        body.visual(
            Box((0.038, 0.018, 0.012)),
            origin=Origin(xyz=(x_pos, 0.024, 0.057)),
            material=shell_steel,
            name=name,
        )
    for index, x_pos in enumerate((-0.084, 0.0, 0.084)):
        body.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(x_pos, -0.004, 0.033)),
            material=base_dark,
            name=f"die_collar_{index}",
        )
        body.visual(
            Box((0.018, 0.024, 0.003)),
            origin=Origin(xyz=(x_pos, -0.012, 0.0095)),
            material=base_dark,
            name=f"die_pad_{index}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.007, length=0.220),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="hinge_bar",
    )
    handle.visual(
        Box((0.268, 0.112, 0.014)),
        origin=Origin(xyz=(0.0, -0.056, 0.006)),
        material=handle_black,
        name="lever_bar",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.252),
        origin=Origin(xyz=(0.0, -0.114, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="front_grip",
    )

    selector_slide = model.part("selector_slide")
    selector_slide.visual(
        Box((0.034, 0.010, 0.016)),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material=accent_red,
        name="selector_carriage",
    )
    selector_slide.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.018, 0.010, 0.010)),
        material=accent_red,
        name="selector_tab",
    )

    paper_guide = model.part("paper_guide")
    paper_guide.visual(
        Box((0.028, 0.016, 0.018)),
        origin=Origin(xyz=(0.014, -0.062, 0.017)),
        material=guide_gray,
        name="guide_clamp",
    )
    paper_guide.visual(
        Box((0.008, 0.008, 0.018)),
        origin=Origin(xyz=(0.014, -0.053, 0.030)),
        material=guide_gray,
        name="guide_bridge",
    )
    paper_guide.visual(
        Box((0.008, 0.020, 0.018)),
        origin=Origin(xyz=(0.014, -0.039, 0.031)),
        material=guide_gray,
        name="guide_stop",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.026, 0.071)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "body_to_selector_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=selector_slide,
        origin=Origin(xyz=(-0.088, 0.043, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.126),
    )
    model.articulation(
        "body_to_paper_guide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=paper_guide,
        origin=Origin(xyz=(-0.124, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    selector_slide = object_model.get_part("selector_slide")
    paper_guide = object_model.get_part("paper_guide")

    handle_joint = object_model.get_articulation("body_to_handle")
    selector_joint = object_model.get_articulation("body_to_selector_slide")
    guide_joint = object_model.get_articulation("body_to_paper_guide")

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="lever_bar",
        negative_elem="housing_top",
        min_gap=0.0005,
        max_gap=0.004,
        name="closed handle hovers just above the punch housing",
    )
    ctx.expect_overlap(
        body,
        handle,
        axes="x",
        elem_a="housing_top",
        elem_b="lever_bar",
        min_overlap=0.24,
        name="closed handle spans the width of the punch housing",
    )

    ctx.expect_gap(
        selector_slide,
        body,
        axis="y",
        positive_elem="selector_carriage",
        negative_elem="selector_track",
        max_gap=0.001,
        max_penetration=0.0,
        name="selector carriage stays seated against the side track",
    )
    ctx.expect_overlap(
        selector_slide,
        body,
        axes="xz",
        elem_a="selector_carriage",
        elem_b="selector_track",
        min_overlap=0.012,
        name="selector carriage stays retained on the side track",
    )

    ctx.expect_contact(
        body,
        paper_guide,
        elem_a="front_fence",
        elem_b="guide_clamp",
        contact_tol=1e-6,
        name="paper guide clamp stays seated on the front fence",
    )
    ctx.expect_overlap(
        body,
        paper_guide,
        axes="xz",
        elem_a="front_fence",
        elem_b="guide_clamp",
        min_overlap=0.010,
        name="paper guide remains retained along the front fence",
    )

    handle_limits = handle_joint.motion_limits
    selector_limits = selector_joint.motion_limits
    guide_limits = guide_joint.motion_limits

    rest_grip_aabb = ctx.part_element_world_aabb(handle, elem="front_grip")
    with ctx.pose({handle_joint: handle_limits.upper if handle_limits and handle_limits.upper is not None else 1.0}):
        open_grip_aabb = ctx.part_element_world_aabb(handle, elem="front_grip")
    ctx.check(
        "handle opens upward from the rear hinge",
        rest_grip_aabb is not None
        and open_grip_aabb is not None
        and open_grip_aabb[1][2] > rest_grip_aabb[1][2] + 0.06,
        details=f"rest_grip={rest_grip_aabb}, open_grip={open_grip_aabb}",
    )

    rest_selector_pos = ctx.part_world_position(selector_slide)
    with ctx.pose(
        {selector_joint: selector_limits.upper if selector_limits and selector_limits.upper is not None else 0.12}
    ):
        ctx.expect_gap(
            selector_slide,
            body,
            axis="y",
            positive_elem="selector_carriage",
            negative_elem="selector_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="selector carriage stays seated at full travel",
        )
        selector_ext_pos = ctx.part_world_position(selector_slide)
    ctx.check(
        "selector slide travels along the punch head length",
        rest_selector_pos is not None
        and selector_ext_pos is not None
        and selector_ext_pos[0] > rest_selector_pos[0] + 0.10,
        details=f"rest_selector={rest_selector_pos}, extended_selector={selector_ext_pos}",
    )

    rest_guide_pos = ctx.part_world_position(paper_guide)
    with ctx.pose({guide_joint: guide_limits.upper if guide_limits and guide_limits.upper is not None else 0.20}):
        ctx.expect_contact(
            body,
            paper_guide,
            elem_a="front_fence",
            elem_b="guide_clamp",
            contact_tol=1e-6,
            name="paper guide stays seated at full travel",
        )
        guide_ext_pos = ctx.part_world_position(paper_guide)
    ctx.check(
        "paper guide slides across the front fence",
        rest_guide_pos is not None
        and guide_ext_pos is not None
        and guide_ext_pos[0] > rest_guide_pos[0] + 0.18,
        details=f"rest_guide={rest_guide_pos}, extended_guide={guide_ext_pos}",
    )

    return ctx.report()


object_model = build_object_model()
