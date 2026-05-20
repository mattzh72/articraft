from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_easel")

    ash = model.material("ash", rgba=(0.74, 0.64, 0.46, 1.0))
    walnut = model.material("walnut", rgba=(0.47, 0.34, 0.19, 1.0))
    brass = model.material("brass", rgba=(0.71, 0.59, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.36, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.028, 0.030, 1.20)),
        origin=Origin(xyz=(-0.19, 0.0, 0.82)),
        material=ash,
        name="left_rail",
    )
    body.visual(
        Box((0.028, 0.030, 1.20)),
        origin=Origin(xyz=(0.19, 0.0, 0.82)),
        material=ash,
        name="right_rail",
    )
    body.visual(
        Box((0.16, 0.032, 0.052)),
        origin=Origin(xyz=(-0.12, 0.0, 0.58)),
        material=ash,
        name="lower_left",
    )
    body.visual(
        Box((0.16, 0.032, 0.052)),
        origin=Origin(xyz=(0.12, 0.0, 0.58)),
        material=ash,
        name="lower_right",
    )
    body.visual(
        Box((0.11, 0.018, 0.052)),
        origin=Origin(xyz=(0.0, -0.019, 0.58)),
        material=ash,
        name="lower_bridge",
    )
    body.visual(
        Box((0.028, 0.045, 0.82)),
        origin=Origin(xyz=(-0.041, 0.0, 0.99)),
        material=ash,
        name="guide_left",
    )
    body.visual(
        Box((0.028, 0.045, 0.82)),
        origin=Origin(xyz=(0.041, 0.0, 0.99)),
        material=ash,
        name="guide_right",
    )
    body.visual(
        Box((0.16, 0.060, 0.10)),
        origin=Origin(xyz=(-0.18, 0.0, 1.44)),
        material=walnut,
        name="crown_left",
    )
    body.visual(
        Box((0.16, 0.060, 0.10)),
        origin=Origin(xyz=(0.18, 0.0, 1.44)),
        material=walnut,
        name="crown_right",
    )
    body.visual(
        Box((0.36, 0.024, 0.08)),
        origin=Origin(xyz=(0.0, -0.031, 1.42)),
        material=walnut,
        name="crown_bridge",
    )
    body.visual(
        Box((0.03, 0.07, 0.055)),
        origin=Origin(xyz=(-0.14, 0.028, 0.58)),
        material=ash,
        name="shelf_bracket_left",
    )
    body.visual(
        Box((0.03, 0.07, 0.055)),
        origin=Origin(xyz=(0.14, 0.028, 0.58)),
        material=ash,
        name="shelf_bracket_right",
    )
    body.visual(
        Box((0.05, 0.038, 0.05)),
        origin=Origin(xyz=(-0.22, 0.047, 1.43)),
        material=brass,
        name="front_hinge_left",
    )
    body.visual(
        Box((0.05, 0.038, 0.05)),
        origin=Origin(xyz=(0.22, 0.047, 1.43)),
        material=brass,
        name="front_hinge_right",
    )
    body.visual(
        Box((0.10, 0.258, 0.05)),
        origin=Origin(xyz=(0.0, -0.171, 1.43)),
        material=brass,
        name="rear_hinge",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.032, 0.020, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=ash,
        name="mast_bar",
    )
    mast.visual(
        Box((0.10, 0.045, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.985)),
        material=walnut,
        name="top_block",
    )
    mast.visual(
        Box((0.15, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.012, 0.94)),
        material=brass,
        name="top_lip",
    )
    mast.visual(
        Box((0.07, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.015, 0.935)),
        material=steel,
        name="thumb_slider",
    )

    shelf = model.part("shelf")
    shelf.visual(
        Box((0.34, 0.16, 0.016)),
        origin=Origin(xyz=(0.0, 0.08, -0.008)),
        material=ash,
        name="shelf_panel",
    )
    shelf.visual(
        Box((0.30, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.004, 0.006)),
        material=brass,
        name="hinge_leaf",
    )
    shelf.visual(
        Box((0.34, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.153, 0.015)),
        material=walnut,
        name="front_lip",
    )
    shelf.visual(
        Box((0.018, 0.12, 0.026)),
        origin=Origin(xyz=(-0.161, 0.088, 0.005)),
        material=walnut,
        name="left_cheek",
    )
    shelf.visual(
        Box((0.018, 0.12, 0.026)),
        origin=Origin(xyz=(0.161, 0.088, 0.005)),
        material=walnut,
        name="right_cheek",
    )

    front_leg_0 = model.part("front_leg_0")
    front_leg_0.visual(
        Box((0.028, 0.026, 1.46)),
        origin=Origin(xyz=(0.0, 0.0, -0.73), rpy=(0.0, 0.18, 0.0)),
        material=ash,
        name="leg_bar",
    )
    front_leg_0.visual(
        Box((0.060, 0.040, 0.100)),
        origin=Origin(xyz=(-0.128, 0.0, -1.39), rpy=(0.0, 0.18, 0.0)),
        material=rubber,
        name="foot",
    )

    front_leg_1 = model.part("front_leg_1")
    front_leg_1.visual(
        Box((0.028, 0.026, 1.46)),
        origin=Origin(xyz=(0.0, 0.0, -0.73), rpy=(0.0, -0.18, 0.0)),
        material=ash,
        name="leg_bar",
    )
    front_leg_1.visual(
        Box((0.060, 0.040, 0.100)),
        origin=Origin(xyz=(0.128, 0.0, -1.39), rpy=(0.0, -0.18, 0.0)),
        material=rubber,
        name="foot",
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Box((0.10, 0.258, 0.050)),
        origin=Origin(xyz=(0.0, 0.129, -0.050)),
        material=brass,
        name="hinge_block",
    )
    rear_leg.visual(
        Box((0.028, 0.026, 1.50)),
        origin=Origin(xyz=(0.0, 0.0, -0.75), rpy=(-0.36, 0.0, 0.0)),
        material=ash,
        name="leg_bar",
    )
    model.articulation(
        "body_to_mast",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.63)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.18,
            lower=0.0,
            upper=0.34,
        ),
    )
    model.articulation(
        "body_to_shelf",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shelf,
        origin=Origin(xyz=(0.0, 0.072, 0.58)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.4,
            lower=0.0,
            upper=1.47,
        ),
    )
    model.articulation(
        "body_to_front_leg_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_leg_0,
        origin=Origin(xyz=(-0.22, 0.043, 1.43)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=0.38,
        ),
    )
    model.articulation(
        "body_to_front_leg_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_leg_1,
        origin=Origin(xyz=(0.22, 0.043, 1.43)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=0.38,
        ),
    )
    model.articulation(
        "body_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_leg,
        origin=Origin(xyz=(0.0, -0.30, 1.43)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    mast = object_model.get_part("mast")
    shelf = object_model.get_part("shelf")
    front_leg_0 = object_model.get_part("front_leg_0")
    front_leg_1 = object_model.get_part("front_leg_1")
    rear_leg = object_model.get_part("rear_leg")

    mast_joint = object_model.get_articulation("body_to_mast")
    shelf_joint = object_model.get_articulation("body_to_shelf")
    front_leg_0_joint = object_model.get_articulation("body_to_front_leg_0")
    front_leg_1_joint = object_model.get_articulation("body_to_front_leg_1")
    rear_leg_joint = object_model.get_articulation("body_to_rear_leg")

    ctx.allow_overlap(
        body,
        rear_leg,
        elem_a="crown_bridge",
        elem_b="hinge_block",
        reason="The rear crown hinge is simplified as interleaved hinge leaves around a shared pivot line.",
    )
    ctx.allow_overlap(
        body,
        rear_leg,
        elem_a="crown_bridge",
        elem_b="leg_bar",
        reason="The rear leg strut is seated directly under the crown hinge bridge with a simplified tight hinge-seat proxy.",
    )

    def elem_aabb(part, elem):
        return ctx.part_element_world_aabb(part, elem=elem)

    def part_aabb(part):
        return ctx.part_world_aabb(part)

    left_guide = elem_aabb(body, "guide_left")
    right_guide = elem_aabb(body, "guide_right")
    mast_bar_rest = elem_aabb(mast, "mast_bar")
    body_rest = part_aabb(body)
    shelf_rest = elem_aabb(shelf, "shelf_panel")

    mast_centered_rest = (
        left_guide is not None
        and right_guide is not None
        and mast_bar_rest is not None
        and mast_bar_rest[0][0] >= left_guide[1][0] + 0.004
        and mast_bar_rest[1][0] <= right_guide[0][0] - 0.004
        and mast_bar_rest[0][1] >= left_guide[0][1] + 0.004
        and mast_bar_rest[1][1] <= left_guide[1][1] - 0.004
    )
    ctx.check(
        "mast starts centered in the guide slot",
        mast_centered_rest,
        details=f"left_guide={left_guide}, right_guide={right_guide}, mast={mast_bar_rest}",
    )

    shelf_projects_forward = (
        shelf_rest is not None
        and body_rest is not None
        and shelf_rest[1][1] > body_rest[1][1] + 0.12
    )
    ctx.check(
        "shelf projects forward in the working pose",
        shelf_projects_forward,
        details=f"body={body_rest}, shelf={shelf_rest}",
    )

    mast_rest_top = mast_bar_rest[1][2] if mast_bar_rest is not None else None
    with ctx.pose({mast_joint: 0.34}):
        mast_bar_high = elem_aabb(mast, "mast_bar")
        mast_extended = (
            mast_rest_top is not None
            and mast_bar_high is not None
            and mast_bar_high[1][2] > mast_rest_top + 0.30
        )
        mast_centered_high = (
            left_guide is not None
            and right_guide is not None
            and mast_bar_high is not None
            and mast_bar_high[0][0] >= left_guide[1][0] + 0.004
            and mast_bar_high[1][0] <= right_guide[0][0] - 0.004
            and mast_bar_high[0][1] >= left_guide[0][1] + 0.004
            and mast_bar_high[1][1] <= left_guide[1][1] - 0.004
        )
        ctx.check(
            "mast rises when extended",
            mast_extended,
            details=f"rest_top={mast_rest_top}, extended={mast_bar_high}",
        )
        ctx.check(
            "mast stays captured between the guide cheeks at full height",
            mast_centered_high,
            details=f"left_guide={left_guide}, right_guide={right_guide}, mast={mast_bar_high}",
        )

    with ctx.pose({shelf_joint: 1.47}):
        folded_panel = elem_aabb(shelf, "shelf_panel")
        folded_gap = None
        if folded_panel is not None and body_rest is not None:
            folded_gap = folded_panel[0][1] - body_rest[1][1]
        shelf_folds_flat = (
            folded_panel is not None
            and body_rest is not None
            and folded_gap is not None
            and 0.0 <= folded_gap <= 0.02
            and folded_panel[1][2] > 0.72
        )
        ctx.check(
            "shelf folds up close to the frame front",
            shelf_folds_flat,
            details=f"body={body_rest}, folded_panel={folded_panel}, gap={folded_gap}",
        )

    front_spread_rest = None
    left_rest = elem_aabb(front_leg_0, "foot")
    right_rest = elem_aabb(front_leg_1, "foot")
    if left_rest is not None and right_rest is not None:
        front_spread_rest = right_rest[1][0] - left_rest[0][0]

    with ctx.pose({front_leg_0_joint: 0.38, front_leg_1_joint: 0.38}):
        left_folded = elem_aabb(front_leg_0, "foot")
        right_folded = elem_aabb(front_leg_1, "foot")
        front_spread_folded = None
        if left_folded is not None and right_folded is not None:
            front_spread_folded = right_folded[1][0] - left_folded[0][0]
        ctx.check(
            "front legs narrow when folded toward the mast",
            front_spread_rest is not None
            and front_spread_folded is not None
            and front_spread_folded < front_spread_rest - 0.16,
            details=f"rest={front_spread_rest}, folded={front_spread_folded}",
        )

    rear_rest = elem_aabb(rear_leg, "leg_bar")
    rear_rest_max_y = rear_rest[1][1] if rear_rest is not None else None
    with ctx.pose({rear_leg_joint: 0.55}):
        rear_folded = elem_aabb(rear_leg, "leg_bar")
        rear_folded_max_y = rear_folded[1][1] if rear_folded is not None else None
        ctx.check(
            "rear leg swings inward on its crown hinge",
            rear_rest_max_y is not None
            and rear_folded_max_y is not None
            and rear_folded_max_y > rear_rest_max_y + 0.22,
            details=f"rest={rear_rest}, folded={rear_folded}",
        )

    return ctx.report()


object_model = build_object_model()
