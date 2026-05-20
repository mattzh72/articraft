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
    model = ArticulatedObject(name="heavy_duty_punch")

    painted_steel = model.material("painted_steel", rgba=(0.28, 0.31, 0.34, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.13, 0.14, 0.15, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    scale_cover = model.material("scale_cover", rgba=(0.62, 0.70, 0.80, 0.6))
    guide_accent = model.material("guide_accent", rgba=(0.80, 0.12, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.34, 0.20, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=painted_steel,
        name="base_plate",
    )
    body.visual(
        Box((0.22, 0.16, 0.050)),
        origin=Origin(xyz=(-0.015, 0.0, 0.037)),
        material=painted_steel,
        name="housing",
    )
    body.visual(
        Box((0.115, 0.180, 0.008)),
        origin=Origin(xyz=(0.0975, 0.0, 0.066)),
        material=painted_steel,
        name="front_deck",
    )
    body.visual(
        Box((0.062, 0.180, 0.004)),
        origin=Origin(xyz=(0.112, 0.0, 0.070)),
        material=dark_plastic,
        name="track",
    )
    body.visual(
        Box((0.012, 0.180, 0.024)),
        origin=Origin(xyz=(0.149, 0.0, 0.084)),
        material=painted_steel,
        name="front_fence",
    )
    body.visual(
        Box((0.046, 0.100, 0.028)),
        origin=Origin(xyz=(0.065, 0.0, 0.084)),
        material=painted_steel,
        name="punch_head",
    )
    body.visual(
        Box((0.080, 0.160, 0.024)),
        origin=Origin(xyz=(-0.105, 0.0, 0.074)),
        material=painted_steel,
        name="rear_rise",
    )
    for side, y in enumerate((-0.060, 0.060)):
        body.visual(
            Box((0.030, 0.020, 0.050)),
            origin=Origin(xyz=(-0.122, y, 0.099)),
            material=painted_steel,
            name=f"cheek_{side}",
        )
        body.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(-0.122, y, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_steel,
            name=f"hinge_boss_{side}",
        )
    for side, y in enumerate((-0.122, 0.122)):
        body.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(0.132, y / 2.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"foot_{side}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.010, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="rear_barrel",
    )
    handle.visual(
        Box((0.284, 0.075, 0.020)),
        origin=Origin(xyz=(0.138, 0.0, 0.002)),
        material=painted_steel,
        name="arm",
    )
    handle.visual(
        Box((0.140, 0.070, 0.010)),
        origin=Origin(xyz=(0.180, 0.0, 0.017)),
        material=dark_plastic,
        name="grip_pad",
    )
    handle.visual(
        Box((0.050, 0.060, 0.016)),
        origin=Origin(xyz=(0.255, 0.0, -0.002)),
        material=painted_steel,
        name="nose",
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.042, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_plastic,
        name="runner",
    )
    guide.visual(
        Box((0.026, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=guide_accent,
        name="carriage",
    )
    guide.visual(
        Box((0.010, 0.028, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.021)),
        material=guide_accent,
        name="stop_fin",
    )
    guide.visual(
        Box((0.052, 0.028, 0.003)),
        origin=Origin(xyz=(0.004, 0.0, 0.0195)),
        material=painted_steel,
        name="marking_pad",
    )
    guide.visual(
        Cylinder(radius=0.0045, length=0.026),
        origin=Origin(xyz=(-0.015, 0.0, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="finger_nub",
    )
    guide.visual(
        Box((0.006, 0.016, 0.006)),
        origin=Origin(xyz=(0.028, 0.0, 0.024)),
        material=painted_steel,
        name="cover_mount",
    )

    cover = model.part("cover")
    cover.visual(
        Cylinder(radius=0.003, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="cover_barrel",
    )
    cover.visual(
        Box((0.046, 0.026, 0.0025)),
        origin=Origin(xyz=(-0.021, 0.0, -0.002)),
        material=scale_cover,
        name="cover_panel",
    )
    cover.visual(
        Box((0.008, 0.014, 0.004)),
        origin=Origin(xyz=(-0.042, 0.0, -0.0005)),
        material=dark_plastic,
        name="lift_tab",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.122, 0.0, 0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )
    model.articulation(
        "body_to_guide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=guide,
        origin=Origin(xyz=(0.112, 0.0, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.18,
            lower=-0.060,
            upper=0.060,
        ),
    )
    model.articulation(
        "guide_to_cover",
        ArticulationType.REVOLUTE,
        parent=guide,
        child=cover,
        origin=Origin(xyz=(0.034, 0.0, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def _z_bounds(ctx: TestContext, part, elem: str) -> tuple[float, float] | None:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    return aabb[0][2], aabb[1][2]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("guide")
    cover = object_model.get_part("cover")

    handle_joint = object_model.get_articulation("body_to_handle")
    guide_joint = object_model.get_articulation("body_to_guide")
    cover_joint = object_model.get_articulation("guide_to_cover")

    ctx.expect_gap(
        guide,
        body,
        axis="z",
        positive_elem="runner",
        negative_elem="track",
        max_gap=0.001,
        max_penetration=1e-5,
        name="guide runner rests on the front track",
    )
    ctx.expect_overlap(
        cover,
        guide,
        axes="xy",
        elem_a="cover_panel",
        elem_b="marking_pad",
        min_overlap=0.020,
        name="cover spans the guide markings",
    )
    ctx.expect_gap(
        cover,
        guide,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="marking_pad",
        min_gap=0.001,
        max_gap=0.005,
        name="cover hovers just above the guide markings",
    )

    rest_guide_pos = ctx.part_world_position(guide)
    rest_grip_bounds = _z_bounds(ctx, handle, "grip_pad")
    rest_cover_bounds = _z_bounds(ctx, cover, "cover_panel")

    with ctx.pose({guide_joint: guide_joint.motion_limits.upper}):
        ctx.expect_gap(
            guide,
            body,
            axis="z",
            positive_elem="runner",
            negative_elem="track",
            max_gap=0.001,
            max_penetration=1e-5,
            name="guide remains seated on the track at full travel",
        )
        extended_guide_pos = ctx.part_world_position(guide)

    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        raised_grip_bounds = _z_bounds(ctx, handle, "grip_pad")

    with ctx.pose({cover_joint: cover_joint.motion_limits.upper}):
        opened_cover_bounds = _z_bounds(ctx, cover, "cover_panel")

    ctx.check(
        "guide slides laterally along the fence",
        rest_guide_pos is not None
        and extended_guide_pos is not None
        and extended_guide_pos[1] > rest_guide_pos[1] + 0.04,
        details=f"rest={rest_guide_pos}, extended={extended_guide_pos}",
    )
    ctx.check(
        "handle lifts upward from the punch body",
        rest_grip_bounds is not None
        and raised_grip_bounds is not None
        and raised_grip_bounds[0] > rest_grip_bounds[0] + 0.04,
        details=f"rest={rest_grip_bounds}, raised={raised_grip_bounds}",
    )
    ctx.check(
        "cover flips away from the markings",
        rest_cover_bounds is not None
        and opened_cover_bounds is not None
        and opened_cover_bounds[1] > rest_cover_bounds[1] + 0.02,
        details=f"rest={rest_cover_bounds}, opened={opened_cover_bounds}",
    )

    return ctx.report()


object_model = build_object_model()
