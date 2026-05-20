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


BASE_DEPTH = 0.230
BASE_WIDTH = 0.145

LEVER_OPEN = 1.05
RULER_TRAVEL = 0.080
STOP_TRAVEL = 0.030

HINGE_X = -0.103
HINGE_Z = 0.037
RULER_SLOT_X = BASE_DEPTH / 2.0
RULER_SLOT_Z = 0.0092


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _aabb_center_z(aabb) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_punch")

    model.material("body", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("body_trim", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("lever", rgba=(0.68, 0.71, 0.74, 1.0))
    model.material("lever_grip", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("ruler", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("accent", rgba=(0.86, 0.28, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_DEPTH, 0.019, 0.008)),
        origin=Origin(xyz=(0.0, 0.058, 0.004)),
        material="body",
        name="foot_0",
    )
    base.visual(
        Box((BASE_DEPTH, 0.019, 0.008)),
        origin=Origin(xyz=(0.0, -0.058, 0.004)),
        material="body",
        name="foot_1",
    )
    base.visual(
        Box((0.032, 0.116, 0.008)),
        origin=Origin(xyz=(-0.099, 0.0, 0.004)),
        material="body",
        name="rear_cross",
    )
    base.visual(
        Box((0.178, 0.014, 0.022)),
        origin=Origin(xyz=(-0.005, 0.047, 0.019)),
        material="body_trim",
        name="guide_0",
    )
    base.visual(
        Box((0.178, 0.014, 0.022)),
        origin=Origin(xyz=(-0.005, -0.047, 0.019)),
        material="body_trim",
        name="guide_1",
    )
    base.visual(
        Box((0.108, 0.100, 0.012)),
        origin=Origin(xyz=(-0.036, 0.0, 0.014)),
        material="body_trim",
        name="deck",
    )
    base.visual(
        Box((0.030, 0.105, 0.008)),
        origin=Origin(xyz=(0.097, 0.0, 0.0155)),
        material="body_trim",
        name="front_bridge",
    )
    base.visual(
        Box((0.075, 0.104, 0.017)),
        origin=Origin(xyz=(-0.005, 0.0, 0.028)),
        material="body",
        name="die_housing",
    )
    base.visual(
        Box((0.030, 0.056, 0.009)),
        origin=Origin(xyz=(0.032, 0.0, 0.0245)),
        material="body",
        name="nose",
    )
    base.visual(
        Box((0.017, 0.014, 0.030)),
        origin=Origin(xyz=(HINGE_X, 0.054, 0.023)),
        material="body_trim",
        name="hinge_post_0",
    )
    base.visual(
        Box((0.017, 0.014, 0.030)),
        origin=Origin(xyz=(HINGE_X, -0.054, 0.023)),
        material="body_trim",
        name="hinge_post_1",
    )
    for idx, y in enumerate((-0.040, 0.040)):
        base.visual(
            Cylinder(radius=0.008, length=0.002),
            origin=Origin(xyz=(0.008, y, 0.036)),
            material="lever_grip",
            name=f"die_ring_{idx}",
        )

    lever = model.part("lever")
    _add_y_cylinder(
        lever,
        radius=0.0065,
        length=0.094,
        xyz=(0.0, 0.0, 0.0),
        material="lever",
        name="hinge_barrel",
    )
    lever.visual(
        Box((0.022, 0.034, 0.016)),
        origin=Origin(xyz=(0.012, 0.0, 0.006)),
        material="lever",
        name="hinge_knuckle",
    )
    lever.visual(
        Box((0.212, 0.034, 0.010)),
        origin=Origin(xyz=(0.106, 0.0, 0.011)),
        material="lever",
        name="arm",
    )
    lever.visual(
        Box((0.050, 0.085, 0.016)),
        origin=Origin(xyz=(0.197, 0.0, 0.008)),
        material="lever_grip",
        name="handle_pad",
    )
    lever.visual(
        Box((0.050, 0.096, 0.010)),
        origin=Origin(xyz=(0.082, 0.0, 0.006)),
        material="lever",
        name="press_beam",
    )
    for idx, y in enumerate((-0.040, 0.040)):
        lever.visual(
            Cylinder(radius=0.006, length=0.011),
            origin=Origin(xyz=(0.082, y, 0.006)),
            material="lever_grip",
            name=f"press_pin_{idx}",
        )

    ruler = model.part("ruler")
    ruler.visual(
        Box((0.145, 0.094, 0.0025)),
        origin=Origin(xyz=(-0.0375, 0.0, 0.0)),
        material="ruler",
        name="ruler_bar",
    )
    ruler.visual(
        Box((0.006, 0.094, 0.008)),
        origin=Origin(xyz=(0.038, 0.0, 0.0028)),
        material="body_trim",
        name="pull_lip",
    )

    stop_block = model.part("stop_block")
    stop_block.visual(
        Box((0.020, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material="accent",
        name="slider_pad",
    )
    stop_block.visual(
        Box((0.012, 0.028, 0.030)),
        origin=Origin(xyz=(0.001, 0.0, 0.019)),
        material="accent",
        name="fence",
    )

    model.articulation(
        "base_to_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lever,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=LEVER_OPEN, effort=25.0, velocity=2.5),
    )
    model.articulation(
        "base_to_ruler",
        ArticulationType.PRISMATIC,
        parent=base,
        child=ruler,
        origin=Origin(xyz=(RULER_SLOT_X, 0.0, RULER_SLOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=RULER_TRAVEL, effort=30.0, velocity=0.20),
    )
    model.articulation(
        "ruler_to_stop",
        ArticulationType.PRISMATIC,
        parent=ruler,
        child=stop_block,
        origin=Origin(xyz=(0.010, 0.0, 0.00125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-STOP_TRAVEL,
            upper=STOP_TRAVEL,
            effort=5.0,
            velocity=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lever = object_model.get_part("lever")
    ruler = object_model.get_part("ruler")
    stop_block = object_model.get_part("stop_block")

    lever_hinge = object_model.get_articulation("base_to_lever")
    ruler_slide = object_model.get_articulation("base_to_ruler")
    stop_slide = object_model.get_articulation("ruler_to_stop")

    ctx.expect_gap(
        lever,
        base,
        axis="z",
        positive_elem="press_pin_0",
        negative_elem="die_ring_0",
        min_gap=0.0002,
        max_gap=0.0035,
        name="closed lever hovers just above the punch die",
    )
    ctx.expect_gap(
        base,
        ruler,
        axis="z",
        positive_elem="front_bridge",
        negative_elem="ruler_bar",
        min_gap=0.0002,
        max_gap=0.0020,
        name="stored ruler sits just below the front bridge",
    )

    handle_rest = ctx.part_element_world_aabb(lever, elem="handle_pad")
    with ctx.pose({lever_hinge: LEVER_OPEN}):
        ctx.expect_gap(
            lever,
            base,
            axis="z",
            positive_elem="handle_pad",
            negative_elem="die_housing",
            min_gap=0.070,
            name="opened lever lifts high above the base",
        )
        handle_open = ctx.part_element_world_aabb(lever, elem="handle_pad")
    ctx.check(
        "lever opens upward",
        handle_rest is not None
        and handle_open is not None
        and _aabb_center_z(handle_open) is not None
        and _aabb_center_z(handle_rest) is not None
        and _aabb_center_z(handle_open) > _aabb_center_z(handle_rest) + 0.080,
        details=f"rest={handle_rest}, open={handle_open}",
    )

    ruler_rest = ctx.part_world_position(ruler)
    with ctx.pose({ruler_slide: RULER_TRAVEL}):
        ctx.expect_overlap(
            ruler,
            base,
            axes="x",
            elem_a="ruler_bar",
            elem_b="front_bridge",
            min_overlap=0.020,
            name="extended ruler remains captured by the front bridge",
        )
        ctx.expect_gap(
            base,
            ruler,
            axis="z",
            positive_elem="front_bridge",
            negative_elem="ruler_bar",
            min_gap=0.0002,
            max_gap=0.0020,
            name="extended ruler still clears the bridge roof",
        )
        ruler_extended = ctx.part_world_position(ruler)
    ctx.check(
        "ruler pulls forward from the base",
        ruler_rest is not None
        and ruler_extended is not None
        and ruler_extended[0] > ruler_rest[0] + 0.060,
        details=f"rest={ruler_rest}, extended={ruler_extended}",
    )

    with ctx.pose({ruler_slide: RULER_TRAVEL, stop_slide: -STOP_TRAVEL}):
        ctx.expect_within(
            stop_block,
            ruler,
            axes="y",
            inner_elem="fence",
            outer_elem="ruler_bar",
            margin=0.001,
            name="stop block stays on the ruler at one end",
        )
        stop_left = ctx.part_world_position(stop_block)
    with ctx.pose({ruler_slide: RULER_TRAVEL, stop_slide: STOP_TRAVEL}):
        ctx.expect_within(
            stop_block,
            ruler,
            axes="y",
            inner_elem="fence",
            outer_elem="ruler_bar",
            margin=0.001,
            name="stop block stays on the ruler at the other end",
        )
        stop_right = ctx.part_world_position(stop_block)
    ctx.check(
        "stop block traverses across the ruler",
        stop_left is not None and stop_right is not None and stop_right[1] > stop_left[1] + 0.050,
        details=f"left={stop_left}, right={stop_right}",
    )

    return ctx.report()


object_model = build_object_model()
