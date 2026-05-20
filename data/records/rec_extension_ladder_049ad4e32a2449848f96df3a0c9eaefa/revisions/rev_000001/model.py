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


def _add_ladder_section(
    part,
    *,
    rail_depth: float,
    rail_width: float,
    rail_centers_y: tuple[float, float],
    rail_length: float,
    rung_depth: float,
    rung_height: float,
    rung_span: float,
    rung_positions_z: tuple[float, ...],
    rail_material: str,
    rung_material: str,
) -> None:
    for index, rail_y in enumerate(rail_centers_y):
        part.visual(
            Box((rail_depth, rail_width, rail_length)),
            origin=Origin(xyz=(0.0, rail_y, rail_length / 2.0)),
            material=rail_material,
            name=f"rail_{index}",
        )

    for index, rung_z in enumerate(rung_positions_z):
        part.visual(
            Box((rung_depth, rung_span, rung_height)),
            origin=Origin(xyz=(rail_depth * 0.35, 0.0, rung_z)),
            material=rung_material,
            name=f"rung_{index}",
        )


def _add_hook_arm(part, *, material: str) -> None:
    part.visual(
        Cylinder(radius=0.013, length=0.038),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="barrel",
    )
    part.visual(
        Box((0.022, 0.034, 0.060)),
        origin=Origin(xyz=(0.010, 0.0, 0.028)),
        material=material,
        name="strap",
    )
    part.visual(
        Box((0.022, 0.016, 0.340)),
        origin=Origin(xyz=(0.010, 0.0, 0.190)),
        material=material,
        name="arm",
    )
    part.visual(
        Box((0.100, 0.016, 0.022)),
        origin=Origin(xyz=(0.049, 0.0, 0.352)),
        material=material,
        name="hook_span",
    )
    part.visual(
        Box((0.022, 0.016, 0.082)),
        origin=Origin(xyz=(0.088, 0.0, 0.312)),
        material=material,
        name="hook_drop",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.088, 0.0, 0.271), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="hook_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="painters_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    aluminum_dark = model.material("aluminum_dark", rgba=(0.68, 0.70, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.88, 0.42, 0.14, 1.0))

    base = model.part("base")
    _add_ladder_section(
        base,
        rail_depth=0.028,
        rail_width=0.065,
        rail_centers_y=(-0.1825, 0.1825),
        rail_length=3.15,
        rung_depth=0.030,
        rung_height=0.032,
        rung_span=0.335,
        rung_positions_z=tuple(0.34 + 0.28 * index for index in range(10)),
        rail_material=aluminum,
        rung_material=aluminum_dark,
    )
    base.visual(
        Box((0.040, 0.440, 0.045)),
        origin=Origin(xyz=(-0.010, 0.0, 3.1275)),
        material=aluminum_dark,
        name="top_cap",
    )
    for index, rail_y in enumerate((-0.1825, 0.1825)):
        base.visual(
            Box((0.050, 0.072, 0.045)),
            origin=Origin(xyz=(0.0, rail_y, 0.0225)),
            material=rubber,
            name=f"foot_{index}",
        )

    fly = model.part("fly")
    _add_ladder_section(
        fly,
        rail_depth=0.024,
        rail_width=0.055,
        rail_centers_y=(-0.165, 0.165),
        rail_length=2.55,
        rung_depth=0.026,
        rung_height=0.028,
        rung_span=0.300,
        rung_positions_z=tuple(0.25 + 0.28 * index for index in range(8)),
        rail_material=aluminum,
        rung_material=aluminum_dark,
    )
    fly.visual(
        Box((0.034, 0.385, 0.040)),
        origin=Origin(xyz=(-0.004, 0.0, 2.53)),
        material=aluminum_dark,
        name="top_end",
    )
    fly.visual(
        Box((0.020, 0.360, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 1.48)),
        material=aluminum_dark,
        name="lock_plate",
    )
    for index, (rail_y, guide_z) in enumerate(
        (
            (-0.165, 0.28),
            (0.165, 0.28),
            (-0.165, 1.62),
            (0.165, 1.62),
        )
    ):
        fly.visual(
            Box((0.014, 0.050, 0.090)),
            origin=Origin(xyz=(-0.019, rail_y, guide_z)),
            material=aluminum_dark,
            name=f"guide_pad_{index}",
        )
    for index, rail_y in enumerate((-0.165, 0.165)):
        fly.visual(
            Box((0.021, 0.050, 0.120)),
            origin=Origin(xyz=(0.0005, rail_y, 2.16)),
            material=aluminum_dark,
            name=f"hook_bracket_{index}",
        )

    hook_0 = model.part("hook_0")
    _add_hook_arm(hook_0, material=safety_orange)

    hook_1 = model.part("hook_1")
    _add_hook_arm(hook_1, material=safety_orange)

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.040, 0.0, 0.80)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "fly_to_hook_0",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=hook_0,
        origin=Origin(xyz=(0.024, -0.165, 2.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "fly_to_hook_1",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=hook_1,
        origin=Origin(xyz=(0.024, 0.165, 2.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    fly = object_model.get_part("fly")
    fly_slide = object_model.get_articulation("base_to_fly")
    hook_0 = object_model.get_part("hook_0")
    hook_1 = object_model.get_part("hook_1")
    hook_joint_0 = object_model.get_articulation("fly_to_hook_0")
    hook_joint_1 = object_model.get_articulation("fly_to_hook_1")

    ctx.expect_within(
        fly,
        base,
        axes="y",
        margin=0.02,
        name="fly stays inside base rail width at rest",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        min_overlap=2.30,
        name="fly remains deeply nested in base at rest",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({fly_slide: 1.10}):
        ctx.expect_within(
            fly,
            base,
            axes="y",
            margin=0.02,
            name="fly stays inside base rail width when extended",
        )
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=0.55,
            name="fly keeps retained overlap when extended",
        )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 1.05,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    hook_0_rest = ctx.part_element_world_aabb(hook_0, elem="hook_drop")
    hook_1_rest = ctx.part_element_world_aabb(hook_1, elem="hook_drop")
    with ctx.pose({hook_joint_0: 1.10}):
        hook_0_open = ctx.part_element_world_aabb(hook_0, elem="hook_drop")
        hook_1_unchanged = ctx.part_element_world_aabb(hook_1, elem="hook_drop")

    ctx.check(
        "hook_0 pivots forward",
        hook_0_rest is not None
        and hook_0_open is not None
        and hook_0_open[1][0] > hook_0_rest[1][0] + 0.18,
        details=f"rest={hook_0_rest}, open={hook_0_open}",
    )
    ctx.check(
        "hook_1 stays put when hook_0 moves",
        hook_1_rest is not None
        and hook_1_unchanged is not None
        and abs(hook_1_unchanged[0][0] - hook_1_rest[0][0]) < 1e-6
        and abs(hook_1_unchanged[1][0] - hook_1_rest[1][0]) < 1e-6,
        details=f"rest={hook_1_rest}, moved_other={hook_1_unchanged}",
    )

    with ctx.pose({hook_joint_1: 1.10}):
        hook_1_open = ctx.part_element_world_aabb(hook_1, elem="hook_drop")

    ctx.check(
        "hook_1 pivots forward",
        hook_1_rest is not None
        and hook_1_open is not None
        and hook_1_open[1][0] > hook_1_rest[1][0] + 0.18,
        details=f"rest={hook_1_rest}, open={hook_1_open}",
    )

    return ctx.report()


object_model = build_object_model()
