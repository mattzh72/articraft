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
    model = ArticulatedObject(name="drawer_shuttle_paddle")

    dark_oxide = model.material("dark_oxide", rgba=(0.08, 0.09, 0.10, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.05, 0.23, 0.48, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.015, 0.015, 0.014, 1.0))

    channel = model.part("channel")
    # A grounded U-channel: heavy base, upright guide walls, inward lips, and
    # end stops.  The lane is open enough to see the moving block and hinge.
    channel.visual(
        Box((0.70, 0.24, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_oxide,
        name="base_plate",
    )
    for suffix, y in (("0", -0.107), ("1", 0.107)):
        channel.visual(
            Box((0.68, 0.026, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.061)),
            material=worn_steel,
            name=f"side_wall_{suffix}",
        )
        channel.visual(
            Box((0.68, 0.030, 0.014)),
            origin=Origin(xyz=(0.0, 0.079 if y > 0 else -0.079, 0.091)),
            material=worn_steel,
            name=f"retainer_lip_{suffix}",
        )
    for suffix, x in (("0", -0.330), ("1", 0.330)):
        channel.visual(
            Box((0.035, 0.195, 0.052)),
            origin=Origin(xyz=(x, 0.0, 0.052)),
            material=worn_steel,
            name=f"end_stop_{suffix}",
        )

    shuttle = model.part("shuttle")
    # The shuttle frame is at the block center.  Two black runners bear on the
    # base of the channel while the blue block is captured below the top lips.
    shuttle.visual(
        Box((0.160, 0.108, 0.040)),
        origin=Origin(),
        material=blue_anodized,
        name="slide_block",
    )
    for suffix, y in (("0", -0.042), ("1", 0.042)):
        shuttle.visual(
            Box((0.150, 0.024, 0.014)),
            origin=Origin(xyz=(0.0, y, -0.025)),
            material=black_polymer,
            name=f"runner_{suffix}",
        )

    # Side-mounted hinge support on the shuttle.  Outer knuckles are split to
    # leave the paddle's central knuckle free to rotate between them.
    for suffix, x in (("0", -0.045), ("1", 0.045)):
        shuttle.visual(
            Box((0.028, 0.018, 0.045)),
            origin=Origin(xyz=(x, 0.043, 0.026)),
            material=blue_anodized,
            name=f"hinge_cheek_{suffix}",
        )
        shuttle.visual(
            Cylinder(radius=0.012, length=0.032),
            origin=Origin(xyz=(x, 0.043, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=worn_steel,
            name=f"hinge_knuckle_{suffix}",
        )
    shuttle.visual(
        Cylinder(radius=0.004, length=0.130),
        origin=Origin(xyz=(0.0, 0.043, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_oxide,
        name="hinge_pin",
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.0105, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=worn_steel,
        name="paddle_knuckle",
    )
    paddle.visual(
        Box((0.045, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material=safety_orange,
        name="hinge_tongue",
    )
    paddle.visual(
        Box((0.105, 0.110, 0.012)),
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        material=safety_orange,
        name="output_plate",
    )
    paddle.visual(
        Cylinder(radius=0.006, length=0.105),
        origin=Origin(xyz=(0.0, 0.150, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=safety_orange,
        name="rounded_edge",
    )

    model.articulation(
        "channel_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=channel,
        child=shuttle,
        origin=Origin(xyz=(-0.180, 0.0, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.250),
    )
    model.articulation(
        "shuttle_to_paddle",
        ArticulationType.REVOLUTE,
        parent=shuttle,
        child=paddle,
        origin=Origin(xyz=(0.0, 0.043, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    channel = object_model.get_part("channel")
    shuttle = object_model.get_part("shuttle")
    paddle = object_model.get_part("paddle")
    slide = object_model.get_articulation("channel_to_shuttle")
    hinge = object_model.get_articulation("shuttle_to_paddle")

    ctx.allow_overlap(
        shuttle,
        paddle,
        elem_a="hinge_pin",
        elem_b="paddle_knuckle",
        reason="The visible hinge pin is intentionally captured through the paddle knuckle.",
    )

    ctx.expect_contact(
        shuttle,
        channel,
        elem_a="runner_0",
        elem_b="base_plate",
        name="runner bears on channel base",
    )
    ctx.expect_within(
        shuttle,
        channel,
        axes="y",
        margin=0.0,
        name="shuttle is captured laterally by the channel",
    )
    ctx.expect_overlap(
        shuttle,
        channel,
        axes="x",
        min_overlap=0.14,
        elem_a="slide_block",
        elem_b="base_plate",
        name="shuttle remains in the channel at rest",
    )
    ctx.expect_within(
        shuttle,
        paddle,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="paddle_knuckle",
        margin=0.0,
        name="hinge pin is centered in the paddle knuckle bore",
    )
    ctx.expect_overlap(
        shuttle,
        paddle,
        axes="x",
        min_overlap=0.045,
        elem_a="hinge_pin",
        elem_b="paddle_knuckle",
        name="hinge pin spans the paddle knuckle",
    )

    rest_pos = ctx.part_world_position(shuttle)
    with ctx.pose({slide: 0.250}):
        ctx.expect_overlap(
            shuttle,
            channel,
            axes="x",
            min_overlap=0.14,
            elem_a="slide_block",
            elem_b="base_plate",
            name="extended shuttle still has retained insertion",
        )
        ctx.expect_contact(
            shuttle,
            channel,
            elem_a="runner_0",
            elem_b="base_plate",
            name="runner stays supported at full travel",
        )
        extended_pos = ctx.part_world_position(shuttle)

    ctx.check(
        "prismatic joint advances the shuttle along the channel",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_aabb = ctx.part_world_aabb(paddle)
    with ctx.pose({hinge: 1.25}):
        raised_aabb = ctx.part_world_aabb(paddle)
    ctx.check(
        "revolute joint lifts the paddle from the shuttle side",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.08,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
