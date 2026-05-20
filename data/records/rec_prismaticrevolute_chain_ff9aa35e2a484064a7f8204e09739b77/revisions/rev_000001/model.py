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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_slide_with_rotary_tip")

    steel = Material("dark_burnished_steel", rgba=(0.12, 0.14, 0.15, 1.0))
    rail_steel = Material("polished_rail_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    carriage_blue = Material("anodized_carriage_blue", rgba=(0.05, 0.20, 0.42, 1.0))
    pin_metal = Material("hardened_hinge_pin", rgba=(0.82, 0.78, 0.66, 1.0))
    paddle_orange = Material("orange_paddle_grip", rgba=(0.95, 0.36, 0.08, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    bridge = model.part("bridge")
    # A bench-sized rear support and raised guide bridge.  The broad base and
    # rear upright make the slide read as a stationary backed fixture rather
    # than a free-floating rail.
    bridge.visual(
        Box((0.46, 0.26, 0.035)),
        origin=Origin(xyz=(0.035, 0.0, 0.0175)),
        material=steel,
        name="base_plate",
    )
    bridge.visual(
        Box((0.050, 0.25, 0.315)),
        origin=Origin(xyz=(-0.165, 0.0, 0.1925)),
        material=steel,
        name="rear_upright",
    )
    bridge.visual(
        Box((0.345, 0.036, 0.052)),
        origin=Origin(xyz=(0.030, 0.077, 0.225)),
        material=steel,
        name="rail_0",
    )
    bridge.visual(
        Box((0.345, 0.036, 0.052)),
        origin=Origin(xyz=(0.030, -0.077, 0.225)),
        material=steel,
        name="rail_1",
    )
    bridge.visual(
        Cylinder(radius=0.009, length=0.350),
        origin=Origin(xyz=(0.032, 0.077, 0.263), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="guide_rod_0",
    )
    bridge.visual(
        Cylinder(radius=0.009, length=0.350),
        origin=Origin(xyz=(0.032, -0.077, 0.263), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="guide_rod_1",
    )
    bridge.visual(
        Box((0.045, 0.036, 0.090)),
        origin=Origin(xyz=(0.135, 0.077, 0.296)),
        material=steel,
        name="bridge_post_0",
    )
    bridge.visual(
        Box((0.045, 0.036, 0.090)),
        origin=Origin(xyz=(0.135, -0.077, 0.296)),
        material=steel,
        name="bridge_post_1",
    )
    bridge.visual(
        Box((0.055, 0.225, 0.036)),
        origin=Origin(xyz=(0.135, 0.0, 0.359)),
        material=steel,
        name="front_bridge_cap",
    )
    bridge.visual(
        Box((0.018, 0.055, 0.010)),
        origin=Origin(xyz=(-0.125, 0.080, 0.005)),
        material=rubber,
        name="foot_0",
    )
    bridge.visual(
        Box((0.018, 0.055, 0.010)),
        origin=Origin(xyz=(-0.125, -0.080, 0.005)),
        material=rubber,
        name="foot_1",
    )
    bridge.visual(
        Box((0.018, 0.055, 0.010)),
        origin=Origin(xyz=(0.205, 0.080, 0.005)),
        material=rubber,
        name="foot_2",
    )
    bridge.visual(
        Box((0.018, 0.055, 0.010)),
        origin=Origin(xyz=(0.205, -0.080, 0.005)),
        material=rubber,
        name="foot_3",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.280, 0.094, 0.050)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=carriage_blue,
        name="slider_body",
    )
    carriage.visual(
        Box((0.046, 0.110, 0.104)),
        origin=Origin(xyz=(0.181, 0.0, 0.0)),
        material=carriage_blue,
        name="front_plate",
    )
    carriage.visual(
        Box((0.040, 0.024, 0.072)),
        origin=Origin(xyz=(0.224, 0.064, 0.015)),
        material=carriage_blue,
        name="hinge_lug_0",
    )
    carriage.visual(
        Box((0.040, 0.024, 0.072)),
        origin=Origin(xyz=(0.224, -0.064, 0.015)),
        material=carriage_blue,
        name="hinge_lug_1",
    )
    carriage.visual(
        Cylinder(radius=0.006, length=0.164),
        origin=Origin(xyz=(0.225, 0.0, 0.015), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="hinge_pin",
    )

    paddle = model.part("paddle")
    paddle.visual(
        Cylinder(radius=0.014, length=0.100),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="paddle_barrel",
    )
    paddle_panel_shape = (
        cq.Workplane("XY")
        .box(0.020, 0.088, 0.108)
        .edges()
        .fillet(0.004)
    )
    paddle.visual(
        mesh_from_cadquery(paddle_panel_shape, "rounded_paddle_panel"),
        origin=Origin(xyz=(0.021, 0.0, -0.052)),
        material=paddle_orange,
        name="paddle_panel",
    )

    model.articulation(
        "slide_joint",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.25, lower=0.0, upper=0.120),
    )

    model.articulation(
        "paddle_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=paddle,
        origin=Origin(xyz=(0.225, 0.0, 0.015)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.55, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("carriage")
    paddle = object_model.get_part("paddle")
    slide = object_model.get_articulation("slide_joint")
    hinge = object_model.get_articulation("paddle_hinge")

    ctx.check(
        "linear stage is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"slide_joint type is {slide.articulation_type}",
    )
    ctx.check(
        "paddle is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"paddle_hinge type is {hinge.articulation_type}",
    )

    # The visible hinge is represented as a real pin captured inside the paddle
    # barrel, so this intentional local overlap is the mechanism rather than a
    # collision between unrelated pieces.
    ctx.allow_overlap(
        carriage,
        paddle,
        elem_a="hinge_pin",
        elem_b="paddle_barrel",
        reason="The carriage pin is intentionally captured inside the paddle hinge barrel.",
    )
    ctx.expect_within(
        carriage,
        paddle,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="paddle_barrel",
        margin=0.001,
        name="hinge pin is coaxial inside barrel",
    )
    ctx.expect_overlap(
        carriage,
        paddle,
        axes="y",
        elem_a="hinge_pin",
        elem_b="paddle_barrel",
        min_overlap=0.080,
        name="hinge pin spans the paddle barrel",
    )

    ctx.expect_within(
        carriage,
        bridge,
        axes="yz",
        inner_elem="slider_body",
        margin=0.0,
        name="carriage body stays inside bridge corridor",
    )
    ctx.expect_overlap(
        carriage,
        bridge,
        axes="x",
        elem_a="slider_body",
        elem_b="rail_0",
        min_overlap=0.200,
        name="carriage retained in rail at rest",
    )
    ctx.expect_gap(
        paddle,
        carriage,
        axis="x",
        positive_elem="paddle_panel",
        negative_elem="front_plate",
        min_gap=0.010,
        max_gap=0.060,
        name="paddle hangs just proud of carriage front face",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.120}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            bridge,
            axes="yz",
            inner_elem="slider_body",
            margin=0.0,
            name="extended carriage remains in bridge corridor",
        )
        ctx.expect_overlap(
            carriage,
            bridge,
            axes="x",
            elem_a="slider_body",
            elem_b="rail_0",
            min_overlap=0.110,
            name="extended carriage keeps retained insertion",
        )
    ctx.check(
        "slide extends forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_aabb = ctx.part_world_aabb(paddle)
    with ctx.pose({hinge: 0.75}):
        swung_aabb = ctx.part_world_aabb(paddle)
    ctx.check(
        "paddle swings forward about hinge",
        rest_aabb is not None
        and swung_aabb is not None
        and swung_aabb[1][0] > rest_aabb[1][0] + 0.035,
        details=f"rest_aabb={rest_aabb}, swung_aabb={swung_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
