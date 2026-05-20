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
    model = ArticulatedObject(name="low_profile_extension_slide")

    zinc = model.material("brushed_zinc", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_zinc = model.material("dark_zinc", rgba=(0.36, 0.38, 0.38, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.018, 0.015, 1.0))
    nylon = model.material("white_nylon", rgba=(0.86, 0.84, 0.76, 1.0))

    outer = model.part("outer_section")
    outer.visual(
        Box((0.66, 0.12, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=zinc,
        name="base_plate",
    )
    for y, name in ((0.054, "side_wall_0"), (-0.054, "side_wall_1")):
        outer.visual(
            Box((0.66, 0.012, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.024)),
            material=zinc,
            name=name,
        )
    for y, name in ((0.039, "top_lip_0"), (-0.039, "top_lip_1")):
        outer.visual(
            Box((0.66, 0.018, 0.008)),
            origin=Origin(xyz=(0.0, y, 0.036)),
            material=dark_zinc,
            name=name,
        )
    outer.visual(
        Box((0.025, 0.040, 0.020)),
        origin=Origin(xyz=(-0.315, 0.0, 0.018)),
        material=black,
        name="rear_bumper",
    )
    outer.visual(
        Cylinder(radius=0.004, length=0.110),
        origin=Origin(xyz=(0.309, 0.0, 0.036), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_zinc,
        name="front_stop_pin",
    )

    inner = model.part("inner_runner")
    inner.visual(
        Box((0.58, 0.038, 0.018)),
        origin=Origin(xyz=(0.041, 0.0, 0.019)),
        material=dark_zinc,
        name="runner_bar",
    )
    inner.visual(
        Box((0.018, 0.050, 0.050)),
        origin=Origin(xyz=(0.340, 0.0, 0.025)),
        material=zinc,
        name="front_face",
    )
    inner.visual(
        Box((0.014, 0.024, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, 0.03175)),
        material=black,
        name="stop_tab",
    )
    for x, y, name in (
        (-0.205, 0.017, "glide_pad_0"),
        (-0.205, -0.017, "glide_pad_1"),
        (0.205, 0.017, "glide_pad_2"),
        (0.205, -0.017, "glide_pad_3"),
    ):
        inner.visual(
            Box((0.055, 0.012, 0.0025)),
            origin=Origin(xyz=(x, y, 0.00925)),
            material=nylon,
            name=name,
        )

    model.articulation(
        "outer_to_runner",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.28),
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

    outer = object_model.get_part("outer_section")
    inner = object_model.get_part("inner_runner")
    joint = object_model.get_articulation("outer_to_runner")

    ctx.expect_within(
        inner,
        outer,
        axes="yz",
        inner_elem="runner_bar",
        margin=0.001,
        name="runner stays inside outer channel cross section",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="runner_bar",
        min_overlap=0.50,
        name="runner is deeply nested when retracted",
    )
    ctx.expect_gap(
        inner,
        outer,
        axis="x",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="front_face",
        name="square front face sits just ahead of fixed section",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({joint: 0.28}):
        ctx.expect_within(
            inner,
            outer,
            axes="yz",
            inner_elem="runner_bar",
            margin=0.001,
            name="extended runner stays aligned in channel",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            elem_a="runner_bar",
            min_overlap=0.25,
            name="extended runner retains supported insertion",
        )
        extended_pos = ctx.part_world_position(inner)

    ctx.check(
        "prismatic joint extends along shared axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
