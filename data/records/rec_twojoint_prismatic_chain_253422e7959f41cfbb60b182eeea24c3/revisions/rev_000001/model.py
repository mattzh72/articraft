from __future__ import annotations

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
    model = ArticulatedObject(name="box_guide_two_stage_extension_chain")

    anodized_dark = Material("dark_anodized_aluminum", color=(0.08, 0.09, 0.10, 1.0))
    bead_blasted = Material("bead_blasted_aluminum", color=(0.55, 0.58, 0.60, 1.0))
    bright_runner = Material("brushed_inner_runner", color=(0.78, 0.80, 0.78, 1.0))
    black_plastic = Material("black_wear_strip", color=(0.02, 0.02, 0.018, 1.0))
    screw_dark = Material("black_socket_screw", color=(0.005, 0.005, 0.005, 1.0))

    outer = model.part("outer_channel")
    outer.visual(
        Box((0.720, 0.170, 0.012)),
        origin=Origin(xyz=(0.360, 0.0, 0.006)),
        material=anodized_dark,
        name="bottom_pan",
    )
    outer.visual(
        Box((0.720, 0.012, 0.084)),
        origin=Origin(xyz=(0.360, 0.079, 0.042)),
        material=anodized_dark,
        name="side_wall_0",
    )
    outer.visual(
        Box((0.720, 0.012, 0.084)),
        origin=Origin(xyz=(0.360, -0.079, 0.042)),
        material=anodized_dark,
        name="side_wall_1",
    )
    outer.visual(
        Box((0.720, 0.032, 0.014)),
        origin=Origin(xyz=(0.360, 0.059, 0.078)),
        material=anodized_dark,
        name="top_lip_0",
    )
    outer.visual(
        Box((0.720, 0.032, 0.014)),
        origin=Origin(xyz=(0.360, -0.059, 0.078)),
        material=anodized_dark,
        name="top_lip_1",
    )
    outer.visual(
        Box((0.012, 0.170, 0.090)),
        origin=Origin(xyz=(0.006, 0.0, 0.045)),
        material=anodized_dark,
        name="rear_stop",
    )
    for i, x in enumerate((0.180, 0.540)):
        outer.visual(
            Box((0.160, 0.240, 0.010)),
            origin=Origin(xyz=(x, 0.0, -0.004)),
            material=anodized_dark,
            name=f"ground_foot_{i}",
        )
        for j, y in enumerate((-0.105, 0.105)):
            outer.visual(
                Cylinder(radius=0.012, length=0.004),
                origin=Origin(xyz=(x, y, 0.0005)),
                material=screw_dark,
                name=f"screw_head_{i}_{j}",
            )

    middle = model.part("middle_runner")
    middle.visual(
        Box((0.600, 0.096, 0.030)),
        origin=Origin(xyz=(0.300, 0.0, 0.027)),
        material=bead_blasted,
        name="lower_slide_bar",
    )
    middle.visual(
        Box((0.585, 0.012, 0.018)),
        origin=Origin(xyz=(0.305, 0.053, 0.027)),
        material=black_plastic,
        name="wear_strip_0",
    )
    middle.visual(
        Box((0.585, 0.012, 0.018)),
        origin=Origin(xyz=(0.305, -0.053, 0.027)),
        material=black_plastic,
        name="wear_strip_1",
    )
    middle.visual(
        Box((0.600, 0.066, 0.014)),
        origin=Origin(xyz=(0.300, 0.0, 0.048)),
        material=bead_blasted,
        name="upper_saddle",
    )
    middle.visual(
        Box((0.590, 0.007, 0.020)),
        origin=Origin(xyz=(0.305, 0.0245, 0.064)),
        material=bead_blasted,
        name="inner_guide_0",
    )
    middle.visual(
        Box((0.590, 0.007, 0.020)),
        origin=Origin(xyz=(0.305, -0.0245, 0.064)),
        material=bead_blasted,
        name="inner_guide_1",
    )

    inner = model.part("inner_runner")
    inner.visual(
        Box((0.540, 0.028, 0.018)),
        origin=Origin(xyz=(0.270, 0.0, 0.064)),
        material=bright_runner,
        name="inner_bar",
    )
    inner.visual(
        Box((0.008, 0.028, 0.018)),
        origin=Origin(xyz=(0.542, 0.0, 0.064)),
        material=bright_runner,
        name="plain_end_face",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=0.0, upper=0.360),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.45, lower=0.0, upper=0.310),
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

    outer = object_model.get_part("outer_channel")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "two serial prismatic runners",
        outer_slide.articulation_type == ArticulationType.PRISMATIC
        and inner_slide.articulation_type == ArticulationType.PRISMATIC
        and inner_slide.parent == "middle_runner"
        and outer_slide.child == "middle_runner",
        details="The chain must be outer channel -> middle runner -> inner runner.",
    )
    ctx.expect_contact(
        middle,
        outer,
        elem_a="lower_slide_bar",
        elem_b="bottom_pan",
        contact_tol=0.0005,
        name="middle runner rides just above outer channel floor",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="inner_bar",
        elem_b="upper_saddle",
        contact_tol=0.0005,
        name="inner runner rides on middle saddle",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="y",
        margin=0.0,
        inner_elem="lower_slide_bar",
        outer_elem="bottom_pan",
        name="middle lower bar fits within outer width footprint",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        min_overlap=0.55,
        elem_a="lower_slide_bar",
        elem_b="bottom_pan",
        name="collapsed middle runner is substantially captured",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        min_overlap=0.50,
        elem_a="inner_bar",
        elem_b="upper_saddle",
        name="collapsed inner runner is substantially captured",
    )

    rest_inner_position = ctx.part_world_position(inner)
    with ctx.pose({outer_slide: 0.360, inner_slide: 0.310}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.25,
            elem_a="lower_slide_bar",
            elem_b="bottom_pan",
            name="extended middle runner keeps retained insertion",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.22,
            elem_a="inner_bar",
            elem_b="upper_saddle",
            name="extended inner runner keeps retained insertion",
        )
        ctx.expect_contact(
            middle,
            outer,
            elem_a="lower_slide_bar",
            elem_b="bottom_pan",
            contact_tol=0.0005,
            name="extended middle runner stays level in outer channel",
        )
        extended_inner_position = ctx.part_world_position(inner)

    ctx.check(
        "full chain extends in positive x",
        rest_inner_position is not None
        and extended_inner_position is not None
        and extended_inner_position[0] > rest_inner_position[0] + 0.60,
        details=f"rest={rest_inner_position}, extended={extended_inner_position}",
    )

    return ctx.report()


object_model = build_object_model()
