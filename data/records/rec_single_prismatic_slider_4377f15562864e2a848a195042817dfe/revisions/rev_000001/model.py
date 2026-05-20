from __future__ import annotations

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
    model = ArticulatedObject(name="instrument_drawer_slide")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.66, 0.68, 0.66, 1.0))
    graphite = model.material("graphite_glide", rgba=(0.04, 0.045, 0.05, 1.0))
    handle_mat = model.material("black_stop_handle", rgba=(0.015, 0.015, 0.013, 1.0))

    base = model.part("base_channel")
    base.visual(
        Box((0.60, 0.22, 0.018)),
        origin=Origin(xyz=(0.30, 0.0, 0.009)),
        material=anodized,
        name="channel_floor",
    )
    base.visual(
        Box((0.60, 0.018, 0.080)),
        origin=Origin(xyz=(0.30, 0.101, 0.056)),
        material=anodized,
        name="side_wall_0",
    )
    base.visual(
        Box((0.60, 0.034, 0.014)),
        origin=Origin(xyz=(0.30, 0.079, 0.101)),
        material=anodized,
        name="retaining_lip_0",
    )
    base.visual(
        Box((0.60, 0.018, 0.080)),
        origin=Origin(xyz=(0.30, -0.101, 0.056)),
        material=anodized,
        name="side_wall_1",
    )
    base.visual(
        Box((0.60, 0.034, 0.014)),
        origin=Origin(xyz=(0.30, -0.079, 0.101)),
        material=anodized,
        name="retaining_lip_1",
    )
    base.visual(
        Box((0.026, 0.22, 0.082)),
        origin=Origin(xyz=(0.013, 0.0, 0.057)),
        material=anodized,
        name="rear_stop",
    )

    carriage = model.part("carriage_tray")
    carriage.visual(
        Box((0.48, 0.104, 0.014)),
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        material=brushed,
        name="tray_slab",
    )
    carriage.visual(
        Box((0.48, 0.012, 0.034)),
        origin=Origin(xyz=(0.240, 0.052, 0.024)),
        material=brushed,
        name="tray_lip_0",
    )
    carriage.visual(
        Box((0.44, 0.012, 0.006)),
        origin=Origin(xyz=(0.230, 0.034, -0.009)),
        material=graphite,
        name="glide_pad_0",
    )
    carriage.visual(
        Box((0.48, 0.012, 0.034)),
        origin=Origin(xyz=(0.240, -0.052, 0.024)),
        material=brushed,
        name="tray_lip_1",
    )
    carriage.visual(
        Box((0.44, 0.012, 0.006)),
        origin=Origin(xyz=(0.230, -0.034, -0.009)),
        material=graphite,
        name="glide_pad_1",
    )
    carriage.visual(
        Box((0.065, 0.150, 0.058)),
        origin=Origin(xyz=(0.505, 0.0, 0.034)),
        material=handle_mat,
        name="front_stop",
    )

    model.articulation(
        "channel_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.080, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.22),
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

    base = object_model.get_part("base_channel")
    carriage = object_model.get_part("carriage_tray")
    slide = object_model.get_articulation("channel_to_carriage")

    ctx.check(
        "one fixed channel and one traveling carriage",
        len(object_model.parts) == 2 and len(object_model.articulations) == 1,
        details=f"parts={[p.name for p in object_model.parts]}, articulations={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "carriage uses a single prismatic slide axis",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="glide_pad_0",
        negative_elem="channel_floor",
        max_gap=0.0005,
        max_penetration=0.00001,
        name="glide pads bear on the fixed channel floor",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="y",
        inner_elem="tray_slab",
        outer_elem="channel_floor",
        margin=0.0,
        name="carriage tray is centered within the channel width",
    )
    ctx.expect_gap(
        base,
        carriage,
        axis="y",
        positive_elem="retaining_lip_0",
        negative_elem="tray_lip_0",
        min_gap=0.002,
        max_gap=0.006,
        name="upper retaining lip clears the carriage lip on one side",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="y",
        positive_elem="tray_lip_1",
        negative_elem="retaining_lip_1",
        min_gap=0.002,
        max_gap=0.006,
        name="upper retaining lip clears the carriage lip on the other side",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="tray_slab",
        elem_b="channel_floor",
        min_overlap=0.40,
        name="carriage is deeply retained in the collapsed channel",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.22}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="tray_slab",
            elem_b="channel_floor",
            min_overlap=0.25,
            name="carriage remains retained at full travel",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive travel pulls the carriage along the channel",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
