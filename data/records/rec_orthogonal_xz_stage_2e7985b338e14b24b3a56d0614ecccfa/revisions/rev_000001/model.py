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
    model = ArticulatedObject(name="side_wall_xz_module")

    powder_coat = model.material("warm_white_powder_coat", color=(0.78, 0.80, 0.76, 1.0))
    dark_rail = model.material("dark_anodized_rail", color=(0.05, 0.055, 0.06, 1.0))
    carriage_metal = model.material("brushed_carriage_aluminum", color=(0.62, 0.66, 0.68, 1.0))
    stage_blue = model.material("blue_stage_anodized", color=(0.08, 0.22, 0.65, 1.0))
    black_fasteners = model.material("black_fasteners", color=(0.01, 0.01, 0.012, 1.0))
    rubber = model.material("black_rubber_stops", color=(0.015, 0.015, 0.014, 1.0))

    side_support = model.part("side_support")
    side_support.visual(
        Box((1.00, 0.035, 1.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=powder_coat,
        name="wall_plate",
    )
    side_support.visual(
        Box((0.90, 0.033, 0.18)),
        origin=Origin(xyz=(0.0, 0.034, 0.20)),
        material=powder_coat,
        name="lower_rail_pad",
    )
    side_support.visual(
        Box((0.82, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.061, 0.155)),
        material=dark_rail,
        name="lower_rail",
    )
    side_support.visual(
        Box((0.82, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.061, 0.245)),
        material=dark_rail,
        name="upper_rail",
    )
    for x, name in ((-0.435, "rail_stop_0"), (0.435, "rail_stop_1")):
        side_support.visual(
            Box((0.035, 0.055, 0.18)),
            origin=Origin(xyz=(x, 0.077, 0.20)),
            material=rubber,
            name=name,
        )
    for x, name in ((-0.465, "side_rib_0"), (0.465, "side_rib_1")):
        side_support.visual(
            Box((0.035, 0.030, 0.92)),
            origin=Origin(xyz=(x, 0.032, 0.555)),
            material=powder_coat,
            name=name,
        )
    side_support.visual(
        Box((0.86, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, 0.032, 1.015)),
        material=powder_coat,
        name="top_rib",
    )
    bolt_pose = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))
    for i, (x, z) in enumerate(
        (
            (-0.39, 0.085),
            (0.39, 0.085),
            (-0.39, 0.965),
            (0.39, 0.965),
            (-0.28, 0.200),
            (0.28, 0.200),
        )
    ):
        side_support.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(x, 0.0225, z), rpy=bolt_pose.rpy),
            material=black_fasteners,
            name=f"wall_bolt_{i}",
        )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        Box((0.18, 0.046, 0.150)),
        origin=Origin(xyz=(0.0, 0.095, 0.200)),
        material=carriage_metal,
        name="saddle_plate",
    )
    lower_carriage.visual(
        Box((0.11, 0.040, 0.085)),
        origin=Origin(xyz=(0.0, 0.098, 0.315)),
        material=carriage_metal,
        name="guide_pedestal",
    )
    lower_carriage.visual(
        Box((0.020, 0.042, 0.500)),
        origin=Origin(xyz=(-0.055, 0.115, 0.500)),
        material=carriage_metal,
        name="guide_column_0",
    )
    lower_carriage.visual(
        Box((0.020, 0.042, 0.500)),
        origin=Origin(xyz=(0.055, 0.115, 0.500)),
        material=carriage_metal,
        name="guide_column_1",
    )
    for z, name in ((0.250, "guide_bridge_0"), (0.750, "guide_bridge_1")):
        lower_carriage.visual(
            Box((0.130, 0.042, 0.026)),
            origin=Origin(xyz=(0.0, 0.115, z)),
            material=carriage_metal,
            name=name,
        )
    for i, (x, z) in enumerate(((-0.055, 0.170), (0.055, 0.170), (-0.055, 0.235), (0.055, 0.235))):
        lower_carriage.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(x, 0.122, z), rpy=bolt_pose.rpy),
            material=black_fasteners,
            name=f"saddle_bolt_{i}",
        )

    vertical_stage = model.part("vertical_stage")
    vertical_stage.visual(
        Box((0.050, 0.032, 0.520)),
        origin=Origin(xyz=(0.0, 0.152, 0.260)),
        material=stage_blue,
        name="stage_bar",
    )
    vertical_stage.visual(
        Box((0.090, 0.044, 0.036)),
        origin=Origin(xyz=(0.0, 0.190, 0.520)),
        material=stage_blue,
        name="top_cap",
    )
    vertical_stage.visual(
        Box((0.070, 0.016, 0.170)),
        origin=Origin(xyz=(0.0, 0.176, 0.290)),
        material=black_fasteners,
        name="front_runner",
    )
    for i, z in enumerate((0.125, 0.250, 0.375)):
        vertical_stage.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.0, 0.171, z), rpy=bolt_pose.rpy),
            material=black_fasteners,
            name=f"stage_screw_{i}",
        )

    model.articulation(
        "horizontal_slide",
        ArticulationType.PRISMATIC,
        parent=side_support,
        child=lower_carriage,
        origin=Origin(xyz=(-0.25, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.50),
    )
    model.articulation(
        "vertical_slide",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=vertical_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.28),
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

    side_support = object_model.get_part("side_support")
    lower_carriage = object_model.get_part("lower_carriage")
    vertical_stage = object_model.get_part("vertical_stage")
    horizontal_slide = object_model.get_articulation("horizontal_slide")
    vertical_slide = object_model.get_articulation("vertical_slide")

    ctx.check(
        "two prismatic axes",
        horizontal_slide.articulation_type == ArticulationType.PRISMATIC
        and vertical_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(horizontal_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(vertical_slide.axis) == (0.0, 0.0, 1.0),
        details=f"horizontal={horizontal_slide.articulation_type}/{horizontal_slide.axis}, "
        f"vertical={vertical_slide.articulation_type}/{vertical_slide.axis}",
    )

    ctx.expect_gap(
        lower_carriage,
        side_support,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="saddle_plate",
        negative_elem="upper_rail",
        name="horizontal carriage rides on rail faces",
    )
    ctx.expect_within(
        lower_carriage,
        side_support,
        axes="x",
        inner_elem="saddle_plate",
        outer_elem="lower_rail_pad",
        margin=0.0,
        name="carriage starts within the lower slide rail",
    )
    ctx.expect_gap(
        vertical_stage,
        lower_carriage,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="stage_bar",
        negative_elem="guide_column_0",
        name="vertical stage rides on guide face",
    )
    ctx.expect_overlap(
        vertical_stage,
        lower_carriage,
        axes="z",
        elem_a="stage_bar",
        elem_b="guide_column_0",
        min_overlap=0.35,
        name="vertical stage is well engaged at rest",
    )

    carriage_rest = ctx.part_world_position(lower_carriage)
    stage_rest = ctx.part_world_position(vertical_stage)
    with ctx.pose({horizontal_slide: 0.50, vertical_slide: 0.28}):
        carriage_extended = ctx.part_world_position(lower_carriage)
        stage_raised = ctx.part_world_position(vertical_stage)
        ctx.expect_within(
            lower_carriage,
            side_support,
            axes="x",
            inner_elem="saddle_plate",
            outer_elem="lower_rail_pad",
            margin=0.0,
            name="carriage remains within rail at full horizontal travel",
        )
        ctx.expect_overlap(
            vertical_stage,
            lower_carriage,
            axes="z",
            elem_a="stage_bar",
            elem_b="guide_column_0",
            min_overlap=0.15,
            name="raised vertical stage keeps retained insertion",
        )

    ctx.check(
        "horizontal slide moves along X",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.49,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )
    ctx.check(
        "vertical slide raises stage along Z",
        stage_rest is not None
        and stage_raised is not None
        and stage_raised[2] > stage_rest[2] + 0.27,
        details=f"rest={stage_rest}, raised={stage_raised}",
    )

    return ctx.report()


object_model = build_object_model()
