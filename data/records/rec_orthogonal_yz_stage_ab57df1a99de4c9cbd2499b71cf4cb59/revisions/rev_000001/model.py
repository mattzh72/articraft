from __future__ import annotations

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
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_yz_transfer_unit")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("black_hardware", rgba=(0.02, 0.02, 0.018, 1.0))
    rail_steel = model.material("polished_rail_steel", rgba=(0.88, 0.90, 0.88, 1.0))
    blue = model.material("anodized_blue_carriage", rgba=(0.05, 0.22, 0.72, 1.0))
    bronze = model.material("bronze_wear_pads", rgba=(0.76, 0.48, 0.18, 1.0))
    orange = model.material("orange_vertical_stage", rgba=(0.95, 0.36, 0.08, 1.0))

    plate = model.part("back_plate")
    plate.visual(
        Box((0.070, 1.300, 0.900)),
        origin=Origin(xyz=(-0.035, 0.0, 0.450)),
        material=dark_steel,
        name="vertical_plate",
    )
    plate.visual(
        Box((0.290, 1.120, 0.060)),
        origin=Origin(xyz=(0.045, 0.0, 0.030)),
        material=black,
        name="floor_plinth",
    )
    for y, name in [(-0.610, "side_post_0"), (0.610, "side_post_1")]:
        plate.visual(
            Box((0.120, 0.040, 0.860)),
            origin=Origin(xyz=(0.015, y, 0.450)),
            material=black,
            name=name,
        )
    plate.visual(
        Box((0.100, 1.140, 0.105)),
        origin=Origin(xyz=(0.020, 0.0, 0.640)),
        material=aluminum,
        name="bridge_beam",
    )
    plate.visual(
        Box((0.030, 1.060, 0.250)),
        origin=Origin(xyz=(0.057, 0.0, 0.640)),
        material=aluminum,
        name="rail_backing",
    )
    for y, name in [(-0.540, "end_block_0"), (0.540, "end_block_1")]:
        plate.visual(
            Box((0.090, 0.075, 0.260)),
            origin=Origin(xyz=(0.057, y, 0.640)),
            material=aluminum,
            name=name,
        )
    for z, name in [(0.720, "upper_y_rail"), (0.560, "lower_y_rail")]:
        plate.visual(
            Cylinder(radius=0.012, length=1.080),
            origin=Origin(xyz=(0.075, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rail_steel,
            name=name,
        )
    plate.visual(
        Box((0.018, 1.050, 0.035)),
        origin=Origin(xyz=(0.086, 0.0, 0.640)),
        material=black,
        name="center_scale_strip",
    )

    carriage = model.part("beam_carriage")
    carriage.visual(
        Box((0.060, 0.240, 0.170)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=blue,
        name="cross_slide_body",
    )
    carriage.visual(
        Box((0.018, 0.230, 0.195)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        material=aluminum,
        name="bearing_backplate",
    )
    for z, name in [(0.080, "upper_y_shoe"), (-0.080, "lower_y_shoe")]:
        carriage.visual(
            Box((0.020, 0.180, 0.038)),
            origin=Origin(xyz=(-0.023, 0.0, z)),
            material=bronze,
            name=name,
        )
    carriage.visual(
        Box((0.055, 0.150, 0.100)),
        origin=Origin(xyz=(0.045, 0.0, -0.125)),
        material=blue,
        name="stage_hanger",
    )
    carriage.visual(
        Box((0.018, 0.110, 0.315)),
        origin=Origin(xyz=(0.050, 0.0, -0.285)),
        material=aluminum,
        name="vertical_guide_web",
    )
    for y, name in [(-0.066, "guide_cheek_0"), (0.066, "guide_cheek_1")]:
        carriage.visual(
            Box((0.050, 0.026, 0.315)),
            origin=Origin(xyz=(0.065, y, -0.285)),
            material=blue,
            name=name,
        )
        carriage.visual(
            Box((0.010, 0.024, 0.285)),
            origin=Origin(xyz=(0.095, y, -0.285)),
            material=bronze,
            name=f"z_wear_pad_{0 if y < 0 else 1}",
        )

    stage = model.part("vertical_stage")
    stage.visual(
        Box((0.040, 0.110, 0.380)),
        origin=Origin(xyz=(0.020, 0.0, -0.190)),
        material=orange,
        name="vertical_slide_plate",
    )
    for y, name in [(-0.040, "stage_rail_0"), (0.040, "stage_rail_1")]:
        stage.visual(
            Cylinder(radius=0.007, length=0.350),
            origin=Origin(xyz=(0.043, y, -0.185)),
            material=rail_steel,
            name=name,
        )
    stage.visual(
        Box((0.080, 0.150, 0.060)),
        origin=Origin(xyz=(0.025, 0.0, -0.385)),
        material=orange,
        name="tool_mount_plate",
    )
    stage.visual(
        Box((0.050, 0.100, 0.018)),
        origin=Origin(xyz=(0.065, 0.0, -0.4235)),
        material=black,
        name="tool_face",
    )

    model.articulation(
        "plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=plate,
        child=carriage,
        origin=Origin(xyz=(0.120, 0.0, 0.640)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=-0.350, upper=0.350),
    )
    model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=(0.100, 0.0, -0.135)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.30, lower=0.0, upper=0.240),
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

    plate = object_model.get_part("back_plate")
    carriage = object_model.get_part("beam_carriage")
    stage = object_model.get_part("vertical_stage")
    y_slide = object_model.get_articulation("plate_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_stage")

    ctx.expect_within(
        carriage,
        plate,
        axes="y",
        margin=0.0,
        name="beam carriage remains on the bridge width at center",
    )
    ctx.expect_origin_gap(
        carriage,
        stage,
        axis="z",
        min_gap=0.100,
        name="vertical stage is mounted below the beam carriage",
    )
    ctx.expect_overlap(
        stage,
        carriage,
        axes="xy",
        min_overlap=0.010,
        name="vertical stage stays aligned under its carriage guide",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({y_slide: 0.300}):
        shifted_carriage = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            plate,
            axes="y",
            margin=0.0,
            name="beam carriage remains on the bridge width near travel end",
        )
    ctx.check(
        "horizontal prismatic joint moves carriage crosswise",
        rest_carriage is not None
        and shifted_carriage is not None
        and shifted_carriage[1] > rest_carriage[1] + 0.250,
        details=f"rest={rest_carriage}, shifted={shifted_carriage}",
    )

    rest_stage = ctx.part_world_position(stage)
    with ctx.pose({z_slide: 0.200}):
        lowered_stage = ctx.part_world_position(stage)
        ctx.expect_origin_gap(
            carriage,
            stage,
            axis="z",
            min_gap=0.250,
            name="vertical stage lowers below the carriage at extension",
        )
    ctx.check(
        "vertical prismatic joint lowers the short stage",
        rest_stage is not None
        and lowered_stage is not None
        and lowered_stage[2] < rest_stage[2] - 0.150,
        details=f"rest={rest_stage}, lowered={lowered_stage}",
    )

    return ctx.report()


object_model = build_object_model()
