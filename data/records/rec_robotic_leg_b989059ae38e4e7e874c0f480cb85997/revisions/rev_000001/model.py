from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    painted_steel = model.material("painted_steel", rgba=(0.24, 0.27, 0.31, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.13, 0.14, 0.16, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.76, 0.80, 1.0))
    foot_pad = model.material("foot_pad", rgba=(0.09, 0.10, 0.11, 1.0))
    accent = model.material("accent", rgba=(0.84, 0.49, 0.14, 1.0))

    hip_frame = model.part("hip_frame")
    hip_frame.inertial = Inertial.from_geometry(
        Box((0.22, 0.20, 0.26)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    hip_frame.visual(
        Box((0.16, 0.016, 0.26)),
        origin=Origin(xyz=(0.0, 0.078, -0.01)),
        material=painted_steel,
        name="left_cheek",
    )
    hip_frame.visual(
        Box((0.16, 0.016, 0.26)),
        origin=Origin(xyz=(0.0, -0.078, -0.01)),
        material=painted_steel,
        name="right_cheek",
    )
    hip_frame.visual(
        Box((0.16, 0.156, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=painted_steel,
        name="top_bridge",
    )
    hip_frame.visual(
        Box((0.024, 0.156, 0.19)),
        origin=Origin(xyz=(-0.052, 0.0, -0.015)),
        material=dark_steel,
        name="rear_spine",
    )
    hip_frame.visual(
        Box((0.10, 0.156, 0.044)),
        origin=Origin(xyz=(0.022, 0.0, 0.056)),
        material=dark_steel,
        name="front_crossmember",
    )
    hip_frame.visual(
        Cylinder(radius=0.041, length=0.028),
        origin=Origin(xyz=(0.0, 0.099, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="left_hip_cap",
    )
    hip_frame.visual(
        Cylinder(radius=0.041, length=0.028),
        origin=Origin(xyz=(0.0, -0.099, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="right_hip_cap",
    )
    hip_frame.visual(
        Box((0.050, 0.11, 0.070)),
        origin=Origin(xyz=(-0.060, 0.0, 0.078)),
        material=accent,
        name="motor_cover",
    )

    thigh = model.part("thigh")
    thigh.inertial = Inertial.from_geometry(
        Box((0.13, 0.12, 0.34)),
        mass=7.5,
        origin=Origin(xyz=(0.008, 0.0, -0.17)),
    )
    thigh.visual(
        Box((0.056, 0.022, 0.27)),
        origin=Origin(xyz=(0.012, 0.046, -0.146)),
        material=dark_steel,
        name="left_spar",
    )
    thigh.visual(
        Box((0.056, 0.022, 0.27)),
        origin=Origin(xyz=(0.012, -0.046, -0.146)),
        material=dark_steel,
        name="right_spar",
    )
    thigh.visual(
        Box((0.072, 0.076, 0.092)),
        origin=Origin(xyz=(0.018, 0.0, -0.078)),
        material=painted_steel,
        name="upper_housing",
    )
    thigh.visual(
        Box((0.095, 0.086, 0.054)),
        origin=Origin(xyz=(0.024, 0.0, -0.306)),
        material=painted_steel,
        name="knee_block",
    )
    thigh.visual(
        Cylinder(radius=0.034, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hip_hub",
    )
    thigh.visual(
        Cylinder(radius=0.030, length=0.096),
        origin=Origin(xyz=(0.024, 0.0, -0.306), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="knee_hub",
    )
    thigh.visual(
        Box((0.050, 0.086, 0.070)),
        origin=Origin(xyz=(0.030, 0.0, -0.245)),
        material=accent,
        name="thigh_service_cover",
    )

    shank = model.part("shank")
    shank.inertial = Inertial.from_geometry(
        Box((0.12, 0.11, 0.31)),
        mass=5.2,
        origin=Origin(xyz=(0.018, 0.0, -0.16)),
    )
    shank.visual(
        Box((0.026, 0.020, 0.23)),
        origin=Origin(xyz=(0.014, 0.045, -0.165)),
        material=dark_steel,
        name="left_tibia_rail",
    )
    shank.visual(
        Box((0.026, 0.020, 0.23)),
        origin=Origin(xyz=(0.014, -0.045, -0.165)),
        material=dark_steel,
        name="right_tibia_rail",
    )
    shank.visual(
        Box((0.080, 0.078, 0.052)),
        origin=Origin(xyz=(0.020, 0.0, -0.056)),
        material=painted_steel,
        name="upper_shank_bridge",
    )
    shank.visual(
        Box((0.074, 0.072, 0.18)),
        origin=Origin(xyz=(0.028, 0.0, -0.175)),
        material=painted_steel,
        name="shank_body",
    )
    shank.visual(
        Box((0.078, 0.082, 0.046)),
        origin=Origin(xyz=(0.018, 0.0, -0.302)),
        material=painted_steel,
        name="ankle_mount",
    )
    shank.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.012, 0.064, -0.056), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="left_knee_cap",
    )
    shank.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.012, -0.064, -0.056), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="right_knee_cap",
    )
    shank.visual(
        Cylinder(radius=0.023, length=0.020),
        origin=Origin(xyz=(0.018, 0.065, -0.302), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="left_ankle_cap",
    )
    shank.visual(
        Cylinder(radius=0.023, length=0.020),
        origin=Origin(xyz=(0.018, -0.065, -0.302), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="right_ankle_cap",
    )
    shank.visual(
        Box((0.045, 0.070, 0.050)),
        origin=Origin(xyz=(0.040, 0.0, -0.132)),
        material=accent,
        name="shank_cover",
    )

    foot = model.part("foot")
    foot.inertial = Inertial.from_geometry(
        Box((0.22, 0.12, 0.12)),
        mass=2.9,
        origin=Origin(xyz=(0.055, 0.0, -0.075)),
    )
    foot.visual(
        Box((0.064, 0.076, 0.042)),
        origin=Origin(xyz=(0.015, 0.0, -0.044)),
        material=painted_steel,
        name="ankle_case",
    )
    foot.visual(
        Box((0.170, 0.104, 0.032)),
        origin=Origin(xyz=(0.066, 0.0, -0.096)),
        material=dark_steel,
        name="sole_beam",
    )
    foot.visual(
        Box((0.060, 0.098, 0.020)),
        origin=Origin(xyz=(-0.034, 0.0, -0.090)),
        material=dark_steel,
        name="heel_block",
    )
    foot.visual(
        Cylinder(radius=0.017, length=0.102),
        origin=Origin(xyz=(0.155, 0.0, -0.098), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="toe_roller",
    )
    foot.visual(
        Box((0.074, 0.098, 0.014)),
        origin=Origin(xyz=(0.138, 0.0, -0.119)),
        material=foot_pad,
        name="toe_pad",
    )
    foot.visual(
        Box((0.060, 0.094, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0, -0.112)),
        material=foot_pad,
        name="heel_pad",
    )
    foot.visual(
        Box((0.040, 0.072, 0.046)),
        origin=Origin(xyz=(0.030, 0.0, -0.071)),
        material=accent,
        name="ankle_service_cover",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_frame,
        child=thigh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=280.0, velocity=2.2, lower=-0.8, upper=1.1),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.024, 0.0, -0.306)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=240.0, velocity=2.6, lower=0.0, upper=2.2),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.018, 0.0, -0.302)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=3.0, lower=-0.55, upper=0.55),
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

    hip_frame = object_model.get_part("hip_frame")
    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    ctx.expect_gap(
        hip_frame,
        thigh,
        axis="y",
        positive_elem="left_cheek",
        negative_elem="hip_hub",
        min_gap=0.008,
        max_gap=0.030,
        name="left cheek clears hip hub",
    )
    ctx.expect_gap(
        thigh,
        hip_frame,
        axis="y",
        positive_elem="hip_hub",
        negative_elem="right_cheek",
        min_gap=0.008,
        max_gap=0.030,
        name="right cheek clears hip hub",
    )
    ctx.expect_origin_gap(
        thigh,
        shank,
        axis="z",
        min_gap=0.22,
        name="shank joint sits below hip joint",
    )
    ctx.expect_origin_gap(
        shank,
        foot,
        axis="z",
        min_gap=0.24,
        name="ankle joint sits below knee joint",
    )

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({hip_pitch: 0.45}):
        hip_swing_foot = ctx.part_world_position(foot)
    ctx.check(
        "positive hip pitch swings the leg forward",
        rest_foot is not None
        and hip_swing_foot is not None
        and hip_swing_foot[0] > rest_foot[0] + 0.18
        and hip_swing_foot[2] > rest_foot[2] + 0.05,
        details=f"rest={rest_foot}, hip_swing={hip_swing_foot}",
    )

    with ctx.pose({knee_pitch: 1.10}):
        knee_flex_foot = ctx.part_world_position(foot)
    ctx.check(
        "positive knee pitch folds the shank forward",
        rest_foot is not None
        and knee_flex_foot is not None
        and knee_flex_foot[0] > rest_foot[0] + 0.16
        and knee_flex_foot[2] > rest_foot[2] + 0.10,
        details=f"rest={rest_foot}, knee_flex={knee_flex_foot}",
    )

    rest_toe = ctx.part_element_world_aabb(foot, elem="toe_pad")
    with ctx.pose({ankle_pitch: 0.35}):
        raised_toe = ctx.part_element_world_aabb(foot, elem="toe_pad")
    ctx.check(
        "positive ankle pitch raises the toe",
        rest_toe is not None
        and raised_toe is not None
        and raised_toe[1][2] > rest_toe[1][2] + 0.02,
        details=f"rest_toe={rest_toe}, raised_toe={raised_toe}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
