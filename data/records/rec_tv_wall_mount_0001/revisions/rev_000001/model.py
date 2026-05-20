from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
from math import pi


def _box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
    )


def _cylinder_visual(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tv_wall_mount", assets=ASSETS)

    model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("zinc", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    axis_x_rpy = (0.0, pi / 2.0, 0.0)

    wall_plate = model.part("wall_plate")
    _box_visual(wall_plate, (0.014, 0.240, 0.340), (0.007, 0.0, 0.0), "powder_black")
    _box_visual(wall_plate, (0.024, 0.026, 0.280), (0.019, -0.072, 0.0), "graphite")
    _box_visual(wall_plate, (0.024, 0.026, 0.280), (0.019, 0.072, 0.0), "graphite")
    _box_visual(wall_plate, (0.038, 0.120, 0.120), (0.024, 0.0, 0.0), "graphite")
    _box_visual(wall_plate, (0.024, 0.088, 0.060), (0.038, 0.0, 0.056), "graphite")
    _box_visual(wall_plate, (0.024, 0.088, 0.060), (0.038, 0.0, -0.056), "graphite")
    _box_visual(wall_plate, (0.020, 0.100, 0.070), (0.047, 0.0, 0.0), "graphite")
    _cylinder_visual(wall_plate, 0.018, 0.050, (0.055, 0.0, 0.0), "zinc")
    for screw_y in (-0.080, 0.080):
        for screw_z in (-0.110, 0.110):
            _cylinder_visual(
                wall_plate,
                0.009,
                0.004,
                (0.010, screw_y, screw_z),
                "zinc",
                axis_x_rpy,
            )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.070, 0.240, 0.340)),
        mass=3.2,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
    )

    inner_arm = model.part("inner_arm")
    _cylinder_visual(inner_arm, 0.017, 0.040, (0.0, 0.0, 0.0), "zinc")
    _box_visual(inner_arm, (0.032, 0.070, 0.046), (0.012, 0.0, 0.0), "graphite")
    _box_visual(inner_arm, (0.128, 0.014, 0.026), (0.078, -0.022, 0.0), "powder_black")
    _box_visual(inner_arm, (0.128, 0.014, 0.026), (0.078, 0.022, 0.0), "powder_black")
    _box_visual(inner_arm, (0.020, 0.058, 0.018), (0.042, 0.0, 0.0), "graphite")
    _box_visual(inner_arm, (0.020, 0.058, 0.018), (0.110, 0.0, 0.0), "graphite")
    _box_visual(inner_arm, (0.082, 0.020, 0.010), (0.082, 0.0, 0.014), "graphite")
    _box_visual(inner_arm, (0.026, 0.070, 0.040), (0.145, 0.0, 0.0), "graphite")
    _cylinder_visual(inner_arm, 0.018, 0.040, (0.145, 0.0, 0.0), "zinc")
    inner_arm.inertial = Inertial.from_geometry(
        Box((0.170, 0.070, 0.050)),
        mass=1.7,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    outer_arm = model.part("outer_arm")
    _cylinder_visual(outer_arm, 0.017, 0.040, (0.0, 0.0, 0.0), "zinc")
    _box_visual(outer_arm, (0.030, 0.062, 0.044), (0.011, 0.0, 0.0), "graphite")
    _box_visual(outer_arm, (0.120, 0.013, 0.024), (0.074, -0.019, 0.0), "powder_black")
    _box_visual(outer_arm, (0.120, 0.013, 0.024), (0.074, 0.019, 0.0), "powder_black")
    _box_visual(outer_arm, (0.018, 0.052, 0.018), (0.036, 0.0, 0.0), "graphite")
    _box_visual(outer_arm, (0.018, 0.052, 0.018), (0.100, 0.0, 0.0), "graphite")
    _box_visual(outer_arm, (0.064, 0.018, 0.010), (0.072, 0.0, -0.013), "graphite")
    _box_visual(outer_arm, (0.028, 0.062, 0.040), (0.137, 0.0, 0.0), "graphite")
    _cylinder_visual(outer_arm, 0.0175, 0.040, (0.137, 0.0, 0.0), "zinc")
    outer_arm.inertial = Inertial.from_geometry(
        Box((0.158, 0.064, 0.048)),
        mass=1.4,
        origin=Origin(xyz=(0.079, 0.0, 0.0)),
    )

    tilt_head = model.part("tilt_head")
    _cylinder_visual(tilt_head, 0.018, 0.042, (0.0, 0.0, 0.0), "zinc")
    _box_visual(tilt_head, (0.032, 0.072, 0.044), (0.012, 0.0, 0.0), "graphite")
    _box_visual(tilt_head, (0.060, 0.050, 0.028), (0.042, 0.0, 0.0), "powder_black")
    _box_visual(tilt_head, (0.060, 0.010, 0.090), (0.050, -0.028, 0.0), "graphite")
    _box_visual(tilt_head, (0.060, 0.010, 0.090), (0.050, 0.028, 0.0), "graphite")
    _box_visual(tilt_head, (0.028, 0.072, 0.014), (0.038, 0.0, 0.032), "graphite")
    _box_visual(tilt_head, (0.024, 0.072, 0.014), (0.046, 0.0, -0.032), "graphite")
    _cylinder_visual(tilt_head, 0.015, 0.074, (0.072, 0.0, 0.0), "zinc", (pi / 2.0, 0.0, 0.0))
    tilt_head.inertial = Inertial.from_geometry(
        Box((0.090, 0.082, 0.098)),
        mass=0.9,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
    )

    screen_bracket = model.part("screen_bracket")
    _box_visual(screen_bracket, (0.028, 0.110, 0.030), (0.014, 0.0, 0.0), "graphite")
    _box_visual(screen_bracket, (0.018, 0.140, 0.170), (0.012, 0.0, 0.0), "powder_black")
    _box_visual(screen_bracket, (0.014, 0.220, 0.018), (0.010, 0.0, 0.040), "graphite")
    _box_visual(screen_bracket, (0.014, 0.220, 0.018), (0.010, 0.0, -0.040), "graphite")
    _box_visual(screen_bracket, (0.016, 0.320, 0.020), (0.014, 0.0, 0.110), "powder_black")
    _box_visual(screen_bracket, (0.016, 0.320, 0.020), (0.014, 0.0, -0.110), "powder_black")
    _box_visual(screen_bracket, (0.014, 0.024, 0.280), (0.015, -0.110, 0.0), "powder_black")
    _box_visual(screen_bracket, (0.014, 0.024, 0.280), (0.015, 0.110, 0.0), "powder_black")
    _box_visual(screen_bracket, (0.020, 0.050, 0.040), (0.020, -0.110, 0.060), "graphite")
    _box_visual(screen_bracket, (0.020, 0.050, 0.040), (0.020, 0.110, 0.060), "graphite")
    _box_visual(screen_bracket, (0.020, 0.050, 0.040), (0.020, -0.110, -0.060), "graphite")
    _box_visual(screen_bracket, (0.020, 0.050, 0.040), (0.020, 0.110, -0.060), "graphite")
    _box_visual(screen_bracket, (0.028, 0.028, 0.024), (0.024, -0.110, 0.138), "graphite")
    _box_visual(screen_bracket, (0.028, 0.028, 0.024), (0.024, 0.110, 0.138), "graphite")
    _box_visual(screen_bracket, (0.028, 0.028, 0.024), (0.024, -0.110, -0.138), "graphite")
    _box_visual(screen_bracket, (0.028, 0.028, 0.024), (0.024, 0.110, -0.138), "graphite")
    _box_visual(screen_bracket, (0.006, 0.050, 0.050), (0.009, -0.085, 0.065), "rubber")
    _box_visual(screen_bracket, (0.006, 0.050, 0.050), (0.009, 0.085, 0.065), "rubber")
    _box_visual(screen_bracket, (0.006, 0.050, 0.050), (0.009, -0.085, -0.065), "rubber")
    _box_visual(screen_bracket, (0.006, 0.050, 0.050), (0.009, 0.085, -0.065), "rubber")
    for boss_y in (-0.100, 0.100):
        for boss_z in (-0.060, 0.060):
            _cylinder_visual(
                screen_bracket,
                0.010,
                0.008,
                (0.020, boss_y, boss_z),
                "zinc",
                axis_x_rpy,
            )
    screen_bracket.inertial = Inertial.from_geometry(
        Box((0.040, 0.320, 0.300)),
        mass=2.4,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "wall_swing",
        ArticulationType.REVOLUTE,
        parent="wall_plate",
        child="inner_arm",
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.15,
            upper=1.15,
            effort=180.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "elbow_swing",
        ArticulationType.REVOLUTE,
        parent="inner_arm",
        child="outer_arm",
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.45,
            upper=1.45,
            effort=160.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent="outer_arm",
        child="tilt_head",
        origin=Origin(xyz=(0.137, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-0.45,
            upper=0.45,
            effort=90.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "screen_tilt",
        ArticulationType.REVOLUTE,
        parent="tilt_head",
        child="screen_bracket",
        origin=Origin(xyz=(0.072, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=0.20,
            effort=70.0,
            velocity=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "wall_plate",
        "inner_arm",
        reason="segmented wall hinge sleeves are modeled with tight practical clearances",
    )
    ctx.allow_overlap(
        "inner_arm",
        "outer_arm",
        reason="elbow hinge sleeves are closely nested around the revolute pin axis",
    )
    ctx.allow_overlap(
        "outer_arm",
        "tilt_head",
        reason="front swivel hinge geometry uses compact concentric sleeves",
    )
    ctx.allow_overlap(
        "tilt_head",
        "screen_bracket",
        reason="tilt trunnion sleeves can register as conservative collision contact",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("inner_arm", "wall_plate", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_overlap("outer_arm", "inner_arm", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_overlap("screen_bracket", "tilt_head", axes="xy", min_overlap=0.01)
    ctx.expect_origin_distance("screen_bracket", "wall_plate", axes="xy", max_dist=0.42)
    ctx.expect_aabb_gap("screen_bracket", "wall_plate", axis="z", max_gap=0.06, max_penetration=0.34)

    ctx.expect_joint_motion_axis(
        "wall_swing",
        "inner_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "elbow_swing",
        "outer_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "head_swivel",
        "screen_bracket",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "screen_tilt",
        "screen_bracket",
        world_axis="z",
        direction="negative",
        min_delta=0.01,
    )

    with ctx.pose(wall_swing=1.00, elbow_swing=1.20, head_swivel=0.35):
        ctx.expect_origin_distance("screen_bracket", "wall_plate", axes="xy", max_dist=0.56)
        ctx.expect_aabb_gap("screen_bracket", "wall_plate", axis="z", max_gap=0.10, max_penetration=0.36)

    with ctx.pose(wall_swing=-1.00, elbow_swing=-1.10, head_swivel=-0.35):
        ctx.expect_origin_distance("screen_bracket", "wall_plate", axes="xy", max_dist=0.56)
        ctx.expect_aabb_gap("screen_bracket", "wall_plate", axis="z", max_gap=0.10, max_penetration=0.36)

    with ctx.pose(screen_tilt=0.20):
        ctx.expect_aabb_overlap("screen_bracket", "tilt_head", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance("screen_bracket", "tilt_head", axes="xy", max_dist=0.16)

    with ctx.pose(screen_tilt=-0.30):
        ctx.expect_aabb_overlap("screen_bracket", "tilt_head", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance("screen_bracket", "tilt_head", axes="xy", max_dist=0.16)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
