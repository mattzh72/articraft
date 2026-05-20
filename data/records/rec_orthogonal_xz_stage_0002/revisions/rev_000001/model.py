from __future__ import annotations

from math import pi

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
def _origin(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(xyz=xyz, rpy=rpy)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str | None = None,
    collision: bool = False,
) -> None:
    origin = _origin(xyz)
    part.visual(Box(size), origin=origin, material=material)
    if collision:
        pass


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material: str | None = None,
    collision: bool = False,
) -> None:
    origin = _origin(xyz, rpy=rpy)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material)
    if collision:
        pass


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_positioning_stage", assets=ASSETS)

    model.material("frame_gray", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    model.material("dark_hardware", rgba=(0.15, 0.15, 0.17, 1.0))
    model.material("payload_orange", rgba=(0.96, 0.56, 0.18, 1.0))

    base = model.part("base")
    _add_box(base, (0.36, 0.18, 0.028), (0.0, 0.0, 0.014), material="frame_gray", collision=True)
    _add_box(base, (0.28, 0.016, 0.018), (0.0, 0.055, 0.037), material="rail_steel", collision=True)
    _add_box(
        base, (0.28, 0.016, 0.018), (0.0, -0.055, 0.037), material="rail_steel", collision=True
    )
    _add_cylinder(
        base,
        radius=0.006,
        length=0.24,
        xyz=(0.0, 0.0, 0.040),
        rpy=(0.0, pi / 2.0, 0.0),
        material="dark_hardware",
        collision=True,
    )
    _add_box(base, (0.022, 0.090, 0.028), (0.145, 0.0, 0.042), material="dark_hardware")
    _add_box(base, (0.022, 0.090, 0.028), (-0.145, 0.0, 0.042), material="dark_hardware")
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.18, 0.028)),
        mass=7.5,
        origin=_origin((0.0, 0.0, 0.014)),
    )

    x_stage = model.part("x_stage")
    _add_box(
        x_stage, (0.108, 0.050, 0.014), (0.0, 0.0, 0.007), material="dark_hardware", collision=True
    )
    _add_box(
        x_stage, (0.150, 0.140, 0.012), (0.0, 0.0, 0.021), material="frame_gray", collision=True
    )
    _add_box(x_stage, (0.032, 0.024, 0.020), (0.040, 0.055, 0.010), material="dark_hardware")
    _add_box(x_stage, (0.032, 0.024, 0.020), (-0.040, 0.055, 0.010), material="dark_hardware")
    _add_box(x_stage, (0.032, 0.024, 0.020), (0.040, -0.055, 0.010), material="dark_hardware")
    _add_box(x_stage, (0.032, 0.024, 0.020), (-0.040, -0.055, 0.010), material="dark_hardware")
    _add_box(x_stage, (0.132, 0.018, 0.180), (0.0, 0.0, 0.105), material="frame_gray")
    _add_box(x_stage, (0.120, 0.016, 0.180), (0.0, 0.0, 0.105), collision=True)
    _add_box(x_stage, (0.016, 0.012, 0.140), (0.036, 0.015, 0.087), material="rail_steel")
    _add_box(x_stage, (0.016, 0.012, 0.140), (-0.036, 0.015, 0.087), material="rail_steel")
    _add_box(x_stage, (0.090, 0.040, 0.010), (0.0, 0.012, 0.190), material="dark_hardware")
    x_stage.inertial = Inertial.from_geometry(
        Box((0.15, 0.14, 0.19)),
        mass=2.4,
        origin=_origin((0.0, 0.0, 0.095)),
    )

    z_stage = model.part("z_stage")
    _add_box(z_stage, (0.092, 0.026, 0.060), (0.0, 0.023, 0.030), material="frame_gray")
    _add_box(z_stage, (0.092, 0.022, 0.060), (0.0, 0.021, 0.030), collision=True)
    _add_box(z_stage, (0.028, 0.014, 0.022), (0.036, 0.009, 0.018), material="dark_hardware")
    _add_box(z_stage, (0.028, 0.014, 0.022), (-0.036, 0.009, 0.018), material="dark_hardware")
    _add_box(z_stage, (0.028, 0.014, 0.022), (0.036, 0.009, 0.052), material="dark_hardware")
    _add_box(z_stage, (0.028, 0.014, 0.022), (-0.036, 0.009, 0.052), material="dark_hardware")
    _add_box(z_stage, (0.050, 0.040, 0.038), (0.0, 0.035, 0.065), material="dark_hardware")
    _add_box(z_stage, (0.050, 0.036, 0.038), (0.0, 0.035, 0.065), collision=True)
    _add_box(
        z_stage,
        (0.110, 0.110, 0.012),
        (0.0, 0.055, 0.094),
        material="payload_orange",
        collision=True,
    )
    _add_box(z_stage, (0.090, 0.020, 0.018), (0.0, 0.095, 0.075), material="dark_hardware")
    z_stage.inertial = Inertial.from_geometry(
        Box((0.11, 0.11, 0.11)),
        mass=1.3,
        origin=_origin((0.0, 0.055, 0.055)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=_origin((0.0, 0.0, 0.046)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.075, upper=0.075, effort=150.0, velocity=0.25),
    )
    model.articulation(
        "x_to_z",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=z_stage,
        origin=_origin((0.0, 0.008, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.12, effort=120.0, velocity=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("x_stage", "base", axes="xy", max_dist=0.02)
    ctx.expect_aabb_overlap("x_stage", "base", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_gap("x_stage", "base", axis="z", max_gap=0.004, max_penetration=0.001)
    ctx.expect_origin_gap("x_stage", "base", axis="z", min_gap=0.0)

    ctx.expect_origin_distance("z_stage", "x_stage", axes="xy", max_dist=0.05)
    ctx.expect_aabb_overlap("z_stage", "x_stage", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_overlap("z_stage", "base", axes="xy", min_overlap=0.07)
    ctx.expect_origin_gap("z_stage", "base", axis="z", min_gap=0.05)

    ctx.expect_joint_motion_axis(
        "base_to_x",
        "x_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "x_to_z",
        "z_stage",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
