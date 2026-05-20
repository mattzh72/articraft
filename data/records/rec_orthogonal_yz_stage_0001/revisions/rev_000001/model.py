from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _base_shape() -> cq.Workplane:
    shape = _box((0.18, 0.18, 0.012), (0.0, 0.0, 0.006))
    shape = shape.union(_box((0.018, 0.150, 0.010), (-0.046, 0.0, 0.017)))
    shape = shape.union(_box((0.018, 0.150, 0.010), (0.046, 0.0, 0.017)))
    shape = shape.union(_box((0.012, 0.150, 0.006), (-0.046, 0.0, 0.025)))
    shape = shape.union(_box((0.012, 0.150, 0.006), (0.046, 0.0, 0.025)))
    shape = shape.union(_box((0.018, 0.140, 0.010), (0.0, 0.0, 0.017)))
    shape = shape.union(_box((0.120, 0.016, 0.014), (0.0, -0.068, 0.019)))
    shape = shape.union(_box((0.120, 0.016, 0.014), (0.0, 0.068, 0.019)))
    shape = shape.cut(_box((0.060, 0.110, 0.004), (0.0, 0.0, 0.010)))
    shape = shape.cut(_box((0.022, 0.050, 0.004), (-0.060, 0.0, 0.010)))
    shape = shape.cut(_box((0.022, 0.050, 0.004), (0.060, 0.0, 0.010)))
    return shape


def _y_carriage_shape() -> cq.Workplane:
    shape = _box((0.028, 0.060, 0.014), (0.0, 0.0, 0.007))
    shape = shape.union(_box((0.072, 0.060, 0.012), (0.004, 0.0, 0.030)))
    shape = shape.union(_box((0.012, 0.060, 0.110), (0.026, 0.0, 0.075)))
    shape = shape.union(_box((0.004, 0.010, 0.092), (0.034, -0.018, 0.073)))
    shape = shape.union(_box((0.004, 0.010, 0.092), (0.034, 0.018, 0.073)))
    shape = shape.cut(_box((0.044, 0.034, 0.010), (0.0, 0.0, 0.023)))
    shape = shape.cut(_box((0.006, 0.032, 0.052), (0.026, 0.0, 0.082)))
    return shape


def _z_slide_shape() -> cq.Workplane:
    shape = _box((0.018, 0.048, 0.060), (0.009, 0.0, 0.0))
    shape = shape.union(_box((0.006, 0.010, 0.050), (0.003, -0.017, 0.0)))
    shape = shape.union(_box((0.006, 0.010, 0.050), (0.003, 0.017, 0.0)))
    shape = shape.cut(_box((0.010, 0.026, 0.030), (0.011, 0.0, 0.0)))
    shape = shape.cut(_box((0.006, 0.044, 0.014), (0.013, 0.0, -0.023)))
    return shape


def _tool_platform_shape() -> cq.Workplane:
    shape = _box((0.008, 0.024, 0.032), (0.004, 0.0, 0.0))
    shape = shape.union(_box((0.032, 0.050, 0.010), (0.022, 0.0, 0.0)))
    shape = shape.union(_box((0.008, 0.016, 0.018), (0.036, 0.0, 0.014)))
    shape = shape.cut(_box((0.020, 0.008, 0.012), (0.026, -0.014, 0.0)))
    shape = shape.cut(_box((0.020, 0.008, 0.012), (0.026, 0.014, 0.0)))
    return shape


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material: str) -> None:
    mesh = mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0006,
        angular_tolerance=0.08,
    )
    part.visual(mesh, material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_yz_positioner", assets=ASSETS)

    model.material("base_steel", rgba=(0.45, 0.47, 0.50, 1.0))
    model.material("machined_aluminum", rgba=(0.79, 0.81, 0.84, 1.0))
    model.material("anodized_dark", rgba=(0.20, 0.21, 0.24, 1.0))

    base = model.part("base")
    _add_visual_mesh(base, _base_shape(), "base.obj", "base_steel")




    base.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.028)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    y_carriage = model.part("y_carriage")
    _add_visual_mesh(y_carriage, _y_carriage_shape(), "y_carriage.obj", "machined_aluminum")


    y_carriage.inertial = Inertial.from_geometry(
        Box((0.072, 0.060, 0.120)),
        mass=1.15,
        origin=Origin(xyz=(0.010, 0.0, 0.060)),
    )

    z_slide = model.part("z_slide")
    _add_visual_mesh(z_slide, _z_slide_shape(), "z_slide.obj", "machined_aluminum")

    z_slide.inertial = Inertial.from_geometry(
        Box((0.018, 0.048, 0.060)),
        mass=0.55,
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
    )

    tool_platform = model.part("tool_platform")
    _add_visual_mesh(tool_platform, _tool_platform_shape(), "tool_platform.obj", "anodized_dark")


    tool_platform.inertial = Inertial.from_geometry(
        Box((0.040, 0.050, 0.032)),
        mass=0.22,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.045,
            upper=0.045,
            effort=80.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "y_carriage_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.032, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.050,
            effort=60.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "z_slide_to_tool_platform",
        ArticulationType.FIXED,
        parent=z_slide,
        child=tool_platform,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.0015, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("y_carriage", "base", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("y_carriage", "base", axis="z", max_gap=0.0015, max_penetration=1e-6)
    ctx.expect_origin_distance("y_carriage", "base", axes="xy", max_dist=0.001)
    ctx.expect_origin_distance("z_slide", "y_carriage", axes="xy", max_dist=0.035)
    ctx.expect_origin_distance("tool_platform", "z_slide", axes="xy", max_dist=0.020)
    ctx.expect_aabb_overlap("tool_platform", "base", axes="xy", min_overlap=0.020)
    ctx.expect_origin_gap("z_slide", "base", axis="z", min_gap=0.045)
    ctx.expect_origin_gap("tool_platform", "base", axis="z", min_gap=0.060)
    ctx.expect_joint_motion_axis(
        "base_to_y_carriage",
        "y_carriage",
        world_axis="y",
        direction="positive",
        min_delta=0.010,
    )
    ctx.expect_joint_motion_axis(
        "y_carriage_to_z_slide",
        "z_slide",
        world_axis="z",
        direction="positive",
        min_delta=0.010,
    )

    with ctx.pose(base_to_y_carriage=-0.045):
        ctx.expect_aabb_overlap("y_carriage", "base", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_gap("y_carriage", "base", axis="z", max_gap=0.0015, max_penetration=1e-6)
        ctx.expect_aabb_overlap("tool_platform", "base", axes="xy", min_overlap=0.020)

    with ctx.pose(base_to_y_carriage=0.045):
        ctx.expect_aabb_overlap("y_carriage", "base", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_gap("y_carriage", "base", axis="z", max_gap=0.0015, max_penetration=1e-6)
        ctx.expect_aabb_overlap("tool_platform", "base", axes="xy", min_overlap=0.020)

    with ctx.pose(y_carriage_to_z_slide=0.050):
        ctx.expect_origin_gap("z_slide", "base", axis="z", min_gap=0.090)
        ctx.expect_origin_gap("tool_platform", "base", axis="z", min_gap=0.110)
        ctx.expect_origin_distance("tool_platform", "z_slide", axes="xy", max_dist=0.020)

    with ctx.pose(base_to_y_carriage=0.045, y_carriage_to_z_slide=0.050):
        ctx.expect_aabb_overlap("tool_platform", "base", axes="xy", min_overlap=0.020)
        ctx.expect_origin_gap("tool_platform", "base", axis="z", min_gap=0.110)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
