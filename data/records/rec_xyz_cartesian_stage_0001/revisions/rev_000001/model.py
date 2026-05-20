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
def _box_shape(
    size: tuple[float, float, float], center: tuple[float, float, float]
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_visual() -> cq.Workplane:
    shape = _box_shape((0.66, 0.44, 0.012), (0.0, 0.0, 0.024))
    for center, size in (
        ((-0.38, 0.0, 0.03), (0.04, 0.62, 0.06)),
        ((0.38, 0.0, 0.03), (0.04, 0.62, 0.06)),
        ((0.0, -0.29, 0.03), (0.76, 0.04, 0.06)),
        ((0.0, 0.29, 0.03), (0.76, 0.04, 0.06)),
        ((-0.25, 0.0, 0.012), (0.10, 0.08, 0.024)),
        ((0.25, 0.0, 0.012), (0.10, 0.08, 0.024)),
    ):
        shape = shape.union(_box_shape(size, center))
    return shape


def _build_gantry_visual() -> cq.Workplane:
    shape = _box_shape((0.09, 0.12, 0.028), (0.0, 0.0, 0.014))
    for center, size in (
        ((0.61, 0.0, 0.014), (0.09, 0.12, 0.028)),
        ((0.0, 0.0, 0.223), (0.06, 0.10, 0.39)),
        ((0.61, 0.0, 0.223), (0.06, 0.10, 0.39)),
        ((0.305, 0.0, 0.322), (0.70, 0.06, 0.06)),
    ):
        shape = shape.union(_box_shape(size, center))
    return shape


def _build_x_carriage_visual() -> cq.Workplane:
    shape = _box_shape((0.13, 0.11, 0.10), (0.0, 0.0, 0.0))
    for center, size in (
        ((0.0, 0.02, 0.055), (0.15, 0.06, 0.03)),
        ((0.0, 0.0, -0.08), (0.18, 0.08, 0.02)),
    ):
        shape = shape.union(_box_shape(size, center))
    return shape


def _build_z_slide_visual() -> cq.Workplane:
    shape = _box_shape((0.16, 0.10, 0.04), (0.0, 0.0, 0.0))
    for center, size in (
        ((0.0, 0.0, -0.07), (0.07, 0.06, 0.10)),
        ((0.0, 0.0, -0.13), (0.10, 0.08, 0.02)),
    ):
        shape = shape.union(_box_shape(size, center))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_axis_gantry", assets=ASSETS)

    model.material("frame_gray", rgba=(0.66, 0.68, 0.72, 1.0))
    model.material("rail_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("gantry_blue", rgba=(0.26, 0.42, 0.72, 1.0))
    model.material("carriage_orange", rgba=(0.90, 0.50, 0.12, 1.0))
    model.material("tool_black", rgba=(0.14, 0.14, 0.16, 1.0))

    base = model.part("base")
    _mesh_visual(base, _build_base_visual(), "gantry_base.obj", "frame_gray")
    base.visual(
        Box((0.028, 0.54, 0.024)), origin=Origin(xyz=(-0.305, 0.0, 0.088)), material="rail_dark"
    )
    base.visual(
        Box((0.028, 0.54, 0.024)), origin=Origin(xyz=(0.305, 0.0, 0.088)), material="rail_dark"
    )
    for center, size in (
        ((-0.38, 0.0, 0.03), (0.04, 0.62, 0.06)),
        ((0.38, 0.0, 0.03), (0.04, 0.62, 0.06)),
        ((0.0, -0.29, 0.03), (0.76, 0.04, 0.06)),
        ((0.0, 0.29, 0.03), (0.76, 0.04, 0.06)),
        ((0.0, 0.0, 0.024), (0.66, 0.44, 0.012)),
        ((-0.305, 0.0, 0.088), (0.028, 0.54, 0.024)),
        ((0.305, 0.0, 0.088), (0.028, 0.54, 0.024)),
    ):
        pass
    base.inertial = Inertial.from_geometry(
        Box((0.84, 0.62, 0.12)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    gantry = model.part("gantry")
    _mesh_visual(gantry, _build_gantry_visual(), "gantry_bridge.obj", "gantry_blue")
    gantry.visual(
        Box((0.58, 0.022, 0.04)),
        origin=Origin(xyz=(0.305, -0.035, 0.305)),
        material="rail_dark",
    )
    for center, size in (
        ((0.0, 0.0, 0.014), (0.09, 0.12, 0.028)),
        ((0.61, 0.0, 0.014), (0.09, 0.12, 0.028)),
        ((0.0, 0.0, 0.223), (0.06, 0.10, 0.39)),
        ((0.61, 0.0, 0.223), (0.06, 0.10, 0.39)),
        ((0.305, 0.0, 0.322), (0.70, 0.06, 0.06)),
        ((0.305, -0.035, 0.305), (0.58, 0.022, 0.04)),
    ):
        pass
    gantry.inertial = Inertial.from_geometry(
        Box((0.72, 0.14, 0.42)),
        mass=6.5,
        origin=Origin(xyz=(0.305, 0.0, 0.21)),
    )

    x_carriage = model.part("x_carriage")
    _mesh_visual(x_carriage, _build_x_carriage_visual(), "x_carriage.obj", "carriage_orange")
    for center, size in (
        ((0.0, 0.0, 0.0), (0.13, 0.11, 0.10)),
        ((0.0, 0.02, 0.055), (0.15, 0.06, 0.03)),
        ((0.0, 0.0, -0.08), (0.18, 0.08, 0.02)),
    ):
        pass
    x_carriage.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.14)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
    )

    z_slide = model.part("z_slide")
    _mesh_visual(z_slide, _build_z_slide_visual(), "z_slide.obj", "tool_black")
    for center, size in (
        ((0.0, 0.0, 0.0), (0.16, 0.10, 0.04)),
        ((0.0, 0.0, -0.07), (0.07, 0.06, 0.10)),
        ((0.0, 0.0, -0.13), (0.10, 0.08, 0.02)),
    ):
        pass
    z_slide.inertial = Inertial.from_geometry(
        Box((0.16, 0.10, 0.16)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
    )

    model.articulation(
        "base_to_gantry",
        ArticulationType.PRISMATIC,
        parent=base,
        child=gantry,
        origin=Origin(xyz=(-0.305, 0.0, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.16, upper=0.16, effort=120.0, velocity=0.60),
    )
    model.articulation(
        "gantry_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=gantry,
        child=x_carriage,
        origin=Origin(xyz=(0.305, -0.035, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.18, upper=0.18, effort=90.0, velocity=0.80),
    )
    model.articulation(
        "x_carriage_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.06, effort=70.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_gantry",
        "gantry",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "gantry_to_x_carriage",
        "x_carriage",
        world_axis="x",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "x_carriage_to_z_slide",
        "z_slide",
        world_axis="z",
        direction="negative",
        min_delta=0.01,
    )

    ctx.expect_origin_distance("x_carriage", "z_slide", axes="xy", max_dist=0.01)
    ctx.expect_origin_distance("z_slide", "base", axes="xy", max_dist=0.06)

    ctx.expect_aabb_overlap("gantry", "base", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("x_carriage", "gantry", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("z_slide", "x_carriage", axes="xy", min_overlap=0.02)

    ctx.expect_aabb_gap("gantry", "base", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_origin_gap("gantry", "base", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("x_carriage", "base", axis="z", min_gap=0.22)
    ctx.expect_origin_gap("z_slide", "base", axis="z", min_gap=0.10)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
