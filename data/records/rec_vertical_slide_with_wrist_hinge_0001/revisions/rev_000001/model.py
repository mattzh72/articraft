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
RAIL_X = 0.040
RAIL_Y = 0.018
CARRIAGE_ZERO_Z = 0.240


def _add_mesh_visual(part, shape, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_slide_wrist", assets=ASSETS)

    model.material("frame_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("rail_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    model.material("carriage_light", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("tool_dark", rgba=(0.16, 0.16, 0.18, 1.0))
    model.material("accent", rgba=(0.84, 0.47, 0.15, 1.0))

    base = model.part("base")
    base_shape = (
        cq.Workplane("XY")
        .box(0.160, 0.130, 0.020)
        .translate((0.0, 0.0, 0.010))
        .union(cq.Workplane("XY").box(0.070, 0.032, 0.520).translate((0.0, -0.004, 0.280)))
        .union(cq.Workplane("XY").box(0.014, 0.018, 0.440).translate((RAIL_X, RAIL_Y, 0.280)))
        .union(cq.Workplane("XY").box(0.014, 0.018, 0.440).translate((-RAIL_X, RAIL_Y, 0.280)))
        .union(cq.Workplane("XY").box(0.110, 0.026, 0.030).translate((0.0, 0.006, 0.505)))
        .union(cq.Workplane("XY").box(0.100, 0.016, 0.080).translate((0.0, -0.018, 0.090)))
    )
    _add_mesh_visual(base, base_shape, "vertical_slide_base.obj", "frame_gray")





    base.inertial = Inertial.from_geometry(
        Box((0.160, 0.130, 0.540)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
    )

    carriage = model.part("carriage")
    carriage_shape = (
        cq.Workplane("XY")
        .box(0.125, 0.020, 0.100)
        .translate((0.0, 0.035, 0.000))
        .union(cq.Workplane("XY").box(0.060, 0.012, 0.090).translate((0.0, 0.010, 0.000)))
        .union(cq.Workplane("XY").box(0.024, 0.026, 0.050).translate((RAIL_X, 0.000, -0.030)))
        .union(cq.Workplane("XY").box(0.024, 0.026, 0.050).translate((RAIL_X, 0.000, 0.030)))
        .union(cq.Workplane("XY").box(0.024, 0.026, 0.050).translate((-RAIL_X, 0.000, -0.030)))
        .union(cq.Workplane("XY").box(0.024, 0.026, 0.050).translate((-RAIL_X, 0.000, 0.030)))
        .union(cq.Workplane("XY").box(0.046, 0.020, 0.085).translate((0.0, 0.035, 0.090)))
        .union(cq.Workplane("XY").box(0.070, 0.018, 0.026).translate((0.0, 0.035, 0.145)))
        .union(cq.Workplane("XY").box(0.014, 0.030, 0.034).translate((0.028, 0.035, 0.145)))
        .union(cq.Workplane("XY").box(0.014, 0.030, 0.034).translate((-0.028, 0.035, 0.145)))
        .cut(cq.Workplane("XY").box(0.040, 0.032, 0.022).translate((0.0, 0.035, 0.145)))
    )
    _add_mesh_visual(carriage, carriage_shape, "vertical_slide_carriage.obj", "carriage_light")
    carriage.inertial = Inertial.from_geometry(
        Box((0.130, 0.060, 0.190)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.025, 0.075)),
    )

    wrist = model.part("wrist")
    wrist_shape = (
        cq.Workplane("XY")
        .box(0.034, 0.018, 0.028)
        .translate((0.0, 0.000, 0.000))
        .union(cq.Workplane("XY").box(0.022, 0.050, 0.092).translate((0.0, 0.055, -0.050)))
        .union(cq.Workplane("XY").box(0.060, 0.020, 0.020).translate((0.0, 0.095, -0.018)))
        .union(cq.Workplane("XY").box(0.050, 0.030, 0.040).translate((0.0, 0.090, -0.060)))
        .union(cq.Workplane("XY").circle(0.018).extrude(0.085).translate((0.0, 0.095, -0.175)))
        .union(cq.Workplane("XY").circle(0.010).extrude(0.028).translate((0.0, 0.095, -0.203)))
    )
    _add_mesh_visual(wrist, wrist_shape, "vertical_slide_wrist.obj", "tool_dark")
    wrist.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.180)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.085, -0.095)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, RAIL_Y, CARRIAGE_ZERO_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.000,
            upper=0.180,
            effort=220.0,
            velocity=0.400,
        ),
    )
    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, 0.035, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.000,
            upper=1.100,
            effort=20.0,
            velocity=2.500,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.expect_joint_motion_axis(
        "base_to_carriage",
        "carriage",
        world_axis="z",
        direction="positive",
        min_delta=0.040,
    )
    ctx.expect_joint_motion_axis(
        "carriage_to_wrist",
        "wrist",
        world_axis="x",
        direction="negative",
        min_delta=0.120,
    )
    ctx.expect_origin_gap("carriage", "base", axis="z", min_gap=0.180)
    ctx.expect_origin_gap("wrist", "base", axis="z", min_gap=0.300)
    ctx.expect_origin_distance("carriage", "base", axes="xy", max_dist=0.030)
    ctx.expect_origin_distance("wrist", "carriage", axes="xy", max_dist=0.060)
    ctx.expect_aabb_overlap("carriage", "base", axes="xy", min_overlap=0.020)
    ctx.expect_origin_distance("wrist", "base", axes="xy", max_dist=0.120)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
