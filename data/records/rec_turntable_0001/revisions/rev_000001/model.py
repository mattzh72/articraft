from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
SATIN_BLACK = Material("satin_black", (0.08, 0.08, 0.09, 1.0))
BRUSHED_ALUMINUM = Material("brushed_aluminum", (0.73, 0.73, 0.75, 1.0))
DARK_RUBBER = Material("dark_rubber", (0.12, 0.12, 0.12, 1.0))
GRAPHITE = Material("graphite", (0.18, 0.18, 0.20, 1.0))
STEEL = Material("steel", (0.64, 0.65, 0.67, 1.0))
CARTRIDGE_RED = Material("cartridge_red", (0.54, 0.07, 0.05, 1.0))


def _plinth_mesh():
    profile = rounded_rect_profile(0.46, 0.35, radius=0.018, corner_segments=12)
    geometry = ExtrudeGeometry.from_z0(profile, 0.045)
    return mesh_from_geometry(geometry, ASSETS.mesh_path("turntable_plinth.obj"))


def _platter_mesh():
    profile = [
        (0.0, 0.000),
        (0.018, 0.000),
        (0.024, 0.001),
        (0.132, 0.001),
        (0.145, 0.004),
        (0.145, 0.016),
        (0.139, 0.018),
        (0.038, 0.018),
        (0.022, 0.020),
        (0.0, 0.020),
    ]
    geometry = LatheGeometry(profile, segments=64)
    return mesh_from_geometry(geometry, ASSETS.mesh_path("turntable_platter.obj"))


def _tonearm_tube_mesh():
    geometry = tube_from_spline_points(
        [
            (0.002, 0.000, 0.035),
            (-0.010, 0.026, 0.037),
            (-0.031, 0.082, 0.039),
            (-0.060, 0.158, 0.040),
            (-0.082, 0.222, 0.041),
        ],
        radius=0.0038,
        samples_per_segment=18,
        radial_segments=14,
        cap_ends=True,
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path("turntable_tonearm_tube.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hifi_turntable", assets=ASSETS)

    plinth = model.part("plinth")
    plinth.visual(_plinth_mesh(), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=SATIN_BLACK)
    plinth.visual(
        Box((0.446, 0.336, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0565)),
        material=BRUSHED_ALUMINUM,
    )
    plinth.visual(
        Box((0.060, 0.040, 0.002)),
        origin=Origin(xyz=(0.150, -0.090, 0.059)),
        material=BRUSHED_ALUMINUM,
    )
    plinth.visual(
        Cylinder(radius=0.042, length=0.004),
        origin=Origin(xyz=(-0.045, 0.0, 0.057)),
        material=GRAPHITE,
    )
    for x in (-0.185, 0.185):
        for y in (-0.130, 0.130):
            plinth.visual(
                Cylinder(radius=0.020, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=DARK_RUBBER,
            )
    plinth.visual(
        Box((0.030, 0.010, 0.002)),
        origin=Origin(xyz=(-0.180, -0.120, 0.059)),
        material=STEEL,
    )
    plinth.visual(
        Cylinder(radius=0.011, length=0.002),
        origin=Origin(xyz=(0.185, -0.120, 0.059)),
        material=STEEL,
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.46, 0.35, 0.058)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    platter = model.part("platter")
    platter.visual(_platter_mesh(), origin=Origin(), material=BRUSHED_ALUMINUM)
    platter.visual(
        Cylinder(radius=0.136, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=DARK_RUBBER,
    )
    platter.visual(
        Cylinder(radius=0.030, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=GRAPHITE,
    )
    platter.visual(
        Cylinder(radius=0.0028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=STEEL,
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.020),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.015, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=GRAPHITE,
    )
    tonearm.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=BRUSHED_ALUMINUM,
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=BRUSHED_ALUMINUM,
    )
    tonearm.visual(_tonearm_tube_mesh(), origin=Origin(), material=STEEL)
    tonearm.visual(
        Cylinder(radius=0.0032, length=0.030),
        origin=Origin(xyz=(0.015, 0.000, 0.039), rpy=(0.0, pi / 2.0, 0.0)),
        material=STEEL,
    )
    tonearm.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(xyz=(0.040, 0.000, 0.039), rpy=(0.0, pi / 2.0, 0.0)),
        material=GRAPHITE,
    )
    tonearm.visual(
        Box((0.030, 0.018, 0.005)),
        origin=Origin(xyz=(-0.086, 0.228, 0.0405), rpy=(0.0, 0.0, -0.32)),
        material=STEEL,
    )
    tonearm.visual(
        Box((0.014, 0.011, 0.007)),
        origin=Origin(xyz=(-0.092, 0.235, 0.0365), rpy=(0.0, 0.0, -0.32)),
        material=CARTRIDGE_RED,
    )
    tonearm.visual(
        Cylinder(radius=0.0010, length=0.0035),
        origin=Origin(xyz=(-0.098, 0.240, 0.0338)),
        material=STEEL,
    )
    tonearm.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.014, 0.012, 0.002)),
        material=GRAPHITE,
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.29, 0.08, 0.05)),
        mass=0.35,
        origin=Origin(xyz=(-0.015, 0.100, 0.025)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent="plinth",
        child="platter",
        origin=Origin(xyz=(-0.045, 0.0, 0.064)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent="plinth",
        child="tonearm",
        origin=Origin(xyz=(0.155, -0.090, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=1.8, lower=0.0, upper=0.92),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("platter", "plinth", axes="xy", min_overlap=0.28)
    ctx.expect_origin_distance("platter", "plinth", axes="xy", max_dist=0.06)
    ctx.expect_aabb_gap("platter", "plinth", axis="z", max_gap=0.003, max_penetration=0.0)
    ctx.expect_aabb_overlap("tonearm", "plinth", axes="xy", min_overlap=0.04)
    ctx.expect_joint_motion_axis(
        "tonearm_pivot",
        "tonearm",
        world_axis="x",
        direction="negative",
        min_delta=0.03,
    )

    with ctx.pose(tonearm_pivot=0.0):
        ctx.expect_aabb_overlap("tonearm", "plinth", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("tonearm", "platter", axes="xy", max_dist=0.22)

    with ctx.pose(tonearm_pivot=0.88):
        ctx.expect_aabb_overlap("tonearm", "platter", axes="xy", min_overlap=0.08)
        ctx.expect_origin_distance("tonearm", "platter", axes="xy", max_dist=0.24)

    with ctx.pose(platter_spin=pi / 2.0):
        ctx.expect_aabb_overlap("platter", "plinth", axes="xy", min_overlap=0.28)
        ctx.expect_aabb_gap("platter", "plinth", axis="z", max_gap=0.003, max_penetration=0.0)

    with ctx.pose(platter_spin=pi, tonearm_pivot=0.45):
        ctx.expect_aabb_overlap("tonearm", "plinth", axes="xy", min_overlap=0.04)
        ctx.expect_aabb_overlap("platter", "plinth", axes="xy", min_overlap=0.28)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
