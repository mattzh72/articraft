from __future__ import annotations

import math

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        return Material(name=name, color=rgba)


def _build_head_shell_mesh():
    profile = [
        (0.0, -0.090),
        (0.042, -0.090),
        (0.050, -0.070),
        (0.056, -0.015),
        (0.054, 0.035),
        (0.048, 0.082),
        (0.0, 0.082),
    ]
    geom = LatheGeometry(profile, segments=56)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path("floor_lamp_head_shell.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_lamp", assets=ASSETS)

    satin_black = _material("satin_black", (0.12, 0.12, 0.13, 1.0))
    brushed_brass = _material("brushed_brass", (0.68, 0.58, 0.34, 1.0))
    dark_steel = _material("dark_steel", (0.21, 0.22, 0.24, 1.0))
    soft_rubber = _material("soft_rubber", (0.07, 0.07, 0.08, 1.0))
    diffuser = _material("diffuser", (0.96, 0.91, 0.80, 0.45))

    head_shell_mesh = _build_head_shell_mesh()

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.166, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=soft_rubber,
        name="floor_ring",
    )
    base.visual(
        Cylinder(radius=0.162, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_steel,
        name="weighted_disc",
    )
    base.visual(
        Cylinder(radius=0.136, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=satin_black,
        name="top_cover",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=brushed_brass,
        name="stem_socket",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.162, length=0.024),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    upright = model.part("upright")
    upright.visual(
        Cylinder(radius=0.017, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=brushed_brass,
        name="lower_insert",
    )
    upright.visual(
        Cylinder(radius=0.013, length=1.330),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        material=satin_black,
        name="main_stem",
    )
    upright.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 1.335)),
        material=brushed_brass,
        name="upper_sleeve",
    )
    upright.visual(
        Cylinder(radius=0.011, length=0.055),
        origin=Origin(xyz=(0.028, 0.0, 1.390), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="head_neck",
    )
    upright.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(xyz=(0.056, 0.0, 1.390), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_brass,
        name="pivot_pin",
    )
    upright.visual(
        Box((0.018, 0.014, 0.050)),
        origin=Origin(xyz=(0.048, 0.028, 1.390)),
        material=dark_steel,
        name="left_cheek",
    )
    upright.visual(
        Box((0.018, 0.014, 0.050)),
        origin=Origin(xyz=(0.048, -0.028, 1.390)),
        material=dark_steel,
        name="right_cheek",
    )
    upright.inertial = Inertial.from_geometry(
        Cylinder(radius=0.017, length=1.390),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.695)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_brass,
        name="pivot_hub",
    )
    head.visual(
        Box((0.086, 0.014, 0.038)),
        origin=Origin(xyz=(0.039, 0.031, 0.0)),
        material=dark_steel,
        name="left_yoke",
    )
    head.visual(
        Box((0.086, 0.014, 0.038)),
        origin=Origin(xyz=(0.039, -0.031, 0.0)),
        material=dark_steel,
        name="right_yoke",
    )
    head.visual(
        Cylinder(radius=0.033, length=0.044),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_brass,
        name="knuckle_collar",
    )
    head.visual(
        head_shell_mesh,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=satin_black,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.015),
        origin=Origin(xyz=(0.196, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_brass,
        name="bezel_ring",
    )
    head.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.204, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=diffuser,
        name="lens",
    )
    head.visual(
        Box((0.060, 0.024, 0.006)),
        origin=Origin(xyz=(0.138, 0.0, 0.053)),
        material=soft_rubber,
        name="top_switch",
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.215),
        mass=1.2,
        origin=Origin(xyz=(0.107, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_mount",
        ArticulationType.FIXED,
        parent="base",
        child="upright",
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent="upright",
        child="head",
        origin=Origin(xyz=(0.056, 0.0, 1.390)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.90,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("upright", "base", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_gap("upright", "base", axis="z", max_gap=0.001, max_penetration=0.035)

    ctx.expect_aabb_overlap("head", "upright", axes="xy", min_overlap=0.02)
    ctx.expect_origin_distance("head", "upright", axes="xy", max_dist=0.16)
    ctx.expect_aabb_overlap("head", "base", axes="xy", min_overlap=0.09)
    ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.20)
    ctx.expect_joint_motion_axis(
        "head_tilt",
        "head",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose(head_tilt=-0.85):
        ctx.expect_aabb_overlap("head", "upright", axes="xy", min_overlap=0.015)
        ctx.expect_aabb_overlap("head", "base", axes="xy", min_overlap=0.09)
        ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.22)

    with ctx.pose(head_tilt=0.50):
        ctx.expect_aabb_overlap("head", "upright", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_overlap("head", "base", axes="xy", min_overlap=0.10)
        ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.18)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
