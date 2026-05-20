from __future__ import annotations

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
from math import pi

import cadquery as cq

from sdk import Cylinder


def _origin_x(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _origin_y(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def _make_body_housing() -> cq.Workplane:
    core = (
        cq.Workplane("XY")
        .box(0.074, 0.046, 0.050)
        .edges("|Z")
        .fillet(0.0035)
        .translate((0.0, -0.007, 0.0))
    )
    front_shoulder = (
        cq.Workplane("XY")
        .box(0.090, 0.012, 0.058)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.010, 0.0))
    )
    rear_cap = (
        cq.Workplane("XY")
        .box(0.054, 0.020, 0.036)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, -0.036, 0.0))
    )
    mount_tongue = cq.Workplane("XY").box(0.042, 0.008, 0.050).translate((0.0, -0.050, 0.0))
    upper_support = cq.Workplane("XY").box(0.088, 0.006, 0.010).translate((0.0, 0.013, 0.015))
    lower_support = cq.Workplane("XY").box(0.088, 0.006, 0.010).translate((0.0, 0.013, -0.015))

    housing = (
        core.union(front_shoulder)
        .union(rear_cap)
        .union(mount_tongue)
        .union(upper_support)
        .union(lower_support)
    )

    center_relief = cq.Workplane("XY").box(0.026, 0.018, 0.030).translate((0.0, 0.008, 0.0))
    left_window = cq.Workplane("XY").box(0.016, 0.016, 0.018).translate((-0.024, 0.001, 0.0))
    right_window = cq.Workplane("XY").box(0.016, 0.016, 0.018).translate((0.024, 0.001, 0.0))

    return housing.cut(center_relief).cut(left_window).cut(right_window)


def _make_jaw(side_sign: float) -> cq.Workplane:
    inward = -side_sign

    carriage = cq.Workplane("XY").box(0.032, 0.018, 0.040, centered=(True, False, True))
    carriage = carriage.edges("|Z").fillet(0.0015)

    support_arm = (
        cq.Workplane("XY")
        .box(0.016, 0.020, 0.016, centered=(True, False, True))
        .translate((inward * 0.012, 0.018, 0.0))
    )
    finger = (
        cq.Workplane("XY")
        .box(0.010, 0.016, 0.036, centered=(True, False, True))
        .translate((inward * 0.020, 0.028, 0.0))
    )
    toe = (
        cq.Workplane("XY")
        .box(0.012, 0.008, 0.022, centered=(True, False, True))
        .translate((inward * 0.016, 0.040, 0.0))
    )

    jaw = carriage.union(support_arm).union(finger).union(toe)

    guide_grooves = (
        cq.Workplane("YZ")
        .pushPoints([(0.0, 0.015), (0.0, -0.015)])
        .circle(0.0046)
        .extrude(0.040, both=True)
    )
    weight_relief = (
        cq.Workplane("XY")
        .box(0.010, 0.014, 0.020, centered=(True, False, True))
        .translate((inward * 0.006, 0.004, 0.0))
    )

    return jaw.cut(guide_grooves).cut(weight_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="opposed_twin_slide_gripper", assets=ASSETS)

    model.material("body_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("body_mid", rgba=(0.30, 0.33, 0.37, 1.0))
    model.material("jaw_alloy", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("guide_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("fastener", rgba=(0.56, 0.58, 0.62, 1.0))
    model.material("grip_pad", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_housing(), "gripper_body.obj", assets=ASSETS),
        origin=Origin(),
        material="body_dark",
    )
    for bolt_x in (-0.024, 0.024):
        for bolt_z in (-0.016, 0.016):
            body.visual(
                Cylinder(radius=0.0032, length=0.004),
                origin=_origin_y((bolt_x, 0.0175, bolt_z)),
                material="fastener",
            )
    body.inertial = Inertial.from_geometry(
        Box((0.092, 0.062, 0.060)),
        mass=1.35,
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
    )

    guide_rails = model.part("guide_rails")
    guide_rails.visual(
        Box((0.034, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material="body_mid",
    )
    for carrier_x in (-0.037, 0.037):
        guide_rails.visual(
            Box((0.014, 0.008, 0.038)),
            origin=Origin(xyz=(carrier_x, -0.006, 0.0)),
            material="body_mid",
        )
    for rail_z in (-0.015, 0.015):
        guide_rails.visual(
            Cylinder(radius=0.004, length=0.124),
            origin=_origin_x((0.0, 0.0, rail_z)),
            material="guide_steel",
        )
    guide_rails.inertial = Inertial.from_geometry(
        Box((0.124, 0.020, 0.042)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.001, 0.0)),
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        mesh_from_cadquery(_make_jaw(-1.0), "left_jaw.obj", assets=ASSETS),
        origin=Origin(),
        material="jaw_alloy",
    )
    left_jaw.visual(
        Box((0.006, 0.012, 0.028)),
        origin=Origin(xyz=(0.023, 0.028, 0.0)),
        material="grip_pad",
    )
    left_jaw.inertial = Inertial.from_geometry(
        Box((0.042, 0.050, 0.048)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        mesh_from_cadquery(_make_jaw(1.0), "right_jaw.obj", assets=ASSETS),
        origin=Origin(),
        material="jaw_alloy",
    )
    right_jaw.visual(
        Box((0.006, 0.012, 0.028)),
        origin=Origin(xyz=(-0.023, 0.028, 0.0)),
        material="grip_pad",
    )
    right_jaw.inertial = Inertial.from_geometry(
        Box((0.042, 0.050, 0.048)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
    )

    model.articulation(
        "body_to_guide_rails",
        ArticulationType.FIXED,
        parent=body,
        child=guide_rails,
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
    )
    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=guide_rails,
        child=left_jaw,
        origin=Origin(xyz=(-0.037, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.018, effort=120.0, velocity=0.20),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=guide_rails,
        child=right_jaw,
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.018, effort=120.0, velocity=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "guide_rails",
        "left_jaw",
        reason="generated convex hulls can conservatively bridge the open rear guide grooves",
    )
    ctx.allow_overlap(
        "guide_rails",
        "right_jaw",
        reason="generated convex hulls can conservatively bridge the open rear guide grooves",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("guide_rails", "body", axes="xy", max_dist=0.035)
    ctx.expect_joint_motion_axis(
        "left_slide", "left_jaw", world_axis="x", direction="negative", min_delta=0.01
    )
    ctx.expect_joint_motion_axis(
        "right_slide", "right_jaw", world_axis="x", direction="positive", min_delta=0.01
    )
    ctx.expect_aabb_overlap("left_jaw", "guide_rails", axes="xy", min_overlap=0.003)
    ctx.expect_aabb_overlap("right_jaw", "guide_rails", axes="xy", min_overlap=0.003)
    ctx.expect_origin_distance("left_jaw", "guide_rails", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("right_jaw", "guide_rails", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("left_jaw", "right_jaw", axes="xy", max_dist=0.08)

    with ctx.pose(left_slide=0.018, right_slide=0.018):
        ctx.expect_aabb_overlap("left_jaw", "guide_rails", axes="xy", min_overlap=0.003)
        ctx.expect_aabb_overlap("right_jaw", "guide_rails", axes="xy", min_overlap=0.003)
        ctx.expect_origin_distance("left_jaw", "guide_rails", axes="xy", max_dist=0.07)
        ctx.expect_origin_distance("right_jaw", "guide_rails", axes="xy", max_dist=0.07)
        ctx.expect_origin_distance("left_jaw", "right_jaw", axes="xy", max_dist=0.12)

    with ctx.pose(left_slide=0.018, right_slide=0.0):
        ctx.expect_aabb_overlap("left_jaw", "guide_rails", axes="xy", min_overlap=0.003)
        ctx.expect_aabb_overlap("right_jaw", "guide_rails", axes="xy", min_overlap=0.003)
        ctx.expect_origin_distance("left_jaw", "guide_rails", axes="xy", max_dist=0.07)
        ctx.expect_origin_distance("right_jaw", "guide_rails", axes="xy", max_dist=0.05)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
