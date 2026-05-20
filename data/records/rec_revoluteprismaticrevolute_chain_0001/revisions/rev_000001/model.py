from __future__ import annotations

from math import pi

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
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

BASE_SWIVEL_LIMIT = pi
SLIDE_MAX = 0.18
WRIST_LOWER = -1.15
WRIST_UPPER = 1.10


def _base_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(0.22, 0.18, 0.045)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.0, 0.0, 0.0225))
    )
    pedestal = (
        cq.Workplane("XY")
        .box(0.11, 0.09, 0.08)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, 0.085))
    )
    top_ring = cq.Workplane("XY").circle(0.06).extrude(0.015).translate((0.0, 0.0, 0.115))
    return foot.union(pedestal).union(top_ring)


def _arm_outer_shape() -> cq.Workplane:
    turret = cq.Workplane("XY").circle(0.055).extrude(0.022)
    housing = (
        cq.Workplane("XY")
        .box(0.255, 0.09, 0.07)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.1475, 0.0, 0.035))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.05, 0.076, 0.05)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.228, 0.0, 0.033))
    )
    channel = cq.Workplane("XY").box(0.145, 0.055, 0.036).translate((0.152, 0.0, 0.038))
    return turret.union(housing).cut(channel).union(nose)


def _slide_shape() -> cq.Workplane:
    rail = (
        cq.Workplane("XY")
        .box(0.25, 0.055, 0.045)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.065, 0.0, 0.0))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.10, 0.072, 0.016)
        .edges("|Z")
        .fillet(0.003)
        .translate((-0.005, 0.0, 0.018))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.028, 0.028, 0.028)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.19, 0.0, 0.0))
    )
    ear_left = cq.Workplane("XY").box(0.04, 0.01, 0.06).translate((0.185, 0.026, 0.0))
    ear_right = cq.Workplane("XY").box(0.04, 0.01, 0.06).translate((0.185, -0.026, 0.0))
    return rail.union(top_cap).union(nose).union(ear_left).union(ear_right)


def _wrist_shape() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(0.014).extrude(0.048).translate((0.0, -0.024, 0.0))
    housing = (
        cq.Workplane("XY")
        .box(0.106, 0.055, 0.07)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.061, 0.0, 0.0))
    )
    tool_face = cq.Workplane("XY").box(0.016, 0.07, 0.082).translate((0.122, 0.0, 0.0))
    lower_shoe = (
        cq.Workplane("XY")
        .box(0.038, 0.045, 0.02)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.045, 0.0, -0.03))
    )
    return barrel.union(housing).union(tool_face).union(lower_shoe)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_manipulator", assets=ASSETS)

    model.material("charcoal", rgba=(0.22, 0.23, 0.26, 1.0))
    model.material("silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("accent_blue", rgba=(0.17, 0.35, 0.58, 1.0))
    model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "base.obj", assets=ASSETS), material="charcoal")
    base.visual(
        Box((0.07, 0.004, 0.035)),
        origin=Origin(xyz=(0.0, 0.089, 0.052)),
        material="accent_blue",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.13)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    arm_outer = model.part("arm_outer")
    arm_outer.visual(
        mesh_from_cadquery(_arm_outer_shape(), "arm_outer.obj", assets=ASSETS),
        material="charcoal",
    )
    arm_outer.visual(
        Box((0.085, 0.05, 0.006)),
        origin=Origin(xyz=(0.165, 0.0, 0.067)),
        material="accent_blue",
    )
    arm_outer.inertial = Inertial.from_geometry(
        Box((0.28, 0.10, 0.075)),
        mass=4.5,
        origin=Origin(xyz=(0.12, 0.0, 0.0375)),
    )

    slide = model.part("slide")
    slide.visual(mesh_from_cadquery(_slide_shape(), "slide.obj", assets=ASSETS), material="silver")
    slide.visual(
        Box((0.028, 0.012, 0.03)),
        origin=Origin(xyz=(0.03, 0.033, 0.0)),
        material="rubber",
    )
    slide.inertial = Inertial.from_geometry(
        Box((0.265, 0.08, 0.06)),
        mass=2.2,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(mesh_from_cadquery(_wrist_shape(), "wrist.obj", assets=ASSETS), material="silver")
    wrist.visual(
        Box((0.01, 0.05, 0.05)),
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
        material="rubber",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.14, 0.07, 0.09)),
        mass=1.6,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
    )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-BASE_SWIVEL_LIMIT,
            upper=BASE_SWIVEL_LIMIT,
            effort=50.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "mid_slide",
        ArticulationType.PRISMATIC,
        parent=arm_outer,
        child=slide,
        origin=Origin(xyz=(0.135, 0.0, 0.047)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_MAX,
            effort=40.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=wrist,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=WRIST_LOWER,
            upper=WRIST_UPPER,
            effort=18.0,
            velocity=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("arm_outer", "base", axes="xy", max_dist=0.001)
    ctx.expect_joint_motion_axis(
        "mid_slide",
        "slide",
        world_axis="x",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "wrist_pitch",
        "wrist",
        world_axis="z",
        direction="positive",
        min_delta=0.06,
    )

    for angle in (0.0, pi / 2.0, -pi / 2.0):
        with ctx.pose(base_swivel=angle):
            ctx.expect_aabb_contact("arm_outer", "base")
            ctx.expect_aabb_overlap("arm_outer", "base", axes="xy", min_overlap=0.06)
            ctx.expect_aabb_gap("arm_outer", "base", axis="z", max_gap=0.002, max_penetration=0.0)

    ctx.expect_aabb_contact("arm_outer", "slide")
    ctx.expect_aabb_overlap("arm_outer", "slide", axes="yz", min_overlap=0.04)
    ctx.expect_aabb_contact("slide", "wrist")
    ctx.expect_aabb_overlap("slide", "wrist", axes="yz", min_overlap=0.018)

    with ctx.pose(mid_slide=SLIDE_MAX):
        ctx.expect_aabb_contact("arm_outer", "slide")
        ctx.expect_aabb_overlap("arm_outer", "slide", axes="yz", min_overlap=0.04)
        ctx.expect_origin_gap("wrist", "base", axis="x", min_gap=0.45)

    with ctx.pose(base_swivel=pi / 2.0):
        ctx.expect_joint_motion_axis(
            "mid_slide",
            "slide",
            world_axis="y",
            direction="positive",
            min_delta=0.10,
        )

    with ctx.pose(base_swivel=pi / 2.0, mid_slide=SLIDE_MAX):
        ctx.expect_origin_gap("wrist", "base", axis="y", min_gap=0.45)

    for pitch in (-1.0, 0.0, 1.0):
        with ctx.pose(wrist_pitch=pitch):
            ctx.expect_aabb_contact("slide", "wrist")
            ctx.expect_aabb_overlap("slide", "wrist", axes="yz", min_overlap=0.018)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
