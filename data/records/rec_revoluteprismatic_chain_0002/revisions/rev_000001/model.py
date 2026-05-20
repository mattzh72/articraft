from __future__ import annotations

from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
# In sdk_hybrid, author visual meshes with cadquery + mesh_from_cadquery.
from math import pi

HALF_PI = pi / 2.0


def _build_visual_meshes() -> dict[str, object]:
    import cadquery as cq

    base_plate = cq.Workplane("XY").box(0.22, 0.14, 0.028, centered=(True, True, False))
    rear_support = (
        cq.Workplane("XY")
        .box(0.04, 0.05, 0.05, centered=(True, True, False))
        .translate((-0.078, 0.0, 0.028))
    )
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.026, 0.032, 0.018, centered=(True, True, False))
        .translate((-0.067, 0.0, 0.028))
    )
    left_ear = (
        cq.Workplane("XY")
        .box(0.014, 0.008, 0.054, centered=(True, True, False))
        .translate((-0.056, 0.019, 0.028))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(0.014, 0.008, 0.054, centered=(True, True, False))
        .translate((-0.056, -0.019, 0.028))
    )
    base_shape = base_plate.union(rear_support).union(lower_bridge).union(left_ear).union(right_ear)

    outer_sleeve = cq.Workplane("XY").box(0.185, 0.036, 0.034, centered=(False, True, True))
    sleeve_pocket = (
        cq.Workplane("XY")
        .box(0.155, 0.024, 0.022, centered=(False, True, True))
        .translate((0.018, 0.0, 0.0))
    )
    hinge_knuckle = cq.Workplane("XZ").circle(0.017).extrude(0.016, both=True)
    hinge_cheek = (
        cq.Workplane("XY")
        .box(0.024, 0.024, 0.024, centered=(True, True, True))
        .translate((0.01, 0.0, 0.0))
    )
    outer_arm_shape = outer_sleeve.cut(sleeve_pocket).union(hinge_knuckle).union(hinge_cheek)

    slider_shape = (
        cq.Workplane("XY")
        .box(0.19, 0.024, 0.02, centered=(False, True, True))
        .union(
            cq.Workplane("XY")
            .box(0.022, 0.028, 0.022, centered=(False, True, True))
            .translate((0.168, 0.0, 0.0))
        )
    )

    head_body = (
        cq.Workplane("YZ").circle(0.034).extrude(0.022, both=True).translate((0.235, 0.0, 0.01))
    )
    head_neck = (
        cq.Workplane("XY")
        .box(0.04, 0.03, 0.034, centered=(False, True, True))
        .translate((0.185, 0.0, 0.01))
    )
    handle_bar = (
        cq.Workplane("XY")
        .box(0.05, 0.01, 0.012, centered=(True, True, True))
        .translate((0.235, 0.0, 0.052))
    )
    handle_post_left = (
        cq.Workplane("XY")
        .box(0.01, 0.01, 0.022, centered=(True, True, False))
        .translate((0.215, 0.0, 0.03))
    )
    handle_post_right = (
        cq.Workplane("XY")
        .box(0.01, 0.01, 0.022, centered=(True, True, False))
        .translate((0.255, 0.0, 0.03))
    )
    lamp_head_shape = (
        head_body.union(head_neck)
        .union(handle_bar)
        .union(handle_post_left)
        .union(handle_post_right)
    )

    return {
        "base": mesh_from_cadquery(base_shape, MESH_DIR / "work_light_base.obj"),
        "outer_arm": mesh_from_cadquery(outer_arm_shape, MESH_DIR / "work_light_outer_arm.obj"),
        "slider": mesh_from_cadquery(slider_shape, MESH_DIR / "work_light_slider.obj"),
        "head": mesh_from_cadquery(lamp_head_shape, MESH_DIR / "work_light_head.obj"),
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_work_light", assets=ASSETS)

    model.material("dark_body", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("arm_yellow", rgba=(0.92, 0.77, 0.16, 1.0))
    model.material("lamp_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("lens", rgba=(0.85, 0.92, 0.98, 0.4))

    meshes = _build_visual_meshes()

    base = model.part("base")
    base.visual(meshes["base"], material="dark_body")
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.14, 0.08)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(meshes["outer_arm"], material="arm_yellow")
    outer_arm.inertial = Inertial.from_geometry(
        Box((0.2, 0.04, 0.04)),
        mass=0.65,
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
    )

    inner_slide = model.part("inner_slide")
    inner_slide.visual(meshes["slider"], material="arm_yellow")
    inner_slide.visual(meshes["head"], material="lamp_gray")
    inner_slide.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.259, 0.0, 0.01), rpy=(0.0, HALF_PI, 0.0)),
        material="lens",
    )



    inner_slide.inertial = Inertial.from_geometry(
        Box((0.28, 0.07, 0.08)),
        mass=0.75,
        origin=Origin(xyz=(0.17, 0.0, 0.015)),
    )

    model.articulation(
        "base_tilt",
        ArticulationType.REVOLUTE,
        parent="base",
        child="outer_arm",
        origin=Origin(xyz=(-0.056, 0.0, 0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.25,
            effort=12.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "arm_extension",
        ArticulationType.PRISMATIC,
        parent="outer_arm",
        child="inner_slide",
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.12,
            effort=20.0,
            velocity=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "outer_arm",
        "inner_slide",
        reason="The telescoping rail intentionally nests inside the outer sleeve.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.expect_joint_motion_axis(
        "base_tilt",
        "outer_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "arm_extension",
        "inner_slide",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(base_tilt=0.0, arm_extension=0.0):
        ctx.expect_aabb_gap("outer_arm", "base", axis="z", max_gap=0.005, max_penetration=0.04)
        ctx.expect_aabb_gap("inner_slide", "base", axis="z", max_gap=0.005, max_penetration=0.05)
        ctx.expect_aabb_overlap("outer_arm", "base", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_overlap("inner_slide", "base", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_overlap("inner_slide", "outer_arm", axes="xy", min_overlap=0.02)

    with ctx.pose(base_tilt=0.0, arm_extension=0.12):
        ctx.expect_aabb_gap("inner_slide", "base", axis="z", max_gap=0.005, max_penetration=0.05)
        ctx.expect_aabb_overlap("inner_slide", "outer_arm", axes="xy", min_overlap=0.015)
        ctx.expect_origin_distance("inner_slide", "outer_arm", axes="xy", max_dist=0.25)

    with ctx.pose(base_tilt=1.25, arm_extension=0.12):
        ctx.expect_origin_gap("outer_arm", "base", axis="z", min_gap=0.04)
        ctx.expect_origin_gap("inner_slide", "base", axis="z", min_gap=0.08)
        ctx.expect_aabb_gap("inner_slide", "base", axis="z", max_gap=0.4, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
