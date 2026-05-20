from __future__ import annotations

from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
PI = pi
Y_AXIS_RPY = (-PI / 2.0, 0.0, 0.0)
X_AXIS_RPY = (0.0, PI / 2.0, 0.0)


def _add_mesh_visual(part, shape, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_shape():
    base_plate = cq.Workplane("XY").box(0.18, 0.12, 0.018)
    pedestal = cq.Workplane("XY").box(0.072, 0.082, 0.10).translate((0.0, 0.0, 0.059))
    rear_brace = cq.Workplane("XY").box(0.03, 0.082, 0.04).translate((-0.02, 0.0, 0.108))
    front_nose = cq.Workplane("XY").box(0.03, 0.05, 0.03).translate((0.018, 0.0, 0.116))
    left_cheek = cq.Workplane("XY").box(0.03, 0.012, 0.072).translate((0.0, 0.028, 0.141))
    right_cheek = cq.Workplane("XY").box(0.03, 0.012, 0.072).translate((0.0, -0.028, 0.141))
    shoulder_sleeve = (
        cq.Workplane("XZ").circle(0.022).extrude(0.014, both=True).translate((0.0, 0.0, 0.153))
    )
    return (
        base_plate.union(pedestal)
        .union(rear_brace)
        .union(front_nose)
        .union(left_cheek)
        .union(right_cheek)
        .union(shoulder_sleeve)
    )


def _build_upper_arm_shape():
    left_plate = cq.Workplane("XY").box(0.14, 0.006, 0.038).translate((0.085, 0.017, 0.0))
    right_plate = cq.Workplane("XY").box(0.14, 0.006, 0.038).translate((0.085, -0.017, 0.0))
    center_web = cq.Workplane("XY").box(0.105, 0.014, 0.014).translate((0.085, 0.0, 0.0))
    spacer_a = cq.Workplane("XZ").center(0.055, 0.0).circle(0.007).extrude(0.019, both=True)
    spacer_b = cq.Workplane("XZ").center(0.115, 0.0).circle(0.007).extrude(0.019, both=True)
    shoulder_hub = cq.Workplane("XZ").circle(0.026).extrude(0.019, both=True)
    elbow_hub = cq.Workplane("XZ").center(0.17, 0.0).circle(0.023).extrude(0.019, both=True)
    return (
        left_plate.union(right_plate)
        .union(center_web)
        .union(spacer_a)
        .union(spacer_b)
        .union(shoulder_hub)
        .union(elbow_hub)
    )


def _build_forearm_shape():
    elbow_hub = cq.Workplane("XZ").circle(0.022).extrude(0.021, both=True)
    left_plate = cq.Workplane("XY").box(0.085, 0.006, 0.032).translate((0.0475, 0.016, 0.0))
    right_plate = cq.Workplane("XY").box(0.085, 0.006, 0.032).translate((0.0475, -0.016, 0.0))
    elbow_web = cq.Workplane("XY").box(0.03, 0.024, 0.014).translate((0.03, 0.0, 0.0))
    front_web = cq.Workplane("XY").box(0.02, 0.024, 0.014).translate((0.085, 0.0, 0.0))
    guide_top = cq.Workplane("XY").box(0.05, 0.03, 0.006).translate((0.12, 0.0, 0.017))
    guide_bottom = cq.Workplane("XY").box(0.05, 0.03, 0.006).translate((0.12, 0.0, -0.017))
    guide_left = cq.Workplane("XY").box(0.05, 0.006, 0.028).translate((0.12, 0.012, 0.0))
    guide_right = cq.Workplane("XY").box(0.05, 0.006, 0.028).translate((0.12, -0.012, 0.0))
    return (
        elbow_hub.union(left_plate)
        .union(right_plate)
        .union(elbow_web)
        .union(front_web)
        .union(guide_top)
        .union(guide_bottom)
        .union(guide_left)
        .union(guide_right)
    )


def _build_slide_shape():
    carriage = cq.Workplane("XY").box(0.03, 0.02, 0.026).translate((0.015, 0.0, 0.0))
    upper_rail = cq.Workplane("YZ").center(0.0, 0.006).circle(0.0035).extrude(0.145)
    lower_rail = cq.Workplane("YZ").center(0.0, -0.006).circle(0.0035).extrude(0.145)
    front_plate = cq.Workplane("XY").box(0.04, 0.038, 0.022).translate((0.165, 0.0, 0.0))
    tool_pad = cq.Workplane("XY").box(0.012, 0.026, 0.036).translate((0.191, 0.0, 0.0))
    return carriage.union(upper_rail).union(lower_rail).union(front_plate).union(tool_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_manipulator", assets=ASSETS)

    model.material("aluminum", rgba=(0.76, 0.78, 0.8, 1.0))
    model.material("bearing_dark", rgba=(0.2, 0.22, 0.26, 1.0))
    model.material("anodized_dark", rgba=(0.28, 0.3, 0.34, 1.0))

    bench_base = model.part("bench_base")
    _add_mesh_visual(bench_base, _build_base_shape(), "bench_base.obj", "aluminum")
    for y in (-0.024, 0.024):
        bench_base.visual(
            Cylinder(radius=0.024, length=0.008),
            origin=Origin(xyz=(0.0, y, 0.153), rpy=Y_AXIS_RPY),
            material="bearing_dark",
        )





    bench_base.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.14)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
    )

    upper_arm = model.part("upper_arm")
    _add_mesh_visual(upper_arm, _build_upper_arm_shape(), "upper_arm.obj", "aluminum")
    for y in (-0.022, 0.022):
        upper_arm.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(xyz=(0.17, y, 0.0), rpy=Y_AXIS_RPY),
            material="bearing_dark",
        )



    upper_arm.inertial = Inertial.from_geometry(
        Box((0.17, 0.04, 0.05)),
        mass=0.9,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    _add_mesh_visual(forearm, _build_forearm_shape(), "forearm.obj", "aluminum")






    forearm.inertial = Inertial.from_geometry(
        Box((0.15, 0.04, 0.04)),
        mass=0.65,
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
    )

    tool_slide = model.part("tool_slide")
    _add_mesh_visual(tool_slide, _build_slide_shape(), "tool_slide.obj", "anodized_dark")



    tool_slide.inertial = Inertial.from_geometry(
        Box((0.20, 0.04, 0.04)),
        mass=0.3,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=bench_base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=18.0, velocity=2.5),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.45, upper=1.25, effort=12.0, velocity=2.8),
    )
    model.articulation(
        "telescoping_slide",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=tool_slide,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.10, effort=8.0, velocity=0.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "bench_base",
        "upper_arm",
        reason="shared shoulder sleeve and hub at the revolute axle",
    )
    ctx.allow_overlap(
        "upper_arm",
        "forearm",
        reason="shared elbow sleeve and hub at the revolute axle",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.002, overlap_volume_tol=0.0)
    ctx.expect_origin_gap("upper_arm", "bench_base", axis="z", min_gap=0.10)
    ctx.expect_origin_gap("forearm", "bench_base", axis="z", min_gap=0.10)
    ctx.expect_origin_gap("tool_slide", "bench_base", axis="z", min_gap=0.10)
    ctx.expect_origin_distance("upper_arm", "bench_base", axes="xy", max_dist=0.11)
    ctx.expect_origin_distance("forearm", "upper_arm", axes="xy", max_dist=0.18)
    ctx.expect_origin_distance("tool_slide", "forearm", axes="xy", max_dist=0.12)
    ctx.expect_aabb_overlap("upper_arm", "bench_base", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("upper_arm", "forearm", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("tool_slide", "forearm", axes="xy", min_overlap=0.01)
    ctx.expect_joint_motion_axis(
        "shoulder_joint",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "elbow_joint",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "telescoping_slide",
        "tool_slide",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
