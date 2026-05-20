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
def _rounded_box(
    length: float,
    width: float,
    height: float,
    center: tuple[float, float, float],
    radius: float,
):
    solid = cq.Workplane("XY").box(length, width, height)
    if radius > 0.0:
        solid = solid.edges("|X").fillet(radius)
    return solid.translate(center)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .translate(center)
    )


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def _add_visual_mesh(part, shape, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape.val(), filename, assets=ASSETS), material=material)


def _make_support_shape():
    support = _rounded_box(0.024, 0.19, 0.32, (0.012, 0.0, 0.18), 0.006)
    support = support.union(_rounded_box(0.07, 0.14, 0.12, (0.052, 0.0, 0.18), 0.008))
    support = support.union(_rounded_box(0.05, 0.11, 0.05, (0.04, 0.0, 0.245), 0.006))
    support = support.union(_rounded_box(0.05, 0.11, 0.05, (0.04, 0.0, 0.115), 0.006))
    support = support.union(_z_cylinder(0.055, 0.12, (0.075, 0.0, 0.18)))

    for y_offset in (-0.06, 0.06):
        for z_offset in (0.085, 0.275):
            support = support.cut(_x_cylinder(0.012, 0.04, (0.012, y_offset, z_offset)))

    return support


def _make_upper_arm_shape():
    arm = _z_cylinder(0.05, 0.105, (0.024, 0.0, 0.0))
    arm = arm.union(_rounded_box(0.07, 0.10, 0.09, (0.04, 0.0, 0.0), 0.01))
    arm = arm.union(_rounded_box(0.21, 0.08, 0.065, (0.15, 0.0, 0.0), 0.012))
    arm = arm.union(_rounded_box(0.15, 0.06, 0.03, (0.145, 0.0, -0.026), 0.006))
    arm = arm.union(_z_cylinder(0.045, 0.10, (0.29, 0.0, 0.0)))
    arm = arm.cut(cq.Workplane("XY").box(0.08, 0.10, 0.022).translate((0.16, 0.0, 0.0)))
    return arm


def _make_forearm_shape():
    arm = _z_cylinder(0.043, 0.095, (0.02, 0.0, 0.0))
    arm = arm.union(_rounded_box(0.06, 0.09, 0.08, (0.032, 0.0, 0.0), 0.008))
    arm = arm.union(_rounded_box(0.175, 0.07, 0.058, (0.125, 0.0, 0.0), 0.01))
    arm = arm.union(_rounded_box(0.12, 0.05, 0.026, (0.118, 0.0, -0.022), 0.005))
    arm = arm.union(_z_cylinder(0.038, 0.085, (0.24, 0.0, 0.0)))
    arm = arm.cut(cq.Workplane("XY").box(0.07, 0.09, 0.018).translate((0.13, 0.0, 0.0)))
    return arm


def _make_wrist_shape():
    wrist = _z_cylinder(0.036, 0.08, (0.018, 0.0, 0.0))
    wrist = wrist.union(_rounded_box(0.055, 0.085, 0.07, (0.028, 0.0, 0.0), 0.007))
    wrist = wrist.union(_rounded_box(0.11, 0.065, 0.05, (0.09, 0.0, 0.0), 0.008))
    wrist = wrist.union(_rounded_box(0.03, 0.10, 0.08, (0.155, 0.0, 0.0), 0.006))
    wrist = wrist.union(_rounded_box(0.035, 0.045, 0.11, (0.175, 0.0, 0.0), 0.004))
    return wrist


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilevered_industrial_arm", assets=ASSETS)

    model.material("support_gray", rgba=(0.24, 0.26, 0.30, 1.0))
    model.material("arm_orange", rgba=(0.89, 0.46, 0.17, 1.0))
    model.material("tool_dark", rgba=(0.13, 0.13, 0.15, 1.0))

    support = model.part("support")
    _add_visual_mesh(support, _make_support_shape(), "support.obj", "support_gray")




    support.inertial = Inertial.from_geometry(
        Box((0.11, 0.19, 0.32)),
        mass=16.0,
        origin=Origin(xyz=(0.05, 0.0, 0.18)),
    )

    upper_arm = model.part("upper_arm")
    _add_visual_mesh(upper_arm, _make_upper_arm_shape(), "upper_arm.obj", "arm_orange")



    upper_arm.inertial = Inertial.from_geometry(
        Box((0.33, 0.12, 0.10)),
        mass=8.0,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    _add_visual_mesh(forearm, _make_forearm_shape(), "forearm.obj", "arm_orange")



    forearm.inertial = Inertial.from_geometry(
        Box((0.27, 0.10, 0.09)),
        mass=5.0,
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    _add_visual_mesh(wrist, _make_wrist_shape(), "wrist.obj", "tool_dark")



    wrist.inertial = Inertial.from_geometry(
        Box((0.20, 0.10, 0.11)),
        mass=2.5,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_swing",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper_arm,
        origin=Origin(xyz=(0.075, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=1.35,
            effort=420.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "elbow_swing",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.29, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.1,
            upper=1.35,
            effort=280.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "wrist_swing",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.0,
            upper=1.15,
            effort=120.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "support",
        "upper_arm",
        reason="shoulder bearing envelopes overlap conservatively in AABB around the coaxial pivot",
    )
    ctx.allow_overlap(
        "upper_arm",
        "forearm",
        reason="elbow hub envelopes overlap conservatively around the revolute pin",
    )
    ctx.allow_overlap(
        "forearm",
        "wrist",
        reason="wrist hub envelopes overlap conservatively around the revolute pin",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)
    ctx.expect_aabb_overlap("support", "upper_arm", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("upper_arm", "forearm", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("forearm", "wrist", axes="xy", min_overlap=0.01)
    ctx.expect_origin_distance("support", "upper_arm", axes="xy", max_dist=0.12)
    ctx.expect_origin_distance("upper_arm", "forearm", axes="xy", max_dist=0.35)
    ctx.expect_origin_distance("forearm", "wrist", axes="xy", max_dist=0.28)
    ctx.expect_joint_motion_axis(
        "shoulder_swing",
        "upper_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "elbow_swing",
        "forearm",
        world_axis="y",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "wrist_swing",
        "wrist",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
