from __future__ import annotations

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
def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_base_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(0.110, 0.090, 0.044)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, 0.0, 0.022))
    )
    cover_recess = cq.Workplane("XY").box(0.070, 0.050, 0.006).translate((0.0, 0.0, 0.041))
    top_ring = cq.Workplane("XY").circle(0.030).extrude(0.012).translate((0.0, 0.0, 0.044))
    return housing.cut(cover_recess).union(top_ring)


def _make_shoulder_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(0.032).extrude(0.008)
    housing = (
        cq.Workplane("XY")
        .box(0.060, 0.070, 0.046)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.055, 0.0, 0.031))
    )
    rear_cap = cq.Workplane("YZ").circle(0.026).extrude(0.012).translate((0.019, 0.0, 0.031))
    output_boss = cq.Workplane("YZ").circle(0.020).extrude(0.018).translate((0.067, 0.0, 0.031))
    top_pocket = cq.Workplane("XY").box(0.034, 0.042, 0.006).translate((0.055, 0.0, 0.049))
    return flange.union(housing).union(rear_cap).union(output_boss).cut(top_pocket)


def _make_elbow_link_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ").slot2D(0.120, 0.028, 0).extrude(0.018).translate((0.075, -0.009, 0.0))
    )
    lightening_window = (
        cq.Workplane("XZ").slot2D(0.066, 0.012, 0).extrude(0.026).translate((0.075, -0.013, 0.0))
    )
    offset_bridge = cq.Workplane("XY").box(0.048, 0.022, 0.018).translate((0.122, 0.011, 0.0))
    return outer.cut(lightening_window).union(offset_bridge)


def _make_elbow_actuator_shape() -> cq.Workplane:
    rotor = cq.Workplane("XZ").circle(0.021).extrude(0.028).translate((0.0, -0.014, 0.0))
    housing = (
        cq.Workplane("XY")
        .box(0.052, 0.058, 0.042)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.150, 0.024, 0.010))
    )
    rear_cap = cq.Workplane("YZ").circle(0.022).extrude(0.012).translate((0.124, 0.024, 0.010))
    output_boss = cq.Workplane("YZ").circle(0.018).extrude(0.014).translate((0.162, 0.024, 0.010))
    top_pocket = cq.Workplane("XY").box(0.028, 0.036, 0.006).translate((0.150, 0.024, 0.027))
    return rotor.union(housing).union(rear_cap).union(output_boss).cut(top_pocket)


def _make_wrist_housing_shape() -> cq.Workplane:
    rotor = cq.Workplane("YZ").circle(0.018).extrude(0.024).translate((-0.012, 0.0, 0.0))
    housing = (
        cq.Workplane("XY")
        .box(0.042, 0.052, 0.040)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.024, 0.0, 0.0))
    )
    top_pocket = cq.Workplane("XY").box(0.020, 0.030, 0.006).translate((0.024, 0.0, 0.017))
    return rotor.union(housing).cut(top_pocket)


def _make_wrist_fork_shape() -> cq.Workplane:
    center_block = cq.Workplane("XY").box(0.030, 0.028, 0.014).translate((0.050, 0.024, 0.0))
    upper_arm = cq.Workplane("XY").box(0.060, 0.010, 0.010).translate((0.072, 0.037, 0.012))
    lower_arm = cq.Workplane("XY").box(0.060, 0.010, 0.010).translate((0.072, 0.011, -0.012))
    tool_pad = cq.Workplane("XY").box(0.018, 0.028, 0.034).translate((0.105, 0.024, 0.0))
    return center_block.union(upper_arm).union(lower_arm).union(tool_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="staggered_rotary_assembly", assets=ASSETS)
    model.material("housing_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("machined_aluminum", rgba=(0.77, 0.79, 0.82, 1.0))

    base = model.part("base_housing")
    _add_mesh_visual(base, _make_base_shape(), "base_housing.obj", "housing_dark")


    base.inertial = Inertial.from_geometry(
        Box((0.110, 0.090, 0.056)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    shoulder = model.part("shoulder_module")
    _add_mesh_visual(shoulder, _make_shoulder_shape(), "shoulder_module.obj", "housing_dark")


    shoulder.inertial = Inertial.from_geometry(
        Box((0.088, 0.070, 0.054)),
        mass=1.1,
        origin=Origin(xyz=(0.044, 0.0, 0.027)),
    )

    elbow = model.part("elbow_module")
    _add_mesh_visual(elbow, _make_elbow_link_shape(), "elbow_link.obj", "machined_aluminum")
    _add_mesh_visual(elbow, _make_elbow_actuator_shape(), "elbow_actuator.obj", "housing_dark")



    elbow.inertial = Inertial.from_geometry(
        Box((0.178, 0.058, 0.044)),
        mass=0.85,
        origin=Origin(xyz=(0.089, 0.012, 0.005)),
    )

    wrist = model.part("wrist_module")
    _add_mesh_visual(wrist, _make_wrist_housing_shape(), "wrist_housing.obj", "housing_dark")
    _add_mesh_visual(wrist, _make_wrist_fork_shape(), "wrist_fork.obj", "machined_aluminum")


    wrist.inertial = Inertial.from_geometry(
        Box((0.110, 0.052, 0.040)),
        mass=0.45,
        origin=Origin(xyz=(0.055, 0.013, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.6, upper=1.6, effort=12.0, velocity=2.5),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(0.085, 0.0, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.0, upper=0.45, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=elbow,
        child=wrist,
        origin=Origin(xyz=(0.176, 0.024, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.8, upper=1.8, effort=4.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "base_yaw",
        "shoulder_module",
        world_axis="y",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "shoulder_pitch",
        "elbow_module",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "wrist_roll",
        "wrist_module",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )

    ctx.expect_origin_distance("shoulder_module", "base_housing", axes="xy", max_dist=0.02)
    ctx.expect_aabb_overlap("shoulder_module", "base_housing", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_gap("shoulder_module", "base_housing", axis="z", max_gap=0.01, max_penetration=0.0)

    ctx.expect_origin_distance("elbow_module", "base_housing", axes="xy", max_dist=0.18)
    ctx.expect_aabb_gap("elbow_module", "base_housing", axis="z", max_gap=0.04, max_penetration=0.0)
    ctx.expect_origin_distance("wrist_module", "base_housing", axes="xy", max_dist=0.30)
    ctx.expect_aabb_gap("wrist_module", "base_housing", axis="z", max_gap=0.06, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
