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
YAW_LIMIT = 2.35
PITCH_LIMIT = 1.05
ROLL_LIMIT = 2.40
PITCH_ORIGIN_X = 0.058
ROLL_ORIGIN_X = 0.040


def _add_visual_mesh(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_base_shape() -> cq.Workplane:
    rear_flange = cq.Workplane("YZ", origin=(-0.044, 0.0, 0.0)).circle(0.034).extrude(0.010)
    stator = cq.Workplane("YZ", origin=(-0.034, 0.0, 0.0)).circle(0.029).extrude(0.030)
    front_shoulder = cq.Workplane("YZ", origin=(-0.014, 0.0, 0.0)).circle(0.032).extrude(0.014)

    base = rear_flange.union(stator).union(front_shoulder)
    base = (
        base.faces("<X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.034, 0.0), (-0.034, 0.0), (0.0, 0.034), (0.0, -0.034)])
        .hole(0.006)
    )
    base = base.faces("<X").workplane(centerOption="CenterOfMass").hole(0.018)
    return base


def _make_yaw_shape() -> cq.Workplane:
    outer_shell = cq.Workplane("YZ", origin=(0.0, 0.0, 0.0)).circle(0.050).extrude(0.052)
    inner_relief = cq.Workplane("YZ", origin=(0.004, 0.0, 0.0)).circle(0.038).extrude(0.048)
    window = cq.Workplane("XY", origin=(0.031, 0.0, 0.0)).box(0.038, 0.036, 0.062)
    front_clearance = cq.Workplane("XY", origin=(0.040, 0.0, 0.0)).box(0.022, 0.070, 0.032)

    left_boss = (
        cq.Workplane("XZ", origin=(PITCH_ORIGIN_X, -0.043, 0.0)).circle(0.013).extrude(0.012)
    )
    right_boss = (
        cq.Workplane("XZ", origin=(PITCH_ORIGIN_X, 0.031, 0.0)).circle(0.013).extrude(0.012)
    )
    cable_gland = cq.Workplane("XY", origin=(0.012, 0.049, 0.010)).box(0.010, 0.008, 0.012)

    return (
        outer_shell.cut(inner_relief)
        .cut(window)
        .cut(front_clearance)
        .union(left_boss)
        .union(right_boss)
        .union(cable_gland)
    )


def _make_pitch_shape() -> cq.Workplane:
    trunnion = cq.Workplane("XZ", origin=(0.0, -0.015, 0.0)).circle(0.0095).extrude(0.030)
    rear_web = cq.Workplane("XY", origin=(0.012, 0.0, 0.0)).box(0.010, 0.022, 0.022)
    inner_block = cq.Workplane("XY", origin=(0.024, 0.0, 0.0)).box(0.018, 0.016, 0.020)
    upper_arm = cq.Workplane("XY", origin=(0.034, 0.0, 0.018)).box(0.036, 0.022, 0.008)
    lower_arm = cq.Workplane("XY", origin=(0.034, 0.0, -0.018)).box(0.036, 0.022, 0.008)
    nose_pad = cq.Workplane("XY", origin=(0.050, 0.0, 0.0)).box(0.010, 0.024, 0.018)
    center_relief = cq.Workplane("XY", origin=(0.038, 0.0, 0.0)).box(0.020, 0.014, 0.018)

    yoke = (
        trunnion.union(rear_web)
        .union(inner_block)
        .union(upper_arm)
        .union(lower_arm)
        .union(nose_pad)
    )
    return yoke.cut(center_relief)


def _make_roll_shape() -> cq.Workplane:
    rear_barrel = cq.Workplane("YZ", origin=(-0.013, 0.0, 0.0)).circle(0.010).extrude(0.026)
    main_body = cq.Workplane("YZ", origin=(0.018, 0.0, 0.0)).circle(0.014).extrude(0.048)
    nose = cq.Workplane("YZ", origin=(0.064, 0.0, 0.0)).circle(0.012).extrude(0.012)
    front_flange = cq.Workplane("YZ", origin=(0.074, 0.0, 0.0)).circle(0.014).extrude(0.006)
    sensor_pod = cq.Workplane("XY", origin=(0.044, 0.032, 0.0)).box(0.022, 0.020, 0.012)
    service_flat = cq.Workplane("XY", origin=(0.042, 0.0, -0.006)).box(0.032, 0.018, 0.010)

    roll = (
        rear_barrel.union(main_body)
        .union(nose)
        .union(front_flange)
        .union(sensor_pod)
        .union(service_flat)
    )
    roll = (
        roll.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, 0.016), (0.016, 0.0), (-0.016, 0.0), (0.0, -0.016)])
        .hole(0.004)
    )
    roll = roll.faces(">X").workplane(centerOption="CenterOfMass").hole(0.012)
    return roll


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_wrist_head", assets=ASSETS)

    model.material("dark_anodized", rgba=(0.17, 0.19, 0.23, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("brushed_steel", rgba=(0.60, 0.62, 0.66, 1.0))

    base_mount = model.part("base_mount")
    _add_visual_mesh(base_mount, _make_base_shape(), "base_mount.obj", "dark_anodized")


    base_mount.inertial = Inertial.from_geometry(
        Box((0.040, 0.072, 0.072)),
        mass=1.15,
        origin=Origin(xyz=(-0.024, 0.0, 0.0)),
    )

    yaw_housing = model.part("yaw_housing")
    _add_visual_mesh(yaw_housing, _make_yaw_shape(), "yaw_housing.obj", "dark_anodized")








    yaw_housing.inertial = Inertial.from_geometry(
        Box((0.056, 0.108, 0.108)),
        mass=0.80,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Cylinder(radius=0.0095, length=0.030),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_aluminum",
    )
    pitch_yoke.visual(
        Box((0.010, 0.022, 0.022)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material="machined_aluminum",
    )
    pitch_yoke.visual(
        Box((0.018, 0.016, 0.020)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material="machined_aluminum",
    )
    pitch_yoke.visual(
        Box((0.036, 0.022, 0.008)),
        origin=Origin(xyz=(0.034, 0.0, 0.018)),
        material="machined_aluminum",
    )
    pitch_yoke.visual(
        Box((0.036, 0.022, 0.008)),
        origin=Origin(xyz=(0.034, 0.0, -0.018)),
        material="machined_aluminum",
    )
    pitch_yoke.visual(
        Box((0.010, 0.024, 0.018)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material="machined_aluminum",
    )





    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.058, 0.072, 0.062)),
        mass=0.55,
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
    )

    roll_head = model.part("roll_head")
    _add_visual_mesh(roll_head, _make_roll_shape(), "roll_head.obj", "brushed_steel")



    roll_head.inertial = Inertial.from_geometry(
        Box((0.086, 0.056, 0.056)),
        mass=0.45,
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=yaw_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-YAW_LIMIT, upper=YAW_LIMIT, effort=18.0, velocity=3.0),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_housing,
        child=pitch_yoke,
        origin=Origin(xyz=(PITCH_ORIGIN_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-PITCH_LIMIT, upper=PITCH_LIMIT, effort=12.0, velocity=2.5
        ),
    )
    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_head,
        origin=Origin(xyz=(ROLL_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-ROLL_LIMIT, upper=ROLL_LIMIT, effort=10.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "base_mount",
        "pitch_yoke",
        reason="Conservative AABB false-positive at full yaw where the slim pitch bracket swings past the compact base flange.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.001, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("pitch_yoke", "yaw_housing", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("roll_head", "pitch_yoke", axes="xy", min_overlap=0.010)
    ctx.expect_origin_distance("pitch_yoke", "yaw_housing", axes="xy", max_dist=0.060)
    ctx.expect_origin_distance("roll_head", "pitch_yoke", axes="xy", max_dist=0.050)

    ctx.expect_joint_motion_axis(
        "yaw_axis", "yaw_housing", world_axis="x", direction="negative", min_delta=0.003
    )
    ctx.expect_joint_motion_axis(
        "pitch_axis", "pitch_yoke", world_axis="z", direction="negative", min_delta=0.008
    )
    ctx.expect_joint_motion_axis(
        "roll_axis", "roll_head", world_axis="z", direction="positive", min_delta=0.005
    )

    with ctx.pose({"yaw_axis": YAW_LIMIT}):
        ctx.expect_aabb_overlap("pitch_yoke", "yaw_housing", axes="xy", min_overlap=0.010)
        ctx.expect_origin_distance("pitch_yoke", "yaw_housing", axes="xy", max_dist=0.060)

    with ctx.pose({"yaw_axis": -YAW_LIMIT}):
        ctx.expect_aabb_overlap("pitch_yoke", "yaw_housing", axes="xy", min_overlap=0.010)
        ctx.expect_origin_distance("pitch_yoke", "yaw_housing", axes="xy", max_dist=0.060)

    with ctx.pose({"pitch_axis": PITCH_LIMIT}):
        ctx.expect_aabb_overlap("roll_head", "pitch_yoke", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance("roll_head", "pitch_yoke", axes="xy", max_dist=0.070)

    with ctx.pose({"pitch_axis": -PITCH_LIMIT}):
        ctx.expect_aabb_overlap("roll_head", "pitch_yoke", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance("roll_head", "pitch_yoke", axes="xy", max_dist=0.070)

    with ctx.pose({"roll_axis": pi / 2.0}):
        ctx.expect_aabb_overlap("roll_head", "pitch_yoke", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance("roll_head", "pitch_yoke", axes="xy", max_dist=0.060)

    with ctx.pose({"roll_axis": -pi / 2.0}):
        ctx.expect_aabb_overlap("roll_head", "pitch_yoke", axes="xy", min_overlap=0.008)
        ctx.expect_origin_distance("roll_head", "pitch_yoke", axes="xy", max_dist=0.060)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
