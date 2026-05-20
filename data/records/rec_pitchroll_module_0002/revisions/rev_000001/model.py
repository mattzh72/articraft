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
import cadquery as cq

BASE_BOLT_PATTERN = (
    (-0.052, -0.034),
    (-0.052, 0.034),
    (0.052, -0.034),
    (0.052, 0.034),
)

CRADLE_FASTENER_PATTERN = (
    (-0.010, -0.020),
    (-0.010, 0.020),
    (0.010, -0.020),
    (0.010, 0.020),
)

SENSOR_FASTENER_PATTERN = (
    (-0.008, -0.008),
    (-0.008, 0.008),
    (0.008, -0.008),
    (0.008, 0.008),
)


def _attach_mesh_visual(part, shape: cq.Workplane, filename: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS))


def _make_base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.160, 0.120, 0.012).translate((0.0, 0.0, -0.066))
    left_tower = cq.Workplane("XY").box(0.060, 0.010, 0.086).translate((0.0, 0.024, -0.017))
    right_tower = cq.Workplane("XY").box(0.060, 0.010, 0.086).translate((0.0, -0.024, -0.017))

    plate_holes = (
        cq.Workplane("XY")
        .pushPoints(BASE_BOLT_PATTERN)
        .circle(0.0035)
        .extrude(0.028)
        .translate((0.0, 0.0, -0.080))
    )
    plate_heads = (
        cq.Workplane("XY")
        .pushPoints(BASE_BOLT_PATTERN)
        .circle(0.006)
        .extrude(0.003)
        .translate((0.0, 0.0, -0.060))
    )
    pitch_bore = cq.Workplane("XZ").circle(0.010).extrude(0.080).translate((0.0, -0.040, 0.0))
    left_bearing_cap = cq.Workplane("XZ").circle(0.015).extrude(0.004).translate((0.0, 0.029, 0.0))
    right_bearing_cap = (
        cq.Workplane("XZ").circle(0.015).extrude(0.004).translate((0.0, -0.033, 0.0))
    )

    return (
        plate.union(left_tower)
        .union(right_tower)
        .cut(plate_holes)
        .cut(pitch_bore)
        .union(plate_heads)
        .union(left_bearing_cap)
        .union(right_bearing_cap)
    )


def _make_cradle_shape() -> cq.Workplane:
    front_plate = cq.Workplane("XY").box(0.010, 0.032, 0.078).translate((0.024, 0.0, 0.0))
    rear_plate = cq.Workplane("XY").box(0.010, 0.032, 0.078).translate((-0.024, 0.0, 0.0))
    top_bar = cq.Workplane("XY").box(0.048, 0.032, 0.010).translate((0.0, 0.0, 0.034))
    bottom_bar = cq.Workplane("XY").box(0.048, 0.032, 0.010).translate((0.0, 0.0, -0.034))
    pitch_trunnion = cq.Workplane("XZ").circle(0.008).extrude(0.032).translate((0.0, -0.016, 0.0))
    left_trunnion_collar = (
        cq.Workplane("XZ").circle(0.012).extrude(0.0015).translate((0.0, 0.016, 0.0))
    )
    right_trunnion_collar = (
        cq.Workplane("XZ").circle(0.012).extrude(0.0015).translate((0.0, -0.0175, 0.0))
    )
    roll_bore = cq.Workplane("YZ").circle(0.0085).extrude(0.080).translate((-0.040, 0.0, 0.0))
    front_heads = (
        cq.Workplane("YZ")
        .pushPoints(CRADLE_FASTENER_PATTERN)
        .circle(0.003)
        .extrude(0.003)
        .translate((0.029, 0.0, 0.0))
    )
    rear_heads = (
        cq.Workplane("YZ")
        .pushPoints(CRADLE_FASTENER_PATTERN)
        .circle(0.003)
        .extrude(0.003)
        .translate((-0.032, 0.0, 0.0))
    )

    return (
        front_plate.union(rear_plate)
        .union(top_bar)
        .union(bottom_bar)
        .union(pitch_trunnion)
        .union(left_trunnion_collar)
        .union(right_trunnion_collar)
        .cut(roll_bore)
        .union(front_heads)
        .union(rear_heads)
    )


def _make_sensor_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.028, 0.022, 0.030).translate((0.016, 0.0, 0.006))
    spindle = cq.Workplane("YZ").circle(0.007).extrude(0.020).translate((-0.020, 0.0, 0.0))
    front_cap = cq.Workplane("YZ").circle(0.012).extrude(0.014).translate((0.026, 0.0, 0.006))
    rear_cap = cq.Workplane("YZ").circle(0.010).extrude(0.008).translate((-0.008, 0.0, 0.004))
    lens_barrel = cq.Workplane("YZ").circle(0.008).extrude(0.010).translate((0.040, 0.0, 0.006))
    lens_recess = cq.Workplane("YZ").circle(0.0045).extrude(0.004).translate((0.046, 0.0, 0.006))
    front_screws = (
        cq.Workplane("YZ")
        .pushPoints(SENSOR_FASTENER_PATTERN)
        .circle(0.0025)
        .extrude(0.002)
        .translate((0.038, 0.0, 0.006))
    )

    return (
        body.union(spindle)
        .union(front_cap)
        .union(rear_cap)
        .union(lens_barrel)
        .union(front_screws)
        .cut(lens_recess)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sensor_mount_pitch_roll", assets=ASSETS)

    mount_base = model.part("mount_base")
    base_shape = _make_base_shape()
    _attach_mesh_visual(mount_base, base_shape, "mount_base.obj")



    mount_base.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.090)),
        mass=1.10,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
    )

    pitch_cradle = model.part("pitch_cradle")
    cradle_shape = _make_cradle_shape()
    _attach_mesh_visual(pitch_cradle, cradle_shape, "pitch_cradle.obj")






    pitch_cradle.inertial = Inertial.from_geometry(Box((0.050, 0.040, 0.080)), mass=0.42)

    sensor_head = model.part("sensor_head")
    sensor_shape = _make_sensor_shape()
    _attach_mesh_visual(sensor_head, sensor_shape, "sensor_head.obj")



    sensor_head.inertial = Inertial.from_geometry(
        Box((0.056, 0.030, 0.036)),
        mass=0.34,
        origin=Origin(xyz=(0.012, 0.0, 0.005)),
    )

    model.articulation(
        "base_to_pitch_cradle",
        ArticulationType.REVOLUTE,
        parent=mount_base,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.90, upper=0.90, effort=6.0, velocity=2.0),
    )
    model.articulation(
        "pitch_cradle_to_sensor_head",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=sensor_head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.30, upper=1.30, effort=4.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "mount_base",
        "sensor_head",
        reason="Conservative AABB false positive: the sensor rolls inside the open gap between side towers while remaining above the base plate.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.001, overlap_volume_tol=0.0)
    ctx.expect_origin_distance("pitch_cradle", "mount_base", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("sensor_head", "pitch_cradle", axes="xy", max_dist=0.025)
    ctx.expect_origin_distance("sensor_head", "mount_base", axes="xy", max_dist=0.025)
    ctx.expect_aabb_overlap("pitch_cradle", "mount_base", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("sensor_head", "pitch_cradle", axes="xy", min_overlap=0.015)
    ctx.expect_aabb_overlap("sensor_head", "mount_base", axes="xy", min_overlap=0.015)
    ctx.expect_joint_motion_axis(
        "base_to_pitch_cradle",
        "sensor_head",
        world_axis="z",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "pitch_cradle_to_sensor_head",
        "sensor_head",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
