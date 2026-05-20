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
HALF_PI = 1.5707963267948966

BASE_SIZE = (0.090, 0.070, 0.008)
PEDESTAL_RADIUS = 0.018
PEDESTAL_HEIGHT = 0.010

PAN_COLLAR_RADIUS = 0.028
PAN_COLLAR_HEIGHT = 0.010
PAN_DECK_SIZE = (0.062, 0.054, 0.008)
YOKE_ARM_SIZE = (0.024, 0.008, 0.050)
YOKE_ARM_Y = 0.022
TILT_AXIS_Z = 0.054

SENSOR_BODY_SIZE = (0.058, 0.032, 0.036)
SENSOR_MASS_PROXY_SIZE = (0.072, 0.032, 0.036)
SENSOR_X_OFFSET = -0.020
PAN_LIMIT = 2.80
TILT_LOWER = -0.70
TILT_UPPER = 1.00


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _make_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY").box(*BASE_SIZE, centered=(True, True, False)).edges("|Z").fillet(0.006)
    )
    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_SIZE[2]))
    )
    mounting_ring = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.028, -0.018),
                (-0.028, 0.018),
                (0.028, -0.018),
                (0.028, 0.018),
            ]
        )
        .circle(0.004)
        .extrude(0.0025)
        .translate((0.0, 0.0, BASE_SIZE[2]))
    )
    return plate.union(pedestal).union(mounting_ring)


def _make_pan_yoke_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(PAN_COLLAR_RADIUS).extrude(PAN_COLLAR_HEIGHT)
    deck = (
        cq.Workplane("XY")
        .box(*PAN_DECK_SIZE, centered=(True, True, False))
        .translate((0.0, 0.0, PAN_COLLAR_HEIGHT))
    )
    left_arm = (
        cq.Workplane("XY")
        .box(*YOKE_ARM_SIZE, centered=(True, True, False))
        .translate((0.0, YOKE_ARM_Y, PAN_COLLAR_HEIGHT + PAN_DECK_SIZE[2]))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(*YOKE_ARM_SIZE, centered=(True, True, False))
        .translate((0.0, -YOKE_ARM_Y, PAN_COLLAR_HEIGHT + PAN_DECK_SIZE[2]))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(0.010, 0.030, 0.022, centered=(True, True, False))
        .translate((0.016, 0.0, PAN_COLLAR_HEIGHT + PAN_DECK_SIZE[2]))
    )
    return collar.union(deck).union(left_arm).union(right_arm).union(rear_rib)


def _make_sensor_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(*SENSOR_BODY_SIZE).edges("|Z").fillet(0.004)
    lens_bezel = cq.Workplane("YZ").circle(0.014).extrude(0.006).translate((-0.035, 0.0, 0.0))
    lens_barrel = cq.Workplane("YZ").circle(0.011).extrude(0.014).translate((-0.043, 0.0, 0.0))
    rear_block = cq.Workplane("XY").box(0.010, 0.024, 0.024).translate((0.032, 0.0, 0.0))
    top_rib = cq.Workplane("XY").box(0.024, 0.020, 0.004).translate((0.008, 0.0, 0.020))
    left_trunnion = cq.Workplane("XZ").circle(0.0065).extrude(0.006).translate((0.0, 0.016, 0.0))
    right_trunnion = cq.Workplane("XZ").circle(0.0065).extrude(0.006).translate((0.0, -0.022, 0.0))
    return (
        body.union(lens_bezel)
        .union(lens_barrel)
        .union(rear_block)
        .union(top_rib)
        .union(left_trunnion)
        .union(right_trunnion)
    ).translate((SENSOR_X_OFFSET, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_pan_tilt", assets=ASSETS)

    model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("anodized_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    model.material("sensor_glass", rgba=(0.34, 0.54, 0.66, 0.65))

    base = model.part("base")
    _add_mesh_visual(base, _make_base_shape(), "base.obj", "anodized_gray")


    base.inertial = Inertial.from_geometry(
        Box((BASE_SIZE[0], BASE_SIZE[1], BASE_SIZE[2] + PEDESTAL_HEIGHT)),
        mass=0.80,
        origin=Origin(xyz=(0.0, 0.0, (BASE_SIZE[2] + PEDESTAL_HEIGHT) / 2.0)),
    )

    pan_yoke = model.part("pan_yoke")
    _add_mesh_visual(pan_yoke, _make_pan_yoke_shape(), "pan_yoke.obj", "powder_black")




    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.062, 0.054, 0.068)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
    )

    sensor_head = model.part("sensor_head")
    _add_mesh_visual(sensor_head, _make_sensor_shape(), "sensor_head.obj", "anodized_gray")
    sensor_head.visual(
        Cylinder(radius=0.0095, length=0.0035),
        origin=Origin(xyz=(SENSOR_X_OFFSET - 0.048, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material="sensor_glass",
    )


    sensor_head.inertial = Inertial.from_geometry(
        Box(SENSOR_MASS_PROXY_SIZE),
        mass=0.22,
        origin=Origin(xyz=(SENSOR_X_OFFSET, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_pan",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] + PEDESTAL_HEIGHT + 0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-PAN_LIMIT,
            upper=PAN_LIMIT,
            effort=8.0,
            velocity=2.5,
        ),
    )
    model.articulation(
        "pan_to_sensor",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=sensor_head,
        origin=Origin(xyz=(0.0, 0.0, TILT_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=TILT_LOWER,
            upper=TILT_UPPER,
            effort=4.0,
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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("pan_yoke", "base", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("sensor_head", "base", axes="xy", max_dist=0.030)
    ctx.expect_origin_distance("sensor_head", "pan_yoke", axes="xy", max_dist=0.030)
    ctx.expect_aabb_overlap("pan_yoke", "base", axes="xy", min_overlap=0.040)
    ctx.expect_aabb_overlap("sensor_head", "pan_yoke", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("pan_yoke", "base", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_origin_gap("sensor_head", "base", axis="z", min_gap=0.020)
    ctx.expect_joint_motion_axis(
        "base_to_pan",
        "sensor_head",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "pan_to_sensor",
        "sensor_head",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )

    with ctx.pose(base_to_pan=PAN_LIMIT):
        ctx.expect_origin_distance("pan_yoke", "base", axes="xy", max_dist=0.005)
        ctx.expect_origin_distance("sensor_head", "base", axes="xy", max_dist=0.030)
        ctx.expect_origin_gap("sensor_head", "base", axis="z", min_gap=0.020)

    with ctx.pose(base_to_pan=-PAN_LIMIT):
        ctx.expect_origin_distance("sensor_head", "base", axes="xy", max_dist=0.030)
        ctx.expect_aabb_overlap("pan_yoke", "base", axes="xy", min_overlap=0.040)

    with ctx.pose(pan_to_sensor=TILT_LOWER):
        ctx.expect_origin_distance("sensor_head", "base", axes="xy", max_dist=0.030)
        ctx.expect_origin_gap("sensor_head", "base", axis="z", min_gap=0.020)
        ctx.expect_aabb_overlap("sensor_head", "pan_yoke", axes="xy", min_overlap=0.018)

    with ctx.pose(pan_to_sensor=TILT_UPPER):
        ctx.expect_origin_distance("sensor_head", "base", axes="xy", max_dist=0.030)
        ctx.expect_origin_gap("sensor_head", "base", axis="z", min_gap=0.020)
        ctx.expect_aabb_overlap("sensor_head", "pan_yoke", axes="xy", min_overlap=0.018)

    with ctx.pose(base_to_pan=1.6, pan_to_sensor=TILT_LOWER):
        ctx.expect_origin_distance("sensor_head", "base", axes="xy", max_dist=0.030)
        ctx.expect_origin_gap("sensor_head", "base", axis="z", min_gap=0.020)

    with ctx.pose(base_to_pan=-1.6, pan_to_sensor=TILT_UPPER):
        ctx.expect_origin_distance("sensor_head", "base", axes="xy", max_dist=0.030)
        ctx.expect_origin_gap("sensor_head", "base", axis="z", min_gap=0.020)

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END
