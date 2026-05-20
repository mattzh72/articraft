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
PLATE_RADIUS = 0.046
PLATE_THICKNESS = 0.0025

HOUSING_REAR_RADIUS = 0.037
HOUSING_REAR_DEPTH = 0.011
HOUSING_FRONT_RADIUS = 0.0315
HOUSING_FRONT_DEPTH = 0.011

DIAL_OUTER_RADIUS = 0.0415
DIAL_DEPTH = 0.012
DIAL_GLASS_RADIUS = 0.0283
DIAL_REAR_CAVITY_RADIUS = 0.0332
DIAL_REAR_CAVITY_DEPTH = 0.006

DIAL_AXIS_Z_IN_HOUSING = 0.0175


def _wall_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(PLATE_RADIUS)
        .workplane(offset=PLATE_THICKNESS)
        .circle(PLATE_RADIUS - 0.0038)
        .loft(combine=True)
    )


def _housing_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(HOUSING_REAR_RADIUS)
        .workplane(offset=HOUSING_REAR_DEPTH * 0.55)
        .circle(HOUSING_REAR_RADIUS - 0.0015)
        .workplane(offset=HOUSING_REAR_DEPTH * 0.45)
        .circle(HOUSING_FRONT_RADIUS + 0.003)
        .workplane(offset=HOUSING_FRONT_DEPTH)
        .circle(HOUSING_FRONT_RADIUS)
        .loft(combine=True)
    )


def _dial_ring_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XY", origin=(0.0, 0.0, -DIAL_DEPTH / 2.0))
        .circle(DIAL_REAR_CAVITY_RADIUS + 0.004)
        .workplane(offset=DIAL_DEPTH * 0.35)
        .circle(DIAL_OUTER_RADIUS)
        .workplane(offset=DIAL_DEPTH * 0.35)
        .circle(DIAL_OUTER_RADIUS - 0.0015)
        .workplane(offset=DIAL_DEPTH * 0.30)
        .circle(DIAL_GLASS_RADIUS + 0.0035)
        .loft(combine=True)
    )
    front_pocket = (
        cq.Workplane("XY", origin=(0.0, 0.0, DIAL_DEPTH / 2.0 - 0.0045))
        .circle(DIAL_GLASS_RADIUS - 0.0005)
        .extrude(0.0045)
    )
    rear_cavity = (
        cq.Workplane("XY", origin=(0.0, 0.0, -DIAL_DEPTH / 2.0))
        .circle(DIAL_REAR_CAVITY_RADIUS)
        .extrude(DIAL_REAR_CAVITY_DEPTH)
    )
    ring = ring.cut(front_pocket).cut(rear_cavity)
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat", assets=ASSETS)

    model.material("ceramic_white", rgba=(0.93, 0.94, 0.92, 1.0))
    model.material("soft_white", rgba=(0.97, 0.97, 0.95, 1.0))
    model.material("brushed_aluminum", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("graphite_glass", rgba=(0.11, 0.12, 0.14, 1.0))
    model.material("display_blue", rgba=(0.41, 0.69, 0.97, 0.95))
    model.material("sensor_black", rgba=(0.07, 0.08, 0.09, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_wall_plate_shape(), "wall_plate.obj", assets=ASSETS),
        material="soft_white",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATE_RADIUS, length=PLATE_THICKNESS),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS / 2.0)),
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing.obj", assets=ASSETS),
        material="ceramic_white",
    )
    housing.visual(
        Box((0.014, 0.0032, 0.001)),
        origin=Origin(
            xyz=(
                0.0,
                -0.015,
                HOUSING_REAR_DEPTH + HOUSING_FRONT_DEPTH - 0.0005,
            )
        ),
        material="sensor_black",
    )
    housing.inertial = Inertial.from_geometry(
        Cylinder(
            radius=HOUSING_REAR_RADIUS,
            length=HOUSING_REAR_DEPTH + HOUSING_FRONT_DEPTH,
        ),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, (HOUSING_REAR_DEPTH + HOUSING_FRONT_DEPTH) / 2.0)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_ring_shape(), "dial_ring.obj", assets=ASSETS),
        material="brushed_aluminum",
    )
    dial.visual(
        Cylinder(radius=DIAL_GLASS_RADIUS, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material="graphite_glass",
    )
    dial.visual(
        Box((0.017, 0.005, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0105, 0.0038)),
        material="display_blue",
    )
    dial.visual(
        Cylinder(radius=0.0015, length=0.001),
        origin=Origin(xyz=(0.0, -0.0095, 0.0035)),
        material="sensor_black",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=DIAL_OUTER_RADIUS, length=DIAL_DEPTH),
        mass=0.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "plate_mount",
        ArticulationType.FIXED,
        parent="wall_plate",
        child="housing",
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS)),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent="housing",
        child="dial",
        origin=Origin(xyz=(0.0, 0.0, DIAL_AXIS_Z_IN_HOUSING)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("housing", "wall_plate", axes="xy", max_dist=0.001)
    ctx.expect_aabb_overlap("housing", "wall_plate", axes="xy", min_overlap=0.07)
    ctx.expect_aabb_gap("housing", "wall_plate", axis="z", max_gap=0.001, max_penetration=0.0005)

    ctx.expect_origin_distance("dial", "housing", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("dial", "housing", axes="xy", min_overlap=0.07)
    ctx.expect_origin_distance("dial", "wall_plate", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("dial", "wall_plate", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_gap("dial", "wall_plate", axis="z", max_gap=0.02, max_penetration=0.0)

    for angle in (0.0, pi / 2.0, pi, 3.0 * pi / 2.0):
        with ctx.pose(dial_spin=angle):
            ctx.expect_origin_distance("dial", "housing", axes="xy", max_dist=0.002)
            ctx.expect_aabb_overlap("dial", "housing", axes="xy", min_overlap=0.07)
            ctx.expect_origin_distance("dial", "wall_plate", axes="xy", max_dist=0.002)
            ctx.expect_aabb_overlap("dial", "wall_plate", axes="xy", min_overlap=0.08)
            ctx.expect_aabb_gap("dial", "wall_plate", axis="z", max_gap=0.02, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
