from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
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


def _build_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.68, 0.50, 0.035)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, 0.0175))
    )
    pads = (
        cq.Workplane("XY")
        .pushPoints([(-0.24, -0.17), (-0.24, 0.17), (0.24, -0.17), (0.24, 0.17)])
        .circle(0.032)
        .extrude(0.018)
        .translate((0.0, 0.0, 0.035))
    )
    lower_pedestal = (
        cq.Workplane("XY")
        .box(0.56, 0.36, 0.10)
        .edges("|Z")
        .fillet(0.02)
        .translate((0.0, 0.0, 0.085))
    )
    upper_pedestal = (
        cq.Workplane("XY")
        .box(0.44, 0.28, 0.08)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, 0.175))
    )
    rear_access_box = (
        cq.Workplane("XY")
        .box(0.16, 0.22, 0.09)
        .edges("|Z")
        .fillet(0.012)
        .translate((-0.19, 0.0, 0.085))
    )
    slew_ring = (
        cq.Workplane("XY")
        .circle(0.18)
        .circle(0.12)
        .extrude(0.04)
        .translate((0.0, 0.0, 0.20))
    )
    ring_cap = cq.Workplane("XY").circle(0.12).extrude(0.015).translate((0.0, 0.0, 0.225))

    shape = plate.union(pads)
    shape = shape.union(lower_pedestal)
    shape = shape.union(upper_pedestal)
    shape = shape.union(rear_access_box)
    shape = shape.union(slew_ring)
    shape = shape.union(ring_cap)
    return shape


def _build_housing_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(0.17)
        .circle(0.11)
        .extrude(0.02)
        .translate((0.0, 0.0, -0.02))
    )
    lower_body = (
        cq.Workplane("XY")
        .box(0.34, 0.28, 0.18)
        .edges("|Z")
        .fillet(0.015)
        .translate((-0.06, 0.0, 0.09))
    )
    roof_block = (
        cq.Workplane("XY")
        .box(0.26, 0.24, 0.08)
        .edges("|Z")
        .fillet(0.012)
        .translate((-0.02, 0.0, 0.26))
    )
    top_sensor_mount = (
        cq.Workplane("XY")
        .box(0.12, 0.14, 0.06)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.03, 0.0, 0.34))
    )
    rear_bustle = (
        cq.Workplane("XY")
        .box(0.18, 0.20, 0.12)
        .edges("|Z")
        .fillet(0.01)
        .translate((-0.23, 0.0, 0.19))
    )
    front_visor = (
        cq.Workplane("XY")
        .box(0.12, 0.22, 0.12)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.11, 0.0, 0.20))
    )
    left_applique = (
        cq.Workplane("XY")
        .box(0.24, 0.08, 0.20)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.00, 0.18, 0.18))
    )
    right_applique = (
        cq.Workplane("XY")
        .box(0.24, 0.08, 0.20)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.00, -0.18, 0.18))
    )
    left_support = cq.Workplane("XY").box(0.20, 0.06, 0.10).translate((0.10, 0.17, 0.24))
    right_support = cq.Workplane("XY").box(0.20, 0.06, 0.10).translate((0.10, -0.17, 0.24))
    left_trunnion_boss = (
        cq.Workplane("XZ").circle(0.05).extrude(0.06).translate((0.18, 0.14, 0.25))
    )
    right_trunnion_boss = (
        cq.Workplane("XZ").circle(0.05).extrude(0.06).translate((0.18, -0.20, 0.25))
    )

    shape = collar.union(lower_body)
    shape = shape.union(roof_block)
    shape = shape.union(top_sensor_mount)
    shape = shape.union(rear_bustle)
    shape = shape.union(front_visor)
    shape = shape.union(left_applique)
    shape = shape.union(right_applique)
    shape = shape.union(left_support)
    shape = shape.union(right_support)
    shape = shape.union(left_trunnion_boss)
    shape = shape.union(right_trunnion_boss)
    return shape


def _build_cradle_shape() -> cq.Workplane:
    trunnion_shaft = (
        cq.Workplane("XZ").circle(0.038).extrude(0.36).translate((0.0, -0.18, 0.0))
    )
    left_yoke = (
        cq.Workplane("XY")
        .box(0.30, 0.05, 0.18)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.07, 0.16, 0.0))
    )
    right_yoke = (
        cq.Workplane("XY")
        .box(0.30, 0.05, 0.18)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.07, -0.16, 0.0))
    )
    receiver_block = (
        cq.Workplane("XY")
        .box(0.30, 0.24, 0.14)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.14, 0.0, 0.02))
    )
    recoil_cover = (
        cq.Workplane("XY")
        .box(0.34, 0.16, 0.08)
        .edges("|Z")
        .fillet(0.008)
        .translate((0.20, 0.0, 0.10))
    )
    rear_counterweight = (
        cq.Workplane("XY")
        .box(0.14, 0.18, 0.14)
        .edges("|Z")
        .fillet(0.01)
        .translate((-0.10, 0.0, 0.02))
    )
    undercarriage = cq.Workplane("XY").box(0.18, 0.10, 0.08).translate((0.03, 0.0, -0.08))
    ammo_box = (
        cq.Workplane("XY")
        .box(0.22, 0.12, 0.18)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.08, -0.20, -0.01))
    )
    blast_shield = cq.Workplane("XY").box(0.02, 0.28, 0.20).translate((0.24, 0.0, 0.04))
    sight_mount = (
        cq.Workplane("XY")
        .box(0.10, 0.08, 0.05)
        .edges("|Z")
        .fillet(0.006)
        .translate((0.34, 0.09, 0.15))
    )
    barrel_jacket = cq.Workplane("YZ").circle(0.035).extrude(0.30).translate((0.28, 0.0, 0.02))
    barrel = cq.Workplane("YZ").circle(0.018).extrude(0.56).translate((0.58, 0.0, 0.02))
    muzzle_collar = (
        cq.Workplane("YZ").circle(0.026).extrude(0.08).translate((1.10, 0.0, 0.02))
    )
    muzzle = cq.Workplane("YZ").circle(0.021).extrude(0.05).translate((1.18, 0.0, 0.02))

    shape = trunnion_shaft.union(left_yoke)
    shape = shape.union(right_yoke)
    shape = shape.union(receiver_block)
    shape = shape.union(recoil_cover)
    shape = shape.union(rear_counterweight)
    shape = shape.union(undercarriage)
    shape = shape.union(ammo_box)
    shape = shape.union(blast_shield)
    shape = shape.union(sight_mount)
    shape = shape.union(barrel_jacket)
    shape = shape.union(barrel)
    shape = shape.union(muzzle_collar)
    shape = shape.union(muzzle)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station", assets=ASSETS)

    olive_drab = model.material("olive_drab", rgba=(0.38, 0.43, 0.29, 1.0))
    darker_olive = model.material("darker_olive", rgba=(0.29, 0.33, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.22, 0.23, 0.25, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.29, 0.34, 0.95))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "remote_weapon_station_base.obj", assets=ASSETS),
        material=darker_olive,
    )
    for x in (-0.26, -0.08, 0.08, 0.26):
        for y in (-0.19, 0.19):
            base.visual(
                Cylinder(radius=0.012, length=0.01),
                origin=Origin(xyz=(x, y, 0.04)),
                material=steel,
            )
    base.inertial = Inertial.from_geometry(
        Box((0.70, 0.52, 0.28)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(
            _build_housing_shape(),
            "remote_weapon_station_housing.obj",
            assets=ASSETS,
        ),
        material=olive_drab,
    )
    housing.visual(
        Box((0.02, 0.09, 0.06)),
        origin=Origin(xyz=(0.16, -0.09, 0.31)),
        material=glass,
    )
    housing.visual(
        Box((0.02, 0.06, 0.04)),
        origin=Origin(xyz=(0.15, 0.08, 0.30)),
        material=glass,
    )
    housing.visual(
        Box((0.05, 0.10, 0.04)),
        origin=Origin(xyz=(-0.20, 0.0, 0.31)),
        material=matte_black,
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.48, 0.42, 0.42)),
        mass=82.0,
        origin=Origin(xyz=(-0.03, 0.0, 0.18)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        mesh_from_cadquery(
            _build_cradle_shape(),
            "remote_weapon_station_cradle.obj",
            assets=ASSETS,
        ),
        material=matte_black,
    )
    cradle.visual(
        Box((0.07, 0.05, 0.035)),
        origin=Origin(xyz=(0.35, 0.10, 0.17)),
        material=glass,
    )
    cradle.visual(
        Box((0.16, 0.10, 0.02)),
        origin=Origin(xyz=(0.09, -0.26, 0.02)),
        material=steel,
    )
    cradle.inertial = Inertial.from_geometry(
        Box((1.24, 0.46, 0.30)),
        mass=68.0,
        origin=Origin(xyz=(0.52, 0.0, 0.02)),
    )

    model.articulation(
        "base_to_housing_pan",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="housing",
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=1.4),
    )
    model.articulation(
        "housing_to_cradle_elevation",
        ArticulationType.REVOLUTE,
        parent="housing",
        child="cradle",
        origin=Origin(xyz=(0.18, 0.0, 0.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=950.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("housing", "base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("housing", "base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_gap("housing", "base", axis="z", max_gap=0.01, max_penetration=0.025)
    ctx.expect_aabb_contact("housing", "cradle")
    ctx.expect_aabb_overlap("housing", "cradle", axes="yz", min_overlap=0.14)
    ctx.expect_aabb_overlap("housing", "cradle", axes="y", min_overlap=0.18)
    ctx.expect_joint_motion_axis(
        "housing_to_cradle_elevation",
        "cradle",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    for pan_angle in (0.0, pi / 2, pi):
        with ctx.pose(base_to_housing_pan=pan_angle):
            ctx.expect_origin_distance("housing", "base", axes="xy", max_dist=0.01)
            ctx.expect_aabb_overlap("housing", "base", axes="xy", min_overlap=0.18)
            ctx.expect_aabb_gap("housing", "base", axis="z", max_gap=0.01, max_penetration=0.025)
            ctx.expect_aabb_contact("housing", "cradle")

    for elev_angle in (-0.35, 1.10):
        with ctx.pose(housing_to_cradle_elevation=elev_angle):
            ctx.expect_aabb_contact("housing", "cradle")
            ctx.expect_aabb_overlap("housing", "cradle", axes="yz", min_overlap=0.10)
            ctx.expect_aabb_overlap("housing", "cradle", axes="y", min_overlap=0.18)

    with ctx.pose(base_to_housing_pan=pi / 2):
        ctx.expect_joint_motion_axis(
            "housing_to_cradle_elevation",
            "cradle",
            world_axis="z",
            direction="positive",
            min_delta=0.02,
        )

    with ctx.pose({"base_to_housing_pan": pi / 2, "housing_to_cradle_elevation": 1.10}):
        ctx.expect_origin_distance("housing", "base", axes="xy", max_dist=0.01)
        ctx.expect_aabb_contact("housing", "cradle")
        ctx.expect_aabb_overlap("housing", "cradle", axes="y", min_overlap=0.18)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
