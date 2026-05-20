from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _side_plate_mesh(y_sign: float) -> object:
    """Armored cheek plate with a real trunnion clearance hole."""
    thickness = 0.050
    y_ref = 0.305 if y_sign > 0.0 else -0.255
    local_pivot_z = 0.260
    bottom_z = 0.180

    profile = [
        (-0.315, 0.000),
        (0.310, 0.000),
        (0.285, 0.335),
        (0.085, 0.455),
        (-0.260, 0.420),
        (-0.335, 0.190),
    ]
    plate = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(thickness)
        .translate((0.0, y_ref, bottom_z))
    )
    hole = (
        cq.Workplane("XZ")
        .circle(0.088)
        .extrude(thickness + 0.010)
        .translate((0.0, y_ref + 0.005, bottom_z + local_pivot_z))
    )
    return plate.cut(hole)


def _pivot_ring_mesh(y_sign: float) -> object:
    """A hollow bearing ring that surrounds the elevation shaft."""
    thickness = 0.032
    y_ref = 0.337 if y_sign > 0.0 else -0.305
    return (
        cq.Workplane("XZ")
        .circle(0.120)
        .circle(0.056)
        .extrude(thickness)
        .translate((0.0, y_ref, 0.440))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_remote_weapon_station")

    olive = model.material("mat_olive_drab", rgba=(0.23, 0.28, 0.19, 1.0))
    dark_olive = model.material("mat_dark_armor", rgba=(0.16, 0.18, 0.14, 1.0))
    dark_metal = model.material("mat_dark_phosphate", rgba=(0.035, 0.038, 0.037, 1.0))
    worn_steel = model.material("mat_worn_steel", rgba=(0.36, 0.37, 0.34, 1.0))
    black = model.material("mat_flat_black", rgba=(0.005, 0.005, 0.004, 1.0))
    glass = model.material("mat_sensor_glass", rgba=(0.02, 0.05, 0.08, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.62, 0.62, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_olive,
        name="base_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.165, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=olive,
        name="low_column",
    )
    pedestal.visual(
        Cylinder(radius=0.265, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=dark_olive,
        name="top_flange",
    )
    for index, (x, y) in enumerate(
        ((0.235, 0.235), (-0.235, 0.235), (-0.235, -0.235), (0.235, -0.235))
    ):
        pedestal.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(xyz=(x, y, 0.055)),
            material=worn_steel,
            name=f"anchor_bolt_{index}",
        )

    turret_base = model.part("turret_base")
    turret_base.visual(
        Cylinder(radius=0.285, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_olive,
        name="turntable_disk",
    )
    turret_base.visual(
        Box((0.560, 0.620, 0.125)),
        origin=Origin(xyz=(0.0, 0.0, 0.1425)),
        material=olive,
        name="rotary_housing",
    )
    turret_base.visual(
        Box((0.410, 0.520, 0.055)),
        origin=Origin(xyz=(0.015, 0.0, 0.2325)),
        material=dark_olive,
        name="raised_deck",
    )
    for index, y_sign in enumerate((1.0, -1.0)):
        turret_base.visual(
            mesh_from_cadquery(_side_plate_mesh(y_sign), f"side_plate_{index}"),
            material=olive,
            name=f"side_plate_{index}",
        )
        turret_base.visual(
            mesh_from_cadquery(_pivot_ring_mesh(y_sign), f"pivot_ring_{index}"),
            material=dark_metal,
            name=f"pivot_ring_{index}",
        )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Cylinder(radius=0.042, length=0.490),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_shaft",
    )
    for index, y in enumerate((0.245, -0.245)):
        weapon_cradle.visual(
            Cylinder(radius=0.105, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"trunnion_cap_{index}",
        )
    weapon_cradle.visual(
        Box((0.240, 0.360, 0.150)),
        origin=Origin(xyz=(-0.030, 0.0, -0.055)),
        material=dark_metal,
        name="cradle_block",
    )
    weapon_cradle.visual(
        Box((0.235, 0.140, 0.180)),
        origin=Origin(xyz=(0.125, -0.125, -0.005)),
        material=dark_olive,
        name="sensor_body",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.044, length=0.030),
        origin=Origin(xyz=(0.257, -0.125, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="main_lens",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.254, -0.125, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="thermal_lens",
    )
    weapon_cradle.visual(
        Box((0.235, 0.145, 0.155)),
        origin=Origin(xyz=(0.155, 0.120, -0.015)),
        material=dark_metal,
        name="receiver",
    )
    weapon_cradle.visual(
        Box((0.155, 0.105, 0.120)),
        origin=Origin(xyz=(0.030, 0.170, -0.100)),
        material=dark_olive,
        name="feed_box",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.027, length=0.540),
        origin=Origin(xyz=(0.530, 0.120, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="barrel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.042, length=0.230),
        origin=Origin(xyz=(0.385, 0.120, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="barrel_shroud",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.034, length=0.080),
        origin=Origin(xyz=(0.840, 0.120, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="muzzle_brake",
    )

    model.articulation(
        "pedestal_to_turret",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turret_base,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.3,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "turret_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turret_base,
        child=weapon_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.9,
            lower=-0.35,
            upper=0.85,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    pedestal = object_model.get_part("pedestal")
    turret_base = object_model.get_part("turret_base")
    weapon_cradle = object_model.get_part("weapon_cradle")
    yaw = object_model.get_articulation("pedestal_to_turret")
    pitch = object_model.get_articulation("turret_to_cradle")

    ctx.expect_gap(
        turret_base,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="top_flange",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on pedestal flange",
    )
    ctx.expect_overlap(
        turret_base,
        pedestal,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="top_flange",
        min_overlap=0.20,
        name="yaw bearing is centered on pedestal",
    )
    ctx.expect_within(
        weapon_cradle,
        turret_base,
        axes="y",
        inner_elem="cradle_block",
        outer_elem="raised_deck",
        margin=0.001,
        name="cradle block stays between side plates",
    )

    rest_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="muzzle_brake")
    with ctx.pose({pitch: 0.70}):
        raised_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="muzzle_brake")
    ctx.check(
        "positive pitch elevates the muzzle",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[0][2] > rest_aabb[0][2] + 0.12,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    with ctx.pose({yaw: 0.65}):
        yawed_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="muzzle_brake")
    ctx.check(
        "yaw rotates the weapon off centerline",
        rest_aabb is not None
        and yawed_aabb is not None
        and abs(yawed_aabb[0][1] - rest_aabb[0][1]) > 0.15,
        details=f"rest={rest_aabb}, yawed={yawed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
