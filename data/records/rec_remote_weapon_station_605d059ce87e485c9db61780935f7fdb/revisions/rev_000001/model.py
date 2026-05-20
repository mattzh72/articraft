from __future__ import annotations

from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


AZIMUTH_Z = 0.99
ELEVATION_Z = 0.58
ELEVATION_LOWER = radians(-10.0)
ELEVATION_UPPER = radians(70.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_remote_weapon_station")

    model.material("olive_armor", rgba=(0.22, 0.28, 0.18, 1.0))
    model.material("dark_steel", rgba=(0.06, 0.07, 0.07, 1.0))
    model.material("worn_ring", rgba=(0.38, 0.38, 0.34, 1.0))
    model.material("matte_black", rgba=(0.015, 0.016, 0.016, 1.0))
    model.material("optic_glass", rgba=(0.02, 0.09, 0.12, 0.78))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.56, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material="olive_armor",
        name="floor_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.24, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
        material="olive_armor",
        name="pedestal_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.42, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        material="olive_armor",
        name="top_flange",
    )
    pedestal.visual(
        mesh_from_geometry(TorusGeometry(radius=0.34, tube=0.035), "fixed_slewing_race"),
        origin=Origin(xyz=(0.0, 0.0, AZIMUTH_Z - 0.035)),
        material="worn_ring",
        name="fixed_race",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_geometry(TorusGeometry(radius=0.34, tube=0.035), "moving_slewing_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material="worn_ring",
        name="moving_race",
    )
    yaw_stage.visual(
        Box((0.92, 0.94, 0.10)),
        origin=Origin(xyz=(0.08, 0.0, 0.10)),
        material="olive_armor",
        name="rotating_deck",
    )
    yaw_stage.visual(
        Box((0.72, 0.08, 0.70)),
        origin=Origin(xyz=(0.16, 0.38, 0.50)),
        material="olive_armor",
        name="side_cheek_0",
    )
    yaw_stage.visual(
        Box((0.72, 0.08, 0.70)),
        origin=Origin(xyz=(0.16, -0.38, 0.50)),
        material="olive_armor",
        name="side_cheek_1",
    )
    yaw_stage.visual(
        Box((0.12, 0.78, 0.54)),
        origin=Origin(xyz=(-0.24, 0.0, 0.43)),
        material="olive_armor",
        name="rear_bridge",
    )
    yaw_stage.visual(
        Cylinder(radius=0.14, length=0.10),
        origin=Origin(xyz=(0.0, 0.405, ELEVATION_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="trunnion_boss_0",
    )
    yaw_stage.visual(
        Cylinder(radius=0.14, length=0.10),
        origin=Origin(xyz=(0.0, -0.405, ELEVATION_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="trunnion_boss_1",
    )
    yaw_stage.visual(
        Box((0.64, 0.07, 0.48)),
        origin=Origin(xyz=(0.20, 0.445, 0.42)),
        material="olive_armor",
        name="side_shield_0",
    )
    yaw_stage.visual(
        Box((0.64, 0.07, 0.48)),
        origin=Origin(xyz=(0.20, -0.445, 0.42)),
        material="olive_armor",
        name="side_shield_1",
    )
    yaw_stage.visual(
        Box((0.07, 0.18, 0.34)),
        origin=Origin(xyz=(0.49, 0.31, 0.32), rpy=(0.0, -0.18, 0.0)),
        material="olive_armor",
        name="front_shield_0",
    )
    yaw_stage.visual(
        Box((0.07, 0.18, 0.34)),
        origin=Origin(xyz=(0.49, -0.31, 0.32), rpy=(0.0, -0.18, 0.0)),
        material="olive_armor",
        name="front_shield_1",
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Cylinder(radius=0.065, length=0.68),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="trunnion",
    )
    weapon_cradle.visual(
        Box((0.54, 0.26, 0.22)),
        origin=Origin(xyz=(0.23, 0.0, 0.0)),
        material="dark_steel",
        name="breech_block",
    )
    weapon_cradle.visual(
        Box((0.58, 0.18, 0.16)),
        origin=Origin(xyz=(0.49, 0.0, 0.01)),
        material="matte_black",
        name="launcher_box",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.045, length=0.76),
        origin=Origin(xyz=(0.84, 0.0, 0.02), rpy=(0.0, pi / 2.0, 0.0)),
        material="matte_black",
        name="gun_barrel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.065, length=0.09),
        origin=Origin(xyz=(1.26, 0.0, 0.02), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="muzzle_brake",
    )
    weapon_cradle.visual(
        Box((0.08, 0.10, 0.22)),
        origin=Origin(xyz=(0.32, 0.0, -0.12)),
        material="dark_steel",
        name="sensor_strut",
    )
    weapon_cradle.visual(
        Box((0.28, 0.20, 0.14)),
        origin=Origin(xyz=(0.46, 0.0, -0.23)),
        material="matte_black",
        name="sensor_housing",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.055, length=0.07),
        origin=Origin(xyz=(0.635, 0.0, -0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material="optic_glass",
        name="sensor_window",
    )

    model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, AZIMUTH_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=weapon_cradle,
        origin=Origin(xyz=(0.0, 0.0, ELEVATION_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=0.8,
            lower=ELEVATION_LOWER,
            upper=ELEVATION_UPPER,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_stage = object_model.get_part("yaw_stage")
    weapon_cradle = object_model.get_part("weapon_cradle")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.expect_gap(
        yaw_stage,
        pedestal,
        axis="z",
        positive_elem="moving_race",
        negative_elem="fixed_race",
        max_penetration=0.00001,
        max_gap=0.003,
        name="slewing races seat together without penetration",
    )
    ctx.expect_gap(
        yaw_stage,
        weapon_cradle,
        axis="y",
        positive_elem="side_cheek_0",
        negative_elem="trunnion",
        max_penetration=0.00001,
        max_gap=0.003,
        name="positive cheek bears on the trunnion",
    )
    ctx.expect_gap(
        weapon_cradle,
        yaw_stage,
        axis="y",
        positive_elem="trunnion",
        negative_elem="side_cheek_1",
        max_penetration=0.00001,
        max_gap=0.003,
        name="negative cheek bears on the trunnion",
    )
    ctx.check(
        "azimuth axis is vertical",
        tuple(round(v, 6) for v in azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"axis={azimuth.axis}",
    )
    ctx.check(
        "elevation limits match weapon station",
        elevation.motion_limits is not None
        and abs(elevation.motion_limits.lower - ELEVATION_LOWER) < 1e-6
        and abs(elevation.motion_limits.upper - ELEVATION_UPPER) < 1e-6,
        details=f"limits={elevation.motion_limits}",
    )

    rest_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="muzzle_brake")
    with ctx.pose({elevation: ELEVATION_UPPER}):
        raised_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="muzzle_brake")
    with ctx.pose({elevation: ELEVATION_LOWER}):
        lowered_aabb = ctx.part_element_world_aabb(weapon_cradle, elem="muzzle_brake")
    ctx.check(
        "elevation raises the muzzle",
        rest_aabb is not None
        and raised_aabb is not None
        and lowered_aabb is not None
        and (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0
        > (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
        + 0.35
        and (lowered_aabb[0][2] + lowered_aabb[1][2]) / 2.0
        < (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0
        - 0.08,
        details=f"rest={rest_aabb}, raised={raised_aabb}, lowered={lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
