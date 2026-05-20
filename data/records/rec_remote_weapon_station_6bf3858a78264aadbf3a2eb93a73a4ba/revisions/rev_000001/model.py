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


def _make_slewing_ring() -> cq.Workplane:
    """A low, open annular bearing ring with visible bolt heads."""

    outer_radius = 0.43
    inner_radius = 0.27
    height = 0.055
    ring = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)

    bolt_radius = 0.017
    bolt_circle = 0.355
    for i in range(16):
        angle = 2.0 * math.pi * i / 16
        x = bolt_circle * math.cos(angle)
        y = bolt_circle * math.sin(angle)
        bolt = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(bolt_radius)
            .extrude(0.014)
            .translate((0.0, 0.0, height))
        )
        ring = ring.union(bolt)
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_remote_weapon_station")

    model.material("olive_armor", rgba=(0.23, 0.27, 0.18, 1.0))
    model.material("dark_steel", rgba=(0.05, 0.055, 0.052, 1.0))
    model.material("worn_edge", rgba=(0.42, 0.45, 0.38, 1.0))
    model.material("rubber_black", rgba=(0.01, 0.012, 0.012, 1.0))
    model.material("sensor_glass", rgba=(0.05, 0.12, 0.16, 0.78))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.56, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material="olive_armor",
        name="ground_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.23, length=0.550),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material="olive_armor",
        name="pedestal_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.39, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material="olive_armor",
        name="top_flange",
    )
    for i, (x, y) in enumerate(
        ((0.36, 0.36), (-0.36, 0.36), (-0.36, -0.36), (0.36, -0.36))
    ):
        pedestal.visual(
            Cylinder(radius=0.060, length=0.028),
            origin=Origin(xyz=(x, y, 0.094)),
            material="dark_steel",
            name=f"anchor_bolt_{i}",
        )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        Cylinder(radius=0.47, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material="olive_armor",
        name="rotating_deck",
    )
    azimuth_stage.visual(
        mesh_from_cadquery(_make_slewing_ring(), "central_slewing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material="dark_steel",
        name="central_slewing_ring",
    )
    azimuth_stage.visual(
        Box((0.66, 0.78, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material="olive_armor",
        name="yoke_base",
    )
    for side, y, cheek_name, boss_name, shield_name in (
        (0, 0.35, "side_cheek_0", "pivot_boss_0", "sloped_shield_0"),
        (1, -0.35, "side_cheek_1", "pivot_boss_1", "sloped_shield_1"),
    ):
        azimuth_stage.visual(
            Box((0.56, 0.100, 0.600)),
            origin=Origin(xyz=(0.00, y, 0.550)),
            material="olive_armor",
            name=cheek_name,
        )
        azimuth_stage.visual(
            Cylinder(radius=0.115, length=0.045),
            origin=Origin(xyz=(0.0, 0.419 if y > 0.0 else -0.419, 0.600), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="dark_steel",
            name=boss_name,
        )
        azimuth_stage.visual(
            Box((0.110, 0.210, 0.440)),
            origin=Origin(
                xyz=(0.285, 0.455 if y > 0.0 else -0.455, 0.465),
                rpy=(0.0, 0.0, 0.18 if y > 0.0 else -0.18),
            ),
            material="olive_armor",
            name=shield_name,
        )
    azimuth_stage.visual(
        Box((0.120, 0.700, 0.180)),
        origin=Origin(xyz=(-0.265, 0.0, 0.360)),
        material="olive_armor",
        name="rear_bridge",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.062, length=0.600),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="trunnion",
    )
    cradle.visual(
        Box((0.360, 0.250, 0.220)),
        origin=Origin(xyz=(0.130, 0.0, 0.000)),
        material="olive_armor",
        name="breech_box",
    )
    cradle.visual(
        Cylinder(radius=0.078, length=0.360),
        origin=Origin(xyz=(0.290, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="recoil_sleeve",
    )
    cradle.visual(
        Cylinder(radius=0.050, length=0.940),
        origin=Origin(xyz=(0.785, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_steel",
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.073, length=0.160),
        origin=Origin(xyz=(1.315, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber_black",
        name="muzzle_brake",
    )
    cradle.visual(
        Box((0.210, 0.105, 0.060)),
        origin=Origin(xyz=(0.230, -0.165, 0.110)),
        material="olive_armor",
        name="sensor_arm",
    )
    cradle.visual(
        Box((0.220, 0.140, 0.155)),
        origin=Origin(xyz=(0.320, -0.210, 0.160)),
        material="dark_steel",
        name="sensor_housing",
    )
    cradle.visual(
        Cylinder(radius=0.045, length=0.024),
        origin=Origin(xyz=(0.438, -0.210, 0.178), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="sensor_glass",
        name="day_lens",
    )
    cradle.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.438, -0.210, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="sensor_glass",
        name="thermal_lens",
    )

    azimuth = model.articulation(
        "azimuth",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.710)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=650.0,
            velocity=0.9,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    pitch = model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.8,
            lower=-0.35,
            upper=0.95,
        ),
    )
    # Keep local variables deliberately referenced so frame names remain clear.
    _ = (azimuth, pitch)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    azimuth_stage = object_model.get_part("azimuth_stage")
    cradle = object_model.get_part("cradle")
    azimuth = object_model.get_articulation("azimuth")
    pitch = object_model.get_articulation("pitch")

    ctx.expect_gap(
        azimuth_stage,
        pedestal,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0005,
        positive_elem="rotating_deck",
        negative_elem="top_flange",
        name="azimuth deck sits on pedestal flange",
    )
    ctx.expect_overlap(
        azimuth_stage,
        pedestal,
        axes="xy",
        elem_a="rotating_deck",
        elem_b="top_flange",
        min_overlap=0.65,
        name="slewing stage is centered over pedestal",
    )
    ctx.expect_gap(
        azimuth_stage,
        cradle,
        axis="y",
        max_gap=0.007,
        max_penetration=0.0,
        positive_elem="side_cheek_0",
        negative_elem="trunnion",
        name="positive cheek closely captures trunnion",
    )
    ctx.expect_gap(
        cradle,
        azimuth_stage,
        axis="y",
        max_gap=0.007,
        max_penetration=0.0,
        positive_elem="trunnion",
        negative_elem="side_cheek_1",
        name="negative cheek closely captures trunnion",
    )

    rest_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    with ctx.pose({pitch: 0.95}):
        raised_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    ctx.check(
        "pitch axis raises weapon at elevation limit",
        rest_muzzle is not None
        and raised_muzzle is not None
        and (raised_muzzle[0][2] + raised_muzzle[1][2]) * 0.5
        > (rest_muzzle[0][2] + rest_muzzle[1][2]) * 0.5 + 0.55,
        details=f"rest_muzzle={rest_muzzle}, raised_muzzle={raised_muzzle}",
    )

    rest_sensor = ctx.part_element_world_aabb(cradle, elem="sensor_housing")
    with ctx.pose({azimuth: math.pi / 2.0}):
        yawed_sensor = ctx.part_element_world_aabb(cradle, elem="sensor_housing")
    ctx.check(
        "azimuth stage slews upper weapon package",
        rest_sensor is not None
        and yawed_sensor is not None
        and abs(((yawed_sensor[0][1] + yawed_sensor[1][1]) * 0.5) - ((rest_sensor[0][0] + rest_sensor[1][0]) * 0.5))
        < 0.050,
        details=f"rest_sensor={rest_sensor}, yawed_sensor={yawed_sensor}",
    )

    return ctx.report()


object_model = build_object_model()
