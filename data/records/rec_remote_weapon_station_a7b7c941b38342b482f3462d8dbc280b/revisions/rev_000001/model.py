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


def _frustum_mesh(bottom_radius: float, top_radius: float, height: float, name: str):
    """A simple broad-to-narrow armored pedestal skirt."""
    frustum = (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=height)
        .circle(top_radius)
        .loft(combine=True)
    )
    return mesh_from_cadquery(frustum, name, tolerance=0.002, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_remote_weapon_station")

    navy_grey = model.material("navy_grey", rgba=(0.30, 0.34, 0.35, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.08, 0.09, 0.09, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.47, 0.50, 0.50, 1.0))
    optic_black = model.material("optic_black", rgba=(0.005, 0.006, 0.008, 1.0))
    lens_blue = model.material("lens_blue", rgba=(0.05, 0.12, 0.20, 1.0))

    deck_base = model.part("deck_base")
    deck_base.visual(
        Cylinder(radius=0.72, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=navy_grey,
        name="deck_disc",
    )
    deck_base.visual(
        Cylinder(radius=0.50, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=bearing_steel,
        name="fixed_bearing_ring",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        deck_base.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(
                xyz=(0.60 * math.cos(angle), 0.60 * math.sin(angle), 0.109)
            ),
            material=bearing_steel,
            name=f"bolt_{i}",
        )

    pedestal = model.part("pedestal")
    pedestal.visual(
        _frustum_mesh(0.56, 0.38, 0.18, "rotating_skirt"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=navy_grey,
        name="rotating_skirt",
    )
    pedestal.visual(
        Cylinder(radius=0.42, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=bearing_steel,
        name="azimuth_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.31, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=navy_grey,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.76, 0.72, 0.18)),
        origin=Origin(xyz=(0.04, 0.0, 0.56)),
        material=navy_grey,
        name="upper_frame",
    )
    pedestal.visual(
        Box((0.52, 0.09, 0.44)),
        origin=Origin(xyz=(0.08, -0.315, 0.79)),
        material=navy_grey,
        name="cheek_0",
    )
    pedestal.visual(
        Box((0.52, 0.09, 0.44)),
        origin=Origin(xyz=(0.08, 0.315, 0.79)),
        material=navy_grey,
        name="cheek_1",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.060),
        origin=Origin(xyz=(0.08, -0.390, 0.79), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="trunnion_cap_0",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.060),
        origin=Origin(xyz=(0.08, 0.390, 0.79), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="trunnion_cap_1",
    )

    hinge_x = 0.02
    hinge_y = 0.455
    hinge_z = 0.96
    pedestal.visual(
        Box((0.24, 0.070, 0.050)),
        origin=Origin(xyz=(hinge_x, 0.390, hinge_z - 0.060)),
        material=navy_grey,
        name="hinge_standoff",
    )
    pedestal.visual(
        Box((0.24, 0.06, 0.050)),
        origin=Origin(xyz=(hinge_x, 0.450, hinge_z - 0.060)),
        material=navy_grey,
        name="hinge_base",
    )
    pedestal.visual(
        Cylinder(radius=0.035, length=0.054),
        origin=Origin(
            xyz=(hinge_x - 0.073, hinge_y, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bearing_steel,
        name="hinge_knuckle_0",
    )
    pedestal.visual(
        Cylinder(radius=0.035, length=0.054),
        origin=Origin(
            xyz=(hinge_x + 0.073, hinge_y, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bearing_steel,
        name="hinge_knuckle_1",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Box((0.55, 0.32, 0.22)),
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
        material=navy_grey,
        name="armored_receiver",
    )
    cradle.visual(
        Box((0.44, 0.035, 0.27)),
        origin=Origin(xyz=(0.10, -0.178, 0.025)),
        material=navy_grey,
        name="side_armor_0",
    )
    cradle.visual(
        Box((0.44, 0.035, 0.27)),
        origin=Origin(xyz=(0.10, 0.178, 0.025)),
        material=navy_grey,
        name="side_armor_1",
    )
    cradle.visual(
        Cylinder(radius=0.060, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_steel,
        name="trunnion_axle",
    )
    cradle.visual(
        Cylinder(radius=0.058, length=0.70),
        origin=Origin(xyz=(0.745, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.078, length=0.11),
        origin=Origin(xyz=(1.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="muzzle_brake",
    )
    cradle.visual(
        Box((0.44, 0.16, 0.055)),
        origin=Origin(xyz=(0.17, 0.0, 0.1375)),
        material=navy_grey,
        name="top_armor",
    )
    cradle.visual(
        Cylinder(radius=0.028, length=0.48),
        origin=Origin(xyz=(0.56, -0.095, -0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="recoil_rod_0",
    )
    cradle.visual(
        Cylinder(radius=0.028, length=0.48),
        origin=Origin(xyz=(0.56, 0.095, -0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="recoil_rod_1",
    )

    sensor_mast = model.part("sensor_mast")
    sensor_mast.visual(
        Cylinder(radius=0.030, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_steel,
        name="hinge_barrel",
    )
    sensor_mast.visual(
        Cylinder(radius=0.023, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=navy_grey,
        name="mast_tube",
    )
    sensor_mast.visual(
        Box((0.18, 0.13, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        material=navy_grey,
        name="sensor_head",
    )
    sensor_mast.visual(
        Cylinder(radius=0.035, length=0.030),
        origin=Origin(xyz=(0.101, -0.035, 0.485), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=optic_black,
        name="optic_lens",
    )
    sensor_mast.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(xyz=(0.102, 0.040, 0.465), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_blue,
        name="laser_window",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=deck_base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.45),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=cradle,
        origin=Origin(xyz=(0.08, 0.0, 0.79)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.7, lower=-0.25, upper=1.05),
    )
    model.articulation(
        "mast_hinge",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=sensor_mast,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck_base = object_model.get_part("deck_base")
    pedestal = object_model.get_part("pedestal")
    cradle = object_model.get_part("cradle")
    sensor_mast = object_model.get_part("sensor_mast")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")
    mast_hinge = object_model.get_articulation("mast_hinge")

    ctx.expect_gap(
        pedestal,
        deck_base,
        axis="z",
        positive_elem="rotating_skirt",
        negative_elem="fixed_bearing_ring",
        min_gap=0.0,
        max_gap=0.004,
        name="rotating pedestal sits just above azimuth bearing",
    )
    ctx.expect_within(
        cradle,
        pedestal,
        axes="y",
        inner_elem="trunnion_axle",
        outer_elem="upper_frame",
        margin=0.0,
        name="cradle trunnion is contained between the cheek plates",
    )
    ctx.expect_gap(
        pedestal,
        cradle,
        axis="y",
        positive_elem="cheek_1",
        negative_elem="trunnion_axle",
        min_gap=0.0,
        max_gap=0.002,
        name="positive cheek clears trunnion axle",
    )
    ctx.expect_gap(
        cradle,
        pedestal,
        axis="y",
        positive_elem="trunnion_axle",
        negative_elem="cheek_0",
        min_gap=0.0,
        max_gap=0.002,
        name="negative cheek clears trunnion axle",
    )
    ctx.expect_gap(
        sensor_mast,
        pedestal,
        axis="x",
        positive_elem="hinge_barrel",
        negative_elem="hinge_knuckle_0",
        max_penetration=0.0001,
        max_gap=0.002,
        name="mast hinge barrel is clipped after the first knuckle",
    )
    ctx.expect_gap(
        pedestal,
        sensor_mast,
        axis="x",
        positive_elem="hinge_knuckle_1",
        negative_elem="hinge_barrel",
        max_penetration=0.0001,
        max_gap=0.002,
        name="mast hinge barrel is clipped before the second knuckle",
    )
    ctx.expect_overlap(
        sensor_mast,
        pedestal,
        axes="yz",
        elem_a="hinge_barrel",
        elem_b="hinge_knuckle_0",
        min_overlap=0.020,
        name="mast hinge barrel stays coaxial with lower knuckle",
    )
    ctx.expect_overlap(
        sensor_mast,
        pedestal,
        axes="yz",
        elem_a="hinge_barrel",
        elem_b="hinge_knuckle_1",
        min_overlap=0.020,
        name="mast hinge barrel stays coaxial with upper knuckle",
    )

    rest_cradle_pos = ctx.part_world_position(cradle)
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_cradle_pos = ctx.part_world_position(cradle)
    ctx.check(
        "azimuth rotates the upper weapon assembly about vertical axis",
        rest_cradle_pos is not None
        and turned_cradle_pos is not None
        and turned_cradle_pos[1] > rest_cradle_pos[1] + 0.06,
        details=f"rest={rest_cradle_pos}, turned={turned_cradle_pos}",
    )

    rest_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    with ctx.pose({elevation: 0.80}):
        elevated_muzzle = ctx.part_element_world_aabb(cradle, elem="muzzle_brake")
    ctx.check(
        "positive elevation raises the weapon muzzle",
        rest_muzzle is not None
        and elevated_muzzle is not None
        and elevated_muzzle[1][2] > rest_muzzle[1][2] + 0.25,
        details=f"rest={rest_muzzle}, elevated={elevated_muzzle}",
    )

    rest_head = ctx.part_element_world_aabb(sensor_mast, elem="sensor_head")
    with ctx.pose({mast_hinge: 1.05}):
        folded_head = ctx.part_element_world_aabb(sensor_mast, elem="sensor_head")
        ctx.expect_gap(
            sensor_mast,
            pedestal,
            axis="x",
            positive_elem="hinge_barrel",
            negative_elem="hinge_knuckle_0",
            max_penetration=0.0001,
            max_gap=0.002,
            name="folded mast remains clipped after first knuckle",
        )
        ctx.expect_gap(
            pedestal,
            sensor_mast,
            axis="x",
            positive_elem="hinge_knuckle_1",
            negative_elem="hinge_barrel",
            max_penetration=0.0001,
            max_gap=0.002,
            name="folded mast remains clipped before second knuckle",
        )
    rest_head_y = None if rest_head is None else (rest_head[0][1] + rest_head[1][1]) * 0.5
    folded_head_y = None if folded_head is None else (folded_head[0][1] + folded_head[1][1]) * 0.5
    ctx.check(
        "mast hinge folds the sensor head outward from the upper frame",
        rest_head_y is not None
        and folded_head_y is not None
        and folded_head_y > rest_head_y + 0.20,
        details=f"rest_y={rest_head_y}, folded_y={folded_head_y}",
    )

    return ctx.report()


object_model = build_object_model()
