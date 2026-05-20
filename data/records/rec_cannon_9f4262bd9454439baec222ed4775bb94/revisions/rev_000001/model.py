from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _hollow_x_tube(*, outer_radius: float, inner_radius: float, length: float, x0: float):
    """CadQuery tube along +X with a visible bore."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_deck_cannon")

    gunmetal = model.material("gunmetal", rgba=(0.13, 0.15, 0.16, 1.0))
    parkerized = model.material("parkerized", rgba=(0.05, 0.055, 0.06, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.30, 0.32, 0.34, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.52, 0.54, 0.55, 1.0))
    brass = model.material("oiled_bronze", rgba=(0.72, 0.55, 0.25, 1.0))
    shadow = model.material("bore_shadow", rgba=(0.005, 0.005, 0.004, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.62, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=deck_gray,
        name="deck_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.48, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=gunmetal,
        name="lower_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.28, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        material=gunmetal,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.52, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=deck_gray,
        name="top_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.22, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.055)),
        material=parkerized,
        name="center_spigot",
    )
    for index in range(12):
        angle = math.tau * index / 12.0
        x = 0.50 * math.cos(angle)
        y = 0.50 * math.sin(angle)
        pedestal.visual(
            Cylinder(radius=0.026, length=0.035),
            origin=Origin(xyz=(x, y, 0.107)),
            material=worn_steel,
            name=f"deck_bolt_{index:02d}",
        )
    for index in range(8):
        angle = math.tau * index / 8.0 + math.pi / 8.0
        x = 0.46 * math.cos(angle)
        y = 0.46 * math.sin(angle)
        pedestal.visual(
            Cylinder(radius=0.020, length=0.030),
            origin=Origin(xyz=(x, y, 1.035)),
            material=worn_steel,
            name=f"bearing_bolt_{index:02d}",
        )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=1.10),
        mass=1850.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.39, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=gunmetal,
        name="turntable_disk",
    )
    carriage.visual(
        Cylinder(radius=0.31, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=parkerized,
        name="slew_housing",
    )
    carriage.visual(
        Box((0.74, 0.080, 0.52)),
        origin=Origin(xyz=(0.06, 0.240, 0.355)),
        material=gunmetal,
        name="cheek_0",
    )
    carriage.visual(
        Box((0.74, 0.080, 0.52)),
        origin=Origin(xyz=(0.06, -0.240, 0.355)),
        material=gunmetal,
        name="cheek_1",
    )
    carriage.visual(
        Box((0.28, 0.58, 0.10)),
        origin=Origin(xyz=(-0.24, 0.0, 0.155)),
        material=gunmetal,
        name="rear_crossmember",
    )
    carriage.visual(
        Box((0.20, 0.58, 0.09)),
        origin=Origin(xyz=(0.32, 0.0, 0.145)),
        material=gunmetal,
        name="front_crossmember",
    )
    carriage.visual(
        Cylinder(radius=0.135, length=0.105),
        origin=_y_cylinder_origin(0.0, 0.2475, 0.420),
        material=worn_steel,
        name="bearing_cap_0",
    )
    carriage.visual(
        Cylinder(radius=0.092, length=0.112),
        origin=_y_cylinder_origin(0.0, 0.2475, 0.420),
        material=brass,
        name="bearing_bush_0",
    )
    carriage.visual(
        Cylinder(radius=0.135, length=0.105),
        origin=_y_cylinder_origin(0.0, -0.2475, 0.420),
        material=worn_steel,
        name="bearing_cap_1",
    )
    carriage.visual(
        Cylinder(radius=0.092, length=0.112),
        origin=_y_cylinder_origin(0.0, -0.2475, 0.420),
        material=brass,
        name="bearing_bush_1",
    )
    for index, y in enumerate((0.255, -0.255)):
        carriage.visual(
            Box((0.08, 0.05, 0.33)),
            origin=Origin(xyz=(-0.19, y, 0.275), rpy=(0.0, -0.35, 0.0)),
            material=parkerized,
            name=f"rear_rib_{index}",
        )
        carriage.visual(
            Box((0.08, 0.05, 0.33)),
            origin=Origin(xyz=(0.29, y, 0.265), rpy=(0.0, 0.34, 0.0)),
            material=parkerized,
            name=f"front_rib_{index}",
        )
    carriage.visual(
        Box((0.34, 0.58, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, 0.655)),
        material=parkerized,
        name="top_tie",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.82, 0.62, 0.66)),
        mass=900.0,
        origin=Origin(xyz=(0.05, 0.0, 0.32)),
    )

    barrel_tube = mesh_from_cadquery(
        _hollow_x_tube(outer_radius=0.092, inner_radius=0.038, length=1.18, x0=-0.02),
        "rifled_barrel_tube",
        tolerance=0.001,
        angular_tolerance=0.08,
    )
    muzzle_ring = mesh_from_cadquery(
        _hollow_x_tube(outer_radius=0.130, inner_radius=0.052, length=0.13, x0=1.08),
        "muzzle_reinforce_mesh",
        tolerance=0.001,
        angular_tolerance=0.08,
    )

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=0.073, length=0.390),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material=worn_steel,
        name="trunnion_pin",
    )
    barrel.visual(
        Cylinder(radius=0.155, length=0.30),
        origin=_x_cylinder_origin(-0.145, 0.0, 0.0),
        material=gunmetal,
        name="breech_reinforce",
    )
    barrel.visual(
        Sphere(radius=0.135),
        origin=Origin(xyz=(-0.315, 0.0, 0.0)),
        material=gunmetal,
        name="rounded_cascabel",
    )
    barrel.visual(
        Cylinder(radius=0.085, length=0.055),
        origin=_x_cylinder_origin(-0.425, 0.0, 0.0),
        material=brass,
        name="breech_knob",
    )
    barrel.visual(
        barrel_tube,
        material=gunmetal,
        name="barrel_tube",
    )
    barrel.visual(
        muzzle_ring,
        material=gunmetal,
        name="muzzle_reinforce",
    )
    barrel.visual(
        Cylinder(radius=0.054, length=0.010),
        origin=_x_cylinder_origin(1.205, 0.0, 0.0),
        material=shadow,
        name="bore_shadow",
    )
    barrel.visual(
        Cylinder(radius=0.112, length=0.070),
        origin=_x_cylinder_origin(0.060, 0.0, 0.0),
        material=parkerized,
        name="trunnion_band",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=1.66),
        mass=1250.0,
        origin=_x_cylinder_origin(0.42, 0.0, 0.0),
    )

    model.articulation(
        "traverse",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 1.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=0.55),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=42000.0, velocity=0.35, lower=-0.10, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    traverse = object_model.get_articulation("traverse")
    elevation = object_model.get_articulation("elevation")

    ctx.check(
        "carriage traverses continuously",
        traverse.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={traverse.articulation_type}",
    )
    ctx.check(
        "barrel elevates on revolute trunnion",
        elevation.articulation_type == ArticulationType.REVOLUTE and elevation.axis == (0.0, -1.0, 0.0),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    ctx.allow_overlap(
        pedestal,
        carriage,
        elem_a="center_spigot",
        elem_b="turntable_disk",
        reason="The fixed center spigot is intentionally nested into the turntable disk as the azimuth bearing proxy.",
    )
    ctx.expect_within(
        pedestal,
        carriage,
        axes="xy",
        inner_elem="center_spigot",
        outer_elem="turntable_disk",
        margin=0.0,
        name="center spigot is contained by turntable socket footprint",
    )
    ctx.expect_overlap(
        pedestal,
        carriage,
        axes="z",
        elem_a="center_spigot",
        elem_b="turntable_disk",
        min_overlap=0.040,
        name="center spigot remains inserted in turntable disk",
    )
    ctx.expect_gap(
        carriage,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="top_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="turntable sits on pedestal bearing",
    )
    ctx.expect_overlap(
        carriage,
        pedestal,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="top_bearing",
        min_overlap=0.70,
        name="turntable footprint is centered on bearing",
    )
    ctx.expect_gap(
        carriage,
        barrel,
        axis="y",
        positive_elem="bearing_bush_0",
        negative_elem="trunnion_pin",
        max_gap=0.006,
        max_penetration=0.005,
        name="positive trunnion end seated in bush",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="y",
        positive_elem="trunnion_pin",
        negative_elem="bearing_bush_1",
        max_gap=0.006,
        max_penetration=0.005,
        name="negative trunnion end seated in bush",
    )

    rest_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_reinforce")
    rest_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5 if rest_aabb else None
    with ctx.pose({elevation: 0.40}):
        raised_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_reinforce")
        raised_z = (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5 if raised_aabb else None
    ctx.check(
        "elevation raises muzzle",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.30,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
