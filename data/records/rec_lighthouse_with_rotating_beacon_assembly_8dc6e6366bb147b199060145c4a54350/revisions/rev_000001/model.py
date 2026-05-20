from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    ConeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _tower_radius(z: float) -> float:
    """Radius of the tapered lighthouse shaft at world height z."""
    z0, z1 = 0.48, 4.95
    r0, r1 = 0.92, 0.48
    t = max(0.0, min(1.0, (z - z0) / (z1 - z0)))
    return r0 + (r1 - r0) * t


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offshore_lighthouse_turntable_beacon")

    concrete = model.material("weathered_concrete", rgba=(0.78, 0.76, 0.68, 1.0))
    white = model.material("marine_white_paint", rgba=(0.92, 0.91, 0.84, 1.0))
    red = model.material("red_navigation_bands", rgba=(0.72, 0.05, 0.04, 1.0))
    dark = model.material("dark_wet_foundation", rgba=(0.16, 0.16, 0.15, 1.0))
    metal = model.material("galvanized_metal", rgba=(0.56, 0.59, 0.57, 1.0))
    glass = model.material("pale_blue_glass", rgba=(0.55, 0.82, 0.95, 0.38))
    bronze = model.material("aged_bronze", rgba=(0.70, 0.45, 0.18, 1.0))
    beacon_glass = model.material("warm_beacon_glass", rgba=(1.0, 0.86, 0.28, 0.78))

    lighthouse = model.part("lighthouse")

    # Offshore pile and caisson base: broad, low, and heavy relative to the tower.
    lighthouse.visual(
        Cylinder(radius=1.85, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark,
        name="caisson_plinth",
    )
    lighthouse.visual(
        Cylinder(radius=1.22, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=concrete,
        name="round_foundation",
    )

    shaft_profile = [
        (0.0, 0.40),
        (0.94, 0.40),
        (0.90, 0.80),
        (0.72, 2.70),
        (0.54, 4.72),
        (0.50, 4.95),
        (0.0, 4.95),
    ]
    lighthouse.visual(
        mesh_from_geometry(LatheGeometry(shaft_profile, segments=80), "tapered_tower_mesh"),
        material=white,
        name="tapered_tower",
    )

    for i, (z, h) in enumerate(((1.30, 0.28), (2.75, 0.32), (4.16, 0.36))):
        lighthouse.visual(
            Cylinder(radius=_tower_radius(z) + 0.035, length=h),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=red,
            name=f"red_band_{i}",
        )

    # Gallery deck with a continuous outer guard rail and individual stanchions.
    lighthouse.visual(
        Cylinder(radius=1.05, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 5.02)),
        material=metal,
        name="gallery_deck",
    )
    lighthouse.visual(
        mesh_from_geometry(TorusGeometry(radius=0.96, tube=0.025, radial_segments=20, tubular_segments=96), "lower_gallery_rail"),
        origin=Origin(xyz=(0.0, 0.0, 5.25)),
        material=metal,
        name="lower_gallery_rail",
    )
    lighthouse.visual(
        mesh_from_geometry(TorusGeometry(radius=0.96, tube=0.025, radial_segments=20, tubular_segments=96), "upper_gallery_rail"),
        origin=Origin(xyz=(0.0, 0.0, 5.58)),
        material=metal,
        name="upper_gallery_rail",
    )
    for i in range(12):
        theta = 2.0 * math.pi * i / 12.0
        lighthouse.visual(
            Cylinder(radius=0.022, length=0.54),
            origin=Origin(xyz=(0.96 * math.cos(theta), 0.96 * math.sin(theta), 5.33)),
            material=metal,
            name=f"gallery_post_{i}",
        )

    # Compact lantern room below the beacon: short glazed panels, not a tall cage.
    lighthouse.visual(
        Cylinder(radius=0.60, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 5.14)),
        material=metal,
        name="lantern_sill",
    )
    lighthouse.visual(
        Cylinder(radius=0.60, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 5.84)),
        material=metal,
        name="lantern_header",
    )
    for i in range(8):
        theta = 2.0 * math.pi * i / 8.0
        lighthouse.visual(
            Box((0.36, 0.024, 0.70)),
            origin=Origin(
                xyz=(0.575 * math.cos(theta), 0.575 * math.sin(theta), 5.49),
                rpy=(0.0, 0.0, theta - math.pi / 2.0),
            ),
            material=glass,
            name=f"glass_panel_{i}",
        )
    for i in range(8):
        theta = 2.0 * math.pi * (i + 0.5) / 8.0
        lighthouse.visual(
            Cylinder(radius=0.024, length=0.76),
            origin=Origin(xyz=(0.60 * math.cos(theta), 0.60 * math.sin(theta), 5.49)),
            material=metal,
            name=f"lantern_mullion_{i}",
        )

    lighthouse.visual(
        mesh_from_geometry(ConeGeometry(radius=0.70, height=0.44, radial_segments=72), "compact_roof_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 6.10)),
        material=red,
        name="compact_roof",
    )
    lighthouse.visual(
        Cylinder(radius=0.16, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 6.43)),
        material=metal,
        name="turntable_pedestal",
    )

    beacon = model.part("beacon_head")
    beacon.visual(
        Cylinder(radius=0.23, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=metal,
        name="turntable_disk",
    )
    beacon.visual(
        Cylinder(radius=0.075, length=0.33),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=metal,
        name="rotating_stem",
    )
    beacon.visual(
        Box((0.18, 0.46, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=bronze,
        name="yoke_crossbar",
    )
    for y, suffix in ((-0.19, "0"), (0.19, "1")):
        beacon.visual(
            Box((0.13, 0.055, 0.42)),
            origin=Origin(xyz=(0.0, y, 0.62)),
            material=bronze,
            name=f"yoke_plate_{suffix}",
        )
    beacon.visual(
        Cylinder(radius=0.165, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.68), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="beacon_barrel",
    )
    for x, suffix in ((-0.36, "rear"), (0.36, "front")):
        beacon.visual(
            Cylinder(radius=0.145, length=0.08),
            origin=Origin(xyz=(x, 0.0, 0.68), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=beacon_glass,
            name=f"{suffix}_lens",
        )
    beacon.visual(
        Box((0.48, 0.10, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.87)),
        material=bronze,
        name="beacon_visor",
    )

    model.articulation(
        "pedestal_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=lighthouse,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 6.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lighthouse = object_model.get_part("lighthouse")
    beacon = object_model.get_part("beacon_head")
    turntable = object_model.get_articulation("pedestal_to_beacon")

    ctx.check(
        "beacon uses continuous vertical turntable",
        turntable.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in turntable.axis) == (0.0, 0.0, 1.0),
        details=f"type={turntable.articulation_type}, axis={turntable.axis}",
    )
    ctx.expect_gap(
        beacon,
        lighthouse,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="turntable_pedestal",
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
        name="rotating disk sits on pedestal top",
    )
    ctx.expect_gap(
        beacon,
        lighthouse,
        axis="z",
        positive_elem="beacon_barrel",
        negative_elem="compact_roof",
        min_gap=0.40,
        name="beacon housing is carried above roof line",
    )

    rest_aabb = ctx.part_element_world_aabb(beacon, elem="beacon_barrel")
    with ctx.pose({turntable: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(beacon, elem="beacon_barrel")
    ctx.check(
        "beacon head yaws about pedestal",
        rest_aabb is not None
        and turned_aabb is not None
        and (rest_aabb[1][0] - rest_aabb[0][0]) > (rest_aabb[1][1] - rest_aabb[0][1]) + 0.25
        and (turned_aabb[1][1] - turned_aabb[0][1]) > (turned_aabb[1][0] - turned_aabb[0][0]) + 0.25,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
