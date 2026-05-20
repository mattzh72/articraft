from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_band(outer_radius: float, inner_radius: float, height: float, fillet: float) -> cq.Workplane:
    """Create a low annular metal band in meters, with softened machined edges."""
    band = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    if fillet > 0.0:
        band = band.edges().fillet(fillet)
    return band


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_fidget_ring_set")

    matte_black = model.material("matte_black", rgba=(0.015, 0.015, 0.018, 1.0))
    dark_race = model.material("dark_bearing_race", rgba=(0.08, 0.085, 0.09, 1.0))
    cobalt = model.material("cobalt_anodized_aluminum", rgba=(0.05, 0.18, 0.85, 1.0))
    graphite = model.material("graphite_anodized_aluminum", rgba=(0.19, 0.20, 0.22, 1.0))
    nickel = model.material("brushed_nickel_magnet", rgba=(0.78, 0.76, 0.70, 1.0))
    red_pole = model.material("red_magnet_pole", rgba=(0.85, 0.05, 0.04, 1.0))
    blue_pole = model.material("blue_magnet_pole", rgba=(0.02, 0.18, 0.95, 1.0))

    base_height = 0.0035
    race_height = 0.0012
    race_top_z = base_height + race_height - 0.0002
    ring_height = 0.0075
    ring_center_z = race_top_z + ring_height / 2.0

    inner_ring_inner = 0.0125
    inner_ring_outer = 0.0235
    outer_ring_inner = 0.0295
    outer_ring_outer = 0.0410

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.047, length=base_height),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=matte_black,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(
            _annular_band(0.0232, 0.0128, race_height, 0.00025),
            "inner_bearing_race",
            tolerance=0.00035,
            angular_tolerance=0.04,
        ),
        origin=Origin(xyz=(0.0, 0.0, base_height - 0.0002)),
        material=dark_race,
        name="inner_bearing_race",
    )
    base.visual(
        mesh_from_cadquery(
            _annular_band(0.0405, 0.0300, race_height, 0.00025),
            "outer_bearing_race",
            tolerance=0.00035,
            angular_tolerance=0.04,
        ),
        origin=Origin(xyz=(0.0, 0.0, base_height - 0.0002)),
        material=dark_race,
        name="outer_bearing_race",
    )
    base.visual(
        mesh_from_cadquery(
            _annular_band(0.0275, 0.0255, 0.0080, 0.0002),
            "center_separator_ridge",
            tolerance=0.00035,
            angular_tolerance=0.04,
        ),
        origin=Origin(xyz=(0.0, 0.0, base_height - 0.0002)),
        material=matte_black,
        name="separator_ridge",
    )
    base.visual(
        mesh_from_cadquery(
            _annular_band(0.0465, 0.0430, 0.0080, 0.00025),
            "outer_guard_rim",
            tolerance=0.00035,
            angular_tolerance=0.04,
        ),
        origin=Origin(xyz=(0.0, 0.0, base_height - 0.0002)),
        material=matte_black,
        name="guard_rim",
    )
    base.visual(
        Cylinder(radius=0.0090, length=0.0100),
        origin=Origin(xyz=(0.0, 0.0, base_height + 0.0050)),
        material=dark_race,
        name="center_spindle",
    )
    base.visual(
        Cylinder(radius=0.0140, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.01345)),
        material=matte_black,
        name="retainer_cap",
    )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        mesh_from_cadquery(
            _annular_band(inner_ring_outer, inner_ring_inner, ring_height, 0.0010),
            "inner_ring_band",
            tolerance=0.00035,
            angular_tolerance=0.035,
        ),
        origin=Origin(xyz=(0.0, 0.0, -ring_height / 2.0)),
        material=cobalt,
        name="ring_band",
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(
            _annular_band(outer_ring_outer, outer_ring_inner, ring_height, 0.0010),
            "outer_ring_band",
            tolerance=0.00035,
            angular_tolerance=0.035,
        ),
        origin=Origin(xyz=(0.0, 0.0, -ring_height / 2.0)),
        material=graphite,
        name="ring_band",
    )

    magnet_height = 0.0008
    magnet_embed = 0.00015
    magnet_z = ring_height / 2.0 + magnet_height / 2.0 - magnet_embed
    for index in range(8):
        angle = index * 6.283185307179586 / 8
        radius = 0.0180
        inner_ring.visual(
            Cylinder(radius=0.0028, length=magnet_height),
            origin=Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), magnet_z)),
            material=red_pole if index % 2 == 0 else nickel,
            name=f"magnet_{index}",
        )

    for index in range(12):
        angle = index * 6.283185307179586 / 12
        radius = 0.0352
        outer_ring.visual(
            Cylinder(radius=0.0031, length=magnet_height),
            origin=Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), magnet_z)),
            material=blue_pole if index % 2 == 0 else nickel,
            name=f"magnet_{index}",
        )

    spin_props = MotionProperties(damping=0.003, friction=0.0005)
    model.articulation(
        "inner_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=inner_ring,
        origin=Origin(xyz=(0.0, 0.0, ring_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=30.0),
        motion_properties=spin_props,
    )
    model.articulation(
        "outer_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, ring_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=30.0),
        motion_properties=spin_props,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    inner_ring = object_model.get_part("inner_ring")
    outer_ring = object_model.get_part("outer_ring")
    inner_spin = object_model.get_articulation("inner_spin")
    outer_spin = object_model.get_articulation("outer_spin")

    ctx.check(
        "inner ring has continuous spin joint",
        inner_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(inner_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={inner_spin.articulation_type}, axis={inner_spin.axis}",
    )
    ctx.check(
        "outer ring has independent continuous spin joint",
        outer_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(outer_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={outer_spin.articulation_type}, axis={outer_spin.axis}",
    )

    ctx.expect_origin_distance(
        inner_ring,
        outer_ring,
        axes="xy",
        max_dist=0.0001,
        name="rings share central spin axis",
    )
    ctx.expect_gap(
        inner_ring,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="ring_band",
        negative_elem="inner_bearing_race",
        name="inner ring rides on bearing race",
    )
    ctx.expect_gap(
        outer_ring,
        base,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="ring_band",
        negative_elem="outer_bearing_race",
        name="outer ring rides on bearing race",
    )
    with ctx.pose({inner_spin: 1.25, outer_spin: -0.85}):
        ctx.expect_origin_distance(
            inner_ring,
            outer_ring,
            axes="xy",
            max_dist=0.0001,
            name="independent rotations keep rings coaxial",
        )

    return ctx.report()


object_model = build_object_model()
