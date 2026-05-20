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


def _annulus_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    """A flat annular ring, authored with its bottom face on local z=0."""
    ring = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    return mesh_from_cadquery(ring, name, tolerance=0.001, angular_tolerance=0.04)


def _bronze_barrel_mesh():
    """Turned naval cannon barrel with a visible muzzle bore."""
    # Profile is in the XZ plane and revolved around the X axis.  The trunnion
    # joint frame is at x=0 on the bore centerline.
    profile = [
        (-0.38, 0.000),
        (-0.38, 0.070),
        (-0.34, 0.105),
        (-0.22, 0.122),
        (-0.14, 0.108),
        (-0.03, 0.116),
        (0.06, 0.102),
        (0.22, 0.096),
        (0.62, 0.084),
        (1.05, 0.072),
        (1.18, 0.088),
        (1.30, 0.078),
        (1.30, 0.000),
    ]
    barrel = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .revolve(360, axisStart=(0, 0, 0), axisEnd=(1, 0, 0))
    )
    bore_cutter = cq.Workplane("YZ").circle(0.038).extrude(0.24).translate((1.10, 0, 0))
    barrel = barrel.cut(bore_cutter)
    return mesh_from_cadquery(barrel, "bronze_barrel", tolerance=0.0008, angular_tolerance=0.035)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_deck_cannon")

    bronze = model.material("aged_bronze", rgba=(0.63, 0.39, 0.17, 1.0))
    dark_bore = model.material("shadowed_bore", rgba=(0.015, 0.012, 0.010, 1.0))
    black_iron = model.material("blackened_iron", rgba=(0.025, 0.028, 0.030, 1.0))
    worn_iron = model.material("worn_iron", rgba=(0.18, 0.17, 0.16, 1.0))
    oak = model.material("oiled_oak", rgba=(0.45, 0.26, 0.12, 1.0))
    deck_oak = model.material("salted_deck_oak", rgba=(0.55, 0.41, 0.25, 1.0))
    seam = model.material("caulked_seam", rgba=(0.035, 0.028, 0.020, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((2.40, 1.80, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=deck_oak,
        name="deck_planks",
    )
    # Dark caulk seams drawn into one continuous deck slab.
    for i, y in enumerate((-0.60, -0.30, 0.00, 0.30, 0.60)):
        deck.visual(
            Box((2.42, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.001)),
            material=seam,
            name=f"plank_seam_{i}",
        )
    deck.visual(
        _annulus_mesh(0.76, 0.56, 0.030, "deck_traverse_ring"),
        material=worn_iron,
        name="traverse_ring",
    )
    for i in range(16):
        angle = 2.0 * math.pi * i / 16.0
        deck.visual(
            Cylinder(radius=0.022, length=0.014),
            origin=Origin(xyz=(0.66 * math.cos(angle), 0.66 * math.sin(angle), 0.036)),
            material=black_iron,
            name=f"ring_bolt_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.60, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black_iron,
        name="turntable_plate",
    )
    carriage.visual(
        Box((0.92, 0.46, 0.100)),
        origin=Origin(xyz=(0.02, 0.0, 0.098)),
        material=oak,
        name="wooden_bed",
    )
    carriage.visual(
        Box((0.22, 0.62, 0.060)),
        origin=Origin(xyz=(-0.30, 0.0, 0.178)),
        material=black_iron,
        name="rear_crossband",
    )
    carriage.visual(
        Box((0.22, 0.62, 0.060)),
        origin=Origin(xyz=(0.34, 0.0, 0.178)),
        material=black_iron,
        name="front_crossband",
    )
    for side, y in enumerate((-0.34, 0.34)):
        carriage.visual(
            Box((0.76, 0.090, 0.105)),
            origin=Origin(xyz=(0.03, y, 0.215)),
            material=oak,
            name=f"cheek_rail_{side}",
        )
        for fore_aft, x in enumerate((-0.105, 0.105)):
            carriage.visual(
                Box((0.072, 0.090, 0.360)),
                origin=Origin(xyz=(x, y, 0.430)),
                material=oak,
                name=f"trunnion_post_{side}_{fore_aft}",
            )
        carriage.visual(
            Box((0.180, 0.092, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.473)),
            material=black_iron,
            name=f"lower_bearing_{side}",
        )
        carriage.visual(
            Box((0.235, 0.092, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.630)),
            material=black_iron,
            name=f"trunnion_cap_{side}",
        )
    carriage.visual(
        Cylinder(radius=0.055, length=0.76),
        origin=Origin(xyz=(-0.28, 0.0, 0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="rear_axle_bar",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=0.76),
        origin=Origin(xyz=(0.36, 0.0, 0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_iron,
        name="front_axle_bar",
    )

    barrel = model.part("barrel")
    barrel.visual(_bronze_barrel_mesh(), material=bronze, name="barrel_tube")
    barrel.visual(
        Cylinder(radius=0.052, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="trunnion_pin",
    )
    barrel.visual(
        Cylinder(radius=0.074, length=0.050),
        origin=Origin(xyz=(0.0, -0.430, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="trunnion_knob_0",
    )
    barrel.visual(
        Cylinder(radius=0.074, length=0.050),
        origin=Origin(xyz=(0.0, 0.430, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="trunnion_knob_1",
    )
    barrel.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(1.285, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_bore,
        name="muzzle_shadow",
    )

    model.articulation(
        "ring_to_carriage",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.6, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        # The barrel extends along local +X, so -Y makes positive q elevate.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.35, lower=-0.12, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    traverse = object_model.get_articulation("ring_to_carriage")
    elevation = object_model.get_articulation("carriage_to_barrel")

    ctx.check(
        "carriage traverses on vertical ring joint",
        traverse.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in traverse.axis) == (0.0, 0.0, 1.0),
        details=f"type={traverse.articulation_type}, axis={traverse.axis}",
    )
    ctx.check(
        "barrel elevates about transverse trunnions",
        elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in elevation.axis) == (0.0, -1.0, 0.0),
        details=f"type={elevation.articulation_type}, axis={elevation.axis}",
    )
    ctx.expect_contact(
        deck,
        carriage,
        elem_a="traverse_ring",
        elem_b="turntable_plate",
        contact_tol=0.002,
        name="turntable rests on deck ring",
    )
    ctx.expect_within(
        barrel,
        carriage,
        axes="y",
        inner_elem="trunnion_pin",
        outer_elem="turntable_plate",
        margin=0.20,
        name="trunnion is captured between carriage cheeks",
    )

    rest_pos = ctx.part_world_position(barrel)
    with ctx.pose({elevation: 0.45}):
        raised_pos = ctx.part_world_aabb(barrel)
    ctx.check(
        "positive elevation raises the muzzle",
        rest_pos is not None and raised_pos is not None and raised_pos[1][2] > 0.92,
        details=f"rest_origin={rest_pos}, raised_aabb={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
