from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_disc(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Flat cylindrical ring with a real central opening."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _tray_shell() -> cq.Workplane:
    """One connected rotating tray with a tall outer rail and radial bins."""
    bearing_plate_h = 0.008
    floor_h = 0.016
    floor_top = bearing_plate_h + floor_h

    bearing_plate = _annular_disc(0.078, 0.026, bearing_plate_h)
    floor = cq.Workplane("XY").circle(0.202).extrude(floor_h).translate((0, 0, bearing_plate_h))
    outer_rail = _annular_disc(0.214, 0.195, 0.080).translate((0, 0, floor_top))
    inner_rim = _annular_disc(0.080, 0.064, 0.034).translate((0, 0, floor_top))

    tray = bearing_plate.union(floor).union(outer_rail).union(inner_rim)

    # Low radial dividers make the perimeter read as individual shallow spice pockets.
    rib_inner = 0.073
    rib_outer = 0.200
    rib_length = rib_outer - rib_inner
    rib_center_r = (rib_outer + rib_inner) / 2.0
    rib_h = 0.034
    for index in range(12):
        angle = 360.0 * index / 12.0
        rib = (
            cq.Workplane("XY")
            .box(rib_length, 0.008, rib_h)
            .translate((rib_center_r, 0.0, floor_top + rib_h / 2.0))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        tray = tray.union(rib)

    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spice_carousel_lazy_susan")

    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.052, 0.047, 1.0))
    warm_wood = model.material("warm_bamboo", rgba=(0.72, 0.46, 0.23, 1.0))
    end_grain = model.material("end_grain", rgba=(0.54, 0.32, 0.14, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.71, 0.68, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.90, 0.89, 0.84, 1.0))
    brass = model.material("brass_badge", rgba=(0.88, 0.62, 0.20, 1.0))

    base = model.part("bearing_base")
    base.visual(
        mesh_from_cadquery(_annular_disc(0.175, 0.100, 0.018), "low_base_ring"),
        material=dark_plastic,
        name="low_base_ring",
    )
    for index in range(4):
        angle = index * math.pi / 2.0
        base.visual(
            Box((0.170, 0.030, 0.014)),
            origin=Origin(
                xyz=(0.085 * math.cos(angle), 0.085 * math.sin(angle), 0.012),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_plastic,
            name=f"base_spoke_{index}",
        )

    base.visual(
        Cylinder(radius=0.058, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=brushed_steel,
        name="central_pedestal",
    )
    base.visual(
        mesh_from_cadquery(_annular_disc(0.074, 0.026, 0.008), "bearing_race"),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=brushed_steel,
        name="bearing_race",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=brushed_steel,
        name="center_boss",
    )
    for index in range(12):
        angle = 2.0 * math.pi * index / 12.0
        base.visual(
            Sphere(radius=0.004),
            origin=Origin(
                xyz=(0.052 * math.cos(angle), 0.052 * math.sin(angle), 0.056)
            ),
            material=bearing_steel,
            name=f"bearing_ball_{index}",
        )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shell(), "compartment_tray", tolerance=0.0008),
        material=warm_wood,
        name="compartment_tray",
    )
    # A small proud brass maker's badge makes the continuous rotation visually legible.
    tray.visual(
        Box((0.006, 0.045, 0.030)),
        origin=Origin(xyz=(0.215, 0.0, 0.066)),
        material=brass,
        name="index_badge",
    )
    tray.visual(
        mesh_from_cadquery(_annular_disc(0.199, 0.083, 0.002), "compartment_floor_tint"),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=end_grain,
        name="compartment_floor_tint",
    )

    model.articulation(
        "base_to_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("bearing_base")
    tray = object_model.get_part("tray")
    spin = object_model.get_articulation("base_to_tray")

    ctx.check(
        "tray uses continuous vertical bearing rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_origin_distance(
        tray,
        base,
        axes="xy",
        max_dist=0.001,
        name="tray remains centered on bearing",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        name="rotating tray is seated on bearing balls",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        min_overlap=0.040,
        name="bearing footprint sits under tray center",
    )

    badge_rest = ctx.part_element_world_aabb(tray, elem="index_badge")
    with ctx.pose({spin: math.pi / 2.0}):
        badge_quarter = ctx.part_element_world_aabb(tray, elem="index_badge")

    if badge_rest is None or badge_quarter is None:
        ctx.fail("badge demonstrates tray rotation", "missing index_badge AABB")
    else:
        rest_center = (
            (badge_rest[0][0] + badge_rest[1][0]) / 2.0,
            (badge_rest[0][1] + badge_rest[1][1]) / 2.0,
        )
        quarter_center = (
            (badge_quarter[0][0] + badge_quarter[1][0]) / 2.0,
            (badge_quarter[0][1] + badge_quarter[1][1]) / 2.0,
        )
        ctx.check(
            "badge demonstrates tray rotation",
            rest_center[0] > 0.20 and abs(rest_center[1]) < 0.03
            and quarter_center[1] > 0.20 and abs(quarter_center[0]) < 0.03,
            details=f"rest={rest_center}, quarter_turn={quarter_center}",
        )

    return ctx.report()


object_model = build_object_model()
