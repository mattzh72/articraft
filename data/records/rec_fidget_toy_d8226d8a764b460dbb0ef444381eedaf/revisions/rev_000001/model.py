from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_plate_xy(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """Flat annular plate centered on the XY plane with thickness along Z."""
    outer = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(thickness / 2.0, both=True)
    )
    inner = _cylinder_z(thickness * 3.0, inner_radius, (0.0, 0.0, 0.0))
    return outer.cut(inner)


def _annular_plate_xz(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """Flat annular plate centered on the XZ plane with thickness along Y."""
    outer = (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .extrude(thickness / 2.0, both=True)
    )
    inner = _cylinder_y(thickness * 3.0, inner_radius, (0.0, 0.0, 0.0))
    return outer.cut(inner)


def _cylinder_x(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    """CadQuery cylinder whose axis is the global X axis."""
    x, y, z = center
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x, 0.0, 0.0))
    )


def _cylinder_y(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    """CadQuery cylinder whose axis is the global Y axis."""
    x, y, z = center
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _cylinder_z(length: float, radius: float, center: tuple[float, float, float]) -> cq.Workplane:
    """CadQuery cylinder whose axis is the global Z axis."""
    x, y, z = center
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, 0.0, z))
    )


def _build_outer_ring() -> cq.Workplane:
    # A palm-sized, wide annular body with two radial bearing bores for the
    # inner gimbal axle pins.
    body = _annular_plate_xz(outer_radius=0.070, inner_radius=0.052, thickness=0.016)

    for side in (-1.0, 1.0):
        # Inward-facing bearing collars are fused into the inner rim.
        body = body.union(_cylinder_x(0.025, 0.012, (side * 0.055, 0.0, 0.0)))
        # A snug radial bore receives the gimbal pin; the authored pin has a
        # tiny intentional interference fit so the bearing reads captured.
        body = body.cut(_cylinder_x(0.060, 0.0039, (side * 0.058, 0.0, 0.0)))

    return body


def _build_gimbal_ring() -> cq.Workplane:
    # The inner ring is narrower and has its own perpendicular rotor bearings
    # at the top and bottom of the ring.
    body = _annular_plate_xz(outer_radius=0.041, inner_radius=0.0315, thickness=0.011)

    for side in (-1.0, 1.0):
        body = body.union(_cylinder_z(0.018, 0.0085, (0.0, 0.0, side * 0.0335)))

    # A single through-bore forms both top and bottom rotor bearing holes.
    body = body.cut(_cylinder_z(0.090, 0.0028, (0.0, 0.0, 0.0)))
    return body


def _build_weighted_disc() -> cq.Workplane:
    # A small flywheel: heavy outer rim, central hub, and three spokes.  It is
    # centered on the XY plane and spins about Z.
    rim = _annular_plate_xy(outer_radius=0.0275, inner_radius=0.0185, thickness=0.007)
    hub = _annular_plate_xy(outer_radius=0.0090, inner_radius=0.0028, thickness=0.008)
    body = rim.union(hub)

    radial_length = 0.017
    spoke_width = 0.0045
    for angle in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .box(radial_length, spoke_width, 0.006)
            .translate((0.0090 + radial_length / 2.0, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        body = body.union(spoke)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gyro_fidget_ring")

    black_anodized = model.material("black_anodized", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.62, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.95, 0.62, 0.18, 1.0))

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(_build_outer_ring(), "outer_ring_body", tolerance=0.0006),
        material=black_anodized,
        name="outer_body",
    )

    gimbal_ring = model.part("gimbal_ring")
    gimbal_ring.visual(
        mesh_from_cadquery(_build_gimbal_ring(), "gimbal_ring_body", tolerance=0.0006),
        material=satin_steel,
        name="gimbal_body",
    )
    for idx, side in enumerate((-1.0, 1.0)):
        gimbal_ring.visual(
            Cylinder(radius=0.0042, length=0.024),
            origin=Origin(xyz=(side * 0.0505, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"outer_axle_{idx}",
        )

    weighted_disc = model.part("weighted_disc")
    weighted_disc.visual(
        mesh_from_cadquery(_build_weighted_disc(), "weighted_disc_flywheel", tolerance=0.0005),
        material=warm_brass,
        name="flywheel",
    )
    weighted_disc.visual(
        Cylinder(radius=0.0031, length=0.073),
        origin=Origin(),
        material=dark_steel,
        name="spin_axle",
    )

    model.articulation(
        "outer_to_gimbal",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=gimbal_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "gimbal_to_disc",
        ArticulationType.CONTINUOUS,
        parent=gimbal_ring,
        child=weighted_disc,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_ring")
    gimbal = object_model.get_part("gimbal_ring")
    disc = object_model.get_part("weighted_disc")
    gimbal_joint = object_model.get_articulation("outer_to_gimbal")
    disc_joint = object_model.get_articulation("gimbal_to_disc")

    for pin_name in ("outer_axle_0", "outer_axle_1"):
        ctx.allow_overlap(
            gimbal,
            outer,
            elem_a=pin_name,
            elem_b="outer_body",
            reason="Each gimbal axle pin is intentionally captured in a snug outer-ring bearing bore.",
        )
    ctx.allow_overlap(
        disc,
        gimbal,
        elem_a="spin_axle",
        elem_b="gimbal_body",
        reason="The spinning axle is intentionally seated with a slight interference fit in the gimbal bearing bores.",
    )

    ctx.check(
        "gimbal axle is left-right",
        tuple(round(v, 4) for v in gimbal_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={gimbal_joint.axis}",
    )
    ctx.check(
        "disc axle is perpendicular",
        tuple(round(v, 4) for v in disc_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={disc_joint.axis}",
    )

    ctx.expect_origin_distance(
        outer,
        gimbal,
        axes="xyz",
        max_dist=0.001,
        name="gimbal pivots at outer-ring center",
    )
    ctx.expect_origin_distance(
        gimbal,
        disc,
        axes="xyz",
        max_dist=0.001,
        name="disc spins at gimbal center",
    )
    ctx.expect_within(
        gimbal,
        outer,
        axes="xz",
        margin=0.001,
        name="gimbal ring fits inside outer ring",
    )
    ctx.expect_within(
        disc,
        gimbal,
        axes="xz",
        margin=0.001,
        inner_elem="spin_axle",
        outer_elem="gimbal_body",
        name="spin axle stays inside gimbal bearings",
    )
    for pin_name in ("outer_axle_0", "outer_axle_1"):
        ctx.expect_overlap(
            gimbal,
            outer,
            axes="x",
            min_overlap=0.010,
            elem_a=pin_name,
            elem_b="outer_body",
            name=f"{pin_name} remains inserted in outer bearing",
        )
    ctx.expect_overlap(
        disc,
        gimbal,
        axes="z",
        min_overlap=0.055,
        elem_a="spin_axle",
        elem_b="gimbal_body",
        name="spin axle remains inserted in gimbal bearings",
    )

    rest_center = ctx.part_world_position(gimbal)
    with ctx.pose({gimbal_joint: math.pi / 2.0, disc_joint: 0.8}):
        ctx.expect_origin_distance(
            outer,
            gimbal,
            axes="xyz",
            max_dist=0.001,
            name="gimbal rotates about fixed outer axle",
        )
        moved_center = ctx.part_world_position(gimbal)
    ctx.check(
        "gimbal rotation keeps center captured",
        rest_center is not None
        and moved_center is not None
        and max(abs(a - b) for a, b in zip(rest_center, moved_center)) < 0.001,
        details=f"rest={rest_center}, moved={moved_center}",
    )

    return ctx.report()


object_model = build_object_model()
