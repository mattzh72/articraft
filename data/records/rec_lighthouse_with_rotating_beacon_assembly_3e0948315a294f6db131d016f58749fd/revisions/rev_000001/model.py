from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _ring(outer_radius: float, inner_radius: float, z_min: float, height: float) -> cq.Workplane:
    """Annular cast ring extruded upward from z_min."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _solid_cylinder(radius: float, z_min: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z_min))


def _box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    yaw_deg: float = 0.0,
) -> cq.Workplane:
    wp = cq.Workplane("XY").box(*size).translate(center)
    if yaw_deg:
        wp = wp.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), yaw_deg)
    return wp


def _lantern_iron_frame() -> cq.Workplane:
    """Connected cast-iron lantern cage, base, roof, door jamb, and shaft."""
    frame = _solid_cylinder(0.86, 0.00, 0.12)
    for piece in (
        _solid_cylinder(0.76, 0.12, 0.34),
        _ring(0.91, 0.74, 0.46, 0.10),
        _ring(0.88, 0.80, 1.22, 0.055),
        _ring(0.91, 0.74, 1.98, 0.12),
        _solid_cylinder(0.045, 0.38, 1.78),  # central shaft for the rotating beacon
    ):
        frame = frame.union(piece)

    # Faceted cylindrical wall mullions: 12 slim radial cast posts.
    for i in range(12):
        angle = 15.0 + i * 30.0
        post = _box((0.070, 0.048, 1.58), (0.855, 0.0, 1.265), yaw_deg=angle)
        frame = frame.union(post)

    # Narrow service-door frame on the +Y side, proud of the cylindrical cage.
    for piece in (
        _box((0.050, 0.105, 1.50), (-0.265, 0.865, 1.255)),
        _box((0.050, 0.105, 1.50), (0.265, 0.865, 1.255)),
        _box((0.58, 0.105, 0.060), (0.0, 0.865, 0.535)),
        _box((0.58, 0.105, 0.065), (0.0, 0.865, 1.985)),
        _box((0.055, 0.040, 0.135), (-0.235, 0.905, 0.86)),
        _box((0.055, 0.040, 0.135), (-0.235, 0.905, 1.30)),
        _box((0.055, 0.040, 0.135), (-0.235, 0.905, 1.74)),
    ):
        frame = frame.union(piece)

    # Low conical roof, vent, and ball finial typical of cast-iron lantern rooms.
    roof = (
        cq.Workplane("XY")
        .circle(0.93)
        .workplane(offset=0.42)
        .circle(0.23)
        .loft()
        .translate((0.0, 0.0, 2.07))
    )
    for piece in (
        roof,
        _solid_cylinder(0.21, 2.47, 0.18),
        _solid_cylinder(0.12, 2.65, 0.12),
        cq.Workplane("XY").sphere(0.085).translate((0.0, 0.0, 2.80)),
    ):
        frame = frame.union(piece)

    return frame


def _beacon_carriage() -> cq.Workplane:
    """Rotating beacon yoke with a hollow sleeve around the fixed central shaft."""
    carriage = _ring(0.078, 0.052, 1.145, 0.23)

    # Two opposed arms begin outside the shaft clearance and carry the lenses.
    for sign in (-1.0, 1.0):
        arm = _box((0.40, 0.050, 0.050), (sign * 0.265, 0.0, 1.260))
        carriage = carriage.union(arm)
        carriage = carriage.union(_box((0.055, 0.30, 0.045), (sign * 0.465, 0.0, 1.260)))
        carriage = carriage.union(_box((0.050, 0.055, 0.31), (sign * 0.465, 0.0, 1.260)))

    # A small brass lamp cradle carried by the rotating sleeve, clear of the shaft.
    for y in (-0.105, 0.105):
        carriage = carriage.union(_box((0.035, 0.055, 0.24), (0.0, y, 1.260)))

    return carriage


def _door_frame() -> cq.Workplane:
    """Narrow hinged service door; local origin is the lower hinge axis."""
    height = 1.30
    width = 0.42
    thick = 0.030
    door = cq.Workplane("XY").circle(0.018).extrude(1.14).translate((0.0, 0.0, 0.08))
    for piece in (
        _box((0.040, thick, height), (0.030, 0.0, height / 2.0)),
        _box((0.040, thick, height), (width - 0.020, 0.0, height / 2.0)),
        _box((width, thick, 0.040), (width / 2.0, 0.0, 0.020)),
        _box((width, thick, 0.040), (width / 2.0, 0.0, height - 0.020)),
        _box((0.030, thick, 1.22), (0.21, 0.0, 0.65)),
    ):
        door = door.union(piece)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_lighthouse_lantern")

    cast_iron = model.material("cast_iron", rgba=(0.025, 0.029, 0.030, 1.0))
    dark_iron = model.material("blackened_hinge", rgba=(0.006, 0.007, 0.008, 1.0))
    glass = model.material("pale_green_glass", rgba=(0.62, 0.88, 0.86, 0.34))
    amber_glass = model.material("amber_lens_glass", rgba=(1.0, 0.62, 0.10, 0.58))
    brass = model.material("warm_brass", rgba=(0.85, 0.58, 0.25, 1.0))
    light = model.material("warm_lantern_light", rgba=(1.0, 0.88, 0.42, 0.82))

    lantern = model.part("lantern")
    lantern.visual(
        mesh_from_cadquery(_lantern_iron_frame(), "lantern_iron_frame", tolerance=0.002),
        material=cast_iron,
        name="iron_frame",
    )

    # Transparent panes fill all cylindrical wall bays except the hinged service-door bay.
    pane_width = 0.365
    pane_height = 1.36
    pane_radius = 0.835
    for i in range(12):
        theta = math.radians(i * 30.0)
        if abs((i * 30.0) - 90.0) < 1e-6:
            continue
        lantern.visual(
            Box((pane_width, 0.014, pane_height)),
            origin=Origin(
                xyz=(pane_radius * math.cos(theta), pane_radius * math.sin(theta), 1.255),
                rpy=(0.0, 0.0, theta + math.pi / 2.0),
            ),
            material=glass,
            name=f"glass_pane_{i}",
        )

    beacon = model.part("beacon")
    beacon.visual(
        mesh_from_cadquery(_beacon_carriage(), "beacon_carriage", tolerance=0.0015),
        material=brass,
        name="carriage",
    )
    for name, x in (("lens_0", 0.495), ("lens_1", -0.495)):
        beacon.visual(
            Cylinder(radius=0.145, length=0.050),
            origin=Origin(xyz=(x, 0.0, 1.260), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=amber_glass,
            name=name,
        )
    for name, y in (("lamp_globe_0", -0.105), ("lamp_globe_1", 0.105)):
        beacon.visual(
            Sphere(0.040),
            origin=Origin(xyz=(0.0, y, 1.260)),
            material=light,
            name=name,
        )

    service_door = model.part("service_door")
    service_door.visual(
        mesh_from_cadquery(_door_frame(), "service_door_frame", tolerance=0.001),
        material=dark_iron,
        name="door_frame",
    )
    service_door.visual(
        Box((0.300, 0.010, 0.88)),
        origin=Origin(xyz=(0.220, 0.0, 0.68)),
        material=glass,
        name="door_glass",
    )
    service_door.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.350, 0.035, 0.665), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="handle_stem",
    )
    service_door.visual(
        Sphere(0.025),
        origin=Origin(xyz=(0.350, 0.072, 0.665)),
        material=brass,
        name="handle_knob",
    )

    model.articulation(
        "shaft_to_beacon",
        ArticulationType.CONTINUOUS,
        parent=lantern,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6),
    )
    model.articulation(
        "wall_to_door",
        ArticulationType.REVOLUTE,
        parent=lantern,
        child=service_door,
        origin=Origin(xyz=(-0.235, 0.960, 0.590)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beacon = object_model.get_part("beacon")
    lantern = object_model.get_part("lantern")
    service_door = object_model.get_part("service_door")
    beacon_joint = object_model.get_articulation("shaft_to_beacon")
    door_joint = object_model.get_articulation("wall_to_door")

    ctx.expect_within(
        beacon,
        lantern,
        axes="xy",
        margin=0.01,
        name="rotating beacon carriage stays within the lantern room footprint",
    )
    ctx.expect_gap(
        service_door,
        lantern,
        axis="y",
        min_gap=0.004,
        max_gap=0.080,
        positive_elem="door_frame",
        negative_elem="iron_frame",
        name="closed service door sits just proud of the outer wall frame",
    )
    ctx.expect_overlap(
        service_door,
        lantern,
        axes="z",
        min_overlap=1.0,
        elem_a="door_frame",
        elem_b="iron_frame",
        name="service door is mounted through the lantern wall height",
    )

    lens0_rest = ctx.part_element_world_aabb(beacon, elem="lens_0")
    with ctx.pose({beacon_joint: math.pi / 2.0}):
        lens0_quarter = ctx.part_element_world_aabb(beacon, elem="lens_0")
    if lens0_rest is not None and lens0_quarter is not None:
        c0 = (
            (lens0_rest[0][0] + lens0_rest[1][0]) / 2.0,
            (lens0_rest[0][1] + lens0_rest[1][1]) / 2.0,
        )
        c1 = (
            (lens0_quarter[0][0] + lens0_quarter[1][0]) / 2.0,
            (lens0_quarter[0][1] + lens0_quarter[1][1]) / 2.0,
        )
        ctx.check(
            "beacon lens orbits the central vertical shaft",
            c0[0] > 0.40 and c1[1] > 0.40 and abs(c1[0]) < 0.18,
            details=f"rest_center={c0}, quarter_turn_center={c1}",
        )
    else:
        ctx.fail("beacon lens orbits the central vertical shaft", "missing lens AABB")

    door_rest = ctx.part_world_aabb(service_door)
    with ctx.pose({door_joint: 1.20}):
        door_open = ctx.part_world_aabb(service_door)
    if door_rest is not None and door_open is not None:
        ctx.check(
            "service door swings outward on its vertical wall hinge",
            door_open[1][1] > door_rest[1][1] + 0.20,
            details=f"closed_aabb={door_rest}, open_aabb={door_open}",
        )
    else:
        ctx.fail("service door swings outward on its vertical wall hinge", "missing door AABB")

    return ctx.report()


object_model = build_object_model()
