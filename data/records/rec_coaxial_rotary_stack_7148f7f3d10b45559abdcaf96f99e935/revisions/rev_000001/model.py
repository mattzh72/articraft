from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cylinder(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _annulus(outer_radius: float, inner_radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z0))
    )


def _socket_head(radius: float, height: float, socket_radius: float) -> cq.Workplane:
    head = _cylinder(radius, height)
    socket = (
        cq.Workplane("XY")
        .polygon(6, socket_radius * 2.0)
        .extrude(height * 0.58)
        .translate((0.0, 0.0, height * 0.50))
    )
    return head.cut(socket)


def _revolved_profile(points: list[tuple[float, float]]) -> cq.Workplane:
    return cq.Workplane("XZ").polyline(points).close().revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def _base_casting() -> cq.Workplane:
    shape = _cylinder(0.225, 0.022)
    shape = shape.union(_annulus(0.205, 0.105, 0.018, 0.022))
    shape = shape.union(_cylinder(0.072, 0.023, 0.022))
    shape = shape.union(_annulus(0.124, 0.086, 0.008, 0.036))
    return shape


def _platter_casting() -> cq.Workplane:
    return _revolved_profile(
        [
            (0.001, -0.002),
            (0.158, -0.002),
            (0.158, 0.002),
            (0.182, 0.002),
            (0.182, 0.024),
            (0.188, 0.024),
            (0.188, 0.031),
            (0.095, 0.031),
            (0.095, 0.046),
            (0.001, 0.046),
        ]
    )


def _intermediate_ring() -> cq.Workplane:
    shape = _annulus(0.148, 0.064, 0.012)
    shape = shape.union(_annulus(0.128, 0.073, 0.046, 0.010))
    shape = shape.union(_annulus(0.116, 0.046, 0.014, 0.056))
    shape = shape.union(_annulus(0.140, 0.126, 0.006, 0.026))
    return shape


def _fixture_cup() -> cq.Workplane:
    cup = _cylinder(0.052, 0.068, 0.012)
    cavity = _cylinder(0.035, 0.058, 0.026)
    cup = cup.cut(cavity)
    cup = cup.union(_annulus(0.073, 0.040, 0.012, 0.000))
    cup = cup.union(_annulus(0.057, 0.035, 0.009, 0.074))
    for angle in (0.0, 120.0, 240.0):
        lug = cq.Workplane("XY").box(0.026, 0.014, 0.026).translate((0.059, 0.0, 0.052))
        lug = lug.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        cup = cup.union(lug)
    return cup

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_ring_turntable")

    cast_iron = model.material("warm_cast_iron", rgba=(0.43, 0.45, 0.43, 1.0))
    dark_iron = model.material("dark_recessed_metal", rgba=(0.10, 0.11, 0.11, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    oil_bronze = model.material("oiled_bronze_bearing", rgba=(0.66, 0.48, 0.23, 1.0))
    fixture_blue = model.material("blued_fixture_steel", rgba=(0.18, 0.25, 0.34, 1.0))
    screw_black = model.material("black_oxide_screws", rgba=(0.02, 0.02, 0.018, 1.0))

    base_screw = mesh_from_cadquery(_socket_head(0.0075, 0.0055, 0.0027), "base_socket_head")
    stage_screw = mesh_from_cadquery(_socket_head(0.0058, 0.0050, 0.0022), "stage_socket_head")
    cup_screw = mesh_from_cadquery(_socket_head(0.0044, 0.0042, 0.0018), "cup_socket_head")

    base = model.part("base_housing")
    base.visual(
        mesh_from_cadquery(_base_casting(), "base_casting"),
        material=cast_iron,
        name="stepped_base",
    )
    base.visual(
        mesh_from_cadquery(_annulus(0.126, 0.085, 0.003, 0.044), "base_bearing_race"),
        material=oil_bronze,
        name="lower_bearing_race",
    )
    for idx, (x_sign, y_sign) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        base.visual(
            Box((0.075, 0.050, 0.012)),
            origin=Origin(xyz=(0.150 * x_sign, 0.125 * y_sign, 0.006)),
            material=cast_iron,
            name=f"mount_foot_{idx}",
        )
    for idx in range(12):
        angle = 2.0 * math.pi * idx / 12.0
        base.visual(
            base_screw,
            origin=Origin(xyz=(0.188 * math.cos(angle), 0.188 * math.sin(angle), 0.039)),
            material=screw_black,
            name=f"base_screw_{idx}",
        )

    platter = model.part("low_platter")
    platter.visual(
        Cylinder(radius=0.182, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=machined_steel,
        name="platter_disk",
    )
    platter.visual(
        Cylinder(radius=0.188, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=machined_steel,
        name="outer_flange_lip",
    )
    platter.visual(
        Cylinder(radius=0.095, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0385)),
        material=machined_steel,
        name="raised_center_cap",
    )
    platter.visual(
        Cylinder(radius=0.158, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_steel,
        name="lower_labyrinth_skirt",
    )
    platter.visual(
        mesh_from_cadquery(_annulus(0.177, 0.152, 0.0024, 0.031), "platter_outer_machined_face"),
        material=dark_iron,
        name="outer_graduation_band",
    )
    platter.visual(
        mesh_from_cadquery(_annulus(0.096, 0.063, 0.003, 0.046), "platter_upper_bearing_race"),
        material=oil_bronze,
        name="middle_bearing_race",
    )
    for idx in range(10):
        angle = 2.0 * math.pi * idx / 10.0 + math.radians(9.0)
        platter.visual(
            stage_screw,
            origin=Origin(xyz=(0.140 * math.cos(angle), 0.140 * math.sin(angle), 0.0280)),
            material=screw_black,
            name=("platter_screw_0" if idx == 0 else f"platter_screw_{idx}"),
        )

    ring = model.part("intermediate_ring")
    ring.visual(
        mesh_from_cadquery(_intermediate_ring(), "intermediate_ring_casting"),
        material=cast_iron,
        name="thick_ring_body",
    )
    ring.visual(
        mesh_from_cadquery(_annulus(0.122, 0.103, 0.0025, 0.068), "ring_top_machined_face"),
        material=dark_iron,
        name="ring_cap_interface",
    )
    ring.visual(
        mesh_from_cadquery(_annulus(0.069, 0.048, 0.0030, 0.069), "ring_upper_bearing_race"),
        material=oil_bronze,
        name="upper_bearing_race",
    )
    for idx in range(8):
        angle = 2.0 * math.pi * idx / 8.0 + math.radians(22.5)
        ring.visual(
            stage_screw,
            origin=Origin(xyz=(0.105 * math.cos(angle), 0.105 * math.sin(angle), 0.0665)),
            material=screw_black,
            name=("ring_screw_0" if idx == 0 else f"ring_screw_{idx}"),
        )

    cup = model.part("fixture_cup")
    cup.visual(
        mesh_from_cadquery(_fixture_cup(), "fixture_cup_body"),
        material=fixture_blue,
        name="hollow_fixture_cup",
    )
    cup.visual(
        mesh_from_cadquery(_annulus(0.056, 0.037, 0.0024, 0.083), "cup_bright_rim"),
        material=machined_steel,
        name="machined_cup_rim",
    )
    for idx in range(6):
        angle = 2.0 * math.pi * idx / 6.0
        cup.visual(
            cup_screw,
            origin=Origin(xyz=(0.058 * math.cos(angle), 0.058 * math.sin(angle), 0.0095)),
            material=screw_black,
            name=f"cup_flange_screw_{idx}",
        )

    full_turn = MotionLimits(effort=30.0, velocity=2.0, lower=-math.pi, upper=math.pi)
    model.articulation(
        "base_to_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    model.articulation(
        "platter_to_ring",
        ArticulationType.REVOLUTE,
        parent=platter,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    model.articulation(
        "ring_to_cup",
        ArticulationType.REVOLUTE,
        parent=ring,
        child=cup,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=full_turn,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    platter = object_model.get_part("low_platter")
    ring = object_model.get_part("intermediate_ring")
    cup = object_model.get_part("fixture_cup")
    base_to_platter = object_model.get_articulation("base_to_platter")
    platter_to_ring = object_model.get_articulation("platter_to_ring")
    ring_to_cup = object_model.get_articulation("ring_to_cup")

    for joint in (base_to_platter, platter_to_ring, ring_to_cup):
        ctx.check(
            f"{joint.name} is a vertical rotary axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_origin_distance(platter, base, axes="xy", max_dist=0.001, name="platter is centered on base axis")
    ctx.expect_origin_distance(ring, base, axes="xy", max_dist=0.001, name="ring is centered on base axis")
    ctx.expect_origin_distance(cup, base, axes="xy", max_dist=0.001, name="fixture cup is centered on base axis")

    ctx.expect_gap(platter, base, axis="z", max_gap=0.001, max_penetration=0.0, name="platter sits on lower bearing level")
    ctx.expect_gap(ring, platter, axis="z", max_gap=0.001, max_penetration=0.0, name="ring sits on middle bearing level")
    ctx.expect_gap(cup, ring, axis="z", max_gap=0.001, max_penetration=0.0, name="cup sits on upper bearing level")

    ctx.expect_overlap(
        platter,
        base,
        axes="xy",
        elem_a="lower_labyrinth_skirt",
        elem_b="lower_bearing_race",
        min_overlap=0.10,
        name="lower platter has broad bearing overlap",
    )
    ctx.expect_overlap(
        ring,
        platter,
        axes="xy",
        elem_a="thick_ring_body",
        elem_b="middle_bearing_race",
        min_overlap=0.09,
        name="intermediate ring is supported by middle race",
    )
    ctx.expect_overlap(
        cup,
        ring,
        axes="xy",
        elem_a="hollow_fixture_cup",
        elem_b="upper_bearing_race",
        min_overlap=0.08,
        name="fixture cup is supported by upper race",
    )

    with ctx.pose({base_to_platter: 1.4, platter_to_ring: -0.9, ring_to_cup: 2.2}):
        ctx.expect_gap(
            ring,
            platter,
            axis="z",
            min_gap=0.010,
            positive_elem="thick_ring_body",
            negative_elem="platter_screw_0",
            name="ring clears platter cap screws while indexed",
        )
        ctx.expect_gap(
            cup,
            ring,
            axis="z",
            min_gap=0.0003,
            positive_elem="hollow_fixture_cup",
            negative_elem="ring_screw_0",
            name="cup flange clears ring cap screws while indexed",
        )
        ctx.expect_origin_distance(cup, base, axes="xy", max_dist=0.001, name="rotated cup stays on common centerline")

    return ctx.report()


object_model = build_object_model()
