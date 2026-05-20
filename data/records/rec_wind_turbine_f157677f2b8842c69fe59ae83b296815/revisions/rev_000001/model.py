from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(i * math.tau / segments), radius * math.sin(i * math.tau / segments))
        for i in range(segments)
    ]


def _tower_geometry() -> LatheGeometry:
    """One-piece exterior for the concrete pad, rolled steel tower, and flange rings."""
    return LatheGeometry(
        [
            (0.00, 0.00),
            (1.45, 0.00),
            (1.45, 0.22),
            (0.62, 0.22),
            (0.62, 0.34),
            (0.47, 0.34),
            (0.42, 0.80),
            (0.31, 5.30),
            (0.25, 9.70),
            (0.41, 9.70),
            (0.41, 10.05),
            (0.00, 10.05),
        ],
        segments=88,
    )


def _nacelle_body_geometry() -> ExtrudeWithHolesGeometry:
    """Rounded molded cover with an axial generator/shaft clearance bore."""
    outer = rounded_rect_profile(0.84, 0.90, 0.105, corner_segments=8)
    bore = _circle_profile(0.18, 40)
    return ExtrudeWithHolesGeometry(outer, [bore], 2.00, center=True)


def _airfoil_profile(
    chord: float,
    thickness: float,
    twist: float,
    *,
    samples: int = 9,
) -> list[tuple[float, float]]:
    """Airfoil-like closed loop in local XY; loft span is local Z."""
    upper: list[tuple[float, float]] = []
    lower: list[tuple[float, float]] = []
    for i in range(samples):
        p = i / (samples - 1)
        y = (p - 0.50) * chord
        half_t = 0.50 * thickness * (math.sin(math.pi * p) ** 0.62) * (1.0 - 0.10 * p)
        camber = 0.07 * thickness * (1.0 - (2.0 * p - 1.0) ** 2)
        upper.append((camber + half_t, y))
        lower.append((camber - half_t, y))

    loop = upper + list(reversed(lower))
    c = math.cos(twist)
    s = math.sin(twist)
    return [(x * c - y * s, x * s + y * c) for x, y in loop]


def _blade_solid() -> cq.Workplane:
    """Single fixed-pitch blade with thick molded root and thinner twisted tip."""
    sections = [
        (0.00, 0.55, 0.120, math.radians(15.0)),
        (0.45, 0.50, 0.105, math.radians(12.0)),
        (1.65, 0.38, 0.074, math.radians(7.0)),
        (3.25, 0.27, 0.048, math.radians(3.5)),
        (4.85, 0.15, 0.026, math.radians(1.0)),
    ]

    wp = cq.Workplane("XY").polyline(_airfoil_profile(*sections[0][1:])).close()
    last_z = sections[0][0]
    for z_pos, chord, thick, twist in sections[1:]:
        wp = wp.workplane(offset=z_pos - last_z).polyline(_airfoil_profile(chord, thick, twist)).close()
        last_z = z_pos

    blade = wp.loft(combine=True).translate((0.0, 0.0, 0.58))

    root_tube = cq.Workplane("XY").circle(0.185).extrude(0.64).translate((0.0, 0.0, 0.12))
    root_flange = cq.Workplane("XY").circle(0.275).extrude(0.085).translate((0.0, 0.0, 0.235))

    bolts = cq.Workplane("XY")
    for index in range(8):
        angle = index * math.tau / 8.0
        x = 0.218 * math.cos(angle)
        y = 0.218 * math.sin(angle)
        bolt = cq.Workplane("XY").circle(0.022).extrude(0.045).translate((x, y, 0.305))
        bolts = bolts.union(bolt)

    return root_tube.union(root_flange).union(bolts).union(blade)


def _rotor_shell() -> cq.Workplane:
    """Low part-count rotor: molded hub, integral blade roots, and bolted root pads."""
    hub = cq.Workplane("XY").sphere(0.300)
    front_cap = cq.Workplane("YZ").circle(0.245).extrude(0.26).translate((-0.23, 0.0, 0.0))
    shell = hub.union(front_cap)

    blade = _blade_solid()
    for angle in (0.0, 120.0, 240.0):
        shell = shell.union(blade.rotate((0, 0, 0), (1, 0, 0), angle))
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_wind_turbine")

    tower_white = model.material("powder_coated_steel", rgba=(0.86, 0.88, 0.84, 1.0))
    concrete = model.material("poured_concrete", rgba=(0.55, 0.55, 0.51, 1.0))
    nacelle_white = model.material("molded_off_white", rgba=(0.92, 0.93, 0.88, 1.0))
    dark_seal = model.material("dark_seal", rgba=(0.08, 0.09, 0.09, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    blade_white = model.material("gelcoat_blade_white", rgba=(0.96, 0.97, 0.93, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_geometry(_tower_geometry(), "tower_shell"),
        material=tower_white,
        name="tower_shell",
    )
    # A slightly exposed concrete skirt gives the tower a realistic fixed support path.
    tower.visual(
        Cylinder(radius=1.46, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=concrete,
        name="concrete_skirt",
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_nacelle_body_geometry(), "nacelle_body"),
        origin=Origin(xyz=(0.55, 0.0, 0.66), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nacelle_white,
        name="nacelle_body",
    )
    nacelle.visual(
        Cylinder(radius=0.43, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=galvanized,
        name="yaw_bearing",
    )
    # Four stamped bearing-clamp pads leave an open, inspectable shaft clearance
    # instead of an expensive one-piece machined annulus.
    nacelle.visual(
        Box((0.18, 0.58, 0.09)),
        origin=Origin(xyz=(-0.535, 0.0, 0.850)),
        material=galvanized,
        name="bearing_top",
    )
    nacelle.visual(
        Box((0.18, 0.58, 0.09)),
        origin=Origin(xyz=(-0.535, 0.0, 0.505)),
        material=galvanized,
        name="bearing_bottom",
    )
    nacelle.visual(
        Box((0.18, 0.09, 0.36)),
        origin=Origin(xyz=(-0.535, 0.190, 0.660)),
        material=galvanized,
        name="bearing_side_0",
    )
    nacelle.visual(
        Box((0.18, 0.09, 0.36)),
        origin=Origin(xyz=(-0.535, -0.190, 0.660)),
        material=galvanized,
        name="bearing_side_1",
    )
    nacelle.visual(
        Box((0.018, 0.68, 0.38)),
        origin=Origin(xyz=(0.44, -0.456, 0.68)),
        material=dark_seal,
        name="access_panel_seam",
    )
    nacelle.visual(
        Box((0.055, 0.96, 0.09)),
        origin=Origin(xyz=(-0.27, 0.0, 0.315)),
        material=galvanized,
        name="clamp_band",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_cadquery(_rotor_shell(), "rotor_shell", tolerance=0.006, angular_tolerance=0.18),
        material=blade_white,
        name="rotor_shell",
    )
    rotor.visual(
        Cylinder(radius=0.110, length=0.32),
        origin=Origin(xyz=(0.33, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="rotor_shaft",
    )

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.FIXED,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 10.05)),
    )
    model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(-0.95, 0.0, 0.66)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="yaw_bearing",
        negative_elem="tower_shell",
        name="yaw bearing seats on tower flange",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="x",
        min_overlap=0.12,
        elem_a="rotor_shaft",
        elem_b="bearing_top",
        name="rotor shaft overlaps clamp depth",
    )
    ctx.expect_gap(
        nacelle,
        rotor,
        axis="z",
        min_gap=0.020,
        positive_elem="bearing_top",
        negative_elem="rotor_shaft",
        name="top clamp clears rotating shaft",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rotor_shaft",
        negative_elem="bearing_bottom",
        name="bottom bearing pad supports shaft",
    )
    ctx.expect_gap(
        nacelle,
        rotor,
        axis="y",
        min_gap=0.020,
        positive_elem="bearing_side_0",
        negative_elem="rotor_shaft",
        name="side clamp 0 clears rotating shaft",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="y",
        min_gap=0.020,
        positive_elem="rotor_shaft",
        negative_elem="bearing_side_1",
        name="side clamp 1 clears rotating shaft",
    )
    ctx.expect_gap(
        nacelle,
        rotor,
        axis="x",
        min_gap=0.004,
        positive_elem="nacelle_body",
        negative_elem="rotor_shaft",
        name="shaft stops before molded nacelle body",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="rotor_shell")
    with ctx.pose({spin: math.pi / 3.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="rotor_shell")
        ctx.expect_overlap(
            rotor,
            nacelle,
            axes="x",
            min_overlap=0.12,
            elem_a="rotor_shaft",
            elem_b="bearing_top",
            name="shaft remains captured while rotor spins",
        )

    ctx.check(
        "continuous rotor changes blade envelope",
        rest_aabb is not None
        and turned_aabb is not None
        and abs(rest_aabb[1][2] - turned_aabb[1][2]) > 0.20,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
