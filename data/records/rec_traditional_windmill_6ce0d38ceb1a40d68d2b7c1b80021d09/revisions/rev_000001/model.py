from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _frustum_profile(base_radius: float, top_radius: float, z0: float, z1: float):
    """Closed lathe profile for a truncated circular masonry tower section."""
    return [(0.0, z0), (base_radius, z0), (top_radius, z1), (0.0, z1)]


def _tower_radius_at(z: float) -> float:
    # Main stone body linearly tapers from about 1.36 m to 0.92 m.
    z0, z1 = 0.28, 5.05
    t = max(0.0, min(1.0, (z - z0) / (z1 - z0)))
    return 1.36 + (0.92 - 1.36) * t


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_tower_windmill")

    stone = model.material("warm_limestone", rgba=(0.68, 0.62, 0.50, 1.0))
    stone_shadow = model.material("aged_stone_courses", rgba=(0.48, 0.44, 0.36, 1.0))
    dark_recess = model.material("dark_recessed_openings", rgba=(0.05, 0.045, 0.04, 1.0))
    oak = model.material("weathered_oak", rgba=(0.45, 0.25, 0.12, 1.0))
    pale_oak = model.material("pale_sail_lattice", rgba=(0.62, 0.42, 0.23, 1.0))
    thatch = model.material("dry_thatch_roof", rgba=(0.53, 0.41, 0.22, 1.0))
    canvas = model.material("cream_canvas_sails", rgba=(0.88, 0.84, 0.70, 1.0))
    iron = model.material("dark_wrought_iron", rgba=(0.08, 0.075, 0.07, 1.0))

    tower = model.part("tower")

    # Heavy circular stone base and the tapered masonry tower.
    tower.visual(
        Cylinder(radius=1.55, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=stone_shadow,
        name="circular_foundation",
    )
    tower.visual(
        mesh_from_geometry(
            LatheGeometry(_frustum_profile(1.36, 0.92, 0.26, 5.10), segments=72),
            "tapered_stone_tower",
        ),
        material=stone,
        name="tapered_stone_tower",
    )

    # Slightly proud stone courses give the tower scale and historic texture.
    for i, z in enumerate((0.62, 1.08, 1.55, 2.02, 2.50, 2.98, 3.47, 3.96, 4.46)):
        r = _tower_radius_at(z) + 0.035
        tower.visual(
            mesh_from_geometry(
                LatheGeometry(_frustum_profile(r, r - 0.010, z, z + 0.045), segments=72),
                f"stone_course_{i}",
            ),
            material=stone_shadow,
            name=f"stone_course_{i}",
        )

    # Front arched door and small deep-set windows.
    tower.visual(
        Box((0.55, 0.055, 0.92)),
        origin=Origin(xyz=(0.0, -1.395, 0.74)),
        material=dark_recess,
        name="arched_door_panel",
    )
    tower.visual(
        Cylinder(radius=0.275, length=0.060),
        origin=Origin(xyz=(0.0, -1.398, 1.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_recess,
        name="arched_door_top",
    )
    tower.visual(
        Box((0.70, 0.075, 0.080)),
        origin=Origin(xyz=(0.0, -1.425, 0.285)),
        material=oak,
        name="door_threshold",
    )
    for i, z in enumerate((1.95, 3.70)):
        r = _tower_radius_at(z)
        tower.visual(
            Box((0.38, 0.045, 0.50)),
            origin=Origin(xyz=(0.0, -r - 0.020, z)),
            material=dark_recess,
            name=f"front_window_{i}",
        )
        tower.visual(
            Box((0.52, 0.055, 0.060)),
            origin=Origin(xyz=(0.0, -r - 0.035, z - 0.28)),
            material=oak,
            name=f"window_sill_{i}",
        )
        tower.visual(
            Box((0.055, 0.055, 0.58)),
            origin=Origin(xyz=(-0.24, -r - 0.035, z)),
            material=oak,
            name=f"window_jamb_{i}_0",
        )
        tower.visual(
            Box((0.055, 0.055, 0.58)),
            origin=Origin(xyz=(0.24, -r - 0.035, z)),
            material=oak,
            name=f"window_jamb_{i}_1",
        )

    # A timber stage around the tower, with posts and a circular rail.
    tower.visual(
        Cylinder(radius=1.34, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 3.00)),
        material=oak,
        name="balcony_deck",
    )
    tower.visual(
        mesh_from_geometry(TorusGeometry(1.32, 0.035, radial_segments=12, tubular_segments=72), "balcony_top_rail"),
        origin=Origin(xyz=(0.0, 0.0, 3.68)),
        material=oak,
        name="balcony_top_rail",
    )
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        tower.visual(
            Cylinder(radius=0.026, length=0.64),
            origin=Origin(xyz=(1.32 * math.cos(a), 1.32 * math.sin(a), 3.35)),
            material=oak,
            name=f"balcony_post_{i}",
        )

    # Wooden cap, thatched cone roof, and fixed bearing where the windshaft exits.
    tower.visual(
        Box((1.62, 1.42, 0.58)),
        origin=Origin(xyz=(0.0, -0.06, 5.20)),
        material=oak,
        name="timber_cap_body",
    )
    tower.visual(
        mesh_from_geometry(ConeGeometry(1.16, 1.05, radial_segments=72), "conical_thatch_cap"),
        origin=Origin(xyz=(0.0, -0.06, 5.86)),
        material=thatch,
        name="conical_thatch_cap",
    )
    tower.visual(
        Cylinder(radius=0.43, length=0.22),
        origin=Origin(xyz=(0.0, -1.03, 5.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="bearing",
    )
    tower.visual(
        Box((0.72, 0.16, 0.36)),
        origin=Origin(xyz=(0.0, -1.00, 4.79)),
        material=oak,
        name="bearing_bracket",
    )

    rotor = model.part("sail_rotor")
    rotor.visual(
        Cylinder(radius=0.16, length=0.70),
        origin=Origin(xyz=(0.0, -0.24, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    rotor.visual(
        Cylinder(radius=0.34, length=0.36),
        origin=Origin(xyz=(0.0, -0.64, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=oak,
        name="hub",
    )
    rotor.visual(
        mesh_from_geometry(ConeGeometry(0.31, 0.36, radial_segments=48), "hub_nose"),
        origin=Origin(xyz=(0.0, -1.02, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_nose",
    )

    # Four full sails: heavy stocks, canvas fields, battens, and diagonal braces.
    spar_length = 3.15
    canvas_length = 1.78
    canvas_width = 0.64
    for i, beta in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        d = (math.sin(beta), 0.0, math.cos(beta))
        p = (math.cos(beta), 0.0, -math.sin(beta))

        def loc(along: float, across: float, y: float):
            return (
                d[0] * along + p[0] * across,
                y,
                d[2] * along + p[2] * across,
            )

        rotor.visual(
            Box((0.18, 0.13, spar_length)),
            origin=Origin(xyz=loc(spar_length / 2.0, 0.0, -0.78), rpy=(0.0, beta, 0.0)),
            material=pale_oak,
            name=f"blade_{i}_spar",
        )
        rotor.visual(
            Box((canvas_width, 0.030, canvas_length)),
            origin=Origin(xyz=loc(1.95, canvas_width / 2.0 - 0.035, -0.82), rpy=(0.0, beta, 0.0)),
            material=canvas,
            name=f"blade_{i}_canvas",
        )
        for j, along in enumerate((1.18, 1.60, 2.02, 2.44)):
            rotor.visual(
                Box((canvas_width + 0.10, 0.055, 0.050)),
                origin=Origin(xyz=loc(along, canvas_width / 2.0 - 0.035, -0.85), rpy=(0.0, beta, 0.0)),
                material=pale_oak,
                name=f"blade_{i}_batten_{j}",
            )
        rotor.visual(
            Box((0.055, 0.060, 2.05)),
            origin=Origin(xyz=loc(1.92, 0.27, -0.86), rpy=(0.0, beta + 0.28, 0.0)),
            material=pale_oak,
            name=f"blade_{i}_diagonal",
        )

    model.articulation(
        "tower_to_sail_rotor",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=rotor,
        origin=Origin(xyz=(0.0, -1.14, 5.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    rotor = object_model.get_part("sail_rotor")
    spin = object_model.get_articulation("tower_to_sail_rotor")

    ctx.allow_overlap(
        rotor,
        tower,
        elem_a="axle",
        elem_b="bearing",
        reason="The iron windshaft is intentionally captured inside the fixed bearing sleeve.",
    )
    ctx.expect_gap(
        tower,
        rotor,
        axis="y",
        positive_elem="bearing",
        negative_elem="axle",
        max_gap=0.004,
        max_penetration=0.12,
        name="windshaft is retained inside the fixed bearing",
    )
    ctx.expect_overlap(
        rotor,
        tower,
        axes="xz",
        elem_a="axle",
        elem_b="bearing",
        min_overlap=0.20,
        name="rotating shaft is centered in the bearing face",
    )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0_spar")
    with ctx.pose({spin: math.pi / 2.0}):
        quarter_aabb = ctx.part_element_world_aabb(rotor, elem="blade_0_spar")

    if rest_aabb is not None and quarter_aabb is not None:
        rest_size = (
            rest_aabb[1][0] - rest_aabb[0][0],
            rest_aabb[1][2] - rest_aabb[0][2],
        )
        quarter_size = (
            quarter_aabb[1][0] - quarter_aabb[0][0],
            quarter_aabb[1][2] - quarter_aabb[0][2],
        )
        ctx.check(
            "sail rotation carries the upper blade into a side blade",
            rest_size[1] > 2.8 and quarter_size[0] > 2.8 and quarter_size[1] < 0.35,
            details=f"rest_xz={rest_size}, quarter_xz={quarter_size}",
        )
    else:
        ctx.fail("sail rotation carries the upper blade into a side blade", "missing blade AABB")

    return ctx.report()


object_model = build_object_model()
