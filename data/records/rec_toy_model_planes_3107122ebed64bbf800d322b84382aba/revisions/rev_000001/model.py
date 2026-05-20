from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _superellipse_section(
    x: float,
    half_width: float,
    half_height: float,
    *,
    center_z: float = 0.0,
    exponent: float = 2.4,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    """A rounded rectangular/elliptic fuselage section in the YZ plane."""
    pts: list[tuple[float, float, float]] = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        ct = math.cos(t)
        st = math.sin(t)
        y = half_width * math.copysign(abs(ct) ** (2.0 / exponent), ct)
        z = center_z + half_height * math.copysign(abs(st) ** (2.0 / exponent), st)
        pts.append((x, y, z))
    return pts


def _airfoil_section(
    y: float,
    chord: float,
    thickness: float,
    *,
    x_center: float,
    z_center: float,
    samples: int = 24,
) -> list[tuple[float, float, float]]:
    """Closed wing section loop in the XZ plane, lofted along span (Y)."""
    pts: list[tuple[float, float, float]] = []
    for i in range(samples):
        t = 2.0 * math.pi * i / samples
        ct = math.cos(t)
        st = math.sin(t)
        # A blunt, thick, service-toy airfoil: rounded leading edge and durable
        # trailing edge, without duplicate cusp vertices that can break lofting.
        x = x_center + 0.5 * chord * math.copysign(abs(ct) ** 0.82, ct)
        camber = 0.010 * thickness * (1.0 - ct)
        z = z_center + camber + 0.5 * thickness * math.copysign(abs(st) ** 0.86, st)
        pts.append((x, y, z))
    return pts


def _loft_mesh(
    sections: list[list[tuple[float, float, float]]],
    *,
    cap: bool = True,
) -> MeshGeometry:
    """Lightweight mesh loft for corresponding closed loops."""
    geom = MeshGeometry()
    if len(sections) < 2:
        return geom
    count = len(sections[0])
    for section in sections:
        if len(section) != count:
            raise ValueError("all loft sections must have matching point counts")
        for x, y, z in section:
            geom.add_vertex(x, y, z)
    for s in range(len(sections) - 1):
        base = s * count
        next_base = (s + 1) * count
        for i in range(count):
            j = (i + 1) % count
            geom.add_face(base + i, next_base + i, next_base + j)
            geom.add_face(base + i, next_base + j, base + j)
    if cap:
        first_center = geom.add_vertex(
            sum(p[0] for p in sections[0]) / count,
            sum(p[1] for p in sections[0]) / count,
            sum(p[2] for p in sections[0]) / count,
        )
        for i in range(count):
            geom.add_face(first_center, i, (i + 1) % count)
        last_offset = (len(sections) - 1) * count
        last_center = geom.add_vertex(
            sum(p[0] for p in sections[-1]) / count,
            sum(p[1] for p in sections[-1]) / count,
            sum(p[2] for p in sections[-1]) / count,
        )
        for i in range(count):
            geom.add_face(last_center, last_offset + (i + 1) % count, last_offset + i)
    return geom


def _managed_mesh(geometry, name: str):
    try:
        return mesh_from_geometry(geometry, name)
    except Exception as exc:  # pragma: no cover - compile-time authoring diagnostic
        raise RuntimeError(f"mesh generation failed for {name}") from exc


def _propeller_blades_mesh() -> MeshGeometry:
    """Two broad, thick toy prop blades in a single connected mesh."""
    geom = MeshGeometry()
    sections = (
        (-0.165, 0.045, 0.008, -0.010),
        (-0.030, 0.080, 0.014, 0.016),
        (0.030, 0.080, 0.014, 0.016),
        (0.165, 0.045, 0.008, -0.010),
    )
    for z, chord, thick, pitch in sections:
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            # Chordwise offset in X gives visible blade pitch/twist.
            x = 0.040 + sx * thick * 0.5 + sy * pitch
            y = sy * chord * 0.5
            geom.add_vertex(x, y, z)
    for s in range(len(sections) - 1):
        base = s * 4
        next_base = (s + 1) * 4
        for i in range(4):
            j = (i + 1) % 4
            geom.add_face(base + i, next_base + i, next_base + j)
            geom.add_face(base + i, next_base + j, base + j)
    geom.add_face(0, 1, 2)
    geom.add_face(0, 2, 3)
    last = (len(sections) - 1) * 4
    geom.add_face(last + 0, last + 2, last + 1)
    geom.add_face(last + 0, last + 3, last + 2)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_toy_plane")

    safety_orange = model.material("safety_orange", rgba=(0.93, 0.42, 0.08, 1.0))
    graphite = model.material("graphite_composite", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    black_oxide = model.material("black_oxide_hardware", rgba=(0.02, 0.025, 0.03, 1.0))
    service_gray = model.material("service_gray", rgba=(0.56, 0.59, 0.58, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.63, 0.64, 0.61, 1.0))
    white_marking = model.material("white_service_marking", rgba=(0.92, 0.91, 0.84, 1.0))

    airframe = model.part("airframe")

    fuselage = _loft_mesh(
        [
            _superellipse_section(-0.50, 0.028, 0.030, center_z=0.000, exponent=2.0),
            _superellipse_section(-0.38, 0.060, 0.052, center_z=0.002),
            _superellipse_section(-0.18, 0.095, 0.078, center_z=0.004),
            _superellipse_section(0.08, 0.118, 0.092, center_z=0.004),
            _superellipse_section(0.30, 0.092, 0.076, center_z=0.000),
            _superellipse_section(0.43, 0.045, 0.045, center_z=-0.003, exponent=2.0),
        ],
    )
    airframe.visual(
        _managed_mesh(fuselage, "fuselage_shell"),
        material=safety_orange,
        name="fuselage_shell",
    )

    # Durable one-piece main wing with a thick rounded toy airfoil and embedded root.
    main_wing = _loft_mesh(
        [
            _airfoil_section(-0.62, 0.22, 0.030, x_center=0.00, z_center=0.010),
            _airfoil_section(-0.18, 0.31, 0.045, x_center=0.01, z_center=0.018),
            _airfoil_section(0.18, 0.31, 0.045, x_center=0.01, z_center=0.018),
            _airfoil_section(0.62, 0.22, 0.030, x_center=0.00, z_center=0.010),
        ],
    )
    airframe.visual(
        _managed_mesh(main_wing, "main_wing"),
        material=safety_orange,
        name="main_wing",
    )

    tail_plane = _loft_mesh(
        [
            _airfoil_section(-0.28, 0.155, 0.020, x_center=-0.425, z_center=0.030, samples=10),
            _airfoil_section(0.28, 0.155, 0.020, x_center=-0.425, z_center=0.030, samples=10),
        ],
    )
    airframe.visual(
        _managed_mesh(tail_plane, "tail_plane"),
        material=safety_orange,
        name="tail_plane",
    )
    airframe.visual(
        Box((0.035, 0.018, 0.155)),
        origin=Origin(xyz=(-0.465, 0.0, 0.105), rpy=(0.0, 0.0, 0.0)),
        material=safety_orange,
        name="vertical_fin",
    )

    # Wear pieces and serviceable covers are slightly seated into the shell.
    airframe.visual(
        Box((0.36, 0.165, 0.014)),
        origin=Origin(xyz=(0.02, 0.0, 0.102)),
        material=service_gray,
        name="top_service_hatch",
    )
    airframe.visual(
        Cylinder(radius=0.010, length=0.36),
        origin=Origin(xyz=(0.02, -0.089, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="hatch_piano_hinge",
    )
    airframe.visual(
        Box((0.030, 0.015, 0.012)),
        origin=Origin(xyz=(0.15, 0.090, 0.114)),
        material=black_oxide,
        name="hatch_latch_0",
    )
    airframe.visual(
        Box((0.030, 0.015, 0.012)),
        origin=Origin(xyz=(-0.11, 0.090, 0.114)),
        material=black_oxide,
        name="hatch_latch_1",
    )
    airframe.visual(
        Box((0.42, 0.035, 0.010)),
        origin=Origin(xyz=(0.02, 0.138, 0.008)),
        material=brushed_metal,
        name="wing_root_wear_strip_0",
    )
    airframe.visual(
        Box((0.42, 0.035, 0.010)),
        origin=Origin(xyz=(0.02, -0.138, 0.008)),
        material=brushed_metal,
        name="wing_root_wear_strip_1",
    )
    airframe.visual(
        Box((0.22, 0.020, 0.018)),
        origin=Origin(xyz=(-0.36, 0.0, -0.040)),
        material=brushed_metal,
        name="tail_boom_skid_plate",
    )
    airframe.visual(
        Box((0.24, 0.022, 0.014)),
        origin=Origin(xyz=(0.23, 0.0, -0.076)),
        material=dark_rubber,
        name="nose_belly_bumper",
    )
    airframe.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.455, 0.0, -0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="prop_bearing_collar",
    )
    airframe.visual(
        Cylinder(radius=0.069, length=0.012),
        origin=Origin(xyz=(0.432, 0.0, -0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="nose_reinforcement_ring",
    )

    # Side access grilles: replaceable plates with recessed dark slot inserts.
    airframe.visual(
        Box((0.150, 0.006, 0.080)),
        origin=Origin(xyz=(0.175, 0.119, 0.000)),
        material=service_gray,
        name="side_grille_0",
    )
    airframe.visual(
        Box((0.150, 0.006, 0.080)),
        origin=Origin(xyz=(0.175, -0.119, 0.000)),
        material=service_gray,
        name="side_grille_1",
    )
    for side, y in (("0", 0.123), ("1", -0.123)):
        for i, z in enumerate((-0.020, 0.000, 0.020)):
            airframe.visual(
                Box((0.105, 0.004, 0.006)),
                origin=Origin(xyz=(0.175, y, z)),
                material=black_oxide,
                name=f"grille_slot_{side}_{i}",
            )

    # Broad alignment stripes are separate replaceable decals/plates, seated into the wing surface.
    airframe.visual(
        Box((0.030, 0.88, 0.004)),
        origin=Origin(xyz=(0.080, 0.0, 0.050)),
        material=white_marking,
        name="wing_alignment_stripe",
    )
    airframe.visual(
        Box((0.018, 0.50, 0.004)),
        origin=Origin(xyz=(-0.070, 0.0, 0.046)),
        material=white_marking,
        name="wing_service_stripe",
    )

    # Stand hinge lugs are part of the airframe; their inner faces touch the stand barrel.
    airframe.visual(
        Box((0.075, 0.040, 0.070)),
        origin=Origin(xyz=(-0.055, 0.202, -0.105)),
        material=black_oxide,
        name="stand_hinge_lug_0",
    )
    airframe.visual(
        Box((0.075, 0.040, 0.070)),
        origin=Origin(xyz=(-0.055, -0.202, -0.105)),
        material=black_oxide,
        name="stand_hinge_lug_1",
    )
    airframe.visual(
        Box((0.110, 0.450, 0.018)),
        origin=Origin(xyz=(-0.055, 0.0, -0.070)),
        material=brushed_metal,
        name="stand_mount_doubler",
    )

    propeller = model.part("propeller")
    propeller.visual(
        _managed_mesh(_propeller_blades_mesh(), "two_blade_prop"),
        material=graphite,
        name="two_blade_prop",
    )
    propeller.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="prop_rear_boss",
    )

    stand = model.part("stand")
    stand_tube = tube_from_spline_points(
        [
            (0.0, -0.145, 0.0),
            (-0.030, -0.145, -0.075),
            (-0.130, -0.145, -0.255),
            (-0.130, 0.145, -0.255),
            (-0.030, 0.145, -0.075),
            (0.0, 0.145, 0.0),
        ],
        radius=0.014,
        samples_per_segment=2,
        spline="catmull_rom",
        radial_segments=14,
        cap_ends=True,
    )
    stand.visual(
        _managed_mesh(stand_tube, "folding_stand_tube"),
        material=black_oxide,
        name="folding_stand_tube",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.364),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="stand_hinge_barrel",
    )
    stand.visual(
        Box((0.050, 0.400, 0.028)),
        origin=Origin(xyz=(-0.142, 0.0, -0.265)),
        material=dark_rubber,
        name="replaceable_stand_foot",
    )

    model.articulation(
        "airframe_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=propeller,
        origin=Origin(xyz=(0.475, 0.0, -0.003)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=40.0),
    )
    model.articulation(
        "airframe_to_stand",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=stand,
        origin=Origin(xyz=(-0.055, 0.0, -0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    propeller = object_model.get_part("propeller")
    stand = object_model.get_part("stand")
    prop_joint = object_model.get_articulation("airframe_to_propeller")
    stand_joint = object_model.get_articulation("airframe_to_stand")

    ctx.expect_contact(
        airframe,
        propeller,
        elem_a="prop_bearing_collar",
        elem_b="prop_rear_boss",
        contact_tol=0.001,
        name="propeller boss seats on the nose bearing collar",
    )
    ctx.expect_contact(
        airframe,
        stand,
        elem_a="stand_hinge_lug_0",
        elem_b="stand_hinge_barrel",
        contact_tol=0.001,
        name="stand barrel is captured by first hinge lug",
    )
    ctx.expect_contact(
        airframe,
        stand,
        elem_a="stand_hinge_lug_1",
        elem_b="stand_hinge_barrel",
        contact_tol=0.001,
        name="stand barrel is captured by second hinge lug",
    )

    prop_rest = ctx.part_world_position(propeller)
    with ctx.pose({prop_joint: math.pi / 2.0}):
        prop_turned = ctx.part_world_position(propeller)
        ctx.expect_contact(
            airframe,
            propeller,
            elem_a="prop_bearing_collar",
            elem_b="prop_rear_boss",
            contact_tol=0.001,
            name="propeller stays seated while rotating",
        )
    ctx.check(
        "propeller rotates about fixed nose shaft",
        prop_rest is not None
        and prop_turned is not None
        and abs(prop_rest[0] - prop_turned[0]) < 1e-6
        and abs(prop_rest[1] - prop_turned[1]) < 1e-6
        and abs(prop_rest[2] - prop_turned[2]) < 1e-6,
        details=f"rest={prop_rest}, turned={prop_turned}",
    )

    rest_aabb = ctx.part_world_aabb(stand)
    with ctx.pose({stand_joint: 0.85}):
        folded_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "folding stand swings upward toward the belly",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > rest_aabb[0][2] + 0.030,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
