from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _frustum_z(z0: float, z1: float, r0: float, r1: float, *, segments: int = 64) -> MeshGeometry:
    """Closed conical-frustum mesh aligned with the local Z axis."""
    geom = MeshGeometry()
    lower = []
    upper = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        lower.append(geom.add_vertex(r0 * ca, r0 * sa, z0))
        upper.append(geom.add_vertex(r1 * ca, r1 * sa, z1))

    c0 = geom.add_vertex(0.0, 0.0, z0)
    c1 = geom.add_vertex(0.0, 0.0, z1)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(lower[i], lower[j], upper[j])
        geom.add_face(lower[i], upper[j], upper[i])
        geom.add_face(c0, lower[i], lower[j])
        geom.add_face(c1, upper[j], upper[i])
    return geom


def _frustum_x(x0: float, x1: float, r0: float, r1: float, *, segments: int = 48) -> MeshGeometry:
    """Closed conical-frustum mesh aligned with the local X axis."""
    geom = MeshGeometry()
    rear = []
    front = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        rear.append(geom.add_vertex(x0, r0 * ca, r0 * sa))
        front.append(geom.add_vertex(x1, r1 * ca, r1 * sa))

    c0 = geom.add_vertex(x0, 0.0, 0.0)
    c1 = geom.add_vertex(x1, 0.0, 0.0)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(rear[i], front[j], rear[j])
        geom.add_face(rear[i], front[i], front[j])
        geom.add_face(c0, rear[j], rear[i])
        geom.add_face(c1, front[i], front[j])
    return geom


def _blade_geometry() -> MeshGeometry:
    """A tapered, twisted wind-turbine blade rooted on +Y and rotating about X."""
    geom = MeshGeometry()
    section_indices: list[list[int]] = []
    # (span fraction, radius from hub center, chord, airfoil thickness, pitch angle)
    sections = (
        (0.00, 0.28, 0.62, 0.16, math.radians(22.0)),
        (0.18, 0.90, 0.50, 0.12, math.radians(16.0)),
        (0.45, 1.85, 0.36, 0.085, math.radians(10.0)),
        (0.72, 2.90, 0.25, 0.060, math.radians(6.0)),
        (1.00, 4.10, 0.14, 0.035, math.radians(3.0)),
    )
    # Airfoil-like loop in the local cross-section: X is thickness, Z is chord.
    profile = (
        (0.00, 0.50),
        (0.45, 0.30),
        (0.55, 0.05),
        (0.35, -0.38),
        (0.00, -0.50),
        (-0.22, -0.30),
        (-0.30, 0.05),
        (-0.18, 0.34),
    )
    for span, y, chord, thickness, pitch in sections:
        indices = []
        sweep_z = -0.16 * span * span
        prebend_x = 0.035 * math.sin(math.pi * span)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        for px, pz in profile:
            x = px * thickness
            z = pz * chord
            # Twist the airfoil section about the radial Y axis.
            xr = x * cp + z * sp
            zr = -x * sp + z * cp
            indices.append(geom.add_vertex(prebend_x + xr, y, sweep_z + zr))
        section_indices.append(indices)

    n = len(profile)
    for a, b in zip(section_indices[:-1], section_indices[1:]):
        for i in range(n):
            j = (i + 1) % n
            geom.add_face(a[i], a[j], b[j])
            geom.add_face(a[i], b[j], b[i])

    root = section_indices[0]
    tip = section_indices[-1]
    root_center = geom.add_vertex(0.0, sections[0][1], 0.0)
    tip_center = geom.add_vertex(0.035, sections[-1][1], -0.16)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(root_center, root[i], root[j])
        geom.add_face(tip_center, tip[j], tip[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine")

    white = Material("warm_white", rgba=(0.86, 0.88, 0.84, 1.0))
    blade_white = Material("blade_white", rgba=(0.93, 0.94, 0.90, 1.0))
    tower_gray = Material("painted_tower_gray", rgba=(0.70, 0.72, 0.70, 1.0))
    concrete = Material("matte_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    dark = Material("dark_rubber_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    service_blue = Material("muted_service_blue", rgba=(0.18, 0.32, 0.44, 1.0))

    tower = model.part("tower")
    tower.visual(
        Cylinder(radius=1.45, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=concrete,
        name="foundation",
    )
    tower.visual(
        mesh_from_geometry(_frustum_z(0.30, 11.88, 0.38, 0.20), "tapered_tower"),
        material=tower_gray,
        name="tapered_tower",
    )
    tower.visual(
        Cylinder(radius=0.42, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 11.94)),
        material=dark,
        name="top_flange",
    )

    nacelle = model.part("nacelle")
    nacelle_body_shape = (
        cq.Workplane("XY")
        .box(1.90, 0.85, 0.70)
        .edges()
        .fillet(0.10)
    )
    nacelle.visual(
        mesh_from_cadquery(nacelle_body_shape, "rounded_nacelle", tolerance=0.002),
        origin=Origin(xyz=(0.25, 0.0, 0.55)),
        material=white,
        name="body_shell",
    )
    nacelle.visual(
        Cylinder(radius=0.40, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark,
        name="yaw_bearing",
    )
    nacelle.visual(
        Cylinder(radius=0.27, length=0.26),
        origin=Origin(xyz=(1.33, 0.0, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="shaft_collar",
    )
    nacelle.visual(
        Box((0.42, 0.012, 0.24)),
        origin=Origin(xyz=(0.15, 0.431, 0.55)),
        material=service_blue,
        name="side_panel_0",
    )
    nacelle.visual(
        Box((0.42, 0.012, 0.24)),
        origin=Origin(xyz=(0.15, -0.431, 0.55)),
        material=service_blue,
        name="side_panel_1",
    )
    nacelle.visual(
        Box((0.018, 0.42, 0.26)),
        origin=Origin(xyz=(-0.709, 0.0, 0.55)),
        material=dark,
        name="rear_grille",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.34, length=0.46),
        origin=Origin(xyz=(0.23, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="hub_shell",
    )
    rotor.visual(
        mesh_from_geometry(_frustum_x(0.36, 0.78, 0.34, 0.025), "front_spinner"),
        material=white,
        name="spinner",
    )
    rotor.visual(
        Cylinder(radius=0.115, length=0.22),
        origin=Origin(xyz=(0.11, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="root_bolt_ring",
    )
    blade_mesh = mesh_from_geometry(_blade_geometry(), "turbine_blade")
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=blade_white,
            name=f"blade_{i}",
        )

    yaw = model.articulation(
        "tower_to_nacelle",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 12.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=0.25),
    )
    yaw.meta["description"] = "Nacelle yaw about the vertical tower axis."

    spin = model.articulation(
        "nacelle_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(1.46, 0.0, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=12.0),
    )
    spin.meta["description"] = "Compact front rotor spinning about the main shaft axis."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("tower_to_nacelle")
    spin = object_model.get_articulation("nacelle_to_rotor")

    ctx.check(
        "nacelle yaw is continuous about vertical",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "rotor spin is continuous about horizontal shaft",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_gap(
        nacelle,
        tower,
        axis="z",
        positive_elem="yaw_bearing",
        negative_elem="top_flange",
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw bearing sits on tower top flange",
    )
    ctx.expect_gap(
        rotor,
        nacelle,
        axis="x",
        positive_elem="hub_shell",
        negative_elem="shaft_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotor hub is compact against nacelle collar",
    )
    ctx.expect_overlap(
        rotor,
        nacelle,
        axes="yz",
        elem_a="hub_shell",
        elem_b="shaft_collar",
        min_overlap=0.40,
        name="rotor hub aligns with main shaft collar",
    )

    rest_rotor_pos = ctx.part_world_position(rotor)
    with ctx.pose({yaw: 0.75}):
        yawed_rotor_pos = ctx.part_world_position(rotor)
    ctx.check(
        "yaw joint swings rotor around tower axis",
        rest_rotor_pos is not None
        and yawed_rotor_pos is not None
        and abs(yawed_rotor_pos[1] - rest_rotor_pos[1]) > 0.8,
        details=f"rest={rest_rotor_pos}, yawed={yawed_rotor_pos}",
    )

    rest_blade = ctx.part_element_world_aabb(rotor, elem="blade_0")
    with ctx.pose({spin: math.pi / 2.0}):
        spun_blade = ctx.part_element_world_aabb(rotor, elem="blade_0")
    if rest_blade is None or spun_blade is None:
        ctx.fail("rotor spin moves a tracked blade", "Missing blade_0 AABB.")
    else:
        rest_center_z = 0.5 * (rest_blade[0][2] + rest_blade[1][2])
        spun_center_z = 0.5 * (spun_blade[0][2] + spun_blade[1][2])
        ctx.check(
            "rotor spin moves a tracked blade",
            abs(spun_center_z - rest_center_z) > 1.5,
            details=f"rest_blade={rest_blade}, spun_blade={spun_blade}",
        )

    return ctx.report()


object_model = build_object_model()
