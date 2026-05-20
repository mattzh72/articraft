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
    Sphere,
    LatheGeometry,
    mesh_from_geometry,
    TestContext,
    TestReport,
)


def _hollow_cylinder_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    """Closed annular cylinder centered on local Z."""
    geom = MeshGeometry()
    z0 = -length / 2.0
    z1 = length / 2.0
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca = math.cos(a)
        sa = math.sin(a)
        geom.add_vertex(outer_radius * ca, outer_radius * sa, z0)
        geom.add_vertex(outer_radius * ca, outer_radius * sa, z1)
        geom.add_vertex(inner_radius * ca, inner_radius * sa, z0)
        geom.add_vertex(inner_radius * ca, inner_radius * sa, z1)

    for i in range(segments):
        j = (i + 1) % segments
        ob0, ot0, ib0, it0 = 4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3
        ob1, ot1, ib1, it1 = 4 * j, 4 * j + 1, 4 * j + 2, 4 * j + 3
        # Outside wall.
        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        # Inside wall.
        geom.add_face(ib0, it0, it1)
        geom.add_face(ib0, it1, ib1)
        # Top and bottom annular faces.
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        geom.add_face(ob0, ib0, ib1)
        geom.add_face(ob0, ib1, ob1)
    return geom


def _gable_roof_mesh(width: float, length: float, eave_z: float, ridge_z: float) -> MeshGeometry:
    """Simple closed pitched-roof prism, centered in X/Y."""
    geom = MeshGeometry()
    y0 = -length / 2.0
    y1 = length / 2.0
    pts = [
        (-width / 2.0, y0, eave_z),
        (width / 2.0, y0, eave_z),
        (0.0, y0, ridge_z),
        (-width / 2.0, y1, eave_z),
        (width / 2.0, y1, eave_z),
        (0.0, y1, ridge_z),
    ]
    for p in pts:
        geom.add_vertex(*p)
    # End triangles.
    geom.add_face(0, 1, 2)
    geom.add_face(3, 5, 4)
    # Sloping roof planes.
    geom.add_face(0, 2, 5)
    geom.add_face(0, 5, 3)
    geom.add_face(1, 4, 5)
    geom.add_face(1, 5, 2)
    # Flat underside that sits on the cap body.
    geom.add_face(0, 3, 4)
    geom.add_face(0, 4, 1)
    return geom


def _beam_origin_between(
    p0: tuple[float, float],
    p1: tuple[float, float],
    *,
    y: float,
) -> tuple[float, Origin]:
    """Return length and origin for a Box whose local X axis spans p0->p1 in XZ."""
    x0, z0 = p0
    x1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    angle = math.atan2(dz, dx)
    return length, Origin(xyz=((x0 + x1) / 2.0, y, (z0 + z1) / 2.0), rpy=(0.0, -angle, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_windmill")

    stone = Material("warm_limestone", color=(0.64, 0.58, 0.48, 1.0))
    dark_stone = Material("dark_foundation_stone", color=(0.35, 0.34, 0.32, 1.0))
    plaster = Material("aged_whitewash", color=(0.82, 0.79, 0.68, 1.0))
    wood = Material("weathered_oak", color=(0.50, 0.31, 0.16, 1.0))
    dark_wood = Material("dark_tarred_wood", color=(0.18, 0.12, 0.08, 1.0))
    roof_mat = Material("brown_thatch_roof", color=(0.46, 0.33, 0.16, 1.0))
    metal = Material("blackened_iron", color=(0.05, 0.05, 0.045, 1.0))
    window_mat = Material("dark_glass", color=(0.03, 0.05, 0.07, 1.0))

    # Root module: a heavy masonry foundation and tapered tower body.
    base = model.part("base")
    base.visual(
        Cylinder(radius=1.28, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_stone,
        name="foundation",
    )
    tower_mesh = LatheGeometry(
        [
            (0.0, 0.34),
            (0.90, 0.34),
            (0.54, 4.00),
            (0.0, 4.00),
        ],
        segments=72,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(tower_mesh, "tapered_tower"),
        material=plaster,
        name="tower_body",
    )
    base.visual(
        Box((0.42, 0.045, 0.82)),
        origin=Origin(xyz=(0.0, 0.875, 0.76)),
        material=dark_wood,
        name="front_door",
    )
    base.visual(
        Box((0.26, 0.04, 0.32)),
        origin=Origin(xyz=(0.0, 0.68, 2.30)),
        material=window_mat,
        name="front_window",
    )
    base.visual(
        Box((0.035, 0.24, 0.28)),
        origin=Origin(xyz=(0.62, 0.0, 2.95)),
        material=window_mat,
        name="side_window",
    )
    base.visual(
        Cylinder(radius=0.60, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 4.03)),
        material=stone,
        name="tower_capstone",
    )

    # Fixed bearing module: the visible curb and annular yaw bearing on the tower.
    bearing = model.part("bearing_module")
    bearing.visual(
        Cylinder(radius=0.62, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_wood,
        name="lower_curb",
    )
    yaw_ring_mesh = _hollow_cylinder_mesh(0.72, 0.50, 0.12, segments=72)
    yaw_ring_mesh.translate(0.0, 0.0, 0.18)
    bearing.visual(
        mesh_from_geometry(yaw_ring_mesh, "annular_yaw_bearing"),
        material=metal,
        name="yaw_ring",
    )

    # Rotating cap/head module: timber body, pitched roof, and front shaft bearing.
    head = model.part("head")
    head.visual(
        Cylinder(radius=0.56, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_wood,
        name="turntable",
    )
    head.visual(
        Box((1.20, 1.25, 0.55)),
        origin=Origin(xyz=(0.0, -0.02, 0.375)),
        material=wood,
        name="cap_body",
    )
    head.visual(
        mesh_from_geometry(_gable_roof_mesh(1.48, 1.58, 0.65, 0.98), "pitched_cap_roof"),
        material=roof_mat,
        name="roof",
    )
    head.visual(
        Box((0.42, 0.12, 0.34)),
        origin=Origin(xyz=(0.0, 0.635, 0.42)),
        material=dark_wood,
        name="front_bearing_block",
    )
    front_bearing_mesh = _hollow_cylinder_mesh(0.18, 0.075, 0.18, segments=48)
    front_bearing_mesh.rotate_x(-math.pi / 2.0).translate(0.0, 0.695, 0.42)
    head.visual(
        mesh_from_geometry(front_bearing_mesh, "hollow_front_bearing"),
        material=metal,
        name="front_bearing",
    )

    # Rotating sail hub with four open lattice blades.  The part frame is the
    # shaft axis at the front face of the head bearing, with +Y pointing outward.
    hub = model.part("sail_hub")
    hub.visual(
        Cylinder(radius=0.055, length=0.38),
        origin=Origin(xyz=(0.0, 0.16, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="shaft",
    )
    hub.visual(
        Cylinder(radius=0.14, length=0.035),
        origin=Origin(xyz=(0.0, 0.0175, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="thrust_collar",
    )
    hub.visual(
        Cylinder(radius=0.20, length=0.16),
        origin=Origin(xyz=(0.0, 0.36, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="hub_disk",
    )
    hub.visual(
        Sphere(radius=0.105),
        origin=Origin(xyz=(0.0, 0.465, 0.0)),
        material=metal,
        name="nose_cap",
    )

    blade_y = 0.42
    beam_depth = 0.065
    beam_thick = 0.055

    def add_blade(blade_index: int, phi: float) -> None:
        radial = (math.sin(phi), math.cos(phi))
        tangent = (math.cos(phi), -math.sin(phi))

        def point(radius: float, half_width: float = 0.0) -> tuple[float, float]:
            return (
                radial[0] * radius + tangent[0] * half_width,
                radial[1] * radius + tangent[1] * half_width,
            )

        def add_beam(name: str, p0: tuple[float, float], p1: tuple[float, float], thickness: float = beam_thick) -> None:
            length, origin = _beam_origin_between(p0, p1, y=blade_y)
            hub.visual(
                Box((length, beam_depth, thickness)),
                origin=origin,
                material=dark_wood,
                name=name,
            )

        # Main root stock dives into the hub, then two side rails form the sail.
        add_beam(f"blade_{blade_index}_spar", point(0.10), point(1.55), thickness=0.065)
        add_beam(f"blade_{blade_index}_rail_0", point(0.30, -0.08), point(1.48, -0.28))
        add_beam(f"blade_{blade_index}_rail_1", point(0.30, 0.08), point(1.48, 0.28))

        stations = [0.45, 0.70, 0.95, 1.20, 1.43]
        prev_left = point(0.30, -0.08)
        prev_right = point(0.30, 0.08)
        for idx, radius in enumerate(stations):
            t = (radius - 0.30) / (1.48 - 0.30)
            half = 0.08 + t * (0.28 - 0.08)
            left = point(radius, -half)
            right = point(radius, half)
            add_beam(f"blade_{blade_index}_cross_{idx}", left, right, thickness=0.045)
            if idx % 2 == 0:
                add_beam(f"blade_{blade_index}_brace_{idx}", prev_left, right, thickness=0.035)
            else:
                add_beam(f"blade_{blade_index}_brace_{idx}", prev_right, left, thickness=0.035)
            prev_left = left
            prev_right = right

    for i in range(4):
        add_blade(i, i * math.pi / 2.0)

    model.articulation(
        "base_to_bearing",
        ArticulationType.FIXED,
        parent=base,
        child=bearing,
        origin=Origin(xyz=(0.0, 0.0, 4.06)),
    )
    model.articulation(
        "bearing_to_head",
        ArticulationType.CONTINUOUS,
        parent=bearing,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35),
    )
    model.articulation(
        "head_to_sail_hub",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hub,
        origin=Origin(xyz=(0.0, 0.785, 0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bearing = object_model.get_part("bearing_module")
    head = object_model.get_part("head")
    hub = object_model.get_part("sail_hub")
    yaw = object_model.get_articulation("bearing_to_head")
    spin = object_model.get_articulation("head_to_sail_hub")

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    ctx.check(
        "cap yaw is continuous about the tower vertical",
        yaw.articulation_type == ArticulationType.CONTINUOUS and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "sail hub spins continuously about the shaft",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_gap(
        bearing,
        base,
        axis="z",
        positive_elem="lower_curb",
        negative_elem="tower_capstone",
        max_gap=0.001,
        max_penetration=0.00001,
        name="bearing module is seated on the base capstone",
    )
    ctx.expect_gap(
        head,
        bearing,
        axis="z",
        positive_elem="turntable",
        negative_elem="yaw_ring",
        max_gap=0.001,
        max_penetration=0.00001,
        name="head turntable rides on the bearing module",
    )
    ctx.expect_within(
        hub,
        head,
        axes="xz",
        inner_elem="shaft",
        outer_elem="front_bearing",
        margin=0.0,
        name="wind shaft is centered in the front bearing",
    )
    ctx.expect_overlap(
        hub,
        head,
        axes="y",
        elem_a="shaft",
        elem_b="front_bearing",
        min_overlap=0.02,
        name="wind shaft remains inserted through the bearing",
    )
    ctx.expect_contact(
        hub,
        head,
        elem_a="thrust_collar",
        elem_b="front_bearing",
        contact_tol=0.002,
        name="hub thrust collar bears on the front bearing",
    )

    rest_spar = center_from_aabb(ctx.part_element_world_aabb(hub, elem="blade_0_spar"))
    with ctx.pose({spin: math.pi / 2.0}):
        spun_spar = center_from_aabb(ctx.part_element_world_aabb(hub, elem="blade_0_spar"))
    ctx.check(
        "hub rotation carries a lattice blade around the shaft",
        rest_spar is not None
        and spun_spar is not None
        and spun_spar[0] > rest_spar[0] + 0.50
        and spun_spar[2] < rest_spar[2] - 0.50,
        details=f"rest={rest_spar}, spun={spun_spar}",
    )

    rest_hub_pos = ctx.part_world_position(hub)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_hub_pos = ctx.part_world_position(hub)
    ctx.check(
        "cap yaw swings the shaft around the tower axis",
        rest_hub_pos is not None
        and yawed_hub_pos is not None
        and yawed_hub_pos[0] < rest_hub_pos[0] - 0.50
        and abs(yawed_hub_pos[2] - rest_hub_pos[2]) < 0.01,
        details=f"rest={rest_hub_pos}, yawed={yawed_hub_pos}",
    )

    return ctx.report()


object_model = build_object_model()
