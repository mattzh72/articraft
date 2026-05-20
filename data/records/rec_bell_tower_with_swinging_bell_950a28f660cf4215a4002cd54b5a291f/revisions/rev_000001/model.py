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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _triangular_gusset(width: float, height: float, thickness: float) -> MeshGeometry:
    """Right-triangular welded plate in local XZ, extruded through local Y."""
    y0 = -thickness / 2.0
    y1 = thickness / 2.0
    pts = [
        (0.0, y0, 0.0),
        (width, y0, 0.0),
        (0.0, y0, height),
        (0.0, y1, 0.0),
        (width, y1, 0.0),
        (0.0, y1, height),
    ]
    geom = MeshGeometry()
    for p in pts:
        geom.add_vertex(*p)
    # Back/front triangular faces and three rectangular edge faces.
    faces = [
        (0, 2, 1),
        (3, 4, 5),
        (0, 1, 4),
        (0, 4, 3),
        (1, 2, 5),
        (1, 5, 4),
        (2, 0, 3),
        (2, 3, 5),
    ]
    for face in faces:
        geom.add_face(*face)
    return geom


def _bell_shell(segments: int = 80) -> MeshGeometry:
    """A hollow lathed bronze bell: flared rim, waist, shoulder, and crown."""
    outer = [
        (0.335, -1.055),
        (0.350, -1.005),
        (0.318, -0.930),
        (0.280, -0.815),
        (0.232, -0.660),
        (0.182, -0.500),
        (0.138, -0.360),
    ]
    inner = [
        (0.270, -1.015),
        (0.262, -0.930),
        (0.236, -0.805),
        (0.198, -0.640),
        (0.150, -0.480),
        (0.075, -0.360),
    ]
    geom = MeshGeometry()

    def add_ring(radius: float, z: float) -> list[int]:
        ids: list[int] = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ids.append(geom.add_vertex(radius * math.cos(a), radius * math.sin(a), z))
        return ids

    def connect(a: list[int], b: list[int], *, reverse: bool = False) -> None:
        for i in range(segments):
            j = (i + 1) % segments
            if reverse:
                geom.add_face(a[i], b[j], a[j])
                geom.add_face(a[i], b[i], b[j])
            else:
                geom.add_face(a[i], a[j], b[j])
                geom.add_face(a[i], b[j], b[i])

    outer_rings = [add_ring(r, z) for r, z in outer]
    inner_rings = [add_ring(r, z) for r, z in inner]

    for low, high in zip(outer_rings[:-1], outer_rings[1:]):
        connect(low, high)
    for low, high in zip(inner_rings[:-1], inner_rings[1:]):
        connect(low, high, reverse=True)

    # Thick, rounded-looking mouth lip and a closed crown annulus; the center
    # remains open inside so the dark clapper is visible through the mouth.
    connect(outer_rings[0], inner_rings[0], reverse=True)
    connect(outer_rings[-1], inner_rings[-1])
    return geom


def _add_cylinder_between(part, name: str, p0, p1, radius: float, material: Material) -> None:
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    yaw = math.atan2(dy, dx)
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    part.visual(
        Cylinder(radius, length),
        origin=Origin(
            xyz=((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="welded_bell_tower")

    painted_steel = Material("blackened_welded_steel", rgba=(0.045, 0.050, 0.050, 1.0))
    edge_worn_steel = Material("worn_steel_edges", rgba=(0.18, 0.19, 0.18, 1.0))
    cast_iron = Material("dark_cast_yoke", rgba=(0.025, 0.027, 0.028, 1.0))
    bronze = Material("aged_bell_bronze", rgba=(0.78, 0.53, 0.22, 1.0))
    dark_bronze = Material("shadowed_bronze", rgba=(0.36, 0.24, 0.11, 1.0))

    frame = model.part("frame")

    # Square welded baseplate.
    frame.visual(
        Box((1.80, 1.80, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=painted_steel,
        name="baseplate",
    )

    post_xy = 0.65
    post_bottom = 0.020
    post_top = 2.090
    post_len = post_top - post_bottom
    post_positions = [
        (-post_xy, -post_xy),
        (post_xy, -post_xy),
        (post_xy, post_xy),
        (-post_xy, post_xy),
    ]
    for idx, (x, y) in enumerate(post_positions):
        frame.visual(
            Cylinder(0.035, post_len),
            origin=Origin(xyz=(x, y, (post_top + post_bottom) / 2.0)),
            material=painted_steel,
            name=f"post_{idx}",
        )

    # Top perimeter tubes tie all four posts together.
    top_z = 2.100
    frame.visual(
        Box((1.42, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, post_xy, top_z)),
        material=painted_steel,
        name="top_front_beam",
    )
    frame.visual(
        Box((1.42, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, -post_xy, top_z)),
        material=painted_steel,
        name="top_rear_beam",
    )
    frame.visual(
        Box((0.12, 1.42, 0.12)),
        origin=Origin(xyz=(post_xy, 0.0, top_z)),
        material=painted_steel,
        name="top_side_beam_0",
    )
    frame.visual(
        Box((0.12, 1.42, 0.12)),
        origin=Origin(xyz=(-post_xy, 0.0, top_z)),
        material=painted_steel,
        name="top_side_beam_1",
    )

    # The central horizontal box-section crossbeam is modeled as a hollow
    # rectangular tube with visible web and flange thickness.
    beam_len = 1.58
    beam_depth = 0.16
    beam_height = 0.14
    wall = 0.018
    frame.visual(
        Box((beam_len, beam_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, top_z + beam_height / 2.0 - wall / 2.0)),
        material=painted_steel,
        name="crossbeam_top",
    )
    frame.visual(
        Box((beam_len, beam_depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, top_z - beam_height / 2.0 + wall / 2.0)),
        material=painted_steel,
        name="crossbeam_bottom",
    )
    frame.visual(
        Box((beam_len, wall, beam_height)),
        origin=Origin(xyz=(0.0, beam_depth / 2.0 - wall / 2.0, top_z)),
        material=painted_steel,
        name="crossbeam_front_web",
    )
    frame.visual(
        Box((beam_len, wall, beam_height)),
        origin=Origin(xyz=(0.0, -beam_depth / 2.0 + wall / 2.0, top_z)),
        material=painted_steel,
        name="crossbeam_rear_web",
    )

    # X-bracing on all four tower faces, made from smaller round tube.
    brace_r = 0.018
    z_low = 0.135
    z_high = 1.925
    side_specs = [
        ((-post_xy, post_xy, z_low), (post_xy, post_xy, z_high), "front_brace_0"),
        ((post_xy, post_xy, z_low), (-post_xy, post_xy, z_high), "front_brace_1"),
        ((-post_xy, -post_xy, z_low), (post_xy, -post_xy, z_high), "rear_brace_0"),
        ((post_xy, -post_xy, z_low), (-post_xy, -post_xy, z_high), "rear_brace_1"),
        ((post_xy, -post_xy, z_low), (post_xy, post_xy, z_high), "side_brace_0"),
        ((post_xy, post_xy, z_low), (post_xy, -post_xy, z_high), "side_brace_1"),
        ((-post_xy, -post_xy, z_low), (-post_xy, post_xy, z_high), "side_brace_2"),
        ((-post_xy, post_xy, z_low), (-post_xy, -post_xy, z_high), "side_brace_3"),
    ]
    for p0, p1, name in side_specs:
        _add_cylinder_between(frame, name, p0, p1, brace_r, painted_steel)

    # Visible triangular gusset plates at the post bases and under the top frame.
    base_gusset = _triangular_gusset(0.18, 0.22, 0.012)
    top_gusset = _triangular_gusset(0.16, 0.20, 0.012)
    for idx, (x, y) in enumerate(post_positions):
        yaw = math.atan2(-y, -x)
        frame.visual(
            mesh_from_geometry(base_gusset, f"base_gusset_mesh_{idx}"),
            origin=Origin(xyz=(x, y, 0.035), rpy=(0.0, 0.0, yaw)),
            material=edge_worn_steel,
            name=f"base_gusset_{idx}",
        )
        frame.visual(
            mesh_from_geometry(top_gusset, f"top_gusset_mesh_{idx}"),
            origin=Origin(xyz=(x, y, 2.105), rpy=(math.pi, 0.0, yaw)),
            material=edge_worn_steel,
            name=f"top_gusset_{idx}",
        )

    # Bearing hanger plates and the through-bolt that carries the swinging yoke.
    pivot_z = 1.740
    for idx, x in enumerate((-0.55, 0.55)):
        frame.visual(
            Box((0.035, 0.22, 0.50)),
            origin=Origin(xyz=(x, 0.0, 1.865)),
            material=painted_steel,
            name=f"bearing_plate_{idx}",
        )
    frame.visual(
        Cylinder(0.018, 1.16),
        origin=Origin(xyz=(0.0, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=edge_worn_steel,
        name="pivot_bolt",
    )
    frame.visual(
        Cylinder(0.040, 0.040),
        origin=Origin(xyz=(-0.595, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=edge_worn_steel,
        name="bolt_head_0",
    )
    frame.visual(
        Cylinder(0.040, 0.040),
        origin=Origin(xyz=(0.595, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=edge_worn_steel,
        name="bolt_head_1",
    )

    bell = model.part("bell")
    bell.visual(
        mesh_from_geometry(_bell_shell(), "bell_body_mesh"),
        origin=Origin(),
        material=bronze,
        name="bell_body",
    )
    bell.visual(
        Cylinder(0.112, 0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.390)),
        material=dark_bronze,
        name="crown_saddle",
    )
    bell.visual(
        Box((0.80, 0.14, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.335)),
        material=cast_iron,
        name="yoke_bar",
    )
    for idx, x in enumerate((-0.38, 0.38)):
        bell.visual(
            Cylinder(0.060, 0.120),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cast_iron,
            name=f"hub_{idx}",
        )
        bell.visual(
            Box((0.060, 0.115, 0.340)),
            origin=Origin(xyz=(x, 0.0, -0.200)),
            material=cast_iron,
            name=f"yoke_arm_{idx}",
        )
    bell.visual(
        Cylinder(0.010, 0.570),
        origin=Origin(xyz=(0.0, 0.0, -0.655)),
        material=cast_iron,
        name="clapper_stem",
    )
    bell.visual(
        Sphere(0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.965)),
        material=cast_iron,
        name="clapper_ball",
    )

    model.articulation(
        "bell_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-0.28, upper=0.28),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    bell = object_model.get_part("bell")
    pivot = object_model.get_articulation("bell_pivot")

    for hub in ("hub_0", "hub_1"):
        ctx.allow_overlap(
            frame,
            bell,
            elem_a="pivot_bolt",
            elem_b=hub,
            reason="The through-bolt is intentionally captured inside the yoke hub bore proxy.",
        )
        ctx.expect_overlap(
            frame,
            bell,
            axes="x",
            elem_a="pivot_bolt",
            elem_b=hub,
            min_overlap=0.080,
            name=f"{hub} has bolt engagement along pivot axis",
        )
        ctx.expect_overlap(
            frame,
            bell,
            axes="yz",
            elem_a="pivot_bolt",
            elem_b=hub,
            min_overlap=0.030,
            name=f"{hub} surrounds the through-bolt",
        )

    with ctx.pose({pivot: 0.0}):
        ctx.expect_gap(
            frame,
            bell,
            axis="z",
            positive_elem="crossbeam_bottom",
            negative_elem="yoke_bar",
            min_gap=0.18,
            name="crossbeam clears swinging yoke",
        )
        rest_body = ctx.part_element_world_aabb(bell, elem="bell_body")

    with ctx.pose({pivot: 0.25}):
        swung_body = ctx.part_element_world_aabb(bell, elem="bell_body")

    rest_y = None if rest_body is None else (rest_body[0][1] + rest_body[1][1]) / 2.0
    swung_y = None if swung_body is None else (swung_body[0][1] + swung_body[1][1]) / 2.0
    ctx.check(
        "bell rotates forward on the horizontal pivot",
        rest_y is not None and swung_y is not None and swung_y > rest_y + 0.14,
        details=f"rest_y={rest_y}, swung_y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
