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
    PerforatedPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_between(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
    *,
    extra: float = 0.0,
) -> None:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    ux, uy, uz = dx / length, dy / length, dz / length
    sx -= ux * extra
    sy -= uy * extra
    sz -= uz * extra
    ex += ux * extra
    ey += uy * extra
    ez += uz * extra
    length += 2.0 * extra
    mx, my, mz = (sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5
    radial = math.sqrt(ux * ux + uy * uy)
    pitch = math.atan2(radial, uz)
    yaw = math.atan2(uy, ux) if radial > 1.0e-9 else 0.0
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _lathe_shell_x(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Thin revolved shell whose axis is local X; profile tuples are (x, radius)."""
    geom = MeshGeometry()

    def add_ring(x: float, radius: float) -> list[int]:
        ring = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ring.append(geom.add_vertex(x, radius * math.cos(a), radius * math.sin(a)))
        return ring

    outer = [add_ring(x, r) for x, r in outer_profile]
    inner = [add_ring(x, r) for x, r in inner_profile]

    def connect(a: list[int], b: list[int], *, flip: bool = False) -> None:
        for i in range(segments):
            j = (i + 1) % segments
            if not flip:
                geom.add_face(a[i], b[i], b[j])
                geom.add_face(a[i], b[j], a[j])
            else:
                geom.add_face(a[i], b[j], b[i])
                geom.add_face(a[i], a[j], b[j])

    for idx in range(len(outer) - 1):
        connect(outer[idx], outer[idx + 1])
    for idx in range(len(inner) - 1):
        connect(inner[idx], inner[idx + 1], flip=True)

    # Annular lips at the rear and front tie the inner and outer skins together.
    connect(outer[0], inner[0], flip=True)
    connect(inner[-1], outer[-1], flip=True)
    return geom


def _conical_reflector_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    segments = 72
    sections = [
        (-0.285, 0.040),
        (-0.210, 0.082),
        (-0.060, 0.145),
        (0.245, 0.205),
    ]
    rings: list[list[int]] = []
    for x, radius in sections:
        ring = []
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            ring.append(geom.add_vertex(x, radius * math.cos(a), radius * math.sin(a)))
        rings.append(ring)
    for ri in range(len(rings) - 1):
        a_ring, b_ring = rings[ri], rings[ri + 1]
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(a_ring[i], b_ring[i], b_ring[j])
            geom.add_face(a_ring[i], b_ring[j], a_ring[j])
    return geom


def _top_visor_mesh() -> MeshGeometry:
    geom = MeshGeometry()
    segments = 28
    x_values = (-0.015, 0.430)
    radii = (0.246, 0.270)
    angles = [
        math.radians(34.0 + (146.0 - 34.0) * i / (segments - 1))
        for i in range(segments)
    ]

    grid: dict[tuple[int, int], list[int]] = {}
    for xi, x in enumerate(x_values):
        for ri, radius in enumerate(radii):
            strip = []
            for a in angles:
                strip.append(geom.add_vertex(x, radius * math.cos(a), radius * math.sin(a)))
            grid[(xi, ri)] = strip

    def strip_faces(a: list[int], b: list[int]) -> None:
        for i in range(segments - 1):
            geom.add_face(a[i], b[i], b[i + 1])
            geom.add_face(a[i], b[i + 1], a[i + 1])

    # Outer and inner curved skins, plus front/rear/end edge thickness.
    strip_faces(grid[(0, 1)], grid[(1, 1)])
    strip_faces(grid[(1, 0)], grid[(0, 0)])
    strip_faces(grid[(0, 0)], grid[(0, 1)])
    strip_faces(grid[(1, 1)], grid[(1, 0)])
    for end_idx in (0, segments - 1):
        a, b, c, d = (
            grid[(0, 0)][end_idx],
            grid[(1, 0)][end_idx],
            grid[(1, 1)][end_idx],
            grid[(0, 1)][end_idx],
        )
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_searchlight_tower")

    matte_graphite = model.material("matte_graphite", rgba=(0.045, 0.050, 0.055, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.017, 0.020, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    warm_bronze = model.material("warm_bronze", rgba=(0.60, 0.48, 0.31, 1.0))
    reflector = model.material("soft_reflector", rgba=(0.86, 0.84, 0.78, 1.0))
    lens_glass = model.material("frosted_lens", rgba=(0.64, 0.82, 0.95, 0.46))
    cable_black = model.material("cable_black", rgba=(0.005, 0.006, 0.007, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((1.22, 1.22, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=satin_black,
        name="base_plinth",
    )
    tower.visual(
        Box((1.05, 1.05, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.177)),
        material=warm_bronze,
        name="base_trim",
    )
    tower.visual(
        Cylinder(radius=0.060, length=2.93),
        origin=Origin(xyz=(0.0, 0.0, 1.625)),
        material=matte_graphite,
        name="central_mast",
    )
    tower.visual(
        Cylinder(radius=0.102, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=brushed_steel,
        name="mast_foot_collar",
    )

    lower_nodes = [
        (-0.46, -0.46, 0.17),
        (0.46, -0.46, 0.17),
        (0.46, 0.46, 0.17),
        (-0.46, 0.46, 0.17),
    ]
    upper_nodes = [
        (-0.34, -0.34, 2.72),
        (0.34, -0.34, 2.72),
        (0.34, 0.34, 2.72),
        (-0.34, 0.34, 2.72),
    ]
    for idx, (low, high) in enumerate(zip(lower_nodes, upper_nodes)):
        _cylinder_between(
            tower,
            f"corner_leg_{idx}",
            low,
            high,
            0.021,
            matte_graphite,
            extra=0.020,
        )
        tower.visual(
            Sphere(radius=0.038),
            origin=Origin(xyz=low),
            material=brushed_steel,
            name=f"base_node_{idx}",
        )
        tower.visual(
            Sphere(radius=0.032),
            origin=Origin(xyz=high),
            material=brushed_steel,
            name=f"platform_node_{idx}",
        )

    for level, z in enumerate((0.63, 1.10, 1.57, 2.04, 2.51)):
        span = 0.46 - 0.12 * (z - 0.17) / (2.72 - 0.17)
        pts = [(-span, -span, z), (span, -span, z), (span, span, z), (-span, span, z)]
        for idx in range(4):
            _cylinder_between(
                tower,
                f"belt_{level}_{idx}",
                pts[idx],
                pts[(idx + 1) % 4],
                0.012,
                brushed_steel if level in (0, 4) else matte_graphite,
                extra=0.008,
            )
        lower_span = span + 0.030
        upper_span = span - 0.030
        lower = [
            (-lower_span, -lower_span, z - 0.23),
            (lower_span, -lower_span, z - 0.23),
            (lower_span, lower_span, z - 0.23),
            (-lower_span, lower_span, z - 0.23),
        ]
        upper = [
            (-upper_span, -upper_span, z + 0.23),
            (upper_span, -upper_span, z + 0.23),
            (upper_span, upper_span, z + 0.23),
            (-upper_span, upper_span, z + 0.23),
        ]
        for idx in range(4):
            _cylinder_between(
                tower,
                f"diagonal_{level}_{idx}",
                lower[idx],
                upper[(idx + 1) % 4],
                0.010,
                matte_graphite,
                extra=0.006,
            )

    deck_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.94, 0.94),
            0.050,
            hole_diameter=0.032,
            pitch=(0.075, 0.075),
            frame=0.075,
            corner_radius=0.025,
            stagger=True,
        ),
        "service_deck_grating",
    )
    tower.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 2.760)),
        material=satin_graphite,
        name="service_deck",
    )
    for idx, (x, y) in enumerate(((-0.49, -0.49), (0.49, -0.49), (0.49, 0.49), (-0.49, 0.49))):
        tower.visual(
            Cylinder(radius=0.013, length=0.520),
            origin=Origin(xyz=(x, y, 3.030)),
            material=brushed_steel,
            name=f"rail_post_{idx}",
        )
    rail_corners = [(-0.49, -0.49, 3.270), (0.49, -0.49, 3.270), (0.49, 0.49, 3.270), (-0.49, 0.49, 3.270)]
    mid_rail_corners = [(-0.49, -0.49, 3.090), (0.49, -0.49, 3.090), (0.49, 0.49, 3.090), (-0.49, 0.49, 3.090)]
    for idx in range(4):
        _cylinder_between(tower, f"top_rail_{idx}", rail_corners[idx], rail_corners[(idx + 1) % 4], 0.012, brushed_steel, extra=0.006)
        _cylinder_between(tower, f"mid_rail_{idx}", mid_rail_corners[idx], mid_rail_corners[(idx + 1) % 4], 0.009, matte_graphite, extra=0.006)
        _cylinder_between(
            tower,
            f"rail_knee_{idx}",
            upper_nodes[idx],
            (rail_corners[idx][0], rail_corners[idx][1], 2.800),
            0.011,
            brushed_steel,
            extra=0.018,
        )

    tower.visual(
        Box((0.035, 0.900, 0.060)),
        origin=Origin(xyz=(-0.468, 0.0, 2.785)),
        material=warm_bronze,
        name="toe_board",
    )
    _cylinder_between(tower, "ladder_rail_0", (-0.545, 0.31, 0.22), (-0.545, 0.31, 2.70), 0.010, brushed_steel, extra=0.010)
    _cylinder_between(tower, "ladder_rail_1", (-0.545, 0.43, 0.22), (-0.545, 0.43, 2.70), 0.010, brushed_steel, extra=0.010)
    for idx, z in enumerate((0.42, 0.68, 0.94, 1.20, 1.46, 1.72, 1.98, 2.24, 2.50)):
        _cylinder_between(tower, f"ladder_rung_{idx}", (-0.545, 0.30, z), (-0.545, 0.44, z), 0.007, brushed_steel, extra=0.006)
    _cylinder_between(tower, "ladder_stand_off_0", (-0.545, 0.31, 0.42), (-0.448, 0.448, 0.42), 0.008, brushed_steel, extra=0.012)
    _cylinder_between(tower, "ladder_stand_off_1", (-0.545, 0.43, 1.46), (-0.399, 0.399, 1.46), 0.008, brushed_steel, extra=0.012)
    _cylinder_between(tower, "ladder_stand_off_2", (-0.545, 0.31, 2.50), (-0.350, 0.350, 2.50), 0.008, brushed_steel, extra=0.012)

    tower.visual(
        Box((0.18, 0.11, 0.16)),
        origin=Origin(xyz=(0.39, -0.39, 2.865)),
        material=satin_black,
        name="service_box",
    )
    tower.visual(
        Box((0.012, 0.105, 0.065)),
        origin=Origin(xyz=(0.296, -0.39, 2.875)),
        material=warm_bronze,
        name="service_box_latch",
    )
    tower.visual(
        Cylinder(radius=0.245, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 3.105)),
        material=brushed_steel,
        name="top_bearing_seat",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.238, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=warm_bronze,
        name="lower_bearing",
    )
    turntable.visual(
        Cylinder(radius=0.176, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=brushed_steel,
        name="upper_bearing",
    )
    turntable.visual(
        Cylinder(radius=0.070, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=satin_graphite,
        name="pan_spindle",
    )
    turntable.visual(
        Box((0.340, 0.830, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=satin_graphite,
        name="yoke_saddle",
    )
    turntable.visual(
        Box((0.120, 0.080, 0.650)),
        origin=Origin(xyz=(0.0, -0.380, 0.615)),
        material=matte_graphite,
        name="yoke_arm_0",
    )
    turntable.visual(
        Cylinder(radius=0.082, length=0.090),
        origin=Origin(xyz=(0.0, -0.3553, 0.635), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="bearing_0",
    )
    turntable.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, -0.4085, 0.635), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_bronze,
        name="bearing_cap_0",
    )
    turntable.visual(
        Box((0.120, 0.080, 0.650)),
        origin=Origin(xyz=(0.0, 0.380, 0.615)),
        material=matte_graphite,
        name="yoke_arm_1",
    )
    turntable.visual(
        Cylinder(radius=0.082, length=0.090),
        origin=Origin(xyz=(0.0, 0.3553, 0.635), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="bearing_1",
    )
    turntable.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, 0.4085, 0.635), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_bronze,
        name="bearing_cap_1",
    )
    turntable.visual(
        Box((0.060, 0.690, 0.045)),
        origin=Origin(xyz=(-0.078, 0.0, 0.930)),
        material=brushed_steel,
        name="rear_yoke_tie",
    )

    spotlight = model.part("spotlight")
    shell = _lathe_shell_x(
        [
            (-0.375, 0.176),
            (-0.325, 0.204),
            (-0.095, 0.224),
            (0.300, 0.244),
            (0.390, 0.252),
        ],
        [
            (-0.360, 0.144),
            (-0.300, 0.176),
            (-0.080, 0.196),
            (0.292, 0.216),
            (0.382, 0.224),
        ],
        segments=80,
    )
    spotlight.visual(
        mesh_from_geometry(shell, "spotlight_shell"),
        origin=Origin(),
        material=satin_graphite,
        name="lamp_shell",
    )
    spotlight.visual(
        mesh_from_geometry(_top_visor_mesh(), "spotlight_top_visor"),
        origin=Origin(),
        material=matte_graphite,
        name="top_visor",
    )
    spotlight.visual(
        mesh_from_geometry(_conical_reflector_mesh(), "spotlight_reflector"),
        origin=Origin(),
        material=reflector,
        name="reflector_bowl",
    )
    spotlight.visual(
        Cylinder(radius=0.222, length=0.020),
        origin=Origin(xyz=(0.392, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    front_bezel = _lathe_shell_x(
        [(0.366, 0.256), (0.410, 0.256)],
        [(0.366, 0.218), (0.410, 0.218)],
        segments=80,
    )
    spotlight.visual(
        mesh_from_geometry(front_bezel, "front_bezel"),
        origin=Origin(),
        material=matte_graphite,
        name="front_bezel",
    )
    spotlight.visual(
        Cylinder(radius=0.160, length=0.055),
        origin=Origin(xyz=(-0.386, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_graphite,
        name="rear_cover",
    )
    spotlight.visual(
        Sphere(radius=0.042),
        origin=Origin(xyz=(-0.235, 0.0, 0.0)),
        material=warm_bronze,
        name="lamp_bulb",
    )
    spotlight.visual(
        Cylinder(radius=0.017, length=0.150),
        origin=Origin(xyz=(-0.300, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="bulb_stem",
    )
    spotlight.visual(
        Cylinder(radius=0.040, length=0.650),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="tilt_spindle",
    )
    for side, y in enumerate((-0.270, 0.270)):
        spotlight.visual(
            Cylinder(radius=0.077, length=0.060),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=warm_bronze,
            name=f"side_trunnion_{side}",
        )
    spotlight.visual(
        Box((0.170, 0.050, 0.035)),
        origin=Origin(xyz=(-0.310, 0.0, 0.185)),
        material=brushed_steel,
        name="rear_handle",
    )

    model.articulation(
        "tower_to_turntable",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 3.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "turntable_to_spotlight",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=spotlight,
        origin=Origin(xyz=(0.0, 0.0, 0.635)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=-0.45, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    turntable = object_model.get_part("turntable")
    spotlight = object_model.get_part("spotlight")
    pan = object_model.get_articulation("tower_to_turntable")
    tilt = object_model.get_articulation("turntable_to_spotlight")

    ctx.allow_overlap(
        turntable,
        spotlight,
        elem_a="bearing_0",
        elem_b="tilt_spindle",
        reason="The polished tilt spindle is intentionally captured inside the yoke bearing collar.",
    )
    ctx.allow_overlap(
        turntable,
        spotlight,
        elem_a="bearing_1",
        elem_b="tilt_spindle",
        reason="The opposite tilt spindle end is intentionally seated in the matching yoke bearing collar.",
    )

    ctx.expect_contact(
        tower,
        turntable,
        elem_a="top_bearing_seat",
        elem_b="lower_bearing",
        contact_tol=0.0015,
        name="pan bearing sits on tower seat",
    )
    ctx.expect_within(
        spotlight,
        turntable,
        axes="xz",
        inner_elem="tilt_spindle",
        outer_elem="bearing_0",
        margin=0.002,
        name="tilt spindle centered in bearing 0",
    )
    ctx.expect_within(
        spotlight,
        turntable,
        axes="xz",
        inner_elem="tilt_spindle",
        outer_elem="bearing_1",
        margin=0.002,
        name="tilt spindle centered in bearing 1",
    )
    ctx.expect_overlap(
        spotlight,
        turntable,
        axes="y",
        elem_a="tilt_spindle",
        elem_b="bearing_0",
        min_overlap=0.010,
        name="bearing 0 retains spindle insertion",
    )
    ctx.expect_overlap(
        spotlight,
        turntable,
        axes="y",
        elem_a="tilt_spindle",
        elem_b="bearing_1",
        min_overlap=0.010,
        name="bearing 1 retains spindle insertion",
    )

    def elem_center(part, elem: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    rest_lens = elem_center(spotlight, "front_lens")
    with ctx.pose({tilt: 0.70}):
        raised_lens = elem_center(spotlight, "front_lens")
    ctx.check(
        "tilt stage elevates the beam",
        rest_lens is not None and raised_lens is not None and raised_lens[2] > rest_lens[2] + 0.18,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    with ctx.pose({pan: 0.90}):
        panned_lens = elem_center(spotlight, "front_lens")
    ctx.check(
        "pan stage slews the spotlight head",
        rest_lens is not None and panned_lens is not None and abs(panned_lens[1] - rest_lens[1]) > 0.20,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    return ctx.report()


object_model = build_object_model()
