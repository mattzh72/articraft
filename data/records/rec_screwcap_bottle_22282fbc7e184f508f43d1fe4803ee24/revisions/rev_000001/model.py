from __future__ import annotations

from math import cos, pi, sin

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


THREAD_PITCH = 0.012
CAP_TURNS = 3.0
CAP_TRAVEL = THREAD_PITCH * CAP_TURNS


def _cached_vertex(mesh: MeshGeometry, cache: dict[tuple[float, float, float], int], x: float, y: float, z: float) -> int:
    key = (round(x, 6), round(y, 6), round(z, 6))
    if key not in cache:
        cache[key] = mesh.add_vertex(x, y, z)
    return cache[key]


def _ring(mesh: MeshGeometry, cache: dict[tuple[float, float, float], int], radius: float, z: float, segments: int) -> list[int]:
    return [
        _cached_vertex(mesh, cache, radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments), z)
        for i in range(segments)
    ]


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _add_lathe_band(mesh: MeshGeometry, cache: dict[tuple[float, float, float], int], profile: list[tuple[float, float]], segments: int) -> None:
    rings = [_ring(mesh, cache, r, z, segments) for r, z in profile]
    for lower, upper in zip(rings, rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            _add_quad(mesh, lower[i], lower[j], upper[j], upper[i])


def _add_annular_face(
    mesh: MeshGeometry,
    cache: dict[tuple[float, float, float], int],
    inner_radius: float,
    outer_radius: float,
    z: float,
    segments: int,
) -> None:
    inner = _ring(mesh, cache, inner_radius, z, segments)
    outer = _ring(mesh, cache, outer_radius, z, segments)
    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(mesh, inner[i], inner[j], outer[j], outer[i])


def _add_disk_face(mesh: MeshGeometry, cache: dict[tuple[float, float, float], int], radius: float, z: float, segments: int) -> None:
    center = _cached_vertex(mesh, cache, 0.0, 0.0, z)
    ring = _ring(mesh, cache, radius, z, segments)
    for i in range(segments):
        mesh.add_face(center, ring[i], ring[(i + 1) % segments])


def _bottle_shell_mesh() -> MeshGeometry:
    """Thin-walled, visibly hollow legacy bottle body with a thick serviceable base."""
    segments = 96
    mesh = MeshGeometry()
    cache: dict[tuple[float, float, float], int] = {}

    outer = [
        (0.032, 0.000),
        (0.041, 0.006),
        (0.043, 0.014),
        (0.044, 0.026),
        (0.044, 0.128),
        (0.039, 0.150),
        (0.030, 0.168),
        (0.021, 0.180),
        (0.020, 0.218),
        (0.023, 0.225),
    ]
    inner = [
        (0.020, 0.014),
        (0.037, 0.026),
        (0.037, 0.125),
        (0.033, 0.146),
        (0.025, 0.164),
        (0.016, 0.182),
        (0.016, 0.225),
    ]
    _add_lathe_band(mesh, cache, outer, segments)
    _add_lathe_band(mesh, cache, inner, segments)
    _add_annular_face(mesh, cache, 0.016, 0.023, 0.225, segments)
    _add_annular_face(mesh, cache, 0.020, 0.043, 0.014, segments)
    _add_disk_face(mesh, cache, 0.032, 0.000, segments)
    return mesh


def _cap_shell_mesh() -> MeshGeometry:
    """Open-bottom screw cap cup; transparent material exposes the internal thread."""
    segments = 96
    mesh = MeshGeometry()
    cache: dict[tuple[float, float, float], int] = {}

    _add_lathe_band(mesh, cache, [(0.038, 0.000), (0.038, 0.056)], segments)
    _add_lathe_band(mesh, cache, [(0.031, 0.003), (0.031, 0.049)], segments)
    _add_annular_face(mesh, cache, 0.031, 0.038, 0.003, segments)
    _add_annular_face(mesh, cache, 0.031, 0.038, 0.049, segments)
    _add_lathe_band(mesh, cache, [(0.038, 0.049), (0.038, 0.056)], segments)
    _add_disk_face(mesh, cache, 0.038, 0.056, segments)
    _add_disk_face(mesh, cache, 0.031, 0.049, segments)
    return mesh


def _ring_mesh(inner_radius: float, outer_radius: float, z_min: float, z_max: float, segments: int = 96) -> MeshGeometry:
    mesh = MeshGeometry()
    cache: dict[tuple[float, float, float], int] = {}
    _add_lathe_band(mesh, cache, [(outer_radius, z_min), (outer_radius, z_max)], segments)
    _add_lathe_band(mesh, cache, [(inner_radius, z_min), (inner_radius, z_max)], segments)
    _add_annular_face(mesh, cache, inner_radius, outer_radius, z_min, segments)
    _add_annular_face(mesh, cache, inner_radius, outer_radius, z_max, segments)
    return mesh


def _helical_thread_mesh(
    *,
    root_radius: float,
    tip_radius: float,
    z_start: float,
    pitch: float,
    turns: float,
    flank_width: float,
    theta_offset: float = 0.0,
    samples_per_turn: int = 72,
) -> MeshGeometry:
    """Trapezoidal helical ridge; root is slightly buried in its parent wall."""
    mesh = MeshGeometry()
    samples = int(turns * samples_per_turn) + 1
    rails: list[list[int]] = [[], [], [], []]
    half = flank_width * 0.5
    tip_half = half * 0.42

    for i in range(samples):
        u = i / (samples - 1)
        theta = theta_offset + turns * 2.0 * pi * u
        z = z_start + pitch * turns * u
        points = (
            (root_radius, z - half),
            (tip_radius, z - tip_half),
            (tip_radius, z + tip_half),
            (root_radius, z + half),
        )
        for rail, (radius, zz) in zip(rails, points):
            rail.append(mesh.add_vertex(radius * cos(theta), radius * sin(theta), zz))

    for i in range(samples - 1):
        for rail_index in range(4):
            next_rail = (rail_index + 1) % 4
            _add_quad(mesh, rails[rail_index][i], rails[rail_index][i + 1], rails[next_rail][i + 1], rails[next_rail][i])

    for end in (0, samples - 1):
        mesh.add_face(rails[0][end], rails[1][end], rails[2][end])
        mesh.add_face(rails[0][end], rails[2][end], rails[3][end])
    return mesh


def _add_radial_box(part, *, radius: float, theta: float, size: tuple[float, float, float], z: float, material: Material, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=(radius * cos(theta), radius * sin(theta), z), rpy=(0.0, 0.0, theta)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_screwcap_bottle")

    amber = model.material("aged_amber_glass", rgba=(0.62, 0.37, 0.12, 0.48))
    gunmetal = model.material("oiled_gunmetal", rgba=(0.12, 0.13, 0.13, 1.0))
    brass = model.material("worn_brass", rgba=(0.72, 0.52, 0.20, 1.0))
    gasket = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    bakelite = model.material("smoked_bakelite", rgba=(0.16, 0.07, 0.035, 0.74))
    enamel = model.material("cream_enamel", rgba=(0.86, 0.80, 0.66, 1.0))

    bottle = model.part("bottle")
    bottle.visual(mesh_from_geometry(_bottle_shell_mesh(), "bottle_shell"), material=amber, name="hollow_shell")
    bottle.visual(mesh_from_geometry(_helical_thread_mesh(root_radius=0.0195, tip_radius=0.0244, z_start=0.186, pitch=THREAD_PITCH, turns=2.15, flank_width=0.0042), "neck_thread"), material=brass, name="neck_thread")
    bottle.visual(mesh_from_geometry(_ring_mesh(0.020, 0.034, 0.170, 0.178), "neck_adapter"), material=gunmetal, name="neck_adapter")
    bottle.visual(mesh_from_geometry(_ring_mesh(0.036, 0.047, 0.018, 0.028), "base_adapter"), material=gunmetal, name="base_adapter")

    # Old service access plates are deliberately surface-mounted and bolted, as on a retrofit field repair.
    bottle.visual(Box((0.034, 0.006, 0.048)), origin=Origin(xyz=(0.0, 0.0445, 0.092)), material=gunmetal, name="front_hatch")
    bottle.visual(Box((0.030, 0.006, 0.038)), origin=Origin(xyz=(0.0, -0.0445, 0.105)), material=gunmetal, name="rear_hatch")
    bottle.visual(Box((0.020, 0.003, 0.020)), origin=Origin(xyz=(0.0, 0.0480, 0.092)), material=enamel, name="hatch_label")

    bolt_id = 0
    for y, zc, xs, zs in (
        (0.0480, 0.092, (-0.013, 0.013), (0.073, 0.111)),
        (-0.0480, 0.105, (-0.011, 0.011), (0.090, 0.120)),
    ):
        for x in xs:
            for z in zs:
                bottle.visual(Sphere(0.0026), origin=Origin(xyz=(x, y, z)), material=brass, name=f"hatch_bolt_{bolt_id}")
                bolt_id += 1

    for i in range(8):
        theta = 2.0 * pi * i / 8.0
        _add_radial_box(bottle, radius=0.028, theta=theta, size=(0.005, 0.014, 0.030), z=0.163, material=gunmetal, name=f"neck_rib_{i}")
    for i in range(10):
        theta = 2.0 * pi * i / 10.0
        bottle.visual(Cylinder(radius=0.0024, length=0.004), origin=Origin(xyz=(0.029 * cos(theta), 0.029 * sin(theta), 0.179)), material=brass, name=f"adapter_bolt_{i}")

    cap_spindle = model.part("cap_spindle")
    cap = model.part("cap")
    cap.visual(mesh_from_geometry(_cap_shell_mesh(), "cap_shell"), material=bakelite, name="cap_shell")
    cap.visual(
        mesh_from_geometry(
            _helical_thread_mesh(
                root_radius=0.0312,
                tip_radius=0.0266,
                z_start=0.010,
                pitch=THREAD_PITCH,
                turns=2.15,
                flank_width=0.0040,
                theta_offset=pi,
            ),
            "cap_thread",
        ),
        material=brass,
        name="cap_thread",
    )
    cap.visual(mesh_from_geometry(_ring_mesh(0.031, 0.0395, 0.000, 0.006), "cap_adapter"), material=gunmetal, name="cap_adapter")
    cap.visual(Cylinder(radius=0.018, length=0.004), origin=Origin(xyz=(0.0, 0.0, 0.058)), material=brass, name="top_service_plate")
    cap.visual(Box((0.020, 0.006, 0.018)), origin=Origin(xyz=(0.0, 0.0395, 0.031)), material=gunmetal, name="cap_hatch")
    for i in range(20):
        theta = 2.0 * pi * i / 20.0
        _add_radial_box(cap, radius=0.0395, theta=theta, size=(0.0040, 0.0040, 0.041), z=0.027, material=gunmetal, name=f"grip_rib_{i}")
    for i in range(6):
        theta = 2.0 * pi * i / 6.0
        cap.visual(Sphere(0.0023), origin=Origin(xyz=(0.022 * cos(theta), 0.022 * sin(theta), 0.057)), material=brass, name=f"top_bolt_{i}")
    for i, x in enumerate((-0.006, 0.006)):
        cap.visual(Sphere(0.0022), origin=Origin(xyz=(x, 0.0430, 0.025)), material=brass, name=f"cap_hatch_bolt_{i}")

    cap_turn = model.articulation(
        "cap_turn",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=cap_spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=0.0, upper=2.0 * pi * CAP_TURNS),
    )
    model.articulation(
        "thread_lift",
        ArticulationType.PRISMATIC,
        parent=cap_spindle,
        child=cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=CAP_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_turn = object_model.get_articulation("cap_turn")
    thread_lift = object_model.get_articulation("thread_lift")

    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="neck_thread",
        outer_elem="cap_thread",
        margin=0.001,
        name="coaxial thread envelopes stay nested",
    )
    ctx.expect_overlap(
        bottle,
        cap,
        axes="z",
        elem_a="neck_thread",
        elem_b="cap_thread",
        min_overlap=0.014,
        name="closed threads have axial engagement",
    )

    closed_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_turn: 2.0 * pi * CAP_TURNS, thread_lift: CAP_TRAVEL}):
        raised_pos = ctx.part_world_position(cap)
        ctx.expect_gap(
            cap,
            bottle,
            axis="z",
            positive_elem="cap_thread",
            negative_elem="neck_thread",
            min_gap=0.006,
            name="unscrewed cap thread lifts above neck thread",
        )

    ctx.check(
        "service pose follows screw pitch",
        closed_pos is not None and raised_pos is not None and raised_pos[2] > closed_pos[2] + CAP_TRAVEL * 0.9,
        details=f"closed={closed_pos}, raised={raised_pos}, expected_travel={CAP_TRAVEL}",
    )
    ctx.check(
        "thread lift range equals two turn lead",
        thread_lift.motion_limits is not None
        and cap_turn.motion_limits is not None
        and thread_lift.motion_limits.upper is not None
        and cap_turn.motion_limits.upper is not None
        and abs(thread_lift.motion_limits.upper / cap_turn.motion_limits.upper - THREAD_PITCH / (2.0 * pi)) < 1e-9,
        details=f"turn_limits={cap_turn.motion_limits}, lift_limits={thread_lift.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
