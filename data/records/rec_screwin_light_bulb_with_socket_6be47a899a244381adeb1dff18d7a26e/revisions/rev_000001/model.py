from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


TWIST_TRAVEL = 6.0 * pi


def _lathe_polygon(profile, *, segments: int = 96) -> MeshGeometry:
    """Revolve a closed radial/Z cross-section into a connected mesh shell."""

    points = list(profile)
    if points[0] == points[-1]:
        points = points[:-1]

    geom = MeshGeometry()
    rings: list[list[int]] = []
    for i in range(segments):
        angle = 2.0 * pi * i / segments
        ca = cos(angle)
        sa = sin(angle)
        ring = []
        for radius, z_pos in points:
            ring.append(geom.add_vertex(radius * ca, radius * sa, z_pos))
        rings.append(ring)

    count = len(points)
    for i in range(segments):
        ni = (i + 1) % segments
        for j in range(count):
            nj = (j + 1) % count
            a = rings[i][j]
            b = rings[ni][j]
            c = rings[ni][nj]
            d = rings[i][nj]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _helix_points(
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    *,
    phase: float = 0.0,
    samples_per_turn: int = 20,
) -> list[tuple[float, float, float]]:
    steps = max(8, int(abs(turns) * samples_per_turn))
    pts: list[tuple[float, float, float]] = []
    for i in range(steps + 1):
        u = i / steps
        angle = phase + 2.0 * pi * turns * u
        pts.append((radius * cos(angle), radius * sin(angle), z_start + (z_end - z_start) * u))
    return pts


def _socket_body_mesh() -> MeshGeometry:
    # One continuous porcelain cross-section: wide mounting flange, thick cup wall,
    # rolled top lip, and a raised inner floor around the live contact.
    return _lathe_polygon(
        [
            (0.0140, 0.0060),
            (0.0140, 0.0180),
            (0.0175, 0.0520),
            (0.0200, 0.0590),
            (0.0275, 0.0590),
            (0.0300, 0.0490),
            (0.0300, 0.0060),
            (0.0400, 0.0060),
            (0.0400, 0.0000),
            (0.0150, 0.0000),
        ],
        segments=112,
    )


def _socket_sleeve_mesh() -> MeshGeometry:
    # Thin nickel-plated sleeve with a small rolled rim captured in the ceramic lip.
    return _lathe_polygon(
        [
            (0.0147, 0.0170),
            (0.0147, 0.0538),
            (0.0185, 0.0538),
            (0.0185, 0.0568),
            (0.0165, 0.0568),
            (0.0165, 0.0170),
        ],
        segments=112,
    )


def _glass_envelope_mesh() -> MeshGeometry:
    # Hollow A19-style blown glass envelope.  The cross-section has a real inner
    # wall so the transparent bulb reads as a shell instead of a solid blob.
    return _lathe_polygon(
        [
            (0.0105, -0.0040),
            (0.0115, 0.0050),
            (0.0190, 0.0170),
            (0.0280, 0.0300),
            (0.0310, 0.0430),
            (0.0290, 0.0550),
            (0.0220, 0.0710),
            (0.0100, 0.0840),
            (0.0022, 0.0890),
            (0.0011, 0.0862),
            (0.0080, 0.0807),
            (0.0200, 0.0675),
            (0.0274, 0.0532),
            (0.0290, 0.0430),
            (0.0264, 0.0320),
            (0.0170, 0.0205),
            (0.0090, 0.0062),
            (0.0082, -0.0028),
        ],
        segments=128,
    )


def _bulb_thread_mesh() -> MeshGeometry:
    return tube_from_spline_points(
        _helix_points(0.0128, -0.0250, -0.0045, 5.65, phase=0.45, samples_per_turn=22),
        radius=0.00065,
        samples_per_segment=1,
        radial_segments=12,
        cap_ends=True,
    )


def _socket_thread_mesh() -> MeshGeometry:
    return tube_from_spline_points(
        _helix_points(0.0145, 0.0210, 0.0510, 8.2, phase=1.15, samples_per_turn=20),
        radius=0.00045,
        samples_per_segment=1,
        radial_segments=10,
        cap_ends=True,
    )


def _filament_mesh() -> MeshGeometry:
    # A single continuous fine tube: lead-in wire, coiled tungsten filament, and
    # return wire, so the internal detail is not a floating island.
    pts: list[tuple[float, float, float]] = [
        (-0.0040, 0.0000, 0.0020),
        (-0.0055, 0.0000, 0.0180),
        (-0.0088, 0.0000, 0.0375),
    ]
    coil_radius = 0.00175
    coil_len = 0.0176
    coil_steps = 72
    loops = 5.2
    for i in range(coil_steps + 1):
        u = i / coil_steps
        angle = 2.0 * pi * loops * u
        x = -coil_len / 2.0 + coil_len * u
        pts.append((x, coil_radius * sin(angle), 0.0390 + coil_radius * cos(angle)))
    pts.extend(
        [
            (0.0088, 0.0000, 0.0375),
            (0.0055, 0.0000, 0.0180),
            (0.0040, 0.0000, 0.0020),
        ]
    )
    return tube_from_spline_points(
        pts,
        radius=0.00034,
        samples_per_segment=2,
        radial_segments=8,
        cap_ends=True,
        spline="catmull_rom",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_socket")

    model.material("frosted_glass", rgba=(0.80, 0.94, 1.00, 0.34))
    model.material("glass_stem", rgba=(0.76, 0.91, 1.00, 0.46))
    model.material("warm_tungsten", rgba=(1.00, 0.55, 0.12, 1.00))
    model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.00))
    model.material("nickel", rgba=(0.62, 0.64, 0.63, 1.00))
    model.material("brass", rgba=(0.92, 0.68, 0.30, 1.00))
    model.material("black_insulator", rgba=(0.025, 0.022, 0.020, 1.00))
    model.material("porcelain", rgba=(0.93, 0.91, 0.85, 1.00))
    model.material("slot_shadow", rgba=(0.08, 0.075, 0.065, 1.00))

    socket = model.part("socket")
    socket.visual(
        mesh_from_geometry(_socket_body_mesh(), "socket_body"),
        material="porcelain",
        name="socket_body",
    )
    socket.visual(
        mesh_from_geometry(_socket_sleeve_mesh(), "socket_sleeve"),
        material="nickel",
        name="socket_sleeve",
    )
    socket.visual(
        mesh_from_geometry(_socket_thread_mesh(), "socket_thread"),
        material="nickel",
        name="socket_thread",
    )
    socket.visual(
        Cylinder(radius=0.0066, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0200)),
        material="brass",
        name="center_contact",
    )
    socket.visual(
        Cylinder(radius=0.0033, length=0.0148),
        origin=Origin(xyz=(0.0, 0.0, 0.01125)),
        material="brass",
        name="contact_post",
    )
    socket.visual(
        Cylinder(radius=0.0146, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0030)),
        material="porcelain",
        name="contact_insulator",
    )

    for x_pos in (-0.047, 0.047):
        socket.visual(
            Box((0.026, 0.020, 0.006)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0030)),
            material="porcelain",
            name=f"mount_ear_{0 if x_pos < 0 else 1}",
        )
        socket.visual(
            Cylinder(radius=0.0065, length=0.0030),
            origin=Origin(xyz=(x_pos, 0.0, 0.0075)),
            material="nickel",
            name=f"mount_screw_{0 if x_pos < 0 else 1}",
        )
        socket.visual(
            Box((0.0075, 0.0011, 0.0006)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0092), rpy=(0.0, 0.0, 0.0)),
            material="slot_shadow",
            name=f"screw_slot_{0 if x_pos < 0 else 1}",
        )

    for idx, z_pos in enumerate((0.026, 0.040)):
        socket.visual(
            Box((0.016, 0.007, 0.010)),
            origin=Origin(xyz=(0.0, -0.0320, z_pos)),
            material="brass",
            name=f"terminal_lug_{idx}",
        )
        socket.visual(
            Cylinder(radius=0.0048, length=0.0040),
            origin=Origin(xyz=(0.0, -0.0362, z_pos), rpy=(pi / 2.0, 0.0, 0.0)),
            material="nickel",
            name=f"terminal_screw_{idx}",
        )
        socket.visual(
            Box((0.0070, 0.0006, 0.0010)),
            origin=Origin(xyz=(0.0, -0.0384, z_pos), rpy=(0.0, 0.0, pi / 2.0)),
            material="slot_shadow",
            name=f"terminal_slot_{idx}",
        )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(_glass_envelope_mesh(), "glass_envelope"),
        material="frosted_glass",
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0043, length=0.0170),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material="glass_stem",
        name="glass_stem",
    )
    bulb.visual(
        mesh_from_geometry(_filament_mesh(), "filament"),
        material="warm_tungsten",
        name="filament",
    )
    bulb.visual(
        Cylinder(radius=0.0120, length=0.0240),
        origin=Origin(xyz=(0.0, 0.0, -0.0150)),
        material="brushed_aluminum",
        name="screw_core",
    )
    bulb.visual(
        mesh_from_geometry(_bulb_thread_mesh(), "edison_thread"),
        material="brushed_aluminum",
        name="edison_thread",
    )
    bulb.visual(
        Box((0.0012, 0.0050, 0.0100)),
        origin=Origin(xyz=(0.0128, 0.0, -0.0145)),
        material="slot_shadow",
        name="orientation_mark",
    )
    bulb.visual(
        Cylinder(radius=0.0134, length=0.0042),
        origin=Origin(xyz=(0.0, 0.0, -0.0022)),
        material="brushed_aluminum",
        name="crimp_ring",
    )
    bulb.visual(
        Cylinder(radius=0.0100, length=0.0037),
        origin=Origin(xyz=(0.0, 0.0, -0.0270)),
        material="black_insulator",
        name="insulator_disk",
    )
    bulb.visual(
        Cylinder(radius=0.0057, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, -0.0290)),
        material="brass",
        name="bottom_contact",
    )

    model.articulation(
        "bulb_twist",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0520)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=TWIST_TRAVEL),
        motion_properties=MotionProperties(damping=0.02, friction=0.04),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    twist = object_model.get_articulation("bulb_twist")

    ctx.expect_contact(
        bulb,
        socket,
        elem_a="bottom_contact",
        elem_b="center_contact",
        contact_tol=0.0008,
        name="seated bulb makes center electrical contact",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="screw_core",
        outer_elem="socket_sleeve",
        margin=0.0005,
        name="edison screw base is centered inside socket sleeve",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="screw_core",
        elem_b="socket_sleeve",
        min_overlap=0.020,
        name="threaded base is deeply engaged at rest",
    )

    rest_mark = ctx.part_element_world_aabb(bulb, elem="orientation_mark")
    with ctx.pose({twist: pi / 2.0}):
        turned_mark = ctx.part_element_world_aabb(bulb, elem="orientation_mark")
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="screw_core",
            elem_b="socket_sleeve",
            min_overlap=0.015,
            name="quarter turn still leaves retained thread engagement",
        )

    rest_center = None
    turned_center = None
    if rest_mark is not None:
        rest_center = tuple((rest_mark[0][i] + rest_mark[1][i]) / 2.0 for i in range(3))
    if turned_mark is not None:
        turned_center = tuple((turned_mark[0][i] + turned_mark[1][i]) / 2.0 for i in range(3))
    ctx.check(
        "twist rotates the indexed bulb shell around the socket axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.010
        and abs(rest_center[1]) < 0.004
        and turned_center[1] > 0.010
        and abs(turned_center[0]) < 0.004,
        details=f"rest_mark={rest_center}, turned_mark={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
