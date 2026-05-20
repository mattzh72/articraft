from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_ring(geom: MeshGeometry, radius: float, z: float, segments: int) -> list[int]:
    ring: list[int] = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        ring.append(geom.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
    return ring


def _connect_rings(
    geom: MeshGeometry,
    lower: list[int],
    upper: list[int],
    *,
    inward: bool = False,
) -> None:
    count = len(lower)
    for i in range(count):
        j = (i + 1) % count
        if inward:
            geom.add_face(lower[i], upper[j], upper[i])
            geom.add_face(lower[i], lower[j], upper[j])
        else:
            geom.add_face(lower[i], upper[i], upper[j])
            geom.add_face(lower[i], upper[j], lower[j])


def _lathe_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
) -> MeshGeometry:
    """Thin, capped surface of revolution for the hollow bulb and socket collar."""
    geom = MeshGeometry()
    outer_rings = [_add_ring(geom, r, z, segments) for r, z in outer_profile]
    inner_rings = [_add_ring(geom, r, z, segments) for r, z in inner_profile]

    for a, b in zip(outer_rings, outer_rings[1:]):
        _connect_rings(geom, a, b)
    for a, b in zip(inner_rings, inner_rings[1:]):
        _connect_rings(geom, a, b, inward=True)

    # Cap the start and end with annular faces so the shell reads as a molded
    # or blown part rather than two unconnected skins.
    _connect_rings(geom, inner_rings[0], outer_rings[0], inward=True)
    _connect_rings(geom, outer_rings[-1], inner_rings[-1], inward=True)
    return geom


def _helical_thread(
    *,
    z_min: float,
    z_max: float,
    pitch: float,
    root_radius: float,
    crest_radius: float,
    tooth_width: float,
    segments_per_turn: int = 28,
) -> MeshGeometry:
    """A low-cost triangular thread ridge, used for both male and female threads."""
    geom = MeshGeometry()
    turns = (z_max - z_min) / pitch
    steps = max(8, int(turns * segments_per_turn))
    rows: list[list[int]] = []

    for i in range(steps + 1):
        t = i / steps
        theta = 2.0 * math.pi * turns * t
        z = z_min + (z_max - z_min) * t
        cross_section = (
            (root_radius, z - 0.5 * tooth_width),
            (crest_radius, z),
            (root_radius, z + 0.5 * tooth_width),
        )
        row: list[int] = []
        for radius, z_point in cross_section:
            row.append(
                geom.add_vertex(
                    radius * math.cos(theta),
                    radius * math.sin(theta),
                    z_point,
                )
            )
        rows.append(row)

    for row_a, row_b in zip(rows, rows[1:]):
        for j in range(2):
            geom.add_face(row_a[j], row_b[j], row_b[j + 1])
            geom.add_face(row_a[j], row_b[j + 1], row_a[j + 1])

    # Close the two cut ends of the spiral ridge.
    geom.add_face(rows[0][0], rows[0][1], rows[0][2])
    geom.add_face(rows[-1][0], rows[-1][2], rows[-1][1])
    return geom


def _bulb_glass_mesh() -> MeshGeometry:
    # A19-like frosted hollow envelope: broad shoulder, rounded crown, narrow neck.
    return _lathe_shell(
        outer_profile=[
            (0.0098, 0.002),
            (0.0120, 0.007),
            (0.0180, 0.016),
            (0.0255, 0.030),
            (0.0300, 0.047),
            (0.0285, 0.065),
            (0.0210, 0.083),
            (0.0080, 0.096),
            (0.0018, 0.100),
        ],
        inner_profile=[
            (0.0082, 0.0035),
            (0.0102, 0.008),
            (0.0160, 0.017),
            (0.0238, 0.031),
            (0.0283, 0.047),
            (0.0268, 0.064),
            (0.0195, 0.080),
            (0.0068, 0.092),
            (0.0010, 0.096),
        ],
        segments=96,
    )


def _socket_body_mesh() -> MeshGeometry:
    # One molded phenolic part: a wide flange blends into the collar with no
    # separate brackets or ornamental inserts.
    return _lathe_shell(
        outer_profile=[
            (0.0430, 0.000),
            (0.0430, 0.006),
            (0.0300, 0.008),
            (0.0230, 0.018),
            (0.0220, 0.055),
            (0.0250, 0.063),
            (0.0240, 0.067),
        ],
        inner_profile=[
            (0.0130, 0.008),
            (0.0145, 0.016),
            (0.0172, 0.023),
            (0.0172, 0.055),
            (0.0180, 0.063),
            (0.0180, 0.067),
        ],
        segments=96,
    )


def _bulb_screw_mesh() -> MeshGeometry:
    core = CylinderGeometry(0.0117, 0.044, radial_segments=72, closed=True)
    core.translate(0.0, 0.0, -0.016)
    thread = _helical_thread(
        z_min=-0.035,
        z_max=0.004,
        pitch=0.006,
        root_radius=0.0115,
        crest_radius=0.0132,
        tooth_width=0.0022,
    )
    return core.merge(thread)


def _socket_liner_mesh() -> MeshGeometry:
    sleeve = _lathe_shell(
        outer_profile=[(0.0167, 0.018), (0.0167, 0.062)],
        inner_profile=[(0.0149, 0.018), (0.0149, 0.062)],
        segments=96,
    )
    thread = _helical_thread(
        z_min=0.021,
        z_max=0.060,
        pitch=0.006,
        root_radius=0.0152,
        crest_radius=0.0140,
        tooth_width=0.0022,
    )
    return sleeve.merge(thread)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_screw_in_bulb_socket")

    phenolic = Material("black_molded_phenolic", rgba=(0.015, 0.014, 0.012, 1.0))
    brass = Material("stamped_brass_contacts", rgba=(0.95, 0.68, 0.25, 1.0))
    aluminum = Material("rolled_aluminum_base", rgba=(0.76, 0.78, 0.78, 1.0))
    solder = Material("tin_silver_contact", rgba=(0.88, 0.86, 0.80, 1.0))
    glass = Material("frosted_translucent_glass", rgba=(0.86, 0.94, 1.0, 0.42))
    cement = Material("white_cement_ring", rgba=(0.88, 0.86, 0.78, 1.0))
    shadow = Material("recess_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    socket = model.part(
        "socket",
        meta={
            "manufacturing_note": (
                "Single molded flange/collar with a stamped threaded brass sleeve "
                "crimped by tabs; only fixed contacts are added to the root assembly."
            )
        },
    )
    socket.visual(
        mesh_from_geometry(_socket_body_mesh(), "molded_socket_body"),
        material=phenolic,
        name="molded_body",
    )
    socket.visual(
        mesh_from_geometry(_socket_liner_mesh(), "threaded_socket_liner"),
        material=brass,
        name="thread_liner",
    )
    socket.visual(
        Cylinder(0.0060, 0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0190)),
        material=brass,
        name="center_contact",
    )
    socket.visual(
        Cylinder(0.0132, 0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0100)),
        material=phenolic,
        name="contact_boss",
    )
    socket.visual(
        Cylinder(0.0025, 0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0150)),
        material=brass,
        name="contact_rivet",
    )

    # Stamped snap tabs show how the sleeve is retained in the plastic collar.
    for index, (x, y, size) in enumerate(
        (
            (0.0190, 0.0, (0.0060, 0.0030, 0.0100)),
            (-0.0190, 0.0, (0.0060, 0.0030, 0.0100)),
            (0.0, 0.0190, (0.0030, 0.0060, 0.0100)),
            (0.0, -0.0190, (0.0030, 0.0060, 0.0100)),
        )
    ):
        socket.visual(
            Box(size),
            origin=Origin(xyz=(x, y, 0.060)),
            material=brass,
            name=f"liner_tab_{index}",
        )

    # Two bolt-hole shadows and two straight stamped wire pads keep the socket
    # manufacturable and make the assembly direction obvious without extra
    # articulated parts.
    for index, x in enumerate((-0.031, 0.031)):
        socket.visual(
            Cylinder(0.0048, 0.0040),
            origin=Origin(xyz=(x, 0.0, 0.0060)),
            material=shadow,
            name=f"bolt_hole_{index}",
        )
    for index, y in enumerate((-0.033, 0.033)):
        socket.visual(
            Box((0.022, 0.005, 0.0030)),
            origin=Origin(xyz=(0.0, y, 0.0068)),
            material=brass,
            name=f"wire_pad_{index}",
        )

    bulb = model.part(
        "bulb",
        meta={
            "manufacturing_note": (
                "Blown frosted envelope cemented to a rolled screw shell; the "
                "helical ridge shares pitch with the socket liner."
            )
        },
    )
    bulb.visual(
        mesh_from_geometry(_bulb_glass_mesh(), "hollow_frosted_bulb_envelope"),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        mesh_from_geometry(_bulb_screw_mesh(), "external_screw_threads"),
        material=aluminum,
        name="screw_threads",
    )
    bulb.visual(
        Cylinder(0.0105, 0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=cement,
        name="cement_collar",
    )
    bulb.visual(
        Cylinder(0.0060, 0.0050),
        origin=Origin(xyz=(0.0, 0.0, -0.0405)),
        material=solder,
        name="center_button",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0630)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=0.0, upper=2.0 * math.pi),
        meta={
            "mechanism": (
                "Coaxial screw engagement represented by rotational freedom at the "
                "socket mouth; matching visible thread pitch provides the helical constraint."
            )
        },
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    screw_joint = object_model.get_articulation("socket_to_bulb")

    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="screw_threads",
        outer_elem="thread_liner",
        margin=0.001,
        name="screw shell is coaxially captured by socket liner",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="screw_threads",
        elem_b="thread_liner",
        min_overlap=0.030,
        name="threaded shell remains inserted in socket collar",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="center_button",
        elem_b="center_contact",
        contact_tol=0.0005,
        name="center button seats on spring contact",
    )

    rest_position = ctx.part_world_position(bulb)
    with ctx.pose({screw_joint: math.pi}):
        turned_position = ctx.part_world_position(bulb)
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="screw_threads",
            outer_elem="thread_liner",
            margin=0.001,
            name="turned bulb stays coaxial in threaded socket",
        )

    ctx.check(
        "screw joint rotation does not translate bulb off axis",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) < 1e-6
        and abs(rest_position[1] - turned_position[1]) < 1e-6
        and abs(rest_position[2] - turned_position[2]) < 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
