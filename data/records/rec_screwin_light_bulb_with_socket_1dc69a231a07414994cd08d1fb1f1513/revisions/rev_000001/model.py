from __future__ import annotations

import math

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
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


SOCKET_TOP_Z = 0.045
THREAD_PITCH = 0.0036
THREAD_TRAVEL = 2.0 * THREAD_PITCH
THREAD_TURNS = 4.0 * math.pi


def _shell_of_revolution(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 96,
) -> MeshGeometry:
    """Build a thin lathed shell from paired outer and inner (radius, z) profiles."""

    if len(outer_profile) != len(inner_profile):
        raise ValueError("outer and inner profiles must have the same count")

    geom = MeshGeometry()

    def add_ring(radius: float, z: float) -> list[int]:
        return [
            geom.add_vertex(
                radius * math.cos(2.0 * math.pi * j / segments),
                radius * math.sin(2.0 * math.pi * j / segments),
                z,
            )
            for j in range(segments)
        ]

    outer_rings = [add_ring(r, z) for r, z in outer_profile]
    inner_rings = [add_ring(r, z) for r, z in inner_profile]

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for rings, reverse in ((outer_rings, False), (inner_rings, True)):
        for i in range(len(rings) - 1):
            for j in range(segments):
                a = rings[i][j]
                b = rings[i][(j + 1) % segments]
                c = rings[i + 1][(j + 1) % segments]
                d = rings[i + 1][j]
                if reverse:
                    add_quad(a, d, c, b)
                else:
                    add_quad(a, b, c, d)

    for idx in (0, len(outer_rings) - 1):
        for j in range(segments):
            a = outer_rings[idx][j]
            b = outer_rings[idx][(j + 1) % segments]
            c = inner_rings[idx][(j + 1) % segments]
            d = inner_rings[idx][j]
            if idx == 0:
                add_quad(a, d, c, b)
            else:
                add_quad(a, b, c, d)

    return geom


def _helical_ridge(
    *,
    base_radius: float,
    peak_radius: float,
    z_start: float,
    z_end: float,
    pitch: float,
    band_width: float,
    segments_per_turn: int = 28,
) -> MeshGeometry:
    """A visible raised helical thread band on a cylindrical surface."""

    turns = (z_end - z_start) / pitch
    steps = max(8, int(abs(turns) * segments_per_turn))
    geom = MeshGeometry()
    rings: list[tuple[int, int, int, int]] = []

    for k in range(steps + 1):
        t = k / steps
        theta = 2.0 * math.pi * turns * t
        z = z_start + (z_end - z_start) * t
        row = []
        for radius, z_offset in (
            (base_radius, -0.5 * band_width),
            (peak_radius, -0.5 * band_width),
            (peak_radius, 0.5 * band_width),
            (base_radius, 0.5 * band_width),
        ):
            row.append(
                geom.add_vertex(
                    radius * math.cos(theta),
                    radius * math.sin(theta),
                    z + z_offset,
                )
            )
        rings.append(tuple(row))  # type: ignore[arg-type]

    def add_quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for k in range(steps):
        a0, b0, c0, d0 = rings[k]
        a1, b1, c1, d1 = rings[k + 1]
        add_quad(a0, a1, b1, b0)
        add_quad(b0, b1, c1, c0)
        add_quad(c0, c1, d1, d0)
        add_quad(d0, d1, a1, a0)

    add_quad(*rings[0])
    a, b, c, d = rings[-1]
    add_quad(d, c, b, a)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screw_in_bulb_socket")

    glass = model.material("warm_clear_glass", rgba=(0.88, 0.96, 1.0, 0.38))
    glass_edge = model.material("polished_glass_edge", rgba=(0.72, 0.88, 1.0, 0.46))
    nickel = model.material("satin_nickel", rgba=(0.72, 0.70, 0.64, 1.0))
    nickel_high = model.material("thread_highlight", rgba=(0.88, 0.82, 0.68, 1.0))
    brass = model.material("brushed_brass", rgba=(0.96, 0.72, 0.36, 1.0))
    ceramic = model.material("matte_black_ceramic", rgba=(0.025, 0.026, 0.028, 1.0))
    soft_black = model.material("satin_black_break", rgba=(0.08, 0.08, 0.075, 1.0))
    steel = model.material("restrained_steel", rgba=(0.56, 0.57, 0.58, 1.0))
    filament = model.material("warm_tungsten", rgba=(1.0, 0.58, 0.18, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_geometry(
            _shell_of_revolution(
                [
                    (0.026, -0.009),
                    (0.029, -0.006),
                    (0.026, 0.000),
                    (0.024, 0.010),
                    (0.023, 0.036),
                    (0.026, 0.044),
                    (0.025, 0.048),
                ],
                [
                    (0.008, -0.006),
                    (0.011, 0.000),
                    (0.015, 0.008),
                    (0.0153, 0.030),
                    (0.0155, 0.040),
                    (0.0167, 0.046),
                    (0.0168, 0.048),
                ],
                segments=112,
            ),
            "socket_ceramic_shell",
        ),
        material=ceramic,
        name="socket_shell",
    )
    socket.visual(
        mesh_from_geometry(
            _shell_of_revolution(
                [(0.0158, 0.007), (0.0161, 0.011), (0.0161, 0.039), (0.0165, 0.043)],
                [(0.0145, 0.008), (0.0148, 0.012), (0.0148, 0.038), (0.0152, 0.041)],
                segments=112,
            ),
            "socket_brass_liner",
        ),
        material=brass,
        name="socket_liner",
    )
    socket.visual(
        mesh_from_geometry(
            _helical_ridge(
                base_radius=0.01495,
                peak_radius=0.0140,
                z_start=0.012,
                z_end=0.039,
                pitch=THREAD_PITCH,
                band_width=0.0013,
                segments_per_turn=30,
            ),
            "socket_internal_thread",
        ),
        material=brass,
        name="internal_thread",
    )
    socket.visual(
        mesh_from_geometry(TorusGeometry(0.0208, 0.0012, radial_segments=96, tubular_segments=12), "socket_lip_bead"),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=soft_black,
        name="lip_bead",
    )
    socket.visual(
        Cylinder(radius=0.029, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=soft_black,
        name="base_plinth",
    )
    socket.visual(
        Cylinder(radius=0.0152, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=soft_black,
        name="contact_boss",
    )
    socket.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=brass,
        name="center_contact",
    )
    for idx, sx in enumerate((-1.0, 1.0)):
        socket.visual(
            Cylinder(radius=0.003, length=0.003),
            origin=Origin(xyz=(sx * 0.0245, 0.0, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"set_screw_{idx}",
        )
        socket.visual(
            Box((0.0008, 0.0048, 0.0007)),
            origin=Origin(xyz=(sx * 0.0264, 0.0, 0.021)),
            material=soft_black,
            name=f"screw_slot_{idx}",
        )

    # A tiny coaxial carrier encodes the screw rotation axis; it is hidden by the
    # collar and remains axisymmetric so the visible bulb/socket read as the mechanism.
    thread_axis = model.part("thread_axis")
    thread_axis.visual(
        Cylinder(radius=0.002, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=brass,
        name="axis_bushing",
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0128, length=0.027),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=nickel,
        name="screw_sleeve",
    )
    bulb.visual(
        mesh_from_geometry(
            _helical_ridge(
                base_radius=0.0127,
                peak_radius=0.0135,
                z_start=-0.030,
                z_end=-0.006,
                pitch=THREAD_PITCH,
                band_width=0.00155,
                segments_per_turn=32,
            ),
            "bulb_external_thread",
        ),
        material=nickel_high,
        name="external_thread",
    )
    bulb.visual(
        Cylinder(radius=0.0072, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, -0.03225)),
        material=soft_black,
        name="contact_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0050, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0355)),
        material=brass,
        name="base_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0142, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=soft_black,
        name="neck_seam_band",
    )
    bulb.visual(
        mesh_from_geometry(TorusGeometry(0.0134, 0.0008, radial_segments=96, tubular_segments=10), "glass_crimp_seam"),
        origin=Origin(xyz=(0.0, 0.0, -0.0008)),
        material=nickel_high,
        name="crimp_seam",
    )
    bulb.visual(
        mesh_from_geometry(
            _shell_of_revolution(
                [
                    (0.0105, 0.000),
                    (0.0140, 0.008),
                    (0.0260, 0.025),
                    (0.0300, 0.047),
                    (0.0270, 0.062),
                    (0.0180, 0.077),
                    (0.0060, 0.088),
                    (0.0020, 0.091),
                ],
                [
                    (0.0093, 0.001),
                    (0.0128, 0.009),
                    (0.0248, 0.025),
                    (0.0288, 0.047),
                    (0.0258, 0.062),
                    (0.0168, 0.077),
                    (0.0049, 0.087),
                    (0.0009, 0.090),
                ],
                segments=128,
            ),
            "pear_glass_envelope",
        ),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        mesh_from_geometry(TorusGeometry(0.0285, 0.00055, radial_segments=128, tubular_segments=8), "glass_equator_seam"),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=glass_edge,
        name="equator_seam",
    )
    bulb.visual(
        Cylinder(radius=0.0022, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=glass_edge,
        name="glass_stem",
    )
    bulb.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-0.004, 0.0, 0.001),
                    (-0.006, 0.0, 0.022),
                    (-0.006, 0.0, 0.038),
                    (0.006, 0.0, 0.038),
                    (0.006, 0.0, 0.022),
                    (0.004, 0.0, 0.001),
                ],
                radius=0.00070,
                samples_per_segment=12,
                radial_segments=10,
                cap_ends=True,
            ),
            "filament_leads",
        ),
        material=filament,
        name="filament_leads",
    )
    coil_points = [
        (
            -0.0048 + 0.0096 * i / 72,
            0.00065 * math.cos(2.0 * math.pi * 7.0 * i / 72),
            0.038 + 0.00065 * math.sin(2.0 * math.pi * 7.0 * i / 72),
        )
        for i in range(73)
    ]
    bulb.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                coil_points,
                radius=0.00035,
                samples_per_segment=1,
                radial_segments=8,
                cap_ends=True,
            ),
            "coiled_tungsten_filament",
        ),
        material=filament,
        name="filament_coil",
    )
    bulb.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.0045, 0.0, 0.001), (0.0, 0.0, -0.001), (0.0045, 0.0, 0.001)],
                radius=0.00075,
                samples_per_segment=10,
                radial_segments=10,
                cap_ends=True,
            ),
            "filament_anchor_bridge",
        ),
        material=glass_edge,
        name="filament_anchor",
    )

    model.articulation(
        "screw_rotation",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=thread_axis,
        origin=Origin(xyz=(0.0, 0.0, SOCKET_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=THREAD_TURNS, effort=1.2, velocity=1.0),
        motion_properties=MotionProperties(damping=0.08, friction=0.04),
    )
    model.articulation(
        "thread_advance",
        ArticulationType.PRISMATIC,
        parent=thread_axis,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=THREAD_TRAVEL, effort=18.0, velocity=0.015),
        motion_properties=MotionProperties(damping=0.4, friction=0.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    screw = object_model.get_articulation("screw_rotation")
    advance = object_model.get_articulation("thread_advance")

    ctx.check("coaxial screw axis", screw.axis == (0.0, 0.0, 1.0), details=f"axis={screw.axis}")
    ctx.check(
        "thread advance shares axis",
        advance.axis == (0.0, 0.0, 1.0),
        details=f"axis={advance.axis}",
    )

    with ctx.pose({screw: 0.0}):
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="external_thread",
            outer_elem="socket_liner",
            margin=0.001,
            name="external thread centered inside liner",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="screw_sleeve",
            elem_b="socket_liner",
            min_overlap=0.024,
            name="seated screw has practical engagement",
        )
        ctx.expect_contact(
            bulb,
            socket,
            elem_a="base_contact",
            elem_b="center_contact",
            contact_tol=0.0002,
            name="base contact seats on socket contact",
        )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({screw: THREAD_TURNS, advance: THREAD_TRAVEL}):
        raised_pos = ctx.part_world_position(bulb)
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="screw_sleeve",
            elem_b="socket_liner",
            min_overlap=0.016,
            name="unscrewed pose remains retained in collar",
        )
    ctx.check(
        "thread rotation advances bulb outward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.0065,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
