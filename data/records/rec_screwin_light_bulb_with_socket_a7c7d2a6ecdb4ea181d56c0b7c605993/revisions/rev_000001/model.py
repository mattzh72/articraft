from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _lathe_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 80,
) -> MeshGeometry:
    """Build a hollow turned shell from (radius, z) profiles."""

    geom = MeshGeometry()

    def ring(radius: float, z: float) -> list[int]:
        return [
            geom.add_vertex(
                radius * math.cos(2.0 * math.pi * i / segments),
                radius * math.sin(2.0 * math.pi * i / segments),
                z,
            )
            for i in range(segments)
        ]

    outer = [ring(r, z) for r, z in outer_profile]
    inner = [ring(r, z) for r, z in inner_profile]

    def connect(a: list[int], b: list[int], *, reverse: bool = False) -> None:
        for i in range(segments):
            j = (i + 1) % segments
            if reverse:
                geom.add_face(a[i], b[j], b[i])
                geom.add_face(a[i], a[j], b[j])
            else:
                geom.add_face(a[i], b[i], b[j])
                geom.add_face(a[i], b[j], a[j])

    for a, b in zip(outer, outer[1:]):
        connect(a, b)
    for a, b in zip(inner, inner[1:]):
        connect(a, b, reverse=True)

    # Annular end faces make thick lips/end caps while preserving the hollow bore.
    connect(outer[0], inner[0], reverse=True)
    connect(outer[-1], inner[-1])
    return geom


def _helical_strip(
    *,
    radius_inner: float,
    radius_outer: float,
    z_start: float,
    z_end: float,
    turns: float,
    axial_width: float,
    phase: float = 0.0,
    segments_per_turn: int = 36,
) -> MeshGeometry:
    """A raised rectangular helical thread strip wrapped around the Z axis."""

    geom = MeshGeometry()
    steps = max(8, int(abs(turns) * segments_per_turn))
    rings: list[list[int]] = []
    for step in range(steps + 1):
        t = step / steps
        theta = phase + 2.0 * math.pi * turns * t
        z = z_start + (z_end - z_start) * t
        c = math.cos(theta)
        s = math.sin(theta)
        corners = [
            (radius_inner, z - axial_width / 2.0),
            (radius_outer, z - axial_width / 2.0),
            (radius_outer, z + axial_width / 2.0),
            (radius_inner, z + axial_width / 2.0),
        ]
        rings.append([geom.add_vertex(r * c, r * s, zz) for r, zz in corners])

    for a, b in zip(rings, rings[1:]):
        for k in range(4):
            n = (k + 1) % 4
            geom.add_face(a[k], b[k], b[n])
            geom.add_face(a[k], b[n], a[n])

    # Cap the two cut ends of the helical rib.
    start = rings[0]
    end = rings[-1]
    geom.add_face(start[0], start[1], start[2])
    geom.add_face(start[0], start[2], start[3])
    geom.add_face(end[0], end[2], end[1])
    geom.add_face(end[0], end[3], end[2])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_socket")

    ceramic = model.material("warm_ceramic", rgba=(0.86, 0.80, 0.68, 1.0))
    nickel = model.material("brushed_nickel", rgba=(0.68, 0.68, 0.64, 1.0))
    brass = model.material("brass_contact", rgba=(0.95, 0.68, 0.25, 1.0))
    glass = model.material("pale_glass", rgba=(0.70, 0.88, 1.0, 0.34))
    tungsten = model.material("dark_tungsten", rgba=(0.18, 0.15, 0.12, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_geometry(
            _lathe_shell(
                outer_profile=[
                    (0.030, 0.000),
                    (0.040, 0.010),
                    (0.041, 0.055),
                    (0.037, 0.088),
                    (0.034, 0.102),
                ],
                inner_profile=[
                    (0.012, 0.012),
                    (0.017, 0.026),
                    (0.030, 0.055),
                    (0.030, 0.094),
                    (0.027, 0.102),
                ],
            ),
            "ceramic_socket_body",
        ),
        material=ceramic,
        name="ceramic_body",
    )
    socket.visual(
        mesh_from_geometry(
            _lathe_shell(
                outer_profile=[
                    (0.032, 0.030),
                    (0.027, 0.036),
                    (0.027, 0.088),
                    (0.031, 0.091),
                    (0.031, 0.094),
                ],
                inner_profile=[
                    (0.021, 0.030),
                    (0.021, 0.036),
                    (0.021, 0.088),
                    (0.021, 0.091),
                    (0.021, 0.094),
                ],
            ),
            "socket_threaded_collar",
        ),
        material=nickel,
        name="threaded_collar",
    )
    socket.visual(
        mesh_from_geometry(
            _helical_strip(
                radius_inner=0.0189,
                radius_outer=0.0214,
                z_start=0.041,
                z_end=0.085,
                turns=4.15,
                axial_width=0.0021,
                phase=math.pi / 5.0,
            ),
            "socket_internal_thread",
        ),
        material=nickel,
        name="internal_thread",
    )
    socket.visual(
        Cylinder(radius=0.0130, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=ceramic,
        name="insulator_pedestal",
    )
    socket.visual(
        Cylinder(radius=0.0065, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=brass,
        name="contact_post",
    )
    socket.visual(
        Cylinder(radius=0.0090, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=brass,
        name="center_contact",
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0146, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=nickel,
        name="thread_core",
    )
    bulb.visual(
        mesh_from_geometry(
            _helical_strip(
                radius_inner=0.0140,
                radius_outer=0.0171,
                z_start=-0.047,
                z_end=0.004,
                turns=5.10,
                axial_width=0.0022,
                phase=0.0,
            ),
            "bulb_external_thread",
        ),
        material=nickel,
        name="external_thread",
    )
    bulb.visual(
        Cylinder(radius=0.0180, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=nickel,
        name="base_shoulder",
    )
    bulb.visual(
        Cylinder(radius=0.0140, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=nickel,
        name="neck_band",
    )
    bulb.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=brass,
        name="tip_contact",
    )
    bulb.visual(
        mesh_from_geometry(
            _lathe_shell(
                outer_profile=[
                    (0.013, 0.010),
                    (0.017, 0.020),
                    (0.031, 0.038),
                    (0.043, 0.065),
                    (0.046, 0.092),
                    (0.039, 0.119),
                    (0.020, 0.142),
                    (0.004, 0.154),
                ],
                inner_profile=[
                    (0.010, 0.014),
                    (0.014, 0.023),
                    (0.028, 0.041),
                    (0.040, 0.067),
                    (0.043, 0.091),
                    (0.036, 0.116),
                    (0.017, 0.137),
                    (0.002, 0.148),
                ],
            ),
            "glass_envelope",
        ),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0032, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=glass,
        name="glass_stem",
    )
    bulb.visual(
        Cylinder(radius=0.0007, length=0.050),
        origin=Origin(xyz=(-0.0055, 0.0, 0.042)),
        material=tungsten,
        name="support_wire_0",
    )
    bulb.visual(
        Cylinder(radius=0.0007, length=0.050),
        origin=Origin(xyz=(0.0055, 0.0, 0.042)),
        material=tungsten,
        name="support_wire_1",
    )
    bulb.visual(
        Cylinder(radius=0.0062, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.067), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tungsten,
        name="filament_loop",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("socket_to_bulb")

    ctx.check(
        "bulb has continuous screw-axis rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="thread_core",
        outer_elem="threaded_collar",
        margin=0.0,
        name="threaded base sits inside socket collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="thread_core",
        elem_b="threaded_collar",
        min_overlap=0.040,
        name="screw base remains deeply inserted",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="tip_contact",
        negative_elem="center_contact",
        max_gap=0.001,
        max_penetration=0.00005,
        name="bulb tip seats on socket contact",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: math.pi * 1.5}):
        turned_pos = ctx.part_world_position(bulb)
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="thread_core",
            outer_elem="threaded_collar",
            margin=0.0,
            name="rotated bulb stays coaxial in collar",
        )
    ctx.check(
        "continuous spin keeps bulb on shared axis",
        rest_pos is not None and turned_pos is not None and rest_pos == turned_pos,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
