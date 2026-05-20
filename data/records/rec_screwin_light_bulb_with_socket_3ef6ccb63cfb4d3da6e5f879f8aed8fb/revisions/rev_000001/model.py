from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


TAU = 2.0 * math.pi


def _thread_peak(phase: float, width: float = 0.28) -> float:
    """A narrow triangular peak used to make a helical thread ridge."""
    centered = abs((phase % 1.0) - 0.5)
    return max(0.0, 1.0 - centered / width)


def _threaded_cylinder_geometry(
    *,
    base_radius: float,
    ridge_height: float,
    pitch: float,
    z_min: float,
    z_max: float,
    radial_segments: int = 80,
    z_segments: int = 72,
) -> MeshGeometry:
    """Connected screw-shell mesh with a raised external helical thread."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for zi in range(z_segments + 1):
        z = z_min + (z_max - z_min) * zi / z_segments
        ring: list[int] = []
        for ai in range(radial_segments):
            theta = TAU * ai / radial_segments
            phase = theta / TAU - (z - z_min) / pitch
            r = base_radius + ridge_height * _thread_peak(phase)
            ring.append(geom.add_vertex(r * math.cos(theta), r * math.sin(theta), z))
        rings.append(ring)

    for zi in range(z_segments):
        for ai in range(radial_segments):
            a = rings[zi][ai]
            b = rings[zi][(ai + 1) % radial_segments]
            c = rings[zi + 1][(ai + 1) % radial_segments]
            d = rings[zi + 1][ai]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    bottom_center = geom.add_vertex(0.0, 0.0, z_min)
    top_center = geom.add_vertex(0.0, 0.0, z_max)
    for ai in range(radial_segments):
        geom.add_face(bottom_center, rings[0][(ai + 1) % radial_segments], rings[0][ai])
        geom.add_face(top_center, rings[-1][ai], rings[-1][(ai + 1) % radial_segments])

    return geom


def _threaded_sleeve_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    thread_depth: float,
    pitch: float,
    z_min: float,
    z_max: float,
    radial_segments: int = 80,
    z_segments: int = 72,
) -> MeshGeometry:
    """Connected hollow collar with a visible internal helical thread."""
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for zi in range(z_segments + 1):
        z = z_min + (z_max - z_min) * zi / z_segments
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for ai in range(radial_segments):
            theta = TAU * ai / radial_segments
            outer_ring.append(
                geom.add_vertex(
                    outer_radius * math.cos(theta),
                    outer_radius * math.sin(theta),
                    z,
                )
            )
            phase = theta / TAU - (z - z_min) / pitch + 0.5
            r_inner = inner_radius - thread_depth * _thread_peak(phase)
            inner_ring.append(
                geom.add_vertex(
                    r_inner * math.cos(theta),
                    r_inner * math.sin(theta),
                    z,
                )
            )
        outer.append(outer_ring)
        inner.append(inner_ring)

    for zi in range(z_segments):
        for ai in range(radial_segments):
            aj = (ai + 1) % radial_segments
            geom.add_face(outer[zi][ai], outer[zi][aj], outer[zi + 1][aj])
            geom.add_face(outer[zi][ai], outer[zi + 1][aj], outer[zi + 1][ai])
            geom.add_face(inner[zi][aj], inner[zi][ai], inner[zi + 1][ai])
            geom.add_face(inner[zi][aj], inner[zi + 1][ai], inner[zi + 1][aj])

    # Annular caps make the sleeve one connected, open-bore solid.
    for ai in range(radial_segments):
        aj = (ai + 1) % radial_segments
        geom.add_face(outer[0][ai], inner[0][ai], inner[0][aj])
        geom.add_face(outer[0][ai], inner[0][aj], outer[0][aj])
        geom.add_face(outer[-1][aj], inner[-1][aj], inner[-1][ai])
        geom.add_face(outer[-1][aj], inner[-1][ai], outer[-1][ai])

    return geom


def _bulb_envelope_geometry() -> MeshGeometry:
    outer_profile = [
        (0.010, -0.002),
        (0.012, 0.010),
        (0.019, 0.024),
        (0.028, 0.043),
        (0.031, 0.066),
        (0.028, 0.090),
        (0.018, 0.112),
        (0.006, 0.125),
        (0.001, 0.128),
    ]
    inner_profile = [
        (0.0085, -0.0005),
        (0.0105, 0.011),
        (0.0175, 0.025),
        (0.0265, 0.044),
        (0.0295, 0.066),
        (0.0265, 0.089),
        (0.0165, 0.110),
        (0.0050, 0.122),
        (0.0010, 0.124),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _socket_body_geometry() -> MeshGeometry:
    outer_profile = [
        (0.029, -0.076),
        (0.036, -0.064),
        (0.038, -0.040),
        (0.035, -0.010),
        (0.031, 0.006),
    ]
    inner_profile = [
        (0.014, -0.076),
        (0.014, -0.055),
        (0.015, -0.040),
        (0.019, -0.012),
        (0.023, 0.006),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb")

    warm_glass = model.material("warm_clear_glass", rgba=(0.90, 0.96, 1.0, 0.36))
    nickel = model.material("brushed_nickel", rgba=(0.70, 0.72, 0.70, 1.0))
    brass = model.material("warm_brass", rgba=(0.86, 0.61, 0.27, 1.0))
    ceramic = model.material("glazed_ceramic", rgba=(0.92, 0.90, 0.84, 1.0))
    black = model.material("black_insulator", rgba=(0.02, 0.018, 0.014, 1.0))
    tungsten = model.material("warm_filament", rgba=(1.0, 0.56, 0.16, 1.0))
    plate_metal = model.material("painted_wall_plate", rgba=(0.68, 0.70, 0.72, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_geometry(_socket_body_geometry(), "socket_body"),
        material=ceramic,
        name="socket_body",
    )
    socket.visual(
        mesh_from_geometry(
            _threaded_sleeve_geometry(
                outer_radius=0.023,
                inner_radius=0.0156,
                thread_depth=0.0007,
                pitch=0.006,
                z_min=-0.040,
                z_max=0.004,
            ),
            "threaded_collar",
        ),
        material=brass,
        name="threaded_collar",
    )
    socket.visual(
        Cylinder(radius=0.0155, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=black,
        name="bottom_insulator",
    )
    socket.visual(
        Cylinder(radius=0.007, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, -0.04875)),
        material=brass,
        name="socket_contact",
    )
    socket.visual(
        Box((0.120, 0.012, 0.160)),
        origin=Origin(xyz=(0.0, 0.090, -0.022)),
        material=plate_metal,
        name="wall_plate",
    )
    socket.visual(
        Box((0.044, 0.064, 0.030)),
        origin=Origin(xyz=(0.0, 0.058, -0.042)),
        material=plate_metal,
        name="support_arm",
    )
    for screw_z, screw_name in ((0.034, "plate_screw_0"), (-0.088, "plate_screw_1")):
        socket.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(0.0, 0.082, screw_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=nickel,
            name=screw_name,
        )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(_bulb_envelope_geometry(), "glass_envelope"),
        material=warm_glass,
        name="glass_envelope",
    )
    bulb.visual(
        mesh_from_geometry(
            _threaded_cylinder_geometry(
                base_radius=0.0122,
                ridge_height=0.0014,
                pitch=0.006,
                z_min=-0.038,
                z_max=0.000,
            ),
            "threaded_base",
        ),
        material=nickel,
        name="threaded_base",
    )
    bulb.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=black,
        name="base_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0455)),
        material=brass,
        name="center_contact",
    )
    for x, lead_name in ((-0.006, "lead_wire_0"), (0.006, "lead_wire_1")):
        bulb.visual(
            Cylinder(radius=0.00045, length=0.062),
            origin=Origin(xyz=(x, 0.0, 0.029)),
            material=tungsten,
            name=lead_name,
        )
    bulb.visual(
        Cylinder(radius=0.00065, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tungsten,
        name="filament",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("socket_to_bulb")

    ctx.check(
        "bulb uses continuous screw rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="threaded_collar",
        margin=0.0,
        name="threaded base is centered inside collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_base",
        elem_b="threaded_collar",
        min_overlap=0.030,
        name="threaded base remains seated in collar",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="center_contact",
        elem_b="socket_contact",
        contact_tol=0.0005,
        name="bulb center contact seats on socket contact",
    )
    with ctx.pose({spin: math.pi * 1.25}):
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="threaded_base",
            outer_elem="threaded_collar",
            margin=0.0,
            name="rotated bulb stays coaxial in collar",
        )
        ctx.expect_contact(
            bulb,
            socket,
            elem_a="center_contact",
            elem_b="socket_contact",
            contact_tol=0.0005,
            name="rotated bulb keeps bottom contact seated",
        )

    return ctx.report()


object_model = build_object_model()
