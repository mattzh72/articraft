from __future__ import annotations

from math import cos, pi, sin, tau

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    mesh_from_cadquery,
)


def _helix(radius: float, radius_eps: float, pitch: float, height: float, d: float = 0.0, frac: float = 0.08):
    """Thread path with short eased lead-in/lead-out so the rib does not end square."""

    def func(t: float):
        if frac < t < 1.0 - frac:
            z = height * t + d
            r = radius + radius_eps
        elif t <= frac:
            z = height * t + d * sin(pi / 2.0 * t / frac)
            r = radius + radius_eps * sin(pi / 2.0 * t / frac)
        else:
            z = height * t - d * sin(2.0 * pi - pi / 2.0 * (1.0 - t) / frac)
            r = radius - radius_eps * sin(2.0 * pi - pi / 2.0 * (1.0 - t) / frac)

        x = r * sin(-2.0 * pi / (pitch / height) * t)
        y = r * sin(pi / 2.0 + 2.0 * pi / (pitch / height) * t)
        return x, y, z

    return func


def _thread_solid(radius: float, pitch: float, height: float, radial_depth: float) -> cq.Solid:
    """Make a small helical rectangular thread tooth around the Z axis."""
    half_pitch = pitch / 4.0
    e1_bottom = cq.Workplane("XY").parametricCurve(_helix(radius, 0.0, pitch, height, -half_pitch)).val()
    e1_top = cq.Workplane("XY").parametricCurve(_helix(radius, 0.0, pitch, height, half_pitch)).val()
    e2_bottom = cq.Workplane("XY").parametricCurve(_helix(radius, radial_depth, pitch, height, -half_pitch / 8.0)).val()
    e2_top = cq.Workplane("XY").parametricCurve(_helix(radius, radial_depth, pitch, height, half_pitch / 8.0)).val()

    faces = [
        cq.Face.makeRuledSurface(e1_bottom, e1_top),
        cq.Face.makeRuledSurface(e2_bottom, e2_top),
        cq.Face.makeRuledSurface(e1_bottom, e2_bottom),
        cq.Face.makeRuledSurface(e1_top, e2_top),
    ]
    return cq.Solid.makeSolid(cq.Shell.makeShell(faces))


def _ring(radius_outer: float, radius_inner: float, height: float, z_min: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius_outer)
        .circle(radius_inner)
        .extrude(height)
        .translate((0.0, 0.0, z_min))
    )


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _partial_threaded_ring_geometry(
    outer_radius: float,
    inner_radius: float,
    thread_depth: float,
    z_min: float,
    z_max: float,
    *,
    pitch: float = 0.004,
    angle_start: float = -5.0 * pi / 6.0,
    angle_end: float = 5.0 * pi / 6.0,
    radial_segments: int = 72,
    height_segments: int = 84,
) -> MeshGeometry:
    """Open C-shaped collar with a continuous helical rib on its inner wall."""
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for j in range(height_segments + 1):
        z = z_min + (z_max - z_min) * j / height_segments
        outer_row: list[int] = []
        inner_row: list[int] = []
        for i in range(radial_segments + 1):
            theta = angle_start + (angle_end - angle_start) * i / radial_segments
            phase = ((z - z_min) / pitch - theta / tau) % 1.0
            tooth = 1.0 - abs(2.0 * phase - 1.0)
            r_inner = inner_radius - thread_depth * tooth
            outer_row.append(geom.add_vertex(outer_radius * cos(theta), outer_radius * sin(theta), z))
            inner_row.append(geom.add_vertex(r_inner * cos(theta), r_inner * sin(theta), z))
        outer.append(outer_row)
        inner.append(inner_row)

    for j in range(height_segments):
        for i in range(radial_segments):
            _add_quad(geom, outer[j][i], outer[j][i + 1], outer[j + 1][i + 1], outer[j + 1][i])
            _add_quad(geom, inner[j][i + 1], inner[j][i], inner[j + 1][i], inner[j + 1][i + 1])

    for i in range(radial_segments):
        _add_quad(geom, outer[0][i + 1], outer[0][i], inner[0][i], inner[0][i + 1])
        _add_quad(geom, outer[-1][i], outer[-1][i + 1], inner[-1][i + 1], inner[-1][i])

    for j in range(height_segments):
        _add_quad(geom, outer[j][0], outer[j + 1][0], inner[j + 1][0], inner[j][0])
        _add_quad(geom, outer[j + 1][-1], outer[j][-1], inner[j][-1], inner[j + 1][-1])
    return geom


def _partial_socket_shell_geometry() -> MeshGeometry:
    """Dark phenolic cutaway socket body, open at one side to show the threaded insert."""
    body = _partial_threaded_ring_geometry(
        outer_radius=0.0275,
        inner_radius=0.0190,
        thread_depth=0.0,
        z_min=0.000,
        z_max=0.045,
        radial_segments=60,
        height_segments=3,
    )
    top_lip = _partial_threaded_ring_geometry(
        outer_radius=0.0290,
        inner_radius=0.0188,
        thread_depth=0.0,
        z_min=0.039,
        z_max=0.046,
        radial_segments=60,
        height_segments=1,
    )
    bottom_lip = _partial_threaded_ring_geometry(
        outer_radius=0.0290,
        inner_radius=0.0188,
        thread_depth=0.0,
        z_min=0.000,
        z_max=0.006,
        radial_segments=60,
        height_segments=1,
    )
    return body.merge(top_lip).merge(bottom_lip)


def _threaded_bulb_base_geometry() -> MeshGeometry:
    """One connected corrugated screw shell mesh with a helical thread profile."""
    geom = MeshGeometry()
    z_min = 0.009
    z_max = 0.053
    root_radius = 0.0119
    crest_radius = 0.0132
    pitch = 0.004
    radial_segments = 96
    height_segments = 104
    side: list[list[int]] = []
    for j in range(height_segments + 1):
        z = z_min + (z_max - z_min) * j / height_segments
        row: list[int] = []
        for i in range(radial_segments + 1):
            theta = tau * i / radial_segments
            phase = ((z - z_min) / pitch - theta / tau) % 1.0
            tooth = 1.0 - abs(2.0 * phase - 1.0)
            r = root_radius + (crest_radius - root_radius) * tooth
            row.append(geom.add_vertex(r * cos(theta), r * sin(theta), z))
        side.append(row)

    for j in range(height_segments):
        for i in range(radial_segments):
            _add_quad(geom, side[j][i], side[j][i + 1], side[j + 1][i + 1], side[j + 1][i])

    top_center = geom.add_vertex(0.0, 0.0, z_max)
    bottom_center = geom.add_vertex(0.0, 0.0, z_min)
    for i in range(radial_segments):
        geom.add_face(top_center, side[-1][i], side[-1][i + 1])
        geom.add_face(bottom_center, side[0][i + 1], side[0][i])
    return geom


def _glass_envelope_geometry() -> LatheGeometry:
    outer_profile = [
        (0.0136, 0.052),
        (0.0142, 0.064),
        (0.0200, 0.072),
        (0.0287, 0.087),
        (0.0282, 0.101),
        (0.0180, 0.115),
        (0.0040, 0.122),
        (0.0010, 0.1225),
    ]
    inner_profile = [
        (0.0120, 0.0536),
        (0.0127, 0.0650),
        (0.0184, 0.0740),
        (0.0269, 0.0876),
        (0.0264, 0.1004),
        (0.0165, 0.1132),
        (0.0048, 0.1202),
        (0.0022, 0.1206),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=80,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )


def _socket_shell() -> cq.Workplane:
    shell_height = 0.045
    floor_height = 0.006
    outer = cq.Workplane("XY").circle(0.026).extrude(shell_height)
    bore = cq.Workplane("XY").circle(0.0190).extrude(shell_height + 0.004).translate((0.0, 0.0, floor_height))
    cup = outer.cut(bore)

    top_lip = _ring(0.028, 0.0190, 0.005, shell_height - 0.005)
    bottom_flange = cq.Workplane("XY").circle(0.028).extrude(0.005)
    cord_boss = (
        cq.Workplane("XY")
        .box(0.018, 0.010, 0.010)
        .translate((0.0, -0.026, 0.008))
    )
    return cup.union(top_lip).union(bottom_flange).union(cord_boss)


def _socket_collar() -> cq.Workplane:
    collar = _ring(0.0194, 0.0147, 0.037, 0.007)
    internal_thread = (
        cq.Workplane("XY")
        .add(_thread_solid(0.0147, 0.0040, 0.029, -0.00075))
        .translate((0.0, 0.0, 0.012))
    )
    seating_rim = _ring(0.0200, 0.0144, 0.003, 0.041)
    return collar.union(internal_thread).union(seating_rim)


def _bulb_base() -> cq.Workplane:
    core = cq.Workplane("XY").circle(0.0122).extrude(0.043).translate((0.0, 0.0, 0.010))
    external_thread = (
        cq.Workplane("XY")
        .add(_thread_solid(0.0122, 0.0040, 0.031, 0.00105))
        .translate((0.0, 0.0, 0.016))
    )
    top_crimp = _ring(0.0130, 0.0045, 0.004, 0.049)
    bottom_insulator = cq.Workplane("XY").circle(0.0078).extrude(0.004).translate((0.0, 0.0, 0.007))
    contact_button = cq.Workplane("XY").circle(0.0048).extrude(0.003).translate((0.0, 0.0, 0.0055))
    return core.union(external_thread).union(top_crimp).union(bottom_insulator).union(contact_button)


def _glass_envelope() -> cq.Workplane:
    # Closed thin-wall pear profile in the XZ plane, revolved around the socket axis.
    outer = [
        (0.0136, 0.052),
        (0.0142, 0.064),
        (0.0200, 0.072),
        (0.0287, 0.087),
        (0.0282, 0.101),
        (0.0180, 0.115),
        (0.0040, 0.122),
        (0.0010, 0.1225),
    ]
    inner = [
        (0.0022, 0.1206),
        (0.0048, 0.1202),
        (0.0165, 0.1132),
        (0.0264, 0.1004),
        (0.0269, 0.0876),
        (0.0184, 0.0740),
        (0.0127, 0.0650),
        (0.0120, 0.0536),
    ]
    profile = outer + inner
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .revolve(360.0, axisStart=(0.0, 0.0, 0.0), axisEnd=(0.0, 0.0, 1.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_bulb_socket")

    black = model.material("black_phenolic", rgba=(0.015, 0.013, 0.011, 1.0))
    brass = model.material("brass_threaded_collar", rgba=(0.93, 0.67, 0.27, 1.0))
    nickel = model.material("brushed_nickel_base", rgba=(0.72, 0.74, 0.70, 1.0))
    glass = model.material("warm_clear_glass", rgba=(0.82, 0.94, 1.0, 0.32))
    ceramic = model.material("dark_insulator", rgba=(0.03, 0.026, 0.022, 1.0))
    tungsten = model.material("warm_filament", rgba=(1.0, 0.64, 0.20, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_geometry(_partial_socket_shell_geometry(), "socket_shell"),
        material=black,
        name="socket_shell",
    )
    socket.visual(
        mesh_from_geometry(
            _partial_threaded_ring_geometry(0.0195, 0.0147, 0.00085, 0.007, 0.044),
            "threaded_collar",
        ),
        material=brass,
        name="threaded_collar",
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(_threaded_bulb_base_geometry(), "threaded_base"),
        material=nickel,
        name="threaded_base",
    )
    bulb.visual(
        Cylinder(radius=0.0152, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.04525)),
        material=nickel,
        name="base_seating_flange",
    )
    bulb.visual(
        mesh_from_geometry(_glass_envelope_geometry(), "glass_envelope"),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.0076, length=0.0042),
        origin=Origin(xyz=(0.0, 0.0, 0.0074)),
        material=ceramic,
        name="base_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0046, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, 0.0048)),
        material=nickel,
        name="base_contact",
    )
    bulb.visual(
        Cylinder(radius=0.00038, length=0.027),
        origin=Origin(xyz=(-0.0048, 0.0, 0.0655)),
        material=tungsten,
        name="filament_support_0",
    )
    bulb.visual(
        Cylinder(radius=0.00038, length=0.027),
        origin=Origin(xyz=(0.0048, 0.0, 0.0655)),
        material=tungsten,
        name="filament_support_1",
    )
    bulb.visual(
        Cylinder(radius=0.00062, length=0.0096),
        origin=Origin(xyz=(0.0, 0.0, 0.0790), rpy=(0.0, pi / 2.0, 0.0)),
        material=tungsten,
        name="filament_bridge",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("socket_to_bulb")

    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="threaded_collar",
        margin=0.0,
        name="bulb base is radially captured by collar footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_base",
        elem_b="threaded_collar",
        min_overlap=0.026,
        name="threaded base remains deeply seated in collar",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="base_seating_flange",
        elem_b="threaded_collar",
        contact_tol=0.0002,
        name="bulb flange seats on socket collar lip",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: pi / 2.0}):
        spun_pos = ctx.part_world_position(bulb)
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="threaded_base",
            outer_elem="threaded_collar",
            margin=0.0,
            name="base stays centered while spinning",
        )

    ctx.check(
        "continuous joint spins bulb in place",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-7
        and abs(rest_pos[1] - spun_pos[1]) < 1e-7
        and abs(rest_pos[2] - spun_pos[2]) < 1e-7,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
