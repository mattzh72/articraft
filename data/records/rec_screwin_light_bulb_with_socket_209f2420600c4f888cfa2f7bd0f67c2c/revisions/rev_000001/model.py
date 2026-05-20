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
    tube_from_spline_points,
)


def _merge_meshes(meshes: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for mesh in meshes:
        merged.merge(mesh)
    return merged


def _helix_thread(
    *,
    radius: float,
    tube_radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    phase: float = 0.0,
    points_per_turn: int = 24,
) -> MeshGeometry:
    """A small raised helical tube used as a visual screw thread."""
    count = max(8, int(turns * points_per_turn))
    points: list[tuple[float, float, float]] = []
    for i in range(count + 1):
        u = i / count
        theta = phase + turns * math.tau * u
        points.append(
            (
                radius * math.cos(theta),
                radius * math.sin(theta),
                z_start + (z_end - z_start) * u,
            )
        )
    return tube_from_spline_points(
        points,
        radius=tube_radius,
        samples_per_segment=1,
        radial_segments=10,
        cap_ends=True,
    )


def _socket_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.034, -0.034),
            (0.043, -0.020),
            (0.045, 0.024),
            (0.039, 0.066),
            (0.033, 0.076),
        ],
        [
            (0.020, -0.012),
            (0.025, -0.004),
            (0.027, 0.024),
            (0.026, 0.064),
            (0.029, 0.074),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _metal_collar() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0260, 0.000),
            (0.0270, 0.012),
            (0.0265, 0.052),
            (0.0240, 0.067),
        ],
        [
            (0.0175, 0.002),
            (0.0174, 0.016),
            (0.0178, 0.055),
            (0.0205, 0.067),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )


def _glass_envelope() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0075, 0.057),
            (0.0120, 0.064),
            (0.0220, 0.080),
            (0.0300, 0.108),
            (0.0310, 0.135),
            (0.0250, 0.158),
            (0.0130, 0.179),
            (0.0025, 0.190),
        ],
        [
            (0.0050, 0.060),
            (0.0095, 0.067),
            (0.0190, 0.083),
            (0.0270, 0.110),
            (0.0278, 0.134),
            (0.0220, 0.155),
            (0.0110, 0.174),
            (0.0016, 0.184),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _filament_supports() -> MeshGeometry:
    stem = CylinderGeometry(radius=0.0017, height=0.054, radial_segments=12).translate(
        0.0, 0.0, 0.086
    )
    left_support = tube_from_spline_points(
        [(0.0, 0.0, 0.108), (-0.0045, 0.0, 0.120), (-0.0075, 0.0, 0.126)],
        radius=0.00055,
        radial_segments=8,
        cap_ends=True,
    )
    right_support = tube_from_spline_points(
        [(0.0, 0.0, 0.108), (0.0045, 0.0, 0.120), (0.0075, 0.0, 0.126)],
        radius=0.00055,
        radial_segments=8,
        cap_ends=True,
    )
    filament = tube_from_spline_points(
        [
            (-0.0075, 0.0, 0.126),
            (-0.0050, 0.0020, 0.128),
            (-0.0025, -0.0020, 0.128),
            (0.0000, 0.0020, 0.128),
            (0.0025, -0.0020, 0.128),
            (0.0050, 0.0020, 0.128),
            (0.0075, 0.0, 0.126),
        ],
        radius=0.00065,
        samples_per_segment=4,
        radial_segments=8,
        cap_ends=True,
    )
    return _merge_meshes([stem, left_support, right_support, filament])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_socket")

    ceramic = model.material("warm_ceramic", rgba=(0.86, 0.82, 0.72, 1.0))
    brass = model.material("brass", rgba=(0.83, 0.64, 0.34, 1.0))
    dark_bakelite = model.material("dark_bakelite", rgba=(0.055, 0.050, 0.045, 1.0))
    glass = model.material("clear_warm_glass", rgba=(0.86, 0.95, 1.0, 0.36))
    nickel = model.material("threaded_nickel", rgba=(0.70, 0.72, 0.68, 1.0))
    tungsten = model.material("warm_tungsten", rgba=(1.0, 0.74, 0.32, 1.0))
    black_insulator = model.material("black_insulator", rgba=(0.025, 0.023, 0.020, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_geometry(_socket_shell(), "socket_ceramic_shell"),
        material=ceramic,
        name="ceramic_shell",
    )
    socket.visual(
        mesh_from_geometry(_metal_collar(), "socket_metal_collar"),
        material=brass,
        name="metal_collar",
    )
    socket.visual(
        mesh_from_geometry(
            _helix_thread(
                radius=0.0184,
                tube_radius=0.00075,
                z_start=0.007,
                z_end=0.061,
                turns=7.2,
                phase=0.55,
            ),
            "socket_internal_thread",
        ),
        material=brass,
        name="internal_thread",
    )
    socket.visual(
        Cylinder(radius=0.0064, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material=brass,
        name="bottom_contact",
    )
    socket.visual(
        Cylinder(radius=0.021, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=black_insulator,
        name="contact_well",
    )
    socket.visual(
        Box((0.076, 0.076, 0.074)),
        origin=Origin(xyz=(-0.076, 0.0, -0.004)),
        material=dark_bakelite,
        name="offset_mount",
    )
    socket.visual(
        Box((0.014, 0.112, 0.120)),
        origin=Origin(xyz=(-0.120, 0.0, 0.000)),
        material=dark_bakelite,
        name="wall_plate",
    )
    socket.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(-0.145, 0.0, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_bakelite,
        name="cable_entry",
    )
    socket.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(-0.110, 0.035, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nickel,
        name="mount_screw",
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(_glass_envelope(), "bulb_glass_envelope"),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        Cylinder(radius=0.01345, length=0.051),
        origin=Origin(xyz=(0.0, 0.0, 0.0285)),
        material=nickel,
        name="threaded_base",
    )
    bulb.visual(
        mesh_from_geometry(
            _helix_thread(
                radius=0.0144,
                tube_radius=0.00095,
                z_start=0.006,
                z_end=0.053,
                turns=7.1,
            ),
            "bulb_external_thread",
        ),
        material=nickel,
        name="external_thread",
    )
    bulb.visual(
        Cylinder(radius=0.0097, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=black_insulator,
        name="insulator_ring",
    )
    bulb.visual(
        Cylinder(radius=0.0057, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=brass,
        name="center_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0120, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=nickel,
        name="base_crimp",
    )
    bulb.visual(
        mesh_from_geometry(_filament_supports(), "bulb_filament_supports"),
        material=tungsten,
        name="filament_supports",
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
        "bulb uses continuous screw axis",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="center_contact",
        elem_b="bottom_contact",
        contact_tol=0.0002,
        name="bulb contact is seated on socket contact",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="metal_collar",
        margin=0.0,
        name="threaded base is inside collar footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_base",
        elem_b="metal_collar",
        min_overlap=0.045,
        name="threaded base remains engaged in collar",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: math.pi * 1.5}):
        ctx.expect_contact(
            bulb,
            socket,
            elem_a="center_contact",
            elem_b="bottom_contact",
            contact_tol=0.0002,
            name="rotated bulb stays seated on contact",
        )
        turned_pos = ctx.part_world_position(bulb)
    ctx.check(
        "continuous rotation keeps bulb on shared axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
