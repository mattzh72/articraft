from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _thread_profile(
    *,
    z0: float,
    z1: float,
    root_radius: float,
    crest_radius: float,
    pitch: float,
) -> list[tuple[float, float]]:
    """Small lathe profile with rounded-looking screw-thread crests."""
    profile: list[tuple[float, float]] = [(root_radius, z0)]
    z = z0
    while z < z1 - 1e-9:
        z_next = min(z + pitch, z1)
        z_mid = 0.5 * (z + z_next)
        # The short four-point wave makes individual ridges visible without
        # creating separate, floating ring meshes.
        profile.extend(
            [
                (root_radius, z + 0.0010),
                (crest_radius, z_mid - 0.0010),
                (crest_radius, z_mid + 0.0010),
                (root_radius, z_next - 0.0010),
            ]
        )
        z = z_next
    profile.append((root_radius, z1))
    return profile


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_socket")

    porcelain = model.material("warm_porcelain", rgba=(0.86, 0.82, 0.72, 1.0))
    brass = model.material("brass_threaded_metal", rgba=(0.92, 0.70, 0.34, 1.0))
    dark_metal = model.material("dark_terminal_metal", rgba=(0.18, 0.17, 0.16, 1.0))
    glass = model.material("translucent_warm_glass", rgba=(0.95, 0.92, 0.78, 0.34))
    black = model.material("black_insulator", rgba=(0.015, 0.014, 0.012, 1.0))
    copper = model.material("copper_contact", rgba=(0.82, 0.48, 0.22, 1.0))
    index_red = model.material("red_index_mark", rgba=(0.9, 0.05, 0.02, 1.0))

    socket = model.part("socket")

    # A single hollow porcelain cup/body, slightly tapered like a small lamp
    # socket.  The inner profile leaves a real open bore for the threaded collar.
    socket_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.030, 0.000),
            (0.045, 0.010),
            (0.046, 0.060),
            (0.041, 0.092),
            (0.038, 0.101),
        ],
        inner_profile=[
            (0.000, 0.010),
            (0.018, 0.014),
            (0.028, 0.033),
            (0.028, 0.097),
        ],
        segments=72,
        lip_samples=8,
    )
    socket.visual(
        mesh_from_geometry(socket_shell, "socket_shell"),
        material=porcelain,
        name="socket_shell",
    )

    # The stationary mating threaded collar is embedded in the socket wall and
    # has shallow internal corrugations to read as the female screw thread.
    collar_outer = [(0.0285, 0.030), (0.0285, 0.103)]
    collar_inner = _thread_profile(
        z0=0.034,
        z1=0.099,
        root_radius=0.0237,
        crest_radius=0.0220,
        pitch=0.0070,
    )
    collar = LatheGeometry.from_shell_profiles(
        outer_profile=collar_outer,
        inner_profile=collar_inner,
        segments=72,
        lip_samples=4,
    )
    socket.visual(
        mesh_from_geometry(collar, "threaded_collar"),
        material=brass,
        name="threaded_collar",
    )

    # A brass floor/contact disk inside the cup and a supported external
    # terminal screw give the socket recognizable electrical details.
    socket.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=copper,
        name="bottom_contact",
    )
    socket.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.046, 0.0, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="side_screw_head",
    )
    socket.visual(
        Box((0.014, 0.004, 0.010)),
        origin=Origin(xyz=(0.043, 0.0, 0.056)),
        material=dark_metal,
        name="side_screw_slot",
    )
    socket.inertial = Inertial.from_geometry(Cylinder(radius=0.046, length=0.103), mass=0.16)

    bulb = model.part("bulb")

    # The rotating male screw base is narrower than the glass envelope and sits
    # down inside the socket collar.
    base_outer = _thread_profile(
        z0=0.037,
        z1=0.106,
        root_radius=0.0182,
        crest_radius=0.0207,
        pitch=0.0070,
    )
    base_inner = [(0.0145, 0.039), (0.0145, 0.104)]
    screw_base = LatheGeometry.from_shell_profiles(
        outer_profile=base_outer,
        inner_profile=base_inner,
        segments=72,
        lip_samples=4,
    )
    bulb.visual(
        mesh_from_geometry(screw_base, "screw_base"),
        material=brass,
        name="screw_base",
    )
    bulb.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=copper,
        name="center_contact",
    )
    bulb.visual(
        Cylinder(radius=0.0155, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=black,
        name="contact_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=black,
        name="insulator_ring",
    )

    # A small asymmetric paint/index tab on the rotating base makes the
    # continuous spin legible even though the main envelope is axisymmetric.
    bulb.visual(
        Box((0.004, 0.003, 0.016)),
        origin=Origin(xyz=(0.0212, 0.0, 0.116)),
        material=index_red,
        name="index_mark",
    )

    # Tall, thin-walled pear-shaped glass envelope.  It is explicitly modeled as
    # a shell rather than a solid blob.
    glass_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.013, 0.112),
            (0.020, 0.125),
            (0.031, 0.150),
            (0.039, 0.184),
            (0.038, 0.205),
            (0.025, 0.230),
            (0.006, 0.243),
            (0.000, 0.246),
        ],
        inner_profile=[
            (0.0105, 0.116),
            (0.0175, 0.128),
            (0.0285, 0.151),
            (0.0365, 0.184),
            (0.0355, 0.204),
            (0.0230, 0.226),
            (0.0040, 0.239),
            (0.000, 0.242),
        ],
        segments=96,
        lip_samples=8,
    )
    bulb.visual(
        mesh_from_geometry(glass_shell, "glass_envelope"),
        material=glass,
        name="glass_envelope",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.215),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
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
        inner_elem="screw_base",
        outer_elem="threaded_collar",
        margin=0.0,
        name="male base is centered inside collar footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="screw_base",
        elem_b="threaded_collar",
        min_overlap=0.055,
        name="threaded base remains seated in socket collar",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="center_contact",
        elem_b="bottom_contact",
        contact_tol=0.0005,
        name="bulb center contact seats on socket contact",
    )

    rest_position = ctx.part_world_position(bulb)
    with ctx.pose({spin: math.pi / 2.0}):
        turned_position = ctx.part_world_position(bulb)
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="screw_base",
            outer_elem="threaded_collar",
            margin=0.0,
            name="base stays coaxial after quarter turn",
        )
    ctx.check(
        "continuous spin keeps bulb on the shared socket axis",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0] - turned_position[0]) < 1e-6
        and abs(rest_position[1] - turned_position[1]) < 1e-6,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
