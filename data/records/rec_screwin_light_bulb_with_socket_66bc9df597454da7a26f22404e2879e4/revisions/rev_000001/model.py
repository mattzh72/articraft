from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _helical_thread_mesh(
    *,
    radius: float,
    z_min: float,
    z_max: float,
    turns: float,
    height: float,
    width: float,
    samples: int = 144,
    internal: bool = False,
) -> MeshGeometry:
    """Low-relief helical thread band attached to a cylinder wall.

    The ridge is intentionally shallow so the rotating bulb keeps a clean,
    continuous silhouette instead of many separate ring breaks.
    """

    geom = MeshGeometry()
    rail_count = 4

    for i in range(samples + 1):
        t = i / samples
        theta = 2.0 * math.pi * turns * t
        z = z_min + (z_max - z_min) * t
        c = math.cos(theta)
        s = math.sin(theta)

        if internal:
            cross_section = (
                (radius + 0.00025, -width),
                (radius - height, -0.55 * width),
                (radius - height, 0.55 * width),
                (radius + 0.00025, width),
            )
        else:
            cross_section = (
                (radius - 0.00025, -width),
                (radius + height, -0.55 * width),
                (radius + height, 0.55 * width),
                (radius - 0.00025, width),
            )

        for r, dz in cross_section:
            geom.add_vertex(r * c, r * s, z + dz)

    for i in range(samples):
        row = i * rail_count
        nxt = (i + 1) * rail_count
        for j in range(rail_count):
            a = row + j
            b = row + ((j + 1) % rail_count)
            c = nxt + ((j + 1) % rail_count)
            d = nxt + j
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)

    # Cap the two ends of the helical band.
    geom.add_face(0, 1, 2)
    geom.add_face(0, 2, 3)
    end = samples * rail_count
    geom.add_face(end, end + 2, end + 1)
    geom.add_face(end, end + 3, end + 2)

    return geom


def _bulb_base_mesh() -> MeshGeometry:
    base = LatheGeometry(
        [
            (0.000, -0.034),
            (0.006, -0.034),
            (0.007, -0.032),
            (0.011, -0.031),
            (0.013, -0.028),
            (0.013, 0.020),
            (0.011, 0.026),
            (0.000, 0.026),
        ],
        segments=96,
        closed=True,
    )
    thread = _helical_thread_mesh(
        radius=0.013,
        z_min=-0.026,
        z_max=0.018,
        turns=5.0,
        height=0.0018,
        width=0.0016,
        samples=180,
    )
    return base.merge(thread)


def _socket_collar_mesh() -> MeshGeometry:
    collar = LatheGeometry(
        [
            (0.0142, 0.020),
            (0.0265, 0.020),
            (0.0265, 0.069),
            (0.0142, 0.069),
        ],
        segments=96,
        closed=True,
    )
    thread = _helical_thread_mesh(
        radius=0.0142,
        z_min=0.023,
        z_max=0.064,
        turns=4.7,
        height=0.0012,
        width=0.0014,
        samples=170,
        internal=True,
    )
    return collar.merge(thread)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_bulb_socket")

    ceramic = model.material("warm_ceramic", rgba=(0.86, 0.82, 0.70, 1.0))
    brushed_metal = model.material("brushed_nickel", rgba=(0.72, 0.70, 0.65, 1.0))
    brass = model.material("aged_brass", rgba=(0.86, 0.62, 0.24, 1.0))
    glass = model.material("warm_clear_glass", rgba=(0.90, 0.97, 1.00, 0.34))
    tungsten = model.material("warm_filament", rgba=(1.00, 0.62, 0.22, 1.0))

    socket = model.part("socket")
    socket_body = LatheGeometry.from_shell_profiles(
        [
            (0.026, 0.000),
            (0.034, 0.010),
            (0.036, 0.058),
            (0.034, 0.074),
        ],
        [
            (0.004, 0.014),
            (0.018, 0.018),
            (0.026, 0.066),
            (0.026, 0.074),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    socket.visual(
        mesh_from_geometry(socket_body, "socket_body"),
        material=ceramic,
        name="body",
    )
    socket.visual(
        mesh_from_geometry(_socket_collar_mesh(), "socket_threaded_collar"),
        material=brass,
        name="threaded_collar",
    )
    bulb = model.part("bulb")
    glass_envelope = LatheGeometry.from_shell_profiles(
        [
            (0.011, 0.025),
            (0.014, 0.034),
            (0.023, 0.051),
            (0.030, 0.073),
            (0.030, 0.091),
            (0.022, 0.113),
            (0.007, 0.128),
            (0.001, 0.131),
        ],
        [
            (0.0085, 0.027),
            (0.012, 0.036),
            (0.020, 0.052),
            (0.0275, 0.073),
            (0.0275, 0.089),
            (0.020, 0.109),
            (0.0045, 0.124),
            (0.001, 0.127),
        ],
        segments=112,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    bulb.visual(
        mesh_from_geometry(glass_envelope, "bulb_envelope"),
        material=glass,
        name="envelope",
    )
    bulb.visual(
        mesh_from_geometry(_bulb_base_mesh(), "bulb_threaded_base"),
        material=brushed_metal,
        name="threaded_base",
    )
    filament = tube_from_spline_points(
        [
            (-0.0045, 0.0, 0.023),
            (-0.0045, 0.0, 0.054),
            (-0.0030, 0.0, 0.060),
            (-0.0013, 0.0, 0.058),
            (0.0000, 0.0, 0.061),
            (0.0013, 0.0, 0.058),
            (0.0030, 0.0, 0.060),
            (0.0045, 0.0, 0.054),
            (0.0045, 0.0, 0.023),
        ],
        radius=0.00045,
        samples_per_segment=8,
        radial_segments=10,
        cap_ends=True,
    )
    bulb.visual(
        mesh_from_geometry(filament, "filament_support"),
        material=tungsten,
        name="filament",
    )

    model.articulation(
        "socket_to_bulb",
        ArticulationType.CONTINUOUS,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    spin = object_model.get_articulation("socket_to_bulb")

    ctx.allow_overlap(
        socket,
        bulb,
        elem_a="threaded_collar",
        elem_b="threaded_base",
        reason=(
            "The screw shell and mating collar use shallow visual thread ridges; "
            "their tiny local interference represents the seated threaded engagement."
        ),
    )

    ctx.check(
        "bulb uses continuous screw-axis rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="threaded_base",
        outer_elem="threaded_collar",
        margin=0.002,
        name="threaded base is centered inside collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="threaded_base",
        elem_b="threaded_collar",
        min_overlap=0.035,
        name="threaded base remains seated in collar",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="envelope",
        negative_elem="body",
        min_gap=0.010,
        name="glass envelope stands above socket lip",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: 1.7}):
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="threaded_base",
            outer_elem="threaded_collar",
            margin=0.002,
            name="rotated threaded base stays on socket axis",
        )
        ctx.expect_overlap(
            bulb,
            socket,
            axes="z",
            elem_a="threaded_base",
            elem_b="threaded_collar",
            min_overlap=0.035,
            name="rotated bulb remains seated",
        )
        rotated_pos = ctx.part_world_position(bulb)

    ctx.check(
        "continuous spin keeps bulb origin fixed",
        rest_pos is not None
        and rotated_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, rotated_pos)),
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
