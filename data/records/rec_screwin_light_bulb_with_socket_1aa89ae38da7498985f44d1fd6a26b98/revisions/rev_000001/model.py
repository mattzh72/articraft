from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _threaded_outer_profile(
    z_min: float,
    z_max: float,
    *,
    root_radius: float,
    crest_radius: float,
    pitch: float,
) -> list[tuple[float, float]]:
    """Lathed V-thread-like ridges for the bulb's metal screw shell."""
    pts: list[tuple[float, float]] = [(root_radius, z_min)]
    z = z_min
    while z + pitch < z_max:
        pts.extend(
            [
                (root_radius, z + 0.18 * pitch),
                (crest_radius, z + 0.42 * pitch),
                (crest_radius, z + 0.60 * pitch),
                (root_radius, z + 0.84 * pitch),
            ]
        )
        z += pitch
    pts.append((root_radius, z_max))
    return pts


def _threaded_inner_profile(
    z_min: float,
    z_max: float,
    *,
    bore_radius: float,
    ridge_radius: float,
    pitch: float,
) -> list[tuple[float, float]]:
    """Lathed inward ridges showing the socket's mating threaded collar."""
    pts: list[tuple[float, float]] = [(bore_radius, z_min)]
    z = z_min
    while z + pitch < z_max:
        pts.extend(
            [
                (bore_radius, z + 0.16 * pitch),
                (ridge_radius, z + 0.40 * pitch),
                (ridge_radius, z + 0.58 * pitch),
                (bore_radius, z + 0.82 * pitch),
            ]
        )
        z += pitch
    pts.append((bore_radius, z_max))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_bulb_socket")

    glass = model.material("warm_clear_glass", rgba=(0.95, 0.90, 0.68, 0.38))
    glow = model.material("warm_filament", rgba=(1.0, 0.63, 0.16, 1.0))
    shell_metal = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    socket_metal = model.material("nickel_collar", rgba=(0.66, 0.68, 0.70, 1.0))
    ceramic = model.material("white_ceramic", rgba=(0.88, 0.84, 0.76, 1.0))
    insulator = model.material("black_insulator", rgba=(0.025, 0.022, 0.020, 1.0))

    socket = model.part("socket")
    socket_body = LatheGeometry.from_shell_profiles(
        [
            (0.032, -0.064),
            (0.038, -0.056),
            (0.040, -0.022),
            (0.038, 0.006),
            (0.033, 0.016),
        ],
        [
            (0.000, -0.056),
            (0.012, -0.052),
            (0.024, -0.046),
            (0.026, 0.006),
            (0.026, 0.016),
        ],
        segments=72,
        start_cap="round",
        end_cap="round",
    )
    socket.visual(
        _mesh(socket_body, "socket_body"),
        material=ceramic,
        name="socket_body",
    )

    socket_collar_outer = [
        (0.020, -0.050),
        (0.020, -0.004),
        (0.026, 0.002),
        (0.026, 0.010),
        (0.021, 0.014),
    ]
    socket_collar_inner = _threaded_inner_profile(
        -0.049,
        0.013,
        bore_radius=0.0157,
        ridge_radius=0.0146,
        pitch=0.0072,
    )
    socket.visual(
        _mesh(
            LatheGeometry.from_shell_profiles(
                socket_collar_outer,
                socket_collar_inner,
                segments=80,
                start_cap="flat",
                end_cap="flat",
            ),
            "threaded_collar",
        ),
        material=socket_metal,
        name="threaded_collar",
    )
    socket.visual(
        Cylinder(radius=0.0075, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=socket_metal,
        name="center_contact",
    )

    bulb = model.part("bulb")
    bulb.visual(
        _mesh(
            LatheGeometry.from_shell_profiles(
                [
                    (0.0105, 0.006),
                    (0.0155, 0.016),
                    (0.0240, 0.034),
                    (0.0290, 0.060),
                    (0.0260, 0.087),
                    (0.0140, 0.108),
                    (0.0018, 0.119),
                ],
                [
                    (0.0089, 0.0075),
                    (0.0137, 0.018),
                    (0.0222, 0.036),
                    (0.0270, 0.060),
                    (0.0240, 0.085),
                    (0.0120, 0.105),
                    (0.0010, 0.116),
                ],
                segments=80,
                start_cap="round",
                end_cap="round",
            ),
            "glass_envelope",
        ),
        material=glass,
        name="glass_envelope",
    )
    bulb.visual(
        _mesh(
            LatheGeometry.from_shell_profiles(
                _threaded_outer_profile(
                    -0.048,
                    0.000,
                    root_radius=0.0114,
                    crest_radius=0.0132,
                    pitch=0.0072,
                ),
                [
                    (0.0093, -0.047),
                    (0.0095, -0.010),
                    (0.0102, 0.000),
                ],
                segments=80,
                start_cap="flat",
                end_cap="flat",
            ),
            "screw_shell",
        ),
        material=shell_metal,
        name="screw_shell",
    )
    bulb.visual(
        _mesh(
            LatheGeometry(
                [
                    (0.000, -0.054),
                    (0.004, -0.054),
                    (0.007, -0.051),
                    (0.008, -0.048),
                    (0.011, -0.047),
                    (0.000, -0.047),
                ],
                segments=56,
            ),
            "contact_tip",
        ),
        material=shell_metal,
        name="contact_tip",
    )
    bulb.visual(
        Cylinder(radius=0.0128, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=insulator,
        name="neck_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.002, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=glass,
        name="glass_stem",
    )
    bulb.visual(
        Cylinder(radius=0.00075, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glow,
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
        "bulb joint is continuous about socket axis",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="screw_shell",
        outer_elem="threaded_collar",
        margin=0.0,
        name="screw shell centered inside collar",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="screw_shell",
        elem_b="threaded_collar",
        min_overlap=0.040,
        name="threaded base is seated deeply in socket",
    )

    rest_pos = ctx.part_world_position(bulb)
    with ctx.pose({spin: math.pi * 1.5}):
        turned_pos = ctx.part_world_position(bulb)
        ctx.expect_within(
            bulb,
            socket,
            axes="xy",
            inner_elem="screw_shell",
            outer_elem="threaded_collar",
            margin=0.0,
            name="rotated screw shell remains coaxial",
        )
    ctx.check(
        "continuous rotation keeps bulb on shared axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
