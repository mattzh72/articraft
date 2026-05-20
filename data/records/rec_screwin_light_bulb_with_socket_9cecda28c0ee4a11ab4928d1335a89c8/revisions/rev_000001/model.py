from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _profile_at_z(
    width: float,
    height: float,
    radius: float,
    z: float,
    *,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)]


def _helix_points(
    *,
    radius: float,
    z0: float,
    z1: float,
    turns: float,
    phase: float = 0.0,
    samples: int = 56,
) -> list[tuple[float, float, float]]:
    points = []
    for i in range(samples + 1):
        t = i / samples
        angle = phase + (turns * math.tau * t)
        z = z0 + ((z1 - z0) * t)
        points.append((radius * math.cos(angle), radius * math.sin(angle), z))
    return points


def _thread_mesh(
    *,
    radius: float,
    tube_radius: float,
    z0: float,
    z1: float,
    turns: float,
    phase: float,
    radial_segments: int = 14,
) -> MeshGeometry:
    return tube_from_spline_points(
        _helix_points(radius=radius, z0=z0, z1=z1, turns=turns, phase=phase),
        radius=tube_radius,
        samples_per_segment=2,
        radial_segments=radial_segments,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_screw_bulb_socket")

    aged_brass = model.material("aged_brass", rgba=(0.70, 0.59, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.53, 0.56, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    ceramic = model.material("ceramic", rgba=(0.86, 0.84, 0.78, 1.0))
    bakelite = model.material("bakelite", rgba=(0.16, 0.12, 0.09, 1.0))
    amber_glass = model.material("amber_glass", rgba=(0.88, 0.76, 0.42, 0.48))
    gasket = model.material("gasket", rgba=(0.10, 0.10, 0.10, 1.0))

    socket = model.part("socket")
    socket.visual(
        Box((0.14, 0.09, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="adapter_plate",
    )
    socket.visual(
        _save_mesh(
            "socket_adapter_shroud",
            section_loft(
                [
                    _profile_at_z(0.092, 0.060, 0.010, 0.012),
                    _profile_at_z(0.074, 0.054, 0.012, 0.024),
                    _profile_at_z(0.060, 0.050, 0.014, 0.038),
                ]
            ),
        ),
        material=dark_steel,
        name="adapter_shroud",
    )
    socket.visual(
        _save_mesh(
            "socket_housing_drum",
            LatheGeometry.from_shell_profiles(
                [
                    (0.0290, -0.023),
                    (0.0310, -0.020),
                    (0.0310, 0.020),
                    (0.0290, 0.023),
                ],
                [
                    (0.0178, -0.021),
                    (0.0178, 0.021),
                ],
                segments=52,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=bakelite,
        name="housing_drum",
    )
    socket.visual(
        _save_mesh(
            "socket_adapter_band",
            LatheGeometry.from_shell_profiles(
                [
                    (0.0340, -0.004),
                    (0.0360, -0.002),
                    (0.0360, 0.002),
                    (0.0340, 0.004),
                ],
                [
                    (0.0250, -0.004),
                    (0.0250, 0.004),
                ],
                segments=48,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=dark_steel,
        name="adapter_band",
    )
    socket.visual(
        _save_mesh(
            "socket_collar_reinforcement",
            LatheGeometry.from_shell_profiles(
                [
                    (0.0245, -0.007),
                    (0.0260, -0.004),
                    (0.0260, 0.004),
                    (0.0245, 0.007),
                ],
                [
                    (0.0182, -0.007),
                    (0.0182, 0.007),
                ],
                segments=48,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_steel,
        name="collar_reinforcement",
    )
    socket.visual(
        _save_mesh(
            "socket_collar_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.0205, -0.018),
                    (0.0220, -0.012),
                    (0.0220, 0.018),
                    (0.0205, 0.021),
                ],
                [
                    (0.0170, -0.016),
                    (0.0170, 0.019),
                ],
                segments=52,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=aged_brass,
        name="socket_collar",
    )
    socket.visual(
        _save_mesh(
            "socket_female_thread",
            _thread_mesh(
                radius=0.0163,
                tube_radius=0.0007,
                z0=0.078,
                z1=0.096,
                turns=4.9,
                phase=0.45,
            ),
        ),
        material=aged_brass,
        name="female_thread",
    )
    socket.visual(
        Cylinder(radius=0.0145, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=ceramic,
        name="ceramic_seat",
    )
    socket.visual(
        Cylinder(radius=0.0038, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=aged_brass,
        name="contact_button",
    )
    socket.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=gasket,
        name="seat_gasket",
    )
    for index, (x_pos, y_pos, yaw) in enumerate(
        (
            (0.0162, 0.0, 0.0),
            (-0.0162, 0.0, 0.0),
            (0.0, 0.0162, math.pi / 2.0),
            (0.0, -0.0162, math.pi / 2.0),
        )
    ):
        socket.visual(
            Box((0.0042, 0.0040, 0.012)),
            origin=Origin(xyz=(x_pos, y_pos, 0.069), rpy=(0.0, 0.0, yaw)),
            material=ceramic,
            name=f"ceramic_web_{index}",
        )

    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        socket.visual(
            Box((0.010, 0.006, 0.028)),
            origin=Origin(
                xyz=(0.027 * math.cos(angle), 0.027 * math.sin(angle), 0.077),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"gusset_{index}",
        )

    for name, x_pos, yaw in (
        ("service_hatch_right", 0.0305, 0.0),
        ("service_hatch_left", -0.0305, math.pi),
    ):
        socket.visual(
            Box((0.007, 0.030, 0.024)),
            origin=Origin(xyz=(x_pos, 0.0, 0.056)),
            material=dark_steel,
            name=name,
        )
        socket.visual(
            Cylinder(radius=0.0032, length=0.030),
            origin=Origin(
                xyz=(x_pos * 0.92, 0.0, 0.069),
                rpy=(0.0, math.pi / 2.0, math.pi / 2.0),
            ),
            material=steel,
            name=f"{name}_hinge",
        )
        socket.visual(
            Cylinder(radius=0.0027, length=0.007),
            origin=Origin(xyz=(x_pos * 1.03, 0.009, 0.056), rpy=(0.0, math.pi / 2.0, yaw)),
            material=steel,
            name=f"{name}_bolt_upper",
        )
        socket.visual(
            Cylinder(radius=0.0027, length=0.007),
            origin=Origin(xyz=(x_pos * 1.03, -0.009, 0.056), rpy=(0.0, math.pi / 2.0, yaw)),
            material=steel,
            name=f"{name}_bolt_lower",
        )

    for name, x_pos, y_pos in (
        ("plate_bolt_fr", 0.049, 0.026),
        ("plate_bolt_rr", -0.049, 0.026),
        ("plate_bolt_fl", 0.049, -0.026),
        ("plate_bolt_rl", -0.049, -0.026),
    ):
        socket.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.012)),
            material=steel,
            name=name,
        )

    socket.inertial = Inertial.from_geometry(
        Box((0.14, 0.09, 0.11)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        Cylinder(radius=0.0034, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=aged_brass,
        name="contact_tip",
    )
    bulb.visual(
        Cylinder(radius=0.0134, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=aged_brass,
        name="screw_shell",
    )
    bulb.visual(
        _save_mesh(
            "bulb_male_thread",
            _thread_mesh(
                radius=0.0140,
                tube_radius=0.0007,
                z0=-0.019,
                z1=0.004,
                turns=5.7,
                phase=0.45,
            ),
        ),
        material=aged_brass,
        name="male_thread",
    )
    bulb.visual(
        Cylinder(radius=0.0148, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=ceramic,
        name="insulator_collar",
    )
    bulb.visual(
        Cylinder(radius=0.0158, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="retaining_band",
    )
    bulb.visual(
        _save_mesh(
            "bulb_glass_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.0145, 0.000),
                    (0.0180, 0.010),
                    (0.0290, 0.035),
                    (0.0350, 0.060),
                    (0.0310, 0.086),
                    (0.0130, 0.112),
                    (0.0000, 0.118),
                ],
                [
                    (0.0122, 0.002),
                    (0.0154, 0.011),
                    (0.0265, 0.036),
                    (0.0324, 0.060),
                    (0.0285, 0.084),
                    (0.0110, 0.109),
                    (0.0000, 0.115),
                ],
                segments=64,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=amber_glass,
        name="glass_shell",
    )
    bulb.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=gasket,
        name="glass_neck_gasket",
    )
    bulb.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.145),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    model.articulation(
        "bulb_rotation",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=bulb,
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=14.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    bulb = object_model.get_part("bulb")
    bulb_rotation = object_model.get_articulation("bulb_rotation")

    socket.get_visual("socket_collar")
    socket.get_visual("female_thread")
    socket.get_visual("service_hatch_right")
    socket.get_visual("service_hatch_left")
    bulb.get_visual("male_thread")
    bulb.get_visual("screw_shell")
    bulb.get_visual("glass_shell")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "bulb_rotation_axis_is_coaxial",
        tuple(round(v, 6) for v in bulb_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={bulb_rotation.axis}",
    )
    limits = bulb_rotation.motion_limits
    ctx.check(
        "bulb_rotation_has_realistic_turn_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and math.isclose(limits.lower, 0.0, abs_tol=1e-6)
        and limits.upper >= 10.0,
        details=f"limits={limits}",
    )
    ctx.expect_origin_distance(
        bulb,
        socket,
        axes="xy",
        max_dist=1e-6,
        name="bulb_and_socket_are_coaxial",
    )
    ctx.expect_within(
        bulb,
        socket,
        axes="xy",
        inner_elem="male_thread",
        outer_elem="socket_collar",
        margin=0.007,
        name="male_thread_stays_within_socket_collar_footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="xy",
        elem_a="male_thread",
        elem_b="socket_collar",
        min_overlap=0.026,
        name="threaded_regions_share_xy_footprint",
    )
    ctx.expect_overlap(
        bulb,
        socket,
        axes="z",
        elem_a="male_thread",
        elem_b="socket_collar",
        min_overlap=0.014,
        name="threaded_regions_share_axial_depth",
    )
    ctx.expect_contact(
        bulb,
        socket,
        elem_a="contact_tip",
        elem_b="contact_button",
        contact_tol=0.0005,
        name="bulb_tip_reaches_socket_contact",
    )
    ctx.expect_gap(
        bulb,
        socket,
        axis="z",
        positive_elem="glass_shell",
        negative_elem="socket_collar",
        min_gap=0.008,
        name="glass_envelope_clears_socket_collar",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
