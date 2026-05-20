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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _mesh_member(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    radial_segments: int = 18,
) -> MeshGeometry:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
    geom = CylinderGeometry(radius=radius, height=length, radial_segments=radial_segments)
    if length > 1e-9:
        ux = dx / length
        uy = dy / length
        uz = dz / length
        dot = max(-1.0, min(1.0, uz))
        if dot < 1.0 - 1e-9:
            if dot <= -1.0 + 1e-9:
                geom.rotate_x(math.pi)
            else:
                axis = (-uy, ux, 0.0)
                axis_len = math.sqrt((axis[0] * axis[0]) + (axis[1] * axis[1]) + (axis[2] * axis[2]))
                if axis_len > 1e-9:
                    geom.rotate(
                        (axis[0] / axis_len, axis[1] / axis_len, axis[2] / axis_len),
                        math.acos(dot),
                    )
    geom.translate((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)
    return geom


def _helical_thread(
    *,
    mean_radius: float,
    z_start: float,
    height: float,
    turns: float,
    thread_radius: float,
    start_angle: float = 0.0,
    samples_per_turn: int = 36,
) -> MeshGeometry:
    point_count = max(24, int(math.ceil(turns * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for index in range(point_count + 1):
        t = index / point_count
        angle = start_angle + (math.tau * turns * t)
        z = z_start + (height * t)
        points.append((mean_radius * math.cos(angle), mean_radius * math.sin(angle), z))
    return wire_from_points(
        points,
        radius=thread_radius,
        radial_segments=14,
        closed_path=False,
        cap_ends=True,
        corner_mode="miter",
    )


def _build_bulb_glass_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.016, 0.060),
            (0.026, 0.082),
            (0.041, 0.113),
            (0.050, 0.142),
            (0.046, 0.172),
            (0.033, 0.197),
            (0.018, 0.214),
            (0.005, 0.222),
        ],
        [
            (0.013, 0.063),
            (0.022, 0.085),
            (0.037, 0.114),
            (0.046, 0.142),
            (0.042, 0.170),
            (0.029, 0.194),
            (0.015, 0.209),
            (0.002, 0.216),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_screw_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0188, 0.004),
            (0.0198, 0.016),
            (0.0192, 0.040),
            (0.0182, 0.054),
        ],
        [
            (0.0168, 0.006),
            (0.0174, 0.018),
            (0.0170, 0.040),
            (0.0164, 0.052),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def _build_socket_collar_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0315, 0.056),
            (0.0330, 0.062),
            (0.0330, 0.108),
            (0.0310, 0.116),
        ],
        [
            (0.0245, 0.068),
            (0.0245, 0.109),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def _build_ceramic_liner_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0240, 0.068),
            (0.0240, 0.106),
        ],
        [
            (0.0222, 0.070),
            (0.0222, 0.104),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_lower_housing_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.038, 0.000),
            (0.042, 0.014),
            (0.041, 0.042),
            (0.036, 0.070),
        ],
        [
            (0.027, 0.004),
            (0.029, 0.016),
            (0.028, 0.042),
            (0.026, 0.068),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def _build_reinforcement_band_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0395, 0.096),
            (0.0405, 0.101),
            (0.0405, 0.111),
            (0.0395, 0.116),
        ],
        [
            (0.0310, 0.096),
            (0.0320, 0.101),
            (0.0320, 0.111),
            (0.0310, 0.116),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )


def _build_insulator_ring_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    z0: float,
    z1: float,
    segments: int = 56,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, z0),
            (outer_radius, z1),
        ],
        [
            (inner_radius, z0),
            (inner_radius, z1),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _build_guard_mesh() -> MeshGeometry:
    base_radius = 0.066
    mid_radius = 0.066
    crown_radius = 0.028
    rod_radius = 0.0036
    ring_radius = 0.0040

    geometries = [
        TorusGeometry(radius=base_radius, tube=ring_radius, radial_segments=18, tubular_segments=44).translate(
            0.0, 0.0, 0.022
        ),
        TorusGeometry(radius=mid_radius, tube=ring_radius, radial_segments=18, tubular_segments=44).translate(
            0.0, 0.0, 0.130
        ),
        TorusGeometry(radius=crown_radius, tube=ring_radius, radial_segments=18, tubular_segments=36).translate(
            0.0, 0.0, 0.186
        ),
    ]

    for index in range(6):
        angle = index * math.tau / 6.0
        x = base_radius * math.cos(angle)
        y = base_radius * math.sin(angle)
        geometries.append(
            _mesh_member((x, y, 0.022), (x, y, 0.130), radius=rod_radius, radial_segments=16)
        )

    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        geometries.append(
            _mesh_member(
                (mid_radius * math.cos(angle), mid_radius * math.sin(angle), 0.130),
                (crown_radius * math.cos(angle), crown_radius * math.sin(angle), 0.186),
                radius=rod_radius,
                radial_segments=16,
            )
        )

    foot_radius = 0.054
    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        foot_x = foot_radius * math.cos(angle)
        foot_y = foot_radius * math.sin(angle)
        ring_x = base_radius * math.cos(angle)
        ring_y = base_radius * math.sin(angle)
        geometries.append(_mesh_member((foot_x, foot_y, 0.000), (foot_x, foot_y, 0.016), radius=0.0042))
        geometries.append(_mesh_member((foot_x, foot_y, 0.016), (ring_x, ring_y, 0.022), radius=0.0036))

    return _merge_meshes(*geometries)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_socket_bulb")

    housing_steel = model.material("housing_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    galvanized = model.material("galvanized", rgba=(0.63, 0.66, 0.69, 1.0))
    ceramic = model.material("ceramic", rgba=(0.84, 0.82, 0.76, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.74, 0.15, 0.10, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.61, 0.31, 1.0))
    bulb_metal = model.material("bulb_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    glass = model.material("glass", rgba=(0.92, 0.94, 0.96, 0.34))
    stem_metal = model.material("stem_metal", rgba=(0.74, 0.76, 0.79, 1.0))

    fixture = model.part("fixture_body")
    fixture.visual(
        Box((0.160, 0.010, 0.220)),
        origin=Origin(xyz=(0.000, -0.076, 0.110)),
        material=housing_steel,
        name="backplate",
    )
    fixture.visual(
        Box((0.016, 0.078, 0.056)),
        origin=Origin(xyz=(-0.025, -0.037, 0.046)),
        material=housing_steel,
        name="left_support_cheek",
    )
    fixture.visual(
        Box((0.016, 0.078, 0.056)),
        origin=Origin(xyz=(0.025, -0.037, 0.046)),
        material=housing_steel,
        name="right_support_cheek",
    )
    fixture.visual(
        mesh_from_geometry(_build_lower_housing_shell_mesh(), "lower_housing_shell"),
        material=housing_steel,
        name="lower_housing",
    )
    fixture.visual(
        mesh_from_geometry(_build_reinforcement_band_mesh(), "reinforcement_band_shell"),
        material=galvanized,
        name="reinforcement_band",
    )
    fixture.visual(
        mesh_from_geometry(_build_socket_collar_mesh(), "socket_collar_shell"),
        material=galvanized,
        name="socket_collar",
    )
    fixture.visual(
        mesh_from_geometry(
            _helical_thread(
                mean_radius=0.0226,
                z_start=0.069,
                height=0.040,
                turns=3.4,
                thread_radius=0.0016,
                start_angle=0.35,
            ),
            "socket_internal_thread",
        ),
        material=galvanized,
        name="socket_thread",
    )
    fixture.visual(
        mesh_from_geometry(_build_ceramic_liner_mesh(), "socket_ceramic_liner"),
        material=ceramic,
        name="ceramic_liner",
    )
    fixture.visual(
        Cylinder(radius=0.0065, length=0.0040),
        origin=Origin(xyz=(0.000, 0.000, 0.0580)),
        material=brass,
        name="socket_contact",
    )
    fixture.visual(
        mesh_from_geometry(
            _build_insulator_ring_mesh(
                inner_radius=0.0054,
                outer_radius=0.0120,
                z0=0.0580,
                z1=0.0710,
            ),
            "socket_contact_insulator",
        ),
        origin=Origin(),
        material=ceramic,
        name="contact_insulator",
    )
    fixture.visual(
        Cylinder(radius=0.0224, length=0.0020),
        origin=Origin(xyz=(0.000, 0.000, 0.0710)),
        material=ceramic,
        name="contact_floor",
    )
    fixture.visual(
        Box((0.008, 0.074, 0.090)),
        origin=Origin(xyz=(-0.038, -0.021, 0.061), rpy=(0.78, 0.0, 0.0)),
        material=housing_steel,
        name="left_gusset",
    )
    fixture.visual(
        Box((0.008, 0.074, 0.090)),
        origin=Origin(xyz=(0.038, -0.021, 0.061), rpy=(-0.78, 0.0, 0.0)),
        material=housing_steel,
        name="right_gusset",
    )
    fixture.visual(
        Box((0.012, 0.028, 0.032)),
        origin=Origin(xyz=(-0.045, 0.000, 0.112)),
        material=lockout_red,
        name="left_lockout_tab",
    )
    fixture.visual(
        Box((0.012, 0.028, 0.032)),
        origin=Origin(xyz=(0.045, 0.000, 0.112)),
        material=lockout_red,
        name="right_lockout_tab",
    )
    for bolt_index, (x_pos, z_pos) in enumerate(
        (
            (-0.055, 0.038),
            (0.055, 0.038),
            (-0.055, 0.182),
            (0.055, 0.182),
        )
    ):
        fixture.visual(
            Cylinder(radius=0.0070, length=0.0120),
            origin=Origin(xyz=(x_pos, -0.076, z_pos), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"mount_bolt_{bolt_index}",
        )
    for arm_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        fixture.visual(
            Box((0.024, 0.008, 0.008)),
            origin=Origin(
                xyz=(0.0435 * math.cos(angle), 0.0435 * math.sin(angle), 0.1120),
                rpy=(0.0, 0.0, angle),
            ),
            material=galvanized,
            name=f"guard_arm_{arm_index}",
        )
    for pad_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        pad_radius = 0.054
        fixture.visual(
            Cylinder(radius=0.0060, length=0.0080),
            origin=Origin(
                xyz=(pad_radius * math.cos(angle), pad_radius * math.sin(angle), 0.1120),
            ),
            material=galvanized,
            name=f"guard_pad_{pad_index}",
        )
    fixture.inertial = Inertial.from_geometry(
        Box((0.180, 0.120, 0.230)),
        mass=2.8,
        origin=Origin(xyz=(0.000, -0.020, 0.105)),
    )

    bulb = model.part("bulb")
    bulb.visual(
        mesh_from_geometry(_build_bulb_glass_mesh(), "rough_service_bulb_glass"),
        material=glass,
        name="bulb_glass",
    )
    bulb.visual(
        Cylinder(radius=0.0155, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, 0.068)),
        material=bulb_metal,
        name="neck_collar",
    )
    bulb.visual(
        Cylinder(radius=0.0245, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.058)),
        material=bulb_metal,
        name="shoulder_flange",
    )
    bulb.visual(
        mesh_from_geometry(_build_screw_shell_mesh(), "bulb_screw_shell"),
        material=bulb_metal,
        name="screw_shell",
    )
    bulb.visual(
        mesh_from_geometry(
            _helical_thread(
                mean_radius=0.0192,
                z_start=0.010,
                height=0.040,
                turns=3.4,
                thread_radius=0.00145,
                start_angle=0.0,
            ),
            "bulb_external_thread",
        ),
        material=bulb_metal,
        name="screw_thread",
    )
    bulb.visual(
        Cylinder(radius=0.0168, length=0.0080),
        origin=Origin(xyz=(0.000, 0.000, 0.0140)),
        material=ceramic,
        name="center_insulator",
    )
    bulb.visual(
        Cylinder(radius=0.0042, length=0.0100),
        origin=Origin(xyz=(0.000, 0.000, 0.0030)),
        material=brass,
        name="contact_button",
    )
    bulb.visual(
        Cylinder(radius=0.0024, length=0.0560),
        origin=Origin(xyz=(0.000, 0.000, 0.0360)),
        material=stem_metal,
        name="center_lead",
    )
    bulb.visual(
        Cylinder(radius=0.0028, length=0.095),
        origin=Origin(xyz=(0.000, 0.000, 0.116)),
        material=stem_metal,
        name="support_stem",
    )
    bulb.visual(
        Box((0.020, 0.010, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.147)),
        material=stem_metal,
        name="emitter_bridge",
    )
    bulb.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 0.225)),
        mass=0.42,
        origin=Origin(xyz=(0.000, 0.000, 0.115)),
    )

    guard = model.part("guard_cage")
    guard.visual(
        mesh_from_geometry(_build_guard_mesh(), "industrial_guard_cage"),
        material=galvanized,
        name="guard_frame",
    )
    guard.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.190),
        mass=0.85,
        origin=Origin(xyz=(0.000, 0.000, 0.095)),
    )

    model.articulation(
        "fixture_to_bulb",
        ArticulationType.REVOLUTE,
        parent=fixture,
        child=bulb,
        origin=Origin(xyz=(0.000, 0.000, 0.0620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
            lower=0.0,
            upper=4.0 * math.pi,
        ),
    )
    model.articulation(
        "fixture_to_guard",
        ArticulationType.FIXED,
        parent=fixture,
        child=guard,
        origin=Origin(xyz=(0.000, 0.000, 0.1160)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixture = object_model.get_part("fixture_body")
    bulb = object_model.get_part("bulb")
    guard = object_model.get_part("guard_cage")
    bulb_spin = object_model.get_articulation("fixture_to_bulb")

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

    limits = bulb_spin.motion_limits
    ctx.check(
        "bulb_spin_axis_is_vertical",
        tuple(round(value, 6) for value in bulb_spin.axis) == (0.0, 0.0, 1.0),
        f"expected screw axis (0,0,1), got {bulb_spin.axis}",
    )
    ctx.check(
        "bulb_spin_limits_cover_threaded_travel",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= (4.0 * math.pi) - 1e-6,
        f"unexpected bulb spin limits: {limits}",
    )

    with ctx.pose({bulb_spin: 0.0}):
        ctx.expect_origin_distance(
            bulb,
            fixture,
            axes="xy",
            max_dist=0.0005,
            name="bulb_and_socket_are_coaxial",
        )
        ctx.expect_contact(
            bulb,
            fixture,
            elem_a="contact_button",
            elem_b="socket_contact",
            name="bulb_button_seats_on_socket_contact",
        )
        ctx.expect_gap(
            bulb,
            fixture,
            axis="z",
            positive_elem="shoulder_flange",
            negative_elem="socket_collar",
            max_gap=0.0015,
            max_penetration=1e-6,
            name="bulb_shoulder_stops_on_socket_collar",
        )
        ctx.expect_overlap(
            bulb,
            fixture,
            axes="xy",
            elem_a="screw_shell",
            elem_b="socket_collar",
            min_overlap=0.035,
            name="screw_shell_stays_nested_in_socket_collar",
        )
        ctx.expect_contact(
            guard,
            fixture,
            elem_a="guard_frame",
            elem_b="guard_pad_0",
            name="guard_mounts_on_fixture_pads",
        )
        ctx.expect_within(
            bulb,
            guard,
            axes="xy",
            inner_elem="bulb_glass",
            outer_elem="guard_frame",
            margin=0.0,
            name="guard_encloses_bulb_footprint",
        )

    with ctx.pose({bulb_spin: 2.0 * math.pi}):
        ctx.expect_origin_distance(
            bulb,
            fixture,
            axes="xy",
            max_dist=0.0005,
            name="bulb_remains_coaxial_when_rotated",
        )
        ctx.expect_contact(
            bulb,
            fixture,
            elem_a="contact_button",
            elem_b="socket_contact",
            name="bulb_contact_persists_when_rotated",
        )
        ctx.expect_overlap(
            bulb,
            fixture,
            axes="xy",
            elem_a="screw_shell",
            elem_b="socket_collar",
            min_overlap=0.035,
            name="threaded_engagement_stays_centered_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
