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
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _add_strip(mesh: MeshGeometry, row_a: list[int], row_b: list[int]) -> None:
    count = len(row_a)
    if count != len(row_b) or count < 2:
        return
    collapsed_a = all(index == row_a[0] for index in row_a)
    collapsed_b = all(index == row_b[0] for index in row_b)
    for i in range(count - 1):
        a0 = row_a[0] if collapsed_a else row_a[i]
        a1 = row_a[0] if collapsed_a else row_a[i + 1]
        b0 = row_b[0] if collapsed_b else row_b[i]
        b1 = row_b[0] if collapsed_b else row_b[i + 1]
        if collapsed_a and not collapsed_b:
            if a0 != b0 and b0 != b1 and a0 != b1:
                mesh.add_face(a0, b0, b1)
        elif collapsed_b and not collapsed_a:
            if a0 != a1 and a1 != b0 and a0 != b0:
                mesh.add_face(a0, a1, b0)
        else:
            if a0 != a1 and a1 != b1 and a0 != b1:
                mesh.add_face(a0, a1, b1)
            if a0 != b1 and b1 != b0 and a0 != b0:
                mesh.add_face(a0, b1, b0)


def _partial_revolved_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    phi_start: float,
    phi_end: float,
    phi_segments: int,
    cap_sides: bool = True,
) -> MeshGeometry:
    mesh = MeshGeometry()
    phi_values = [
        phi_start + (phi_end - phi_start) * index / phi_segments
        for index in range(phi_segments + 1)
    ]

    def build_rows(profile: list[tuple[float, float]]) -> list[list[int]]:
        rows: list[list[int]] = []
        for radius, z_pos in profile:
            if abs(radius) < 1e-9:
                vertex = mesh.add_vertex(0.0, 0.0, z_pos)
                rows.append([vertex] * len(phi_values))
            else:
                row = [
                    mesh.add_vertex(radius * math.cos(phi), radius * math.sin(phi), z_pos)
                    for phi in phi_values
                ]
                rows.append(row)
        return rows

    outer_rows = build_rows(outer_profile)
    inner_rows = build_rows(inner_profile)

    for row_index in range(len(outer_rows) - 1):
        _add_strip(mesh, outer_rows[row_index], outer_rows[row_index + 1])
    for row_index in range(len(inner_rows) - 1):
        _add_strip(mesh, inner_rows[row_index + 1], inner_rows[row_index])

    _add_strip(mesh, outer_rows[0], inner_rows[0])
    _add_strip(mesh, outer_rows[-1], inner_rows[-1])

    if cap_sides:
        start_outer = [row[0] for row in outer_rows]
        start_inner = [row[0] for row in inner_rows]
        end_outer = [row[-1] for row in outer_rows]
        end_inner = [row[-1] for row in inner_rows]
        _add_strip(mesh, start_outer, start_inner)
        _add_strip(mesh, end_inner, end_outer)
    return mesh


def _arc_about_y(rho: float, y_pos: float, start: float, end: float, samples: int) -> list[tuple[float, float, float]]:
    return [
        (
            rho * math.cos(angle),
            y_pos,
            rho * math.sin(angle),
        )
        for angle in [start + (end - start) * index / (samples - 1) for index in range(samples)]
    ]


def _spherical_meridian(
    radius: float,
    phi: float,
    theta_start: float,
    theta_end: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(samples):
        theta = theta_start + (theta_end - theta_start) * index / (samples - 1)
        points.append(
            (
                radius * math.sin(theta) * math.cos(phi),
                radius * math.sin(theta) * math.sin(phi),
                radius * math.cos(theta),
            )
        )
    return points


def _profile_edge_path(
    profile: list[tuple[float, float]],
    phi: float,
    *,
    radial_offset: float = 0.0,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (
            (radius + radial_offset) * math.cos(phi),
            (radius + radial_offset) * math.sin(phi),
            z_pos + z_offset,
        )
        for radius, z_pos in profile
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotating_observatory_dome")

    shell_white = model.material("shell_white", rgba=(0.91, 0.93, 0.95, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.75, 0.77, 0.79, 1.0))
    roller_black = model.material("roller_black", rgba=(0.10, 0.10, 0.11, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.26, 0.28, 0.30, 1.0))

    slit_half_angle = 0.27
    dome_outer_radius = 1.42
    shell_thickness = 0.035
    dome_inner_radius = dome_outer_radius - shell_thickness

    shell_outer_profile = [
        (0.10, 1.38),
        (0.30, 1.35),
        (0.70, 1.22),
        (1.03, 0.95),
        (1.27, 0.58),
        (1.39, 0.20),
        (1.42, 0.0),
        (1.42, -0.07),
        (1.50, -0.07),
        (1.50, -0.115),
        (1.33, -0.115),
    ]
    shell_inner_profile = [
        (0.075, 1.345),
        (0.27, 1.318),
        (0.66, 1.19),
        (0.98, 0.92),
        (1.22, 0.57),
        (1.34, 0.20),
        (1.385, 0.0),
        (1.385, -0.03),
        (1.44, -0.03),
        (1.44, -0.07),
        (1.30, -0.07),
    ]

    shutter_outer_profile = [
        (0.055, 1.405),
        (0.29, 1.372),
        (0.68, 1.24),
        (1.01, 0.97),
        (1.25, 0.60),
        (1.38, 0.22),
        (1.445, -0.005),
    ]
    shutter_inner_profile = [
        (0.03, 1.377),
        (0.26, 1.345),
        (0.64, 1.21),
        (0.96, 0.94),
        (1.20, 0.59),
        (1.33, 0.22),
        (1.395, 0.01),
    ]

    base_ring = model.part("base_ring")
    base_ring.visual(
        _save_mesh(
            "base_curb_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (1.22, -0.30),
                    (1.56, -0.30),
                    (1.56, -0.14),
                    (1.48, -0.14),
                ],
                [
                    (1.28, -0.25),
                    (1.50, -0.25),
                    (1.50, -0.17),
                    (1.34, -0.17),
                ],
                segments=88,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=frame_gray,
        name="curb_shell",
    )
    base_ring.visual(
        _save_mesh(
            "base_track_band",
            LatheGeometry.from_shell_profiles(
                [
                    (1.36, -0.155),
                    (1.52, -0.155),
                    (1.52, -0.135),
                ],
                [
                    (1.39, -0.15),
                    (1.49, -0.15),
                    (1.49, -0.143),
                ],
                segments=88,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=rail_aluminum,
        name="track_band",
    )
    for index in range(6):
        angle = (math.tau * index / 6.0) + (math.tau / 12.0)
        c = math.cos(angle)
        s = math.sin(angle)
        base_ring.visual(
            Box((0.11, 0.14, 0.13)),
            origin=Origin(xyz=(1.49 * c, 1.49 * s, -0.205), rpy=(0.0, 0.0, angle)),
            material=frame_gray,
            name=f"roller_bracket_{index:02d}",
        )
        base_ring.visual(
            Box((0.09, 0.06, 0.05)),
            origin=Origin(xyz=(1.465 * c, 1.465 * s, -0.175), rpy=(0.0, 0.0, angle)),
            material=frame_gray,
            name=f"axle_block_{index:02d}",
        )
        base_ring.visual(
            Cylinder(radius=0.055, length=0.10),
            origin=Origin(
                xyz=(1.44 * c, 1.44 * s, -0.17),
                rpy=(0.0, math.pi / 2.0, angle + math.pi / 2.0),
            ),
            material=roller_black,
            name=f"support_wheel_{index:02d}",
        )
        base_ring.visual(
            Box((0.10, 0.14, 0.02)),
            origin=Origin(xyz=(1.60 * c, 1.60 * s, -0.20), rpy=(0.0, 0.0, angle)),
            material=frame_gray,
            name=f"anchor_pad_{index:02d}",
        )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=1.56, length=0.30),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        _save_mesh(
            "dome_shell_skin",
            _partial_revolved_shell(
                shell_outer_profile,
                shell_inner_profile,
                phi_start=slit_half_angle,
                phi_end=(math.tau - slit_half_angle),
                phi_segments=96,
                cap_sides=False,
            ),
        ),
        material=shell_white,
        name="shell_skin",
    )
    dome_shell.visual(
        _save_mesh(
            "rotation_flange_band",
            LatheGeometry.from_shell_profiles(
                [
                    (1.41, -0.115),
                    (1.50, -0.115),
                    (1.50, -0.075),
                    (1.42, -0.075),
                ],
                [
                    (1.43, -0.105),
                    (1.47, -0.105),
                    (1.47, -0.085),
                    (1.43, -0.085),
                ],
                segments=88,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=frame_gray,
        name="rotation_flange",
    )
    dome_shell.visual(
        Box((0.10, 0.74, 0.08)),
        origin=Origin(xyz=(1.33, 0.0, -0.075)),
        material=frame_gray,
        name="slit_sill",
    )
    slit_rib_profile = shell_outer_profile[:7]
    guide_rail_profile = shutter_outer_profile[:6]
    shell_edge_left_path = _profile_edge_path(slit_rib_profile, slit_half_angle, radial_offset=0.0)
    shell_edge_right_path = _profile_edge_path(slit_rib_profile, -slit_half_angle, radial_offset=0.0)
    track_phi = 0.46
    full_left_track = _profile_edge_path(guide_rail_profile, track_phi, radial_offset=0.085)
    full_right_track = _profile_edge_path(guide_rail_profile, -track_phi, radial_offset=0.085)
    rail_radius = 0.018
    track_segments = {
        "high": (0, 1, 2),
        "mid": (2, 3, 4),
        "low": (4, 5),
    }
    for side_name, full_track in (
        ("left", full_left_track),
        ("right", full_right_track),
    ):
        for wheel_name, sample_ids in track_segments.items():
            segment_points = [full_track[index] for index in sample_ids]
            rail_visual_name = (
                f"guide_rail_{side_name}"
                if wheel_name == "mid"
                else f"guide_rail_{side_name}_{wheel_name}"
            )
            dome_shell.visual(
                _save_mesh(
                    rail_visual_name,
                    tube_from_spline_points(
                        segment_points,
                        radius=rail_radius,
                        samples_per_segment=6,
                        radial_segments=16,
                        cap_ends=True,
                    ),
                ),
                material=rail_aluminum,
                name=rail_visual_name,
            )
    for side_name, shell_path, full_track in (
        ("left", shell_edge_left_path, full_left_track),
        ("right", shell_edge_right_path, full_right_track),
    ):
        for anchor_name, index in (("upper", 2), ("lower", 4)):
            dome_shell.visual(
                _save_mesh(
                    f"rail_post_{side_name}_{anchor_name}",
                    tube_from_spline_points(
                        [shell_path[index], full_track[index]],
                        radius=0.008,
                        samples_per_segment=2,
                        radial_segments=12,
                        cap_ends=True,
                    ),
                ),
                material=frame_gray,
                name=f"rail_post_{side_name}_{anchor_name}",
            )
    dome_shell.inertial = Inertial.from_geometry(
        Box((3.10, 3.10, 1.60)),
        mass=160.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    shutter = model.part("shutter")
    shutter.visual(
        _save_mesh(
            "shutter_panel_skin",
            _partial_revolved_shell(
                shutter_outer_profile,
                shutter_inner_profile,
                phi_start=-0.245,
                phi_end=0.245,
                phi_segments=40,
                cap_sides=False,
            ),
        ),
        material=shell_white,
        name="panel_skin",
    )
    runner_phi = 0.245
    runner_profile = shutter_outer_profile[:6]
    panel_edge_left_path = _profile_edge_path(runner_profile, runner_phi, radial_offset=0.0)
    panel_edge_right_path = _profile_edge_path(runner_profile, -runner_phi, radial_offset=0.0)
    wheel_radius = 0.022
    wheel_clearance = 0.001
    mount_specs = (("high", 1), ("mid", 3), ("low", 5))
    for side_name, phi_value, edge_path, full_track in (
        ("left", runner_phi, panel_edge_left_path, full_left_track),
        ("right", -runner_phi, panel_edge_right_path, full_right_track),
    ):
        axis_dir = (-math.sin(phi_value), math.cos(phi_value), 0.0)
        for wheel_name, index in mount_specs:
            anchor_point = edge_path[index]
            track_point = full_track[index]
            normal_xy_len = math.hypot(track_point[0], track_point[1])
            normal_dir = (
                track_point[0] / normal_xy_len,
                track_point[1] / normal_xy_len,
                0.0,
            )
            wheel_center = (
                track_point[0] + normal_dir[0] * (rail_radius + wheel_radius + wheel_clearance),
                track_point[1] + normal_dir[1] * (rail_radius + wheel_radius + wheel_clearance),
                track_point[2],
            )
            axle_center = (
                wheel_center[0] - normal_dir[0] * 0.010,
                wheel_center[1] - normal_dir[1] * 0.010,
                wheel_center[2],
            )
            shutter.visual(
                _save_mesh(
                    f"{side_name}_bracket_{wheel_name}",
                    tube_from_spline_points(
                        [anchor_point, axle_center],
                        radius=0.011,
                        samples_per_segment=2,
                        radial_segments=14,
                        cap_ends=True,
                    ),
                ),
                material=frame_gray,
                name=f"{side_name}_bracket_{wheel_name}",
            )
            shutter.visual(
                Box((0.034, 0.034, 0.034)),
                origin=Origin(xyz=axle_center),
                material=frame_gray,
                name=f"{side_name}_axle_block_{wheel_name}",
            )
            shutter.visual(
                Cylinder(radius=wheel_radius, length=0.040),
                origin=Origin(
                    xyz=wheel_center,
                    rpy=(-math.pi / 2.0, 0.0, phi_value),
                ),
                material=roller_black,
                name=f"{side_name}_wheel_{wheel_name}",
            )
    shutter.inertial = Inertial.from_geometry(
        Box((1.60, 0.90, 1.55)),
        mass=42.0,
        origin=Origin(xyz=(0.72, 0.0, 0.66)),
    )

    model.articulation(
        "base_to_dome",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.40),
    )
    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.60,
            lower=0.0,
            upper=1.30,
        ),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float]:
    (min_corner, max_corner) = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_ring = object_model.get_part("base_ring")
    dome_shell = object_model.get_part("dome_shell")
    shutter = object_model.get_part("shutter")
    dome_joint = object_model.get_articulation("base_to_dome")
    shutter_joint = object_model.get_articulation("dome_to_shutter")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    for wheel_name, rail_name in (
        ("left_wheel_high", "guide_rail_left_high"),
        ("left_wheel_mid", "guide_rail_left"),
        ("left_wheel_low", "guide_rail_left_low"),
        ("right_wheel_high", "guide_rail_right_high"),
        ("right_wheel_mid", "guide_rail_right"),
        ("right_wheel_low", "guide_rail_right_low"),
    ):
        ctx.allow_overlap(
            shutter,
            dome_shell,
            elem_a=wheel_name,
            elem_b=rail_name,
            reason="Wheel cylinders intentionally stand in for grooved capture rollers hugging the dome guide rods.",
        )
    for block_name, rail_name in (
        ("left_axle_block_high", "guide_rail_left_high"),
        ("left_axle_block_mid", "guide_rail_left"),
        ("left_axle_block_low", "guide_rail_left_low"),
        ("right_axle_block_high", "guide_rail_right_high"),
        ("right_axle_block_mid", "guide_rail_right"),
        ("right_axle_block_low", "guide_rail_right_low"),
    ):
        ctx.allow_overlap(
            shutter,
            dome_shell,
            elem_a=block_name,
            elem_b=rail_name,
            reason="Axle blocks approximate clamped carriage housings that wrap the guide rod around each shutter roller.",
        )
    for bracket_name, rail_name in (
        ("left_bracket_high", "guide_rail_left_high"),
        ("left_bracket_mid", "guide_rail_left"),
        ("left_bracket_low", "guide_rail_left_low"),
        ("right_bracket_high", "guide_rail_right_high"),
        ("right_bracket_mid", "guide_rail_right"),
        ("right_bracket_low", "guide_rail_right_low"),
    ):
        ctx.allow_overlap(
            shutter,
            dome_shell,
            elem_a=bracket_name,
            elem_b=rail_name,
            reason="Each shutter bracket is a simplified one-piece carriage arm; the real stamped U-bracket would wrap around the guide rod without solid interference.",
        )
    for bracket_name in (
        "left_bracket_mid",
        "left_bracket_low",
        "right_bracket_mid",
        "right_bracket_low",
    ):
        ctx.allow_overlap(
            shutter,
            dome_shell,
            elem_a=bracket_name,
            elem_b="shell_skin",
            reason="The dome shell is modeled as a clean outer skin and omits the recessed slit-channel pocket that receives the closed shutter carriage arms.",
        )

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
        "key_parts_and_joints_exist",
        all(
            item is not None
            for item in (base_ring, dome_shell, shutter, dome_joint, shutter_joint)
        ),
        "Expected stationary base ring, rotating dome shell, shutter, and both articulations.",
    )
    ctx.expect_overlap(dome_shell, base_ring, axes="xy", min_overlap=2.4, name="dome_centered_on_ring")
    ctx.expect_contact(
        dome_shell,
        base_ring,
        elem_a="rotation_flange",
        elem_b="support_wheel_00",
        contact_tol=0.003,
        name="flange_supported_by_front_left_wheel",
    )
    ctx.expect_contact(
        dome_shell,
        base_ring,
        elem_a="rotation_flange",
        elem_b="support_wheel_03",
        contact_tol=0.003,
        name="flange_supported_by_rear_wheel",
    )
    ctx.expect_contact(
        shutter,
        dome_shell,
        elem_a="left_wheel_mid",
        elem_b="guide_rail_left",
        contact_tol=0.003,
        name="left_shutter_wheel_rides_left_rail",
    )
    ctx.expect_contact(
        shutter,
        dome_shell,
        elem_a="right_wheel_mid",
        elem_b="guide_rail_right",
        contact_tol=0.003,
        name="right_shutter_wheel_rides_right_rail",
    )

    closed_rib_aabb = ctx.part_element_world_aabb(dome_shell, elem="guide_rail_left")
    with ctx.pose({dome_joint: math.pi / 2.0}):
        rotated_rib_aabb = ctx.part_element_world_aabb(dome_shell, elem="guide_rail_left")
    if closed_rib_aabb is not None and rotated_rib_aabb is not None:
        closed_rib_center = _aabb_center(closed_rib_aabb)
        rotated_rib_center = _aabb_center(rotated_rib_aabb)
        ctx.check(
            "dome_rotates_about_vertical_axis",
            abs(rotated_rib_center[1]) > abs(closed_rib_center[1]) + 0.25
            and abs(rotated_rib_center[0]) < abs(closed_rib_center[0]) - 0.20,
            f"Closed rib center {closed_rib_center}, rotated rib center {rotated_rib_center}.",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(shutter, elem="panel_skin")
    with ctx.pose({shutter_joint: 0.8}):
        opened_panel_aabb = ctx.part_element_world_aabb(shutter, elem="panel_skin")
    if closed_panel_aabb is not None and opened_panel_aabb is not None:
        closed_panel_center = _aabb_center(closed_panel_aabb)
        opened_panel_center = _aabb_center(opened_panel_aabb)
        ctx.check(
            "shutter_opens_up_and_back",
            opened_panel_center[2] > closed_panel_center[2] + 0.22
            and opened_panel_center[0] < closed_panel_center[0] - 0.45,
            f"Closed panel center {closed_panel_center}, opened panel center {opened_panel_center}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
