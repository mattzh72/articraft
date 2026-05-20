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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_remote_weapon_station")

    def _mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _arc_points(
        radius: float,
        start_deg: float,
        end_deg: float,
        *,
        z: float,
        samples: int,
    ) -> list[tuple[float, float, float]]:
        if end_deg < start_deg:
            end_deg += 360.0
        return [
            (
                radius * math.cos(math.radians(start_deg + (end_deg - start_deg) * t)),
                radius * math.sin(math.radians(start_deg + (end_deg - start_deg) * t)),
                z,
            )
            for t in [index / samples for index in range(samples + 1)]
        ]

    def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(x, -y, z) for x, y, z in points]

    structural_olive = model.material("structural_olive", rgba=(0.33, 0.38, 0.29, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.19, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.24, 0.25, 0.27, 1.0))
    shield_gray = model.material("shield_gray", rgba=(0.45, 0.48, 0.50, 1.0))
    black_finish = model.material("black_finish", rgba=(0.07, 0.08, 0.09, 1.0))
    ammo_green = model.material("ammo_green", rgba=(0.40, 0.43, 0.31, 1.0))

    front_left_strut_points = [(0.19, 0.41, 0.135), (0.22, 0.43, 0.34), (0.25, 0.43, 0.56), (0.29, 0.41, 0.76)]
    rear_left_strut_points = [(-0.34, 0.30, 0.135), (-0.31, 0.36, 0.34), (-0.27, 0.41, 0.56), (-0.23, 0.45, 0.76)]
    left_shield_rail_points = [(-0.22, 0.43, 0.76), (-0.10, 0.38, 0.94), (0.18, 0.34, 1.10), (0.60, 0.32, 1.13), (0.82, 0.30, 0.94)]
    left_lug_brace_points = [(0.03, 0.23, 0.88), (0.08, 0.28, 0.97), (0.18, 0.34, 1.10)]
    left_cradle_frame_points = [
        (-0.16, 0.105, -0.12),
        (0.30, 0.105, -0.12),
        (0.42, 0.105, -0.08),
        (0.44, 0.105, 0.00),
        (0.04, 0.105, 0.02),
        (-0.16, 0.105, -0.03),
    ]

    lower_ring_mesh = _mesh(
        "azimuth_low_ring",
        TorusGeometry(radius=0.45, tube=0.035, radial_segments=18, tubular_segments=64),
    )
    upper_ring_mesh = _mesh(
        "upper_ring_arch_open_v4",
        tube_from_spline_points(
            [
                (
                    0.22 + 0.46 * math.cos(math.radians(angle_deg)),
                    0.0,
                    0.92 + 0.46 * math.sin(math.radians(angle_deg)),
                )
                for angle_deg in range(95, 266, 10)
            ],
            radius=0.022,
            samples_per_segment=10,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    front_left_strut_mesh = _mesh(
        "front_left_strut",
        tube_from_spline_points(
            front_left_strut_points,
            radius=0.022,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    front_right_strut_mesh = _mesh(
        "front_right_strut",
        tube_from_spline_points(
            _mirror_y(front_left_strut_points),
            radius=0.022,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    rear_left_strut_mesh = _mesh(
        "rear_left_strut",
        tube_from_spline_points(
            rear_left_strut_points,
            radius=0.022,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    rear_right_strut_mesh = _mesh(
        "rear_right_strut",
        tube_from_spline_points(
            _mirror_y(rear_left_strut_points),
            radius=0.022,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    left_shield_rail_mesh = _mesh(
        "left_shield_rail",
        tube_from_spline_points(
            left_shield_rail_points,
            radius=0.018,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    right_shield_rail_mesh = _mesh(
        "right_shield_rail",
        tube_from_spline_points(
            _mirror_y(left_shield_rail_points),
            radius=0.018,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    left_lug_brace_mesh = _mesh(
        "left_lug_brace",
        tube_from_spline_points(
            left_lug_brace_points,
            radius=0.018,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    right_lug_brace_mesh = _mesh(
        "right_lug_brace",
        tube_from_spline_points(
            _mirror_y(left_lug_brace_points),
            radius=0.018,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    left_cradle_frame_mesh = _mesh(
        "left_cradle_frame_v2",
        wire_from_points(
            left_cradle_frame_points,
            radius=0.015,
            radial_segments=14,
            closed_path=True,
            cap_ends=False,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=8,
        ),
    )
    right_cradle_frame_mesh = _mesh(
        "right_cradle_frame_v2",
        wire_from_points(
            _mirror_y(left_cradle_frame_points),
            radius=0.015,
            radial_segments=14,
            closed_path=True,
            cap_ends=False,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=8,
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((1.05, 1.05, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_metal,
        name="base_plate",
    )
    pedestal_base.visual(
        Cylinder(radius=0.27, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=gunmetal,
        name="pedestal_drum",
    )
    pedestal_base.visual(
        Cylinder(radius=0.43, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        material=shield_gray,
        name="bearing_collar",
    )
    pedestal_base.visual(
        Box((0.34, 0.24, 0.18)),
        origin=Origin(xyz=(-0.27, 0.0, 0.17)),
        material=structural_olive,
        name="power_pack",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((1.05, 1.05, 0.43)),
        mass=880.0,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
    )

    upper_mount = model.part("upper_mount")
    upper_mount.visual(
        Cylinder(radius=0.42, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=shield_gray,
        name="turntable_deck",
    )
    upper_mount.visual(
        lower_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=structural_olive,
        name="low_ring",
    )
    upper_mount.visual(
        Box((0.30, 0.42, 0.24)),
        origin=Origin(xyz=(-0.16, 0.0, 0.22)),
        material=structural_olive,
        name="rear_electronics_pack",
    )
    upper_mount.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(-0.10, 0.24, 0.16)),
        material=gunmetal,
        name="azimuth_drive_module",
    )
    upper_mount.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(-0.10, -0.24, 0.16)),
        material=gunmetal,
        name="counter_drive_cover",
    )
    upper_mount.visual(
        Box((0.10, 0.08, 0.66)),
        origin=Origin(xyz=(-0.18, 0.28, 0.43)),
        material=structural_olive,
        name="left_lug_support",
    )
    upper_mount.visual(
        Box((0.10, 0.08, 0.66)),
        origin=Origin(xyz=(-0.18, -0.28, 0.43)),
        material=structural_olive,
        name="right_lug_support",
    )
    upper_mount.visual(
        Box((0.08, 0.64, 0.08)),
        origin=Origin(xyz=(-0.22, 0.0, 0.96)),
        material=structural_olive,
        name="rear_bridge",
    )
    upper_mount.visual(
        Cylinder(radius=0.022, length=0.68),
        origin=Origin(xyz=(0.46, 0.0, 0.82), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structural_olive,
        name="upper_ring",
    )
    upper_mount.visual(
        Cylinder(radius=0.022, length=0.70),
        origin=Origin(xyz=(0.11, 0.34, 0.82), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=structural_olive,
        name="left_ring_side",
    )
    upper_mount.visual(
        Cylinder(radius=0.022, length=0.70),
        origin=Origin(xyz=(0.11, -0.34, 0.82), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=structural_olive,
        name="right_ring_side",
    )
    upper_mount.visual(
        Box((0.10, 0.04, 0.16)),
        origin=Origin(xyz=(0.04, 0.28, 0.70)),
        material=shield_gray,
        name="left_pitch_lug",
    )
    upper_mount.visual(
        Box((0.10, 0.04, 0.16)),
        origin=Origin(xyz=(0.04, -0.28, 0.70)),
        material=shield_gray,
        name="right_pitch_lug",
    )
    upper_mount.visual(
        Cylinder(radius=0.018, length=0.74),
        origin=Origin(xyz=(0.08, 0.36, 1.03), rpy=(0.0, 0.48, 0.0)),
        material=shield_gray,
        name="left_shield_rail",
    )
    upper_mount.visual(
        Cylinder(radius=0.018, length=0.74),
        origin=Origin(xyz=(0.08, -0.36, 1.03), rpy=(0.0, 0.48, 0.0)),
        material=shield_gray,
        name="right_shield_rail",
    )
    upper_mount.visual(
        Cylinder(radius=0.018, length=0.74),
        origin=Origin(xyz=(-0.10, 0.0, 1.18), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shield_gray,
        name="shield_rear_header",
    )
    upper_mount.visual(
        Cylinder(radius=0.018, length=0.70),
        origin=Origin(xyz=(0.22, 0.0, 1.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shield_gray,
        name="shield_top_header",
    )
    upper_mount.visual(
        Cylinder(radius=0.018, length=0.62),
        origin=Origin(xyz=(0.72, 0.0, 1.74), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shield_gray,
        name="shield_front_header",
    )
    upper_mount.inertial = Inertial.from_geometry(
        Box((1.58, 1.28, 1.46)),
        mass=360.0,
        origin=Origin(xyz=(0.08, 0.0, 0.74)),
    )

    gun_cradle = model.part("gun_cradle")
    gun_cradle.visual(
        Box((0.60, 0.03, 0.18)),
        origin=Origin(xyz=(0.10, 0.10, -0.12)),
        material=gunmetal,
        name="left_frame",
    )
    gun_cradle.visual(
        Box((0.60, 0.03, 0.18)),
        origin=Origin(xyz=(0.10, -0.10, -0.12)),
        material=gunmetal,
        name="right_frame",
    )
    gun_cradle.visual(
        Box((0.10, 0.04, 0.10)),
        origin=Origin(xyz=(0.02, 0.16, 0.0)),
        material=gunmetal,
        name="left_trunnion_carrier",
    )
    gun_cradle.visual(
        Box((0.10, 0.04, 0.10)),
        origin=Origin(xyz=(0.02, -0.16, 0.0)),
        material=gunmetal,
        name="right_trunnion_carrier",
    )
    gun_cradle.visual(
        Cylinder(radius=0.05, length=0.04),
        origin=Origin(xyz=(0.02, 0.24, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_finish,
        name="left_trunnion_hub",
    )
    gun_cradle.visual(
        Cylinder(radius=0.05, length=0.04),
        origin=Origin(xyz=(0.02, -0.24, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_finish,
        name="right_trunnion_hub",
    )
    gun_cradle.visual(
        Cylinder(radius=0.016, length=0.20),
        origin=Origin(xyz=(0.15, 0.0, -0.14), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="front_cross_tube",
    )
    gun_cradle.visual(
        Cylinder(radius=0.016, length=0.24),
        origin=Origin(xyz=(-0.06, 0.0, -0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="rear_cross_tube",
    )
    gun_cradle.visual(
        Cylinder(radius=0.014, length=0.20),
        origin=Origin(xyz=(0.10, 0.0, -0.03), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="top_cross_tube",
    )
    gun_cradle.visual(
        Box((0.16, 0.14, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, -0.18)),
        material=structural_olive,
        name="rear_actuator_pack",
    )
    gun_cradle.visual(
        Box((0.50, 0.14, 0.12)),
        origin=Origin(xyz=(0.28, 0.0, -0.10)),
        material=black_finish,
        name="receiver_body",
    )
    gun_cradle.visual(
        Box((0.24, 0.12, 0.04)),
        origin=Origin(xyz=(0.26, 0.0, -0.01)),
        material=black_finish,
        name="feed_cover",
    )
    gun_cradle.visual(
        Box((0.14, 0.10, 0.05)),
        origin=Origin(xyz=(0.10, 0.0, 0.05)),
        material=black_finish,
        name="optic_pod",
    )
    gun_cradle.visual(
        Cylinder(radius=0.042, length=0.32),
        origin=Origin(xyz=(0.58, 0.0, -0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_jacket",
    )
    gun_cradle.visual(
        Cylinder(radius=0.026, length=0.08),
        origin=Origin(xyz=(0.81, 0.0, -0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="gas_block",
    )
    gun_cradle.visual(
        Cylinder(radius=0.018, length=0.82),
        origin=Origin(xyz=(1.16, 0.0, -0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_finish,
        name="barrel",
    )
    gun_cradle.visual(
        Cylinder(radius=0.028, length=0.10),
        origin=Origin(xyz=(1.62, 0.0, -0.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_finish,
        name="muzzle_brake",
    )
    gun_cradle.inertial = Inertial.from_geometry(
        Box((1.84, 0.50, 0.32)),
        mass=185.0,
        origin=Origin(xyz=(0.68, 0.0, -0.08)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=upper_mount,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.9,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_mount,
        child=gun_cradle,
        origin=Origin(xyz=(0.02, 0.0, 0.70)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=9000.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    upper_mount = object_model.get_part("upper_mount")
    gun_cradle = object_model.get_part("gun_cradle")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    cradle_pitch = object_model.get_articulation("cradle_pitch")

    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

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
        "has_three_station_parts",
        len(object_model.parts) == 3,
        details=f"expected 3 authored parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "azimuth_axis_vertical",
        tuple(azimuth_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"azimuth axis was {azimuth_rotation.axis}",
    )
    ctx.check(
        "pitch_axis_horizontal",
        tuple(cradle_pitch.axis) == (0.0, -1.0, 0.0),
        details=f"pitch axis was {cradle_pitch.axis}",
    )

    ctx.expect_contact(upper_mount, pedestal_base, elem_a="turntable_deck", elem_b="bearing_collar")
    ctx.expect_contact(gun_cradle, upper_mount, elem_a="left_trunnion_hub", elem_b="left_pitch_lug")
    ctx.expect_contact(gun_cradle, upper_mount, elem_a="right_trunnion_hub", elem_b="right_pitch_lug")
    ctx.expect_gap(
        gun_cradle,
        pedestal_base,
        axis="z",
        positive_elem="receiver_body",
        min_gap=0.20,
    )

    drive_rest = ctx.part_element_world_aabb(upper_mount, elem="azimuth_drive_module")
    assert drive_rest is not None
    drive_rest_center = _center(drive_rest)
    with ctx.pose({azimuth_rotation: math.pi / 2.0}):
        drive_turned = ctx.part_element_world_aabb(upper_mount, elem="azimuth_drive_module")
        assert drive_turned is not None
        drive_turned_center = _center(drive_turned)
        planar_shift = math.hypot(
            drive_turned_center[0] - drive_rest_center[0],
            drive_turned_center[1] - drive_rest_center[1],
        )
        ctx.check(
            "azimuth_turntable_sweeps_upper_structure",
            planar_shift > 0.30,
            details=(
                f"drive module centers: rest={drive_rest_center}, "
                f"turned={drive_turned_center}, planar_shift={planar_shift}"
            ),
        )

    muzzle_rest = ctx.part_element_world_aabb(gun_cradle, elem="muzzle_brake")
    assert muzzle_rest is not None
    muzzle_rest_center = _center(muzzle_rest)
    with ctx.pose({cradle_pitch: 0.85}):
        muzzle_raised = ctx.part_element_world_aabb(gun_cradle, elem="muzzle_brake")
        assert muzzle_raised is not None
        muzzle_raised_center = _center(muzzle_raised)
        ctx.check(
            "pitch_raises_muzzle",
            muzzle_raised_center[2] > muzzle_rest_center[2] + 0.75,
            details=f"muzzle centers: rest={muzzle_rest_center}, raised={muzzle_raised_center}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_high_elevation")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
