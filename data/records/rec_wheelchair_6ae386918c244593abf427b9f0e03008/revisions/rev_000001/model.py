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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_wheelchair")

    frame_paint = model.material("frame_paint", rgba=(0.22, 0.24, 0.26, 1.0))
    polymer_shell = model.material("polymer_shell", rgba=(0.34, 0.37, 0.39, 1.0))
    pad_black = model.material("pad_black", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    stainless = model.material("stainless", rgba=(0.63, 0.66, 0.69, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _midpoint(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    def _distance(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def _rpy_for_cylinder(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        length_xy = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(length_xy, dz)
        return (0.0, pitch, yaw)

    def _add_member(part, a, b, radius: float, material, *, name: str) -> None:
        part.visual(
            Cylinder(radius=radius, length=_distance(a, b)),
            origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    def _mirror_x(
        points: list[tuple[float, float, float]]
    ) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    def _add_spokes(
        part,
        *,
        count: int,
        x_plane: float,
        inner_radius: float,
        outer_radius: float,
        spoke_radius: float,
        material,
        name_prefix: str,
        phase: float = 0.0,
    ) -> None:
        for index in range(count):
            angle = phase + (2.0 * math.pi * index / count)
            c = math.cos(angle)
            s = math.sin(angle)
            a = (x_plane, inner_radius * c, inner_radius * s)
            b = (x_plane, outer_radius * c, outer_radius * s)
            _add_member(
                part,
                a,
                b,
                spoke_radius,
                material,
                name=f"{name_prefix}_{index}",
            )

    drive_tire_mesh = _save_mesh(
        "drive_tire",
        TorusGeometry(radius=0.262, tube=0.030, radial_segments=18, tubular_segments=84).rotate_y(
            math.pi / 2.0
        ),
    )
    drive_rim_mesh = _save_mesh(
        "drive_rim",
        TorusGeometry(radius=0.194, tube=0.018, radial_segments=16, tubular_segments=72).rotate_y(
            math.pi / 2.0
        ),
    )
    handrim_mesh = _save_mesh(
        "drive_handrim",
        TorusGeometry(radius=0.286, tube=0.006, radial_segments=12, tubular_segments=84).rotate_y(
            math.pi / 2.0
        ),
    )
    caster_tire_mesh = _save_mesh(
        "caster_tire",
        TorusGeometry(radius=0.060, tube=0.017, radial_segments=14, tubular_segments=48).rotate_y(
            math.pi / 2.0
        ),
    )
    caster_rim_mesh = _save_mesh(
        "caster_rim",
        TorusGeometry(radius=0.045, tube=0.010, radial_segments=12, tubular_segments=40).rotate_y(
            math.pi / 2.0
        ),
    )

    chair_frame = model.part("chair_frame")

    left_lower_rail_points = [
        (0.220, -0.120, 0.470),
        (0.230, 0.050, 0.450),
        (0.225, 0.240, 0.350),
        (0.245, 0.385, 0.250),
    ]
    right_lower_rail_points = _mirror_x(left_lower_rail_points)
    chair_frame.visual(
        _save_mesh(
            "left_lower_rail",
            tube_from_spline_points(
                left_lower_rail_points,
                radius=0.017,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="left_lower_rail",
    )
    chair_frame.visual(
        _save_mesh(
            "right_lower_rail",
            tube_from_spline_points(
                right_lower_rail_points,
                radius=0.017,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="right_lower_rail",
    )

    _add_member(
        chair_frame,
        (0.220, -0.120, 0.470),
        (0.245, -0.275, 0.900),
        0.016,
        frame_paint,
        name="left_rear_upright",
    )
    _add_member(
        chair_frame,
        (-0.220, -0.120, 0.470),
        (-0.245, -0.275, 0.900),
        0.016,
        frame_paint,
        name="right_rear_upright",
    )
    _add_member(
        chair_frame,
        (0.240, -0.215, 0.705),
        (0.220, 0.135, 0.695),
        0.014,
        frame_paint,
        name="left_armrest_rail",
    )
    _add_member(
        chair_frame,
        (-0.240, -0.215, 0.705),
        (-0.220, 0.135, 0.695),
        0.014,
        frame_paint,
        name="right_armrest_rail",
    )
    _add_member(
        chair_frame,
        (0.220, 0.135, 0.695),
        (0.215, 0.220, 0.445),
        0.014,
        frame_paint,
        name="left_front_post",
    )
    _add_member(
        chair_frame,
        (-0.220, 0.135, 0.695),
        (-0.215, 0.220, 0.445),
        0.014,
        frame_paint,
        name="right_front_post",
    )
    _add_member(
        chair_frame,
        (-0.225, -0.105, 0.492),
        (0.225, -0.105, 0.492),
        0.013,
        frame_paint,
        name="rear_seat_crossbar",
    )
    _add_member(
        chair_frame,
        (-0.225, 0.110, 0.492),
        (0.225, 0.110, 0.492),
        0.013,
        frame_paint,
        name="front_seat_crossbar",
    )
    _add_member(
        chair_frame,
        (-0.225, -0.205, 0.705),
        (0.225, -0.205, 0.705),
        0.012,
        frame_paint,
        name="back_crossbar",
    )
    _add_member(
        chair_frame,
        (-0.180, 0.305, 0.270),
        (0.180, 0.305, 0.270),
        0.011,
        frame_paint,
        name="front_splay_bar",
    )
    _add_member(
        chair_frame,
        (0.180, 0.305, 0.270),
        (0.112, 0.500, 0.100),
        0.012,
        frame_paint,
        name="left_footrest_hanger",
    )
    _add_member(
        chair_frame,
        (-0.180, 0.305, 0.270),
        (-0.112, 0.500, 0.100),
        0.012,
        frame_paint,
        name="right_footrest_hanger",
    )
    _add_member(
        chair_frame,
        (0.180, 0.305, 0.270),
        (0.245, 0.385, 0.250),
        0.011,
        frame_paint,
        name="left_footrest_brace",
    )
    _add_member(
        chair_frame,
        (-0.180, 0.305, 0.270),
        (-0.245, 0.385, 0.250),
        0.011,
        frame_paint,
        name="right_footrest_brace",
    )

    chair_frame.visual(
        Box((0.430, 0.400, 0.026)),
        origin=Origin(xyz=(0.000, 0.000, 0.507)),
        material=polymer_shell,
        name="seat_pan",
    )
    chair_frame.visual(
        Box((0.445, 0.038, 0.018)),
        origin=Origin(xyz=(0.000, 0.205, 0.518)),
        material=polymer_shell,
        name="seat_drip_lip",
    )
    chair_frame.visual(
        Box((0.385, 0.040, 0.340)),
        origin=Origin(xyz=(0.000, -0.205, 0.735)),
        material=polymer_shell,
        name="back_shell",
    )
    chair_frame.visual(
        Box((0.405, 0.030, 0.018)),
        origin=Origin(xyz=(0.000, -0.225, 0.901)),
        material=polymer_shell,
        name="back_rain_lip",
    )
    chair_frame.visual(
        Box((0.038, 0.310, 0.024)),
        origin=Origin(xyz=(0.235, -0.020, 0.721)),
        material=pad_black,
        name="left_arm_pad",
    )
    chair_frame.visual(
        Box((0.038, 0.310, 0.024)),
        origin=Origin(xyz=(-0.235, -0.020, 0.721)),
        material=pad_black,
        name="right_arm_pad",
    )
    chair_frame.visual(
        Box((0.012, 0.310, 0.248)),
        origin=Origin(xyz=(0.220, -0.020, 0.576)),
        material=polymer_shell,
        name="left_side_guard",
    )
    chair_frame.visual(
        Box((0.012, 0.310, 0.248)),
        origin=Origin(xyz=(-0.220, -0.020, 0.576)),
        material=polymer_shell,
        name="right_side_guard",
    )
    chair_frame.visual(
        Box((0.085, 0.245, 0.024)),
        origin=Origin(xyz=(0.286, -0.020, 0.695)),
        material=polymer_shell,
        name="left_fender",
    )
    chair_frame.visual(
        Box((0.085, 0.245, 0.024)),
        origin=Origin(xyz=(-0.286, -0.020, 0.695)),
        material=polymer_shell,
        name="right_fender",
    )
    chair_frame.visual(
        Box((0.080, 0.016, 0.180)),
        origin=Origin(xyz=(0.250, -0.080, 0.380)),
        material=frame_paint,
        name="left_axle_plate",
    )
    chair_frame.visual(
        Box((0.080, 0.016, 0.180)),
        origin=Origin(xyz=(-0.250, -0.080, 0.380)),
        material=frame_paint,
        name="right_axle_plate",
    )
    chair_frame.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.315, -0.080, 0.310), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="left_axle_boss",
    )
    chair_frame.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(-0.315, -0.080, 0.310), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="right_axle_boss",
    )
    chair_frame.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.255, 0.465, 0.225)),
        material=stainless,
        name="left_caster_socket",
    )
    chair_frame.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(-0.255, 0.465, 0.225)),
        material=stainless,
        name="right_caster_socket",
    )
    chair_frame.visual(
        Box((0.040, 0.080, 0.020)),
        origin=Origin(xyz=(0.250, 0.425, 0.240)),
        material=frame_paint,
        name="left_caster_mount_block",
    )
    chair_frame.visual(
        Box((0.040, 0.080, 0.020)),
        origin=Origin(xyz=(-0.250, 0.425, 0.240)),
        material=frame_paint,
        name="right_caster_mount_block",
    )
    chair_frame.visual(
        Box((0.058, 0.070, 0.050)),
        origin=Origin(xyz=(0.220, -0.120, 0.490)),
        material=stainless,
        name="left_rear_joint_gusset",
    )
    chair_frame.visual(
        Box((0.058, 0.070, 0.050)),
        origin=Origin(xyz=(-0.220, -0.120, 0.490)),
        material=stainless,
        name="right_rear_joint_gusset",
    )
    chair_frame.visual(
        Box((0.052, 0.080, 0.050)),
        origin=Origin(xyz=(0.220, 0.215, 0.445)),
        material=stainless,
        name="left_front_joint_gusset",
    )
    chair_frame.visual(
        Box((0.052, 0.080, 0.050)),
        origin=Origin(xyz=(-0.220, 0.215, 0.445)),
        material=stainless,
        name="right_front_joint_gusset",
    )
    chair_frame.visual(
        Box((0.130, 0.090, 0.015)),
        origin=Origin(xyz=(0.112, 0.530, 0.100)),
        material=polymer_shell,
        name="left_footplate",
    )
    chair_frame.visual(
        Box((0.130, 0.090, 0.015)),
        origin=Origin(xyz=(-0.112, 0.530, 0.100)),
        material=polymer_shell,
        name="right_footplate",
    )
    chair_frame.visual(
        Box((0.130, 0.012, 0.050)),
        origin=Origin(xyz=(0.112, 0.570, 0.125)),
        material=polymer_shell,
        name="left_toe_stop",
    )
    chair_frame.visual(
        Box((0.130, 0.012, 0.050)),
        origin=Origin(xyz=(-0.112, 0.570, 0.125)),
        material=polymer_shell,
        name="right_toe_stop",
    )

    def _build_drive_wheel(part_name: str, *, side_sign: float):
        wheel = model.part(part_name)
        x_tire = side_sign * 0.058
        x_rim = side_sign * 0.052
        x_hub = side_sign * 0.045
        x_handrim = side_sign * 0.100
        wheel.visual(
            drive_tire_mesh,
            origin=Origin(xyz=(x_tire, 0.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            drive_rim_mesh,
            origin=Origin(xyz=(x_rim, 0.0, 0.0)),
            material=aluminum,
            name="rim_ring",
        )
        wheel.visual(
            Cylinder(radius=0.063, length=0.090),
            origin=Origin(xyz=(x_hub, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer_shell,
            name="hub_sleeve",
        )
        wheel.visual(
            Cylinder(radius=0.236, length=0.014),
            origin=Origin(xyz=(side_sign * 0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer_shell,
            name="weather_shield_disc",
        )
        wheel.visual(
            Cylinder(radius=0.062, length=0.006),
            origin=Origin(xyz=(side_sign * 0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer_shell,
            name="inner_bearing_cover",
        )
        wheel.visual(
            Cylinder(radius=0.058, length=0.006),
            origin=Origin(xyz=(side_sign * 0.087, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer_shell,
            name="outer_hub_cap",
        )
        _add_spokes(
            wheel,
            count=6,
            x_plane=x_hub,
            inner_radius=0.060,
            outer_radius=0.190,
            spoke_radius=0.008,
            material=aluminum,
            name_prefix="spoke",
            phase=math.pi / 6.0,
        )
        wheel.visual(
            handrim_mesh,
            origin=Origin(xyz=(x_handrim, 0.0, 0.0)),
            material=stainless,
            name="pushrim",
        )
        for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
            c = math.cos(angle)
            s = math.sin(angle)
            _add_member(
                wheel,
                (side_sign * 0.060, 0.220 * c, 0.220 * s),
                (x_handrim, 0.286 * c, 0.286 * s),
                0.0045,
                stainless,
                name=f"pushrim_standoff_{index}",
            )
        return wheel

    def _build_caster_yoke(part_name: str):
        yoke = model.part(part_name)
        yoke.visual(
            Cylinder(radius=0.030, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=polymer_shell,
            name="swivel_top_cap",
        )
        yoke.visual(
            Cylinder(radius=0.020, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=polymer_shell,
            name="swivel_seal",
        )
        yoke.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -0.054)),
            material=stainless,
            name="swivel_stem",
        )
        yoke.visual(
            Box((0.060, 0.028, 0.024)),
            origin=Origin(xyz=(0.0, 0.0, -0.090)),
            material=frame_paint,
            name="fork_crown",
        )
        yoke.visual(
            Box((0.012, 0.018, 0.110)),
            origin=Origin(xyz=(0.021, 0.0, -0.145)),
            material=frame_paint,
            name="left_fork_arm",
        )
        yoke.visual(
            Box((0.012, 0.018, 0.110)),
            origin=Origin(xyz=(-0.021, 0.0, -0.145)),
            material=frame_paint,
            name="right_fork_arm",
        )
        yoke.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.029, 0.0, -0.193), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="left_axle_pin",
        )
        yoke.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(-0.029, 0.0, -0.193), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="right_axle_pin",
        )
        return yoke

    def _build_caster_wheel(part_name: str):
        wheel = model.part(part_name)
        wheel.visual(caster_tire_mesh, material=rubber, name="tire")
        wheel.visual(caster_rim_mesh, material=aluminum, name="rim_ring")
        wheel.visual(
            Cylinder(radius=0.021, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer_shell,
            name="hub_sleeve",
        )
        wheel.visual(
            Cylinder(radius=0.019, length=0.008),
            origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer_shell,
            name="outer_cap",
        )
        wheel.visual(
            Cylinder(radius=0.019, length=0.008),
            origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer_shell,
            name="inner_cap",
        )
        wheel.visual(
            Cylinder(radius=0.050, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer_shell,
            name="sealed_disc",
        )
        _add_spokes(
            wheel,
            count=5,
            x_plane=0.0,
            inner_radius=0.017,
            outer_radius=0.041,
            spoke_radius=0.004,
            material=aluminum,
            name_prefix="spoke",
            phase=0.0,
        )
        return wheel

    left_drive_wheel = _build_drive_wheel("left_drive_wheel", side_sign=1.0)
    right_drive_wheel = _build_drive_wheel("right_drive_wheel", side_sign=-1.0)
    left_caster_yoke = _build_caster_yoke("left_caster_yoke")
    right_caster_yoke = _build_caster_yoke("right_caster_yoke")
    left_caster_wheel = _build_caster_wheel("left_caster_wheel")
    right_caster_wheel = _build_caster_wheel("right_caster_wheel")

    model.articulation(
        "left_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=chair_frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(0.340, -0.080, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=20.0),
    )
    model.articulation(
        "right_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=chair_frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.340, -0.080, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=20.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=chair_frame,
        child=left_caster_yoke,
        origin=Origin(xyz=(0.255, 0.465, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=chair_frame,
        child=right_caster_yoke,
        origin=Origin(xyz=(-0.255, 0.465, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )
    model.articulation(
        "left_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=left_caster_yoke,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.193)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )
    model.articulation(
        "right_caster_roll",
        ArticulationType.CONTINUOUS,
        parent=right_caster_yoke,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.193)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chair_frame = object_model.get_part("chair_frame")
    left_drive_wheel = object_model.get_part("left_drive_wheel")
    right_drive_wheel = object_model.get_part("right_drive_wheel")
    left_caster_yoke = object_model.get_part("left_caster_yoke")
    right_caster_yoke = object_model.get_part("right_caster_yoke")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_drive_spin = object_model.get_articulation("left_drive_spin")
    right_drive_spin = object_model.get_articulation("right_drive_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_roll = object_model.get_articulation("left_caster_roll")
    right_caster_roll = object_model.get_articulation("right_caster_roll")

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
        "all_major_parts_present",
        all(
            part is not None
            for part in (
                chair_frame,
                left_drive_wheel,
                right_drive_wheel,
                left_caster_yoke,
                right_caster_yoke,
                left_caster_wheel,
                right_caster_wheel,
            )
        ),
        "wheelchair missing a major frame or wheel subassembly",
    )
    ctx.check(
        "articulation_axes_are_mechanically_explicit",
        left_drive_spin.axis == (1.0, 0.0, 0.0)
        and right_drive_spin.axis == (1.0, 0.0, 0.0)
        and left_caster_swivel.axis == (0.0, 0.0, 1.0)
        and right_caster_swivel.axis == (0.0, 0.0, 1.0)
        and left_caster_roll.axis == (1.0, 0.0, 0.0)
        and right_caster_roll.axis == (1.0, 0.0, 0.0),
        "drive wheels should spin on transverse axles and casters should swivel about vertical stems",
    )
    ctx.expect_contact(
        chair_frame,
        left_drive_wheel,
        elem_a="left_axle_boss",
        elem_b="inner_bearing_cover",
        name="left_drive_wheel_bolts_to_frame",
    )
    ctx.expect_contact(
        chair_frame,
        right_drive_wheel,
        elem_a="right_axle_boss",
        elem_b="inner_bearing_cover",
        name="right_drive_wheel_bolts_to_frame",
    )
    ctx.expect_contact(
        chair_frame,
        left_caster_yoke,
        elem_a="left_caster_socket",
        elem_b="swivel_top_cap",
        name="left_caster_swivel_is_seated",
    )
    ctx.expect_contact(
        chair_frame,
        right_caster_yoke,
        elem_a="right_caster_socket",
        elem_b="swivel_top_cap",
        name="right_caster_swivel_is_seated",
    )
    ctx.expect_contact(
        left_caster_yoke,
        left_caster_wheel,
        name="left_caster_wheel_runs_on_fork_pins",
    )
    ctx.expect_contact(
        right_caster_yoke,
        right_caster_wheel,
        name="right_caster_wheel_runs_on_fork_pins",
    )

    with ctx.pose({left_caster_swivel: 0.9, right_caster_swivel: -0.9}):
        ctx.expect_contact(
            chair_frame,
            left_caster_yoke,
            elem_a="left_caster_socket",
            elem_b="swivel_top_cap",
            name="left_caster_seal_stays_engaged_when_swiveled",
        )
        ctx.expect_contact(
            chair_frame,
            right_caster_yoke,
            elem_a="right_caster_socket",
            elem_b="swivel_top_cap",
            name="right_caster_seal_stays_engaged_when_swiveled",
        )

    with ctx.pose({left_drive_spin: 1.3, right_drive_spin: -1.1}):
        ctx.expect_contact(
            chair_frame,
            left_drive_wheel,
            elem_a="left_axle_boss",
            elem_b="inner_bearing_cover",
            name="left_drive_hub_remains_supported_while_spinning",
        )
        ctx.expect_contact(
            chair_frame,
            right_drive_wheel,
            elem_a="right_axle_boss",
            elem_b="inner_bearing_cover",
            name="right_drive_hub_remains_supported_while_spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
