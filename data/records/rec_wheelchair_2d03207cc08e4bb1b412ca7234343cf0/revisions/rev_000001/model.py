from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _circle_points(radius: float, *, y: float = 0.0, segments: int = 28) -> list[tuple[float, float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), y, radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _ring_profile(outer_radius: float, inner_radius: float, half_width: float) -> list[tuple[float, float]]:
    return [
        (inner_radius, -half_width),
        (outer_radius * 0.98, -half_width),
        (outer_radius, -half_width * 0.45),
        (outer_radius, half_width * 0.45),
        (outer_radius * 0.98, half_width),
        (inner_radius, half_width),
        (inner_radius * 0.98, half_width * 0.35),
        (inner_radius * 0.98, -half_width * 0.35),
        (inner_radius, -half_width),
    ]


def _add_drive_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    wheel_radius: float,
    tire_width: float,
    hub_width: float,
    pushrim_side_sign: float,
    rubber,
    wheel_metal,
    hub_metal,
) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (wheel_radius * 0.86, -half_width),
        (wheel_radius * 0.96, -half_width * 0.72),
        (wheel_radius, -half_width * 0.18),
        (wheel_radius, half_width * 0.18),
        (wheel_radius * 0.96, half_width * 0.72),
        (wheel_radius * 0.86, half_width),
        (wheel_radius * 0.81, half_width * 0.50),
        (wheel_radius * 0.79, 0.0),
        (wheel_radius * 0.81, -half_width * 0.50),
        (wheel_radius * 0.86, -half_width),
    ]
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(tire_profile, segments=72).rotate_x(pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber, name="tire")

    rim_outer = wheel_radius * 0.90
    rim_inner = wheel_radius * 0.79
    rim_mesh = _save_mesh(
        f"{mesh_prefix}_rim",
        LatheGeometry(_ring_profile(rim_outer, rim_inner, tire_width * 0.22), segments=64).rotate_x(pi / 2.0),
    )
    part.visual(rim_mesh, material=wheel_metal, name="rim")

    part.visual(
        Cylinder(radius=0.024, length=hub_width),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hub_shell",
    )

    spoke_inner = 0.020
    spoke_outer = rim_outer + 0.002
    spoke_length = spoke_outer - spoke_inner
    spoke_mid = spoke_inner + spoke_length * 0.5
    for index in range(6):
        angle = 2.0 * pi * index / 6.0
        part.visual(
            Box((spoke_length, 0.0045, 0.0035)),
            origin=Origin(
                xyz=(spoke_mid * cos(angle), 0.0, spoke_mid * sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=wheel_metal,
            name=f"spoke_{index}",
        )

    pushrim_radius = rim_outer + 0.018
    pushrim_y = 0.028 * pushrim_side_sign
    pushrim_mesh = _save_mesh(
        f"{mesh_prefix}_pushrim",
        tube_from_spline_points(
            _circle_points(pushrim_radius, y=pushrim_y, segments=28),
            radius=0.0035,
            samples_per_segment=6,
            closed_spline=True,
            radial_segments=14,
            cap_ends=False,
        ),
    )
    part.visual(pushrim_mesh, material=wheel_metal, name="pushrim")

    for index, angle in enumerate((0.38 * pi, pi, 1.62 * pi)):
        part.visual(
            Box((pushrim_radius - rim_outer + 0.012, abs(pushrim_y) + 0.004, 0.005)),
            origin=Origin(
                xyz=(((spoke_outer + pushrim_radius) * 0.5) * cos(angle), pushrim_y * 0.5, ((spoke_outer + pushrim_radius) * 0.5) * sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=hub_metal,
            name=f"pushrim_tab_{index}",
        )


def _add_caster_wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    wheel_radius: float,
    tire_width: float,
    hub_width: float,
    rubber,
    wheel_metal,
) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (wheel_radius * 0.80, -half_width),
        (wheel_radius * 0.93, -half_width * 0.68),
        (wheel_radius, -half_width * 0.16),
        (wheel_radius, half_width * 0.16),
        (wheel_radius * 0.93, half_width * 0.68),
        (wheel_radius * 0.80, half_width),
        (wheel_radius * 0.74, half_width * 0.42),
        (wheel_radius * 0.72, 0.0),
        (wheel_radius * 0.74, -half_width * 0.42),
        (wheel_radius * 0.80, -half_width),
    ]
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_tire",
            LatheGeometry(tire_profile, segments=56).rotate_x(pi / 2.0),
        ),
        material=rubber,
        name="tire",
    )
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_rim",
            LatheGeometry(_ring_profile(wheel_radius * 0.92, wheel_radius * 0.70, tire_width * 0.24), segments=48).rotate_x(
                pi / 2.0
            ),
        ),
        material=wheel_metal,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.012, length=hub_width),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="hub_shell",
    )

    spoke_inner = 0.010
    spoke_outer = wheel_radius * 0.072 / 0.07
    spoke_length = spoke_outer - spoke_inner
    spoke_mid = spoke_inner + spoke_length * 0.5
    for index in range(4):
        angle = pi * 0.25 + 2.0 * pi * index / 4.0
        part.visual(
            Box((spoke_length, 0.0045, 0.0035)),
            origin=Origin(
                xyz=(spoke_mid * cos(angle), 0.0, spoke_mid * sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=wheel_metal,
            name=f"spoke_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_manual_wheelchair")

    frame_paint = model.material("frame_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    upholstery = model.material("upholstery", rgba=(0.11, 0.12, 0.13, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.50, 0.52, 0.56, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    footplate_finish = model.material("footplate_finish", rgba=(0.17, 0.18, 0.20, 1.0))

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.58, 0.72)),
        mass=12.0,
        origin=Origin(xyz=(0.02, 0.0, 0.36)),
    )

    left_side_rail = tube_from_spline_points(
        [(-0.14, 0.21, 0.50), (0.03, 0.21, 0.50), (0.17, 0.20, 0.46), (0.27, 0.18, 0.34)],
        radius=0.012,
        samples_per_segment=14,
        radial_segments=16,
    )
    right_side_rail = tube_from_spline_points(
        _mirror_y([(-0.14, 0.21, 0.50), (0.03, 0.21, 0.50), (0.17, 0.20, 0.46), (0.27, 0.18, 0.34)]),
        radius=0.012,
        samples_per_segment=14,
        radial_segments=16,
    )
    left_rear_brace = tube_from_spline_points(
        [(-0.04, 0.21, 0.255), (-0.09, 0.21, 0.38), (-0.14, 0.21, 0.50)],
        radius=0.010,
        samples_per_segment=12,
        radial_segments=14,
    )
    right_rear_brace = tube_from_spline_points(
        _mirror_y([(-0.04, 0.21, 0.255), (-0.09, 0.21, 0.38), (-0.14, 0.21, 0.50)]),
        radius=0.010,
        samples_per_segment=12,
        radial_segments=14,
    )
    left_front_leg = tube_from_spline_points(
        [(0.18, 0.18, 0.44), (0.24, 0.18, 0.33), (0.29, 0.18, 0.215)],
        radius=0.011,
        samples_per_segment=12,
        radial_segments=14,
    )
    right_front_leg = tube_from_spline_points(
        _mirror_y([(0.18, 0.18, 0.44), (0.24, 0.18, 0.33), (0.29, 0.18, 0.215)]),
        radius=0.011,
        samples_per_segment=12,
        radial_segments=14,
    )
    center_rigging_support = tube_from_spline_points(
        [(0.13, 0.0, 0.40), (0.16, 0.0, 0.43), (0.18, 0.0, 0.44)],
        radius=0.010,
        samples_per_segment=10,
        radial_segments=14,
    )

    main_frame.visual(_save_mesh("wheelchair_left_side_rail", left_side_rail), material=frame_paint, name="left_side_rail")
    main_frame.visual(_save_mesh("wheelchair_right_side_rail", right_side_rail), material=frame_paint, name="right_side_rail")
    main_frame.visual(_save_mesh("wheelchair_left_rear_brace", left_rear_brace), material=frame_paint, name="left_rear_brace")
    main_frame.visual(_save_mesh("wheelchair_right_rear_brace", right_rear_brace), material=frame_paint, name="right_rear_brace")
    main_frame.visual(_save_mesh("wheelchair_left_front_leg", left_front_leg), material=frame_paint, name="left_front_leg")
    main_frame.visual(_save_mesh("wheelchair_right_front_leg", right_front_leg), material=frame_paint, name="right_front_leg")
    main_frame.visual(
        _save_mesh("wheelchair_center_rigging_support", center_rigging_support),
        material=frame_paint,
        name="center_rigging_support",
    )

    main_frame.visual(
        Cylinder(radius=0.014, length=0.44),
        origin=Origin(xyz=(-0.04, 0.0, 0.255), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="axle_tube",
    )
    main_frame.visual(
        Cylinder(radius=0.011, length=0.38),
        origin=Origin(xyz=(0.18, 0.0, 0.44), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="front_cross_tube",
    )
    main_frame.visual(
        Cylinder(radius=0.011, length=0.42),
        origin=Origin(xyz=(-0.08, 0.0, 0.49), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="seat_cross_tube",
    )
    main_frame.visual(
        Box((0.34, 0.398, 0.010)),
        origin=Origin(xyz=(0.02, 0.0, 0.486)),
        material=upholstery,
        name="seat_sling",
    )
    main_frame.visual(
        Box((0.012, 0.36, 0.020)),
        origin=Origin(xyz=(-0.109, 0.0, 0.515)),
        material=frame_paint,
        name="backrest_stop",
    )
    main_frame.visual(
        Box((0.030, 0.36, 0.030)),
        origin=Origin(xyz=(-0.090, 0.0, 0.507)),
        material=frame_paint,
        name="backrest_stop_bridge",
    )
    main_frame.visual(
        Box((0.030, 0.044, 0.020)),
        origin=Origin(xyz=(0.13, 0.0, 0.40)),
        material=frame_paint,
        name="rigging_bridge",
    )
    main_frame.visual(
        Cylinder(radius=0.010, length=0.05),
        origin=Origin(xyz=(0.13, 0.045, 0.40), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="left_footrest_receiver",
    )
    main_frame.visual(
        Cylinder(radius=0.010, length=0.05),
        origin=Origin(xyz=(0.13, -0.045, 0.40), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="right_footrest_receiver",
    )
    main_frame.visual(
        Box((0.032, 0.032, 0.030)),
        origin=Origin(xyz=(0.29, 0.18, 0.200)),
        material=frame_paint,
        name="left_caster_mount",
    )
    main_frame.visual(
        Box((0.032, 0.032, 0.030)),
        origin=Origin(xyz=(0.29, -0.18, 0.200)),
        material=frame_paint,
        name="right_caster_mount",
    )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.14, 0.42, 0.46)),
        mass=2.4,
        origin=Origin(xyz=(0.00, 0.0, 0.23)),
    )
    left_back_upright = tube_from_spline_points(
        [(0.018, 0.19, 0.015), (0.004, 0.20, 0.20), (-0.02, 0.20, 0.42)],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=14,
    )
    right_back_upright = tube_from_spline_points(
        _mirror_y([(0.018, 0.19, 0.015), (0.004, 0.20, 0.20), (-0.02, 0.20, 0.42)]),
        radius=0.011,
        samples_per_segment=14,
        radial_segments=14,
    )
    backrest.visual(_save_mesh("wheelchair_left_back_upright", left_back_upright), material=frame_paint, name="left_upright")
    backrest.visual(_save_mesh("wheelchair_right_back_upright", right_back_upright), material=frame_paint, name="right_upright")
    backrest.visual(
        Cylinder(radius=0.011, length=0.40),
        origin=Origin(xyz=(-0.02, 0.0, 0.42), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="top_grip_bar",
    )
    backrest.visual(
        Box((0.020, 0.38, 0.020)),
        origin=Origin(xyz=(0.020, 0.0, 0.015)),
        material=frame_paint,
        name="stop_pad",
    )
    backrest.visual(
        Box((0.018, 0.40, 0.24)),
        origin=Origin(xyz=(-0.014, 0.0, 0.24)),
        material=upholstery,
        name="back_sling",
    )

    left_drive_wheel = model.part("left_drive_wheel")
    left_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.255, length=0.024),
        mass=2.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_drive_wheel_visuals(
        left_drive_wheel,
        mesh_prefix="left_drive_wheel",
        wheel_radius=0.255,
        tire_width=0.024,
        hub_width=0.070,
        pushrim_side_sign=1.0,
        rubber=rubber,
        wheel_metal=wheel_metal,
        hub_metal=hub_metal,
    )

    right_drive_wheel = model.part("right_drive_wheel")
    right_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.255, length=0.024),
        mass=2.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_drive_wheel_visuals(
        right_drive_wheel,
        mesh_prefix="right_drive_wheel",
        wheel_radius=0.255,
        tire_width=0.024,
        hub_width=0.070,
        pushrim_side_sign=-1.0,
        rubber=rubber,
        wheel_metal=wheel_metal,
        hub_metal=hub_metal,
    )

    left_caster_yoke = model.part("left_caster_yoke")
    left_caster_yoke.inertial = Inertial.from_geometry(
        Box((0.09, 0.05, 0.16)),
        mass=0.35,
        origin=Origin(xyz=(-0.020, 0.0, -0.075)),
    )
    left_caster_yoke.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=hub_metal,
        name="bearing_collar",
    )
    left_caster_yoke.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=hub_metal,
        name="stem",
    )
    left_caster_yoke.visual(
        Box((0.044, 0.050, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, -0.030)),
        material=frame_paint,
        name="fork_bridge",
    )
    left_caster_yoke.visual(
        Box((0.010, 0.008, 0.084)),
        origin=Origin(xyz=(-0.025, 0.019, -0.079)),
        material=frame_paint,
        name="left_fork_arm",
    )
    left_caster_yoke.visual(
        Box((0.010, 0.008, 0.084)),
        origin=Origin(xyz=(-0.025, -0.019, -0.079)),
        material=frame_paint,
        name="right_fork_arm",
    )

    right_caster_yoke = model.part("right_caster_yoke")
    right_caster_yoke.inertial = Inertial.from_geometry(
        Box((0.09, 0.05, 0.16)),
        mass=0.35,
        origin=Origin(xyz=(-0.020, 0.0, -0.075)),
    )
    right_caster_yoke.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=hub_metal,
        name="bearing_collar",
    )
    right_caster_yoke.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=hub_metal,
        name="stem",
    )
    right_caster_yoke.visual(
        Box((0.044, 0.050, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, -0.030)),
        material=frame_paint,
        name="fork_bridge",
    )
    right_caster_yoke.visual(
        Box((0.010, 0.008, 0.084)),
        origin=Origin(xyz=(-0.025, 0.019, -0.079)),
        material=frame_paint,
        name="left_fork_arm",
    )
    right_caster_yoke.visual(
        Box((0.010, 0.008, 0.084)),
        origin=Origin(xyz=(-0.025, -0.019, -0.079)),
        material=frame_paint,
        name="right_fork_arm",
    )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.024),
        mass=0.28,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        left_caster_wheel,
        mesh_prefix="left_caster_wheel",
        wheel_radius=0.070,
        tire_width=0.024,
        hub_width=0.034,
        rubber=rubber,
        wheel_metal=wheel_metal,
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.024),
        mass=0.28,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        right_caster_wheel,
        mesh_prefix="right_caster_wheel",
        wheel_radius=0.070,
        tire_width=0.024,
        hub_width=0.034,
        rubber=rubber,
        wheel_metal=wheel_metal,
    )

    left_footrest = model.part("left_footrest")
    left_footrest.inertial = Inertial.from_geometry(
        Box((0.08, 0.08, 0.24)),
        mass=0.55,
        origin=Origin(xyz=(-0.020, 0.0, 0.110)),
    )
    left_hanger = tube_from_spline_points(
        [(-0.008, 0.0, 0.014), (-0.020, 0.0, 0.10), (-0.040, 0.0, 0.22)],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=12,
    )
    left_footrest.visual(_save_mesh("left_footrest_hanger", left_hanger), material=frame_paint, name="hanger_tube")
    left_footrest.visual(
        Box((0.016, 0.014, 0.018)),
        origin=Origin(xyz=(-0.008, 0.016, 0.009)),
        material=frame_paint,
        name="left_hinge_cheek",
    )
    left_footrest.visual(
        Box((0.016, 0.014, 0.018)),
        origin=Origin(xyz=(-0.008, -0.016, 0.009)),
        material=frame_paint,
        name="right_hinge_cheek",
    )
    left_footrest.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(-0.040, 0.0, 0.22), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="upper_boss",
    )
    left_footrest.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hinge_left_knuckle",
    )
    left_footrest.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hinge_right_knuckle",
    )

    right_footrest = model.part("right_footrest")
    right_footrest.inertial = Inertial.from_geometry(
        Box((0.08, 0.08, 0.24)),
        mass=0.55,
        origin=Origin(xyz=(-0.020, 0.0, 0.110)),
    )
    right_hanger = tube_from_spline_points(
        [(-0.008, 0.0, 0.014), (-0.020, 0.0, 0.10), (-0.040, 0.0, 0.22)],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=12,
    )
    right_footrest.visual(_save_mesh("right_footrest_hanger", right_hanger), material=frame_paint, name="hanger_tube")
    right_footrest.visual(
        Box((0.016, 0.014, 0.018)),
        origin=Origin(xyz=(-0.008, 0.016, 0.009)),
        material=frame_paint,
        name="left_hinge_cheek",
    )
    right_footrest.visual(
        Box((0.016, 0.014, 0.018)),
        origin=Origin(xyz=(-0.008, -0.016, 0.009)),
        material=frame_paint,
        name="right_hinge_cheek",
    )
    right_footrest.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(-0.040, 0.0, 0.22), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="upper_boss",
    )
    right_footrest.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hinge_left_knuckle",
    )
    right_footrest.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hinge_right_knuckle",
    )

    left_footplate = model.part("left_footplate")
    left_footplate.inertial = Inertial.from_geometry(
        Box((0.11, 0.06, 0.03)),
        mass=0.22,
        origin=Origin(xyz=(0.055, 0.0, 0.012)),
    )
    left_footplate.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hinge_bar",
    )
    left_footplate.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(0.015, 0.0, 0.008)),
        material=footplate_finish,
        name="hinge_web",
    )
    left_footplate.visual(
        Box((0.098, 0.06, 0.008)),
        origin=Origin(xyz=(0.074, 0.0, 0.004)),
        material=footplate_finish,
        name="plate_surface",
    )
    left_footplate.visual(
        Box((0.012, 0.06, 0.018)),
        origin=Origin(xyz=(0.127, 0.0, 0.013)),
        material=footplate_finish,
        name="toe_lip",
    )

    right_footplate = model.part("right_footplate")
    right_footplate.inertial = Inertial.from_geometry(
        Box((0.11, 0.06, 0.03)),
        mass=0.22,
        origin=Origin(xyz=(0.055, 0.0, 0.012)),
    )
    right_footplate.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_metal,
        name="hinge_bar",
    )
    right_footplate.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(0.015, 0.0, 0.008)),
        material=footplate_finish,
        name="hinge_web",
    )
    right_footplate.visual(
        Box((0.098, 0.06, 0.008)),
        origin=Origin(xyz=(0.074, 0.0, 0.004)),
        material=footplate_finish,
        name="plate_surface",
    )
    right_footplate.visual(
        Box((0.012, 0.06, 0.018)),
        origin=Origin(xyz=(0.127, 0.0, 0.013)),
        material=footplate_finish,
        name="toe_lip",
    )

    model.articulation(
        "main_to_backrest",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=backrest,
        origin=Origin(xyz=(-0.145, 0.0, 0.50)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "main_to_left_drive_wheel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(-0.04, 0.255, 0.255)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=18.0),
    )
    model.articulation(
        "main_to_right_drive_wheel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.04, -0.255, 0.255)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=18.0),
    )
    model.articulation(
        "main_to_left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_caster_yoke,
        origin=Origin(xyz=(0.29, 0.18, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=10.0),
    )
    model.articulation(
        "main_to_right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_caster_yoke,
        origin=Origin(xyz=(0.29, -0.18, 0.185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=10.0),
    )
    model.articulation(
        "left_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=left_caster_yoke,
        child=left_caster_wheel,
        origin=Origin(xyz=(-0.025, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=right_caster_yoke,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.025, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "main_to_left_footrest",
        ArticulationType.FIXED,
        parent=main_frame,
        child=left_footrest,
        origin=Origin(xyz=(0.17, 0.095, 0.18)),
    )
    model.articulation(
        "main_to_right_footrest",
        ArticulationType.FIXED,
        parent=main_frame,
        child=right_footrest,
        origin=Origin(xyz=(0.17, -0.095, 0.18)),
    )
    model.articulation(
        "left_footrest_to_plate",
        ArticulationType.REVOLUTE,
        parent=left_footrest,
        child=left_footplate,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "right_footrest_to_plate",
        ArticulationType.REVOLUTE,
        parent=right_footrest,
        child=right_footplate,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_frame = object_model.get_part("main_frame")
    backrest = object_model.get_part("backrest")
    left_drive_wheel = object_model.get_part("left_drive_wheel")
    right_drive_wheel = object_model.get_part("right_drive_wheel")
    left_caster_yoke = object_model.get_part("left_caster_yoke")
    right_caster_yoke = object_model.get_part("right_caster_yoke")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_footrest = object_model.get_part("left_footrest")
    right_footrest = object_model.get_part("right_footrest")
    left_footplate = object_model.get_part("left_footplate")
    right_footplate = object_model.get_part("right_footplate")

    backrest_hinge = object_model.get_articulation("main_to_backrest")
    left_caster_swivel = object_model.get_articulation("main_to_left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("main_to_right_caster_swivel")
    left_plate_hinge = object_model.get_articulation("left_footrest_to_plate")
    right_plate_hinge = object_model.get_articulation("right_footrest_to_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        backrest,
        main_frame,
        elem_a="stop_pad",
        elem_b="backrest_stop",
        contact_tol=0.0015,
        name="backrest_upright_stop_contacts_frame",
    )
    ctx.expect_contact(
        left_drive_wheel,
        main_frame,
        elem_a="hub_shell",
        elem_b="axle_tube",
        contact_tol=0.0015,
        name="left_drive_wheel_supported_on_axle",
    )
    ctx.expect_contact(
        right_drive_wheel,
        main_frame,
        elem_a="hub_shell",
        elem_b="axle_tube",
        contact_tol=0.0015,
        name="right_drive_wheel_supported_on_axle",
    )
    ctx.expect_contact(
        left_caster_yoke,
        main_frame,
        elem_a="bearing_collar",
        elem_b="left_caster_mount",
        contact_tol=0.0015,
        name="left_caster_swivel_bearing_seats_on_mount",
    )
    ctx.expect_contact(
        right_caster_yoke,
        main_frame,
        elem_a="bearing_collar",
        elem_b="right_caster_mount",
        contact_tol=0.0015,
        name="right_caster_swivel_bearing_seats_on_mount",
    )
    ctx.expect_contact(
        left_caster_wheel,
        left_caster_yoke,
        elem_a="hub_shell",
        contact_tol=0.0015,
        name="left_caster_wheel_supported_in_fork",
    )
    ctx.expect_contact(
        right_caster_wheel,
        right_caster_yoke,
        elem_a="hub_shell",
        contact_tol=0.0015,
        name="right_caster_wheel_supported_in_fork",
    )
    ctx.expect_contact(
        left_footrest,
        main_frame,
        elem_a="upper_boss",
        elem_b="left_footrest_receiver",
        contact_tol=0.0015,
        name="left_footrest_mounted_to_receiver",
    )
    ctx.expect_contact(
        right_footrest,
        main_frame,
        elem_a="upper_boss",
        elem_b="right_footrest_receiver",
        contact_tol=0.0015,
        name="right_footrest_mounted_to_receiver",
    )
    ctx.expect_contact(
        left_footplate,
        left_footrest,
        elem_a="hinge_bar",
        contact_tol=0.0015,
        name="left_footplate_hinge_captured",
    )
    ctx.expect_contact(
        right_footplate,
        right_footrest,
        elem_a="hinge_bar",
        contact_tol=0.0015,
        name="right_footplate_hinge_captured",
    )
    ctx.expect_gap(
        left_footplate,
        left_caster_wheel,
        axis="z",
        min_gap=0.025,
        name="left_footplate_clears_left_caster",
    )
    ctx.expect_gap(
        right_footplate,
        right_caster_wheel,
        axis="z",
        min_gap=0.025,
        name="right_footplate_clears_right_caster",
    )

    with ctx.pose({left_caster_swivel: pi / 2.0, right_caster_swivel: -pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="caster_swivel_pose_clear")

    with ctx.pose({backrest_hinge: 1.20, left_plate_hinge: 1.35, right_plate_hinge: 1.35}):
        ctx.fail_if_parts_overlap_in_current_pose(name="stowed_pose_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
