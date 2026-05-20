from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return (
        (aabb[0][0] + aabb[1][0]) * 0.5,
        (aabb[0][1] + aabb[1][1]) * 0.5,
        (aabb[0][2] + aabb[1][2]) * 0.5,
    )


def _add_spoked_wheel(
    part,
    *,
    radius: float,
    width: float,
    rim_radius: float,
    hub_radius: float,
    tire_material,
    rim_material,
    hub_material,
    spoke_material,
    valve_material,
    spoke_count: int,
    axle_cap_offset: float,
) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=spin_origin,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.94, length=width * 0.72),
        origin=spin_origin,
        material=tire_material,
        name="tread_band",
    )
    part.visual(
        Cylinder(radius=rim_radius, length=width * 0.78),
        origin=spin_origin,
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=width * 1.02),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )

    spoke_length = rim_radius * 0.72
    spoke_center = rim_radius * 0.38
    for idx in range(spoke_count):
        angle = 2.0 * pi * idx / spoke_count
        part.visual(
            Box((width * 0.42, 0.010, spoke_length)),
            origin=Origin(
                xyz=(0.0, spoke_center * 0.62, 0.0),
                rpy=(angle, 0.0, 0.0),
            ),
            material=spoke_material,
            name=f"spoke_{idx + 1}",
        )

    part.visual(
        Cylinder(radius=0.0026, length=0.016),
        origin=Origin(
            xyz=(width * 0.16, 0.0, radius * 0.84),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=valve_material,
        name="valve_stem",
    )


def _build_front_caster(part, *, fork_material, hardware_material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=fork_material,
        name="stem",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=hardware_material,
        name="bearing_cap",
    )
    part.visual(
        _save_mesh(
            "walker_front_caster_fork.obj",
            wire_from_points(
                [
                    (-0.020, -0.095, -0.060),
                    (-0.020, -0.095, -0.006),
                    (-0.020, -0.050, 0.002),
                    (0.000, -0.012, -0.020),
                    (0.020, -0.050, 0.002),
                    (0.020, -0.095, -0.006),
                    (0.020, -0.095, -0.060),
                ],
                radius=0.0035,
                radial_segments=14,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.008,
                corner_segments=8,
            ),
        ),
        material=fork_material,
        name="fork_loop",
    )
    part.visual(
        Box((0.012, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, -0.014, -0.021)),
        material=fork_material,
        name="offset_link",
    )
    part.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.020, -0.095, -0.060), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="left_axle_stub",
    )
    part.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(-0.020, -0.095, -0.060), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="right_axle_stub",
    )
    part.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(0.020, -0.095, -0.060), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="left_axle_cap",
    )
    part.visual(
        Cylinder(radius=0.0055, length=0.006),
        origin=Origin(xyz=(-0.020, -0.095, -0.060), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="right_axle_cap",
    )


def _build_brake_lever(part, *, side_sign: float, lever_material, hardware_material) -> None:
    part.visual(
        Cylinder(radius=0.0075, length=0.026),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_material,
        name="pivot_barrel",
    )
    part.visual(
        Box((0.012, 0.074, 0.018)),
        origin=Origin(xyz=(0.0, -0.034, -0.006)),
        material=lever_material,
        name="lever_blade",
    )
    part.visual(
        Box((0.012, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.073, -0.017)),
        material=lever_material,
        name="lever_hook",
    )
    part.visual(
        Box((0.010, 0.010, 0.016)),
        origin=Origin(xyz=(0.009 * side_sign, 0.004, 0.006)),
        material=lever_material,
        name="parking_lock_tab",
    )
    part.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(
            xyz=(0.008 * side_sign, -0.082, -0.017),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=hardware_material,
        name="cable_barrel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_rolling_walker", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.29, 0.34, 0.27, 1.0))
    textured_black = model.material("textured_black", rgba=(0.12, 0.12, 0.12, 1.0))
    fork_graphite = model.material("fork_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.40, 0.43, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    nylon_grey = model.material("nylon_grey", rgba=(0.70, 0.72, 0.74, 1.0))
    tip_rubber = model.material("tip_rubber", rgba=(0.22, 0.23, 0.24, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.86, 0.43, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.74, 0.66, 0.96)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
    )

    left_side_points = [
        (0.285, -0.260, 0.044),
        (0.286, -0.185, 0.420),
        (0.282, -0.115, 0.735),
        (0.270, -0.020, 0.865),
        (0.258, 0.090, 0.872),
        (0.248, 0.182, 0.820),
        (0.245, 0.220, 0.162),
    ]
    right_side_points = _mirror_x(left_side_points)
    frame.visual(
        _save_mesh(
            "walker_left_side_rail.obj",
            tube_from_spline_points(
                left_side_points,
                radius=0.0145,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="left_side_rail",
    )
    frame.visual(
        _save_mesh(
            "walker_right_side_rail.obj",
            tube_from_spline_points(
                right_side_points,
                radius=0.0145,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="right_side_rail",
    )

    for name, points, radius in [
        (
            "front_upper_crossbar",
            [(-0.236, 0.188, 0.744), (0.000, 0.196, 0.748), (0.236, 0.188, 0.744)],
            0.0120,
        ),
        (
            "front_mid_crossbar",
            [(-0.236, 0.201, 0.522), (0.000, 0.206, 0.524), (0.236, 0.201, 0.522)],
            0.0110,
        ),
        (
            "front_lower_crossbar",
            [(-0.215, 0.208, 0.338), (0.000, 0.214, 0.342), (0.215, 0.208, 0.338)],
            0.0105,
        ),
        (
            "rear_crossbar",
            [(-0.246, -0.176, 0.307), (0.000, -0.170, 0.312), (0.246, -0.176, 0.307)],
            0.0100,
        ),
    ]:
        frame.visual(
            _save_mesh(
                f"walker_{name}.obj",
                tube_from_spline_points(
                    points,
                    radius=radius,
                    samples_per_segment=14,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=name,
        )

    for side_sign in (1.0, -1.0):
        side_name = "left" if side_sign > 0.0 else "right"
        x = 0.245 * side_sign
        _add_member(
            frame,
            (0.268 * side_sign, 0.062, 0.848),
            (0.236 * side_sign, 0.201, 0.522),
            0.0095,
            frame_paint,
            name=f"{side_name}_upper_diagonal",
        )
        _add_member(
            frame,
            (x, 0.216, 0.206),
            (0.210 * side_sign, 0.205, 0.342),
            0.0095,
            frame_paint,
            name=f"{'left' if side_sign > 0.0 else 'right'}_lower_knee_brace",
        )
        _add_member(
            frame,
            (0.285 * side_sign, -0.140, 0.105),
            (0.246 * side_sign, -0.176, 0.307),
            0.009,
            frame_paint,
            name=f"{side_name}_rear_stiffener",
        )
        frame.visual(
            Box((0.026, 0.040, 0.056)),
            origin=Origin(xyz=(0.236 * side_sign, 0.188, 0.744)),
            material=fork_graphite,
            name=f"{side_name}_upper_joint",
        )
        frame.visual(
            Box((0.024, 0.036, 0.050)),
            origin=Origin(xyz=(0.236 * side_sign, 0.201, 0.522)),
            material=fork_graphite,
            name=f"{side_name}_mid_joint",
        )
        frame.visual(
            Box((0.026, 0.042, 0.054)),
            origin=Origin(xyz=(0.246 * side_sign, -0.176, 0.307)),
            material=fork_graphite,
            name=f"{side_name}_rear_joint",
        )

    _add_member(
        frame,
        (0.246, -0.176, 0.307),
        (-0.236, 0.201, 0.522),
        0.0082,
        dark_steel,
        name="cross_brace_upper",
    )
    _add_member(
        frame,
        (-0.246, -0.176, 0.307),
        (0.236, 0.201, 0.522),
        0.0082,
        dark_steel,
        name="cross_brace_lower",
    )
    frame.visual(
        Box((0.076, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, -0.022, 0.431)),
        material=fork_graphite,
        name="brace_center_clamp",
    )

    frame.visual(
        Cylinder(radius=0.019, length=0.118),
        origin=Origin(xyz=(0.268, 0.030, 0.872), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="left_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.019, length=0.118),
        origin=Origin(xyz=(-0.268, 0.030, 0.872), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="right_handle_grip",
    )

    for side_sign in (1.0, -1.0):
        side_name = "left" if side_sign > 0.0 else "right"
        frame.visual(
            Box((0.046, 0.034, 0.026)),
            origin=Origin(xyz=(0.288 * side_sign, 0.062, 0.848)),
            material=textured_black,
            name=f"{side_name}_brake_clamp",
        )
        frame.visual(
            Cylinder(radius=0.0044, length=0.038),
            origin=Origin(
                xyz=(0.288 * side_sign, 0.062, 0.848),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=zinc_steel,
            name=f"{side_name}_brake_clamp_bolt",
        )
        frame.visual(
            Box((0.012, 0.018, 0.020)),
            origin=Origin(xyz=(0.302 * side_sign, 0.074, 0.860)),
            material=dark_steel,
            name=f"{side_name}_parking_sector",
        )

    frame.visual(
        Cylinder(radius=0.022, length=0.055),
        origin=Origin(xyz=(0.285, -0.260, 0.0275)),
        material=tip_rubber,
        name="left_rear_tip",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.055),
        origin=Origin(xyz=(-0.285, -0.260, 0.0275)),
        material=tip_rubber,
        name="right_rear_tip",
    )

    for side_sign in (1.0, -1.0):
        side_name = "left" if side_sign > 0.0 else "right"
        frame.visual(
            Cylinder(radius=0.018, length=0.036),
            origin=Origin(xyz=(0.245 * side_sign, 0.220, 0.138)),
            material=fork_graphite,
            name=f"{side_name}_front_socket",
        )
        frame.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.245 * side_sign, 0.220, 0.150)),
            material=fork_graphite,
            name=f"{side_name}_front_socket_collar",
        )
        frame.visual(
            Box((0.028, 0.026, 0.068)),
            origin=Origin(xyz=(0.245 * side_sign, 0.220, 0.176)),
            material=fork_graphite,
            name=f"{side_name}_socket_strut",
        )
        frame.visual(
            Box((0.028, 0.055, 0.070)),
            origin=Origin(xyz=(0.285 * side_sign, -0.140, 0.105)),
            material=fork_graphite,
            name=f"{side_name}_rear_mount",
        )
        frame.visual(
            Cylinder(radius=0.012, length=0.028),
            origin=Origin(
                xyz=(0.313 * side_sign, -0.140, 0.102),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=zinc_steel,
            name=f"{side_name}_rear_axle_boss",
        )
        frame.visual(
            Box((0.024, 0.052, 0.048)),
            origin=Origin(xyz=(0.286 * side_sign, -0.106, 0.106)),
            material=fork_graphite,
            name=f"{side_name}_rear_brake_housing",
        )
        frame.visual(
            Cylinder(radius=0.004, length=0.024),
            origin=Origin(
                xyz=(0.298 * side_sign, -0.105, 0.104),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=zinc_steel,
            name=f"{side_name}_brake_actuator",
        )
        frame.visual(
            _save_mesh(
                f"walker_{side_name}_brake_cable.obj",
                tube_from_spline_points(
                    [
                        (0.288 * side_sign, 0.062, 0.848),
                        (0.292 * side_sign, 0.012, 0.786),
                        (0.296 * side_sign, -0.056, 0.604),
                        (0.300 * side_sign, -0.098, 0.372),
                        (0.300 * side_sign, -0.108, 0.122),
                    ],
                    radius=0.0038,
                    samples_per_segment=16,
                    radial_segments=12,
                    up_hint=(0.0, 0.0, 1.0),
                ),
            ),
            material=textured_black,
            name=f"{side_name}_brake_cable",
        )

    left_front_caster = model.part("left_front_caster")
    left_front_caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.110, 0.130)),
        mass=0.45,
        origin=Origin(xyz=(0.0, -0.050, -0.060)),
    )
    _build_front_caster(left_front_caster, fork_material=fork_graphite, hardware_material=zinc_steel)

    right_front_caster = model.part("right_front_caster")
    right_front_caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.110, 0.130)),
        mass=0.45,
        origin=Origin(xyz=(0.0, -0.050, -0.060)),
    )
    _build_front_caster(right_front_caster, fork_material=fork_graphite, hardware_material=zinc_steel)

    left_front_wheel = model.part("left_front_wheel")
    left_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.024),
        mass=0.40,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_spoked_wheel(
        left_front_wheel,
        radius=0.060,
        width=0.024,
        rim_radius=0.046,
        hub_radius=0.015,
        tire_material=tire_rubber,
        rim_material=nylon_grey,
        hub_material=dark_steel,
        spoke_material=dark_steel,
        valve_material=safety_orange,
        spoke_count=5,
        axle_cap_offset=0.010,
    )

    right_front_wheel = model.part("right_front_wheel")
    right_front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.024),
        mass=0.40,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_spoked_wheel(
        right_front_wheel,
        radius=0.060,
        width=0.024,
        rim_radius=0.046,
        hub_radius=0.015,
        tire_material=tire_rubber,
        rim_material=nylon_grey,
        hub_material=dark_steel,
        spoke_material=dark_steel,
        valve_material=safety_orange,
        spoke_count=5,
        axle_cap_offset=-0.010,
    )

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.028),
        mass=0.70,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_spoked_wheel(
        left_rear_wheel,
        radius=0.090,
        width=0.028,
        rim_radius=0.070,
        hub_radius=0.021,
        tire_material=tire_rubber,
        rim_material=nylon_grey,
        hub_material=dark_steel,
        spoke_material=dark_steel,
        valve_material=safety_orange,
        spoke_count=6,
        axle_cap_offset=0.011,
    )

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.028),
        mass=0.70,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_spoked_wheel(
        right_rear_wheel,
        radius=0.090,
        width=0.028,
        rim_radius=0.070,
        hub_radius=0.021,
        tire_material=tire_rubber,
        rim_material=nylon_grey,
        hub_material=dark_steel,
        spoke_material=dark_steel,
        valve_material=safety_orange,
        spoke_count=6,
        axle_cap_offset=-0.011,
    )

    left_brake_lever = model.part("left_brake_lever")
    left_brake_lever.inertial = Inertial.from_geometry(
        Box((0.025, 0.040, 0.095)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.010, -0.040)),
    )
    _build_brake_lever(
        left_brake_lever,
        side_sign=1.0,
        lever_material=dark_steel,
        hardware_material=zinc_steel,
    )

    right_brake_lever = model.part("right_brake_lever")
    right_brake_lever.inertial = Inertial.from_geometry(
        Box((0.025, 0.040, 0.095)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.010, -0.040)),
    )
    _build_brake_lever(
        right_brake_lever,
        side_sign=-1.0,
        lever_material=dark_steel,
        hardware_material=zinc_steel,
    )

    model.articulation(
        "left_front_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_front_caster,
        origin=Origin(xyz=(0.245, 0.220, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )
    model.articulation(
        "right_front_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_front_caster,
        origin=Origin(xyz=(-0.245, 0.220, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )
    model.articulation(
        "left_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_front_caster,
        child=left_front_wheel,
        origin=Origin(xyz=(0.0, -0.095, -0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "right_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_front_caster,
        child=right_front_wheel,
        origin=Origin(xyz=(0.0, -0.095, -0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(0.34128, -0.140, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.34128, -0.140, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "left_brake_lever_pull",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_brake_lever,
        origin=Origin(xyz=(0.320, 0.062, 0.860)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=0.0, upper=0.58),
    )
    model.articulation(
        "right_brake_lever_pull",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_brake_lever,
        origin=Origin(xyz=(-0.320, 0.062, 0.860)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=0.0, upper=0.58),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_front_caster = object_model.get_part("left_front_caster")
    right_front_caster = object_model.get_part("right_front_caster")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")
    left_brake_lever = object_model.get_part("left_brake_lever")
    right_brake_lever = object_model.get_part("right_brake_lever")

    left_front_socket = frame.get_visual("left_front_socket")
    right_front_socket = frame.get_visual("right_front_socket")
    left_rear_mount = frame.get_visual("left_rear_mount")
    right_rear_mount = frame.get_visual("right_rear_mount")
    left_rear_axle_boss = frame.get_visual("left_rear_axle_boss")
    right_rear_axle_boss = frame.get_visual("right_rear_axle_boss")
    left_rear_brake_housing = frame.get_visual("left_rear_brake_housing")
    right_rear_brake_housing = frame.get_visual("right_rear_brake_housing")
    left_rear_tip = frame.get_visual("left_rear_tip")
    right_rear_tip = frame.get_visual("right_rear_tip")
    left_grip = frame.get_visual("left_handle_grip")
    right_grip = frame.get_visual("right_handle_grip")

    left_caster_stem = left_front_caster.get_visual("stem")
    right_caster_stem = right_front_caster.get_visual("stem")
    left_fork_loop = left_front_caster.get_visual("fork_loop")
    right_fork_loop = right_front_caster.get_visual("fork_loop")
    left_front_hub = left_front_wheel.get_visual("hub")
    right_front_hub = right_front_wheel.get_visual("hub")
    left_front_valve = left_front_wheel.get_visual("valve_stem")
    right_front_valve = right_front_wheel.get_visual("valve_stem")
    left_rear_tire = left_rear_wheel.get_visual("tire")
    right_rear_tire = right_rear_wheel.get_visual("tire")
    left_rear_hub = left_rear_wheel.get_visual("hub")
    right_rear_hub = right_rear_wheel.get_visual("hub")
    left_rear_valve = left_rear_wheel.get_visual("valve_stem")
    right_rear_valve = right_rear_wheel.get_visual("valve_stem")
    left_lever_blade = left_brake_lever.get_visual("lever_blade")
    right_lever_blade = right_brake_lever.get_visual("lever_blade")

    left_front_caster_swivel = object_model.get_articulation("left_front_caster_swivel")
    right_front_caster_swivel = object_model.get_articulation("right_front_caster_swivel")
    left_front_wheel_spin = object_model.get_articulation("left_front_wheel_spin")
    right_front_wheel_spin = object_model.get_articulation("right_front_wheel_spin")
    left_rear_wheel_spin = object_model.get_articulation("left_rear_wheel_spin")
    right_rear_wheel_spin = object_model.get_articulation("right_rear_wheel_spin")
    left_brake_lever_pull = object_model.get_articulation("left_brake_lever_pull")
    right_brake_lever_pull = object_model.get_articulation("right_brake_lever_pull")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    frame_aabb = ctx.part_world_aabb(frame)
    assert frame_aabb is not None
    frame_dx = frame_aabb[1][0] - frame_aabb[0][0]
    frame_dy = frame_aabb[1][1] - frame_aabb[0][1]
    frame_dz = frame_aabb[1][2] - frame_aabb[0][2]
    ctx.check(
        "walker_proportions",
        0.60 <= frame_dx <= 0.80 and 0.52 <= frame_dy <= 0.78 and 0.84 <= frame_dz <= 0.98,
        details=f"Unexpected frame size {(frame_dx, frame_dy, frame_dz)}",
    )

    ctx.expect_contact(
        left_front_caster,
        frame,
        elem_a=left_caster_stem,
        elem_b=left_front_socket,
        name="left_caster_socket_contact",
    )
    ctx.expect_contact(
        right_front_caster,
        frame,
        elem_a=right_caster_stem,
        elem_b=right_front_socket,
        name="right_caster_socket_contact",
    )
    ctx.expect_overlap(
        left_front_wheel,
        left_front_caster,
        axes="yz",
        min_overlap=0.014,
        elem_a=left_front_hub,
        elem_b=left_fork_loop,
        name="left_front_wheel_within_fork_projection",
    )
    ctx.expect_overlap(
        right_front_wheel,
        right_front_caster,
        axes="yz",
        min_overlap=0.014,
        elem_a=right_front_hub,
        elem_b=right_fork_loop,
        name="right_front_wheel_within_fork_projection",
    )

    ctx.expect_contact(
        left_rear_wheel,
        frame,
        elem_a=left_rear_hub,
        elem_b=left_rear_axle_boss,
        name="left_rear_wheel_axle_contact",
    )
    ctx.expect_contact(
        right_rear_wheel,
        frame,
        elem_a=right_rear_hub,
        elem_b=right_rear_axle_boss,
        name="right_rear_wheel_axle_contact",
    )
    ctx.expect_overlap(
        left_rear_wheel,
        frame,
        axes="yz",
        min_overlap=0.02,
        elem_a=left_rear_hub,
        elem_b=left_rear_axle_boss,
        name="left_rear_wheel_aligned_to_mount",
    )
    ctx.expect_overlap(
        right_rear_wheel,
        frame,
        axes="yz",
        min_overlap=0.02,
        elem_a=right_rear_hub,
        elem_b=right_rear_axle_boss,
        name="right_rear_wheel_aligned_to_mount",
    )
    ctx.expect_overlap(
        left_rear_wheel,
        frame,
        axes="yz",
        min_overlap=0.04,
        elem_a=left_rear_hub,
        elem_b=left_rear_mount,
        name="left_rear_wheel_supported_by_mount_plate",
    )
    ctx.expect_overlap(
        right_rear_wheel,
        frame,
        axes="yz",
        min_overlap=0.04,
        elem_a=right_rear_hub,
        elem_b=right_rear_mount,
        name="right_rear_wheel_supported_by_mount_plate",
    )

    ctx.expect_within(left_brake_lever, frame, axes="x", margin=0.04, name="left_lever_near_frame_width")
    ctx.expect_within(right_brake_lever, frame, axes="x", margin=0.04, name="right_lever_near_frame_width")
    ctx.expect_overlap(
        frame,
        left_rear_wheel,
        axes="yz",
        min_overlap=0.012,
        elem_a=left_rear_brake_housing,
        elem_b=left_rear_tire,
        name="left_brake_housing_tracks_rear_wheel",
    )
    ctx.expect_overlap(
        frame,
        right_rear_wheel,
        axes="yz",
        min_overlap=0.012,
        elem_a=right_rear_brake_housing,
        elem_b=right_rear_tire,
        name="right_brake_housing_tracks_rear_wheel",
    )

    left_tip_aabb = ctx.part_element_world_aabb(frame, elem=left_rear_tip)
    right_tip_aabb = ctx.part_element_world_aabb(frame, elem=right_rear_tip)
    assert left_tip_aabb is not None and right_tip_aabb is not None
    ctx.check(
        "rear_tips_reach_floor_plane",
        abs(left_tip_aabb[0][2]) <= 1e-6 and abs(right_tip_aabb[0][2]) <= 1e-6,
        details=f"Rear tips not on floor: left {left_tip_aabb}, right {right_tip_aabb}",
    )

    left_grip_aabb = ctx.part_element_world_aabb(frame, elem=left_grip)
    right_grip_aabb = ctx.part_element_world_aabb(frame, elem=right_grip)
    left_blade_aabb = ctx.part_element_world_aabb(left_brake_lever, elem=left_lever_blade)
    right_blade_aabb = ctx.part_element_world_aabb(right_brake_lever, elem=right_lever_blade)
    assert (
        left_grip_aabb is not None
        and right_grip_aabb is not None
        and left_blade_aabb is not None
        and right_blade_aabb is not None
    )
    left_grip_center = _aabb_center(left_grip_aabb)
    right_grip_center = _aabb_center(right_grip_aabb)
    left_blade_center = _aabb_center(left_blade_aabb)
    right_blade_center = _aabb_center(right_blade_aabb)
    ctx.check(
        "left_lever_below_grip",
        left_blade_center[2] < left_grip_center[2] - 0.01
        and left_blade_aabb[0][0] > left_grip_aabb[1][0] + 0.02,
        details=f"Left lever not hanging below/outboard of grip: blade={left_blade_aabb}, grip={left_grip_aabb}",
    )
    ctx.check(
        "right_lever_below_grip",
        right_blade_center[2] < right_grip_center[2] - 0.01
        and right_blade_aabb[1][0] < right_grip_aabb[0][0] - 0.02,
        details=f"Right lever not hanging below/outboard of grip: blade={right_blade_aabb}, grip={right_grip_aabb}",
    )

    left_front_rest = ctx.part_world_position(left_front_wheel)
    assert left_front_rest is not None
    with ctx.pose({left_front_caster_swivel: pi / 2.0}):
        left_front_swiveled = ctx.part_world_position(left_front_wheel)
        assert left_front_swiveled is not None
        ctx.check(
            "left_caster_swivel_moves_trailing_wheel",
            abs(left_front_swiveled[0] - left_front_rest[0]) > 0.008
            and abs(left_front_swiveled[1] - left_front_rest[1]) > 0.008,
            details=f"Left caster wheel did not move enough: rest={left_front_rest}, swivel={left_front_swiveled}",
        )
        ctx.expect_contact(
            left_front_caster,
            frame,
            elem_a=left_caster_stem,
            elem_b=left_front_socket,
            name="left_caster_contact_when_swiveled",
        )

    right_front_rest = ctx.part_world_position(right_front_wheel)
    assert right_front_rest is not None
    with ctx.pose({right_front_caster_swivel: -pi / 2.0}):
        right_front_swiveled = ctx.part_world_position(right_front_wheel)
        assert right_front_swiveled is not None
        ctx.check(
            "right_caster_swivel_moves_trailing_wheel",
            abs(right_front_swiveled[0] - right_front_rest[0]) > 0.008
            and abs(right_front_swiveled[1] - right_front_rest[1]) > 0.008,
            details=f"Right caster wheel did not move enough: rest={right_front_rest}, swivel={right_front_swiveled}",
        )

    left_front_valve_rest = ctx.part_element_world_aabb(left_front_wheel, elem=left_front_valve)
    right_front_valve_rest = ctx.part_element_world_aabb(right_front_wheel, elem=right_front_valve)
    left_rear_valve_rest = ctx.part_element_world_aabb(left_rear_wheel, elem=left_rear_valve)
    right_rear_valve_rest = ctx.part_element_world_aabb(right_rear_wheel, elem=right_rear_valve)
    assert (
        left_front_valve_rest is not None
        and right_front_valve_rest is not None
        and left_rear_valve_rest is not None
        and right_rear_valve_rest is not None
    )
    with ctx.pose(
        {
            left_front_wheel_spin: pi / 2.0,
            right_front_wheel_spin: pi / 2.0,
            left_rear_wheel_spin: pi / 2.0,
            right_rear_wheel_spin: pi / 2.0,
        }
    ):
        left_front_valve_spun = ctx.part_element_world_aabb(left_front_wheel, elem=left_front_valve)
        right_front_valve_spun = ctx.part_element_world_aabb(right_front_wheel, elem=right_front_valve)
        left_rear_valve_spun = ctx.part_element_world_aabb(left_rear_wheel, elem=left_rear_valve)
        right_rear_valve_spun = ctx.part_element_world_aabb(right_rear_wheel, elem=right_rear_valve)
        assert (
            left_front_valve_spun is not None
            and right_front_valve_spun is not None
            and left_rear_valve_spun is not None
            and right_rear_valve_spun is not None
        )
        left_front_center_rest = _aabb_center(left_front_valve_rest)
        right_front_center_rest = _aabb_center(right_front_valve_rest)
        left_rear_center_rest = _aabb_center(left_rear_valve_rest)
        right_rear_center_rest = _aabb_center(right_rear_valve_rest)
        left_front_center_spun = _aabb_center(left_front_valve_spun)
        right_front_center_spun = _aabb_center(right_front_valve_spun)
        left_rear_center_spun = _aabb_center(left_rear_valve_spun)
        right_rear_center_spun = _aabb_center(right_rear_valve_spun)
        ctx.check(
            "left_front_wheel_spin_moves_valve_stem",
            abs(left_front_center_spun[1] - left_front_center_rest[1]) > 0.030
            and abs(left_front_center_spun[2] - left_front_center_rest[2]) > 0.030,
            details=f"Left front valve stem did not move enough: {left_front_center_rest} -> {left_front_center_spun}",
        )
        ctx.check(
            "right_front_wheel_spin_moves_valve_stem",
            abs(right_front_center_spun[1] - right_front_center_rest[1]) > 0.030
            and abs(right_front_center_spun[2] - right_front_center_rest[2]) > 0.030,
            details=f"Right front valve stem did not move enough: {right_front_center_rest} -> {right_front_center_spun}",
        )
        ctx.check(
            "left_rear_wheel_spin_moves_valve_stem",
            abs(left_rear_center_spun[1] - left_rear_center_rest[1]) > 0.040
            and abs(left_rear_center_spun[2] - left_rear_center_rest[2]) > 0.040,
            details=f"Left rear valve stem did not move enough: {left_rear_center_rest} -> {left_rear_center_spun}",
        )
        ctx.check(
            "right_rear_wheel_spin_moves_valve_stem",
            abs(right_rear_center_spun[1] - right_rear_center_rest[1]) > 0.040
            and abs(right_rear_center_spun[2] - right_rear_center_rest[2]) > 0.040,
            details=f"Right rear valve stem did not move enough: {right_rear_center_rest} -> {right_rear_center_spun}",
        )

    left_lever_rest = ctx.part_element_world_aabb(left_brake_lever, elem=left_lever_blade)
    right_lever_rest = ctx.part_element_world_aabb(right_brake_lever, elem=right_lever_blade)
    assert left_lever_rest is not None and right_lever_rest is not None
    with ctx.pose({left_brake_lever_pull: 0.45, right_brake_lever_pull: 0.45}):
        left_lever_pulled = ctx.part_element_world_aabb(left_brake_lever, elem=left_lever_blade)
        right_lever_pulled = ctx.part_element_world_aabb(right_brake_lever, elem=right_lever_blade)
        assert left_lever_pulled is not None and right_lever_pulled is not None
        left_rest_center = _aabb_center(left_lever_rest)
        left_pulled_center = _aabb_center(left_lever_pulled)
        right_rest_center = _aabb_center(right_lever_rest)
        right_pulled_center = _aabb_center(right_lever_pulled)
        ctx.check(
            "left_brake_lever_pulls_toward_grip",
            left_pulled_center[1] > left_rest_center[1] + 0.004
            and left_pulled_center[2] < left_rest_center[2] - 0.01,
            details=f"Left brake lever center did not sweep realistically: {left_rest_center} -> {left_pulled_center}",
        )
        ctx.check(
            "right_brake_lever_pulls_toward_grip",
            right_pulled_center[1] > right_rest_center[1] + 0.004
            and right_pulled_center[2] < right_rest_center[2] - 0.01,
            details=f"Right brake lever center did not sweep realistically: {right_rest_center} -> {right_pulled_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
