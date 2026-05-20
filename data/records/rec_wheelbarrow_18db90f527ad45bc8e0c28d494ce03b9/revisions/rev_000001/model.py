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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _add_front_wheel_visuals(part, *, rubber, galvanized, frame_steel) -> None:
    tire_radius = 0.19
    tire_width = 0.10
    hub_radius = 0.040

    tire_profile = [
        (0.084, -0.050),
        (0.118, -0.049),
        (0.154, -0.043),
        (0.178, -0.025),
        (0.190, -0.010),
        (0.190, 0.010),
        (0.178, 0.025),
        (0.154, 0.043),
        (0.118, 0.049),
        (0.084, 0.050),
        (0.074, 0.028),
        (0.071, 0.000),
        (0.074, -0.028),
        (0.084, -0.050),
    ]
    part.visual(
        _save_mesh(
            "front_wheel_tire",
            LatheGeometry(tire_profile, segments=64).rotate_y(pi / 2.0),
        ),
        material=rubber,
        name="tire",
    )

    rim_outer = superellipse_profile(0.250, 0.250, exponent=2.0, segments=48)
    center_hole = superellipse_profile(0.072, 0.072, exponent=2.0, segments=24)
    window_profile = rounded_rect_profile(0.030, 0.080, 0.010, corner_segments=6)
    hole_profiles = [center_hole]
    for spoke_index in range(4):
        angle = spoke_index * (pi / 2.0)
        hole_profiles.append(
            _transform_profile(
                window_profile,
                dx=cos(angle) * 0.060,
                dy=sin(angle) * 0.060,
                angle=angle,
            )
        )

    rim_face_mesh = _save_mesh(
        "front_wheel_rim_face",
        ExtrudeWithHolesGeometry(
            rim_outer,
            hole_profiles,
            height=0.008,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    part.visual(
        rim_face_mesh,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=galvanized,
        name="rim_face_right",
    )
    part.visual(
        rim_face_mesh,
        origin=Origin(xyz=(-0.022, 0.0, 0.0)),
        material=galvanized,
        name="rim_face_left",
    )
    part.visual(
        Cylinder(radius=0.100, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=0.115),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_steel,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="left_bearing_cap",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(-0.058, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="right_bearing_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_paint = model.material("tray_paint", rgba=(0.77, 0.12, 0.10, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.17, 0.18, 0.18, 1.0))
    galvanized = model.material("galvanized", rgba=(0.63, 0.66, 0.70, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    grip = model.material("grip", rgba=(0.16, 0.13, 0.08, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.78, 1.62, 0.72)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.00, 0.36)),
    )

    chassis.visual(
        Box((0.56, 0.62, 0.018)),
        origin=Origin(xyz=(0.0, 0.00, 0.41)),
        material=tray_paint,
        name="tray_floor",
    )
    chassis.visual(
        Box((0.018, 0.64, 0.18)),
        origin=Origin(xyz=(0.27, 0.00, 0.50), rpy=(0.0, 0.20, 0.0)),
        material=tray_paint,
        name="tray_right_wall",
    )
    chassis.visual(
        Box((0.018, 0.64, 0.18)),
        origin=Origin(xyz=(-0.27, 0.00, 0.50), rpy=(0.0, -0.20, 0.0)),
        material=tray_paint,
        name="tray_left_wall",
    )
    chassis.visual(
        Box((0.46, 0.018, 0.21)),
        origin=Origin(xyz=(0.0, 0.31, 0.51), rpy=(-0.18, 0.0, 0.0)),
        material=tray_paint,
        name="tray_front_wall",
    )
    chassis.visual(
        Box((0.40, 0.018, 0.16)),
        origin=Origin(xyz=(0.0, -0.31, 0.47), rpy=(0.10, 0.0, 0.0)),
        material=tray_paint,
        name="tray_rear_wall",
    )
    chassis.visual(
        Box((0.028, 0.66, 0.018)),
        origin=Origin(xyz=(0.286, 0.00, 0.588), rpy=(0.0, 0.20, 0.0)),
        material=tray_paint,
        name="tray_right_rim",
    )
    chassis.visual(
        Box((0.028, 0.66, 0.018)),
        origin=Origin(xyz=(-0.286, 0.00, 0.588), rpy=(0.0, -0.20, 0.0)),
        material=tray_paint,
        name="tray_left_rim",
    )
    chassis.visual(
        Box((0.48, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.345, 0.606), rpy=(-0.18, 0.0, 0.0)),
        material=tray_paint,
        name="tray_front_rim",
    )
    chassis.visual(
        Box((0.42, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, -0.323, 0.542), rpy=(0.10, 0.0, 0.0)),
        material=tray_paint,
        name="tray_rear_rim",
    )

    left_handle_geom = tube_from_spline_points(
        [
            (0.26, -0.76, 0.45),
            (0.25, -0.52, 0.42),
            (0.22, -0.18, 0.39),
            (0.18, 0.20, 0.34),
            (0.10, 0.58, 0.28),
        ],
        radius=0.022,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_handle_geom = tube_from_spline_points(
        _mirror_x(
            [
                (0.26, -0.76, 0.45),
                (0.25, -0.52, 0.42),
                (0.22, -0.18, 0.39),
                (0.18, 0.20, 0.34),
                (0.10, 0.58, 0.28),
            ]
        ),
        radius=0.022,
        samples_per_segment=16,
        radial_segments=18,
    )
    chassis.visual(_save_mesh("left_handle", left_handle_geom), material=frame_steel, name="left_handle")
    chassis.visual(_save_mesh("right_handle", right_handle_geom), material=frame_steel, name="right_handle")

    left_leg_geom = tube_from_spline_points(
        [
            (0.23, -0.53, 0.41),
            (0.20, -0.57, 0.22),
            (0.14, -0.61, 0.03),
        ],
        radius=0.018,
        samples_per_segment=10,
        radial_segments=16,
    )
    right_leg_geom = tube_from_spline_points(
        _mirror_x(
            [
                (0.23, -0.53, 0.41),
                (0.20, -0.57, 0.22),
                (0.14, -0.61, 0.03),
            ]
        ),
        radius=0.018,
        samples_per_segment=10,
        radial_segments=16,
    )
    chassis.visual(_save_mesh("left_leg", left_leg_geom), material=frame_steel, name="left_leg")
    chassis.visual(_save_mesh("right_leg", right_leg_geom), material=frame_steel, name="right_leg")

    left_fork_geom = tube_from_spline_points(
        [
            (0.10, 0.58, 0.28),
            (0.11, 0.72, 0.25),
            (0.10, 0.83, 0.21),
        ],
        radius=0.018,
        samples_per_segment=8,
        radial_segments=16,
    )
    right_fork_geom = tube_from_spline_points(
        _mirror_x(
            [
                (0.10, 0.58, 0.28),
                (0.11, 0.72, 0.25),
                (0.10, 0.83, 0.21),
            ]
        ),
        radius=0.018,
        samples_per_segment=8,
        radial_segments=16,
    )
    chassis.visual(_save_mesh("left_fork", left_fork_geom), material=frame_steel, name="left_fork")
    chassis.visual(_save_mesh("right_fork", right_fork_geom), material=frame_steel, name="right_fork")

    chassis.visual(
        Box((0.46, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.18, 0.37)),
        material=frame_steel,
        name="rear_support",
    )
    chassis.visual(
        Box((0.40, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.20, 0.33)),
        material=frame_steel,
        name="front_support",
    )
    chassis.visual(
        Box((0.24, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.58, 0.28)),
        material=frame_steel,
        name="fork_crown",
    )
    chassis.visual(
        Box((0.40, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.57, 0.20)),
        material=frame_steel,
        name="leg_crossbar",
    )
    chassis.visual(
        Box((0.07, 0.03, 0.032)),
        origin=Origin(xyz=(0.14, -0.61, 0.016)),
        material=galvanized,
        name="left_foot",
    )
    chassis.visual(
        Box((0.07, 0.03, 0.032)),
        origin=Origin(xyz=(-0.14, -0.61, 0.016)),
        material=galvanized,
        name="right_foot",
    )
    chassis.visual(
        Box((0.12, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 0.39, 0.30), rpy=(-0.16, 0.0, 0.0)),
        material=frame_steel,
        name="nose_support",
    )
    chassis.visual(
        Box((0.026, 0.04, 0.08)),
        origin=Origin(xyz=(0.075, 0.84, 0.20)),
        material=frame_steel,
        name="left_dropout",
    )
    chassis.visual(
        Box((0.026, 0.04, 0.08)),
        origin=Origin(xyz=(-0.075, 0.84, 0.20)),
        material=frame_steel,
        name="right_dropout",
    )
    chassis.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(0.29, -0.78, 0.45), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip,
        name="right_grip",
    )
    chassis.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(-0.29, -0.78, 0.45), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip,
        name="left_grip",
    )

    wheel = model.part("front_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.10),
        mass=3.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_front_wheel_visuals(wheel, rubber=rubber, galvanized=galvanized, frame_steel=frame_steel)

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.84, 0.19)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=24.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    wheel = object_model.get_part("front_wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")
    limits = wheel_spin.motion_limits

    ctx.check(
        "front wheel uses continuous axle spin",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_spin.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}, limits={limits}",
    )

    ctx.expect_contact(
        wheel,
        chassis,
        elem_a="left_bearing_cap",
        elem_b="left_dropout",
        name="left bearing cap seats in fork dropout",
    )
    ctx.expect_contact(
        wheel,
        chassis,
        elem_a="right_bearing_cap",
        elem_b="right_dropout",
        name="right bearing cap seats in fork dropout",
    )
    ctx.expect_gap(
        chassis,
        wheel,
        axis="z",
        positive_elem="tray_floor",
        negative_elem="tire",
        min_gap=0.015,
        max_gap=0.05,
        name="tire clears underside of tray floor",
    )

    chassis_aabb = ctx.part_world_aabb(chassis)
    wheel_aabb = ctx.part_world_aabb(wheel)
    left_foot_aabb = ctx.part_element_world_aabb(chassis, elem="left_foot")
    right_foot_aabb = ctx.part_element_world_aabb(chassis, elem="right_foot")
    support_plane_ok = (
        wheel_aabb is not None
        and left_foot_aabb is not None
        and right_foot_aabb is not None
        and abs(left_foot_aabb[0][2] - wheel_aabb[0][2]) <= 0.02
        and abs(right_foot_aabb[0][2] - wheel_aabb[0][2]) <= 0.02
    )
    ctx.check(
        "rear feet and front wheel share a realistic resting plane",
        support_plane_ok,
        details=f"wheel={wheel_aabb}, left_foot={left_foot_aabb}, right_foot={right_foot_aabb}",
    )

    large_wheel_ok = (
        chassis_aabb is not None
        and wheel_aabb is not None
        and (wheel_aabb[1][2] - wheel_aabb[0][2]) >= 0.36
        and (wheel_aabb[1][2] - wheel_aabb[0][2]) >= 0.55 * (chassis_aabb[1][2] - chassis_aabb[0][2])
    )
    ctx.check(
        "front wheel stays large relative to the frame",
        large_wheel_ok,
        details=f"chassis={chassis_aabb}, wheel={wheel_aabb}",
    )

    with ctx.pose({wheel_spin: 1.3}):
        ctx.expect_contact(
            wheel,
            chassis,
            elem_a="left_bearing_cap",
            elem_b="left_dropout",
            name="left bearing remains seated while wheel spins",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
