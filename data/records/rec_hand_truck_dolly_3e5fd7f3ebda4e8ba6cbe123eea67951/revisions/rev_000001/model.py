from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(xy, dz)
    return (0.0, pitch, yaw)


def _merge_segment(geom, a, b, radius: float) -> None:
    geom.merge(
        CylinderGeometry(radius=radius, height=_distance(a, b))
        .rotate_y(_rpy_for_cylinder(a, b)[1])
        .rotate_z(_rpy_for_cylinder(a, b)[2])
        .translate(*_midpoint(a, b))
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_frame_mesh():
    rail_radius = 0.018
    brace_radius = 0.014

    left_rail = [
        (0.165, 0.010, 0.053),
        (0.180, -0.020, 0.240),
        (0.210, 0.105, 0.570),
        (0.190, 0.085, 0.930),
        (0.170, -0.020, 1.180),
    ]
    right_rail = _mirror_x(left_rail)

    frame = tube_from_spline_points(
        left_rail,
        radius=rail_radius,
        samples_per_segment=16,
        radial_segments=18,
    )
    frame.merge(
        tube_from_spline_points(
            right_rail,
            radius=rail_radius,
            samples_per_segment=16,
            radial_segments=18,
        )
    )

    frame.merge(
        tube_from_spline_points(
            [
                left_rail[-1],
                (0.095, -0.040, 1.215),
                (0.000, -0.050, 1.235),
                (-0.095, -0.040, 1.215),
                right_rail[-1],
            ],
            radius=rail_radius,
            samples_per_segment=14,
            radial_segments=18,
        )
    )

    frame.merge(
        tube_from_spline_points(
            [
                left_rail[1],
                (0.095, 0.015, 0.290),
                (0.000, 0.025, 0.315),
                (-0.095, 0.015, 0.290),
                right_rail[1],
            ],
            radius=brace_radius,
            samples_per_segment=12,
            radial_segments=16,
        )
    )
    frame.merge(
        tube_from_spline_points(
            [
                left_rail[2],
                (0.110, 0.140, 0.595),
                (0.000, 0.155, 0.620),
                (-0.110, 0.140, 0.595),
                right_rail[2],
            ],
            radius=brace_radius,
            samples_per_segment=12,
            radial_segments=16,
        )
    )
    frame.merge(
        tube_from_spline_points(
            [
                left_rail[3],
                (0.095, 0.110, 0.955),
                (0.000, 0.120, 0.975),
                (-0.095, 0.110, 0.955),
                right_rail[3],
            ],
            radius=brace_radius,
            samples_per_segment=12,
            radial_segments=16,
        )
    )

    _merge_segment(frame, (-0.195, -0.015, 0.190), (0.195, -0.015, 0.190), 0.020)
    _merge_segment(frame, (0.195, -0.015, 0.190), (0.245, -0.015, 0.190), 0.010)
    _merge_segment(frame, (-0.195, -0.015, 0.190), (-0.245, -0.015, 0.190), 0.010)

    _merge_segment(frame, left_rail[1], (0.190, -0.086, 0.180), 0.012)
    _merge_segment(frame, right_rail[1], (-0.190, -0.086, 0.180), 0.012)
    _merge_segment(frame, (-0.150, -0.074, 0.190), (0.150, -0.074, 0.190), 0.012)

    frame.merge(BoxGeometry((0.014, 0.026, 0.050)).translate(0.192, -0.095, 0.180))
    frame.merge(BoxGeometry((0.014, 0.026, 0.050)).translate(-0.192, -0.095, 0.180))

    scoop_profile = [
        (0.190, -0.004),
        (0.190, 0.074),
        (0.160, 0.118),
        (0.105, 0.153),
        (0.000, 0.176),
        (-0.105, 0.153),
        (-0.160, 0.118),
        (-0.190, 0.074),
        (-0.190, -0.004),
    ]
    frame.merge(ExtrudeGeometry.from_z0(scoop_profile, 0.010).translate(0.000, 0.000, 0.044))
    frame.merge(
        tube_from_spline_points(
            [
                (0.150, 0.092, 0.051),
                (0.080, 0.145, 0.061),
                (0.000, 0.176, 0.067),
                (-0.080, 0.145, 0.061),
                (-0.150, 0.092, 0.051),
            ],
            radius=0.009,
            samples_per_segment=14,
            radial_segments=16,
        )
    )
    frame.merge(
        tube_from_spline_points(
            [
                (0.150, 0.022, 0.054),
                (0.080, 0.035, 0.064),
                (0.000, 0.044, 0.070),
                (-0.080, 0.035, 0.064),
                (-0.150, 0.022, 0.054),
            ],
            radius=0.010,
            samples_per_segment=10,
            radial_segments=16,
        )
    )

    return frame


def _build_leg_mesh():
    leg = wire_from_points(
        [
            (0.000, -0.015, 0.004),
            (0.000, -0.105, 0.072),
            (0.000, -0.195, 0.148),
            (0.000, -0.285, 0.220),
        ],
        radius=0.010,
        radial_segments=14,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.028,
        corner_segments=8,
        up_hint=(1.0, 0.0, 0.0),
    )
    leg.merge(CylinderGeometry(radius=0.010, height=0.022).rotate_y(pi / 2.0))
    leg.merge(CylinderGeometry(radius=0.010, height=0.060).rotate_y(pi / 2.0).translate(0.0, -0.285, 0.220))
    leg.merge(BoxGeometry((0.020, 0.030, 0.018)).translate(0.0, -0.255, 0.192))
    return leg


def _add_wheel_visuals(part, mesh_name: str, *, tire_radius: float, tire_width: float, tire, rim, hub) -> None:
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.54, -half_width * 0.98),
        (tire_radius * 0.78, -half_width),
        (tire_radius * 0.93, -half_width * 0.82),
        (tire_radius * 0.985, -half_width * 0.35),
        (tire_radius, 0.0),
        (tire_radius * 0.985, half_width * 0.35),
        (tire_radius * 0.93, half_width * 0.82),
        (tire_radius * 0.78, half_width),
        (tire_radius * 0.54, half_width * 0.98),
        (tire_radius * 0.46, half_width * 0.42),
        (tire_radius * 0.44, 0.0),
        (tire_radius * 0.46, -half_width * 0.42),
        (tire_radius * 0.54, -half_width * 0.98),
    ]

    part.visual(
        _save_mesh(mesh_name, LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0)),
        material=tire,
        name="tire",
    )
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(Cylinder(radius=tire_radius * 0.67, length=tire_width * 0.78), origin=spin_origin, material=rim, name="rim")
    part.visual(Cylinder(radius=tire_radius * 0.28, length=tire_width), origin=spin_origin, material=hub, name="hub")
    part.visual(Cylinder(radius=tire_radius * 0.11, length=tire_width * 1.10), origin=spin_origin, material=rim, name="cap")

    for angle in (0.0, pi / 4.0, -pi / 4.0):
        part.visual(
            Box((tire_width * 0.36, tire_radius * 0.82, tire_radius * 0.11)),
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=rim,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drum_hand_truck")

    frame_blue = model.material("frame_blue", rgba=(0.20, 0.36, 0.61, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.32, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    frame = model.part("frame")
    frame.visual(_save_mesh("frame_mesh", _build_frame_mesh()), material=frame_blue, name="frame_body")
    frame.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(0.170, -0.020, 1.180), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.120),
        origin=Origin(xyz=(-0.170, -0.020, 1.180), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.56, 0.46, 1.28)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.03, 0.64)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(
        left_wheel,
        "left_transport_wheel",
        tire_radius=0.170,
        tire_width=0.055,
        tire=rubber,
        rim=steel,
        hub=dark_steel,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.055),
        mass=4.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(
        right_wheel,
        "right_transport_wheel",
        tire_radius=0.170,
        tire_width=0.055,
        tire=rubber,
        rim=steel,
        hub=dark_steel,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.055),
        mass=4.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    left_leg = model.part("left_stabilizer_leg")
    left_leg.visual(_save_mesh("left_stabilizer_leg_mesh", _build_leg_mesh()), material=dark_steel, name="leg_body")
    left_leg.inertial = Inertial.from_geometry(
        Box((0.030, 0.320, 0.250)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.170, 0.125)),
    )

    right_leg = model.part("right_stabilizer_leg")
    right_leg.visual(_save_mesh("right_stabilizer_leg_mesh", _build_leg_mesh()), material=dark_steel, name="leg_body")
    right_leg.inertial = Inertial.from_geometry(
        Box((0.030, 0.320, 0.250)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.170, 0.125)),
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(0.2725, -0.015, 0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-0.2725, -0.015, 0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=18.0),
    )
    model.articulation(
        "left_leg_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_leg,
        origin=Origin(xyz=(0.210, -0.095, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=0.0, upper=1.18),
    )
    model.articulation(
        "right_leg_fold",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_leg,
        origin=Origin(xyz=(-0.210, -0.095, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.6, lower=0.0, upper=1.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_leg = object_model.get_part("left_stabilizer_leg")
    right_leg = object_model.get_part("right_stabilizer_leg")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    left_leg_fold = object_model.get_articulation("left_leg_fold")
    right_leg_fold = object_model.get_articulation("right_leg_fold")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        left_leg,
        frame,
        reason="Left stabilizer uses a clipped hinge knuckle that shares the rear pivot volume with the frame bracket.",
    )
    ctx.allow_overlap(
        right_leg,
        frame,
        reason="Right stabilizer uses a clipped hinge knuckle that shares the rear pivot volume with the frame bracket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_wheel, frame, name="left wheel contacts axle")
    ctx.expect_contact(right_wheel, frame, name="right wheel contacts axle")
    ctx.expect_contact(left_leg, frame, name="left stabilizer remains clipped to hinge")
    ctx.expect_contact(right_leg, frame, name="right stabilizer remains clipped to hinge")
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="x", min_dist=0.50, max_dist=0.60)

    ctx.check(
        "wheel spin axes align with axle",
        left_wheel_spin.axis == (1.0, 0.0, 0.0) and right_wheel_spin.axis == (1.0, 0.0, 0.0),
        details=f"left={left_wheel_spin.axis}, right={right_wheel_spin.axis}",
    )
    ctx.check(
        "stabilizer hinges rotate about side pivots",
        left_leg_fold.axis == (1.0, 0.0, 0.0) and right_leg_fold.axis == (1.0, 0.0, 0.0),
        details=f"left={left_leg_fold.axis}, right={right_leg_fold.axis}",
    )

    folded_left = ctx.part_world_aabb(left_leg)
    folded_right = ctx.part_world_aabb(right_leg)
    assert folded_left is not None
    assert folded_right is not None
    with ctx.pose({left_leg_fold: 1.10, right_leg_fold: 1.10}):
        deployed_left = ctx.part_world_aabb(left_leg)
        deployed_right = ctx.part_world_aabb(right_leg)
        assert deployed_left is not None
        assert deployed_right is not None
        ctx.expect_contact(left_leg, frame, name="left stabilizer stays hinged when deployed")
        ctx.expect_contact(right_leg, frame, name="right stabilizer stays hinged when deployed")
        ctx.check(
            "left stabilizer swings down and back",
            deployed_left[0][1] < -0.40
            and deployed_left[0][2] < folded_left[0][2] - 0.20
            and deployed_left[1][2] < folded_left[1][2] - 0.10,
            details=f"folded={folded_left}, deployed={deployed_left}",
        )
        ctx.check(
            "right stabilizer swings down and back",
            deployed_right[0][1] < -0.40
            and deployed_right[0][2] < folded_right[0][2] - 0.20
            and deployed_right[1][2] < folded_right[1][2] - 0.10,
            details=f"folded={folded_right}, deployed={deployed_right}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
