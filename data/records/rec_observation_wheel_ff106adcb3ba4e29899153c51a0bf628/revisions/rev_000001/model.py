from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sin, cos

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
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_points(radius: float, *, y: float, count: int = 24) -> list[tuple[float, float, float]]:
    return [
        (radius * sin(2.0 * pi * index / count), y, radius * cos(2.0 * pi * index / count))
        for index in range(count)
    ]


def _polar_xz(radius: float, angle: float, *, y: float = 0.0) -> tuple[float, float, float]:
    return (radius * sin(angle), y, radius * cos(angle))


def _add_segment_visual(part, name: str, start, end, *, radius: float, material) -> None:
    part.visual(
        _save_mesh(
            name,
            wire_from_points(
                [start, end],
                radius=radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="miter",
            ),
        ),
        material=material,
    )


def _build_bench_seat(part, *, paint, cushion, steel) -> None:
    pivot_radius = 0.018
    pivot_length = 0.080
    pivot_center_y = 0.150
    hanger_radius = 0.015
    hanger_length = 0.300
    lower_rail_radius = 0.015

    part.visual(
        Cylinder(radius=pivot_radius, length=pivot_length),
        origin=Origin(xyz=(0.0, -pivot_center_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_pivot_sleeve",
    )
    part.visual(
        Cylinder(radius=pivot_radius, length=pivot_length),
        origin=Origin(xyz=(0.0, pivot_center_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_pivot_sleeve",
    )
    for side in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=hanger_radius, length=hanger_length),
            origin=Origin(xyz=(0.0, side * pivot_center_y, -0.150)),
            material=steel,
        )
        part.visual(
            Cylinder(radius=0.012, length=0.220),
            origin=Origin(xyz=(-0.100, side * 0.185, -0.225), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
        )
    part.visual(
        Cylinder(radius=lower_rail_radius, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, -0.305), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lower_rail",
    )
    part.visual(
        Box((0.280, 0.420, 0.025)),
        origin=Origin(xyz=(0.020, 0.0, -0.330)),
        material=cushion,
        name="seat_base",
    )
    part.visual(
        Box((0.025, 0.420, 0.180)),
        origin=Origin(xyz=(-0.115, 0.0, -0.235)),
        material=paint,
        name="seat_back",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(-0.100, 0.0, -0.145), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(0.125, 0.0, -0.318), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    part.visual(
        Box((0.015, 0.440, 0.040)),
        origin=Origin(xyz=(-0.135, 0.0, -0.340)),
        material=paint,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="park_observation_wheel")

    frame_paint = model.material("frame_paint", rgba=(0.86, 0.89, 0.92, 1.0))
    wheel_paint = model.material("wheel_paint", rgba=(0.74, 0.16, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    seat_blue = model.material("seat_blue", rgba=(0.18, 0.36, 0.67, 1.0))
    seat_cushion = model.material("seat_cushion", rgba=(0.19, 0.22, 0.28, 1.0))

    wheel_radius = 1.50
    pivot_radius = 1.38
    wheel_half_depth = 0.280
    wheel_center_z = 2.05
    seat_angles = [2.0 * pi * index / 6.0 for index in range(6)]

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((3.30, 1.60, 2.35)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
    )

    for name, points, radius in [
        (
            "left_base_skid",
            [(-1.35, -0.74, 0.09), (1.35, -0.74, 0.09)],
            0.050,
        ),
        (
            "right_base_skid",
            [(-1.35, 0.74, 0.09), (1.35, 0.74, 0.09)],
            0.050,
        ),
        (
            "front_cross_tie",
            [(1.18, -0.74, 0.09), (1.18, 0.74, 0.09)],
            0.045,
        ),
        (
            "rear_cross_tie",
            [(-1.18, -0.74, 0.09), (-1.18, 0.74, 0.09)],
            0.045,
        ),
        (
            "center_cross_tie",
            [(0.0, -0.74, 0.09), (0.0, 0.74, 0.09)],
            0.040,
        ),
        (
            "left_side_support",
            [(-1.10, -0.74, 0.09), (0.0, -0.74, wheel_center_z), (1.10, -0.74, 0.09)],
            0.050,
        ),
        (
            "right_side_support",
            [(-1.10, 0.74, 0.09), (0.0, 0.74, wheel_center_z), (1.10, 0.74, 0.09)],
            0.050,
        ),
        (
            "left_side_brace",
            [(-0.64, -0.74, 0.95), (0.0, -0.74, wheel_center_z)],
            0.038,
        ),
        (
            "right_side_brace",
            [(0.64, 0.74, 0.95), (0.0, 0.74, wheel_center_z)],
            0.038,
        ),
    ]:
        frame.visual(
            _save_mesh(
                name,
                wire_from_points(
                    points,
                    radius=radius,
                    radial_segments=18,
                    cap_ends=True,
                    corner_mode="fillet",
                    corner_radius=0.14 if len(points) > 2 else 0.0,
                    corner_segments=10,
                ),
            ),
            material=frame_paint,
            name=name,
        )

    frame.visual(
        Cylinder(radius=0.115, length=0.320),
        origin=Origin(xyz=(0.0, -0.610, wheel_center_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing",
    )
    frame.visual(
        Cylinder(radius=0.115, length=0.320),
        origin=Origin(xyz=(0.0, 0.610, wheel_center_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing",
    )
    frame.visual(
        Box((0.18, 0.26, 0.18)),
        origin=Origin(xyz=(0.0, -0.71, wheel_center_z)),
        material=dark_steel,
    )
    frame.visual(
        Box((0.18, 0.26, 0.18)),
        origin=Origin(xyz=(0.0, 0.71, wheel_center_z)),
        material=dark_steel,
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Box((3.15, 0.95, 3.15)),
        mass=180.0,
        origin=Origin(),
    )
    for side_name, y in (("left", -wheel_half_depth), ("right", wheel_half_depth)):
        wheel.visual(
            _save_mesh(
                f"wheel_ring_{side_name}",
                tube_from_spline_points(
                    _circle_points(wheel_radius, y=y, count=28),
                    radius=0.045,
                    samples_per_segment=5,
                    closed_spline=True,
                    radial_segments=18,
                    cap_ends=False,
                ),
            ),
            material=wheel_paint,
            name=f"{side_name}_ring",
        )
        for spoke_index, angle in enumerate(seat_angles):
            rim_point = _polar_xz(wheel_radius - 0.010, angle, y=y)
            wheel.visual(
                _save_mesh(
                    f"{side_name}_spoke_{spoke_index}",
                    wire_from_points(
                        [(0.0, y, 0.0), rim_point],
                        radius=0.026,
                        radial_segments=16,
                        cap_ends=True,
                        corner_mode="miter",
                    ),
                ),
                material=wheel_paint,
            )

    wheel.visual(
        Cylinder(radius=0.080, length=0.900),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.165, length=0.460),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_paint,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=0.105, length=0.660),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
    )

    for seat_index, angle in enumerate(seat_angles):
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            ring_point = _polar_xz(wheel_radius - 0.020, angle, y=side_sign * wheel_half_depth)
            bracket_point = _polar_xz(pivot_radius, angle, y=side_sign * 0.240)
            _add_segment_visual(
                wheel,
                f"hanger_drop_{seat_index}_{side_name}",
                ring_point,
                bracket_point,
                radius=0.018,
                material=dark_steel,
            )
            wheel.visual(
                Cylinder(radius=0.018, length=0.100),
                origin=Origin(
                    xyz=bracket_point,
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=dark_steel,
                name=f"{side_name}_hanger_{seat_index}",
            )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=1.2),
    )

    for seat_index, angle in enumerate(seat_angles):
        seat = model.part(f"seat_{seat_index}")
        seat.inertial = Inertial.from_geometry(
            Box((0.320, 0.460, 0.420)),
            mass=14.0,
            origin=Origin(xyz=(0.0, 0.0, -0.210)),
        )
        _build_bench_seat(seat, paint=seat_blue, cushion=seat_cushion, steel=dark_steel)
        model.articulation(
            f"wheel_to_seat_{seat_index}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=seat,
            origin=Origin(xyz=_polar_xz(pivot_radius, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=45.0,
                velocity=2.4,
                lower=-1.35,
                upper=1.35,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("frame_to_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(wheel, frame, name="wheel axle seated in frame bearings")
    ctx.check(
        "wheel rotates on horizontal axle",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected continuous y-axis wheel spin, got type={wheel_joint.articulation_type} axis={wheel_joint.axis}",
    )

    seat_joints = []
    for seat_index in range(6):
        seat = object_model.get_part(f"seat_{seat_index}")
        seat_joint = object_model.get_articulation(f"wheel_to_seat_{seat_index}")
        seat_joints.append(seat_joint)
        ctx.expect_contact(seat, wheel, name=f"seat_{seat_index} clipped to wheel hanger pivots")
        limits = seat_joint.motion_limits
        ctx.check(
            f"seat_{seat_index} hanger axis and limits",
            seat_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(seat_joint.axis) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -1.2
            and limits.upper >= 1.2,
            details=(
                "Seat hanger should be a y-axis revolute with generous swing. "
                f"type={seat_joint.articulation_type} axis={seat_joint.axis} limits={limits}"
            ),
        )

    counter_pose = {wheel_joint: pi / 4.0}
    for seat_joint in seat_joints:
        counter_pose[seat_joint] = -pi / 4.0

    with ctx.pose(counter_pose):
        for seat_index in range(6):
            ctx.expect_contact(
                object_model.get_part(f"seat_{seat_index}"),
                wheel,
                name=f"seat_{seat_index} stays attached while wheel turns",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
