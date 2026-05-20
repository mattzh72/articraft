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


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


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
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _circle_points(
    radius: float,
    *,
    y: float = 0.0,
    count: int = 20,
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / count),
            y,
            radius * math.sin(2.0 * math.pi * i / count),
        )
        for i in range(count)
    ]


def _add_rear_wheel_visuals(
    part,
    mesh_prefix: str,
    *,
    side_sign: float,
    rim_material,
    hub_material,
    tire_material,
) -> None:
    tire_radius = 0.315
    tire_width = 0.028
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.80, -half_width * 0.95),
        (tire_radius * 0.90, -half_width),
        (tire_radius * 0.975, -half_width * 0.70),
        (tire_radius, -half_width * 0.25),
        (tire_radius, half_width * 0.25),
        (tire_radius * 0.975, half_width * 0.70),
        (tire_radius * 0.90, half_width),
        (tire_radius * 0.80, half_width * 0.95),
        (tire_radius * 0.74, half_width * 0.40),
        (tire_radius * 0.73, 0.0),
        (tire_radius * 0.74, -half_width * 0.40),
        (tire_radius * 0.80, -half_width * 0.95),
    ]
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_tire",
            LatheGeometry(tire_profile, segments=56).rotate_x(math.pi / 2.0),
        ),
        material=tire_material,
        name="tire",
    )

    part.visual(
        _save_mesh(
            f"{mesh_prefix}_rim_ring",
            tube_from_spline_points(
                _circle_points(tire_radius * 0.77, count=18),
                radius=0.010,
                samples_per_segment=6,
                radial_segments=14,
                closed_spline=True,
                cap_ends=False,
            ),
        ),
        material=rim_material,
        name="rim_ring",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.044),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.068),
        origin=Origin(
            xyz=(0.0, -0.032 * side_sign, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hub_material,
        name="inner_hub",
    )
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_handrim",
            tube_from_spline_points(
                _circle_points(tire_radius * 0.92, y=0.020 * side_sign, count=20),
                radius=0.0055,
                samples_per_segment=6,
                radial_segments=12,
                closed_spline=True,
                cap_ends=False,
            ),
        ),
        material=hub_material,
        name="handrim",
    )

    spoke_hub_radius = 0.033
    spoke_rim_radius = tire_radius * 0.74
    for spoke_index in range(8):
        angle = 2.0 * math.pi * spoke_index / 8.0
        a = (
            spoke_hub_radius * math.cos(angle),
            0.0,
            spoke_hub_radius * math.sin(angle),
        )
        b = (
            spoke_rim_radius * math.cos(angle),
            0.0,
            spoke_rim_radius * math.sin(angle),
        )
        _add_member(
            part,
            a,
            b,
            0.003,
            rim_material,
            name=f"spoke_{spoke_index}",
        )
    for standoff_index in range(4):
        angle = 2.0 * math.pi * standoff_index / 4.0 + math.pi / 8.0
        a = (
            tire_radius * 0.79 * math.cos(angle),
            0.0,
            tire_radius * 0.79 * math.sin(angle),
        )
        b = (
            tire_radius * 0.90 * math.cos(angle),
            0.020 * side_sign,
            tire_radius * 0.90 * math.sin(angle),
        )
        _add_member(
            part,
            a,
            b,
            0.003,
            hub_material,
            name=f"handrim_standoff_{standoff_index}",
        )


def _add_caster_wheel_visuals(
    part,
    mesh_prefix: str,
    *,
    rim_material,
    tire_material,
) -> None:
    tire_radius = 0.075
    tire_width = 0.022
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.64, -half_width * 0.95),
        (tire_radius * 0.84, -half_width),
        (tire_radius * 0.96, -half_width * 0.50),
        (tire_radius, -half_width * 0.18),
        (tire_radius, half_width * 0.18),
        (tire_radius * 0.96, half_width * 0.50),
        (tire_radius * 0.84, half_width),
        (tire_radius * 0.64, half_width * 0.95),
        (tire_radius * 0.50, half_width * 0.35),
        (tire_radius * 0.48, 0.0),
        (tire_radius * 0.50, -half_width * 0.35),
        (tire_radius * 0.64, -half_width * 0.95),
    ]
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_tire",
            LatheGeometry(tire_profile, segments=40).rotate_x(math.pi / 2.0),
        ),
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.049, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="wheel_core",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name="hub",
    )


def _add_caster_fork_visuals(part, material) -> None:
    part.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=material,
        name="top_collar",
    )
    part.visual(
        Cylinder(radius=0.011, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=material,
        name="stem",
    )
    part.visual(
        Box((0.036, 0.032, 0.012)),
        origin=Origin(xyz=(-0.006, 0.0, -0.024)),
        material=material,
        name="fork_crown",
    )
    _add_member(
        part,
        (-0.004, 0.018, -0.030),
        (-0.090, 0.018, -0.115),
        0.005,
        material,
        name="outer_leg",
    )
    _add_member(
        part,
        (-0.004, -0.018, -0.030),
        (-0.090, -0.018, -0.115),
        0.005,
        material,
        name="inner_leg",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(-0.090, 0.019, -0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="outer_axle_boss",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(-0.090, -0.019, -0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="inner_axle_boss",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    frame_paint = model.material("frame_paint", rgba=(0.13, 0.15, 0.17, 1.0))
    upholstery = model.material("upholstery", rgba=(0.08, 0.08, 0.09, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    footplate = model.material("footplate", rgba=(0.58, 0.60, 0.63, 1.0))

    chair_frame = model.part("chair_frame")
    chair_frame.inertial = Inertial.from_geometry(
        Box((0.70, 0.70, 1.00)),
        mass=12.0,
        origin=Origin(xyz=(0.18, 0.0, 0.50)),
    )

    left_main_side = [
        (-0.07, 0.22, 0.89),
        (-0.05, 0.22, 0.78),
        (0.02, 0.22, 0.54),
        (0.00, 0.22, 0.36),
        (0.24, 0.22, 0.35),
        (0.40, 0.22, 0.28),
        (0.43, 0.20, 0.20),
    ]
    chair_frame.visual(
        _save_mesh(
            "wheelchair_left_main_side",
            tube_from_spline_points(
                left_main_side,
                radius=0.014,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="left_main_side",
    )
    chair_frame.visual(
        _save_mesh(
            "wheelchair_right_main_side",
            tube_from_spline_points(
                _mirror_y(left_main_side),
                radius=0.014,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="right_main_side",
    )

    left_seat_rail = [
        (0.02, 0.22, 0.54),
        (0.19, 0.22, 0.54),
        (0.30, 0.21, 0.50),
    ]
    chair_frame.visual(
        _save_mesh(
            "wheelchair_left_seat_rail",
            tube_from_spline_points(
                left_seat_rail,
                radius=0.012,
                samples_per_segment=16,
                radial_segments=16,
            ),
        ),
        material=frame_paint,
        name="left_seat_rail",
    )
    chair_frame.visual(
        _save_mesh(
            "wheelchair_right_seat_rail",
            tube_from_spline_points(
                _mirror_y(left_seat_rail),
                radius=0.012,
                samples_per_segment=16,
                radial_segments=16,
            ),
        ),
        material=frame_paint,
        name="right_seat_rail",
    )

    left_front_hanger = [
        (0.30, 0.21, 0.50),
        (0.33, 0.18, 0.28),
        (0.37, 0.12, 0.16),
    ]
    chair_frame.visual(
        _save_mesh(
            "wheelchair_left_front_hanger",
            tube_from_spline_points(
                left_front_hanger,
                radius=0.011,
                samples_per_segment=16,
                radial_segments=16,
            ),
        ),
        material=frame_paint,
        name="left_front_hanger",
    )
    chair_frame.visual(
        _save_mesh(
            "wheelchair_right_front_hanger",
            tube_from_spline_points(
                _mirror_y(left_front_hanger),
                radius=0.011,
                samples_per_segment=16,
                radial_segments=16,
            ),
        ),
        material=frame_paint,
        name="right_front_hanger",
    )

    _add_member(
        chair_frame,
        (0.04, -0.205, 0.49),
        (0.04, 0.205, 0.49),
        0.018,
        frame_paint,
        name="rear_seat_crossbar",
    )
    _add_member(
        chair_frame,
        (0.25, -0.205, 0.49),
        (0.25, 0.205, 0.49),
        0.018,
        frame_paint,
        name="front_seat_crossbar",
    )
    _add_member(
        chair_frame,
        (0.24, -0.220, 0.35),
        (0.24, 0.220, 0.35),
        0.016,
        frame_paint,
        name="lower_crossbrace",
    )
    _add_member(
        chair_frame,
        (-0.06, -0.205, 0.84),
        (-0.06, 0.205, 0.84),
        0.012,
        frame_paint,
        name="push_handle_crossbar",
    )
    _add_member(
        chair_frame,
        (0.37, -0.13, 0.15),
        (0.37, 0.13, 0.15),
        0.016,
        frame_paint,
        name="footrest_bridge",
    )
    _add_member(
        chair_frame,
        (0.04, 0.22, 0.36),
        (0.21, 0.22, 0.52),
        0.010,
        frame_paint,
        name="left_diagonal_brace",
    )
    _add_member(
        chair_frame,
        (0.04, -0.22, 0.36),
        (0.21, -0.22, 0.52),
        0.010,
        frame_paint,
        name="right_diagonal_brace",
    )

    chair_frame.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.43, 0.20, 0.2175)),
        material=frame_paint,
        name="left_caster_socket",
    )
    chair_frame.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.43, -0.20, 0.2175)),
        material=frame_paint,
        name="right_caster_socket",
    )
    chair_frame.visual(
        Cylinder(radius=0.019, length=0.060),
        origin=Origin(xyz=(0.03, 0.25, 0.315), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="left_axle_sleeve",
    )
    chair_frame.visual(
        Cylinder(radius=0.019, length=0.060),
        origin=Origin(xyz=(0.03, -0.25, 0.315), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="right_axle_sleeve",
    )
    chair_frame.visual(
        Box((0.40, 0.41, 0.012)),
        origin=Origin(xyz=(0.15, 0.0, 0.508)),
        material=upholstery,
        name="seat_sling",
    )
    chair_frame.visual(
        Box((0.045, 0.44, 0.26)),
        origin=Origin(xyz=(-0.035, 0.0, 0.70)),
        material=upholstery,
        name="backrest_panel",
    )
    chair_frame.visual(
        Box((0.11, 0.10, 0.012)),
        origin=Origin(xyz=(0.385, 0.085, 0.095)),
        material=footplate,
        name="left_footplate",
    )
    chair_frame.visual(
        Box((0.11, 0.10, 0.012)),
        origin=Origin(xyz=(0.385, -0.085, 0.095)),
        material=footplate,
        name="right_footplate",
    )
    _add_member(
        chair_frame,
        (0.37, 0.12, 0.16),
        (0.34, 0.11, 0.10),
        0.007,
        frame_paint,
        name="left_footplate_outer_support",
    )
    _add_member(
        chair_frame,
        (0.37, 0.12, 0.16),
        (0.41, 0.06, 0.10),
        0.007,
        frame_paint,
        name="left_footplate_inner_support",
    )
    _add_member(
        chair_frame,
        (0.37, -0.12, 0.16),
        (0.34, -0.11, 0.10),
        0.007,
        frame_paint,
        name="right_footplate_outer_support",
    )
    _add_member(
        chair_frame,
        (0.37, -0.12, 0.16),
        (0.41, -0.06, 0.10),
        0.007,
        frame_paint,
        name="right_footplate_inner_support",
    )

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.315, length=0.028),
        mass=2.8,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        left_rear_wheel,
        "wheelchair_left_rear_wheel",
        side_sign=1.0,
        rim_material=aluminum,
        hub_material=frame_paint,
        tire_material=rubber,
    )

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.315, length=0.028),
        mass=2.8,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_rear_wheel_visuals(
        right_rear_wheel,
        "wheelchair_right_rear_wheel",
        side_sign=-1.0,
        rim_material=aluminum,
        hub_material=frame_paint,
        tire_material=rubber,
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.16)),
        mass=0.7,
        origin=Origin(xyz=(-0.022, 0.0, -0.080)),
    )
    _add_caster_fork_visuals(left_caster_fork, frame_paint)

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.16)),
        mass=0.7,
        origin=Origin(xyz=(-0.022, 0.0, -0.080)),
    )
    _add_caster_fork_visuals(right_caster_fork, frame_paint)

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.022),
        mass=0.45,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        left_caster_wheel,
        "wheelchair_left_caster_wheel",
        rim_material=aluminum,
        tire_material=rubber,
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.022),
        mass=0.45,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_caster_wheel_visuals(
        right_caster_wheel,
        "wheelchair_right_caster_wheel",
        rim_material=aluminum,
        tire_material=rubber,
    )

    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chair_frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(0.03, 0.346, 0.315)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chair_frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(0.03, -0.346, 0.315)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=chair_frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.43, 0.20, 0.19)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=chair_frame,
        child=right_caster_fork,
        origin=Origin(xyz=(0.43, -0.20, 0.19)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(-0.090, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=24.0),
    )
    model.articulation(
        "right_caster_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.090, 0.0, -0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("chair_frame")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")

    left_rear_spin = object_model.get_articulation("left_rear_wheel_spin")
    right_rear_spin = object_model.get_articulation("right_rear_wheel_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_spin = object_model.get_articulation("left_caster_wheel_spin")
    right_caster_spin = object_model.get_articulation("right_caster_wheel_spin")

    for joint_name, axis in (
        ("left_rear_wheel_spin", (0.0, 1.0, 0.0)),
        ("right_rear_wheel_spin", (0.0, 1.0, 0.0)),
        ("left_caster_swivel", (0.0, 0.0, 1.0)),
        ("right_caster_swivel", (0.0, 0.0, 1.0)),
        ("left_caster_wheel_spin", (0.0, 1.0, 0.0)),
        ("right_caster_wheel_spin", (0.0, 1.0, 0.0)),
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous on the intended axis",
            joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == axis,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_contact(
        left_rear_wheel,
        frame,
        elem_a="inner_hub",
        elem_b="left_axle_sleeve",
        name="left rear wheel is carried by the axle sleeve",
    )
    ctx.expect_contact(
        right_rear_wheel,
        frame,
        elem_a="inner_hub",
        elem_b="right_axle_sleeve",
        name="right rear wheel is carried by the axle sleeve",
    )
    ctx.expect_contact(
        left_caster_fork,
        frame,
        elem_a="top_collar",
        elem_b="left_caster_socket",
        name="left caster swivel collar seats against the socket",
    )
    ctx.expect_contact(
        right_caster_fork,
        frame,
        elem_a="top_collar",
        elem_b="right_caster_socket",
        name="right caster swivel collar seats against the socket",
    )
    ctx.expect_gap(
        left_caster_fork,
        left_caster_wheel,
        axis="y",
        positive_elem="outer_axle_boss",
        negative_elem="hub",
        min_gap=0.001,
        max_gap=0.004,
        name="left caster wheel hub clears the outer fork boss",
    )
    ctx.expect_gap(
        left_caster_wheel,
        left_caster_fork,
        axis="y",
        positive_elem="hub",
        negative_elem="inner_axle_boss",
        min_gap=0.001,
        max_gap=0.004,
        name="left caster wheel hub clears the inner fork boss",
    )
    ctx.expect_gap(
        right_caster_fork,
        right_caster_wheel,
        axis="y",
        positive_elem="outer_axle_boss",
        negative_elem="hub",
        min_gap=0.001,
        max_gap=0.004,
        name="right caster wheel hub clears the outer fork boss",
    )
    ctx.expect_gap(
        right_caster_wheel,
        right_caster_fork,
        axis="y",
        positive_elem="hub",
        negative_elem="inner_axle_boss",
        min_gap=0.001,
        max_gap=0.004,
        name="right caster wheel hub clears the inner fork boss",
    )

    left_rear_pos = ctx.part_world_position(left_rear_wheel)
    right_rear_pos = ctx.part_world_position(right_rear_wheel)
    left_fork_pos = ctx.part_world_position(left_caster_fork)
    compact_wheelbase = (
        left_rear_pos is not None
        and left_fork_pos is not None
        and 0.30 <= left_fork_pos[0] - left_rear_pos[0] <= 0.45
    )
    ctx.check(
        "wheelbase stays compact",
        compact_wheelbase,
        details=f"rear={left_rear_pos}, front={left_fork_pos}",
    )
    rear_track_ok = (
        left_rear_pos is not None
        and right_rear_pos is not None
        and abs(left_rear_pos[0] - right_rear_pos[0]) <= 1e-6
        and abs(left_rear_pos[2] - right_rear_pos[2]) <= 1e-6
        and 0.60 <= left_rear_pos[1] - right_rear_pos[1] <= 0.72
    )
    ctx.check(
        "rear wheels stay symmetric outside the seat frame",
        rear_track_ok,
        details=f"left={left_rear_pos}, right={right_rear_pos}",
    )

    left_caster_rest = ctx.part_world_position(left_caster_wheel)
    with ctx.pose({left_caster_swivel: 1.2, right_caster_swivel: -1.2}):
        left_caster_turned = ctx.part_world_position(left_caster_wheel)
        right_caster_turned = ctx.part_world_position(right_caster_wheel)

    swivel_motion_ok = (
        left_caster_rest is not None
        and left_caster_turned is not None
        and abs(left_caster_rest[0] - left_caster_turned[0]) > 0.04
        and abs(left_caster_rest[1] - left_caster_turned[1]) > 0.04
    )
    ctx.check(
        "left caster swivel swings the trailing wheel around the stem",
        swivel_motion_ok,
        details=f"rest={left_caster_rest}, turned={left_caster_turned}",
    )
    caster_ground_ok = (
        left_caster_rest is not None
        and left_caster_turned is not None
        and right_caster_turned is not None
        and abs(left_caster_rest[2] - 0.075) <= 1e-6
        and abs(left_caster_turned[2] - 0.075) <= 1e-6
        and abs(right_caster_turned[2] - 0.075) <= 1e-6
    )
    ctx.check(
        "caster wheels keep the same ground-contact height while swiveling",
        caster_ground_ok,
        details=f"rest={left_caster_rest}, left_turned={left_caster_turned}, right_turned={right_caster_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
