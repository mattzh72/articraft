from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = sqrt(dx * dx + dy * dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    kwargs = {}
    if name is not None:
        kwargs["name"] = name
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        **kwargs,
    )


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _bushing_mesh(name: str, *, inner_radius: float, outer_radius: float, length: float):
    half = length * 0.5
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=48,
        ),
    )


def _wheel_mesh(
    name: str,
    *,
    radius: float,
    width: float,
    bore_radius: float,
):
    half = width * 0.5
    outer_profile = [
        (radius * 0.48, -half),
        (radius * 0.76, -half * 0.96),
        (radius * 0.95, -half * 0.62),
        (radius, -half * 0.18),
        (radius, half * 0.18),
        (radius * 0.95, half * 0.62),
        (radius * 0.76, half * 0.96),
        (radius * 0.48, half),
    ]
    inner_profile = [
        (bore_radius, -half * 0.96),
        (radius * 0.28, -half * 0.92),
        (radius * 0.40, -half * 0.30),
        (radius * 0.43, 0.0),
        (radius * 0.40, half * 0.30),
        (radius * 0.28, half * 0.92),
        (bore_radius, half * 0.96),
    ]
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
        ).rotate_x(pi / 2.0),
    )


def _add_plate_bolts(
    part,
    *,
    x: float,
    y: float,
    z_values: tuple[float, float],
    material,
    along: str = "y",
) -> None:
    rpy = _axis_rpy(along)
    for z in z_values:
        part.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(xyz=(x, y, z), rpy=rpy),
            material=material,
        )
        head_offset = 0.009 if along == "y" else 0.0
        part.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(xyz=(x, y + head_offset, z), rpy=rpy),
            material=material,
        )


def _build_side_frame(
    part,
    *,
    mesh_prefix: str,
    side_sign: float,
    frame_paint,
    guard_yellow,
    lock_red,
    grip_black,
    fastener_steel,
    rubber_black,
) -> None:
    main_loop = tube_from_spline_points(
        [
            (-0.028, 0.0, 0.18),
            (-0.08, 0.0, 0.28),
            (-0.28, 0.0, 0.35),
            (-0.49, 0.0, 0.36),
            (-0.58, 0.0, 0.38),
            (-0.60, 0.0, 0.12),
            (-0.59, 0.0, -0.23),
            (-0.57, 0.0, -0.47),
            (-0.42, 0.0, -0.24),
            (-0.24, 0.0, -0.18),
            (-0.07, 0.0, -0.12),
            (-0.028, 0.0, -0.10),
        ],
        radius=0.018,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    part.visual(
        _save_mesh(f"{mesh_prefix}_main_loop", main_loop),
        material=frame_paint,
        name=f"{mesh_prefix}_main_loop",
    )

    diagonal_brace = tube_from_spline_points(
        [
            (-0.54, 0.0, -0.22),
            (-0.38, 0.0, -0.08),
            (-0.21, 0.0, 0.00),
            (-0.10, 0.0, 0.08),
        ],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=14,
        cap_ends=True,
    )
    part.visual(
        _save_mesh(f"{mesh_prefix}_diagonal_brace", diagonal_brace),
        material=frame_paint,
        name=f"{mesh_prefix}_diagonal_brace",
    )

    handle_stalk = tube_from_spline_points(
        [
            (-0.49, 0.0, 0.36),
            (-0.55, side_sign * 0.016, 0.41),
            (-0.60, side_sign * 0.032, 0.43),
        ],
        radius=0.0115,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    part.visual(
        _save_mesh(f"{mesh_prefix}_handle_stalk", handle_stalk),
        material=frame_paint,
        name=f"{mesh_prefix}_handle_stalk",
    )

    guard_loop = tube_from_spline_points(
        [
            (-0.58, side_sign * 0.018, 0.445),
            (-0.60, side_sign * 0.052, 0.470),
            (-0.635, side_sign * 0.055, 0.430),
        ],
        radius=0.0055,
        samples_per_segment=12,
        radial_segments=12,
        cap_ends=True,
    )
    part.visual(
        _save_mesh(f"{mesh_prefix}_lever_guard", guard_loop),
        material=guard_yellow,
        name=f"{mesh_prefix}_lever_guard",
    )

    hinge_sleeve = _bushing_mesh(
        f"{mesh_prefix}_hinge_sleeve",
        inner_radius=0.0094,
        outer_radius=0.016,
        length=0.072,
    )
    part.visual(
        hinge_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=frame_paint,
        name=f"{mesh_prefix}_upper_hinge_sleeve",
    )
    part.visual(
        hinge_sleeve,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=frame_paint,
        name=f"{mesh_prefix}_lower_hinge_sleeve",
    )

    part.visual(
        Box((0.028, 0.010, 0.120)),
        origin=Origin(xyz=(-0.014, 0.0, 0.15)),
        material=frame_paint,
        name=f"{mesh_prefix}_upper_hinge_gusset",
    )
    part.visual(
        Box((0.028, 0.010, 0.100)),
        origin=Origin(xyz=(-0.014, 0.0, -0.11)),
        material=frame_paint,
        name=f"{mesh_prefix}_lower_hinge_gusset",
    )

    part.visual(
        Cylinder(radius=0.022, length=0.160),
        origin=Origin(xyz=(-0.46, 0.0, 0.36), rpy=_axis_rpy("x")),
        material=grip_black,
        name=f"{mesh_prefix}_grip_sleeve",
    )
    part.visual(
        Box((0.050, 0.022, 0.016)),
        origin=Origin(xyz=(-0.612, side_sign * 0.034, 0.440)),
        material=lock_red,
        name=f"{mesh_prefix}_brake_ready_perch",
    )
    part.visual(
        Box((0.120, 0.006, 0.220)),
        origin=Origin(xyz=(-0.315, side_sign * 0.011, -0.165)),
        material=guard_yellow,
        name=f"{mesh_prefix}_side_guard",
    )
    part.visual(
        Box((0.085, 0.055, 0.050)),
        origin=Origin(xyz=(-0.570, 0.0, -0.495)),
        material=rubber_black,
        name=f"{mesh_prefix}_rear_foot",
    )
    _add_plate_bolts(
        part,
        x=-0.315,
        y=side_sign * 0.013,
        z_values=(-0.245, -0.085),
        material=fastener_steel,
        along="y",
    )
    _add_plate_bolts(
        part,
        x=-0.012,
        y=side_sign * 0.006,
        z_values=(0.15, -0.11),
        material=fastener_steel,
        along="y",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_walker")

    frame_paint = model.material("frame_paint", rgba=(0.27, 0.29, 0.31, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.88, 0.76, 0.15, 1.0))
    lock_red = model.material("lock_red", rgba=(0.74, 0.17, 0.13, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.06, 0.06, 0.06, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.32, 0.70, 0.90)),
        mass=8.8,
        origin=Origin(xyz=(0.19, 0.0, 0.46)),
    )

    front_frame.visual(
        Cylinder(radius=0.018, length=0.720),
        origin=Origin(xyz=(0.195, 0.295, 0.520)),
        material=frame_paint,
        name="left_front_leg",
    )
    front_frame.visual(
        Cylinder(radius=0.018, length=0.720),
        origin=Origin(xyz=(0.195, -0.295, 0.520)),
        material=frame_paint,
        name="right_front_leg",
    )
    front_frame.visual(
        Cylinder(radius=0.015, length=0.570),
        origin=Origin(xyz=(0.175, 0.0, 0.800), rpy=_axis_rpy("y")),
        material=frame_paint,
        name="upper_crossbar",
    )
    front_frame.visual(
        Cylinder(radius=0.015, length=0.550),
        origin=Origin(xyz=(0.175, 0.0, 0.350), rpy=_axis_rpy("y")),
        material=frame_paint,
        name="lower_crossbar",
    )
    front_frame.visual(
        Box((0.095, 0.115, 0.065)),
        origin=Origin(xyz=(0.145, 0.0, 0.590)),
        material=frame_paint,
        name="center_release_housing",
    )
    front_frame.visual(
        Box((0.070, 0.090, 0.020)),
        origin=Origin(xyz=(0.128, 0.0, 0.635)),
        material=guard_yellow,
        name="release_guard_plate",
    )
    front_frame.visual(
        Box((0.050, 0.060, 0.018)),
        origin=Origin(xyz=(0.128, 0.0, 0.602)),
        material=lock_red,
        name="release_lockout",
    )
    front_frame.visual(
        Box((0.030, 0.065, 0.195)),
        origin=Origin(xyz=(0.170, 0.0, 0.460)),
        material=frame_paint,
        name="release_column_lower",
    )
    front_frame.visual(
        Box((0.030, 0.065, 0.165)),
        origin=Origin(xyz=(0.170, 0.0, 0.705)),
        material=frame_paint,
        name="release_column_upper",
    )

    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        y = side_sign * 0.295
        front_frame.visual(
            _bushing_mesh(
                f"{side_name}_caster_socket",
                inner_radius=0.0095,
                outer_radius=0.0155,
                length=0.038,
            ),
            origin=Origin(xyz=(0.205, y, 0.177)),
            material=frame_paint,
            name=f"{side_name}_socket_shell",
        )
        front_frame.visual(
            Box((0.060, 0.036, 0.012)),
            origin=Origin(xyz=(0.205, y, 0.164)),
            material=frame_paint,
            name=f"{side_name}_socket_plate",
        )
        front_frame.visual(
            Box((0.040, 0.008, 0.120)),
            origin=Origin(xyz=(0.173, side_sign * 0.277, 0.218)),
            material=guard_yellow,
            name=f"{side_name}_caster_guard_plate",
        )
        front_frame.visual(
            Box((0.055, 0.028, 0.028)),
            origin=Origin(xyz=(0.206, side_sign * 0.332, 0.734)),
            material=lock_red,
            name=f"{side_name}_fold_lockout",
        )
        front_frame.visual(
            Box((0.026, 0.010, 0.230)),
            origin=Origin(xyz=(0.176, side_sign * 0.311, 0.560)),
            material=guard_yellow,
            name=f"{side_name}_pinch_guard",
        )
        front_frame.visual(
            Box((0.038, 0.008, 0.110)),
            origin=Origin(xyz=(0.196, side_sign * 0.324, 0.705)),
            material=frame_paint,
            name=f"{side_name}_upper_hinge_block",
        )
        front_frame.visual(
            Box((0.038, 0.008, 0.110)),
            origin=Origin(xyz=(0.196, side_sign * 0.324, 0.415)),
            material=frame_paint,
            name=f"{side_name}_lower_hinge_block",
        )
        front_frame.visual(
            Box((0.024, 0.012, 0.110)),
            origin=Origin(xyz=(0.188, side_sign * 0.318, 0.705)),
            material=frame_paint,
            name=f"{side_name}_upper_hinge_bridge",
        )
        front_frame.visual(
            Box((0.024, 0.012, 0.110)),
            origin=Origin(xyz=(0.188, side_sign * 0.318, 0.415)),
            material=frame_paint,
            name=f"{side_name}_lower_hinge_bridge",
        )
        front_frame.visual(
            Box((0.022, 0.016, 0.110)),
            origin=Origin(xyz=(0.171, side_sign * 0.303, 0.710)),
            material=guard_yellow,
            name=f"{side_name}_open_stop",
        )
        _add_plate_bolts(
            front_frame,
            x=0.176,
            y=side_sign * 0.315,
            z_values=(0.640, 0.500),
            material=fastener_steel,
            along="y",
        )
        _add_plate_bolts(
            front_frame,
            x=0.173,
            y=side_sign * 0.279,
            z_values=(0.255, 0.185),
            material=fastener_steel,
            along="y",
        )

    _add_member(front_frame, (0.180, 0.240, 0.350), (0.205, 0.285, 0.175), 0.010, frame_paint)
    _add_member(front_frame, (0.180, -0.240, 0.350), (0.205, -0.285, 0.175), 0.010, frame_paint)
    _add_member(front_frame, (0.182, 0.260, 0.640), (0.195, 0.295, 0.780), 0.009, frame_paint)
    _add_member(front_frame, (0.182, -0.260, 0.640), (0.195, -0.295, 0.780), 0.009, frame_paint)

    front_guard = tube_from_spline_points(
        [
            (0.305, -0.220, 0.205),
            (0.330, -0.085, 0.185),
            (0.335, 0.000, 0.175),
            (0.330, 0.085, 0.185),
            (0.305, 0.220, 0.205),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    front_frame.visual(
        _save_mesh("front_guard_hoop", front_guard),
        material=guard_yellow,
        name="front_guard_hoop",
    )
    _add_member(front_frame, (0.195, 0.295, 0.220), (0.305, 0.220, 0.205), 0.010, frame_paint)
    _add_member(front_frame, (0.195, -0.295, 0.220), (0.305, -0.220, 0.205), 0.010, frame_paint)

    left_side_frame = model.part("left_side_frame")
    left_side_frame.inertial = Inertial.from_geometry(
        Box((0.64, 0.08, 0.96)),
        mass=4.6,
        origin=Origin(xyz=(-0.315, 0.0, -0.020)),
    )
    _build_side_frame(
        left_side_frame,
        mesh_prefix="left_side",
        side_sign=1.0,
        frame_paint=frame_paint,
        guard_yellow=guard_yellow,
        lock_red=lock_red,
        grip_black=grip_black,
        fastener_steel=fastener_steel,
        rubber_black=rubber_black,
    )

    right_side_frame = model.part("right_side_frame")
    right_side_frame.inertial = Inertial.from_geometry(
        Box((0.64, 0.08, 0.96)),
        mass=4.6,
        origin=Origin(xyz=(-0.315, 0.0, -0.020)),
    )
    _build_side_frame(
        right_side_frame,
        mesh_prefix="right_side",
        side_sign=-1.0,
        frame_paint=frame_paint,
        guard_yellow=guard_yellow,
        lock_red=lock_red,
        grip_black=grip_black,
        fastener_steel=fastener_steel,
        rubber_black=rubber_black,
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.120, 0.056, 0.180)),
        mass=0.7,
        origin=Origin(xyz=(-0.045, 0.0, -0.085)),
    )
    left_caster_fork.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=fastener_steel,
        name="left_swivel_collar",
    )
    left_caster_fork.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=fastener_steel,
        name="left_swivel_stem",
    )
    left_caster_fork.visual(
        Box((0.024, 0.052, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, -0.076)),
        material=frame_paint,
        name="left_fork_crown",
    )
    left_caster_fork.visual(
        Box((0.062, 0.006, 0.010)),
        origin=Origin(xyz=(-0.053, 0.020, -0.081)),
        material=frame_paint,
        name="left_trailing_arm_outer",
    )
    left_caster_fork.visual(
        Box((0.062, 0.006, 0.010)),
        origin=Origin(xyz=(-0.053, -0.020, -0.081)),
        material=frame_paint,
        name="left_trailing_arm_inner",
    )
    left_caster_fork.visual(
        Box((0.012, 0.006, 0.086)),
        origin=Origin(xyz=(-0.089, 0.020, -0.119)),
        material=frame_paint,
        name="left_outer_tine",
    )
    left_caster_fork.visual(
        Box((0.012, 0.006, 0.086)),
        origin=Origin(xyz=(-0.089, -0.020, -0.119)),
        material=frame_paint,
        name="left_inner_tine",
    )

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.120, 0.056, 0.180)),
        mass=0.7,
        origin=Origin(xyz=(-0.045, 0.0, -0.085)),
    )
    right_caster_fork.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=fastener_steel,
        name="right_swivel_collar",
    )
    right_caster_fork.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=fastener_steel,
        name="right_swivel_stem",
    )
    right_caster_fork.visual(
        Box((0.024, 0.052, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, -0.076)),
        material=frame_paint,
        name="right_fork_crown",
    )
    right_caster_fork.visual(
        Box((0.062, 0.006, 0.010)),
        origin=Origin(xyz=(-0.053, 0.020, -0.081)),
        material=frame_paint,
        name="right_trailing_arm_outer",
    )
    right_caster_fork.visual(
        Box((0.062, 0.006, 0.010)),
        origin=Origin(xyz=(-0.053, -0.020, -0.081)),
        material=frame_paint,
        name="right_trailing_arm_inner",
    )
    right_caster_fork.visual(
        Box((0.012, 0.006, 0.086)),
        origin=Origin(xyz=(-0.089, 0.020, -0.119)),
        material=frame_paint,
        name="right_outer_tine",
    )
    right_caster_fork.visual(
        Box((0.012, 0.006, 0.086)),
        origin=Origin(xyz=(-0.089, -0.020, -0.119)),
        material=frame_paint,
        name="right_inner_tine",
    )

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.034),
        mass=0.55,
        origin=Origin(rpy=_axis_rpy("y")),
    )
    left_caster_wheel.visual(
        _wheel_mesh(
            "left_caster_wheel_shell",
            radius=0.075,
            width=0.034,
            bore_radius=0.0082,
        ),
        material=wheel_black,
        name="left_wheel_shell",
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.034),
        mass=0.55,
        origin=Origin(rpy=_axis_rpy("y")),
    )
    right_caster_wheel.visual(
        _wheel_mesh(
            "right_caster_wheel_shell",
            radius=0.075,
            width=0.034,
            bore_radius=0.0082,
        ),
        material=wheel_black,
        name="right_wheel_shell",
    )

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_side_frame,
        origin=Origin(xyz=(0.145, 0.295, 0.560)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=0.42),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_side_frame,
        origin=Origin(xyz=(0.145, -0.295, 0.560)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=0.42),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=front_frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.205, 0.295, 0.158)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=front_frame,
        child=right_caster_fork,
        origin=Origin(xyz=(0.205, -0.295, 0.158)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )
    model.articulation(
        "left_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(-0.088, 0.0, -0.123)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=18.0),
    )
    model.articulation(
        "right_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.088, 0.0, -0.123)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    left_side_frame = object_model.get_part("left_side_frame")
    right_side_frame = object_model.get_part("right_side_frame")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_spin = object_model.get_articulation("left_caster_spin")
    right_caster_spin = object_model.get_articulation("right_caster_spin")

    left_socket_plate = front_frame.get_visual("left_socket_plate")
    right_socket_plate = front_frame.get_visual("right_socket_plate")
    left_swivel_collar = left_caster_fork.get_visual("left_swivel_collar")
    right_swivel_collar = right_caster_fork.get_visual("right_swivel_collar")

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
        "fold_joint_axes_and_limits",
        left_fold.axis == (0.0, 0.0, 1.0)
        and right_fold.axis == (0.0, 0.0, -1.0)
        and left_fold.motion_limits is not None
        and right_fold.motion_limits is not None
        and left_fold.motion_limits.lower == 0.0
        and right_fold.motion_limits.lower == 0.0
        and left_fold.motion_limits.upper == 0.42
        and right_fold.motion_limits.upper == 0.42,
        "Fold joints should open against hard stops and close inward on mirrored vertical axes.",
    )
    ctx.check(
        "caster_axes",
        left_caster_swivel.axis == (0.0, 0.0, 1.0)
        and right_caster_swivel.axis == (0.0, 0.0, 1.0)
        and left_caster_spin.axis == (0.0, 1.0, 0.0)
        and right_caster_spin.axis == (0.0, 1.0, 0.0),
        "Front casters should swivel about vertical stems and roll about lateral axles.",
    )

    with ctx.pose({left_fold: 0.0, right_fold: 0.0}):
        ctx.expect_contact(
            left_caster_fork,
            front_frame,
            elem_a=left_swivel_collar,
            elem_b=left_socket_plate,
            name="left_caster_mount_contact",
        )
        ctx.expect_contact(
            right_caster_fork,
            front_frame,
            elem_a=right_swivel_collar,
            elem_b=right_socket_plate,
            name="right_caster_mount_contact",
        )
        ctx.expect_contact(
            left_caster_wheel,
            left_caster_fork,
            name="left_wheel_supported_in_fork",
        )
        ctx.expect_contact(
            right_caster_wheel,
            right_caster_fork,
            name="right_wheel_supported_in_fork",
        )
        ctx.expect_gap(
            left_side_frame,
            right_side_frame,
            axis="y",
            min_gap=0.48,
            name="open_frame_width",
        )

    with ctx.pose({left_fold: 0.38, right_fold: 0.38}):
        ctx.expect_gap(
            left_side_frame,
            right_side_frame,
            axis="y",
            min_gap=0.06,
            max_gap=0.28,
            name="folded_frame_narrower_than_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
