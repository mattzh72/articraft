from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

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
)


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
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_axis_cylinder(
    part,
    *,
    center: tuple[float, float, float],
    radius: float,
    length: float,
    axis: str,
    material,
    name: str | None = None,
) -> None:
    if axis == "x":
        rpy = (0.0, pi / 2.0, 0.0)
    elif axis == "y":
        rpy = (pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _add_plate_fasteners(
    part,
    *,
    x_positions: tuple[float, float],
    z_positions: tuple[float, float],
    y: float,
    radius: float,
    length: float,
    material,
    prefix: str,
) -> None:
    for xi, x in enumerate(x_positions):
        for zi, z in enumerate(z_positions):
            _add_axis_cylinder(
                part,
                center=(x, y, z),
                radius=radius,
                length=length,
                axis="y",
                material=material,
                name=f"{prefix}_{xi}_{zi}",
            )


def _build_drive_wheel(part, *, prefix: str, side_sign: float, tire, rim, steel) -> None:
    tire_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.293, tube=0.025, radial_segments=20, tubular_segments=48).rotate_x(pi / 2.0),
        f"{prefix}_tire",
    )
    rim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.251, tube=0.017, radial_segments=18, tubular_segments=40).rotate_x(pi / 2.0),
        f"{prefix}_rim",
    )
    handrim_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.286, tube=0.006, radial_segments=14, tubular_segments=36).rotate_x(pi / 2.0),
        f"{prefix}_handrim",
    )

    tire_offset = 0.040 * side_sign
    rim_offset = 0.036 * side_sign
    handrim_offset = 0.064 * side_sign
    bearing_offset = 0.003 * side_sign
    hub_offset = 0.031 * side_sign

    part.visual(tire_mesh, origin=Origin(xyz=(0.0, tire_offset, 0.0)), material=tire, name="tire")
    part.visual(rim_mesh, origin=Origin(xyz=(0.0, rim_offset, 0.0)), material=rim, name="rim")
    part.visual(
        handrim_mesh,
        origin=Origin(xyz=(0.0, handrim_offset, 0.0)),
        material=steel,
        name="handrim",
    )

    _add_axis_cylinder(
        part,
        center=(0.0, bearing_offset, 0.0),
        radius=0.034,
        length=0.006,
        axis="y",
        material=steel,
        name="bearing_face",
    )
    _add_axis_cylinder(
        part,
        center=(0.0, hub_offset, 0.0),
        radius=0.050,
        length=0.056,
        axis="y",
        material=steel,
        name="hub_shell",
    )
    _add_axis_cylinder(
        part,
        center=(0.0, (0.047 * side_sign), 0.0),
        radius=0.076,
        length=0.014,
        axis="y",
        material=rim,
        name="hub_flange",
    )
    _add_axis_cylinder(
        part,
        center=(0.0, rim_offset, 0.0),
        radius=0.268,
        length=0.010,
        axis="y",
        material=rim,
        name="bead_seat",
    )
    _add_axis_cylinder(
        part,
        center=(0.0, 0.060 * side_sign, 0.0),
        radius=0.026,
        length=0.014,
        axis="y",
        material=steel,
        name="hub_cap",
    )

    spoke_y = 0.036 * side_sign
    for index in range(6):
        angle = 2.0 * pi * index / 6.0
        inner = (cos(angle) * 0.072, spoke_y, sin(angle) * 0.072)
        outer = (cos(angle) * 0.245, spoke_y, sin(angle) * 0.245)
        _add_member(
            part,
            inner,
            outer,
            radius=0.0045,
            material=rim,
            name=f"spoke_{index}",
        )

    for index, angle in enumerate((pi / 4.0, 3.0 * pi / 4.0, 5.0 * pi / 4.0, 7.0 * pi / 4.0)):
        inner = (cos(angle) * 0.266, 0.050 * side_sign, sin(angle) * 0.266)
        outer = (cos(angle) * 0.286, handrim_offset, sin(angle) * 0.286)
        _add_member(
            part,
            inner,
            outer,
            radius=0.0038,
            material=steel,
            name=f"handrim_standoff_{index}",
        )


def _build_caster(part, *, steel, tire, side_sign: float) -> None:
    _add_axis_cylinder(
        part,
        center=(0.0, 0.0, -0.050),
        radius=0.015,
        length=0.100,
        axis="z",
        material=steel,
        name="stem",
    )
    _add_axis_cylinder(
        part,
        center=(0.0, 0.0, -0.005),
        radius=0.024,
        length=0.010,
        axis="z",
        material=steel,
        name="stem_collar",
    )
    part.visual(
        Box((0.060, 0.042, 0.018)),
        origin=Origin(xyz=(-0.020, 0.0, -0.050)),
        material=steel,
        name="crown",
    )
    part.visual(
        Box((0.020, 0.012, 0.028)),
        origin=Origin(xyz=(0.010, 0.0, -0.022)),
        material=steel,
        name="stop_lug",
    )
    part.visual(
        Box((0.018, 0.012, 0.112)),
        origin=Origin(xyz=(-0.038, 0.018, -0.114)),
        material=steel,
        name="left_fork_arm",
    )
    part.visual(
        Box((0.018, 0.012, 0.112)),
        origin=Origin(xyz=(-0.038, -0.018, -0.114)),
        material=steel,
        name="right_fork_arm",
    )
    _add_axis_cylinder(
        part,
        center=(-0.038, 0.0, -0.088),
        radius=0.007,
        length=0.050,
        axis="y",
        material=steel,
        name="fork_bridge",
    )
    _add_axis_cylinder(
        part,
        center=(-0.038, 0.0, -0.110),
        radius=0.007,
        length=0.055,
        axis="y",
        material=steel,
        name="axle_bolt",
    )
    _add_axis_cylinder(
        part,
        center=(-0.038, 0.0, -0.110),
        radius=0.100,
        length=0.028,
        axis="y",
        material=tire,
        name="wheel_tire",
    )
    _add_axis_cylinder(
        part,
        center=(-0.038, 0.0, -0.110),
        radius=0.052,
        length=0.034,
        axis="y",
        material=steel,
        name="wheel_hub",
    )
    cap_y = 0.020 * side_sign
    _add_axis_cylinder(
        part,
        center=(-0.038, cap_y, -0.110),
        radius=0.014,
        length=0.006,
        axis="y",
        material=steel,
        name="axle_cap",
    )


def _build_footrest(part, *, steel, tread, safety_orange) -> None:
    _add_axis_cylinder(
        part,
        center=(0.0, 0.0, -0.050),
        radius=0.012,
        length=0.100,
        axis="z",
        material=steel,
        name="mount_stem",
    )
    _add_axis_cylinder(
        part,
        center=(0.0, 0.0, -0.005),
        radius=0.020,
        length=0.010,
        axis="z",
        material=steel,
        name="mount_collar",
    )
    _add_member(
        part,
        (0.0, 0.0, -0.050),
        (0.038, 0.0, -0.192),
        radius=0.012,
        material=steel,
        name="hanger_tube",
    )
    part.visual(
        Box((0.160, 0.095, 0.012)),
        origin=Origin(xyz=(0.065, 0.0, -0.196)),
        material=tread,
        name="tread_plate",
    )
    part.visual(
        Box((0.130, 0.020, 0.010)),
        origin=Origin(xyz=(0.055, 0.0, -0.188)),
        material=steel,
        name="plate_rib",
    )
    _add_axis_cylinder(
        part,
        center=(0.115, 0.0, -0.182),
        radius=0.008,
        length=0.078,
        axis="y",
        material=steel,
        name="heel_bar",
    )
    _add_member(
        part,
        (0.111, -0.031, -0.182),
        (0.111, -0.031, -0.220),
        radius=0.004,
        material=steel,
        name="heel_strut_right",
    )
    _add_member(
        part,
        (0.111, 0.031, -0.182),
        (0.111, 0.031, -0.220),
        radius=0.004,
        material=steel,
        name="heel_strut_left",
    )
    part.visual(
        Box((0.028, 0.032, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=safety_orange,
        name="travel_stop",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_wheelchair")

    frame_coat = model.material("frame_coat", rgba=(0.20, 0.22, 0.24, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.82, 0.70, 0.18, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    seat_black = model.material("seat_black", rgba=(0.14, 0.14, 0.15, 1.0))
    tread_black = model.material("tread_black", rgba=(0.12, 0.12, 0.13, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.78, 0.35, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.08, 0.74, 1.02)),
        mass=34.0,
        origin=Origin(xyz=(0.02, 0.0, 0.51)),
    )

    # Main side rails and cross structure.
    _add_member(frame, (-0.18, 0.24, 0.54), (0.19, 0.24, 0.54), radius=0.018, material=frame_coat, name="left_seat_rail")
    _add_member(frame, (-0.18, -0.24, 0.54), (0.19, -0.24, 0.54), radius=0.018, material=frame_coat, name="right_seat_rail")
    _add_member(frame, (-0.18, 0.24, 0.54), (-0.18, 0.24, 0.96), radius=0.018, material=frame_coat, name="left_back_post")
    _add_member(frame, (-0.18, -0.24, 0.54), (-0.18, -0.24, 0.96), radius=0.018, material=frame_coat, name="right_back_post")
    _add_member(frame, (-0.08, -0.19, 0.54), (-0.08, 0.19, 0.54), radius=0.015, material=frame_coat, name="rear_crossmember")
    _add_member(frame, (0.18, -0.24, 0.54), (0.18, 0.24, 0.54), radius=0.013, material=frame_coat, name="front_crossmember")
    _add_member(frame, (-0.18, -0.16, 0.88), (-0.18, 0.16, 0.88), radius=0.012, material=frame_coat, name="back_crossbar")

    # Push handles.
    _add_member(frame, (-0.18, 0.24, 0.96), (-0.18, 0.24, 0.99), radius=0.014, material=frame_coat)
    _add_member(frame, (-0.18, -0.24, 0.96), (-0.18, -0.24, 0.99), radius=0.014, material=frame_coat)
    _add_axis_cylinder(frame, center=(-0.225, 0.24, 0.99), radius=0.015, length=0.090, axis="x", material=frame_coat, name="left_push_handle")
    _add_axis_cylinder(frame, center=(-0.225, -0.24, 0.99), radius=0.015, length=0.090, axis="x", material=frame_coat, name="right_push_handle")

    # Armrest and side support structure.
    _add_member(frame, (0.04, 0.24, 0.54), (0.04, 0.24, 0.72), radius=0.012, material=frame_coat, name="left_arm_post")
    _add_member(frame, (0.04, -0.24, 0.54), (0.04, -0.24, 0.72), radius=0.012, material=frame_coat, name="right_arm_post")
    frame.visual(Box((0.240, 0.050, 0.030)), origin=Origin(xyz=(0.055, 0.24, 0.735)), material=guard_yellow, name="left_arm_pad")
    frame.visual(Box((0.240, 0.050, 0.030)), origin=Origin(xyz=(0.055, -0.24, 0.735)), material=guard_yellow, name="right_arm_pad")

    # Front rigging and caster support.
    _add_member(frame, (0.18, 0.24, 0.54), (0.34, 0.18, 0.30), radius=0.016, material=frame_coat, name="left_front_down_tube")
    _add_member(frame, (0.18, -0.24, 0.54), (0.34, -0.18, 0.30), radius=0.016, material=frame_coat, name="right_front_down_tube")
    _add_member(frame, (0.355, -0.18, 0.286), (0.355, 0.18, 0.286), radius=0.014, material=frame_coat, name="lower_front_crossbar")
    frame.visual(Box((0.044, 0.024, 0.120)), origin=Origin(xyz=(0.355, 0.18, 0.286)), material=frame_coat, name="left_caster_block")
    frame.visual(Box((0.044, 0.024, 0.120)), origin=Origin(xyz=(0.355, -0.18, 0.286)), material=frame_coat, name="right_caster_block")
    _add_axis_cylinder(frame, center=(0.390, 0.18, 0.235), radius=0.024, length=0.050, axis="z", material=steel, name="left_caster_receiver")
    _add_axis_cylinder(frame, center=(0.390, -0.18, 0.235), radius=0.024, length=0.050, axis="z", material=steel, name="right_caster_receiver")
    frame.visual(Box((0.030, 0.040, 0.018)), origin=Origin(xyz=(0.382, 0.18, 0.274)), material=steel, name="left_caster_stop_cap")
    frame.visual(Box((0.030, 0.040, 0.018)), origin=Origin(xyz=(0.382, -0.18, 0.274)), material=steel, name="right_caster_stop_cap")
    frame.visual(Box((0.012, 0.012, 0.018)), origin=Origin(xyz=(0.398, 0.202, 0.272)), material=safety_orange, name="left_caster_stop_outer")
    frame.visual(Box((0.012, 0.012, 0.018)), origin=Origin(xyz=(0.398, 0.158, 0.272)), material=safety_orange, name="left_caster_stop_inner")
    frame.visual(Box((0.012, 0.012, 0.018)), origin=Origin(xyz=(0.398, -0.202, 0.272)), material=safety_orange, name="right_caster_stop_outer")
    frame.visual(Box((0.012, 0.012, 0.018)), origin=Origin(xyz=(0.398, -0.158, 0.272)), material=safety_orange, name="right_caster_stop_inner")

    # Axle structure, gussets, and wheel-side safety details.
    frame.visual(Box((0.120, 0.020, 0.260)), origin=Origin(xyz=(-0.080, 0.295, 0.460)), material=frame_coat, name="left_axle_plate")
    frame.visual(Box((0.120, 0.020, 0.260)), origin=Origin(xyz=(-0.080, -0.295, 0.460)), material=frame_coat, name="right_axle_plate")
    frame.visual(Box((0.160, 0.050, 0.030)), origin=Origin(xyz=(-0.100, 0.270, 0.535)), material=frame_coat, name="left_upper_axle_brace")
    frame.visual(Box((0.160, 0.050, 0.030)), origin=Origin(xyz=(-0.100, -0.270, 0.535)), material=frame_coat, name="right_upper_axle_brace")
    frame.visual(Box((0.100, 0.050, 0.030)), origin=Origin(xyz=(-0.095, 0.270, 0.395)), material=frame_coat, name="left_lower_axle_brace")
    frame.visual(Box((0.100, 0.050, 0.030)), origin=Origin(xyz=(-0.095, -0.270, 0.395)), material=frame_coat, name="right_lower_axle_brace")
    _add_axis_cylinder(frame, center=(-0.080, 0.301, 0.320), radius=0.042, length=0.004, axis="y", material=steel, name="left_axle_stop")
    _add_axis_cylinder(frame, center=(-0.080, -0.301, 0.320), radius=0.042, length=0.004, axis="y", material=steel, name="right_axle_stop")
    _add_axis_cylinder(frame, center=(-0.080, 0.312, 0.320), radius=0.026, length=0.022, axis="y", material=steel, name="left_axle_spindle")
    _add_axis_cylinder(frame, center=(-0.080, -0.312, 0.320), radius=0.026, length=0.022, axis="y", material=steel, name="right_axle_spindle")
    _add_plate_fasteners(
        frame,
        x_positions=(-0.103, -0.057),
        z_positions=(0.390, 0.550),
        y=0.295,
        radius=0.005,
        length=0.028,
        material=steel,
        prefix="left_axle_fastener",
    )
    _add_plate_fasteners(
        frame,
        x_positions=(-0.103, -0.057),
        z_positions=(0.390, 0.550),
        y=-0.295,
        radius=0.005,
        length=0.028,
        material=steel,
        prefix="right_axle_fastener",
    )
    frame.visual(Box((0.080, 0.028, 0.050)), origin=Origin(xyz=(-0.020, 0.286, 0.550)), material=frame_coat, name="left_brake_clamp")
    frame.visual(Box((0.080, 0.028, 0.050)), origin=Origin(xyz=(-0.020, -0.286, 0.550)), material=frame_coat, name="right_brake_clamp")
    frame.visual(Box((0.028, 0.012, 0.140)), origin=Origin(xyz=(-0.035, 0.306, 0.470)), material=safety_orange, name="left_brake_lockout")
    frame.visual(Box((0.028, 0.012, 0.140)), origin=Origin(xyz=(-0.035, -0.306, 0.470)), material=safety_orange, name="right_brake_lockout")
    frame.visual(Box((0.018, 0.018, 0.060)), origin=Origin(xyz=(-0.005, 0.292, 0.495)), material=steel, name="left_brake_stop")
    frame.visual(Box((0.018, 0.018, 0.060)), origin=Origin(xyz=(-0.005, -0.292, 0.495)), material=steel, name="right_brake_stop")

    # Safety side guards over the drive wheels.
    frame.visual(Box((0.320, 0.006, 0.220)), origin=Origin(xyz=(0.030, 0.303, 0.630)), material=guard_yellow, name="left_guard_panel")
    frame.visual(Box((0.320, 0.006, 0.220)), origin=Origin(xyz=(0.030, -0.303, 0.630)), material=guard_yellow, name="right_guard_panel")
    frame.visual(Box((0.020, 0.050, 0.180)), origin=Origin(xyz=(-0.045, 0.275, 0.630)), material=steel, name="left_guard_bracket_rear")
    frame.visual(Box((0.020, 0.050, 0.180)), origin=Origin(xyz=(-0.045, -0.275, 0.630)), material=steel, name="right_guard_bracket_rear")
    frame.visual(Box((0.020, 0.050, 0.160)), origin=Origin(xyz=(0.105, 0.275, 0.615)), material=steel, name="left_guard_bracket_front")
    frame.visual(Box((0.020, 0.050, 0.160)), origin=Origin(xyz=(0.105, -0.275, 0.615)), material=steel, name="right_guard_bracket_front")
    _add_plate_fasteners(
        frame,
        x_positions=(-0.045, 0.105),
        z_positions=(0.560, 0.680),
        y=0.303,
        radius=0.004,
        length=0.014,
        material=steel,
        prefix="left_guard_bolt",
    )
    _add_plate_fasteners(
        frame,
        x_positions=(-0.045, 0.105),
        z_positions=(0.560, 0.680),
        y=-0.303,
        radius=0.004,
        length=0.014,
        material=steel,
        prefix="right_guard_bolt",
    )

    # Footrest receiver sockets and stops.
    frame.visual(Box((0.036, 0.024, 0.100)), origin=Origin(xyz=(0.262, 0.130, 0.300)), material=frame_coat, name="left_foot_socket_block")
    frame.visual(Box((0.036, 0.024, 0.100)), origin=Origin(xyz=(0.262, -0.130, 0.300)), material=frame_coat, name="right_foot_socket_block")
    _add_axis_cylinder(frame, center=(0.300, 0.105, 0.295), radius=0.022, length=0.030, axis="z", material=steel, name="left_foot_socket")
    _add_axis_cylinder(frame, center=(0.300, -0.105, 0.295), radius=0.022, length=0.030, axis="z", material=steel, name="right_foot_socket")
    _add_member(frame, (0.335, 0.176, 0.302), (0.262, 0.130, 0.320), radius=0.010, material=frame_coat, name="left_foot_socket_brace")
    _add_member(frame, (0.335, -0.176, 0.302), (0.262, -0.130, 0.320), radius=0.010, material=frame_coat, name="right_foot_socket_brace")
    _add_member(frame, (0.280, 0.120, 0.305), (0.300, 0.105, 0.295), radius=0.010, material=steel, name="left_socket_bridge")
    _add_member(frame, (0.280, -0.120, 0.305), (0.300, -0.105, 0.295), radius=0.010, material=steel, name="right_socket_bridge")
    _add_axis_cylinder(frame, center=(0.262, 0.130, 0.270), radius=0.004, length=0.024, axis="y", material=steel, name="left_socket_bolt_0")
    _add_axis_cylinder(frame, center=(0.262, 0.130, 0.330), radius=0.004, length=0.024, axis="y", material=steel, name="left_socket_bolt_1")
    _add_axis_cylinder(frame, center=(0.262, -0.130, 0.270), radius=0.004, length=0.024, axis="y", material=steel, name="right_socket_bolt_0")
    _add_axis_cylinder(frame, center=(0.262, -0.130, 0.330), radius=0.004, length=0.024, axis="y", material=steel, name="right_socket_bolt_1")

    # Seat and back support surfaces.
    frame.visual(Box((0.440, 0.460, 0.024)), origin=Origin(xyz=(0.010, 0.0, 0.566)), material=seat_black, name="seat_pan")
    frame.visual(Box((0.030, 0.480, 0.340)), origin=Origin(xyz=(-0.195, 0.0, 0.780)), material=seat_black, name="backrest_panel")

    # Rear anti-tip guard / bumper structure.
    _add_member(frame, (-0.180, 0.240, 0.540), (-0.300, 0.180, 0.120), radius=0.012, material=frame_coat, name="left_rear_guard_strut")
    _add_member(frame, (-0.180, -0.240, 0.540), (-0.300, -0.180, 0.120), radius=0.012, material=frame_coat, name="right_rear_guard_strut")
    _add_member(frame, (-0.300, -0.180, 0.120), (-0.300, 0.180, 0.120), radius=0.012, material=frame_coat, name="rear_guard_crossbar")

    left_drive_wheel = model.part("left_drive_wheel")
    left_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.320, length=0.080),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _build_drive_wheel(
        left_drive_wheel,
        prefix="left_drive",
        side_sign=1.0,
        tire=rubber_black,
        rim=rim_silver,
        steel=steel,
    )

    right_drive_wheel = model.part("right_drive_wheel")
    right_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.320, length=0.080),
        mass=7.0,
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    _build_drive_wheel(
        right_drive_wheel,
        prefix="right_drive",
        side_sign=-1.0,
        tire=rubber_black,
        rim=rim_silver,
        steel=steel,
    )

    left_caster = model.part("left_caster")
    left_caster.inertial = Inertial.from_geometry(
        Box((0.120, 0.080, 0.220)),
        mass=2.3,
        origin=Origin(xyz=(-0.020, 0.0, -0.105)),
    )
    _build_caster(left_caster, steel=steel, tire=rubber_black, side_sign=1.0)

    right_caster = model.part("right_caster")
    right_caster.inertial = Inertial.from_geometry(
        Box((0.120, 0.080, 0.220)),
        mass=2.3,
        origin=Origin(xyz=(-0.020, 0.0, -0.105)),
    )
    _build_caster(right_caster, steel=steel, tire=rubber_black, side_sign=-1.0)

    left_footrest = model.part("left_footrest")
    left_footrest.inertial = Inertial.from_geometry(
        Box((0.180, 0.100, 0.220)),
        mass=1.4,
        origin=Origin(xyz=(0.070, 0.0, -0.110)),
    )
    _build_footrest(left_footrest, steel=steel, tread=tread_black, safety_orange=safety_orange)

    right_footrest = model.part("right_footrest")
    right_footrest.inertial = Inertial.from_geometry(
        Box((0.180, 0.100, 0.220)),
        mass=1.4,
        origin=Origin(xyz=(0.070, 0.0, -0.110)),
    )
    _build_footrest(right_footrest, steel=steel, tread=tread_black, safety_orange=safety_orange)

    left_drive_spin = model.articulation(
        "left_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(-0.080, 0.323, 0.320)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=25.0),
    )
    right_drive_spin = model.articulation(
        "right_drive_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.080, -0.323, 0.320)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=25.0),
    )
    left_caster_swivel = model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster,
        origin=Origin(xyz=(0.390, 0.180, 0.210)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    right_caster_swivel = model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster,
        origin=Origin(xyz=(0.390, -0.180, 0.210)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    model.articulation(
        "left_footrest_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=left_footrest,
        origin=Origin(xyz=(0.300, 0.105, 0.280)),
    )
    model.articulation(
        "right_footrest_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=right_footrest,
        origin=Origin(xyz=(0.300, -0.105, 0.280)),
    )

    frame.meta["primary_articulations"] = (
        left_drive_spin.name,
        right_drive_spin.name,
        left_caster_swivel.name,
        right_caster_swivel.name,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_drive_wheel = object_model.get_part("left_drive_wheel")
    right_drive_wheel = object_model.get_part("right_drive_wheel")
    left_caster = object_model.get_part("left_caster")
    right_caster = object_model.get_part("right_caster")
    left_footrest = object_model.get_part("left_footrest")
    right_footrest = object_model.get_part("right_footrest")

    left_drive_spin = object_model.get_articulation("left_drive_spin")
    right_drive_spin = object_model.get_articulation("right_drive_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")

    left_axle_spindle = frame.get_visual("left_axle_spindle")
    right_axle_spindle = frame.get_visual("right_axle_spindle")
    left_receiver = frame.get_visual("left_caster_receiver")
    right_receiver = frame.get_visual("right_caster_receiver")
    left_socket = frame.get_visual("left_foot_socket")
    right_socket = frame.get_visual("right_foot_socket")

    left_bearing_face = left_drive_wheel.get_visual("bearing_face")
    right_bearing_face = right_drive_wheel.get_visual("bearing_face")
    left_stem_collar = left_caster.get_visual("stem_collar")
    right_stem_collar = right_caster.get_visual("stem_collar")
    left_mount_collar = left_footrest.get_visual("mount_collar")
    right_mount_collar = right_footrest.get_visual("mount_collar")

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
        "drive_wheel_axes_are_lateral",
        left_drive_spin.axis == (0.0, 1.0, 0.0) and right_drive_spin.axis == (0.0, 1.0, 0.0),
        details=f"left={left_drive_spin.axis}, right={right_drive_spin.axis}",
    )
    ctx.check(
        "caster_axes_are_vertical",
        left_caster_swivel.axis == (0.0, 0.0, 1.0) and right_caster_swivel.axis == (0.0, 0.0, 1.0),
        details=f"left={left_caster_swivel.axis}, right={right_caster_swivel.axis}",
    )

    ctx.expect_contact(
        left_drive_wheel,
        frame,
        elem_a=left_bearing_face,
        elem_b=left_axle_spindle,
        name="left_drive_wheel_bearing_contact",
    )
    ctx.expect_contact(
        right_drive_wheel,
        frame,
        elem_a=right_bearing_face,
        elem_b=right_axle_spindle,
        name="right_drive_wheel_bearing_contact",
    )
    ctx.expect_contact(
        left_caster,
        frame,
        elem_a=left_stem_collar,
        elem_b=left_receiver,
        name="left_caster_receiver_contact",
    )
    ctx.expect_contact(
        right_caster,
        frame,
        elem_a=right_stem_collar,
        elem_b=right_receiver,
        name="right_caster_receiver_contact",
    )
    ctx.expect_contact(
        left_footrest,
        frame,
        elem_a=left_mount_collar,
        elem_b=left_socket,
        name="left_footrest_socket_contact",
    )
    ctx.expect_contact(
        right_footrest,
        frame,
        elem_a=right_mount_collar,
        elem_b=right_socket,
        name="right_footrest_socket_contact",
    )

    ctx.expect_origin_distance(
        left_drive_wheel,
        right_drive_wheel,
        axes="y",
        min_dist=0.62,
        max_dist=0.68,
        name="drive_track_width",
    )
    ctx.expect_origin_gap(
        left_caster,
        left_drive_wheel,
        axis="x",
        min_gap=0.42,
        max_gap=0.52,
        name="left_caster_leads_drive_wheel",
    )
    ctx.expect_origin_gap(
        left_footrest,
        left_drive_wheel,
        axis="x",
        min_gap=0.34,
        name="left_footrest_forward_of_drive_wheel",
    )

    with ctx.pose({left_caster_swivel: pi / 2.0, right_caster_swivel: -pi / 2.0}):
        ctx.expect_contact(
            left_caster,
            frame,
            elem_a=left_stem_collar,
            elem_b=left_receiver,
            name="left_caster_contact_when_swiveled",
        )
        ctx.expect_contact(
            right_caster,
            frame,
            elem_a=right_stem_collar,
            elem_b=right_receiver,
            name="right_caster_contact_when_swiveled",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
