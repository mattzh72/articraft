from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    tube_from_spline_points,
)


def _signed_power(value: float, power: float) -> float:
    return math.copysign(abs(value) ** power, value)


def _superellipse_loop_x(
    x: float,
    radius_y: float,
    radius_z: float,
    *,
    exponent: float = 2.0,
    count: int = 32,
    z_offset: float = 0.0,
    top_scale: float = 1.0,
    bottom_scale: float = 1.0,
) -> list[tuple[float, float, float]]:
    pts: list[tuple[float, float, float]] = []
    power = 2.0 / exponent
    for index in range(count):
        angle = math.tau * index / count
        y = radius_y * _signed_power(math.cos(angle), power)
        z = radius_z * _signed_power(math.sin(angle), power)
        z *= top_scale if z >= 0.0 else bottom_scale
        pts.append((x, y, z + z_offset))
    return pts


def _airfoil_loop_y(
    y: float,
    chord: float,
    thickness: float,
    *,
    leading_x: float = 0.0,
) -> list[tuple[float, float, float]]:
    trail_x = leading_x - chord
    half_t = thickness * 0.5
    return [
        (leading_x, y, 0.0),
        (leading_x - 0.05 * chord, y, 0.42 * half_t),
        (leading_x - 0.22 * chord, y, 0.62 * half_t),
        (leading_x - 0.54 * chord, y, 0.42 * half_t),
        (trail_x, y, 0.07 * half_t),
        (trail_x + 0.02 * chord, y, -0.07 * half_t),
        (leading_x - 0.52 * chord, y, -0.32 * half_t),
        (leading_x - 0.18 * chord, y, -0.42 * half_t),
        (leading_x - 0.04 * chord, y, -0.28 * half_t),
    ]


def _airfoil_loop_z(
    z: float,
    chord: float,
    thickness: float,
    *,
    leading_x: float = 0.0,
) -> list[tuple[float, float, float]]:
    trail_x = leading_x - chord
    half_t = thickness * 0.5
    return [
        (leading_x, 0.0, z),
        (leading_x - 0.05 * chord, 0.42 * half_t, z),
        (leading_x - 0.22 * chord, 0.62 * half_t, z),
        (leading_x - 0.54 * chord, 0.42 * half_t, z),
        (trail_x, 0.07 * half_t, z),
        (trail_x + 0.02 * chord, -0.07 * half_t, z),
        (leading_x - 0.52 * chord, -0.32 * half_t, z),
        (leading_x - 0.18 * chord, -0.42 * half_t, z),
        (leading_x - 0.04 * chord, -0.28 * half_t, z),
    ]


def _body_mesh(
    name: str,
    sections: list[tuple[float, float, float]],
    *,
    exponent: float = 2.0,
    count: int = 36,
    z_offset: float = 0.0,
    top_scale: float = 1.0,
    bottom_scale: float = 1.0,
):
    loops = [
        _superellipse_loop_x(
            x,
            radius_y,
            radius_z,
            exponent=exponent,
            count=count,
            z_offset=z_offset,
            top_scale=top_scale,
            bottom_scale=bottom_scale,
        )
        for x, radius_y, radius_z in sections
    ]
    return mesh_from_geometry(repair_loft(loops), name)


def _horizontal_surface_mesh(
    name: str,
    *,
    span: float,
    root_chord: float,
    tip_chord: float,
    root_thickness: float,
    tip_thickness: float,
    sweep: float,
    side_sign: float,
):
    sections = []
    for frac in (0.0, 0.52, 1.0):
        y = side_sign * span * frac
        sections.append(
            _airfoil_loop_y(
                y,
                root_chord + (tip_chord - root_chord) * frac,
                root_thickness + (tip_thickness - root_thickness) * frac,
                leading_x=-(sweep * frac),
            )
        )
    return mesh_from_geometry(repair_loft(sections), name)


def _vertical_surface_mesh(
    name: str,
    *,
    span: float,
    root_chord: float,
    tip_chord: float,
    root_thickness: float,
    tip_thickness: float,
    sweep: float,
    upward: bool = True,
):
    sign = 1.0 if upward else -1.0
    sections = []
    for frac in (0.0, 0.54, 1.0):
        z = sign * span * frac
        sections.append(
            _airfoil_loop_z(
                z,
                root_chord + (tip_chord - root_chord) * frac,
                root_thickness + (tip_thickness - root_thickness) * frac,
                leading_x=-(sweep * frac),
            )
        )
    return mesh_from_geometry(repair_loft(sections), name)


def _tube_mesh(name: str, points: list[tuple[float, float, float]], radius: float):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        name,
    )


def _aabb_center_y(aabb) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][1] + aabb[1][1])


def _aabb_max_z(aabb) -> float | None:
    if aabb is None:
        return None
    return aabb[1][2]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="advertising_blimp")

    envelope_white = model.material("envelope_white", rgba=(0.95, 0.97, 0.99, 1.0))
    support_gray = model.material("support_gray", rgba=(0.57, 0.60, 0.64, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.08, 0.27, 0.63, 1.0))
    tail_red = model.material("tail_red", rgba=(0.86, 0.18, 0.15, 1.0))
    nacelle_silver = model.material("nacelle_silver", rgba=(0.82, 0.84, 0.87, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.14, 0.18, 0.24, 0.95))
    prop_black = model.material("prop_black", rgba=(0.09, 0.09, 0.10, 1.0))

    envelope_shell_mesh = _body_mesh(
        "envelope_shell",
        [
            (-27.0, 0.22, 0.22),
            (-24.0, 2.30, 2.40),
            (-18.0, 5.10, 5.30),
            (-8.0, 6.75, 6.95),
            (2.0, 7.10, 7.30),
            (12.0, 6.30, 6.50),
            (20.0, 4.00, 4.15),
            (25.0, 1.55, 1.60),
            (27.0, 0.18, 0.18),
        ],
        exponent=2.05,
        count=44,
        top_scale=1.03,
        bottom_scale=0.90,
    )
    gondola_body_mesh = _body_mesh(
        "gondola_body",
        [
            (-4.25, 0.18, 0.18),
            (-3.40, 0.85, 0.88),
            (-1.40, 1.20, 1.18),
            (0.80, 1.35, 1.28),
            (2.60, 1.15, 1.12),
            (4.25, 0.14, 0.18),
        ],
        exponent=3.2,
        count=30,
        z_offset=-1.22,
        top_scale=0.90,
        bottom_scale=1.06,
    )
    nacelle_shell_mesh = _body_mesh(
        "nacelle_shell",
        [
            (-1.95, 0.18, 0.18),
            (-1.20, 0.52, 0.56),
            (0.00, 0.76, 0.82),
            (1.05, 0.68, 0.73),
            (1.85, 0.40, 0.44),
            (2.15, 0.12, 0.14),
        ],
        exponent=2.3,
        count=28,
        z_offset=-0.56,
        top_scale=0.94,
        bottom_scale=1.00,
    )
    left_stabilizer_mesh = _horizontal_surface_mesh(
        "left_stabilizer",
        span=4.20,
        root_chord=3.15,
        tip_chord=1.50,
        root_thickness=0.42,
        tip_thickness=0.13,
        sweep=0.95,
        side_sign=1.0,
    )
    right_stabilizer_mesh = _horizontal_surface_mesh(
        "right_stabilizer",
        span=4.20,
        root_chord=3.15,
        tip_chord=1.50,
        root_thickness=0.42,
        tip_thickness=0.13,
        sweep=0.95,
        side_sign=-1.0,
    )
    left_elevator_mesh = _horizontal_surface_mesh(
        "left_elevator_panel",
        span=3.85,
        root_chord=1.85,
        tip_chord=0.92,
        root_thickness=0.22,
        tip_thickness=0.08,
        sweep=0.46,
        side_sign=1.0,
    )
    right_elevator_mesh = _horizontal_surface_mesh(
        "right_elevator_panel",
        span=3.85,
        root_chord=1.85,
        tip_chord=0.92,
        root_thickness=0.22,
        tip_thickness=0.08,
        sweep=0.46,
        side_sign=-1.0,
    )
    top_fin_mesh = _vertical_surface_mesh(
        "top_fin",
        span=4.50,
        root_chord=3.00,
        tip_chord=1.55,
        root_thickness=0.45,
        tip_thickness=0.14,
        sweep=0.82,
        upward=True,
    )
    bottom_fin_mesh = _vertical_surface_mesh(
        "bottom_fin",
        span=3.40,
        root_chord=2.75,
        tip_chord=1.45,
        root_thickness=0.36,
        tip_thickness=0.12,
        sweep=0.58,
        upward=False,
    )
    rudder_mesh = _vertical_surface_mesh(
        "rudder_panel",
        span=4.05,
        root_chord=1.72,
        tip_chord=0.95,
        root_thickness=0.23,
        tip_thickness=0.08,
        sweep=0.42,
        upward=True,
    )
    propeller_mesh = mesh_from_geometry(
        FanRotorGeometry(
            1.12,
            0.18,
            4,
            thickness=0.11,
            blade_pitch_deg=26.0,
            blade_sweep_deg=18.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.10),
            hub=FanRotorHub(style="spinner", bore_diameter=0.05),
        ),
        "propeller_rotor",
    )

    envelope = model.part("envelope")
    envelope.visual(envelope_shell_mesh, material=envelope_white, name="envelope_shell")
    envelope.visual(
        Box((9.00, 0.90, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, -8.775)),
        material=support_gray,
        name="keel_beam",
    )
    envelope.visual(
        Box((3.20, 1.10, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, -8.45)),
        material=accent_blue,
        name="keel_fairing",
    )
    for name, points in (
        (
            "keel_strut_front_left",
            [(-3.2, -1.0, -6.25), (-2.9, -0.70, -7.20), (-2.4, -0.34, -8.50)],
        ),
        (
            "keel_strut_front_right",
            [(-3.2, 1.0, -6.25), (-2.9, 0.70, -7.20), (-2.4, 0.34, -8.50)],
        ),
        (
            "keel_strut_rear_left",
            [(3.2, -1.0, -6.25), (2.9, -0.70, -7.20), (2.4, -0.34, -8.50)],
        ),
        (
            "keel_strut_rear_right",
            [(3.2, 1.0, -6.25), (2.9, 0.70, -7.20), (2.4, 0.34, -8.50)],
        ),
    ):
        envelope.visual(_tube_mesh(name, points, 0.11), material=support_gray, name=name)
    envelope.visual(
        top_fin_mesh,
        origin=Origin(xyz=(-21.25, 0.0, 2.85)),
        material=accent_blue,
        name="top_fin",
    )
    envelope.visual(
        bottom_fin_mesh,
        origin=Origin(xyz=(-21.00, 0.0, -2.70)),
        material=accent_blue,
        name="bottom_fin",
    )
    envelope.visual(
        left_stabilizer_mesh,
        origin=Origin(xyz=(-20.75, 3.05, 0.60)),
        material=accent_blue,
        name="left_stabilizer",
    )
    envelope.visual(
        right_stabilizer_mesh,
        origin=Origin(xyz=(-20.75, -3.05, 0.60)),
        material=accent_blue,
        name="right_stabilizer",
    )
    envelope.visual(
        Box((0.06, 0.72, 0.14)),
        origin=Origin(xyz=(-23.837, 3.42, 0.60)),
        material=support_gray,
        name="left_elevator_hinge_tab",
    )
    envelope.visual(
        Box((0.06, 0.72, 0.14)),
        origin=Origin(xyz=(-23.837, -3.42, 0.60)),
        material=support_gray,
        name="right_elevator_hinge_tab",
    )
    envelope.visual(
        Box((0.062, 0.14, 0.92)),
        origin=Origin(xyz=(-24.190, 0.03, 3.42)),
        material=support_gray,
        name="rudder_hinge_tab",
    )
    envelope.inertial = Inertial.from_geometry(
        Box((54.0, 14.2, 14.6)),
        mass=4200.0,
    )

    gondola = model.part("gondola")
    gondola.visual(gondola_body_mesh, material=accent_blue, name="gondola_body")
    gondola.visual(
        Box((2.90, 1.05, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=support_gray,
        name="roof_mount",
    )
    gondola.visual(
        Box((1.55, 0.14, 0.34)),
        origin=Origin(xyz=(2.10, 0.0, -0.24)),
        material=glass_dark,
        name="windscreen",
    )
    gondola.visual(
        Box((4.40, 0.08, 0.68)),
        origin=Origin(xyz=(0.20, 1.36, -1.00)),
        material=glass_dark,
        name="left_window_strip",
    )
    gondola.visual(
        Box((4.40, 0.08, 0.68)),
        origin=Origin(xyz=(0.20, -1.36, -1.00)),
        material=glass_dark,
        name="right_window_strip",
    )
    for name, points in (
        (
            "left_pylon_main",
            [(0.55, 1.42, -0.95), (0.60, 2.30, -0.92), (0.60, 3.35, -0.90), (0.60, 4.10, -0.90)],
        ),
        (
            "left_pylon_front_brace",
            [(2.10, 1.12, -0.26), (1.80, 2.10, -0.42), (1.15, 3.05, -0.70), (0.52, 3.62, -0.90)],
        ),
        (
            "left_pylon_rear_brace",
            [(-1.35, 1.12, -1.34), (-0.76, 2.00, -1.14), (-0.05, 3.08, -0.98), (0.42, 3.58, -0.90)],
        ),
        (
            "right_pylon_main",
            [(0.55, -1.42, -0.95), (0.60, -2.30, -0.92), (0.60, -3.35, -0.90), (0.60, -4.10, -0.90)],
        ),
        (
            "right_pylon_front_brace",
            [(2.10, -1.12, -0.26), (1.80, -2.10, -0.42), (1.15, -3.05, -0.70), (0.52, -3.62, -0.90)],
        ),
        (
            "right_pylon_rear_brace",
            [(-1.35, -1.12, -1.34), (-0.76, -2.00, -1.14), (-0.05, -3.08, -0.98), (0.42, -3.58, -0.90)],
        ),
    ):
        gondola.visual(_tube_mesh(name, points, 0.11 if "main" in name else 0.08), material=support_gray, name=name)
    gondola.visual(
        Cylinder(radius=0.14, length=0.26),
        origin=Origin(xyz=(0.60, 4.23, -0.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=support_gray,
        name="left_pylon_end",
    )
    gondola.visual(
        Box((0.22, 2.92, 0.18)),
        origin=Origin(xyz=(0.60, 2.88, -0.90)),
        material=support_gray,
        name="left_pylon_spine",
    )
    gondola.visual(
        Box((0.56, 1.04, 0.34)),
        origin=Origin(xyz=(1.14, 1.74, -0.80)),
        material=support_gray,
        name="left_pylon_root",
    )
    gondola.visual(
        Box((0.24, 0.78, 0.24)),
        origin=Origin(xyz=(0.58, 3.96, -0.90)),
        material=support_gray,
        name="left_pylon_head",
    )
    gondola.visual(
        Box((0.34, 3.05, 0.26)),
        origin=Origin(xyz=(0.76, 2.83, -0.88)),
        material=support_gray,
        name="left_pylon_fairing",
    )
    gondola.visual(
        Cylinder(radius=0.14, length=0.26),
        origin=Origin(xyz=(0.60, -4.23, -0.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=support_gray,
        name="right_pylon_end",
    )
    gondola.visual(
        Box((0.22, 2.92, 0.18)),
        origin=Origin(xyz=(0.60, -2.88, -0.90)),
        material=support_gray,
        name="right_pylon_spine",
    )
    gondola.visual(
        Box((0.56, 1.04, 0.34)),
        origin=Origin(xyz=(1.14, -1.74, -0.80)),
        material=support_gray,
        name="right_pylon_root",
    )
    gondola.visual(
        Box((0.24, 0.78, 0.24)),
        origin=Origin(xyz=(0.58, -3.96, -0.90)),
        material=support_gray,
        name="right_pylon_head",
    )
    gondola.visual(
        Box((0.34, 3.05, 0.26)),
        origin=Origin(xyz=(0.76, -2.83, -0.88)),
        material=support_gray,
        name="right_pylon_fairing",
    )
    gondola.inertial = Inertial.from_geometry(
        Box((8.5, 2.9, 2.7)),
        mass=680.0,
        origin=Origin(xyz=(0.0, 0.0, -1.10)),
    )

    left_nacelle = model.part("left_nacelle")
    left_nacelle.visual(
        nacelle_shell_mesh,
        origin=Origin(xyz=(0.0, 0.60, 0.0)),
        material=nacelle_silver,
        name="nacelle_shell",
    )
    left_nacelle.visual(
        Cylinder(radius=0.18, length=0.50),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=support_gray,
        name="pivot_collar",
    )
    left_nacelle.visual(
        Box((0.56, 0.38, 0.30)),
        origin=Origin(xyz=(0.10, 0.20, -0.16)),
        material=support_gray,
        name="pivot_block",
    )
    left_nacelle.inertial = Inertial.from_geometry(
        Box((4.4, 1.6, 1.8)),
        mass=180.0,
        origin=Origin(xyz=(0.20, 0.55, -0.55)),
    )

    right_nacelle = model.part("right_nacelle")
    right_nacelle.visual(
        nacelle_shell_mesh,
        origin=Origin(xyz=(0.0, -0.60, 0.0)),
        material=nacelle_silver,
        name="nacelle_shell",
    )
    right_nacelle.visual(
        Cylinder(radius=0.18, length=0.50),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=support_gray,
        name="pivot_collar",
    )
    right_nacelle.visual(
        Box((0.56, 0.38, 0.30)),
        origin=Origin(xyz=(0.10, -0.20, -0.16)),
        material=support_gray,
        name="pivot_block",
    )
    right_nacelle.inertial = Inertial.from_geometry(
        Box((4.4, 1.6, 1.8)),
        mass=180.0,
        origin=Origin(xyz=(0.20, -0.55, -0.55)),
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        propeller_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="rotor",
    )
    left_propeller.inertial = Inertial.from_geometry(Cylinder(radius=1.15, length=0.12), mass=18.0)

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        propeller_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="rotor",
    )
    right_propeller.inertial = Inertial.from_geometry(Cylinder(radius=1.15, length=0.12), mass=18.0)

    rudder = model.part("rudder")
    rudder.visual(rudder_mesh, material=tail_red, name="rudder_panel")
    rudder.inertial = Inertial.from_geometry(
        Box((1.9, 0.26, 4.2)),
        mass=55.0,
        origin=Origin(xyz=(-0.85, 0.0, 2.0)),
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(left_elevator_mesh, material=tail_red, name="elevator_panel")
    left_elevator.inertial = Inertial.from_geometry(
        Box((1.9, 3.9, 0.24)),
        mass=42.0,
        origin=Origin(xyz=(-0.85, 1.9, 0.0)),
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(right_elevator_mesh, material=tail_red, name="elevator_panel")
    right_elevator.inertial = Inertial.from_geometry(
        Box((1.9, 3.9, 0.24)),
        mass=42.0,
        origin=Origin(xyz=(-0.85, -1.9, 0.0)),
    )

    model.articulation(
        "envelope_to_gondola",
        ArticulationType.FIXED,
        parent=envelope,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -9.05)),
    )
    model.articulation(
        "gondola_to_left_nacelle",
        ArticulationType.REVOLUTE,
        parent=gondola,
        child=left_nacelle,
        origin=Origin(xyz=(0.60, 4.61, -0.90)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.80,
            lower=math.radians(-48.0),
            upper=math.radians(42.0),
        ),
    )
    model.articulation(
        "gondola_to_right_nacelle",
        ArticulationType.REVOLUTE,
        parent=gondola,
        child=right_nacelle,
        origin=Origin(xyz=(0.60, -4.61, -0.90)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1500.0,
            velocity=0.80,
            lower=math.radians(-48.0),
            upper=math.radians(42.0),
        ),
    )
    model.articulation(
        "left_nacelle_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=left_nacelle,
        child=left_propeller,
        origin=Origin(xyz=(2.218, 0.60, -0.56)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=35.0),
    )
    model.articulation(
        "right_nacelle_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=right_nacelle,
        child=right_propeller,
        origin=Origin(xyz=(2.218, -0.60, -0.56)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=35.0),
    )
    model.articulation(
        "envelope_to_rudder",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=rudder,
        origin=Origin(xyz=(-24.219, 0.0, 2.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.90,
            lower=math.radians(-28.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "envelope_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=left_elevator,
        origin=Origin(xyz=(-23.867, 3.15, 0.60)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.90,
            lower=math.radians(-22.0),
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "envelope_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=envelope,
        child=right_elevator,
        origin=Origin(xyz=(-23.867, -3.15, 0.60)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.90,
            lower=math.radians(-22.0),
            upper=math.radians(18.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    envelope = object_model.get_part("envelope")
    gondola = object_model.get_part("gondola")
    left_nacelle = object_model.get_part("left_nacelle")
    right_nacelle = object_model.get_part("right_nacelle")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")

    left_vector = object_model.get_articulation("gondola_to_left_nacelle")
    right_vector = object_model.get_articulation("gondola_to_right_nacelle")
    rudder_hinge = object_model.get_articulation("envelope_to_rudder")
    left_elevator_hinge = object_model.get_articulation("envelope_to_left_elevator")
    right_elevator_hinge = object_model.get_articulation("envelope_to_right_elevator")

    ctx.expect_contact(
        gondola,
        envelope,
        elem_a="roof_mount",
        elem_b="keel_beam",
        name="gondola roof mounts into the keel beam",
    )
    ctx.expect_gap(
        envelope,
        gondola,
        axis="z",
        positive_elem="envelope_shell",
        negative_elem="gondola_body",
        min_gap=0.90,
        max_gap=2.60,
        name="gondola hangs visibly below the envelope shell",
    )
    ctx.expect_contact(
        left_nacelle,
        gondola,
        elem_a="pivot_collar",
        elem_b="left_pylon_end",
        name="left nacelle is carried by the left pylon",
    )
    ctx.expect_contact(
        right_nacelle,
        gondola,
        elem_a="pivot_collar",
        elem_b="right_pylon_end",
        name="right nacelle is carried by the right pylon",
    )
    ctx.expect_contact(
        left_elevator,
        envelope,
        elem_a="elevator_panel",
        elem_b="left_elevator_hinge_tab",
        name="left elevator closes onto its hinge tab",
    )
    ctx.expect_contact(
        right_elevator,
        envelope,
        elem_a="elevator_panel",
        elem_b="right_elevator_hinge_tab",
        name="right elevator closes onto its hinge tab",
    )
    ctx.expect_contact(
        rudder,
        envelope,
        elem_a="rudder_panel",
        elem_b="rudder_hinge_tab",
        name="rudder closes onto its hinge tab",
    )
    ctx.expect_origin_gap(
        left_nacelle,
        gondola,
        axis="y",
        min_gap=4.20,
        name="left nacelle stays well outboard of the gondola",
    )
    ctx.expect_origin_gap(
        gondola,
        right_nacelle,
        axis="y",
        min_gap=4.20,
        name="right nacelle stays well outboard of the gondola",
    )
    ctx.allow_overlap(
        left_nacelle,
        left_propeller,
        elem_a="nacelle_shell",
        elem_b="rotor",
        reason="The propeller spinner is intentionally seated into the simplified nacelle nose lip.",
    )
    ctx.allow_overlap(
        right_nacelle,
        right_propeller,
        elem_a="nacelle_shell",
        elem_b="rotor",
        reason="The propeller spinner is intentionally seated into the simplified nacelle nose lip.",
    )

    left_rest = ctx.part_world_position(left_propeller)
    right_rest = ctx.part_world_position(right_propeller)
    left_upper = left_vector.motion_limits.upper if left_vector.motion_limits is not None else None
    right_upper = right_vector.motion_limits.upper if right_vector.motion_limits is not None else None
    if left_rest is not None and left_upper is not None:
        with ctx.pose({left_vector: left_upper}):
            left_vectored = ctx.part_world_position(left_propeller)
        ctx.check(
            "left nacelle vectors upward",
            left_vectored is not None and left_vectored[2] > left_rest[2] + 0.30,
            details=f"rest={left_rest}, vectored={left_vectored}",
        )
    if right_rest is not None and right_upper is not None:
        with ctx.pose({right_vector: right_upper}):
            right_vectored = ctx.part_world_position(right_propeller)
        ctx.check(
            "right nacelle vectors upward",
            right_vectored is not None and right_vectored[2] > right_rest[2] + 0.30,
            details=f"rest={right_rest}, vectored={right_vectored}",
        )

    rudder_upper = rudder_hinge.motion_limits.upper if rudder_hinge.motion_limits is not None else None
    if rudder_upper is not None:
        rudder_rest = ctx.part_element_world_aabb(rudder, elem="rudder_panel")
        with ctx.pose({rudder_hinge: rudder_upper}):
            rudder_deflected = ctx.part_element_world_aabb(rudder, elem="rudder_panel")
        rest_y = _aabb_center_y(rudder_rest)
        deflected_y = _aabb_center_y(rudder_deflected)
        ctx.check(
            "rudder swings laterally",
            rest_y is not None and deflected_y is not None and deflected_y < rest_y - 0.18,
            details=f"rest_center_y={rest_y}, deflected_center_y={deflected_y}",
        )

    elevator_pose = {}
    if left_elevator_hinge.motion_limits is not None and left_elevator_hinge.motion_limits.upper is not None:
        elevator_pose[left_elevator_hinge] = left_elevator_hinge.motion_limits.upper
    if right_elevator_hinge.motion_limits is not None and right_elevator_hinge.motion_limits.upper is not None:
        elevator_pose[right_elevator_hinge] = right_elevator_hinge.motion_limits.upper
    if elevator_pose:
        left_rest_aabb = ctx.part_element_world_aabb(left_elevator, elem="elevator_panel")
        right_rest_aabb = ctx.part_element_world_aabb(right_elevator, elem="elevator_panel")
        with ctx.pose(elevator_pose):
            left_up_aabb = ctx.part_element_world_aabb(left_elevator, elem="elevator_panel")
            right_up_aabb = ctx.part_element_world_aabb(right_elevator, elem="elevator_panel")
        ctx.check(
            "left elevator raises trailing edge",
            _aabb_max_z(left_rest_aabb) is not None
            and _aabb_max_z(left_up_aabb) is not None
            and _aabb_max_z(left_up_aabb) > _aabb_max_z(left_rest_aabb) + 0.18,
            details=f"rest_max_z={_aabb_max_z(left_rest_aabb)}, up_max_z={_aabb_max_z(left_up_aabb)}",
        )
        ctx.check(
            "right elevator raises trailing edge",
            _aabb_max_z(right_rest_aabb) is not None
            and _aabb_max_z(right_up_aabb) is not None
            and _aabb_max_z(right_up_aabb) > _aabb_max_z(right_rest_aabb) + 0.18,
            details=f"rest_max_z={_aabb_max_z(right_rest_aabb)}, up_max_z={_aabb_max_z(right_up_aabb)}",
        )

    return ctx.report()


object_model = build_object_model()
