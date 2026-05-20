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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _horizontal_surface_section(
    y_pos: float,
    front_x: float,
    rear_x: float,
    center_z: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    chord = front_x - rear_x
    return [
        (front_x, y_pos, center_z),
        (front_x - 0.16 * chord, y_pos, center_z + 0.50 * thickness),
        (front_x - 0.66 * chord, y_pos, center_z + 0.18 * thickness),
        (rear_x, y_pos, center_z),
        (front_x - 0.66 * chord, y_pos, center_z - 0.18 * thickness),
        (front_x - 0.16 * chord, y_pos, center_z - 0.50 * thickness),
    ]


def _vertical_surface_section(
    z_pos: float,
    front_x: float,
    rear_x: float,
    center_y: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    chord = front_x - rear_x
    return [
        (front_x, center_y, z_pos),
        (front_x - 0.16 * chord, center_y + 0.50 * thickness, z_pos),
        (front_x - 0.66 * chord, center_y + 0.18 * thickness, z_pos),
        (rear_x, center_y, z_pos),
        (front_x - 0.66 * chord, center_y - 0.18 * thickness, z_pos),
        (front_x - 0.16 * chord, center_y - 0.50 * thickness, z_pos),
    ]


def _envelope_geometry():
    profile = [
        (0.0, -11.6),
        (0.45, -11.2),
        (1.05, -10.5),
        (1.75, -9.5),
        (2.45, -8.0),
        (3.00, -6.0),
        (3.25, -3.8),
        (3.35, -1.0),
        (3.34, 2.0),
        (3.15, 5.2),
        (2.75, 8.4),
        (2.00, 10.4),
        (1.10, 11.4),
        (0.35, 11.9),
        (0.0, 12.2),
    ]
    return LatheGeometry(profile, segments=112).rotate_y(math.pi / 2.0)


def _streamlined_body(length: float, radius: float):
    half_length = 0.5 * length
    profile = [
        (0.0, -half_length),
        (0.32 * radius, -0.92 * half_length),
        (0.68 * radius, -0.74 * half_length),
        (0.92 * radius, -0.42 * half_length),
        (radius, -0.05 * half_length),
        (0.98 * radius, 0.30 * half_length),
        (0.82 * radius, 0.66 * half_length),
        (0.42 * radius, 0.90 * half_length),
        (0.0, half_length),
    ]
    return LatheGeometry(profile, segments=72).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_sign_blimp")

    envelope_white = model.material("envelope_white", rgba=(0.89, 0.90, 0.86, 1.0))
    hull_gray = model.material("hull_gray", rgba=(0.46, 0.48, 0.50, 1.0))
    frame_dark = model.material("frame_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    sign_glow = model.material("sign_glow", rgba=(0.96, 0.80, 0.28, 1.0))
    accent_red = model.material("accent_red", rgba=(0.63, 0.16, 0.15, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.13, 0.17, 0.22, 1.0))
    prop_black = model.material("prop_black", rgba=(0.11, 0.11, 0.12, 1.0))
    metal = model.material("metal", rgba=(0.68, 0.70, 0.73, 1.0))

    airframe = model.part("airframe")

    envelope_mesh = _mesh("envelope", _envelope_geometry())
    gondola_mesh = _mesh("gondola_shell", _streamlined_body(4.2, 0.52))
    nacelle_mesh = _mesh("side_nacelle", _streamlined_body(2.2, 0.42))
    engine_pod_mesh = _mesh("engine_pod", _streamlined_body(1.10, 0.18))

    airframe.visual(envelope_mesh, material=envelope_white, name="envelope")

    airframe.visual(
        gondola_mesh,
        origin=Origin(xyz=(0.35, 0.0, -3.38)),
        material=hull_gray,
        name="gondola_shell",
    )
    airframe.visual(
        Box((1.45, 0.96, 0.42)),
        origin=Origin(xyz=(0.95, 0.0, -3.14)),
        material=glass_tint,
        name="gondola_glazing",
    )
    airframe.visual(
        Box((2.20, 1.12, 0.20)),
        origin=Origin(xyz=(0.30, 0.0, -3.80)),
        material=frame_dark,
        name="gondola_keel",
    )

    for index, point_sets in enumerate(
        [
            [(-1.16, -0.54, -2.50), (-0.96, -0.50, -2.78), (-0.70, -0.40, -3.06)],
            [(-1.16, 0.54, -2.50), (-0.96, 0.50, -2.78), (-0.70, 0.40, -3.06)],
            [(1.80, -0.56, -2.48), (1.54, -0.50, -2.78), (1.16, -0.38, -3.02)],
            [(1.80, 0.56, -2.48), (1.54, 0.50, -2.78), (1.16, 0.38, -3.02)],
        ]
    ):
        airframe.visual(
            _mesh(
                f"gondola_strut_{index}",
                tube_from_spline_points(
                    point_sets,
                    radius=0.055,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_dark,
            name=f"gondola_strut_{index}",
        )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        airframe.visual(
            Box((0.22, 0.16, 0.14)),
            origin=Origin(xyz=(-0.70, side_sign * 0.40, -3.01)),
            material=hull_gray,
            name=f"rear_gondola_mount_{side_name}",
        )
        airframe.visual(
            Box((0.26, 0.18, 0.18)),
            origin=Origin(xyz=(-1.18, side_sign * 0.54, -2.45)),
            material=envelope_white,
            name=f"rear_hull_mount_{side_name}",
        )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        nacelle_y = side_sign * 3.98
        nacelle_z = -1.72
        pivot_y = side_sign * 1.74
        pivot_z = -3.74
        airframe.visual(
            Box((8.55, 0.22, 1.82)),
            origin=Origin(xyz=(0.55, side_sign * 3.56, 0.16)),
            material=frame_dark,
            name=f"{side_name}_sign_frame",
        )
        airframe.visual(
            Box((8.08, 0.05, 1.48)),
            origin=Origin(xyz=(0.55, side_sign * 3.66, 0.16)),
            material=sign_glow,
            name=f"{side_name}_sign_face",
        )
        for mount_index, x_pos in enumerate((-3.30, -1.10, 1.20, 3.40)):
            airframe.visual(
                _mesh(
                    f"{side_name}_sign_standoff_{mount_index}",
                    tube_from_spline_points(
                        [
                            (x_pos, side_sign * 3.02, 0.14),
                            (x_pos, side_sign * 3.24, 0.16),
                            (x_pos, side_sign * 3.48, 0.16),
                        ],
                        radius=0.045,
                        samples_per_segment=8,
                        radial_segments=14,
                    ),
                ),
                material=frame_dark,
                name=f"{side_name}_sign_standoff_{mount_index}",
            )

        if side_name == "left":
            airframe.visual(
                nacelle_mesh,
                origin=Origin(xyz=(-1.05, nacelle_y, nacelle_z)),
                material=hull_gray,
                name="left_nacelle_body",
            )
            airframe.visual(
                Cylinder(radius=0.07, length=0.34),
                origin=Origin(
                    xyz=(0.20, nacelle_y, nacelle_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=metal,
                name="left_nacelle_shaft",
            )
        else:
            airframe.visual(
                nacelle_mesh,
                origin=Origin(xyz=(-1.05, nacelle_y, nacelle_z)),
                material=hull_gray,
                name="right_nacelle_body",
            )
            airframe.visual(
                Cylinder(radius=0.07, length=0.34),
                origin=Origin(
                    xyz=(0.20, nacelle_y, nacelle_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=metal,
                name="right_nacelle_shaft",
            )
        airframe.visual(
            Box((0.86, 0.16, 0.74)),
            origin=Origin(xyz=(-1.18, side_sign * 3.34, -1.42)),
            material=frame_dark,
            name=f"{side_name}_nacelle_mount",
        )
        for mount_index, points in enumerate(
            [
                [
                    (-1.46, side_sign * 2.98, -0.76),
                    (-1.34, side_sign * 3.24, -1.06),
                    (-1.22, side_sign * 3.70, -1.42),
                ],
                [
                    (-1.46, side_sign * 2.98, -1.52),
                    (-1.34, side_sign * 3.22, -1.60),
                    (-1.20, side_sign * 3.70, -1.80),
                ],
                [
                    (-0.70, side_sign * 3.00, -0.74),
                    (-0.76, side_sign * 3.28, -1.04),
                    (-0.84, side_sign * 3.72, -1.42),
                ],
                [
                    (-0.68, side_sign * 3.00, -1.50),
                    (-0.74, side_sign * 3.26, -1.60),
                    (-0.82, side_sign * 3.72, -1.80),
                ],
            ]
        ):
            airframe.visual(
                _mesh(
                    f"{side_name}_nacelle_brace_{mount_index}",
                    tube_from_spline_points(
                        points,
                        radius=0.05,
                        samples_per_segment=8,
                        radial_segments=14,
                    ),
                ),
                material=frame_dark,
                name=f"{side_name}_nacelle_brace_{mount_index}",
            )

        if side_name == "left":
            airframe.visual(
                _mesh(
                    "left_pylon_arm_a",
                    tube_from_spline_points(
                        [
                            (0.08, side_sign * 0.78, -2.92),
                            (0.10, side_sign * 1.12, -3.18),
                            (0.14, side_sign * 1.60, -3.62),
                        ],
                        radius=0.06,
                        samples_per_segment=10,
                        radial_segments=14,
                    ),
                ),
                material=frame_dark,
                name="left_pylon_arm_a",
            )
        else:
            airframe.visual(
                _mesh(
                    "right_pylon_arm_a",
                    tube_from_spline_points(
                        [
                            (0.08, side_sign * 0.78, -2.92),
                            (0.10, side_sign * 1.12, -3.18),
                            (0.14, side_sign * 1.60, -3.62),
                        ],
                        radius=0.06,
                        samples_per_segment=10,
                        radial_segments=14,
                    ),
                ),
                material=frame_dark,
                name="right_pylon_arm_a",
            )
        airframe.visual(
            _mesh(
                f"{side_name}_pylon_arm_b",
                tube_from_spline_points(
                    [
                        (0.54, side_sign * 0.78, -2.90),
                        (0.42, side_sign * 1.12, -3.14),
                        (0.28, side_sign * 1.60, -3.60),
                    ],
                    radius=0.05,
                    samples_per_segment=10,
                    radial_segments=14,
                ),
            ),
            material=frame_dark,
            name=f"{side_name}_pylon_arm_b",
        )
        airframe.visual(
            Box((0.42, 0.26, 0.18)),
            origin=Origin(xyz=(0.30, side_sign * 0.92, -2.92)),
            material=hull_gray,
            name=f"{side_name}_pylon_root",
        )
        airframe.visual(
            Box((0.22, 0.06, 0.34)),
            origin=Origin(xyz=(0.12, side_sign * 1.57, pivot_z)),
            material=frame_dark,
            name=f"{side_name}_pylon_cheek_0",
        )
        airframe.visual(
            Box((0.22, 0.06, 0.34)),
            origin=Origin(xyz=(0.12, side_sign * 1.91, pivot_z)),
            material=frame_dark,
            name=f"{side_name}_pylon_cheek_1",
        )
        airframe.visual(
            Box((0.12, 0.40, 0.14)),
            origin=Origin(xyz=(-0.01, pivot_y, pivot_z)),
            material=frame_dark,
            name=f"{side_name}_pylon_yoke",
        )

    top_fin = _mesh(
        "top_fin",
        section_loft(
            [
                _vertical_surface_section(0.10, -10.40, -11.98, 0.0, 0.36),
                _vertical_surface_section(1.15, -10.82, -12.18, 0.0, 0.26),
                _vertical_surface_section(2.35, -11.18, -12.10, 0.0, 0.16),
            ]
        ),
    )
    bottom_fin = _mesh(
        "bottom_fin",
        section_loft(
            [
                _vertical_surface_section(-0.10, -10.40, -11.98, 0.0, 0.36),
                _vertical_surface_section(-1.10, -10.82, -12.20, 0.0, 0.26),
                _vertical_surface_section(-2.05, -11.12, -11.96, 0.0, 0.18),
            ]
        ),
    )
    left_stabilizer = _mesh(
        "left_stabilizer",
        section_loft(
            [
                _horizontal_surface_section(0.45, -10.46, -11.62, 0.12, 0.28),
                _horizontal_surface_section(2.05, -10.76, -11.60, 0.10, 0.20),
                _horizontal_surface_section(3.98, -11.08, -11.66, 0.08, 0.12),
            ]
        ),
    )
    right_stabilizer = _mesh(
        "right_stabilizer",
        section_loft(
            [
                _horizontal_surface_section(-0.45, -10.46, -11.62, 0.12, 0.28),
                _horizontal_surface_section(-2.05, -10.76, -11.60, 0.10, 0.20),
                _horizontal_surface_section(-3.98, -11.08, -11.66, 0.08, 0.12),
            ]
        ),
    )
    left_end_fin = _mesh(
        "left_end_fin",
        section_loft(
            [
                _vertical_surface_section(-0.92, -11.10, -11.82, 4.02, 0.16),
                _vertical_surface_section(0.28, -11.26, -11.86, 4.02, 0.20),
                _vertical_surface_section(1.86, -11.38, -11.92, 4.02, 0.14),
            ]
        ),
    )
    right_end_fin = _mesh(
        "right_end_fin",
        section_loft(
            [
                _vertical_surface_section(-0.92, -11.10, -11.82, -4.02, 0.16),
                _vertical_surface_section(0.28, -11.26, -11.86, -4.02, 0.20),
                _vertical_surface_section(1.86, -11.38, -11.92, -4.02, 0.14),
            ]
        ),
    )

    airframe.visual(top_fin, material=envelope_white, name="top_fin")
    airframe.visual(bottom_fin, material=envelope_white, name="bottom_fin")
    airframe.visual(left_stabilizer, material=envelope_white, name="left_stabilizer")
    airframe.visual(right_stabilizer, material=envelope_white, name="right_stabilizer")
    airframe.visual(left_end_fin, material=envelope_white, name="left_end_fin")
    airframe.visual(right_end_fin, material=envelope_white, name="right_end_fin")
    airframe.visual(
        Box((0.06, 2.74, 0.06)),
        origin=Origin(xyz=(-11.61, 2.32, 0.09)),
        material=frame_dark,
        name="left_elevator_hinge_base",
    )
    airframe.visual(
        Box((0.06, 2.74, 0.06)),
        origin=Origin(xyz=(-11.61, -2.32, 0.09)),
        material=frame_dark,
        name="right_elevator_hinge_base",
    )

    propeller_mesh = _mesh(
        "propeller_rotor",
        FanRotorGeometry(
            0.88,
            0.16,
            4,
            thickness=0.16,
            blade_pitch_deg=26.0,
            blade_sweep_deg=18.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=10.0, camber=0.10),
            hub=FanRotorHub(style="spinner", bore_diameter=0.10),
        ),
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        propeller_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="rotor",
    )
    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        propeller_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="rotor",
    )

    left_engine_pod = model.part("left_engine_pod")
    left_engine_pod.visual(
        engine_pod_mesh,
        origin=Origin(xyz=(0.80, 0.0, 0.0)),
        material=accent_red,
        name="pod_shell",
    )
    left_engine_pod.visual(
        Box((0.18, 0.22, 0.52)),
        material=metal,
        name="trunnion_collar",
    )
    left_engine_pod.visual(
        Box((0.40, 0.16, 0.16)),
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        material=accent_red,
        name="pod_neck",
    )
    left_engine_pod.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(1.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_dark,
        name="exhaust_ring",
    )

    right_engine_pod = model.part("right_engine_pod")
    right_engine_pod.visual(
        engine_pod_mesh,
        origin=Origin(xyz=(0.80, 0.0, 0.0)),
        material=accent_red,
        name="pod_shell",
    )
    right_engine_pod.visual(
        Box((0.18, 0.22, 0.52)),
        material=metal,
        name="trunnion_collar",
    )
    right_engine_pod.visual(
        Box((0.40, 0.16, 0.16)),
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        material=accent_red,
        name="pod_neck",
    )
    right_engine_pod.visual(
        Cylinder(radius=0.15, length=0.08),
        origin=Origin(xyz=(1.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_dark,
        name="exhaust_ring",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        _mesh(
            "left_elevator",
            section_loft(
                [
                    _horizontal_surface_section(-1.55, 0.0, -0.86, 0.0, 0.15),
                    _horizontal_surface_section(-0.10, 0.0, -0.74, -0.01, 0.13),
                    _horizontal_surface_section(1.18, 0.0, -0.58, -0.02, 0.09),
                ]
            ),
        ),
        material=envelope_white,
        name="surface",
    )
    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        _mesh(
            "right_elevator",
            section_loft(
                [
                    _horizontal_surface_section(1.55, 0.0, -0.86, 0.0, 0.15),
                    _horizontal_surface_section(0.10, 0.0, -0.74, -0.01, 0.13),
                    _horizontal_surface_section(-1.18, 0.0, -0.58, -0.02, 0.09),
                ]
            ),
        ),
        material=envelope_white,
        name="surface",
    )
    left_rudder = model.part("left_rudder")
    left_rudder.visual(
        _mesh(
            "left_rudder",
            section_loft(
                [
                    _vertical_surface_section(-1.18, 0.0, -0.56, 0.0, 0.10),
                    _vertical_surface_section(0.00, 0.0, -0.60, 0.0, 0.12),
                    _vertical_surface_section(1.56, 0.0, -0.42, 0.0, 0.08),
                ]
            ),
        ),
        material=envelope_white,
        name="surface",
    )

    right_rudder = model.part("right_rudder")
    right_rudder.visual(
        _mesh(
            "right_rudder",
            section_loft(
                [
                    _vertical_surface_section(-1.18, 0.0, -0.56, 0.0, 0.10),
                    _vertical_surface_section(0.00, 0.0, -0.60, 0.0, 0.12),
                    _vertical_surface_section(1.56, 0.0, -0.42, 0.0, 0.08),
                ]
            ),
        ),
        material=envelope_white,
        name="surface",
    )

    model.articulation(
        "left_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=left_propeller,
        origin=Origin(xyz=(0.42, 3.98, -1.72)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=35.0),
    )
    model.articulation(
        "right_propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=right_propeller,
        origin=Origin(xyz=(0.42, -3.98, -1.72)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=35.0),
    )
    model.articulation(
        "left_engine_vector",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_engine_pod,
        origin=Origin(xyz=(0.14, 1.74, -3.74)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.95,
            upper=0.65,
        ),
    )
    model.articulation(
        "right_engine_vector",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_engine_pod,
        origin=Origin(xyz=(0.14, -1.74, -3.74)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.95,
            upper=0.65,
        ),
    )
    model.articulation(
        "left_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_elevator,
        origin=Origin(xyz=(-11.64, 2.50, 0.09)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "right_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_elevator,
        origin=Origin(xyz=(-11.64, -2.50, 0.09)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.45,
        ),
    )
    model.articulation(
        "left_rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_rudder,
        origin=Origin(xyz=(-11.92, 4.02, 0.30)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "right_rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_rudder,
        origin=Origin(xyz=(-11.92, -4.02, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    left_propeller = object_model.get_part("left_propeller")
    right_propeller = object_model.get_part("right_propeller")
    left_engine_pod = object_model.get_part("left_engine_pod")
    right_engine_pod = object_model.get_part("right_engine_pod")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")
    left_rudder = object_model.get_part("left_rudder")
    right_rudder = object_model.get_part("right_rudder")

    left_engine_vector = object_model.get_articulation("left_engine_vector")
    right_engine_vector = object_model.get_articulation("right_engine_vector")
    left_elevator_hinge = object_model.get_articulation("left_elevator_hinge")
    right_elevator_hinge = object_model.get_articulation("right_elevator_hinge")
    left_rudder_hinge = object_model.get_articulation("left_rudder_hinge")
    right_rudder_hinge = object_model.get_articulation("right_rudder_hinge")

    ctx.allow_overlap(
        left_propeller,
        airframe,
        elem_a="rotor",
        elem_b="left_nacelle_shaft",
        reason="The left propeller hub is intentionally carried on the nacelle shaft.",
    )
    ctx.allow_overlap(
        right_propeller,
        airframe,
        elem_a="rotor",
        elem_b="right_nacelle_shaft",
        reason="The right propeller hub is intentionally carried on the nacelle shaft.",
    )
    ctx.allow_overlap(
        left_engine_pod,
        airframe,
        elem_a="trunnion_collar",
        elem_b="left_pylon_arm_a",
        reason="The left pylon arm is simplified into the pivot collar support region.",
    )
    ctx.allow_overlap(
        right_engine_pod,
        airframe,
        elem_a="trunnion_collar",
        elem_b="right_pylon_arm_a",
        reason="The right pylon arm is simplified into the pivot collar support region.",
    )

    ctx.expect_overlap(
        left_propeller,
        airframe,
        axes="yz",
        elem_a="rotor",
        elem_b="left_nacelle_body",
        min_overlap=0.70,
        name="left propeller stays centered on left nacelle",
    )
    ctx.expect_overlap(
        right_propeller,
        airframe,
        axes="yz",
        elem_a="rotor",
        elem_b="right_nacelle_body",
        min_overlap=0.70,
        name="right propeller stays centered on right nacelle",
    )
    ctx.expect_gap(
        left_propeller,
        airframe,
        axis="x",
        positive_elem="rotor",
        negative_elem="left_nacelle_body",
        min_gap=0.04,
        max_gap=0.55,
        name="left propeller clears the nacelle nose",
    )
    ctx.expect_gap(
        right_propeller,
        airframe,
        axis="x",
        positive_elem="rotor",
        negative_elem="right_nacelle_body",
        min_gap=0.04,
        max_gap=0.55,
        name="right propeller clears the nacelle nose",
    )

    def _elem_bounds(part, elem):
        return ctx.part_element_world_aabb(part, elem=elem)

    left_collar = _elem_bounds(left_engine_pod, "trunnion_collar")
    left_cheek_a = _elem_bounds(airframe, "left_pylon_cheek_0")
    left_cheek_b = _elem_bounds(airframe, "left_pylon_cheek_1")
    right_collar = _elem_bounds(right_engine_pod, "trunnion_collar")
    right_cheek_a = _elem_bounds(airframe, "right_pylon_cheek_0")
    right_cheek_b = _elem_bounds(airframe, "right_pylon_cheek_1")

    ctx.check(
        "left engine collar sits between left yoke cheeks",
        left_collar is not None
        and left_cheek_a is not None
        and left_cheek_b is not None
        and left_cheek_a[1][1] <= left_collar[0][1] <= left_collar[1][1] <= left_cheek_b[0][1],
        details=f"collar={left_collar}, cheek_a={left_cheek_a}, cheek_b={left_cheek_b}",
    )
    ctx.check(
        "right engine collar sits between right yoke cheeks",
        right_collar is not None
        and right_cheek_b is not None
        and right_cheek_a is not None
        and right_cheek_b[1][1] <= right_collar[0][1] <= right_collar[1][1] <= right_cheek_a[0][1],
        details=f"collar={right_collar}, cheek_a={right_cheek_a}, cheek_b={right_cheek_b}",
    )

    left_pod_rest = _elem_bounds(left_engine_pod, "pod_shell")
    right_pod_rest = _elem_bounds(right_engine_pod, "pod_shell")
    left_elevator_rest = _elem_bounds(left_elevator, "surface")
    right_elevator_rest = _elem_bounds(right_elevator, "surface")
    left_rudder_rest = _elem_bounds(left_rudder, "surface")
    right_rudder_rest = _elem_bounds(right_rudder, "surface")

    with ctx.pose(
        {
            left_engine_vector: 0.55,
            right_engine_vector: 0.55,
            left_elevator_hinge: 0.35,
            right_elevator_hinge: 0.35,
            left_rudder_hinge: 0.45,
            right_rudder_hinge: 0.45,
        }
    ):
        left_pod_up = _elem_bounds(left_engine_pod, "pod_shell")
        right_pod_up = _elem_bounds(right_engine_pod, "pod_shell")
        left_elevator_up = _elem_bounds(left_elevator, "surface")
        right_elevator_up = _elem_bounds(right_elevator, "surface")
        left_rudder_up = _elem_bounds(left_rudder, "surface")
        right_rudder_up = _elem_bounds(right_rudder, "surface")

    ctx.check(
        "left engine pod vectors upward",
        left_pod_rest is not None
        and left_pod_up is not None
        and left_pod_up[1][2] > left_pod_rest[1][2] + 0.08,
        details=f"rest={left_pod_rest}, up={left_pod_up}",
    )
    ctx.check(
        "right engine pod vectors upward",
        right_pod_rest is not None
        and right_pod_up is not None
        and right_pod_up[1][2] > right_pod_rest[1][2] + 0.08,
        details=f"rest={right_pod_rest}, up={right_pod_up}",
    )
    ctx.check(
        "left elevator trailing edge lifts",
        left_elevator_rest is not None
        and left_elevator_up is not None
        and left_elevator_up[1][2] > left_elevator_rest[1][2] + 0.10,
        details=f"rest={left_elevator_rest}, up={left_elevator_up}",
    )
    ctx.check(
        "right elevator trailing edge lifts",
        right_elevator_rest is not None
        and right_elevator_up is not None
        and right_elevator_up[1][2] > right_elevator_rest[1][2] + 0.10,
        details=f"rest={right_elevator_rest}, up={right_elevator_up}",
    )
    ctx.check(
        "left rudder deflects outward",
        left_rudder_rest is not None
        and left_rudder_up is not None
        and left_rudder_up[1][1] > left_rudder_rest[1][1] + 0.08,
        details=f"rest={left_rudder_rest}, deflected={left_rudder_up}",
    )
    ctx.check(
        "right rudder deflects outward",
        right_rudder_rest is not None
        and right_rudder_up is not None
        and right_rudder_up[0][1] < right_rudder_rest[0][1] - 0.08,
        details=f"rest={right_rudder_rest}, deflected={right_rudder_up}",
    )

    return ctx.report()


object_model = build_object_model()
