from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    AssetContext,
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
    superellipse_side_loft,
    superellipse_profile,
    tube_from_spline_points,
    wire_from_points,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


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


def _wheel_visuals(
    part,
    mesh_prefix: str,
    *,
    tire_radius: float,
    tire_width: float,
    hub_width: float,
    wheel_steel,
    dark_steel,
    rubber,
    side_sign: float = 0.0,
    add_knuckle: bool = False,
) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    half_width = tire_width * 0.5
    tire_profile = [
        (tire_radius * 0.50, -half_width * 0.96),
        (tire_radius * 0.72, -half_width * 0.98),
        (tire_radius * 0.88, -half_width * 0.84),
        (tire_radius * 0.97, -half_width * 0.52),
        (tire_radius, -half_width * 0.16),
        (tire_radius, half_width * 0.16),
        (tire_radius * 0.97, half_width * 0.52),
        (tire_radius * 0.88, half_width * 0.84),
        (tire_radius * 0.72, half_width * 0.98),
        (tire_radius * 0.50, half_width * 0.96),
        (tire_radius * 0.42, half_width * 0.34),
        (tire_radius * 0.39, 0.0),
        (tire_radius * 0.42, -half_width * 0.34),
        (tire_radius * 0.50, -half_width * 0.96),
    ]
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire.obj",
        LatheGeometry(tire_profile, segments=64).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber)

    rim_outer = superellipse_profile(
        tire_radius * 1.08,
        tire_radius * 1.08,
        exponent=2.0,
        segments=48,
    )
    window_profile = rounded_rect_profile(
        tire_radius * 0.16,
        tire_radius * 0.28,
        tire_radius * 0.040,
        corner_segments=6,
    )
    hole_profiles = [superellipse_profile(tire_radius * 0.30, tire_radius * 0.30, exponent=2.0, segments=24)]
    for spoke_index in range(5):
        angle = 2.0 * pi * spoke_index / 5.0
        hole_profiles.append(
            _transform_profile(
                window_profile,
                dx=cos(angle) * tire_radius * 0.29,
                dy=sin(angle) * tire_radius * 0.29,
                angle=angle,
            )
        )
    rim_face = _save_mesh(
        f"{mesh_prefix}_rim_face.obj",
        ExtrudeWithHolesGeometry(
            rim_outer,
            hole_profiles,
            height=tire_width * 0.16,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    part.visual(rim_face, origin=Origin(xyz=(tire_width * 0.20, 0.0, 0.0)), material=wheel_steel)
    part.visual(rim_face, origin=Origin(xyz=(-tire_width * 0.20, 0.0, 0.0)), material=wheel_steel)
    part.visual(Cylinder(radius=tire_radius * 0.50, length=tire_width * 0.44), origin=spin_origin, material=dark_steel)
    part.visual(Cylinder(radius=tire_radius * 0.28, length=hub_width), origin=spin_origin, material=dark_steel)
    part.visual(Cylinder(radius=tire_radius * 0.11, length=hub_width * 0.92), origin=spin_origin, material=wheel_steel)
    if add_knuckle:
        arm_x = -0.075 * side_sign
        part.visual(Cylinder(radius=0.030, length=0.24), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=dark_steel)
        part.visual(Box((0.14, 0.055, 0.055)), origin=Origin(xyz=(arm_x, 0.0, 0.0)), material=dark_steel)
        part.visual(Box((0.065, 0.13, 0.04)), origin=Origin(xyz=(arm_x * 0.70, -0.05, 0.085)), material=dark_steel)
        part.visual(
            Cylinder(radius=tire_radius * 0.42, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.06), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_steel,
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.38, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.075), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="atv_quad_bike", assets=ASSETS)

    body_red = model.material("body_red", rgba=(0.66, 0.10, 0.09, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.18, 0.19, 0.18, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    shock_silver = model.material("shock_silver", rgba=(0.78, 0.79, 0.82, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.14, 0.14, 0.15, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((1.20, 1.60, 0.55)),
        mass=230.0,
        origin=Origin(xyz=(0.0, 0.0, 0.57)),
    )

    body_shell = superellipse_side_loft(
        [
            (0.58, 0.50, 0.66, 0.22),
            (0.42, 0.44, 0.58, 0.38),
            (0.26, 0.43, 0.64, 0.40),
            (0.08, 0.44, 0.70, 0.34),
            (-0.10, 0.44, 0.67, 0.25),
            (-0.28, 0.43, 0.57, 0.22),
        ],
        exponents=2.6,
        segments=56,
    )
    chassis.visual(_save_mesh("atv_body_shell.obj", body_shell), material=body_red)

    front_bodywork = model.part("front_bodywork")
    front_bodywork.inertial = Inertial.from_geometry(
        Box((1.02, 0.42, 0.16)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.02, 0.03)),
    )
    front_fender_sections = [
        (0.44, 0.63, 0.70, 0.18),
        (0.58, 0.64, 0.75, 0.27),
        (0.74, 0.63, 0.69, 0.18),
    ]
    front_left_fender = superellipse_side_loft(front_fender_sections, exponents=2.2, segments=42).translate(0.44, -0.58, -0.70)
    front_right_fender = superellipse_side_loft(front_fender_sections, exponents=2.2, segments=42).translate(-0.44, -0.58, -0.70)
    front_bodywork.visual(_save_mesh("atv_front_left_fender.obj", front_left_fender), material=body_red)
    front_bodywork.visual(_save_mesh("atv_front_right_fender.obj", front_right_fender), material=body_red)
    front_bodywork.visual(Box((0.34, 0.14, 0.06)), origin=Origin(xyz=(0.0, 0.02, 0.00)), material=body_red)
    front_bodywork.visual(Box((0.24, 0.12, 0.06)), origin=Origin(xyz=(0.22, 0.04, 0.01)), material=body_red)
    front_bodywork.visual(Box((0.24, 0.12, 0.06)), origin=Origin(xyz=(-0.22, 0.04, 0.01)), material=body_red)
    front_bodywork.visual(Box((0.18, 0.10, 0.06)), origin=Origin(xyz=(0.0, -0.02, 0.01)), material=dark_steel)
    front_bodywork.visual(Box((0.26, 0.08, 0.03)), origin=Origin(xyz=(0.0, -0.06, 0.03)), material=black_plastic)

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.38, 0.40, 0.14)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )
    seat_geom = superellipse_side_loft(
        [
            (-0.02, 0.69, 0.79, 0.29),
            (-0.18, 0.68, 0.82, 0.34),
            (-0.34, 0.67, 0.78, 0.30),
        ],
        exponents=2.0,
        segments=44,
    )
    seat.visual(_save_mesh("atv_seat.obj", seat_geom.translate(0.0, 0.18, -0.74)), material=seat_vinyl)
    seat.visual(Box((0.28, 0.26, 0.04)), origin=Origin(xyz=(0.0, -0.02, -0.01)), material=black_plastic)

    rear_bodywork = model.part("rear_bodywork")
    rear_bodywork.inertial = Inertial.from_geometry(
        Box((1.02, 0.52, 0.18)),
        mass=14.0,
        origin=Origin(xyz=(0.0, -0.04, 0.04)),
    )
    rear_fender_sections = [
        (-0.74, 0.60, 0.69, 0.22),
        (-0.55, 0.61, 0.76, 0.35),
        (-0.34, 0.59, 0.72, 0.26),
    ]
    rear_left_fender = superellipse_side_loft(rear_fender_sections, exponents=2.3, segments=42).translate(0.46, 0.50, -0.64)
    rear_right_fender = superellipse_side_loft(rear_fender_sections, exponents=2.3, segments=42).translate(-0.46, 0.50, -0.64)
    rear_bodywork.visual(_save_mesh("atv_rear_left_fender.obj", rear_left_fender), material=body_red)
    rear_bodywork.visual(_save_mesh("atv_rear_right_fender.obj", rear_right_fender), material=body_red)
    rear_bodywork.visual(Box((0.66, 0.40, 0.05)), origin=Origin(xyz=(0.0, 0.04, -0.07)), material=body_red)
    rear_bodywork.visual(Box((0.30, 0.16, 0.07)), origin=Origin(xyz=(0.0, 0.00, 0.00)), material=dark_steel)
    rear_bodywork.visual(Box((0.78, 0.16, 0.04)), origin=Origin(xyz=(0.0, 0.05, -0.03)), material=body_red)
    rear_bodywork.visual(Box((0.18, 0.18, 0.08)), origin=Origin(xyz=(0.32, 0.10, 0.00)), material=body_red)
    rear_bodywork.visual(Box((0.18, 0.18, 0.08)), origin=Origin(xyz=(-0.32, 0.10, 0.00)), material=body_red)
    rear_bodywork.visual(Box((0.34, 0.06, 0.03)), origin=Origin(xyz=(0.0, 0.12, 0.01)), material=black_plastic)
    chassis.visual(Box((0.14, 0.58, 0.08)), origin=Origin(xyz=(0.0, -0.10, 0.44)), material=black_plastic)
    chassis.visual(Box((0.22, 0.30, 0.06)), origin=Origin(xyz=(0.30, -0.02, 0.40)), material=black_plastic)
    chassis.visual(Box((0.22, 0.30, 0.06)), origin=Origin(xyz=(-0.30, -0.02, 0.40)), material=black_plastic)
    chassis.visual(Box((0.44, 0.12, 0.08)), origin=Origin(xyz=(0.0, 0.37, 0.41)), material=frame_paint)
    chassis.visual(Box((0.54, 0.18, 0.10)), origin=Origin(xyz=(0.0, -0.26, 0.42)), material=frame_paint)
    chassis.visual(Cylinder(radius=0.05, length=0.18), origin=Origin(xyz=(0.0, 0.18, 0.71)), material=dark_steel)
    chassis.visual(Box((0.22, 0.12, 0.08)), origin=Origin(xyz=(0.0, 0.58, 0.66)), material=frame_paint)
    chassis.visual(Box((0.26, 0.30, 0.14)), origin=Origin(xyz=(0.0, -0.18, 0.67)), material=black_plastic)
    chassis.visual(Box((0.32, 0.20, 0.08)), origin=Origin(xyz=(0.0, -0.50, 0.66)), material=frame_paint)

    left_lower_rail = tube_from_spline_points(
        [(0.18, 0.57, 0.39), (0.24, 0.25, 0.40), (0.27, -0.02, 0.39), (0.31, -0.32, 0.38), (0.35, -0.52, 0.34)],
        radius=0.030,
        samples_per_segment=12,
        radial_segments=18,
    )
    right_lower_rail = tube_from_spline_points(
        _mirror_x([(0.18, 0.57, 0.39), (0.24, 0.25, 0.40), (0.27, -0.02, 0.39), (0.31, -0.32, 0.38), (0.35, -0.52, 0.34)]),
        radius=0.030,
        samples_per_segment=12,
        radial_segments=18,
    )
    center_spine = tube_from_spline_points(
        [(0.0, 0.50, 0.50), (0.0, 0.22, 0.58), (0.0, -0.04, 0.63), (0.0, -0.28, 0.60)],
        radius=0.045,
        samples_per_segment=16,
        radial_segments=18,
    )
    chassis.visual(_save_mesh("atv_left_lower_rail.obj", left_lower_rail), material=frame_paint)
    chassis.visual(_save_mesh("atv_right_lower_rail.obj", right_lower_rail), material=frame_paint)
    chassis.visual(_save_mesh("atv_center_spine.obj", center_spine), material=frame_paint)

    for name, points in [
        (
            "atv_left_lower_arm.obj",
            [(0.16, 0.43, 0.40), (0.30, 0.48, 0.37), (0.46, 0.56, 0.33), (0.30, 0.62, 0.37), (0.16, 0.58, 0.40)],
        ),
        (
            "atv_right_lower_arm.obj",
            _mirror_x([(0.16, 0.43, 0.40), (0.30, 0.48, 0.37), (0.46, 0.56, 0.33), (0.30, 0.62, 0.37), (0.16, 0.58, 0.40)]),
        ),
        (
            "atv_left_upper_arm.obj",
            [(0.18, 0.46, 0.54), (0.30, 0.50, 0.52), (0.46, 0.56, 0.50), (0.30, 0.61, 0.52), (0.18, 0.57, 0.54)],
        ),
        (
            "atv_right_upper_arm.obj",
            _mirror_x([(0.18, 0.46, 0.54), (0.30, 0.50, 0.52), (0.46, 0.56, 0.50), (0.30, 0.61, 0.52), (0.18, 0.57, 0.54)]),
        ),
    ]:
        arm = wire_from_points(
            points,
            radius=0.016 if "lower" in name else 0.013,
            radial_segments=14,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.03,
            corner_segments=8,
        )
        chassis.visual(_save_mesh(name, arm), material=frame_paint)

    for name, points in [
        ("atv_left_front_shock_body.obj", [(0.27, 0.46, 0.61), (0.43, 0.55, 0.39)]),
        ("atv_right_front_shock_body.obj", _mirror_x([(0.27, 0.46, 0.61), (0.43, 0.55, 0.39)])),
    ]:
        shock_body = tube_from_spline_points(points, radius=0.030, samples_per_segment=2, radial_segments=16)
        shock_rod = tube_from_spline_points(
            [
                (points[0][0] * 0.96, points[0][1] * 0.98, points[0][2] - 0.01),
                (points[1][0] * 0.98, points[1][1], points[1][2] + 0.01),
            ],
            radius=0.013,
            samples_per_segment=2,
            radial_segments=14,
        )
        chassis.visual(_save_mesh(name, shock_body), material=shock_silver)
        chassis.visual(_save_mesh(name.replace("_body", "_rod"), shock_rod), material=dark_steel)

    for name, points in [
        ("atv_left_swingarm.obj", [(0.17, -0.20, 0.44), (0.27, -0.35, 0.39), (0.36, -0.52, 0.34), (0.46, -0.55, 0.32)]),
        ("atv_right_swingarm.obj", _mirror_x([(0.17, -0.20, 0.44), (0.27, -0.35, 0.39), (0.36, -0.52, 0.34), (0.46, -0.55, 0.32)])),
        ("atv_left_rear_shock.obj", [(0.18, -0.12, 0.64), (0.33, -0.46, 0.41)]),
        ("atv_right_rear_shock.obj", _mirror_x([(0.18, -0.12, 0.64), (0.33, -0.46, 0.41)])),
    ]:
        radius = 0.028 if "swingarm" in name else 0.030
        geom = tube_from_spline_points(points, radius=radius, samples_per_segment=10, radial_segments=16)
        chassis.visual(_save_mesh(name, geom), material=frame_paint if "swingarm" in name else shock_silver)

    chassis.visual(Box((0.06, 0.07, 0.26)), origin=Origin(xyz=(0.46, 0.56, 0.43)), material=dark_steel)
    chassis.visual(Box((0.06, 0.07, 0.26)), origin=Origin(xyz=(-0.46, 0.56, 0.43)), material=dark_steel)
    chassis.visual(Box((0.24, 0.12, 0.10)), origin=Origin(xyz=(0.39, -0.54, 0.32)), material=dark_steel)
    chassis.visual(Box((0.24, 0.12, 0.10)), origin=Origin(xyz=(-0.39, -0.54, 0.32)), material=dark_steel)

    left_footrest = model.part("left_footrest")
    left_footrest.inertial = Inertial.from_geometry(
        Box((0.18, 0.30, 0.05)),
        mass=2.5,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
    )
    left_footrest.visual(Box((0.18, 0.30, 0.05)), origin=Origin(xyz=(0.04, 0.0, 0.0)), material=black_plastic)
    left_footrest.visual(
        _save_mesh(
            "atv_left_footrest_loop.obj",
            tube_from_spline_points(
                [(0.0, 0.12, 0.0), (0.08, 0.08, 0.0), (0.10, 0.0, 0.0), (0.08, -0.12, 0.0), (0.0, -0.16, 0.0)],
                radius=0.013,
                samples_per_segment=12,
                radial_segments=14,
            ),
        ),
        material=frame_paint,
    )

    right_footrest = model.part("right_footrest")
    right_footrest.inertial = Inertial.from_geometry(
        Box((0.18, 0.30, 0.05)),
        mass=2.5,
        origin=Origin(xyz=(-0.04, 0.0, 0.0)),
    )
    right_footrest.visual(Box((0.18, 0.30, 0.05)), origin=Origin(xyz=(-0.04, 0.0, 0.0)), material=black_plastic)
    right_footrest.visual(
        _save_mesh(
            "atv_right_footrest_loop.obj",
            tube_from_spline_points(
                [(0.0, 0.12, 0.0), (-0.08, 0.08, 0.0), (-0.10, 0.0, 0.0), (-0.08, -0.12, 0.0), (0.0, -0.16, 0.0)],
                radius=0.013,
                samples_per_segment=12,
                radial_segments=14,
            ),
        ),
        material=frame_paint,
    )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.72, 0.14, 0.18)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )
    handlebar.visual(Cylinder(radius=0.023, length=0.12), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=dark_steel)
    handlebar.visual(Box((0.08, 0.05, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.11)), material=dark_steel)
    bar_geom = tube_from_spline_points(
        [(-0.31, -0.02, 0.09), (-0.20, -0.01, 0.12), (-0.08, 0.0, 0.14), (0.08, 0.0, 0.14), (0.20, -0.01, 0.12), (0.31, -0.02, 0.09)],
        radius=0.016,
        samples_per_segment=16,
        radial_segments=18,
    )
    handlebar.visual(_save_mesh("atv_handlebar_bar.obj", bar_geom), material=dark_steel)
    handlebar.visual(Cylinder(radius=0.020, length=0.11), origin=Origin(xyz=(0.27, -0.02, 0.09), rpy=(0.0, pi / 2.0, 0.0)), material=rubber)
    handlebar.visual(Cylinder(radius=0.020, length=0.11), origin=Origin(xyz=(-0.27, -0.02, 0.09), rpy=(0.0, pi / 2.0, 0.0)), material=rubber)
    handlebar.visual(Cylinder(radius=0.010, length=0.18), origin=Origin(xyz=(0.0, 0.0, 0.10), rpy=(0.0, pi / 2.0, 0.0)), material=dark_steel)
    handlebar.visual(Box((0.10, 0.04, 0.05)), origin=Origin(xyz=(0.0, -0.01, 0.13)), material=black_plastic)

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.30, length=0.24),
        mass=12.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        front_left_wheel,
        "atv_front_left_wheel",
        tire_radius=0.30,
        tire_width=0.24,
        hub_width=0.30,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        rubber=rubber,
        side_sign=1.0,
        add_knuckle=True,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.30, length=0.24),
        mass=12.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        front_right_wheel,
        "atv_front_right_wheel",
        tire_radius=0.30,
        tire_width=0.24,
        hub_width=0.30,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        rubber=rubber,
        side_sign=-1.0,
        add_knuckle=True,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.31, length=0.30),
        mass=13.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        rear_left_wheel,
        "atv_rear_left_wheel",
        tire_radius=0.31,
        tire_width=0.30,
        hub_width=0.36,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.31, length=0.30),
        mass=13.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        rear_right_wheel,
        "atv_rear_right_wheel",
        tire_radius=0.31,
        tire_width=0.30,
        hub_width=0.36,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )

    model.articulation(
        "front_body_mount",
        ArticulationType.FIXED,
        parent="chassis",
        child="front_bodywork",
        origin=Origin(xyz=(0.0, 0.58, 0.70)),
    )
    model.articulation(
        "seat_mount",
        ArticulationType.FIXED,
        parent="chassis",
        child="seat",
        origin=Origin(xyz=(0.0, -0.18, 0.74)),
    )
    model.articulation(
        "rear_body_mount",
        ArticulationType.FIXED,
        parent="chassis",
        child="rear_bodywork",
        origin=Origin(xyz=(0.0, -0.50, 0.70)),
    )
    model.articulation(
        "left_footrest_mount",
        ArticulationType.FIXED,
        parent="chassis",
        child="left_footrest",
        origin=Origin(xyz=(0.31, -0.02, 0.39)),
    )
    model.articulation(
        "right_footrest_mount",
        ArticulationType.FIXED,
        parent="chassis",
        child="right_footrest",
        origin=Origin(xyz=(-0.31, -0.02, 0.39)),
    )
    model.articulation(
        "handlebar_steer",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="handlebar",
        origin=Origin(xyz=(0.0, 0.18, 0.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "front_left_steer",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="front_left_wheel",
        origin=Origin(xyz=(0.49, 0.58, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "front_right_steer",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="front_right_wheel",
        origin=Origin(xyz=(-0.49, 0.58, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="rear_left_wheel",
        origin=Origin(xyz=(0.50, -0.54, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="rear_right_wheel",
        origin=Origin(xyz=(-0.50, -0.54, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.008,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("front_bodywork", "chassis")
    ctx.expect_aabb_contact("seat", "chassis")
    ctx.expect_aabb_contact("rear_bodywork", "chassis")
    ctx.expect_aabb_contact("left_footrest", "chassis")
    ctx.expect_aabb_contact("right_footrest", "chassis")
    ctx.expect_aabb_contact("handlebar", "chassis")
    ctx.expect_aabb_contact("front_left_wheel", "chassis")
    ctx.expect_aabb_contact("front_right_wheel", "chassis")
    ctx.expect_aabb_contact("rear_left_wheel", "chassis")
    ctx.expect_aabb_contact("rear_right_wheel", "chassis")

    ctx.expect_origin_distance("seat", "chassis", axes="x", max_dist=0.02)
    ctx.expect_origin_distance("handlebar", "front_bodywork", axes="x", max_dist=0.02)
    ctx.expect_origin_distance("left_footrest", "right_footrest", axes="yz", max_dist=0.03)
    ctx.expect_origin_distance("front_left_wheel", "rear_left_wheel", axes="x", max_dist=0.03)
    ctx.expect_origin_distance("front_right_wheel", "rear_right_wheel", axes="x", max_dist=0.03)
    ctx.expect_origin_distance("front_left_wheel", "front_right_wheel", axes="yz", max_dist=0.03)
    ctx.expect_origin_distance("rear_left_wheel", "rear_right_wheel", axes="yz", max_dist=0.03)

    ctx.expect_aabb_overlap("left_footrest", "seat", axes="y", min_overlap=0.10)
    ctx.expect_aabb_overlap("right_footrest", "seat", axes="y", min_overlap=0.10)
    ctx.expect_aabb_gap("seat", "left_footrest", axis="z", max_gap=0.45, max_penetration=0.0)
    ctx.expect_aabb_gap("seat", "right_footrest", axis="z", max_gap=0.45, max_penetration=0.0)
    ctx.expect_aabb_overlap("front_bodywork", "front_left_wheel", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("front_bodywork", "front_right_wheel", axes="xy", min_overlap=0.10)
    ctx.expect_aabb_overlap("rear_bodywork", "rear_left_wheel", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_overlap("rear_bodywork", "rear_right_wheel", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_gap("front_bodywork", "front_left_wheel", axis="z", max_gap=0.10, max_penetration=0.0)
    ctx.expect_aabb_gap("front_bodywork", "front_right_wheel", axis="z", max_gap=0.10, max_penetration=0.0)
    ctx.expect_aabb_gap("rear_bodywork", "rear_left_wheel", axis="z", max_gap=0.12, max_penetration=0.02)
    ctx.expect_aabb_gap("rear_bodywork", "rear_right_wheel", axis="z", max_gap=0.12, max_penetration=0.02)
    ctx.expect_aabb_overlap("front_left_wheel", "rear_left_wheel", axes="xz", min_overlap=0.12)
    ctx.expect_aabb_overlap("front_right_wheel", "rear_right_wheel", axes="xz", min_overlap=0.12)
    ctx.expect_aabb_overlap("front_left_wheel", "front_right_wheel", axes="yz", min_overlap=0.45)
    ctx.expect_aabb_overlap("rear_left_wheel", "rear_right_wheel", axes="yz", min_overlap=0.45)
    ctx.expect_aabb_gap("front_left_wheel", "rear_left_wheel", axis="y", max_gap=2.0, max_penetration=0.0)
    ctx.expect_aabb_gap("front_right_wheel", "rear_right_wheel", axis="y", max_gap=2.0, max_penetration=0.0)
    ctx.expect_aabb_gap("front_left_wheel", "front_right_wheel", axis="x", max_gap=2.0, max_penetration=0.0)
    ctx.expect_aabb_gap("rear_left_wheel", "rear_right_wheel", axis="x", max_gap=2.0, max_penetration=0.0)

    with ctx.pose(handlebar_steer=0.45, front_left_steer=0.45, front_right_steer=0.45):
        ctx.expect_aabb_contact("handlebar", "chassis")
        ctx.expect_aabb_contact("front_left_wheel", "chassis")
        ctx.expect_aabb_contact("front_right_wheel", "chassis")
        ctx.expect_aabb_overlap("front_bodywork", "front_left_wheel", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_overlap("front_bodywork", "front_right_wheel", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_gap("front_left_wheel", "rear_left_wheel", axis="y", max_gap=2.0, max_penetration=0.0)
        ctx.expect_aabb_gap("front_right_wheel", "rear_right_wheel", axis="y", max_gap=2.0, max_penetration=0.0)

    with ctx.pose(handlebar_steer=-0.45, front_left_steer=-0.45, front_right_steer=-0.45):
        ctx.expect_aabb_contact("handlebar", "chassis")
        ctx.expect_aabb_contact("front_left_wheel", "chassis")
        ctx.expect_aabb_contact("front_right_wheel", "chassis")
        ctx.expect_aabb_overlap("front_bodywork", "front_left_wheel", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_overlap("front_bodywork", "front_right_wheel", axes="xy", min_overlap=0.08)
        ctx.expect_aabb_overlap("front_left_wheel", "front_right_wheel", axes="yz", min_overlap=0.40)

    with ctx.pose(rear_left_spin=1.3, rear_right_spin=-0.9):
        ctx.expect_aabb_contact("rear_left_wheel", "chassis")
        ctx.expect_aabb_contact("rear_right_wheel", "chassis")
        ctx.expect_origin_distance("rear_left_wheel", "rear_right_wheel", axes="yz", max_dist=0.03)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
