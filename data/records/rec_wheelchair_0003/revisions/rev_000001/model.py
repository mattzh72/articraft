from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, radians, sin, sqrt

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
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = (dx * dx + dy * dy) ** 0.5
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


def _build_drive_wheel(
    part,
    *,
    side_sign: float,
    tire_mesh,
    rim_mesh,
    handrim_mesh,
    rubber,
    rim_paint,
    aluminum,
    steel,
    reflector,
) -> None:
    part.visual(tire_mesh, material=rubber, name="tire")
    part.visual(rim_mesh, material=rim_paint, name="rim")
    part.visual(
        Cylinder(radius=0.048, length=0.034),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_paint,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.061, length=0.006),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.061, length=0.006),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(-0.010 * side_sign, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="inner_bearing",
    )
    part.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.013 * side_sign, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
    )
    for index in range(12):
        angle = 2.0 * pi * index / 12.0
        spoke_phase = angle + (pi / 28.0 if index % 2 else -pi / 28.0)
        inner = (
            0.009 if index % 2 == 0 else -0.009,
            cos(spoke_phase) * 0.050,
            sin(spoke_phase) * 0.050,
        )
        outer = (0.0, cos(angle) * 0.247, sin(angle) * 0.247)
        _add_member(
            part,
            inner,
            outer,
            0.0022,
            aluminum,
            name="index_spoke" if index == 0 else None,
        )
    part.visual(
        handrim_mesh,
        origin=Origin(xyz=(0.022 * side_sign, 0.0, 0.0)),
        material=aluminum,
        name="handrim",
    )
    for index in range(6):
        angle = 2.0 * pi * index / 6.0 + (pi / 12.0)
        rim_point = (0.012 * side_sign, cos(angle) * 0.214, sin(angle) * 0.214)
        handrim_point = (0.022 * side_sign, cos(angle) * 0.268, sin(angle) * 0.268)
        _add_member(part, rim_point, handrim_point, 0.0042, aluminum)
    part.visual(
        Box((0.012, 0.004, 0.012)),
        origin=Origin(xyz=(0.022 * side_sign, 0.0, 0.270)),
        material=reflector,
        name="wheel_marker",
    )
    for angle in (0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0):
        part.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(
                xyz=(0.013 * side_sign, cos(angle) * 0.028, sin(angle) * 0.028),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
        )


def _build_caster_fork(part, *, frame_paint, steel) -> None:
    part.visual(
        Cylinder(radius=0.012, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=steel,
        name="swivel_stem",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=frame_paint,
    )
    part.visual(
        Box((0.062, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.094)),
        material=frame_paint,
        name="fork_crown",
    )
    part.visual(
        Box((0.010, 0.020, 0.150)),
        origin=Origin(xyz=(0.026, 0.0, -0.169)),
        material=frame_paint,
        name="outer_fork_leg",
    )
    part.visual(
        Box((0.010, 0.020, 0.150)),
        origin=Origin(xyz=(-0.026, 0.0, -0.169)),
        material=frame_paint,
    )
    part.visual(
        Cylinder(radius=0.007, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.169), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    part.visual(
        Box((0.024, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=steel,
        name="fork_bridge",
    )


def _build_caster_wheel(part, *, tire_mesh, rubber, steel, aluminum, reflector) -> None:
    part.visual(
        tire_mesh,
        material=rubber,
        name="caster_tire",
    )
    part.visual(
        Cylinder(radius=0.048, length=0.008),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.048, length=0.008),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )
    part.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
    )
    part.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
    )
    part.visual(
        Box((0.008, 0.018, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.055)),
        material=reflector,
        name="caster_marker",
    )


def _build_footrest_hanger(part, *, side_sign: float, frame_paint, steel) -> float:
    inward_x = -0.055 * side_sign
    part.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=steel,
        name="upper_hinge_barrel",
    )
    _add_member(
        part,
        (0.0, 0.0, -0.007),
        (inward_x * 0.52, 0.040, -0.088),
        0.010,
        frame_paint,
        name="hanger_tube",
    )
    _add_member(
        part,
        (inward_x * 0.52, 0.040, -0.088),
        (inward_x, 0.095, -0.240),
        0.010,
        frame_paint,
    )
    part.visual(
        Box((0.036, 0.026, 0.040)),
        origin=Origin(xyz=(inward_x * 0.48, 0.040, -0.090)),
        material=frame_paint,
    )
    part.visual(
        Box((0.044, 0.012, 0.020)),
        origin=Origin(xyz=(inward_x, 0.095, -0.232)),
        material=frame_paint,
    )
    part.visual(
        Box((0.012, 0.018, 0.026)),
        origin=Origin(xyz=(inward_x - 0.016, 0.095, -0.240)),
        material=frame_paint,
        name="plate_hinge_block_outer",
    )
    part.visual(
        Box((0.012, 0.018, 0.026)),
        origin=Origin(xyz=(inward_x + 0.016, 0.095, -0.240)),
        material=frame_paint,
        name="plate_hinge_block",
    )
    return inward_x


def _build_footplate(part, *, plastic_black, rubber, steel) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    part.visual(
        Box((0.110, 0.130, 0.014)),
        origin=Origin(xyz=(0.0, 0.040, -0.006)),
        material=plastic_black,
        name="tread_plate",
    )
    part.visual(
        Box((0.095, 0.100, 0.006)),
        origin=Origin(xyz=(0.0, 0.042, 0.002)),
        material=rubber,
    )
    part.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(0.0, 0.102, -0.006), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
    )


def _build_brake_lever(part, *, side_sign: float, frame_paint, plastic_black, steel) -> None:
    part.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    _add_member(
        part,
        (0.0, 0.0, 0.0),
        (0.0, 0.030, 0.130),
        0.007,
        frame_paint,
        name="brake_handle",
    )
    part.visual(
        Box((0.022, 0.040, 0.016)),
        origin=Origin(xyz=(0.0, 0.038, 0.138)),
        material=plastic_black,
    )
    part.visual(
        Box((0.014, 0.024, 0.028)),
        origin=Origin(xyz=(-0.008 * side_sign, 0.002, 0.032)),
        material=steel,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_wheelchair", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.24, 0.26, 0.24, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.14, 0.14, 0.15, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.09, 0.09, 0.10, 1.0))
    reflector = model.material("reflector", rgba=(0.80, 0.48, 0.12, 1.0))

    drive_tire_mesh = _save_mesh(
        "drive_tire.obj",
        TorusGeometry(radius=0.285, tube=0.019, radial_segments=20, tubular_segments=84).rotate_y(pi / 2.0),
    )
    drive_rim_mesh = _save_mesh(
        "drive_rim.obj",
        TorusGeometry(radius=0.252, tube=0.016, radial_segments=18, tubular_segments=80).rotate_y(pi / 2.0),
    )
    drive_handrim_mesh = _save_mesh(
        "drive_handrim.obj",
        TorusGeometry(radius=0.274, tube=0.006, radial_segments=14, tubular_segments=72).rotate_y(pi / 2.0),
    )
    caster_tire_mesh = _save_mesh(
        "caster_tire.obj",
        TorusGeometry(radius=0.058, tube=0.018, radial_segments=16, tubular_segments=56).rotate_y(pi / 2.0),
    )

    main_frame = model.part("main_frame")
    main_frame.inertial = Inertial.from_geometry(
        Box((0.70, 0.82, 0.74)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.02, 0.37)),
    )
    main_frame.visual(
        Box((0.440, 0.420, 0.028)),
        origin=Origin(xyz=(0.0, 0.000, 0.455)),
        material=dark_steel,
        name="seat_pan",
    )
    main_frame.visual(
        Box((0.420, 0.410, 0.028)),
        origin=Origin(xyz=(0.0, 0.000, 0.483)),
        material=seat_vinyl,
        name="seat_cushion",
    )
    for side_sign in (-1.0, 1.0):
        sx = 0.230 * side_sign
        main_frame.visual(
            Box((0.032, 0.460, 0.100)),
            origin=Origin(xyz=(sx, 0.000, 0.495)),
            material=frame_paint,
            name="right_side_panel" if side_sign < 0.0 else "left_side_panel",
        )
        main_frame.visual(
            Box((0.056, 0.310, 0.030)),
            origin=Origin(xyz=(sx, -0.010, 0.560)),
            material=plastic_black,
            name="right_arm_pad" if side_sign < 0.0 else "left_arm_pad",
        )
        main_frame.visual(
            Box((0.008, 0.340, 0.130)),
            origin=Origin(xyz=(0.184 * side_sign, -0.030, 0.480)),
            material=dark_steel,
        )
        _add_member(main_frame, (sx, -0.190, 0.535), (sx, 0.165, 0.535), 0.013, frame_paint)
        _add_member(main_frame, (sx, -0.190, 0.535), (sx, -0.190, 0.568), 0.012, frame_paint)
        _add_member(main_frame, (sx, 0.165, 0.535), (sx, 0.165, 0.568), 0.012, frame_paint)
        _add_member(main_frame, (sx, 0.180, 0.440), (0.214 * side_sign, 0.345, 0.275), 0.014, frame_paint)
        _add_member(main_frame, (sx, 0.180, 0.440), (0.170 * side_sign, 0.205, 0.365), 0.012, frame_paint)
        _add_member(main_frame, (sx, 0.000, 0.448), (0.239 * side_sign, -0.060, 0.370), 0.016, frame_paint)
        _add_member(main_frame, (sx, -0.180, 0.448), (0.214 * side_sign, -0.205, 0.500), 0.013, frame_paint)
        main_frame.visual(
            Box((0.056, 0.160, 0.260)),
            origin=Origin(xyz=(0.239 * side_sign, -0.060, 0.370)),
            material=dark_steel,
            name="right_axle_plate" if side_sign < 0.0 else "left_axle_plate",
        )
        main_frame.visual(
            Cylinder(radius=0.017, length=0.034),
            origin=Origin(
                xyz=(0.284 * side_sign, -0.040, 0.304),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name="right_axle_stub" if side_sign < 0.0 else "left_axle_stub",
        )
        main_frame.visual(
            Box((0.072, 0.080, 0.012)),
            origin=Origin(xyz=(0.248 * side_sign, -0.060, 0.520)),
            material=frame_paint,
        )
        for bolt_y in (-0.100, -0.020):
            for bolt_z in (0.320, 0.420):
                main_frame.visual(
                    Cylinder(radius=0.0045, length=0.008),
                    origin=Origin(
                        xyz=(0.268 * side_sign, bolt_y, bolt_z),
                        rpy=(0.0, pi / 2.0, 0.0),
                    ),
                    material=steel,
                )
        main_frame.visual(
            Cylinder(radius=0.025, length=0.020),
            origin=Origin(xyz=(0.225 * side_sign, 0.380, 0.256)),
            material=steel,
            name="right_caster_housing" if side_sign < 0.0 else "left_caster_housing",
        )
        main_frame.visual(
            Box((0.055, 0.035, 0.050)),
            origin=Origin(xyz=(0.214 * side_sign, 0.345, 0.275)),
            material=dark_steel,
        )
        main_frame.visual(
            Box((0.016, 0.030, 0.018)),
            origin=Origin(xyz=(0.182 * side_sign, 0.190, 0.350)),
            material=dark_steel,
            name="right_footrest_receiver" if side_sign < 0.0 else "left_footrest_receiver",
        )
        main_frame.visual(
            Cylinder(radius=0.015, length=0.014),
            origin=Origin(xyz=(0.165 * side_sign, 0.205, 0.372)),
            material=steel,
            name="right_footrest_lug" if side_sign < 0.0 else "left_footrest_lug",
        )
        main_frame.visual(
            Box((0.020, 0.040, 0.080)),
            origin=Origin(xyz=(0.228 * side_sign, 0.035, 0.585)),
            material=dark_steel,
            name="right_brake_mount" if side_sign < 0.0 else "left_brake_mount",
        )
        main_frame.visual(
            Cylinder(radius=0.016, length=0.028),
            origin=Origin(
                xyz=(0.214 * side_sign, -0.225, 0.500),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name="right_backrest_lug" if side_sign < 0.0 else "left_backrest_lug",
        )
    _add_member(main_frame, (-0.180, 0.190, 0.440), (0.180, 0.190, 0.440), 0.013, frame_paint, name="front_cross_tube")
    _add_member(main_frame, (-0.170, -0.175, 0.440), (0.170, -0.175, 0.440), 0.013, frame_paint, name="rear_cross_tube")
    _add_member(main_frame, (-0.180, 0.190, 0.420), (0.180, -0.175, 0.420), 0.010, steel)
    _add_member(main_frame, (0.180, 0.190, 0.420), (-0.180, -0.175, 0.420), 0.010, steel)
    main_frame.visual(
        Box((0.060, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.008, 0.420)),
        material=dark_steel,
    )
    main_frame.visual(
        Box((0.180, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.190, 0.440)),
        material=dark_steel,
    )

    backrest = model.part("backrest")
    backrest.inertial = Inertial.from_geometry(
        Box((0.480, 0.160, 0.520)),
        mass=4.6,
        origin=Origin(xyz=(0.0, -0.030, 0.220)),
    )
    backrest.visual(
        Box((0.400, 0.050, 0.340)),
        origin=Origin(xyz=(0.0, -0.025, 0.230)),
        material=seat_vinyl,
        name="back_pad",
    )
    _add_member(backrest, (-0.190, -0.012, 0.000), (-0.190, -0.020, 0.420), 0.014, frame_paint)
    _add_member(backrest, (0.190, -0.012, 0.000), (0.190, -0.020, 0.420), 0.014, frame_paint)
    _add_member(backrest, (-0.190, -0.020, 0.050), (0.190, -0.020, 0.050), 0.012, frame_paint)
    _add_member(backrest, (-0.190, -0.020, 0.410), (0.190, -0.020, 0.410), 0.012, frame_paint, name="push_bar")
    _add_member(backrest, (-0.190, -0.020, 0.400), (-0.190, -0.055, 0.438), 0.014, frame_paint)
    _add_member(backrest, (0.190, -0.020, 0.400), (0.190, -0.055, 0.438), 0.014, frame_paint)
    backrest.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(-0.190, -0.066, 0.446), rpy=(pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
    )
    backrest.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.190, -0.066, 0.446), rpy=(pi / 2.0, 0.0, 0.0)),
        material=plastic_black,
    )
    backrest.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(-0.189, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="left_backrest_barrel",
    )
    backrest.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(0.189, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="right_backrest_barrel",
    )
    left_drive_wheel = model.part("left_drive_wheel")
    left_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.304, length=0.038),
        mass=3.4,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_drive_wheel(
        left_drive_wheel,
        side_sign=1.0,
        tire_mesh=drive_tire_mesh,
        rim_mesh=drive_rim_mesh,
        handrim_mesh=drive_handrim_mesh,
        rubber=rubber,
        rim_paint=frame_paint,
        aluminum=aluminum,
        steel=steel,
        reflector=reflector,
    )

    right_drive_wheel = model.part("right_drive_wheel")
    right_drive_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.304, length=0.038),
        mass=3.4,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_drive_wheel(
        right_drive_wheel,
        side_sign=-1.0,
        tire_mesh=drive_tire_mesh,
        rim_mesh=drive_rim_mesh,
        handrim_mesh=drive_handrim_mesh,
        rubber=rubber,
        rim_paint=frame_paint,
        aluminum=aluminum,
        steel=steel,
        reflector=reflector,
    )

    left_caster_fork = model.part("left_caster_fork")
    left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.070, 0.040, 0.210)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
    )
    _build_caster_fork(left_caster_fork, frame_paint=frame_paint, steel=steel)

    right_caster_fork = model.part("right_caster_fork")
    right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.070, 0.040, 0.210)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
    )
    _build_caster_fork(right_caster_fork, frame_paint=frame_paint, steel=steel)

    left_caster_wheel = model.part("left_caster_wheel")
    left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.036),
        mass=1.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_caster_wheel(
        left_caster_wheel,
        tire_mesh=caster_tire_mesh,
        rubber=rubber,
        steel=dark_steel,
        aluminum=aluminum,
        reflector=reflector,
    )

    right_caster_wheel = model.part("right_caster_wheel")
    right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.036),
        mass=1.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _build_caster_wheel(
        right_caster_wheel,
        tire_mesh=caster_tire_mesh,
        rubber=rubber,
        steel=dark_steel,
        aluminum=aluminum,
        reflector=reflector,
    )

    left_footrest_hanger = model.part("left_footrest_hanger")
    left_footrest_hanger.inertial = Inertial.from_geometry(
        Box((0.090, 0.160, 0.280)),
        mass=1.1,
        origin=Origin(xyz=(-0.020, 0.060, -0.120)),
    )
    left_hanger_x = _build_footrest_hanger(
        left_footrest_hanger,
        side_sign=1.0,
        frame_paint=frame_paint,
        steel=steel,
    )

    right_footrest_hanger = model.part("right_footrest_hanger")
    right_footrest_hanger.inertial = Inertial.from_geometry(
        Box((0.090, 0.160, 0.280)),
        mass=1.1,
        origin=Origin(xyz=(0.020, 0.060, -0.120)),
    )
    right_hanger_x = _build_footrest_hanger(
        right_footrest_hanger,
        side_sign=-1.0,
        frame_paint=frame_paint,
        steel=steel,
    )

    left_footplate = model.part("left_footplate")
    left_footplate.inertial = Inertial.from_geometry(
        Box((0.110, 0.150, 0.020)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.070, -0.005)),
    )
    _build_footplate(left_footplate, plastic_black=plastic_black, rubber=rubber, steel=steel)

    right_footplate = model.part("right_footplate")
    right_footplate.inertial = Inertial.from_geometry(
        Box((0.110, 0.150, 0.020)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.070, -0.005)),
    )
    _build_footplate(right_footplate, plastic_black=plastic_black, rubber=rubber, steel=steel)

    left_brake_lever = model.part("left_brake_lever")
    left_brake_lever.inertial = Inertial.from_geometry(
        Box((0.030, 0.060, 0.120)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.010, 0.050)),
    )
    _build_brake_lever(
        left_brake_lever,
        side_sign=1.0,
        frame_paint=frame_paint,
        plastic_black=plastic_black,
        steel=steel,
    )

    right_brake_lever = model.part("right_brake_lever")
    right_brake_lever.inertial = Inertial.from_geometry(
        Box((0.030, 0.060, 0.120)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.010, 0.050)),
    )
    _build_brake_lever(
        right_brake_lever,
        side_sign=-1.0,
        frame_paint=frame_paint,
        plastic_black=plastic_black,
        steel=steel,
    )

    model.articulation(
        "backrest_fold",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.225, 0.500)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=radians(68.0),
        ),
    )
    model.articulation(
        "left_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_drive_wheel,
        origin=Origin(xyz=(0.304, -0.040, 0.304)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=20.0),
    )
    model.articulation(
        "right_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_drive_wheel,
        origin=Origin(xyz=(-0.304, -0.040, 0.304)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=20.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_caster_fork,
        origin=Origin(xyz=(0.225, 0.380, 0.256)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=7.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_caster_fork,
        origin=Origin(xyz=(-0.225, 0.380, 0.256)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=7.0),
    )
    model.articulation(
        "left_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_fork,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "right_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_fork,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "left_footrest_swing",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=left_footrest_hanger,
        origin=Origin(xyz=(0.165, 0.205, 0.365)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=0.0,
            upper=radians(35.0),
        ),
    )
    model.articulation(
        "right_footrest_swing",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=right_footrest_hanger,
        origin=Origin(xyz=(-0.165, 0.205, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=0.0,
            upper=radians(35.0),
        ),
    )
    model.articulation(
        "left_footplate_fold",
        ArticulationType.REVOLUTE,
        parent=left_footrest_hanger,
        child=left_footplate,
        origin=Origin(xyz=(left_hanger_x, 0.105, -0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=radians(85.0),
        ),
    )
    model.articulation(
        "right_footplate_fold",
        ArticulationType.REVOLUTE,
        parent=right_footrest_hanger,
        child=right_footplate,
        origin=Origin(xyz=(right_hanger_x, 0.105, -0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=radians(85.0),
        ),
    )
    model.articulation(
        "left_brake_swing",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=left_brake_lever,
        origin=Origin(xyz=(0.236, 0.035, 0.620)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=radians(-25.0),
            upper=radians(22.0),
        ),
    )
    model.articulation(
        "right_brake_swing",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=right_brake_lever,
        origin=Origin(xyz=(-0.236, 0.035, 0.620)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=radians(-25.0),
            upper=radians(22.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    main_frame = object_model.get_part("main_frame")
    backrest = object_model.get_part("backrest")
    left_drive_wheel = object_model.get_part("left_drive_wheel")
    right_drive_wheel = object_model.get_part("right_drive_wheel")
    left_caster_fork = object_model.get_part("left_caster_fork")
    right_caster_fork = object_model.get_part("right_caster_fork")
    left_caster_wheel = object_model.get_part("left_caster_wheel")
    right_caster_wheel = object_model.get_part("right_caster_wheel")
    left_footrest_hanger = object_model.get_part("left_footrest_hanger")
    right_footrest_hanger = object_model.get_part("right_footrest_hanger")
    left_footplate = object_model.get_part("left_footplate")
    right_footplate = object_model.get_part("right_footplate")
    left_brake_lever = object_model.get_part("left_brake_lever")
    right_brake_lever = object_model.get_part("right_brake_lever")

    seat_cushion = main_frame.get_visual("seat_cushion")
    back_push_bar = backrest.get_visual("push_bar")
    rear_marker = left_drive_wheel.get_visual("wheel_marker")
    caster_marker = left_caster_wheel.get_visual("caster_marker")
    caster_crown = left_caster_fork.get_visual("fork_crown")

    backrest_fold = object_model.get_articulation("backrest_fold")
    left_rear_spin = object_model.get_articulation("left_rear_spin")
    right_rear_spin = object_model.get_articulation("right_rear_spin")
    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_caster_spin = object_model.get_articulation("left_caster_spin")
    right_caster_spin = object_model.get_articulation("right_caster_spin")
    left_footrest_swing = object_model.get_articulation("left_footrest_swing")
    right_footrest_swing = object_model.get_articulation("right_footrest_swing")
    left_footplate_fold = object_model.get_articulation("left_footplate_fold")
    right_footplate_fold = object_model.get_articulation("right_footplate_fold")
    left_brake_swing = object_model.get_articulation("left_brake_swing")
    right_brake_swing = object_model.get_articulation("right_brake_swing")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        left_drive_wheel,
        main_frame,
        reason="Rear wheel hub rotates on the frame axle stub with an intentional coaxial bearing overlap.",
    )
    ctx.allow_overlap(
        right_drive_wheel,
        main_frame,
        reason="Rear wheel hub rotates on the frame axle stub with an intentional coaxial bearing overlap.",
    )
    ctx.allow_overlap(
        left_caster_fork,
        main_frame,
        reason="Caster swivel stem seats concentrically inside the welded caster housing.",
    )
    ctx.allow_overlap(
        right_caster_fork,
        main_frame,
        reason="Caster swivel stem seats concentrically inside the welded caster housing.",
    )
    ctx.allow_overlap(
        left_caster_wheel,
        left_caster_fork,
        reason="Caster wheel hub runs on the fork axle with intentional axle-through-hub overlap.",
    )
    ctx.allow_overlap(
        right_caster_wheel,
        right_caster_fork,
        reason="Caster wheel hub runs on the fork axle with intentional axle-through-hub overlap.",
    )
    ctx.allow_overlap(
        left_footrest_hanger,
        main_frame,
        reason="Swing-away footrest uses a concentric hinge barrel captured by the frame lug.",
    )
    ctx.allow_overlap(
        right_footrest_hanger,
        main_frame,
        reason="Swing-away footrest uses a concentric hinge barrel captured by the frame lug.",
    )
    ctx.allow_overlap(
        left_footplate,
        left_footrest_hanger,
        reason="Footplate rotates on a pin captured inside the twin hinge ears of the hanger.",
    )
    ctx.allow_overlap(
        right_footplate,
        right_footrest_hanger,
        reason="Footplate rotates on a pin captured inside the twin hinge ears of the hanger.",
    )
    ctx.allow_overlap(
        left_brake_lever,
        main_frame,
        reason="The parking brake lever pivots inside a boxed welded mount.",
    )
    ctx.allow_overlap(
        right_brake_lever,
        main_frame,
        reason="The parking brake lever pivots inside a boxed welded mount.",
    )
    ctx.allow_overlap(
        backrest,
        main_frame,
        reason="Fold-back backrest nests closely into the rear frame brackets and hinge barrels.",
    )
    ctx.check(
        "rear_wheel_axes",
        left_rear_spin.axis == (1.0, 0.0, 0.0) and right_rear_spin.axis == (1.0, 0.0, 0.0),
        f"rear wheel axes are {left_rear_spin.axis} and {right_rear_spin.axis}",
    )
    ctx.check(
        "caster_swivel_axes",
        left_caster_swivel.axis == (0.0, 0.0, 1.0) and right_caster_swivel.axis == (0.0, 0.0, 1.0),
        f"caster swivel axes are {left_caster_swivel.axis} and {right_caster_swivel.axis}",
    )
    ctx.check(
        "footplate_axes",
        left_footplate_fold.axis == (1.0, 0.0, 0.0) and right_footplate_fold.axis == (1.0, 0.0, 0.0),
        f"footplate axes are {left_footplate_fold.axis} and {right_footplate_fold.axis}",
    )
    ctx.check(
        "backrest_fold_axis",
        backrest_fold.axis == (-1.0, 0.0, 0.0),
        f"backrest fold axis is {backrest_fold.axis}",
    )

    for name, a, b in (
        ("backrest_mount_contact", backrest, main_frame),
        ("left_drive_wheel_axle_contact", left_drive_wheel, main_frame),
        ("right_drive_wheel_axle_contact", right_drive_wheel, main_frame),
        ("left_caster_swivel_contact", left_caster_fork, main_frame),
        ("right_caster_swivel_contact", right_caster_fork, main_frame),
        ("left_caster_wheel_contact", left_caster_wheel, left_caster_fork),
        ("right_caster_wheel_contact", right_caster_wheel, right_caster_fork),
        ("left_hanger_contact", left_footrest_hanger, main_frame),
        ("right_hanger_contact", right_footrest_hanger, main_frame),
        ("left_plate_contact", left_footplate, left_footrest_hanger),
        ("right_plate_contact", right_footplate, right_footrest_hanger),
        ("left_brake_contact", left_brake_lever, main_frame),
        ("right_brake_contact", right_brake_lever, main_frame),
    ):
        ctx.expect_contact(a, b, contact_tol=0.001, name=name)

    ctx.expect_overlap(backrest, main_frame, axes="x", min_overlap=0.36, name="backrest_centered_over_frame")
    ctx.expect_origin_distance(
        left_drive_wheel,
        right_drive_wheel,
        axes="x",
        min_dist=0.60,
        max_dist=0.64,
        name="rear_track_width",
    )
    ctx.expect_origin_distance(
        left_caster_fork,
        right_caster_fork,
        axes="x",
        min_dist=0.40,
        max_dist=0.48,
        name="caster_track_width",
    )
    ctx.expect_origin_gap(
        left_caster_fork,
        left_drive_wheel,
        axis="y",
        min_gap=0.28,
        max_gap=0.42,
        name="front_caster_leads_drive_wheel",
    )

    seat_aabb = ctx.part_element_world_aabb(main_frame, elem=seat_cushion)
    if seat_aabb is not None:
        seat_top = seat_aabb[1][2]
        ctx.check(
            "seat_height_realistic",
            0.49 <= seat_top <= 0.52,
            f"seat top is {seat_top:.3f} m",
        )
    else:
        ctx.fail("seat_height_realistic", "Seat cushion AABB was unavailable.")

    backrest_aabb = ctx.part_world_aabb(backrest)
    if backrest_aabb is not None:
        ctx.check(
            "overall_height_realistic",
            0.90 <= backrest_aabb[1][2] <= 0.98,
            f"backrest max z is {backrest_aabb[1][2]:.3f} m",
        )
    else:
        ctx.fail("overall_height_realistic", "Backrest AABB was unavailable.")

    left_rear_aabb = ctx.part_world_aabb(left_drive_wheel)
    left_front_aabb = ctx.part_world_aabb(left_caster_wheel)
    if left_rear_aabb is not None:
        ctx.check(
            "rear_wheel_ground_height",
            0.0 <= left_rear_aabb[0][2] <= 0.01,
            f"rear wheel min z is {left_rear_aabb[0][2]:.4f} m",
        )
    else:
        ctx.fail("rear_wheel_ground_height", "Rear wheel AABB was unavailable.")
    if left_front_aabb is not None:
        ctx.check(
            "caster_ground_height",
            -0.001 <= left_front_aabb[0][2] <= 0.01,
            f"caster wheel min z is {left_front_aabb[0][2]:.4f} m",
        )
    else:
        ctx.fail("caster_ground_height", "Caster wheel AABB was unavailable.")

    push_bar_rest = ctx.part_element_world_aabb(backrest, elem=back_push_bar)
    if push_bar_rest is not None and backrest_fold.motion_limits is not None and backrest_fold.motion_limits.upper is not None:
        with ctx.pose({backrest_fold: backrest_fold.motion_limits.upper}):
            push_bar_folded = ctx.part_element_world_aabb(backrest, elem=back_push_bar)
            ctx.fail_if_isolated_parts(name="backrest_fold_upper_no_floating")
            ctx.expect_contact(backrest, main_frame, contact_tol=0.001, name="backrest_fold_upper_contact")
            if push_bar_folded is not None:
                ctx.check(
                    "backrest_moves_forward_when_folded",
                    push_bar_folded[0][1] > push_bar_rest[0][1] + 0.18,
                    f"push bar y moved from {push_bar_rest[0][1]:.3f} to {push_bar_folded[0][1]:.3f}",
                )
            else:
                ctx.fail("backrest_moves_forward_when_folded", "Folded push bar AABB was unavailable.")
    else:
        ctx.fail("backrest_moves_forward_when_folded", "Backrest rest AABB or limits were unavailable.")

    left_hanger_rest = ctx.part_world_aabb(left_footrest_hanger)
    right_hanger_rest = ctx.part_world_aabb(right_footrest_hanger)
    if (
        left_hanger_rest is not None
        and right_hanger_rest is not None
        and left_footrest_swing.motion_limits is not None
        and right_footrest_swing.motion_limits is not None
        and left_footrest_swing.motion_limits.upper is not None
        and right_footrest_swing.motion_limits.upper is not None
    ):
        swing_pose = min(left_footrest_swing.motion_limits.upper, radians(6.0))
        with ctx.pose(
            {
                left_footrest_swing: swing_pose,
                right_footrest_swing: swing_pose,
            }
        ):
            left_hanger_swung = ctx.part_world_aabb(left_footrest_hanger)
            right_hanger_swung = ctx.part_world_aabb(right_footrest_hanger)
            ctx.fail_if_isolated_parts(name="footrests_swung_no_floating")
            ctx.expect_contact(left_footrest_hanger, main_frame, contact_tol=0.001, name="left_hanger_swung_contact")
            ctx.expect_contact(right_footrest_hanger, main_frame, contact_tol=0.001, name="right_hanger_swung_contact")
            ctx.expect_gap(
                left_caster_wheel,
                left_footplate,
                axis="x",
                min_gap=0.015,
                name="left_swung_plate_clear_of_caster",
            )
            ctx.expect_gap(
                right_footplate,
                right_caster_wheel,
                axis="x",
                min_gap=0.010,
                name="right_swung_plate_clear_of_caster",
            )
            if left_hanger_swung is not None and right_hanger_swung is not None:
                ctx.check(
                    "left_hanger_swings_outboard",
                    left_hanger_swung[1][0] > left_hanger_rest[1][0] + 0.003,
                    f"left hanger max x moved from {left_hanger_rest[1][0]:.3f} to {left_hanger_swung[1][0]:.3f}",
                )
                ctx.check(
                    "right_hanger_swings_outboard",
                    right_hanger_swung[0][0] < right_hanger_rest[0][0] - 0.003,
                    f"right hanger min x moved from {right_hanger_rest[0][0]:.3f} to {right_hanger_swung[0][0]:.3f}",
                )
            else:
                ctx.fail("left_hanger_swings_outboard", "Swung hanger AABBs were unavailable.")
                ctx.fail("right_hanger_swings_outboard", "Swung hanger AABBs were unavailable.")
    else:
        ctx.fail("left_hanger_swings_outboard", "Rest hanger AABBs or joint limits were unavailable.")
        ctx.fail("right_hanger_swings_outboard", "Rest hanger AABBs or joint limits were unavailable.")

    left_plate_rest = ctx.part_world_aabb(left_footplate)
    if (
        left_plate_rest is not None
        and left_footplate_fold.motion_limits is not None
        and left_footplate_fold.motion_limits.upper is not None
        and right_footplate_fold.motion_limits is not None
        and right_footplate_fold.motion_limits.upper is not None
    ):
        with ctx.pose(
            {
                left_footplate_fold: left_footplate_fold.motion_limits.upper,
                right_footplate_fold: right_footplate_fold.motion_limits.upper,
            }
        ):
            left_plate_folded = ctx.part_world_aabb(left_footplate)
            ctx.fail_if_isolated_parts(name="footplates_folded_no_floating")
            ctx.expect_contact(left_footplate, left_footrest_hanger, contact_tol=0.001, name="left_plate_folded_contact")
            ctx.expect_contact(right_footplate, right_footrest_hanger, contact_tol=0.001, name="right_plate_folded_contact")
            if left_plate_folded is not None:
                ctx.check(
                    "footplate_folds_up",
                    left_plate_folded[1][2] > left_plate_rest[1][2] + 0.09,
                    f"left footplate max z moved from {left_plate_rest[1][2]:.3f} to {left_plate_folded[1][2]:.3f}",
                )
            else:
                ctx.fail("footplate_folds_up", "Folded footplate AABB was unavailable.")
    else:
        ctx.fail("footplate_folds_up", "Rest footplate AABB or limits were unavailable.")

    brake_rest = ctx.part_world_aabb(left_brake_lever)
    if (
        brake_rest is not None
        and left_brake_swing.motion_limits is not None
        and left_brake_swing.motion_limits.upper is not None
        and right_brake_swing.motion_limits is not None
        and right_brake_swing.motion_limits.lower is not None
    ):
        with ctx.pose(
            {
                left_brake_swing: left_brake_swing.motion_limits.upper,
                right_brake_swing: right_brake_swing.motion_limits.lower,
            }
        ):
            brake_engaged = ctx.part_world_aabb(left_brake_lever)
            ctx.expect_contact(left_brake_lever, main_frame, contact_tol=0.001, name="left_brake_engaged_contact")
            ctx.expect_contact(right_brake_lever, main_frame, contact_tol=0.001, name="right_brake_engaged_contact")
            if brake_engaged is not None:
                ctx.check(
                    "brake_handle_moves",
                    brake_engaged[1][2] > brake_rest[1][2] + 0.005,
                    f"brake lever max z moved from {brake_rest[1][2]:.3f} to {brake_engaged[1][2]:.3f}",
                )
            else:
                ctx.fail("brake_handle_moves", "Brake lever engaged AABB was unavailable.")
    else:
        ctx.fail("brake_handle_moves", "Brake lever rest AABB or limits were unavailable.")

    wheel_marker_rest = ctx.part_element_world_aabb(left_drive_wheel, elem=rear_marker)
    if wheel_marker_rest is not None:
        with ctx.pose({left_rear_spin: pi / 2.0}):
            wheel_marker_spun = ctx.part_element_world_aabb(left_drive_wheel, elem=rear_marker)
            ctx.expect_contact(left_drive_wheel, main_frame, contact_tol=0.001, name="rear_wheel_spun_contact")
            if wheel_marker_spun is not None:
                ctx.check(
                    "rear_wheel_rotates",
                    abs(wheel_marker_spun[1][2] - wheel_marker_rest[1][2]) > 0.20,
                    f"wheel marker z changed from {wheel_marker_rest[1][2]:.3f} to {wheel_marker_spun[1][2]:.3f}",
                )
            else:
                ctx.fail("rear_wheel_rotates", "Spun rear wheel marker AABB was unavailable.")
    else:
        ctx.fail("rear_wheel_rotates", "Rear wheel marker AABB was unavailable.")

    caster_marker_rest = ctx.part_element_world_aabb(left_caster_wheel, elem=caster_marker)
    if caster_marker_rest is not None:
        with ctx.pose({left_caster_spin: pi / 2.0}):
            caster_marker_spun = ctx.part_element_world_aabb(left_caster_wheel, elem=caster_marker)
            ctx.expect_contact(left_caster_wheel, left_caster_fork, contact_tol=0.001, name="caster_wheel_spun_contact")
            if caster_marker_spun is not None:
                ctx.check(
                    "caster_wheel_rotates",
                    abs(caster_marker_spun[1][2] - caster_marker_rest[1][2]) > 0.04,
                    f"caster marker z changed from {caster_marker_rest[1][2]:.3f} to {caster_marker_spun[1][2]:.3f}",
                )
            else:
                ctx.fail("caster_wheel_rotates", "Spun caster marker AABB was unavailable.")
    else:
        ctx.fail("caster_wheel_rotates", "Caster marker AABB was unavailable.")

    caster_crown_rest = ctx.part_element_world_aabb(left_caster_fork, elem=caster_crown)
    if caster_crown_rest is not None:
        with ctx.pose({left_caster_swivel: radians(40.0)}):
            caster_crown_swiveled = ctx.part_element_world_aabb(left_caster_fork, elem=caster_crown)
            if caster_crown_swiveled is not None:
                ctx.check(
                    "caster_swivels",
                    caster_crown_swiveled[1][0] > caster_crown_rest[1][0] + 0.0005,
                    f"caster crown max x moved from {caster_crown_rest[1][0]:.3f} to {caster_crown_swiveled[1][0]:.3f}",
                )
            else:
                ctx.fail("caster_swivels", "Swiveled caster crown AABB was unavailable.")
    else:
        ctx.fail("caster_swivels", "Caster crown AABB was unavailable.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
