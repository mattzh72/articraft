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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BARREL_LENGTH = 0.84
BARREL_OUTER_RADIUS = 0.24
BARREL_WALL = 0.012
BARREL_INNER_RADIUS = BARREL_OUTER_RADIUS - BARREL_WALL
LID_OUTER_RADIUS = BARREL_OUTER_RADIUS + 0.018
LID_INNER_RADIUS = LID_OUTER_RADIUS - BARREL_WALL
BARREL_CENTER_Z = 0.86
BARREL_FRONT_ANGLE = math.radians(40.0)
BARREL_REAR_ANGLE = math.radians(140.0)

HOPPER_CENTER_X = 0.58
HOPPER_OUTER_WIDTH = 0.24
HOPPER_OUTER_DEPTH = 0.22
HOPPER_WALL = 0.012
HOPPER_WALL_HEIGHT = 0.26
HOPPER_WALL_CENTER_Z = 0.86
HOPPER_TOP_Z = HOPPER_WALL_CENTER_Z + HOPPER_WALL_HEIGHT / 2.0
HOPPER_HINGE_Y = -(HOPPER_OUTER_DEPTH / 2.0 + 0.004)
HOPPER_HINGE_Z = HOPPER_TOP_Z + 0.005

WHEEL_RADIUS = 0.16
WHEEL_WIDTH = 0.085
WHEEL_CENTER_Z = 0.17
WHEEL_FRONT_Y = 0.17
WHEEL_REAR_Y = -0.17
WHEEL_AXIS_X = 0.55


def _arc_points(radius: float, start_angle: float, end_angle: float, *, segments: int) -> list[tuple[float, float]]:
    if end_angle <= start_angle:
        end_angle += math.tau
    return [
        (
            radius * math.cos(start_angle + (end_angle - start_angle) * index / segments),
            radius * math.sin(start_angle + (end_angle - start_angle) * index / segments),
        )
        for index in range(segments + 1)
    ]


def _ring_segment_profile(
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 28,
    z_offset: float = 0.0,
) -> list[tuple[float, float]]:
    outer = _arc_points(outer_radius, start_angle, end_angle, segments=segments)
    inner = list(reversed(_arc_points(inner_radius, start_angle, end_angle, segments=segments)))
    return [(y, z + z_offset) for y, z in outer + inner]


def _extrude_yz_profile_along_x(profile: list[tuple[float, float]], length: float):
    return ExtrudeGeometry(profile, length, center=True, cap=True, closed=True).rotate_y(math.pi / 2.0).rotate_x(
        math.pi / 2.0
    )


def _barrel_seam_point(angle: float) -> tuple[float, float]:
    return (BARREL_OUTER_RADIUS * math.cos(angle), BARREL_CENTER_Z + BARREL_OUTER_RADIUS * math.sin(angle))


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dy: float = 0.0,
    dz: float = 0.0,
) -> list[tuple[float, float]]:
    return [(y + dy, z + dz) for y, z in profile]


def _wheel_tire_mesh(name: str, radius: float, width: float):
    half_width = width * 0.5
    tire_profile = [
        (radius * 0.52, -half_width * 0.98),
        (radius * 0.72, -half_width),
        (radius * 0.90, -half_width * 0.84),
        (radius * 0.98, -half_width * 0.48),
        (radius, 0.0),
        (radius * 0.98, half_width * 0.48),
        (radius * 0.90, half_width * 0.84),
        (radius * 0.72, half_width),
        (radius * 0.52, half_width * 0.98),
        (radius * 0.42, half_width * 0.36),
        (radius * 0.38, 0.0),
        (radius * 0.42, -half_width * 0.36),
        (radius * 0.52, -half_width * 0.98),
    ]
    return mesh_from_geometry(LatheGeometry(tire_profile, segments=56).rotate_y(math.pi / 2.0), name)


def _c_clip_mesh(name: str, inner_radius: float, outer_radius: float, width: float):
    profile = _ring_segment_profile(
        outer_radius,
        inner_radius,
        math.radians(220.0),
        math.radians(500.0),
        segments=26,
    )
    return mesh_from_geometry(_extrude_yz_profile_along_x(profile, width), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pellet_grill")

    barrel_black = model.material("barrel_black", rgba=(0.17, 0.17, 0.17, 1.0))
    frame_black = model.material("frame_black", rgba=(0.12, 0.12, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.26, 0.28, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    lid_grey = model.material("lid_grey", rgba=(0.22, 0.23, 0.24, 1.0))
    controller_black = model.material("controller_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rear_hinge_y, rear_hinge_z = _barrel_seam_point(BARREL_REAR_ANGLE)
    front_hinge_y, front_hinge_z = _barrel_seam_point(BARREL_FRONT_ANGLE)

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((1.28, 0.62, 1.22)),
        mass=82.0,
        origin=Origin(xyz=(0.10, 0.0, 0.61)),
    )

    lower_chamber_profile = _ring_segment_profile(
        BARREL_OUTER_RADIUS,
        BARREL_INNER_RADIUS,
        BARREL_REAR_ANGLE,
        BARREL_FRONT_ANGLE,
        z_offset=BARREL_CENTER_Z,
    )
    lower_chamber_mesh = mesh_from_geometry(
        _extrude_yz_profile_along_x(lower_chamber_profile, BARREL_LENGTH),
        "lower_chamber_shell",
    )
    body.visual(lower_chamber_mesh, material=barrel_black, name="lower_chamber_shell")

    body.visual(
        Box((0.81, 0.036, 0.036)),
        origin=Origin(xyz=(0.03, 0.17, 0.56)),
        material=frame_black,
        name="front_upper_rail",
    )
    body.visual(
        Box((0.81, 0.036, 0.036)),
        origin=Origin(xyz=(0.03, -0.17, 0.56)),
        material=frame_black,
        name="rear_upper_rail",
    )
    body.visual(
        Box((0.90, 0.030, 0.030)),
        origin=Origin(xyz=(0.05, 0.19, 0.20)),
        material=frame_black,
        name="front_lower_rail",
    )
    body.visual(
        Box((0.90, 0.030, 0.030)),
        origin=Origin(xyz=(0.05, -0.19, 0.20)),
        material=frame_black,
        name="rear_lower_rail",
    )
    body.visual(
        Box((0.040, 0.040, 0.56)),
        origin=Origin(xyz=(-0.34, 0.17, 0.28)),
        material=frame_black,
        name="left_front_leg",
    )
    body.visual(
        Box((0.040, 0.040, 0.56)),
        origin=Origin(xyz=(-0.34, -0.17, 0.28)),
        material=frame_black,
        name="left_rear_leg",
    )
    body.visual(
        Box((0.040, 0.040, 0.46)),
        origin=Origin(xyz=(0.42, 0.17, 0.23)),
        material=frame_black,
        name="right_front_post",
    )
    body.visual(
        Box((0.040, 0.040, 0.46)),
        origin=Origin(xyz=(0.42, -0.17, 0.23)),
        material=frame_black,
        name="right_rear_post",
    )
    body.visual(
        Box((0.070, 0.060, 0.020)),
        origin=Origin(xyz=(-0.34, 0.17, 0.01)),
        material=dark_steel,
        name="left_front_foot",
    )
    body.visual(
        Box((0.070, 0.060, 0.020)),
        origin=Origin(xyz=(-0.34, -0.17, 0.01)),
        material=dark_steel,
        name="left_rear_foot",
    )
    body.visual(
        Box((0.100, 0.040, 0.040)),
        origin=Origin(xyz=(0.50, WHEEL_FRONT_Y, WHEEL_CENTER_Z)),
        material=frame_black,
        name="front_wheel_bracket",
    )
    body.visual(
        Box((0.100, 0.040, 0.040)),
        origin=Origin(xyz=(0.50, WHEEL_REAR_Y, WHEEL_CENTER_Z)),
        material=frame_black,
        name="rear_wheel_bracket",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(WHEEL_AXIS_X - 0.007, WHEEL_FRONT_Y, WHEEL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_axle_stub",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(WHEEL_AXIS_X - 0.007, WHEEL_REAR_Y, WHEEL_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_axle_stub",
    )
    body.visual(
        Box((0.78, 0.045, 0.012)),
        origin=Origin(xyz=(0.04, -0.12, 0.115)),
        material=dark_steel,
        name="lower_shelf_rear",
    )
    body.visual(
        Box((0.78, 0.045, 0.012)),
        origin=Origin(xyz=(0.04, 0.00, 0.115)),
        material=dark_steel,
        name="lower_shelf_mid",
    )
    body.visual(
        Box((0.78, 0.045, 0.012)),
        origin=Origin(xyz=(0.04, 0.12, 0.115)),
        material=dark_steel,
        name="lower_shelf_front",
    )
    body.visual(
        Box((0.024, 0.300, 0.020)),
        origin=Origin(xyz=(-0.34, 0.0, 0.115)),
        material=frame_black,
        name="left_shelf_stringer",
    )
    body.visual(
        Box((0.024, 0.300, 0.020)),
        origin=Origin(xyz=(0.42, 0.0, 0.115)),
        material=frame_black,
        name="right_shelf_stringer",
    )
    body.visual(
        Box((0.060, 0.280, 0.090)),
        origin=Origin(xyz=(-0.19, 0.0, 0.61)),
        material=frame_black,
        name="left_barrel_saddle",
    )
    body.visual(
        Box((0.060, 0.280, 0.090)),
        origin=Origin(xyz=(0.19, 0.0, 0.61)),
        material=frame_black,
        name="right_barrel_saddle",
    )
    body.visual(
        Box((0.080, 0.028, 0.060)),
        origin=Origin(xyz=(-0.19, 0.154, 0.585)),
        material=frame_black,
        name="left_front_saddle_arm",
    )
    body.visual(
        Box((0.080, 0.028, 0.060)),
        origin=Origin(xyz=(-0.19, -0.154, 0.585)),
        material=frame_black,
        name="left_rear_saddle_arm",
    )
    body.visual(
        Box((0.080, 0.028, 0.060)),
        origin=Origin(xyz=(0.19, 0.154, 0.585)),
        material=frame_black,
        name="right_front_saddle_arm",
    )
    body.visual(
        Box((0.080, 0.028, 0.060)),
        origin=Origin(xyz=(0.19, -0.154, 0.585)),
        material=frame_black,
        name="right_rear_saddle_arm",
    )
    body.visual(
        Box((0.180, 0.220, 0.120)),
        origin=Origin(xyz=(0.48, 0.0, 0.61)),
        material=frame_black,
        name="hopper_pedestal",
    )
    body.visual(
        Box((0.180, 0.120, 0.100)),
        origin=Origin(xyz=(0.41, 0.0, 0.72)),
        material=frame_black,
        name="auger_housing",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.74),
        origin=Origin(xyz=(0.0, rear_hinge_y, rear_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="main_hinge_rod",
    )

    body.visual(
        Box((HOPPER_OUTER_WIDTH, HOPPER_WALL, HOPPER_WALL_HEIGHT)),
        origin=Origin(xyz=(HOPPER_CENTER_X, HOPPER_OUTER_DEPTH / 2.0 - HOPPER_WALL / 2.0, HOPPER_WALL_CENTER_Z)),
        material=barrel_black,
        name="hopper_front_wall",
    )
    body.visual(
        Box((HOPPER_OUTER_WIDTH, HOPPER_WALL, HOPPER_WALL_HEIGHT)),
        origin=Origin(xyz=(HOPPER_CENTER_X, -(HOPPER_OUTER_DEPTH / 2.0 - HOPPER_WALL / 2.0), HOPPER_WALL_CENTER_Z)),
        material=barrel_black,
        name="hopper_rear_wall",
    )
    body.visual(
        Box((HOPPER_WALL, HOPPER_OUTER_DEPTH - 2.0 * HOPPER_WALL, HOPPER_WALL_HEIGHT)),
        origin=Origin(xyz=(HOPPER_CENTER_X - HOPPER_OUTER_WIDTH / 2.0 + HOPPER_WALL / 2.0, 0.0, HOPPER_WALL_CENTER_Z)),
        material=barrel_black,
        name="hopper_inner_wall",
    )
    body.visual(
        Box((HOPPER_WALL, HOPPER_OUTER_DEPTH - 2.0 * HOPPER_WALL, HOPPER_WALL_HEIGHT)),
        origin=Origin(xyz=(HOPPER_CENTER_X + HOPPER_OUTER_WIDTH / 2.0 - HOPPER_WALL / 2.0, 0.0, HOPPER_WALL_CENTER_Z)),
        material=barrel_black,
        name="hopper_outer_wall",
    )
    body.visual(
        Box((0.19, 0.19, 0.13)),
        origin=Origin(xyz=(HOPPER_CENTER_X, 0.0, 0.74)),
        material=frame_black,
        name="hopper_funnel_block",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.19),
        origin=Origin(xyz=(HOPPER_CENTER_X, HOPPER_HINGE_Y, HOPPER_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="hopper_hinge_rod",
    )
    body.visual(
        Box((0.026, 0.014, 0.022)),
        origin=Origin(xyz=(HOPPER_CENTER_X - 0.080, -0.107, 0.994)),
        material=barrel_black,
        name="hopper_left_hinge_support",
    )
    body.visual(
        Box((0.026, 0.014, 0.022)),
        origin=Origin(xyz=(HOPPER_CENTER_X + 0.080, -0.107, 0.994)),
        material=barrel_black,
        name="hopper_right_hinge_support",
    )
    body.visual(
        Box((0.060, 0.100, 0.085)),
        origin=Origin(xyz=(HOPPER_CENTER_X + 0.145, 0.0, 0.79)),
        material=controller_black,
        name="controller_box",
    )
    main_lid = model.part("main_lid")
    lid_profile = _shift_profile(
        _ring_segment_profile(
            LID_OUTER_RADIUS,
            LID_INNER_RADIUS,
            BARREL_FRONT_ANGLE,
            BARREL_REAR_ANGLE,
            segments=28,
            z_offset=BARREL_CENTER_Z,
        ),
        dy=-rear_hinge_y,
        dz=-rear_hinge_z,
    )
    lid_shell_mesh = mesh_from_geometry(_extrude_yz_profile_along_x(lid_profile, BARREL_LENGTH), "main_lid_shell")
    main_lid.visual(lid_shell_mesh, material=lid_grey, name="lid_shell")
    main_lid.visual(
        Box((0.032, 0.016, 0.016)),
        origin=Origin(xyz=(-0.27, -0.008, 0.014)),
        material=lid_grey,
        name="left_hinge_block",
    )
    main_lid.visual(
        Box((0.032, 0.016, 0.016)),
        origin=Origin(xyz=(0.27, -0.008, 0.014)),
        material=lid_grey,
        name="right_hinge_block",
    )
    main_lid.visual(
        Box((0.032, 0.020, 0.030)),
        origin=Origin(xyz=(-0.27, 0.006, 0.018)),
        material=lid_grey,
        name="left_hinge_strap",
    )
    main_lid.visual(
        Box((0.032, 0.020, 0.030)),
        origin=Origin(xyz=(0.27, 0.006, 0.018)),
        material=lid_grey,
        name="right_hinge_strap",
    )
    main_lid.visual(
        Box((0.028, 0.028, 0.084)),
        origin=Origin(xyz=(-0.19, 0.285, 0.042)),
        material=brushed_steel,
        name="left_handle_post",
    )
    main_lid.visual(
        Box((0.028, 0.028, 0.084)),
        origin=Origin(xyz=(0.19, 0.285, 0.042)),
        material=brushed_steel,
        name="right_handle_post",
    )
    main_lid.visual(
        Cylinder(radius=0.011, length=0.38),
        origin=Origin(xyz=(0.0, 0.297, 0.084), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="handle_bar",
    )
    main_lid.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.18, 0.100)),
        material=brushed_steel,
        name="thermometer_bezel",
    )
    main_lid.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(xyz=(0.0, 0.18, 0.089)),
        material=dark_steel,
        name="thermometer_stem",
    )
    main_lid.inertial = Inertial.from_geometry(
        Box((BARREL_LENGTH, 0.40, 0.18)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.19, 0.08)),
    )

    hopper_lid = model.part("hopper_lid")
    hopper_lid.visual(
        Box((0.252, 0.236, 0.012)),
        origin=Origin(xyz=(0.0, 0.118, 0.014)),
        material=lid_grey,
        name="top_panel",
    )
    hopper_lid.visual(
        Box((0.012, 0.220, 0.024)),
        origin=Origin(xyz=(-0.126, 0.114, -0.001)),
        material=lid_grey,
        name="left_skirt",
    )
    hopper_lid.visual(
        Box((0.012, 0.220, 0.024)),
        origin=Origin(xyz=(0.126, 0.114, -0.001)),
        material=lid_grey,
        name="right_skirt",
    )
    hopper_lid.visual(
        Box((0.228, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.230, -0.004)),
        material=lid_grey,
        name="front_lip",
    )
    hopper_lid.visual(
        Box((0.024, 0.016, 0.016)),
        origin=Origin(xyz=(-0.078, -0.008, 0.014)),
        material=lid_grey,
        name="left_hinge_block",
    )
    hopper_lid.visual(
        Box((0.024, 0.016, 0.016)),
        origin=Origin(xyz=(0.078, -0.008, 0.014)),
        material=lid_grey,
        name="right_hinge_block",
    )
    hopper_lid.visual(
        Box((0.164, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.005, 0.015)),
        material=lid_grey,
        name="rear_header",
    )
    hopper_lid.inertial = Inertial.from_geometry(
        Box((0.27, 0.24, 0.05)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.11, 0.01)),
    )

    front_wheel = model.part("front_wheel")
    wheel_tire = _wheel_tire_mesh("pellet_grill_wheel_tire", WHEEL_RADIUS, WHEEL_WIDTH)
    front_wheel.visual(
        wheel_tire,
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.072, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="inner_washer",
    )
    front_wheel.visual(
        Cylinder(radius=0.110, length=0.014),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="inner_rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.110, length=0.014),
        origin=Origin(xyz=(0.088, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="outer_rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.048, length=0.050),
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=4.8,
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        wheel_tire,
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.072, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="inner_washer",
    )
    rear_wheel.visual(
        Cylinder(radius=0.110, length=0.014),
        origin=Origin(xyz=(0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="inner_rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.110, length=0.014),
        origin=Origin(xyz=(0.088, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="outer_rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.048, length=0.050),
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=4.8,
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_main_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=main_lid,
        origin=Origin(xyz=(0.0, rear_hinge_y, rear_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )
    model.articulation(
        "body_to_hopper_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hopper_lid,
        origin=Origin(xyz=(HOPPER_CENTER_X, HOPPER_HINGE_Y, HOPPER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "body_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=front_wheel,
        origin=Origin(xyz=(WHEEL_AXIS_X, WHEEL_FRONT_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "body_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rear_wheel,
        origin=Origin(xyz=(WHEEL_AXIS_X, WHEEL_REAR_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    main_lid = object_model.get_part("main_lid")
    hopper_lid = object_model.get_part("hopper_lid")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    main_hinge = object_model.get_articulation("body_to_main_lid")
    hopper_hinge = object_model.get_articulation("body_to_hopper_lid")
    front_spin = object_model.get_articulation("body_to_front_wheel")
    rear_spin = object_model.get_articulation("body_to_rear_wheel")

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

    def _axis_x(joint) -> bool:
        return tuple(round(value, 6) for value in joint.axis) == (1.0, 0.0, 0.0)

    ctx.check(
        "main_lid_joint_is_rear_horizontal_hinge",
        main_hinge.articulation_type == ArticulationType.REVOLUTE
        and _axis_x(main_hinge)
        and main_hinge.motion_limits is not None
        and main_hinge.motion_limits.lower == 0.0
        and main_hinge.motion_limits.upper is not None
        and main_hinge.motion_limits.upper >= math.radians(75.0),
        "Main lid should hinge about the rear horizontal barrel axis.",
    )
    ctx.check(
        "hopper_lid_joint_is_horizontal_hinge",
        hopper_hinge.articulation_type == ArticulationType.REVOLUTE
        and _axis_x(hopper_hinge)
        and hopper_hinge.motion_limits is not None
        and hopper_hinge.motion_limits.lower == 0.0,
        "Hopper refill lid should rotate on its own horizontal hinge line.",
    )
    ctx.check(
        "wheel_joints_spin_about_axles",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and _axis_x(front_spin)
        and _axis_x(rear_spin),
        "Both transport wheels should spin continuously on x-axis axle mounts.",
    )

    ctx.expect_gap(
        main_lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="lower_chamber_shell",
        min_gap=0.0,
        max_gap=0.008,
        name="main_lid_close_clearance_over_chamber",
    )
    ctx.expect_overlap(
        main_lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="lower_chamber_shell",
        min_overlap=0.25,
        name="main_lid_covers_barrel_opening",
    )
    ctx.expect_contact(
        main_lid,
        body,
        elem_a="left_hinge_block",
        elem_b="main_hinge_rod",
        name="main_lid_left_hinge_block_wraps_rod",
    )
    ctx.expect_contact(
        main_lid,
        body,
        elem_a="right_hinge_block",
        elem_b="main_hinge_rod",
        name="main_lid_right_hinge_block_wraps_rod",
    )
    ctx.expect_contact(
        hopper_lid,
        body,
        elem_a="left_hinge_block",
        elem_b="hopper_hinge_rod",
        name="hopper_lid_left_clip_engages_rod",
    )
    ctx.expect_contact(
        hopper_lid,
        body,
        elem_a="right_hinge_block",
        elem_b="hopper_hinge_rod",
        name="hopper_lid_right_clip_engages_rod",
    )
    ctx.expect_contact(
        front_wheel,
        body,
        elem_a="inner_washer",
        elem_b="front_axle_stub",
        name="front_wheel_mounted_to_axle",
    )
    ctx.expect_contact(
        rear_wheel,
        body,
        elem_a="inner_washer",
        elem_b="rear_axle_stub",
        name="rear_wheel_mounted_to_axle",
    )
    ctx.expect_origin_gap(
        front_wheel,
        rear_wheel,
        axis="y",
        min_gap=0.30,
        max_gap=0.38,
        name="transport_wheels_are_spaced_front_to_back",
    )
    ctx.expect_origin_distance(
        front_wheel,
        rear_wheel,
        axes="z",
        max_dist=0.01,
        name="transport_wheels_share_axle_height",
    )

    lid_handle_rest = ctx.part_element_world_aabb(main_lid, elem="handle_bar")
    hopper_front_rest = ctx.part_element_world_aabb(hopper_lid, elem="front_lip")
    assert lid_handle_rest is not None
    assert hopper_front_rest is not None

    with ctx.pose({main_hinge: math.radians(75.0)}):
        lid_handle_open = ctx.part_element_world_aabb(main_lid, elem="handle_bar")
        assert lid_handle_open is not None
        ctx.check(
            "main_lid_opens_upward",
            lid_handle_open[1][2] > lid_handle_rest[1][2] + 0.14,
            "Main lid handle did not rise enough in the open pose.",
        )
        ctx.expect_contact(
            main_lid,
            body,
            elem_a="left_hinge_block",
            elem_b="main_hinge_rod",
            name="main_lid_left_hinge_stays_on_rod_when_open",
        )
        ctx.expect_contact(
            main_lid,
            body,
            elem_a="right_hinge_block",
            elem_b="main_hinge_rod",
            name="main_lid_right_hinge_stays_on_rod_when_open",
        )

    with ctx.pose({hopper_hinge: math.radians(95.0)}):
        hopper_front_open = ctx.part_element_world_aabb(hopper_lid, elem="front_lip")
        assert hopper_front_open is not None
        ctx.check(
            "hopper_lid_front_edge_lifts_when_open",
            hopper_front_open[1][2] > hopper_front_rest[1][2] + 0.10,
            "Hopper refill lid did not lift clearly away from the hopper opening.",
        )
        ctx.expect_contact(
            hopper_lid,
            body,
            elem_a="left_hinge_block",
            elem_b="hopper_hinge_rod",
            name="hopper_left_clip_stays_attached_when_open",
        )
        ctx.expect_contact(
            hopper_lid,
            body,
            elem_a="right_hinge_block",
            elem_b="hopper_hinge_rod",
            name="hopper_right_clip_stays_attached_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
