from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _build_reflector_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.050, -0.020),
            (0.130, 0.030),
            (0.310, 0.130),
            (0.490, 0.255),
            (0.580, 0.345),
            (0.602, 0.382),
        ],
        [
            (0.028, -0.002),
            (0.108, 0.040),
            (0.286, 0.138),
            (0.462, 0.258),
            (0.556, 0.346),
            (0.578, 0.372),
        ],
        segments=80,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )
    shell.rotate_y(-math.pi / 2.0).translate(0.46, 0.0, 0.0)
    shell.merge(
        TorusGeometry(
            radius=0.586,
            tube=0.012,
            radial_segments=18,
            tubular_segments=72,
        )
        .rotate_y(math.pi / 2.0)
        .translate(0.842, 0.0, 0.0)
    )
    return shell


def _build_handwheel_body_mesh(*, direction: float) -> MeshGeometry:
    body = MeshGeometry()
    wheel_center_y = direction * 0.090
    body.merge(
        TorusGeometry(
            radius=0.095,
            tube=0.012,
            radial_segments=16,
            tubular_segments=48,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, wheel_center_y, 0.0)
    )
    body.merge(
        CylinderGeometry(radius=0.026, height=0.030, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, direction * 0.070, 0.0)
    )
    spoke = BoxGeometry((0.110, 0.010, 0.014))
    for angle in (0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0):
        body.merge(spoke.copy().rotate_y(angle).translate(0.0, wheel_center_y, 0.0))
    return body


def _build_rear_support_ribs_mesh() -> MeshGeometry:
    rib_paths = (
        ((0.06, 0.0, 0.055), (0.22, 0.0, 0.085), (0.48, 0.0, 0.128)),
        ((0.06, 0.0, -0.055), (0.22, 0.0, -0.085), (0.48, 0.0, -0.128)),
        ((0.06, 0.055, 0.0), (0.22, 0.085, 0.0), (0.48, 0.128, 0.0)),
        ((0.06, -0.055, 0.0), (0.22, -0.085, 0.0), (0.48, -0.128, 0.0)),
    )
    ribs = MeshGeometry()
    for path in rib_paths:
        ribs.merge(
            tube_from_spline_points(
                path,
                radius=0.016,
                samples_per_segment=12,
                radial_segments=16,
            )
        )
    return ribs


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_tracking_dish")

    base_paint = model.material("base_paint", rgba=(0.27, 0.31, 0.26, 1.0))
    yoke_paint = model.material("yoke_paint", rgba=(0.24, 0.28, 0.24, 1.0))
    reflector_paint = model.material("reflector_paint", rgba=(0.82, 0.84, 0.80, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.13, 0.14, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.58, 0.62, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.320, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_paint,
        name="ground_base",
    )
    base.visual(
        Cylinder(radius=0.220, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=base_paint,
        name="lower_plinth",
    )
    base.visual(
        Cylinder(radius=0.160, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=base_paint,
        name="transition_collar",
    )
    base.visual(
        Cylinder(radius=0.130, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.495)),
        material=base_paint,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.250, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material=base_paint,
        name="azimuth_bearing_housing",
    )
    base.visual(
        Box((0.120, 0.140, 0.100)),
        origin=Origin(xyz=(0.0, -0.320, 0.620)),
        material=dark_hardware,
        name="azimuth_drive_box",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.100),
        origin=Origin(xyz=(0.0, -0.440, 0.620), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="azimuth_wheel_boss",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.64, 0.64, 0.74)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.245, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=yoke_paint,
        name="turntable",
    )
    yoke.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=yoke_paint,
        name="turntable_hub",
    )
    yoke.visual(
        Box((0.180, 0.180, 0.220)),
        origin=Origin(xyz=(-0.100, -0.300, 0.160)),
        material=yoke_paint,
        name="negative_lower_cheek",
    )
    yoke.visual(
        Box((0.180, 0.180, 0.220)),
        origin=Origin(xyz=(-0.100, 0.300, 0.160)),
        material=yoke_paint,
        name="positive_lower_cheek",
    )
    yoke.visual(
        Box((0.300, 0.040, 0.880)),
        origin=Origin(xyz=(-0.100, -0.390, 0.500)),
        material=yoke_paint,
        name="negative_side_plate",
    )
    yoke.visual(
        Box((0.300, 0.040, 0.880)),
        origin=Origin(xyz=(-0.100, 0.390, 0.500)),
        material=yoke_paint,
        name="positive_side_plate",
    )
    yoke.visual(
        Box((0.080, 0.780, 0.090)),
        origin=Origin(xyz=(-0.240, 0.0, 0.780)),
        material=yoke_paint,
        name="rear_crossbeam",
    )
    yoke.visual(
        Box((0.060, 0.660, 0.060)),
        origin=Origin(xyz=(-0.180, 0.0, 0.240)),
        material=dark_hardware,
        name="lower_tie",
    )
    yoke.visual(
        Box((0.220, 0.140, 0.220)),
        origin=Origin(xyz=(-0.090, 0.0, 0.170)),
        material=dark_hardware,
        name="central_spine",
    )
    yoke.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, -0.370, 0.640), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="negative_bearing_boss",
    )
    yoke.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.370, 0.640), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="positive_bearing_boss",
    )
    yoke.visual(
        Cylinder(radius=0.035, length=0.150),
        origin=Origin(xyz=(-0.020, 0.480, 0.600), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elevation_wheel_boss",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.48, 0.86, 0.96)),
        mass=21.0,
        origin=Origin(xyz=(-0.080, 0.0, 0.480)),
    )

    dish = model.part("dish")
    dish.visual(
        mesh_from_geometry(_build_reflector_shell_mesh(), "reflector_shell"),
        material=reflector_paint,
        name="reflector_shell",
    )
    dish.visual(
        Cylinder(radius=0.090, length=0.100),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_hardware,
        name="rear_hub",
    )
    dish.visual(
        Cylinder(radius=0.022, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="trunnion_cross_tube",
    )
    dish.visual(
        mesh_from_geometry(_build_rear_support_ribs_mesh(), "rear_support_ribs"),
        material=dark_hardware,
        name="rear_support_ribs",
    )
    dish.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, -0.300, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="negative_trunnion_hub",
    )
    dish.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, 0.300, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="positive_trunnion_hub",
    )
    dish.inertial = Inertial.from_geometry(
        Box((0.48, 1.22, 1.22)),
        mass=26.0,
        origin=Origin(xyz=(0.540, 0.0, 0.0)),
    )

    azimuth_handwheel = model.part("azimuth_handwheel")
    azimuth_handwheel.visual(
        mesh_from_geometry(
            _build_handwheel_body_mesh(direction=-1.0),
            "azimuth_handwheel_body",
        ),
        material=dark_hardware,
        name="wheel_body",
    )
    azimuth_handwheel.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0, -0.0275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drive_shaft",
    )
    azimuth_handwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.120),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    elevation_handwheel = model.part("elevation_handwheel")
    elevation_handwheel.visual(
        mesh_from_geometry(
            _build_handwheel_body_mesh(direction=1.0),
            "elevation_handwheel_body",
        ),
        material=dark_hardware,
        name="wheel_body",
    )
    elevation_handwheel.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0, 0.0275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drive_shaft",
    )
    elevation_handwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.120),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.8,
            lower=-2.9,
            upper=2.9,
        ),
    )
    model.articulation(
        "yoke_to_dish",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=dish,
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.8,
            lower=math.radians(-12.0),
            upper=math.radians(92.0),
        ),
    )
    model.articulation(
        "base_to_azimuth_handwheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=azimuth_handwheel,
        origin=Origin(xyz=(0.0, -0.490, 0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )
    model.articulation(
        "yoke_to_elevation_handwheel",
        ArticulationType.CONTINUOUS,
        parent=yoke,
        child=elevation_handwheel,
        origin=Origin(xyz=(-0.020, 0.555, 0.600)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    dish = object_model.get_part("dish")
    azimuth_handwheel = object_model.get_part("azimuth_handwheel")
    elevation_handwheel = object_model.get_part("elevation_handwheel")

    azimuth = object_model.get_articulation("base_to_yoke")
    elevation = object_model.get_articulation("yoke_to_dish")
    azimuth_wheel_spin = object_model.get_articulation("base_to_azimuth_handwheel")
    elevation_wheel_spin = object_model.get_articulation("yoke_to_elevation_handwheel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        yoke,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable_seats_on_bearing_housing",
    )
    ctx.expect_overlap(
        yoke,
        base,
        axes="xy",
        min_overlap=0.220,
        name="turntable_over_bearing_housing",
    )
    ctx.expect_contact(
        dish,
        yoke,
        elem_a="negative_trunnion_hub",
        elem_b="negative_bearing_boss",
        name="negative_trunnion_clipped_in_yoke",
    )
    ctx.expect_contact(
        dish,
        yoke,
        elem_a="positive_trunnion_hub",
        elem_b="positive_bearing_boss",
        name="positive_trunnion_clipped_in_yoke",
    )
    ctx.expect_contact(dish, yoke, name="dish_supported_by_yoke")
    ctx.expect_contact(
        azimuth_handwheel,
        base,
        elem_a="drive_shaft",
        elem_b="azimuth_wheel_boss",
        name="azimuth_wheel_shaft_seated",
    )
    ctx.expect_contact(
        elevation_handwheel,
        yoke,
        elem_a="drive_shaft",
        elem_b="elevation_wheel_boss",
        name="elevation_wheel_shaft_seated",
    )
    ctx.expect_contact(azimuth_handwheel, base, name="azimuth_handwheel_mounted")
    ctx.expect_contact(elevation_handwheel, yoke, name="elevation_handwheel_mounted")

    ctx.check(
        "azimuth_axis_is_vertical",
        tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical axis, got {azimuth.axis}",
    )
    ctx.check(
        "elevation_axis_is_horizontal",
        abs(float(elevation.axis[0])) < 1e-9
        and abs(abs(float(elevation.axis[1])) - 1.0) < 1e-9
        and abs(float(elevation.axis[2])) < 1e-9,
        details=f"expected pure y-axis elevation pivot, got {elevation.axis}",
    )
    ctx.check(
        "handwheel_axes_are_local_drive_axes",
        abs(float(azimuth_wheel_spin.axis[0])) < 1e-9
        and abs(abs(float(azimuth_wheel_spin.axis[1])) - 1.0) < 1e-9
        and abs(float(azimuth_wheel_spin.axis[2])) < 1e-9
        and abs(float(elevation_wheel_spin.axis[0])) < 1e-9
        and abs(abs(float(elevation_wheel_spin.axis[1])) - 1.0) < 1e-9
        and abs(float(elevation_wheel_spin.axis[2])) < 1e-9,
        details=(
            f"azimuth wheel axis={azimuth_wheel_spin.axis}, "
            f"elevation wheel axis={elevation_wheel_spin.axis}"
        ),
    )
    ctx.check(
        "tracking_joint_limits_are_realistic",
        azimuth.motion_limits is not None
        and elevation.motion_limits is not None
        and azimuth.motion_limits.lower is not None
        and azimuth.motion_limits.upper is not None
        and elevation.motion_limits.lower is not None
        and elevation.motion_limits.upper is not None
        and azimuth.motion_limits.lower <= -2.8
        and azimuth.motion_limits.upper >= 2.8
        and elevation.motion_limits.lower < 0.0
        and elevation.motion_limits.upper > math.radians(85.0),
        details="expected broad azimuth sweep and near-zenith elevation travel",
    )

    reflector_rest = ctx.part_element_world_aabb(dish, elem="reflector_shell")
    assert reflector_rest is not None

    with ctx.pose({azimuth: math.radians(50.0)}):
        ctx.expect_contact(yoke, base, name="turntable_stays_carried_in_azimuth_pose")

    with ctx.pose({elevation: math.radians(60.0)}):
        reflector_raised = ctx.part_element_world_aabb(dish, elem="reflector_shell")
        assert reflector_raised is not None
        ctx.expect_contact(dish, yoke, name="dish_stays_supported_while_pitched")
        ctx.check(
            "reflector_rises_when_elevated",
            reflector_raised[1][2] > reflector_rest[1][2] + 0.15,
            details=(
                f"rest top z={reflector_rest[1][2]:.3f}, "
                f"raised top z={reflector_raised[1][2]:.3f}"
            ),
        )

    with ctx.pose(
        {
            azimuth_wheel_spin: math.radians(70.0),
            elevation_wheel_spin: math.radians(-85.0),
        }
    ):
        ctx.expect_contact(
            azimuth_handwheel,
            base,
            name="azimuth_handwheel_keeps_mount_contact_when_turned",
        )
        ctx.expect_contact(
            elevation_handwheel,
            yoke,
            name="elevation_handwheel_keeps_mount_contact_when_turned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
