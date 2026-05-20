from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from uuid import uuid4

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
    wire_from_points,
)

_BUILD_TAG = uuid4().hex[:8]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, f"mk_scooter_{_BUILD_TAG}_{name}")


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _wire(name: str, points: list[tuple[float, float, float]], *, radius: float, corner_radius: float = 0.0):
    corner_mode = "fillet" if corner_radius > 0.0 else "miter"
    return _mesh(
        name,
        wire_from_points(
            points,
            radius=radius,
            radial_segments=18,
            cap_ends=True,
            corner_mode=corner_mode,
            corner_radius=corner_radius,
            corner_segments=8,
        ),
    )


def _spline_tube(name: str, points: list[tuple[float, float, float]], *, radius: float):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=14,
            radial_segments=20,
        ),
    )


def _add_wheel(part, *, wheel_radius: float, tire_width: float, boss_sign: float, rubber, aluminum, dark_steel) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=wheel_radius, length=tire_width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.72, length=tire_width * 0.76),
        origin=spin_origin,
        material=aluminum,
        name="rim",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.22, length=tire_width * 0.70),
        origin=spin_origin,
        material=dark_steel,
        name="hub",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.16, length=0.012),
        origin=Origin(xyz=(boss_sign * 0.028, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="inner_boss",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.18, length=0.010),
        origin=Origin(xyz=(-boss_sign * 0.023, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="outer_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_knee_scooter")

    frame_blue = model.material("frame_blue", rgba=(0.17, 0.43, 0.67, 1.0))
    black_foam = model.material("black_foam", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.18, 0.18, 0.18, 1.0))

    wheel_radius = 0.10
    tire_width = 0.045

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.44, 0.86, 0.55)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.02, 0.28)),
    )

    left_rail_points = [
        (0.07, -0.24, 0.19),
        (0.07, -0.08, 0.20),
        (0.06, 0.10, 0.19),
        (0.05, 0.20, 0.22),
    ]
    frame.visual(_spline_tube("left_frame_rail", left_rail_points, radius=0.018), material=frame_blue, name="left_rail")
    frame.visual(_spline_tube("right_frame_rail", _mirror_x(left_rail_points), radius=0.018), material=frame_blue, name="right_rail")
    frame.visual(
        Cylinder(radius=0.018, length=0.14),
        origin=Origin(xyz=(0.0, -0.24, 0.19), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_blue,
        name="rear_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.0, 0.20, 0.22), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_blue,
        name="front_crossbar",
    )
    frame.visual(
        Box((0.26, 0.12, 0.02)),
        origin=Origin(xyz=(0.0, 0.02, 0.46)),
        material=dark_steel,
        name="platform_support",
    )
    frame.visual(
        _wire("left_rear_platform_post", [(0.07, -0.08, 0.20), (0.12, -0.04, 0.45)], radius=0.014),
        material=frame_blue,
        name="left_rear_post",
    )
    frame.visual(
        _wire("left_front_platform_post", [(0.06, 0.10, 0.19), (0.12, 0.08, 0.45)], radius=0.014),
        material=frame_blue,
        name="left_front_post",
    )
    frame.visual(
        _wire("right_rear_platform_post", [(-0.07, -0.08, 0.20), (-0.12, -0.04, 0.45)], radius=0.014),
        material=frame_blue,
        name="right_rear_post",
    )
    frame.visual(
        _wire("right_front_platform_post", [(-0.06, 0.10, 0.19), (-0.12, 0.08, 0.45)], radius=0.014),
        material=frame_blue,
        name="right_front_post",
    )
    frame.visual(
        Box((0.016, 0.016, 0.16)),
        origin=Origin(xyz=(0.045, 0.20, 0.30)),
        material=frame_blue,
        name="left_head_strut",
    )
    frame.visual(
        Box((0.016, 0.016, 0.16)),
        origin=Origin(xyz=(-0.045, 0.20, 0.30)),
        material=frame_blue,
        name="right_head_strut",
    )
    frame.visual(
        Box((0.016, 0.10, 0.016)),
        origin=Origin(xyz=(0.042, 0.25, 0.38)),
        material=frame_blue,
        name="left_head_link",
    )
    frame.visual(
        Box((0.016, 0.10, 0.016)),
        origin=Origin(xyz=(-0.042, 0.25, 0.38)),
        material=frame_blue,
        name="right_head_link",
    )
    frame.visual(
        Box((0.014, 0.060, 0.16)),
        origin=Origin(xyz=(0.035, 0.280, 0.46)),
        material=dark_steel,
        name="head_left_cheek",
    )
    frame.visual(
        Box((0.014, 0.060, 0.16)),
        origin=Origin(xyz=(-0.035, 0.280, 0.46)),
        material=dark_steel,
        name="head_right_cheek",
    )
    frame.visual(
        Box((0.012, 0.060, 0.10)),
        origin=Origin(xyz=(0.120, -0.260, 0.10)),
        material=dark_steel,
        name="rear_left_mount",
    )
    frame.visual(
        Box((0.012, 0.060, 0.10)),
        origin=Origin(xyz=(-0.120, -0.260, 0.10)),
        material=dark_steel,
        name="rear_right_mount",
    )
    frame.visual(
        _wire("rear_left_support", [(0.07, -0.24, 0.19), (0.120, -0.250, 0.15)], radius=0.014),
        material=frame_blue,
        name="rear_left_support",
    )
    frame.visual(
        _wire("rear_right_support", [(-0.07, -0.24, 0.19), (-0.120, -0.250, 0.15)], radius=0.014),
        material=frame_blue,
        name="rear_right_support",
    )

    knee_platform = model.part("knee_platform")
    knee_platform.inertial = Inertial.from_geometry(
        Box((0.36, 0.18, 0.07)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )
    platform_base_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.36, 0.18, 0.024, corner_segments=10),
        0.02,
        cap=True,
        closed=True,
    )
    pad_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.34, 0.16, 0.040, corner_segments=12),
        0.05,
        cap=True,
        closed=True,
    )
    knee_platform.visual(
        _mesh("knee_platform_base", platform_base_geom),
        material=dark_steel,
        name="platform_base",
    )
    knee_platform.visual(
        _mesh("knee_platform_pad", pad_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=black_foam,
        name="pad",
    )

    steering_assembly = model.part("steering_assembly")
    steering_assembly.inertial = Inertial.from_geometry(
        Box((0.66, 0.14, 0.84)),
        mass=2.8,
        origin=Origin(xyz=(0.0, -0.01, 0.04)),
    )
    steering_assembly.visual(
        Cylinder(radius=0.028, length=0.13),
        origin=Origin(),
        material=aluminum,
        name="steer_bearing",
    )
    steering_assembly.visual(
        Cylinder(radius=0.014, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=aluminum,
        name="lower_stem",
    )
    steering_assembly.visual(
        Cylinder(radius=0.016, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.295)),
        material=aluminum,
        name="upper_stem",
    )
    steering_assembly.visual(
        Box((0.08, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.01, 0.44)),
        material=dark_steel,
        name="handlebar_clamp",
    )
    steering_assembly.visual(
        _spline_tube(
            "bicycle_handlebar",
            [
                (-0.28, -0.05, 0.42),
                (-0.18, -0.02, 0.44),
                (-0.06, 0.00, 0.45),
                (0.06, 0.00, 0.45),
                (0.18, -0.02, 0.44),
                (0.28, -0.05, 0.42),
            ],
            radius=0.013,
        ),
        material=aluminum,
        name="handlebar_bar",
    )
    steering_assembly.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.31, -0.055, 0.42), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_gray,
        name="right_grip",
    )
    steering_assembly.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.31, -0.055, 0.42), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_gray,
        name="left_grip",
    )
    steering_assembly.visual(
        Box((0.16, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.028, -0.270)),
        material=dark_steel,
        name="fork_crown",
    )
    steering_assembly.visual(
        Box((0.014, 0.018, 0.110)),
        origin=Origin(xyz=(0.075, 0.028, -0.330)),
        material=aluminum,
        name="left_fork_leg",
    )
    steering_assembly.visual(
        Box((0.014, 0.018, 0.110)),
        origin=Origin(xyz=(-0.075, 0.028, -0.330)),
        material=aluminum,
        name="right_fork_leg",
    )
    steering_assembly.visual(
        Box((0.012, 0.035, 0.050)),
        origin=Origin(xyz=(0.085, 0.035, -0.36)),
        material=dark_steel,
        name="left_dropout",
    )
    steering_assembly.visual(
        Box((0.012, 0.035, 0.050)),
        origin=Origin(xyz=(-0.085, 0.035, -0.36)),
        material=dark_steel,
        name="right_dropout",
    )

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=tire_width),
        mass=1.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel(
        front_left_wheel,
        wheel_radius=wheel_radius,
        tire_width=tire_width,
        boss_sign=-1.0,
        rubber=rubber,
        aluminum=aluminum,
        dark_steel=dark_steel,
    )

    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=tire_width),
        mass=1.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel(
        front_right_wheel,
        wheel_radius=wheel_radius,
        tire_width=tire_width,
        boss_sign=1.0,
        rubber=rubber,
        aluminum=aluminum,
        dark_steel=dark_steel,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=tire_width),
        mass=1.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel(
        rear_left_wheel,
        wheel_radius=wheel_radius,
        tire_width=tire_width,
        boss_sign=-1.0,
        rubber=rubber,
        aluminum=aluminum,
        dark_steel=dark_steel,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=tire_width),
        mass=1.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel(
        rear_right_wheel,
        wheel_radius=wheel_radius,
        tire_width=tire_width,
        boss_sign=1.0,
        rubber=rubber,
        aluminum=aluminum,
        dark_steel=dark_steel,
    )

    model.articulation(
        "platform_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=knee_platform,
        origin=Origin(xyz=(0.0, 0.02, 0.47)),
    )
    model.articulation(
        "head_tube_steer",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering_assembly,
        origin=Origin(xyz=(0.0, 0.28, 0.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.5, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_left_wheel,
        origin=Origin(xyz=(0.125, 0.035, -0.36)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_assembly,
        child=front_right_wheel,
        origin=Origin(xyz=(-0.125, 0.035, -0.36)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.160, -0.260, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.160, -0.260, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    knee_platform = object_model.get_part("knee_platform")
    steering_assembly = object_model.get_part("steering_assembly")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    head_tube_steer = object_model.get_articulation("head_tube_steer")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")

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
        "medical_knee_scooter_parts_present",
        len(object_model.parts) == 7,
        f"expected 7 parts but found {len(object_model.parts)}",
    )
    ctx.check(
        "steering_joint_axis_vertical",
        head_tube_steer.articulation_type == ArticulationType.REVOLUTE and tuple(head_tube_steer.axis) == (0.0, 0.0, 1.0),
        f"steering joint should be a vertical revolute axis, got {head_tube_steer.articulation_type} axis={head_tube_steer.axis}",
    )

    for joint_name, joint in (
        ("front_left_spin", front_left_spin),
        ("front_right_spin", front_right_spin),
        ("rear_left_spin", rear_left_spin),
        ("rear_right_spin", rear_right_spin),
    ):
        ctx.check(
            f"{joint_name}_continuous_x_axis",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"{joint_name} should spin continuously about the wheel axle, got {joint.articulation_type} axis={joint.axis}",
        )

    ctx.expect_contact(
        knee_platform,
        frame,
        elem_a="platform_base",
        elem_b="platform_support",
        name="platform_seated_on_support",
    )
    ctx.expect_gap(
        frame,
        steering_assembly,
        axis="x",
        positive_elem="head_left_cheek",
        negative_elem="steer_bearing",
        max_gap=0.0005,
        max_penetration=0.0,
        name="steering_bearing_seated_against_left_head_cheek",
    )
    ctx.expect_gap(
        steering_assembly,
        frame,
        axis="x",
        positive_elem="steer_bearing",
        negative_elem="head_right_cheek",
        max_gap=0.0005,
        max_penetration=0.0,
        name="steering_bearing_seated_against_right_head_cheek",
    )
    ctx.expect_contact(
        front_left_wheel,
        steering_assembly,
        elem_a="inner_boss",
        elem_b="left_dropout",
        name="front_left_wheel_mounted",
    )
    ctx.expect_contact(
        front_right_wheel,
        steering_assembly,
        elem_a="inner_boss",
        elem_b="right_dropout",
        name="front_right_wheel_mounted",
    )
    ctx.expect_contact(
        rear_left_wheel,
        frame,
        elem_a="inner_boss",
        elem_b="rear_left_mount",
        name="rear_left_wheel_mounted",
    )
    ctx.expect_contact(
        rear_right_wheel,
        frame,
        elem_a="inner_boss",
        elem_b="rear_right_mount",
        name="rear_right_wheel_mounted",
    )

    with ctx.pose({head_tube_steer: 0.0}):
        ctx.expect_origin_gap(
            front_left_wheel,
            rear_left_wheel,
            axis="y",
            min_gap=0.52,
            max_gap=0.62,
            name="wheelbase_realistic",
        )
        ctx.expect_origin_gap(
            front_left_wheel,
            front_right_wheel,
            axis="x",
            min_gap=0.24,
            max_gap=0.27,
            name="front_track_realistic",
        )
        ctx.expect_origin_gap(
            knee_platform,
            rear_left_wheel,
            axis="z",
            min_gap=0.34,
            max_gap=0.40,
            name="knee_platform_height_realistic",
        )

    pad_aabb = ctx.part_element_world_aabb(knee_platform, elem="pad")
    handlebar_aabb = ctx.part_element_world_aabb(steering_assembly, elem="handlebar_bar")
    if pad_aabb is None:
        ctx.fail("knee_pad_aabb_available", "named knee pad visual was not measurable")
    else:
        pad_top = pad_aabb[1][2]
        ctx.check(
            "knee_pad_top_height_realistic",
            0.51 <= pad_top <= 0.55,
            f"knee pad top should be about 0.53 m high, got {pad_top:.3f} m",
        )
    if handlebar_aabb is None:
        ctx.fail("handlebar_aabb_available", "named handlebar visual was not measurable")
    else:
        handlebar_top = handlebar_aabb[1][2]
        ctx.check(
            "handlebar_height_realistic",
            0.89 <= handlebar_top <= 0.95,
            f"handlebar top should be about 0.92 m high, got {handlebar_top:.3f} m",
        )

    limits = head_tube_steer.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({head_tube_steer: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_lower_no_floating")
        with ctx.pose({head_tube_steer: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="steering_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="steering_upper_no_floating")

    with ctx.pose(
        {
            head_tube_steer: 0.45,
            front_left_spin: pi / 3.0,
            front_right_spin: -pi / 4.0,
            rear_left_spin: pi / 2.0,
            rear_right_spin: -pi / 6.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="mixed_operating_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="mixed_operating_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
