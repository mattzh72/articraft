from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(x_pos: float, width_y: float, height_z: float, radius: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for y, z in rounded_rect_profile(width_y, height_z, radius=radius, corner_segments=8)]


def _arm_beam_mesh(length: float):
    return sweep_profile_along_spline(
        [
            (0.008, 0.0, 0.000),
            (0.090, 0.0, 0.003),
            (0.200, 0.0, 0.007),
            (length, 0.0, 0.012),
        ],
        profile=rounded_rect_profile(0.030, 0.018, radius=0.004, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
    )


def _propeller_mesh(radius: float):
    blade_profile = rounded_rect_profile(0.024, 0.0036, radius=0.0009, corner_segments=4)
    blade = sweep_profile_along_spline(
        [
            (0.006, 0.0, 0.0005),
            (0.050, 0.009, 0.0035),
            (0.102, 0.018, 0.0020),
            (radius, 0.023, -0.0010),
        ],
        profile=blade_profile,
        samples_per_segment=12,
        cap_profile=True,
    )
    opposite_blade = blade.copy().rotate_z(pi)
    blade.merge(opposite_blade)
    return blade


def _camera_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_quadcopter")

    body_paint = model.material("body_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    carbon = model.material("carbon", rgba=(0.11, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.06, 0.06, 0.07, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.16, 0.17, 0.18, 1.0))
    motor_metal = model.material("motor_metal", rgba=(0.46, 0.48, 0.50, 1.0))
    prop_black = model.material("prop_black", rgba=(0.07, 0.08, 0.09, 1.0))
    camera_black = model.material("camera_black", rgba=(0.05, 0.05, 0.06, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.10, 0.16, 0.20, 0.70))

    arm_beam_mesh = _mesh("folding_arm_beam", _arm_beam_mesh(0.305))
    propeller_blades_mesh = _mesh("cinema_propeller_blades", _propeller_mesh(0.155))
    landing_gear_mesh = _mesh(
        "cinema_landing_gear",
        tube_from_spline_points(
            [
                (-0.100, 0.0, 0.000),
                (-0.100, 0.0, -0.050),
                (-0.088, 0.0, -0.125),
                (-0.035, 0.0, -0.178),
                (0.090, 0.0, -0.178),
                (0.135, 0.0, -0.128),
                (0.148, 0.0, -0.055),
                (0.148, 0.0, 0.000),
            ],
            radius=0.010,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    body = model.part("body")
    body_shell = _mesh(
        "cinema_body_shell",
        section_loft(
            [
                _yz_section(-0.220, 0.152, 0.055, 0.016),
                _yz_section(-0.120, 0.206, 0.085, 0.022),
                _yz_section(0.040, 0.224, 0.092, 0.022),
                _yz_section(0.190, 0.170, 0.072, 0.018),
                _yz_section(0.245, 0.108, 0.046, 0.010),
            ]
        ),
    )
    body.visual(body_shell, material=body_paint, name="body_shell")
    body.visual(
        Box((0.210, 0.125, 0.030)),
        origin=Origin(xyz=(-0.030, 0.0, 0.058)),
        material=dark_gray,
        name="battery_hatch",
    )
    body.visual(
        Box((0.100, 0.040, 0.018)),
        origin=Origin(xyz=(0.248, 0.0, 0.008)),
        material=matte_black,
        name="nose_sensor_bar",
    )
    body.visual(
        Box((0.090, 0.060, 0.012)),
        origin=Origin(xyz=(0.152, 0.0, -0.046)),
        material=dark_gray,
        name="gimbal_mount",
    )
    for x_pos in (0.115, -0.115):
        for y_pos in (0.108, -0.108):
            body.visual(
                Box((0.052, 0.028, 0.022)),
                origin=Origin(xyz=(x_pos, y_pos, 0.028)),
                material=dark_gray,
            )
    for y_pos in (0.102, -0.102):
        body.visual(
            Box((0.270, 0.018, 0.012)),
            origin=Origin(xyz=(0.020, y_pos, -0.040)),
            material=dark_gray,
        )
    body.inertial = Inertial.from_geometry(
        Box((0.520, 0.280, 0.160)),
        mass=5.8,
        origin=Origin(),
    )

    left_landing_gear = model.part("left_landing_gear")
    left_landing_gear.visual(landing_gear_mesh, material=carbon, name="left_gear_loop")
    for x_pos in (-0.100, 0.148):
        left_landing_gear.visual(
            Box((0.018, 0.014, 0.020)),
            origin=Origin(xyz=(x_pos, 0.0, 0.000)),
            material=dark_gray,
        )
    left_landing_gear.inertial = Inertial.from_geometry(
        Box((0.260, 0.028, 0.190)),
        mass=0.35,
        origin=Origin(xyz=(0.024, 0.0, -0.089)),
    )

    right_landing_gear = model.part("right_landing_gear")
    right_landing_gear.visual(landing_gear_mesh, material=carbon, name="right_gear_loop")
    for x_pos in (-0.100, 0.148):
        right_landing_gear.visual(
            Box((0.018, 0.014, 0.020)),
            origin=Origin(xyz=(x_pos, 0.0, 0.000)),
            material=dark_gray,
        )
    right_landing_gear.inertial = Inertial.from_geometry(
        Box((0.260, 0.028, 0.190)),
        mass=0.35,
        origin=Origin(xyz=(0.024, 0.0, -0.089)),
    )

    def add_arm(name: str):
        arm = model.part(name)
        arm.visual(
            Cylinder(radius=0.012, length=0.026),
            material=motor_metal,
            name="hinge_barrel",
        )
        arm.visual(
            Box((0.040, 0.028, 0.018)),
            origin=Origin(xyz=(0.018, 0.0, 0.000)),
            material=dark_gray,
            name="hinge_knuckle",
        )
        arm.visual(arm_beam_mesh, material=carbon, name="arm_beam")
        arm.visual(
            Cylinder(radius=0.020, length=0.028),
            origin=Origin(xyz=(0.305, 0.0, 0.014)),
            material=motor_metal,
            name="motor_bell",
        )
        arm.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.305, 0.0, 0.034)),
            material=matte_black,
            name="motor_cap",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.350, 0.050, 0.070)),
            mass=0.52,
            origin=Origin(xyz=(0.170, 0.0, 0.015)),
        )
        return arm

    front_left_arm = add_arm("front_left_arm")
    front_right_arm = add_arm("front_right_arm")
    rear_left_arm = add_arm("rear_left_arm")
    rear_right_arm = add_arm("rear_right_arm")

    def add_propeller(name: str):
        propeller = model.part(name)
        propeller.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=motor_metal,
            name="hub",
        )
        propeller.visual(
            propeller_blades_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=prop_black,
            name="blades",
        )
        propeller.inertial = Inertial.from_geometry(
            Cylinder(radius=0.155, length=0.012),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
        )
        return propeller

    front_left_propeller = add_propeller("front_left_propeller")
    front_right_propeller = add_propeller("front_right_propeller")
    rear_left_propeller = add_propeller("rear_left_propeller")
    rear_right_propeller = add_propeller("rear_right_propeller")

    gimbal_yaw = model.part("gimbal_yaw")
    gimbal_yaw.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=motor_metal,
        name="yaw_motor",
    )
    gimbal_yaw.visual(
        Box((0.030, 0.026, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=dark_gray,
        name="yaw_hanger",
    )
    gimbal_yaw.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.080)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
    )

    gimbal_tilt = model.part("gimbal_tilt")
    gimbal_tilt.visual(
        Box((0.028, 0.122, 0.014)),
        origin=Origin(xyz=(0.000, 0.0, -0.004)),
        material=dark_gray,
        name="tilt_bridge",
    )
    for y_pos in (0.054, -0.054):
        gimbal_tilt.visual(
            Box((0.016, 0.014, 0.082)),
            origin=Origin(xyz=(0.000, y_pos, -0.045)),
            material=dark_gray,
        )
        gimbal_tilt.visual(
            Cylinder(radius=0.014, length=0.016),
            origin=Origin(xyz=(0.000, y_pos, -0.082), rpy=(pi / 2.0, 0.0, 0.0)),
            material=motor_metal,
        )
    gimbal_tilt.inertial = Inertial.from_geometry(
        Box((0.060, 0.120, 0.110)),
        mass=0.20,
        origin=Origin(xyz=(0.000, 0.0, -0.045)),
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.064, 0.046, 0.040)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=camera_black,
        name="camera_body",
    )
    camera.visual(
        Box((0.018, 0.030, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0245)),
        material=dark_gray,
        name="camera_top_plate",
    )
    camera.visual(
        Box((0.018, 0.092, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=dark_gray,
        name="roll_crossbar",
    )
    camera.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.056, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    camera.visual(
        Cylinder(radius=0.023, length=0.006),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="lens_bezel",
    )
    camera.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.080)),
        mass=0.42,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_left_landing_gear",
        ArticulationType.FIXED,
        parent=body,
        child=left_landing_gear,
        origin=Origin(xyz=(-0.048, 0.112, -0.056)),
    )
    model.articulation(
        "body_to_right_landing_gear",
        ArticulationType.FIXED,
        parent=body,
        child=right_landing_gear,
        origin=Origin(xyz=(-0.048, -0.112, -0.056)),
    )

    model.articulation(
        "front_left_arm_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_left_arm,
        origin=Origin(xyz=(0.130, 0.134, 0.028), rpy=(0.0, 0.0, radians(40.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=radians(68.0)),
    )
    model.articulation(
        "front_right_arm_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_right_arm,
        origin=Origin(xyz=(0.130, -0.134, 0.028), rpy=(0.0, 0.0, radians(-40.0))),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=radians(68.0)),
    )
    model.articulation(
        "rear_left_arm_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_left_arm,
        origin=Origin(xyz=(-0.130, 0.134, 0.028), rpy=(0.0, 0.0, radians(140.0))),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=radians(68.0)),
    )
    model.articulation(
        "rear_right_arm_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_right_arm,
        origin=Origin(xyz=(-0.130, -0.134, 0.028), rpy=(0.0, 0.0, radians(-140.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=radians(68.0)),
    )

    for arm_name, prop_name in (
        ("front_left_arm", "front_left_propeller"),
        ("front_right_arm", "front_right_propeller"),
        ("rear_left_arm", "rear_left_propeller"),
        ("rear_right_arm", "rear_right_propeller"),
    ):
        model.articulation(
            f"{arm_name}_to_propeller",
            ArticulationType.CONTINUOUS,
            parent=arm_name,
            child=prop_name,
            origin=Origin(xyz=(0.305, 0.0, 0.040)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=80.0),
        )

    model.articulation(
        "body_to_gimbal_yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=gimbal_yaw,
        origin=Origin(xyz=(0.152, 0.0, -0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=radians(-90.0), upper=radians(90.0)),
    )
    model.articulation(
        "gimbal_tilt",
        ArticulationType.REVOLUTE,
        parent=gimbal_yaw,
        child=gimbal_tilt,
        origin=Origin(xyz=(0.0, 0.0, -0.066)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=radians(-35.0), upper=radians(95.0)),
    )
    model.articulation(
        "gimbal_roll",
        ArticulationType.REVOLUTE,
        parent=gimbal_tilt,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=radians(-45.0), upper=radians(45.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    left_landing_gear = object_model.get_part("left_landing_gear")
    right_landing_gear = object_model.get_part("right_landing_gear")
    front_left_propeller = object_model.get_part("front_left_propeller")
    front_right_propeller = object_model.get_part("front_right_propeller")
    rear_left_propeller = object_model.get_part("rear_left_propeller")
    rear_right_propeller = object_model.get_part("rear_right_propeller")
    gimbal_yaw = object_model.get_part("gimbal_yaw")
    camera = object_model.get_part("camera")

    front_left_arm_fold = object_model.get_articulation("front_left_arm_fold")
    front_right_arm_fold = object_model.get_articulation("front_right_arm_fold")
    rear_left_arm_fold = object_model.get_articulation("rear_left_arm_fold")
    rear_right_arm_fold = object_model.get_articulation("rear_right_arm_fold")
    body_to_gimbal_yaw = object_model.get_articulation("body_to_gimbal_yaw")
    gimbal_tilt = object_model.get_articulation("gimbal_tilt")
    gimbal_roll = object_model.get_articulation("gimbal_roll")

    for joint_name in (
        "front_left_arm_to_propeller",
        "front_right_arm_to_propeller",
        "rear_left_arm_to_propeller",
        "rear_right_arm_to_propeller",
    ):
        spin_joint = object_model.get_articulation(joint_name)
        limits = spin_joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous",
            spin_joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={spin_joint.articulation_type}, limits={limits}",
        )

    for fold_joint_name in (
        "front_left_arm_fold",
        "front_right_arm_fold",
        "rear_left_arm_fold",
        "rear_right_arm_fold",
    ):
        fold_joint = object_model.get_articulation(fold_joint_name)
        limits = fold_joint.motion_limits
        ctx.check(
            f"{fold_joint_name} has folding range",
            limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper > radians(45.0),
            details=f"limits={limits}",
        )

    ctx.expect_contact(left_landing_gear, body, contact_tol=0.002, name="left landing gear mounts to body")
    ctx.expect_contact(right_landing_gear, body, contact_tol=0.002, name="right landing gear mounts to body")
    ctx.expect_contact(gimbal_yaw, body, contact_tol=0.002, name="gimbal mount connects to body")

    ctx.expect_origin_distance(
        front_left_propeller,
        front_right_propeller,
        axes="y",
        min_dist=0.58,
        name="front propellers span the frame width",
    )
    ctx.expect_origin_distance(
        front_left_propeller,
        rear_left_propeller,
        axes="x",
        min_dist=0.58,
        name="left propellers span the frame length",
    )
    ctx.expect_origin_gap(camera, body, axis="x", min_gap=0.12, name="camera sits under the nose")
    ctx.expect_origin_gap(body, camera, axis="z", min_gap=0.10, name="camera hangs below the fuselage")

    rest_front_left = ctx.part_world_position(front_left_propeller)
    with ctx.pose({front_left_arm_fold: front_left_arm_fold.motion_limits.upper}):
        folded_front_left = ctx.part_world_position(front_left_propeller)
    ctx.check(
        "front left arm folds alongside the fuselage",
        rest_front_left is not None
        and folded_front_left is not None
        and folded_front_left[0] < rest_front_left[0] - 0.10
        and folded_front_left[1] > rest_front_left[1] + 0.03,
        details=f"rest={rest_front_left}, folded={folded_front_left}",
    )

    rest_rear_left = ctx.part_world_position(rear_left_propeller)
    with ctx.pose({rear_left_arm_fold: rear_left_arm_fold.motion_limits.upper}):
        folded_rear_left = ctx.part_world_position(rear_left_propeller)
    ctx.check(
        "rear left arm folds forward alongside the fuselage",
        rest_rear_left is not None
        and folded_rear_left is not None
        and folded_rear_left[0] > rest_rear_left[0] + 0.10
        and folded_rear_left[1] > rest_rear_left[1] + 0.03,
        details=f"rest={rest_rear_left}, folded={folded_rear_left}",
    )

    lens_rest_aabb = ctx.part_element_world_aabb(camera, elem="lens")
    lens_rest = _camera_center(lens_rest_aabb)
    with ctx.pose({gimbal_tilt: radians(35.0)}):
        lens_tilted_aabb = ctx.part_element_world_aabb(camera, elem="lens")
        lens_tilted = _camera_center(lens_tilted_aabb)
    ctx.check(
        "gimbal tilt drives the lens downward",
        lens_rest_aabb is not None
        and lens_tilted_aabb is not None
        and lens_tilted_aabb[0][2] < lens_rest_aabb[0][2] - 0.02,
        details=f"rest_min_z={None if lens_rest_aabb is None else lens_rest_aabb[0][2]}, tilted_min_z={None if lens_tilted_aabb is None else lens_tilted_aabb[0][2]}, rest_center={lens_rest}, tilted_center={lens_tilted}",
    )

    top_plate_rest = _camera_center(ctx.part_element_world_aabb(camera, elem="camera_top_plate"))
    with ctx.pose({gimbal_roll: radians(25.0)}):
        top_plate_rolled = _camera_center(ctx.part_element_world_aabb(camera, elem="camera_top_plate"))
    ctx.check(
        "gimbal roll banks the camera cage",
        top_plate_rest is not None
        and top_plate_rolled is not None
        and abs(top_plate_rolled[1] - top_plate_rest[1]) > 0.010
        and top_plate_rolled[2] < top_plate_rest[2] - 0.002,
        details=f"rest={top_plate_rest}, rolled={top_plate_rolled}",
    )

    with ctx.pose({body_to_gimbal_yaw: radians(35.0)}):
        yawed_lens = _camera_center(ctx.part_element_world_aabb(camera, elem="lens"))
    ctx.check(
        "gimbal yaw pans the camera",
        lens_rest is not None and yawed_lens is not None and abs(yawed_lens[1] - lens_rest[1]) > 0.02,
        details=f"rest={lens_rest}, yawed={yawed_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
