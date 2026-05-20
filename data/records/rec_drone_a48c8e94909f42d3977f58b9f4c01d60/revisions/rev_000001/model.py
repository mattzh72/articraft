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


def _yz_section(width: float, height: float, radius: float, x: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinematic_folding_quadcopter")

    shell_gray = model.material("shell_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    carbon = model.material("carbon", rgba=(0.12, 0.13, 0.14, 1.0))
    motor_metal = model.material("motor_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    prop_black = model.material("prop_black", rgba=(0.08, 0.08, 0.09, 1.0))
    gimbal_gray = model.material("gimbal_gray", rgba=(0.30, 0.32, 0.34, 1.0))
    glass = model.material("glass", rgba=(0.12, 0.18, 0.22, 0.55))

    fuselage_shell = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.060, 0.040, 0.012, -0.165),
                _yz_section(0.112, 0.074, 0.020, -0.085),
                _yz_section(0.126, 0.086, 0.024, 0.015),
                _yz_section(0.116, 0.080, 0.022, 0.095),
                _yz_section(0.072, 0.050, 0.014, 0.172),
            ]
        ),
        "fuselage_shell",
    )
    canopy_shell = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.040, 0.020, 0.008, -0.060),
                _yz_section(0.060, 0.030, 0.010, 0.000),
                _yz_section(0.054, 0.026, 0.009, 0.090),
            ]
        ),
        "fuselage_canopy",
    )
    arm_beam = mesh_from_geometry(
        sweep_profile_along_spline(
            [(0.040, 0.0, 0.002), (0.112, 0.0, 0.006), (0.190, 0.0, 0.004)],
            profile=rounded_rect_profile(0.028, 0.018, radius=0.006, corner_segments=6),
            samples_per_segment=16,
            cap_profile=True,
        ),
        "folding_arm_beam",
    )
    arm_fairing = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.020, 0.010, 0.003, 0.050),
                _yz_section(0.028, 0.016, 0.005, 0.094),
                _yz_section(0.024, 0.014, 0.004, 0.122),
                _yz_section(0.016, 0.008, 0.003, 0.150),
            ]
        ),
        "folding_arm_fairing",
    )
    prop_blade = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.120, 0.018, radius=0.005, corner_segments=6),
            0.003,
            cap=True,
            center=True,
        ),
        "prop_blade",
    )
    front_leg_left = mesh_from_geometry(
        tube_from_spline_points(
            [(0.060, 0.028, -0.008), (0.078, 0.050, -0.050), (0.064, 0.074, -0.082)],
            radius=0.006,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
        "front_leg_left",
    )
    front_leg_right = mesh_from_geometry(
        tube_from_spline_points(
            _mirror_y([(0.060, 0.028, -0.008), (0.078, 0.050, -0.050), (0.064, 0.074, -0.082)]),
            radius=0.006,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
        "front_leg_right",
    )

    fuselage = model.part("fuselage")
    fuselage.visual(fuselage_shell, material=shell_gray, name="main_shell")
    fuselage.visual(canopy_shell, origin=Origin(xyz=(0.012, 0.0, 0.030)), material=shell_dark, name="top_canopy")
    fuselage.visual(Box((0.124, 0.062, 0.020)), origin=Origin(xyz=(-0.020, 0.0, 0.044)), material=shell_dark, name="battery_hump")
    fuselage.visual(Box((0.040, 0.056, 0.015)), origin=Origin(xyz=(0.156, 0.0, -0.004)), material=shell_dark, name="nose_sensor_bar")
    fuselage.visual(Box((0.046, 0.034, 0.015)), origin=Origin(xyz=(0.138, 0.0, -0.024)), material=shell_dark, name="nose_chin")
    fuselage.visual(Box((0.060, 0.038, 0.010)), origin=Origin(xyz=(0.100, 0.0, -0.037)), material=shell_dark, name="gimbal_mount")
    fuselage.visual(Box((0.060, 0.040, 0.018)), origin=Origin(xyz=(-0.110, 0.0, 0.030)), material=shell_dark, name="rear_battery_latch")
    fuselage.visual(front_leg_left, material=shell_dark, name="front_leg_left")
    fuselage.visual(front_leg_right, material=shell_dark, name="front_leg_right")
    fuselage.visual(Box((0.034, 0.014, 0.008)), origin=Origin(xyz=(0.066, 0.076, -0.084)), material=shell_dark, name="front_foot_left")
    fuselage.visual(Box((0.034, 0.014, 0.008)), origin=Origin(xyz=(0.066, -0.076, -0.084)), material=shell_dark, name="front_foot_right")
    fuselage.visual(Box((0.016, 0.016, 0.048)), origin=Origin(xyz=(-0.106, 0.030, -0.056)), material=shell_dark, name="rear_leg_left")
    fuselage.visual(Box((0.016, 0.016, 0.048)), origin=Origin(xyz=(-0.106, -0.030, -0.056)), material=shell_dark, name="rear_leg_right")
    fuselage.visual(Box((0.032, 0.016, 0.008)), origin=Origin(xyz=(-0.106, 0.030, -0.080)), material=shell_dark, name="rear_foot_left")
    fuselage.visual(Box((0.032, 0.016, 0.008)), origin=Origin(xyz=(-0.106, -0.030, -0.080)), material=shell_dark, name="rear_foot_right")
    fuselage.visual(Box((0.020, 0.008, 0.022)), origin=Origin(xyz=(-0.090, 0.018, 0.056)), material=shell_dark, name="rear_antenna_left")
    fuselage.visual(Box((0.020, 0.008, 0.022)), origin=Origin(xyz=(-0.090, -0.018, 0.056)), material=shell_dark, name="rear_antenna_right")
    fuselage.inertial = Inertial.from_geometry(
        Box((0.400, 0.180, 0.120)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    arm_specs = [
        ("front_left_arm", "front_left_prop", "front_left_fold", "front_left_spin", (0.092, 0.068, 0.010), math.radians(40.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
        ("front_right_arm", "front_right_prop", "front_right_fold", "front_right_spin", (0.092, -0.068, 0.010), math.radians(-40.0), (0.0, 0.0, -1.0), (0.0, 0.0, -1.0)),
        ("rear_left_arm", "rear_left_prop", "rear_left_fold", "rear_left_spin", (-0.092, 0.068, 0.004), math.pi - math.radians(40.0), (0.0, 0.0, -1.0), (0.0, 0.0, -1.0)),
        ("rear_right_arm", "rear_right_prop", "rear_right_fold", "rear_right_spin", (-0.092, -0.068, 0.004), -math.pi + math.radians(40.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
    ]

    for index, (arm_name, prop_name, fold_name, spin_name, root_xyz, yaw, fold_axis, spin_axis) in enumerate(arm_specs):
        dx = math.cos(yaw)
        dy = math.sin(yaw)
        fuselage.visual(
            Box((0.034, 0.040, 0.010)),
            origin=Origin(
                xyz=(root_xyz[0] - 0.030 * dx, root_xyz[1] - 0.030 * dy, root_xyz[2] - 0.006),
                rpy=(0.0, 0.0, yaw),
            ),
            material=shell_dark,
            name=f"arm_mount_{index}_base",
        )
        fuselage.visual(
            Box((0.018, 0.008, 0.022)),
            origin=Origin(
                xyz=(root_xyz[0] - 0.006 * dx - 0.013 * dy, root_xyz[1] - 0.006 * dy + 0.013 * dx, root_xyz[2]),
                rpy=(0.0, 0.0, yaw),
            ),
            material=shell_dark,
            name=f"arm_mount_{index}_lug_outer",
        )
        fuselage.visual(
            Box((0.018, 0.008, 0.022)),
            origin=Origin(
                xyz=(root_xyz[0] - 0.006 * dx + 0.013 * dy, root_xyz[1] - 0.006 * dy - 0.013 * dx, root_xyz[2]),
                rpy=(0.0, 0.0, yaw),
            ),
            material=shell_dark,
            name=f"arm_mount_{index}_lug_inner",
        )

        arm = model.part(arm_name)
        arm.visual(
            Cylinder(radius=0.0085, length=0.020),
            origin=Origin(),
            material=carbon,
            name="arm_root",
        )
        arm.visual(
            Box((0.050, 0.020, 0.018)),
            origin=Origin(xyz=(0.026, 0.0, 0.006)),
            material=carbon,
            name="arm_bridge",
        )
        arm.visual(arm_beam, material=carbon, name="arm_beam")
        arm.visual(arm_fairing, origin=Origin(xyz=(0.0, 0.0, 0.013)), material=shell_dark, name="arm_fairing")
        arm.visual(Cylinder(radius=0.018, length=0.028), origin=Origin(xyz=(0.208, 0.0, 0.014)), material=motor_metal, name="motor_pod")
        arm.visual(Cylinder(radius=0.013, length=0.008), origin=Origin(xyz=(0.208, 0.0, 0.032)), material=shell_dark, name="motor_cap")
        arm.inertial = Inertial.from_geometry(
            Box((0.250, 0.040, 0.050)),
            mass=0.18,
            origin=Origin(xyz=(0.120, 0.0, 0.014)),
        )

        prop = model.part(prop_name)
        prop.visual(Cylinder(radius=0.011, length=0.008), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=motor_metal, name="hub")
        prop.visual(prop_blade, origin=Origin(xyz=(0.054, 0.0, 0.008)), material=prop_black, name="blade_a")
        prop.visual(prop_blade, origin=Origin(xyz=(-0.054, 0.0, 0.008), rpy=(0.0, 0.0, math.pi)), material=prop_black, name="blade_b")
        prop.inertial = Inertial.from_geometry(
            Cylinder(radius=0.110, length=0.010),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
        )

        model.articulation(
            fold_name,
            ArticulationType.REVOLUTE,
            parent=fuselage,
            child=arm,
            origin=Origin(xyz=root_xyz, rpy=(0.0, 0.0, yaw)),
            axis=fold_axis,
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=1.2,
                lower=0.0,
                upper=math.radians(72.0),
            ),
        )
        model.articulation(
            spin_name,
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=prop,
            origin=Origin(xyz=(0.208, 0.0, 0.036)),
            axis=spin_axis,
            motion_limits=MotionLimits(effort=0.4, velocity=80.0),
        )

    gimbal_pan = model.part("gimbal_pan")
    gimbal_pan.visual(Cylinder(radius=0.020, length=0.014), origin=Origin(xyz=(0.0, 0.0, -0.007)), material=gimbal_gray, name="pan_motor")
    gimbal_pan.visual(Box((0.024, 0.046, 0.024)), origin=Origin(xyz=(0.0, 0.0, -0.024)), material=gimbal_gray, name="pan_body")
    gimbal_pan.visual(Box((0.012, 0.010, 0.032)), origin=Origin(xyz=(0.0, 0.028, -0.044)), material=gimbal_gray, name="pan_yoke_left")
    gimbal_pan.visual(Box((0.012, 0.010, 0.032)), origin=Origin(xyz=(0.0, -0.028, -0.044)), material=gimbal_gray, name="pan_yoke_right")
    gimbal_pan.inertial = Inertial.from_geometry(
        Box((0.060, 0.070, 0.080)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    gimbal_tilt = model.part("gimbal_tilt")
    gimbal_tilt.visual(Box((0.040, 0.046, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=gimbal_gray, name="tilt_bridge")
    gimbal_tilt.visual(Box((0.014, 0.010, 0.060)), origin=Origin(xyz=(0.024, 0.017, -0.030)), material=gimbal_gray, name="tilt_hanger_left")
    gimbal_tilt.visual(Box((0.014, 0.010, 0.060)), origin=Origin(xyz=(0.024, -0.017, -0.030)), material=gimbal_gray, name="tilt_hanger_right")
    gimbal_tilt.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 0.090)),
        mass=0.07,
        origin=Origin(xyz=(0.018, 0.0, -0.030)),
    )

    gimbal_roll = model.part("gimbal_roll")
    gimbal_roll.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gimbal_gray,
        name="roll_motor",
    )
    gimbal_roll.visual(Box((0.014, 0.020, 0.010)), origin=Origin(xyz=(0.0, -0.010, -0.005)), material=gimbal_gray, name="roll_connector")
    gimbal_roll.visual(Box((0.014, 0.008, 0.052)), origin=Origin(xyz=(0.0, -0.020, -0.026)), material=gimbal_gray, name="roll_side_arm")
    gimbal_roll.inertial = Inertial.from_geometry(
        Box((0.030, 0.040, 0.070)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.010, -0.020)),
    )

    camera = model.part("camera")
    camera.visual(Box((0.060, 0.028, 0.030)), material=shell_dark, name="camera_body")
    camera.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_dark,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_front",
    )
    camera.visual(Box((0.014, 0.008, 0.008)), origin=Origin(xyz=(-0.010, -0.012, 0.014)), material=shell_dark, name="mount_lug")
    camera.visual(Box((0.020, 0.012, 0.010)), origin=Origin(xyz=(-0.012, 0.005, 0.020)), material=shell_gray, name="top_module")
    camera.inertial = Inertial.from_geometry(
        Box((0.090, 0.040, 0.040)),
        mass=0.18,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    model.articulation(
        "gimbal_pan_joint",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=gimbal_pan,
        origin=Origin(xyz=(0.100, 0.0, -0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=4.0),
    )
    model.articulation(
        "gimbal_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=gimbal_pan,
        child=gimbal_tilt,
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.5,
            lower=math.radians(-75.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "gimbal_roll_joint",
        ArticulationType.REVOLUTE,
        parent=gimbal_tilt,
        child=gimbal_roll,
        origin=Origin(xyz=(0.028, 0.0, -0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=2.5,
            lower=math.radians(-40.0),
            upper=math.radians(40.0),
        ),
    )
    model.articulation(
        "camera_mount",
        ArticulationType.FIXED,
        parent=gimbal_roll,
        child=camera,
        origin=Origin(xyz=(0.008, 0.0, -0.070)),
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

    fuselage = object_model.get_part("fuselage")
    camera = object_model.get_part("camera")
    prop_parts = [
        object_model.get_part("front_left_prop"),
        object_model.get_part("front_right_prop"),
        object_model.get_part("rear_left_prop"),
        object_model.get_part("rear_right_prop"),
    ]
    fold_joints = [
        object_model.get_articulation("front_left_fold"),
        object_model.get_articulation("front_right_fold"),
        object_model.get_articulation("rear_left_fold"),
        object_model.get_articulation("rear_right_fold"),
    ]
    prop_joints = [
        object_model.get_articulation("front_left_spin"),
        object_model.get_articulation("front_right_spin"),
        object_model.get_articulation("rear_left_spin"),
        object_model.get_articulation("rear_right_spin"),
    ]
    gimbal_pan = object_model.get_articulation("gimbal_pan_joint")
    gimbal_tilt = object_model.get_articulation("gimbal_tilt_joint")
    gimbal_roll = object_model.get_articulation("gimbal_roll_joint")

    ctx.expect_gap(
        fuselage,
        camera,
        axis="z",
        min_gap=0.010,
        max_gap=0.140,
        name="camera hangs below fuselage at rest",
    )

    spin_ok = True
    for joint in prop_joints:
        limits = joint.motion_limits
        spin_ok = spin_ok and joint.articulation_type == ArticulationType.CONTINUOUS
        spin_ok = spin_ok and limits is not None and limits.lower is None and limits.upper is None
    ctx.check("all propellers use continuous spin joints", spin_ok)

    open_positions = [ctx.part_world_position(part) for part in prop_parts]
    folded_pose = {joint: joint.motion_limits.upper for joint in fold_joints if joint.motion_limits is not None}
    with ctx.pose(folded_pose):
        folded_positions = [ctx.part_world_position(part) for part in prop_parts]

    fold_ok = True
    fold_details = []
    for part, open_pos, folded_pos in zip(prop_parts, open_positions, folded_positions):
        if open_pos is None or folded_pos is None:
            fold_ok = False
            fold_details.append(f"{part.name}: missing position")
            continue
        moved_inboard = abs(folded_pos[0]) < abs(open_pos[0]) - 0.05
        fold_ok = fold_ok and moved_inboard
        fold_details.append(f"{part.name}: open={open_pos}, folded={folded_pos}")
    ctx.check("folding arms bring rotors inboard", fold_ok, details=" | ".join(fold_details))

    rest_camera_center = _aabb_center(ctx.part_world_aabb(camera))
    with ctx.pose({gimbal_pan: math.pi / 2.0}):
        pan_camera_center = _aabb_center(ctx.part_world_aabb(camera))
    pan_ok = (
        rest_camera_center is not None
        and pan_camera_center is not None
        and pan_camera_center[1] > rest_camera_center[1] + 0.03
    )
    ctx.check(
        "gimbal pan swings camera sideways",
        pan_ok,
        details=f"rest={rest_camera_center}, panned={pan_camera_center}",
    )

    with ctx.pose({gimbal_tilt: math.radians(25.0)}):
        tilt_camera_center = _aabb_center(ctx.part_world_aabb(camera))
        ctx.expect_gap(
            fuselage,
            camera,
            axis="z",
            min_gap=0.005,
            max_gap=0.090,
            name="camera remains below fuselage while tilted",
        )
    tilt_ok = (
        rest_camera_center is not None
        and tilt_camera_center is not None
        and tilt_camera_center[2] > rest_camera_center[2] + 0.010
    )
    ctx.check(
        "gimbal tilt lifts the camera in positive motion",
        tilt_ok,
        details=f"rest={rest_camera_center}, tilted={tilt_camera_center}",
    )

    with ctx.pose({gimbal_roll: math.radians(30.0)}):
        roll_camera_center = _aabb_center(ctx.part_world_aabb(camera))
    roll_ok = (
        rest_camera_center is not None
        and roll_camera_center is not None
        and roll_camera_center[1] > rest_camera_center[1] + 0.02
    )
    ctx.check(
        "gimbal roll shifts the hanging camera laterally",
        roll_ok,
        details=f"rest={rest_camera_center}, rolled={roll_camera_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
