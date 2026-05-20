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
    wire_from_points,
)


SQRT_HALF = math.sqrt(0.5)
FORK_YAW = -math.pi / 4.0
PAN_LIMIT = math.radians(110.0)
TILT_UP_LIMIT = math.radians(20.0)
TILT_DOWN_LIMIT = math.radians(60.0)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rot_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (cos_yaw * x - sin_yaw * y, sin_yaw * x + cos_yaw * y)


def _fork_origin(
    x_local: float,
    y_local: float,
    z_local: float,
    *,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> Origin:
    x_world, y_world = _rot_xy(x_local, y_local, FORK_YAW)
    return Origin(xyz=(x_world, y_world, z_local), rpy=(roll, pitch, yaw + FORK_YAW))


def _bisector_point(distance_from_corner: float, z_value: float) -> tuple[float, float, float]:
    return (distance_from_corner * SQRT_HALF, distance_from_corner * SQRT_HALF, z_value)


def _aabb_center(aabb) -> tuple[float, float, float]:
    return (
        (aabb[0][0] + aabb[1][0]) * 0.5,
        (aabb[0][1] + aabb[1][1]) * 0.5,
        (aabb[0][2] + aabb[1][2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_mount_cctv_arm")

    galvanized = model.material("galvanized_steel", rgba=(0.69, 0.72, 0.74, 1.0))
    dark_powder = model.material("powder_black", rgba=(0.14, 0.15, 0.17, 1.0))
    warm_white = model.material("camera_white", rgba=(0.90, 0.92, 0.93, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.46, 0.55, 0.60, 0.32))
    lens_black = model.material("lens_black", rgba=(0.05, 0.06, 0.07, 1.0))

    plate_profile = [
        (-0.110, -0.110),
        (0.082, -0.110),
        (0.108, 0.000),
        (0.082, 0.110),
        (-0.110, 0.110),
        (-0.126, 0.000),
    ]
    plate_geom = ExtrudeGeometry.centered(plate_profile, 0.006)
    plate_geom.rotate_x(-math.pi / 2.0).rotate_z(-math.pi / 4.0)
    plate_mesh = _mesh("corner_plate_shell", plate_geom)

    brace_geom = wire_from_points(
        [
            (0.060, 0.060, 0.120),
            (0.154, 0.154, 0.198),
        ],
        radius=0.008,
        cap_ends=True,
        corner_mode="miter",
    )
    brace_mesh = _mesh("corner_arm_brace", brace_geom)

    top_shell_mesh = _mesh(
        "dome_top_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.012, 0.032),
                (0.028, 0.034),
                (0.040, 0.028),
                (0.046, 0.018),
                (0.049, 0.006),
                (0.048, -0.004),
            ],
            [
                (0.007, 0.028),
                (0.023, 0.030),
                (0.035, 0.025),
                (0.041, 0.016),
                (0.044, 0.006),
                (0.044, -0.004),
            ],
            segments=56,
        ),
    )
    clear_dome_mesh = _mesh(
        "dome_clear_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.048, -0.004),
                (0.047, -0.018),
                (0.041, -0.034),
                (0.030, -0.050),
                (0.014, -0.060),
                (0.000, -0.064),
            ],
            [
                (0.045, -0.004),
                (0.044, -0.018),
                (0.038, -0.033),
                (0.027, -0.047),
                (0.012, -0.056),
                (0.000, -0.060),
            ],
            segments=56,
        ),
    )

    pan_origin = _bisector_point(0.332, 0.248)
    tilt_offset_xy = _rot_xy(0.0, 0.082, FORK_YAW)

    mount_bracket = model.part("mount_bracket")
    mount_bracket.visual(
        Box((0.165, 0.006, 0.260)),
        origin=Origin(xyz=(0.0825, 0.0, 0.180)),
        material=galvanized,
        name="wall_leg_y",
    )
    mount_bracket.visual(
        Box((0.006, 0.165, 0.260)),
        origin=Origin(xyz=(0.0, 0.0825, 0.180)),
        material=galvanized,
        name="wall_leg_x",
    )
    mount_bracket.visual(
        plate_mesh,
        origin=Origin(xyz=(0.078, 0.078, 0.180)),
        material=galvanized,
        name="diagonal_plate",
    )
    mount_bracket.visual(
        Cylinder(radius=0.028, length=0.032),
        origin=Origin(xyz=_bisector_point(0.129, 0.220), rpy=(0.0, math.pi / 2.0, math.pi / 4.0)),
        material=galvanized,
        name="arm_boss",
    )
    mount_bracket.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(xyz=_bisector_point(0.235, 0.220), rpy=(0.0, math.pi / 2.0, math.pi / 4.0)),
        material=dark_powder,
        name="arm_tube",
    )
    mount_bracket.visual(
        brace_mesh,
        material=galvanized,
        name="arm_brace",
    )
    mount_bracket.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(pan_origin[0], pan_origin[1], pan_origin[2] - 0.011)),
        material=dark_powder,
        name="pan_pedestal",
    )
    for index, (x_val, y_val, z_val, axis_rpy) in enumerate(
        (
            (0.046, 0.0035, 0.110, (math.pi / 2.0, 0.0, 0.0)),
            (0.046, 0.0035, 0.250, (math.pi / 2.0, 0.0, 0.0)),
            (0.0035, 0.046, 0.110, (0.0, math.pi / 2.0, 0.0)),
            (0.0035, 0.046, 0.250, (0.0, math.pi / 2.0, 0.0)),
        )
    ):
        mount_bracket.visual(
            Cylinder(radius=0.009, length=0.005),
            origin=Origin(xyz=(x_val, y_val, z_val), rpy=axis_rpy),
            material=dark_powder,
            name=f"anchor_head_{index}",
        )
    mount_bracket.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.32)),
        mass=4.2,
        origin=Origin(xyz=(0.120, 0.120, 0.170)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_powder,
        name="pan_collar",
    )
    pan_yoke.visual(
        Cylinder(radius=0.009, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=dark_powder,
        name="mast_post",
    )
    pan_yoke.visual(
        Box((0.028, 0.040, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.090)),
        material=dark_powder,
        name="rear_head",
    )
    pan_yoke.visual(
        Box((0.118, 0.045, 0.012)),
        origin=Origin(xyz=(0.0, 0.0625, 0.092)),
        material=dark_powder,
        name="fork_bridge",
    )
    pan_yoke.visual(
        Box((0.008, 0.006, 0.066)),
        origin=Origin(xyz=(-0.055, 0.0875, 0.059)),
        material=dark_powder,
        name="left_arm",
    )
    pan_yoke.visual(
        Box((0.008, 0.006, 0.066)),
        origin=Origin(xyz=(0.055, 0.0875, 0.059)),
        material=dark_powder,
        name="right_arm",
    )
    pan_yoke.visual(
        Box((0.014, 0.024, 0.010)),
        origin=Origin(xyz=(0.051, 0.066, 0.092)),
        material=dark_powder,
        name="service_link",
    )
    pan_yoke.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.064, 0.066, 0.092), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_powder,
        name="service_pod",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.16, 0.13, 0.11)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.050, 0.060)),
    )

    camera_housing = model.part("camera_housing")
    camera_housing.visual(
        Cylinder(radius=0.0065, length=0.008),
        origin=Origin(xyz=(-0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_powder,
        name="left_trunnion",
    )
    camera_housing.visual(
        Cylinder(radius=0.0065, length=0.008),
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_powder,
        name="right_trunnion",
    )
    camera_housing.visual(
        Box((0.094, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_powder,
        name="tilt_hub",
    )
    camera_housing.visual(
        Box((0.032, 0.082, 0.028)),
        origin=Origin(xyz=(0.0, 0.041, -0.018)),
        material=dark_powder,
        name="shell_neck",
    )
    camera_housing.visual(
        top_shell_mesh,
        origin=Origin(xyz=(0.0, 0.078, -0.036)),
        material=warm_white,
        name="top_shell",
    )
    camera_housing.visual(
        clear_dome_mesh,
        origin=Origin(xyz=(0.0, 0.078, -0.036)),
        material=smoked_clear,
        name="clear_dome",
    )
    camera_housing.visual(
        Box((0.066, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.112, -0.010)),
        material=warm_white,
        name="front_visor",
    )
    camera_housing.visual(
        Box((0.018, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.076, -0.018)),
        material=lens_black,
        name="sensor_mount",
    )
    camera_housing.visual(
        Box((0.022, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.086, -0.034)),
        material=lens_black,
        name="sensor_core",
    )
    camera_housing.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.102, -0.034), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_black,
        name="lens_barrel",
    )
    camera_housing.inertial = Inertial.from_geometry(
        Box((0.115, 0.115, 0.110)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.065, -0.032)),
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=mount_bracket,
        child=pan_yoke,
        origin=Origin(xyz=pan_origin, rpy=(0.0, 0.0, FORK_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-PAN_LIMIT,
            upper=PAN_LIMIT,
        ),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera_housing,
        origin=Origin(xyz=(0.0, 0.078, 0.059)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.6,
            lower=-TILT_UP_LIMIT,
            upper=TILT_DOWN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount_bracket = object_model.get_part("mount_bracket")
    pan_yoke = object_model.get_part("pan_yoke")
    camera_housing = object_model.get_part("camera_housing")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=40,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        pan_yoke,
        mount_bracket,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="pan_collar",
        negative_elem="pan_pedestal",
        name="pan_collar_seats_on_pedestal",
    )
    ctx.expect_contact(
        camera_housing,
        pan_yoke,
        elem_a="left_trunnion",
        elem_b="left_arm",
        name="left_trunnion_contacts_left_yoke_arm",
    )
    ctx.expect_contact(
        camera_housing,
        pan_yoke,
        elem_a="right_trunnion",
        elem_b="right_arm",
        name="right_trunnion_contacts_right_yoke_arm",
    )

    pan_origin_pos = ctx.part_world_position(pan_yoke)
    camera_origin_pos = ctx.part_world_position(camera_housing)
    assert pan_origin_pos is not None
    assert camera_origin_pos is not None
    ctx.check(
        "camera_origin_projects_forward_of_pan_axis",
        camera_origin_pos[0] > pan_origin_pos[0] + 0.03 and camera_origin_pos[1] > pan_origin_pos[1] + 0.03,
        details=(
            f"camera origin {camera_origin_pos} should sit forward of pan origin {pan_origin_pos} "
            "along the diagonal viewing direction."
        ),
    )

    service_pod_rest = ctx.part_element_world_aabb(pan_yoke, elem="service_pod")
    assert service_pod_rest is not None
    service_pod_rest_center = _aabb_center(service_pod_rest)
    with ctx.pose({pan_axis: math.radians(70.0)}):
        service_pod_swung = ctx.part_element_world_aabb(pan_yoke, elem="service_pod")
        assert service_pod_swung is not None
        service_pod_swung_center = _aabb_center(service_pod_swung)
        ctx.expect_gap(
            pan_yoke,
            mount_bracket,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="pan_collar",
            negative_elem="pan_pedestal",
            name="pan_collar_stays_seated_when_swung",
        )
        ctx.check(
            "pan_axis_moves_service_pod_around_vertical_axis",
            abs(service_pod_swung_center[2] - service_pod_rest_center[2]) < 0.003
            and abs(service_pod_swung_center[0] - service_pod_rest_center[0]) > 0.020
            and abs(service_pod_swung_center[1] - service_pod_rest_center[1]) > 0.020,
            details=(
                f"service pod center moved from {service_pod_rest_center} to {service_pod_swung_center}; "
                "pan should change XY strongly while keeping height nearly constant."
            ),
        )

    visor_rest = ctx.part_element_world_aabb(camera_housing, elem="front_visor")
    assert visor_rest is not None
    visor_rest_center = _aabb_center(visor_rest)
    with ctx.pose({tilt_axis: math.radians(60.0)}):
        visor_tilted = ctx.part_element_world_aabb(camera_housing, elem="front_visor")
        assert visor_tilted is not None
        visor_tilted_center = _aabb_center(visor_tilted)
        ctx.expect_contact(
            camera_housing,
            pan_yoke,
            elem_a="left_trunnion",
            elem_b="left_arm",
            name="left_trunnion_remains_seated_when_tilted",
        )
        ctx.expect_contact(
            camera_housing,
            pan_yoke,
            elem_a="right_trunnion",
            elem_b="right_arm",
            name="right_trunnion_remains_seated_when_tilted",
        )
        ctx.check(
            "positive_tilt_drops_the_front_visor",
            visor_tilted_center[2] < visor_rest_center[2] - 0.020,
            details=(
                f"visor center moved from {visor_rest_center} to {visor_tilted_center}; "
                "positive tilt should pitch the camera nose downward."
            ),
        )

    for articulation, label in ((pan_axis, "pan_axis"), (tilt_axis, "tilt_axis")):
        limits = articulation.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None
        with ctx.pose({articulation: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_lower_no_floating")
        with ctx.pose({articulation: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_upper_no_floating")

    tilt_limits = tilt_axis.motion_limits
    assert tilt_limits is not None
    assert tilt_limits.upper is not None
    with ctx.pose({pan_axis: math.radians(90.0), tilt_axis: tilt_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pan_tilt_pose_clear")
        ctx.fail_if_isolated_parts(name="combined_pan_tilt_pose_supported")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
