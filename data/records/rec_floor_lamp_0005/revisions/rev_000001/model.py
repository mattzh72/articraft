from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.19
BASE_HEIGHT = 0.06
COLUMN_RADIUS = 0.018
COLUMN_HEIGHT = 1.44
MOUNT_RADIUS = 0.082
MOUNT_COLLAR_RADIUS = 0.032
MOUNT_COLLAR_HEIGHT = 0.020
MOUNT_BRACKET_WIDTH = 0.054
MOUNT_BRACKET_HEIGHT = 0.032
MOUNT_PLATE_DEPTH = 0.048
MOUNT_PLATE_THICKNESS = 0.004
MOUNT_PLATE_HEIGHT = 0.060
HUB_RADIUS = 0.018
HUB_WIDTH = 0.038
HUB_CAP_RADIUS = 0.022
HUB_CAP_THICKNESS = 0.004
ARM_TUBE_RADIUS = 0.010
ROOT_SLEEVE_RADIUS = 0.012
ROOT_SLEEVE_LENGTH = 0.050
SHADE_RADIUS = 0.055
SHADE_LENGTH = 0.120
SHADE_PITCH = math.pi / 2.0 + 0.26
ARM_PATH = (
    (0.015, 0.0, 0.000),
    (0.095, 0.0, 0.028),
    (0.220, 0.0, 0.102),
    (0.345, 0.0, 0.185),
)
ARM_TIP = ARM_PATH[-1]
ARM_MOUNTS = (
    ("lower", math.radians(20.0), 0.88),
    ("middle", math.radians(145.0), 1.13),
    ("upper", math.radians(-105.0), 1.38),
)


def _rotate_xy(x_local: float, y_local: float, azimuth: float) -> tuple[float, float]:
    cos_a = math.cos(azimuth)
    sin_a = math.sin(azimuth)
    return (
        x_local * cos_a - y_local * sin_a,
        x_local * sin_a + y_local * cos_a,
    )


def _local_xyz(x_local: float, y_local: float, z_local: float, azimuth: float) -> tuple[float, float, float]:
    x_world, y_world = _rotate_xy(x_local, y_local, azimuth)
    return (x_world, y_world, z_local)


def _pitch_direction(angle: float) -> tuple[float, float, float]:
    return (math.sin(angle), 0.0, math.cos(angle))


def _offset_point(
    point: tuple[float, float, float],
    direction: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    return (
        point[0] + direction[0] * distance,
        point[1] + direction[1] * distance,
        point[2] + direction[2] * distance,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_arm_tube_mesh():
    return _save_mesh(
        "floor_lamp_arm_tube.obj",
        tube_from_spline_points(
            ARM_PATH,
            radius=ARM_TUBE_RADIUS,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )


def _build_shade_mesh():
    outer_profile = [
        (0.010, -0.060),
        (0.020, -0.056),
        (0.038, -0.020),
        (0.050, 0.030),
        (SHADE_RADIUS, 0.060),
    ]
    inner_profile = [
        (0.000, -0.052),
        (0.014, -0.048),
        (0.032, -0.018),
        (0.044, 0.028),
        (0.049, 0.055),
    ]
    return _save_mesh(
        "floor_lamp_shade_shell.obj",
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=56),
    )


def _build_base_shell_mesh():
    profile = [
        (0.000, 0.000),
        (0.110, 0.000),
        (0.155, 0.004),
        (0.178, 0.012),
        (BASE_RADIUS, 0.028),
        (0.182, 0.040),
        (0.140, 0.050),
        (0.105, 0.054),
        (0.072, 0.058),
        (0.000, BASE_HEIGHT),
    ]
    return _save_mesh(
        "floor_lamp_base_shell.obj",
        LatheGeometry(profile, segments=72),
    )


def _add_mount_visuals(stand, arm_name: str, azimuth: float, z_height: float, metal) -> None:
    stand.visual(
        Cylinder(radius=MOUNT_COLLAR_RADIUS, length=MOUNT_COLLAR_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, z_height),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name=f"{arm_name}_collar",
    )

    bracket_length = MOUNT_RADIUS - 0.024
    stand.visual(
        Box((bracket_length, MOUNT_BRACKET_WIDTH, MOUNT_BRACKET_HEIGHT)),
        origin=Origin(
            xyz=_local_xyz(bracket_length * 0.5, 0.0, z_height, azimuth),
            rpy=(0.0, 0.0, azimuth),
        ),
        material=metal,
        name=f"{arm_name}_bracket",
    )

    cheek_offset = HUB_WIDTH * 0.5 + HUB_CAP_THICKNESS + MOUNT_PLATE_THICKNESS * 0.5
    cheek_x = MOUNT_RADIUS
    for cheek_name, y_sign in (("outer", 1.0), ("inner", -1.0)):
        stand.visual(
            Box((MOUNT_PLATE_DEPTH, MOUNT_PLATE_THICKNESS, MOUNT_PLATE_HEIGHT)),
            origin=Origin(
                xyz=_local_xyz(cheek_x, y_sign * cheek_offset, z_height, azimuth),
                rpy=(0.0, 0.0, azimuth),
            ),
            material=metal,
            name=f"{arm_name}_cheek_{cheek_name}",
        )


def _add_arm_visuals(arm, arm_tube_mesh, shade_mesh, arm_metal, shade_paint, glow) -> None:
    arm.inertial = Inertial.from_geometry(
        Box((0.56, 0.12, 0.30)),
        mass=1.25,
        origin=Origin(xyz=(0.26, 0.0, 0.11)),
    )

    arm.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_WIDTH),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="hub_barrel",
    )
    arm.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=HUB_CAP_THICKNESS),
        origin=Origin(
            xyz=(0.0, HUB_WIDTH * 0.5 + HUB_CAP_THICKNESS * 0.5, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=arm_metal,
        name="hub_cap_outer",
    )
    arm.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=HUB_CAP_THICKNESS),
        origin=Origin(
            xyz=(0.0, -HUB_WIDTH * 0.5 - HUB_CAP_THICKNESS * 0.5, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=arm_metal,
        name="hub_cap_inner",
    )
    arm.visual(
        Cylinder(radius=ROOT_SLEEVE_RADIUS, length=ROOT_SLEEVE_LENGTH),
        origin=Origin(xyz=(0.028, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_metal,
        name="root_sleeve",
    )
    arm.visual(
        arm_tube_mesh,
        material=arm_metal,
        name="arm_tube",
    )

    shade_direction = _pitch_direction(SHADE_PITCH)
    shade_knuckle_center = ARM_TIP
    shade_collar_center = _offset_point(ARM_TIP, shade_direction, 0.010)
    shade_socket_center = _offset_point(ARM_TIP, shade_direction, 0.030)
    shade_center = _offset_point(ARM_TIP, shade_direction, 0.080)
    bulb_center = _offset_point(ARM_TIP, shade_direction, 0.050)

    arm.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=shade_knuckle_center),
        material=arm_metal,
        name="shade_knuckle",
    )
    arm.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=shade_collar_center, rpy=(0.0, SHADE_PITCH, 0.0)),
        material=arm_metal,
        name="shade_collar",
    )
    arm.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=shade_socket_center, rpy=(0.0, SHADE_PITCH, 0.0)),
        material=arm_metal,
        name="shade_socket",
    )
    arm.visual(
        shade_mesh,
        origin=Origin(xyz=shade_center, rpy=(0.0, SHADE_PITCH, 0.0)),
        material=shade_paint,
        name="shade_shell",
    )
    arm.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=bulb_center),
        material=glow,
        name="bulb",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_arm_floor_lamp", assets=ASSETS)

    base_metal = model.material("base_metal", rgba=(0.14, 0.14, 0.16, 1.0))
    stem_metal = model.material("stem_metal", rgba=(0.28, 0.28, 0.30, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.66, 0.57, 0.37, 1.0))
    shade_paint = model.material("shade_paint", rgba=(0.93, 0.91, 0.87, 1.0))
    glow = model.material("glow", rgba=(0.98, 0.92, 0.72, 0.72))

    arm_tube_mesh = _build_arm_tube_mesh()
    shade_mesh = _build_shade_mesh()
    base_shell_mesh = _build_base_shell_mesh()

    stand = model.part("stand")
    stand.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, BASE_HEIGHT + COLUMN_HEIGHT + 0.05)),
        mass=21.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + COLUMN_HEIGHT + 0.05) * 0.5)),
    )
    stand.visual(
        Cylinder(radius=0.160, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=base_metal,
        name="base_foot",
    )
    stand.visual(
        base_shell_mesh,
        origin=Origin(),
        material=base_metal,
        name="base_shell",
    )
    stand.visual(
        Cylinder(radius=0.092, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=base_metal,
        name="base_hub",
    )
    stand.visual(
        Cylinder(radius=0.110, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=stem_metal,
        name="base_cap",
    )
    stand.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.006)),
        material=stem_metal,
        name="column_socket",
    )
    stand.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT * 0.5)),
        material=stem_metal,
        name="column",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT + 0.015)),
        material=stem_metal,
        name="top_finial",
    )

    for mount_name, azimuth, z_height in ARM_MOUNTS:
        _add_mount_visuals(stand, mount_name, azimuth, z_height, stem_metal)

    for mount_name, _, _ in ARM_MOUNTS:
        arm = model.part(f"{mount_name}_arm")
        _add_arm_visuals(arm, arm_tube_mesh, shade_mesh, arm_metal, shade_paint, glow)

    for mount_name, azimuth, z_height in ARM_MOUNTS:
        model.articulation(
            f"{mount_name}_arm_pitch",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=f"{mount_name}_arm",
            origin=Origin(
                xyz=_local_xyz(MOUNT_RADIUS, 0.0, z_height, azimuth),
                rpy=(0.0, 0.0, azimuth),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=14.0,
                velocity=1.5,
                lower=-0.70,
                upper=0.85,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    stand = object_model.get_part("stand")
    base_shell = stand.get_visual("base_shell")
    base_cap = stand.get_visual("base_cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    stand_aabb = ctx.part_world_aabb(stand)
    if stand_aabb is not None:
        stand_height = stand_aabb[1][2] - stand_aabb[0][2]
        ctx.check(
            "stand_height_realistic",
            1.48 <= stand_height <= 1.62,
            f"expected floor lamp height around 1.5-1.6 m, got {stand_height:.3f} m",
        )

    base_aabb = ctx.part_element_world_aabb(stand, elem="base_shell")
    if base_aabb is not None:
        base_diameter = base_aabb[1][0] - base_aabb[0][0]
        ctx.check(
            "base_diameter_realistic",
            0.36 <= base_diameter <= 0.40,
            f"expected weighted base diameter near 0.38 m, got {base_diameter:.3f} m",
        )

    lower_pose_clearances = {
        "lower": 0.58,
        "middle": 0.82,
        "upper": 1.03,
    }
    rest_pose_clearances = {
        "lower": 0.74,
        "middle": 0.99,
        "upper": 1.18,
    }

    for mount_name, _, _ in ARM_MOUNTS:
        arm = object_model.get_part(f"{mount_name}_arm")
        joint = object_model.get_articulation(f"{mount_name}_arm_pitch")
        hub_cap_outer = arm.get_visual("hub_cap_outer")
        hub_cap_inner = arm.get_visual("hub_cap_inner")
        shade_shell = arm.get_visual("shade_shell")
        cheek_outer = stand.get_visual(f"{mount_name}_cheek_outer")
        cheek_inner = stand.get_visual(f"{mount_name}_cheek_inner")

        ctx.check(
            f"{mount_name}_joint_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{mount_name} arm should pitch about local +Y, got {joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{mount_name}_joint_limits_realistic",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and -0.9 <= limits.lower <= -0.5
            and 0.7 <= limits.upper <= 1.0,
            f"{mount_name} arm limits should allow moderate pitch travel, got {limits}",
        )

        ctx.expect_contact(
            arm,
            stand,
            elem_a=hub_cap_outer,
            elem_b=cheek_outer,
            name=f"{mount_name}_outer_cap_contact",
        )
        ctx.expect_contact(
            arm,
            stand,
            elem_a=hub_cap_inner,
            elem_b=cheek_inner,
            name=f"{mount_name}_inner_cap_contact",
        )
        ctx.expect_gap(
            arm,
            stand,
            axis="z",
            min_gap=rest_pose_clearances[mount_name],
            positive_elem=shade_shell,
            negative_elem=base_cap,
            name=f"{mount_name}_shade_rest_clear_of_base",
        )

        rest_aabb = ctx.part_element_world_aabb(arm, elem="shade_shell")

        if limits is not None and limits.lower is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{mount_name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{mount_name}_lower_no_floating")
                ctx.expect_contact(
                    arm,
                    stand,
                    elem_a=hub_cap_outer,
                    elem_b=cheek_outer,
                    name=f"{mount_name}_lower_outer_contact",
                )
                ctx.expect_contact(
                    arm,
                    stand,
                    elem_a=hub_cap_inner,
                    elem_b=cheek_inner,
                    name=f"{mount_name}_lower_inner_contact",
                )
                ctx.expect_gap(
                    arm,
                    stand,
                    axis="z",
                    min_gap=lower_pose_clearances[mount_name],
                    positive_elem=shade_shell,
                    negative_elem=base_shell,
                    name=f"{mount_name}_shade_lower_clear_of_base",
                )
                lower_aabb = ctx.part_element_world_aabb(arm, elem="shade_shell")
                ctx.check(
                    f"{mount_name}_shade_rises_at_lower_limit",
                    rest_aabb is not None
                    and lower_aabb is not None
                    and lower_aabb[0][2] > rest_aabb[0][2] + 0.10,
                    f"{mount_name} shade should move upward at lower limit",
                )

        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{mount_name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{mount_name}_upper_no_floating")
                ctx.expect_contact(
                    arm,
                    stand,
                    elem_a=hub_cap_outer,
                    elem_b=cheek_outer,
                    name=f"{mount_name}_upper_outer_contact",
                )
                ctx.expect_contact(
                    arm,
                    stand,
                    elem_a=hub_cap_inner,
                    elem_b=cheek_inner,
                    name=f"{mount_name}_upper_inner_contact",
                )
                upper_aabb = ctx.part_element_world_aabb(arm, elem="shade_shell")
                ctx.check(
                    f"{mount_name}_shade_drops_at_upper_limit",
                    rest_aabb is not None
                    and upper_aabb is not None
                    and upper_aabb[0][2] < rest_aabb[0][2] - 0.10,
                    f"{mount_name} shade should move downward at upper limit",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
