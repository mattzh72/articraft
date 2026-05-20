from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent

POLAR_TILT = math.radians(52.0)
POLAR_AXIS_DIR = (-math.sin(POLAR_TILT), 0.0, math.cos(POLAR_TILT))
POLAR_JOINT_XYZ = (0.0, 0.0, 0.985)
DECLINATION_JOINT_XYZ = (0.0, 0.0, 0.330)
TUBE_MOUNT_XYZ = (0.118, 0.0, 0.275)


def _offset(
    point: tuple[float, float, float],
    direction: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    return (
        point[0] + direction[0] * distance,
        point[1] + direction[1] * distance,
        point[2] + direction[2] * distance,
    )


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_member(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    name: str,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _build_annular_cylinder_mesh(
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
    segments: int = 48,
) -> MeshGeometry:
    geometry = MeshGeometry()
    half_length = length * 0.5

    outer_back: list[int] = []
    outer_front: list[int] = []
    inner_back: list[int] = []
    inner_front: list[int] = []

    for index in range(segments):
        angle = (2.0 * math.pi * index) / segments
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        outer_back.append(geometry.add_vertex(-half_length, outer_radius * cos_a, outer_radius * sin_a))
        outer_front.append(geometry.add_vertex(half_length, outer_radius * cos_a, outer_radius * sin_a))
        inner_back.append(geometry.add_vertex(-half_length, inner_radius * cos_a, inner_radius * sin_a))
        inner_front.append(geometry.add_vertex(half_length, inner_radius * cos_a, inner_radius * sin_a))

    for index in range(segments):
        next_index = (index + 1) % segments
        _add_quad(
            geometry,
            outer_back[index],
            outer_back[next_index],
            outer_front[next_index],
            outer_front[index],
        )
        _add_quad(
            geometry,
            inner_back[index],
            inner_front[index],
            inner_front[next_index],
            inner_back[next_index],
        )
        _add_quad(
            geometry,
            outer_front[index],
            outer_front[next_index],
            inner_front[next_index],
            inner_front[index],
        )
        _add_quad(
            geometry,
            outer_back[index],
            inner_back[index],
            inner_back[next_index],
            outer_back[next_index],
        )

    return geometry


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="newtonian_reflector_equatorial_mount", assets=ASSETS)

    tripod_steel = model.material("tripod_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    mount_paint = model.material("mount_paint", rgba=(0.77, 0.79, 0.82, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    tube_paint = model.material("tube_paint", rgba=(0.95, 0.95, 0.96, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    mirror_glass = model.material("mirror_glass", rgba=(0.66, 0.77, 0.86, 0.55))
    counterweight_metal = model.material("counterweight_metal", rgba=(0.54, 0.56, 0.60, 1.0))

    tube_shell_mesh = _mesh(
        "tube_shell.obj",
        _build_annular_cylinder_mesh(length=0.920, outer_radius=0.118, inner_radius=0.114, segments=56),
    )
    tube_ring_mesh = _mesh(
        "tube_ring.obj",
        _build_annular_cylinder_mesh(length=0.020, outer_radius=0.132, inner_radius=0.118, segments=48),
    )
    mirror_cell_mesh = _mesh(
        "mirror_cell.obj",
        _build_annular_cylinder_mesh(length=0.060, outer_radius=0.114, inner_radius=0.096, segments=48),
    )
    aperture_ring_mesh = _mesh(
        "aperture_ring.obj",
        _build_annular_cylinder_mesh(length=0.018, outer_radius=0.121, inner_radius=0.114, segments=48),
    )

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.090, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=tripod_steel,
        name="tripod_crown",
    )
    tripod.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.895)),
        material=tripod_steel,
        name="tripod_head",
    )
    latitude_seat_center = _offset(POLAR_JOINT_XYZ, POLAR_AXIS_DIR, -0.016)
    tripod.visual(
        Cylinder(radius=0.042, length=0.032),
        origin=Origin(xyz=latitude_seat_center, rpy=(0.0, -POLAR_TILT, 0.0)),
        material=tripod_steel,
        name="latitude_seat",
    )
    tripod.visual(
        Cylinder(radius=0.024, length=0.100),
        origin=Origin(
            xyz=_offset(POLAR_JOINT_XYZ, POLAR_AXIS_DIR, -0.066),
            rpy=(0.0, -POLAR_TILT, 0.0),
        ),
        material=tripod_steel,
        name="latitude_riser",
    )
    tripod.visual(
        Box((0.080, 0.095, 0.060)),
        origin=Origin(xyz=(0.040, 0.0, 0.925), rpy=(0.0, -POLAR_TILT, 0.0)),
        material=tripod_steel,
        name="latitude_wedge",
    )
    tripod.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=tripod_steel,
        name="spreader_hub",
    )
    tripod.visual(
        Cylinder(radius=0.115, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.396)),
        material=dark_metal,
        name="accessory_tray",
    )
    leg_top_radius = 0.080
    knee_radius = 0.250
    foot_radius = 0.560
    leg_top_z = 0.840
    knee_z = 0.490
    foot_z = 0.020
    brace_hub = (0.0, 0.0, 0.420)
    for index, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        top = (leg_top_radius * math.cos(angle), leg_top_radius * math.sin(angle), leg_top_z)
        knee = (knee_radius * math.cos(angle), knee_radius * math.sin(angle), knee_z)
        foot = (foot_radius * math.cos(angle), foot_radius * math.sin(angle), foot_z)
        _add_member(tripod, f"upper_leg_{index}", top, knee, radius=0.018, material=tripod_steel)
        _add_member(tripod, f"lower_leg_{index}", knee, foot, radius=0.014, material=tripod_steel)
        _add_member(tripod, f"tray_brace_{index}", brace_hub, knee, radius=0.008, material=tripod_steel)
        tripod.visual(
            Cylinder(radius=0.030, length=0.020),
            origin=Origin(xyz=(foot[0], foot[1], 0.010)),
            material=dark_metal,
            name=f"foot_pad_{index}",
        )
    tripod.inertial = Inertial.from_geometry(
        Box((1.20, 1.20, 1.02)),
        mass=8.6,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    polar = model.part("polar_assembly")
    polar.visual(
        Cylinder(radius=0.042, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=mount_paint,
        name="polar_base",
    )
    polar.visual(
        Cylinder(radius=0.029, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=mount_paint,
        name="polar_housing",
    )
    polar.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=trim_black,
        name="setting_circle",
    )
    polar.visual(
        Box((0.070, 0.046, 0.055)),
        origin=Origin(xyz=(0.055, 0.0, 0.112)),
        material=dark_metal,
        name="drive_motor",
    )
    polar.visual(
        Box((0.045, 0.110, 0.085)),
        origin=Origin(xyz=(0.006, 0.0, 0.243)),
        material=mount_paint,
        name="declination_carrier",
    )
    polar.visual(
        Box((0.042, 0.014, 0.150)),
        origin=Origin(xyz=(0.0, 0.060, 0.330)),
        material=mount_paint,
        name="left_cheek",
    )
    polar.visual(
        Box((0.042, 0.014, 0.150)),
        origin=Origin(xyz=(0.0, -0.060, 0.330)),
        material=mount_paint,
        name="right_cheek",
    )
    polar.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, 0.42)),
        mass=5.6,
        origin=Origin(xyz=(0.01, 0.0, 0.21)),
    )

    declination = model.part("declination_head")
    declination.visual(
        Box((0.042, 0.106, 0.042)),
        material=dark_metal,
        name="declination_hub",
    )
    declination.visual(Box((0.055, 0.055, 0.045)), material=mount_paint, name="hub_block")
    declination.visual(
        Box((0.060, 0.045, 0.030)),
        origin=Origin(xyz=(0.045, 0.0, 0.015)),
        material=mount_paint,
        name="saddle_stem",
    )
    declination.visual(
        Box((0.050, 0.048, 0.100)),
        origin=Origin(xyz=(0.080, 0.0, 0.065)),
        material=mount_paint,
        name="saddle_tower",
    )
    declination.visual(
        Box((0.138, 0.055, 0.014)),
        origin=Origin(xyz=(0.090, 0.0, 0.122)),
        material=dark_metal,
        name="saddle_plate",
    )
    declination.visual(
        Box((0.160, 0.015, 0.030)),
        origin=Origin(xyz=(0.118, 0.0325, 0.140)),
        material=dark_metal,
        name="saddle_clamp_left",
    )
    declination.visual(
        Box((0.160, 0.015, 0.030)),
        origin=Origin(xyz=(0.118, -0.0325, 0.140)),
        material=dark_metal,
        name="saddle_clamp_right",
    )
    declination.visual(
        Box((0.250, 0.026, 0.028)),
        origin=Origin(xyz=(-0.125, 0.0, -0.005)),
        material=dark_metal,
        name="counterweight_arm",
    )
    declination.visual(
        Box((0.030, 0.026, 0.090)),
        origin=Origin(xyz=(-0.230, 0.0, -0.045)),
        material=dark_metal,
        name="counterweight_drop",
    )
    declination.visual(
        Box((0.040, 0.045, 0.040)),
        origin=Origin(xyz=(-0.240, 0.0, -0.085)),
        material=dark_metal,
        name="counterweight_clamp",
    )
    declination.visual(
        Cylinder(radius=0.012, length=0.240),
        origin=Origin(xyz=(-0.350, 0.0, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight_shaft",
    )
    declination.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(-0.275, 0.0, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight_collar",
    )
    declination.visual(
        Cylinder(radius=0.052, length=0.048),
        origin=Origin(xyz=(-0.335, 0.0, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=counterweight_metal,
        name="weight_outer",
    )
    declination.visual(
        Cylinder(radius=0.046, length=0.048),
        origin=Origin(xyz=(-0.405, 0.0, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=counterweight_metal,
        name="weight_inner",
    )
    declination.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(-0.462, 0.0, -0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="shaft_stop",
    )
    declination.inertial = Inertial.from_geometry(
        Box((0.72, 0.11, 0.30)),
        mass=6.0,
        origin=Origin(xyz=(-0.14, 0.0, 0.0)),
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(tube_shell_mesh, material=tube_paint, name="tube_shell")
    optical_tube.visual(
        aperture_ring_mesh,
        origin=Origin(xyz=(0.451, 0.0, 0.0)),
        material=trim_black,
        name="aperture_ring",
    )
    optical_tube.visual(
        mirror_cell_mesh,
        origin=Origin(xyz=(-0.420, 0.0, 0.0)),
        material=trim_black,
        name="mirror_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.098, length=0.012),
        origin=Origin(xyz=(-0.404, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror_glass,
        name="primary_mirror",
    )
    optical_tube.visual(
        Box((0.003, 0.228, 0.010)),
        origin=Origin(xyz=(0.292, 0.0, 0.0)),
        material=trim_black,
        name="spider_vane_y",
    )
    optical_tube.visual(
        Box((0.003, 0.010, 0.228)),
        origin=Origin(xyz=(0.292, 0.0, 0.0)),
        material=trim_black,
        name="spider_vane_z",
    )
    optical_tube.visual(
        Box((0.020, 0.040, 0.030)),
        origin=Origin(xyz=(0.292, 0.0, 0.0), rpy=(0.0, math.pi / 4.0, 0.0)),
        material=trim_black,
        name="secondary_mirror",
    )
    optical_tube.visual(tube_ring_mesh, origin=Origin(xyz=(-0.170, 0.0, 0.0)), material=dark_metal, name="rear_ring")
    optical_tube.visual(tube_ring_mesh, origin=Origin(xyz=(0.160, 0.0, 0.0)), material=dark_metal, name="front_ring")
    optical_tube.visual(
        Box((0.028, 0.045, 0.090)),
        origin=Origin(xyz=(-0.170, 0.0, -0.104)),
        material=dark_metal,
        name="rear_ring_post",
    )
    optical_tube.visual(
        Box((0.028, 0.045, 0.090)),
        origin=Origin(xyz=(0.160, 0.0, -0.104)),
        material=dark_metal,
        name="front_ring_post",
    )
    optical_tube.visual(
        Box((0.220, 0.050, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, -0.135)),
        material=dark_metal,
        name="dovetail_rail",
    )
    optical_tube.visual(
        Box((0.110, 0.045, 0.016)),
        origin=Origin(xyz=(0.080, 0.0, -0.119)),
        material=dark_metal,
        name="saddle_block",
    )
    optical_tube.visual(
        Box((0.060, 0.045, 0.008)),
        origin=Origin(xyz=(0.195, 0.0, 0.118)),
        material=trim_black,
        name="focuser_base",
    )
    optical_tube.visual(
        Cylinder(radius=0.021, length=0.058),
        origin=Origin(xyz=(0.195, 0.0, 0.143)),
        material=trim_black,
        name="focuser_housing",
    )
    optical_tube.visual(
        Cylinder(radius=0.015, length=0.086),
        origin=Origin(xyz=(0.195, 0.0, 0.211)),
        material=dark_metal,
        name="eyepiece",
    )
    optical_tube.visual(
        Box((0.085, 0.022, 0.012)),
        origin=Origin(xyz=(0.050, 0.0, 0.124)),
        material=dark_metal,
        name="finder_base",
    )
    optical_tube.visual(
        Box((0.016, 0.020, 0.030)),
        origin=Origin(xyz=(0.000, 0.0, 0.145)),
        material=dark_metal,
        name="finder_rear_post",
    )
    optical_tube.visual(
        Box((0.016, 0.020, 0.030)),
        origin=Origin(xyz=(0.100, 0.0, 0.145)),
        material=dark_metal,
        name="finder_front_post",
    )
    optical_tube.visual(
        Cylinder(radius=0.017, length=0.180),
        origin=Origin(xyz=(0.050, 0.0, 0.177), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_black,
        name="finder_scope",
    )
    optical_tube.inertial = Inertial.from_geometry(
        Box((0.94, 0.27, 0.38)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "polar_axis",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=polar,
        origin=Origin(xyz=POLAR_JOINT_XYZ, rpy=(0.0, -POLAR_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2),
    )
    model.articulation(
        "declination_axis",
        ArticulationType.REVOLUTE,
        parent=polar,
        child=declination,
        origin=Origin(xyz=DECLINATION_JOINT_XYZ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=26.0, velocity=0.9, lower=-0.10, upper=0.90),
    )
    model.articulation(
        "tube_mount",
        ArticulationType.FIXED,
        parent=declination,
        child=optical_tube,
        origin=Origin(xyz=TUBE_MOUNT_XYZ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    tripod = object_model.get_part("tripod")
    polar = object_model.get_part("polar_assembly")
    declination = object_model.get_part("declination_head")
    optical_tube = object_model.get_part("optical_tube")

    polar_axis = object_model.get_articulation("polar_axis")
    declination_axis = object_model.get_articulation("declination_axis")

    latitude_seat = tripod.get_visual("latitude_seat")
    polar_base = polar.get_visual("polar_base")
    left_cheek = polar.get_visual("left_cheek")
    right_cheek = polar.get_visual("right_cheek")
    declination_hub = declination.get_visual("declination_hub")
    saddle_plate = declination.get_visual("saddle_plate")
    saddle_clamp_left = declination.get_visual("saddle_clamp_left")
    saddle_clamp_right = declination.get_visual("saddle_clamp_right")
    tube_shell = optical_tube.get_visual("tube_shell")
    primary_mirror = optical_tube.get_visual("primary_mirror")
    secondary_mirror = optical_tube.get_visual("secondary_mirror")
    spider_vane_y = optical_tube.get_visual("spider_vane_y")
    dovetail_rail = optical_tube.get_visual("dovetail_rail")
    saddle_block = optical_tube.get_visual("saddle_block")
    focuser_base = optical_tube.get_visual("focuser_base")
    focuser_housing = optical_tube.get_visual("focuser_housing")
    eyepiece = optical_tube.get_visual("eyepiece")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "polar_axis_orientation",
        tuple(polar_axis.axis) == (0.0, 0.0, 1.0),
        details=f"Expected polar axis local vector (0, 0, 1), got {polar_axis.axis!r}",
    )
    ctx.check(
        "declination_axis_orientation",
        tuple(declination_axis.axis) == (0.0, 1.0, 0.0),
        details=f"Expected declination axis local vector (0, 1, 0), got {declination_axis.axis!r}",
    )

    ctx.expect_contact(polar, tripod, elem_a=polar_base, elem_b=latitude_seat, contact_tol=5e-4)
    ctx.expect_contact(declination, polar, elem_a=declination_hub, elem_b=left_cheek, contact_tol=5e-4)
    ctx.expect_contact(declination, polar, elem_a=declination_hub, elem_b=right_cheek, contact_tol=5e-4)
    ctx.expect_contact(optical_tube, declination, elem_a=saddle_block, elem_b=saddle_clamp_left, contact_tol=0.003)
    ctx.expect_contact(optical_tube, declination, elem_a=saddle_block, elem_b=saddle_clamp_right, contact_tol=0.003)
    ctx.expect_contact(optical_tube, optical_tube, elem_a=focuser_base, elem_b=focuser_housing, contact_tol=5e-4)
    ctx.expect_contact(optical_tube, optical_tube, elem_a=eyepiece, elem_b=focuser_housing, contact_tol=5e-4)
    ctx.expect_overlap(optical_tube, declination, elem_a=dovetail_rail, elem_b=saddle_plate, axes="xy", min_overlap=0.05)
    ctx.expect_within(
        optical_tube,
        optical_tube,
        axes="yz",
        inner_elem=primary_mirror,
        outer_elem=tube_shell,
        margin=0.0,
    )
    ctx.expect_within(
        optical_tube,
        optical_tube,
        axes="yz",
        inner_elem=secondary_mirror,
        outer_elem=tube_shell,
        margin=0.0,
    )
    ctx.expect_gap(
        optical_tube,
        optical_tube,
        axis="x",
        min_gap=0.30,
        positive_elem=spider_vane_y,
        negative_elem=primary_mirror,
        name="primary_to_secondary_light_path_length",
    )

    limits = declination_axis.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({declination_axis: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="declination_axis_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="declination_axis_lower_no_floating")
            ctx.expect_contact(optical_tube, declination, elem_a=saddle_block, elem_b=saddle_clamp_left, contact_tol=0.003)
            ctx.expect_contact(optical_tube, declination, elem_a=saddle_block, elem_b=saddle_clamp_right, contact_tol=0.003)
        with ctx.pose({declination_axis: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="declination_axis_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="declination_axis_upper_no_floating")
            ctx.expect_contact(optical_tube, declination, elem_a=saddle_block, elem_b=saddle_clamp_left, contact_tol=0.003)
            ctx.expect_contact(optical_tube, declination, elem_a=saddle_block, elem_b=saddle_clamp_right, contact_tol=0.003)

    with ctx.pose({polar_axis: 1.10, declination_axis: 0.78}):
        ctx.fail_if_parts_overlap_in_current_pose(name="operating_pose_clear")
        ctx.fail_if_isolated_parts(name="operating_pose_supported")
        ctx.expect_contact(optical_tube, declination, elem_a=saddle_block, elem_b=saddle_clamp_left, contact_tol=0.003)
        ctx.expect_contact(optical_tube, declination, elem_a=saddle_block, elem_b=saddle_clamp_right, contact_tol=0.003)

    with ctx.pose({polar_axis: -1.10, declination_axis: 0.12}):
        ctx.fail_if_parts_overlap_in_current_pose(name="counterposed_clear")
        ctx.fail_if_isolated_parts(name="counterposed_supported")
        ctx.expect_contact(declination, polar, elem_a=declination_hub, elem_b=left_cheek, contact_tol=5e-4)
        ctx.expect_contact(declination, polar, elem_a=declination_hub, elem_b=right_cheek, contact_tol=5e-4)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
