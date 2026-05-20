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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _circle_profile(radius: float, segments: int = 48, *, reverse: bool = False) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]
    return list(reversed(points)) if reverse else points


def _annulus_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float) -> object:
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius, reverse=True)],
            length,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def _segment_pose(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        (low[0] + high[0]) * 0.5,
        (low[1] + high[1]) * 0.5,
        (low[2] + high[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="goto_newtonian_reflector")

    tripod_black = model.material("tripod_black", rgba=(0.10, 0.11, 0.12, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    light_gray = model.material("light_gray", rgba=(0.71, 0.73, 0.77, 1.0))
    tube_white = model.material("tube_white", rgba=(0.93, 0.95, 0.97, 1.0))
    satin_black = model.material("satin_black", rgba=(0.05, 0.05, 0.06, 1.0))
    anodized = model.material("anodized", rgba=(0.56, 0.60, 0.64, 1.0))

    arm_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (-0.018, 0.196, 0.055),
                (-0.010, 0.202, 0.170),
                (0.005, 0.208, 0.315),
                (0.018, 0.205, 0.455),
            ],
            profile=rounded_rect_profile(0.062, 0.084, radius=0.014, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(1.0, 0.0, 0.0),
        ),
        "fork_arm_shell",
    )
    bearing_mesh = _annulus_mesh(
        "altitude_bearing_ring",
        outer_radius=0.054,
        inner_radius=0.029,
        length=0.032,
    )
    tube_mesh = _annulus_mesh(
        "ota_tube_shell",
        outer_radius=0.125,
        inner_radius=0.117,
        length=0.880,
    )
    front_rim_mesh = _annulus_mesh(
        "ota_front_rim",
        outer_radius=0.130,
        inner_radius=0.117,
        length=0.022,
    )
    rear_cell_mesh = _annulus_mesh(
        "ota_rear_cell",
        outer_radius=0.128,
        inner_radius=0.104,
        length=0.060,
    )
    cradle_ring_mesh = _annulus_mesh(
        "cradle_ring_band",
        outer_radius=0.153,
        inner_radius=0.123,
        length=0.028,
    )

    tripod_base = model.part("tripod_base")

    head_top_z = 1.030
    tripod_base.visual(
        Cylinder(radius=0.130, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, head_top_z - 0.009)),
        material=mount_gray,
        name="tripod_head",
    )
    tripod_base.visual(
        Cylinder(radius=0.054, length=0.214),
        origin=Origin(xyz=(0.0, 0.0, 0.905)),
        material=mount_gray,
        name="center_column",
    )
    tripod_base.visual(
        Cylinder(radius=0.080, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.785)),
        material=tripod_black,
        name="leg_crown",
    )
    tripod_base.visual(
        Cylinder(radius=0.050, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=mount_gray,
        name="spreader_hub",
    )

    leg_top_radius = 0.082
    leg_bottom_radius = 0.560
    leg_top_z = 0.785
    leg_bottom_z = 0.020
    for index, angle in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        top = (
            leg_top_radius * math.cos(angle),
            leg_top_radius * math.sin(angle),
            leg_top_z,
        )
        bottom = (
            leg_bottom_radius * math.cos(angle),
            leg_bottom_radius * math.sin(angle),
            leg_bottom_z,
        )
        leg_pose, leg_length = _segment_pose(top, bottom)
        tripod_base.visual(
            Cylinder(radius=0.018, length=leg_length),
            origin=leg_pose,
            material=tripod_black,
            name=f"leg_{index}",
        )
        tripod_base.visual(
            Box((0.060, 0.032, 0.040)),
            origin=Origin(
                xyz=(
                    0.098 * math.cos(angle),
                    0.098 * math.sin(angle),
                    0.800,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_gray,
            name=f"leg_hinge_{index}",
        )
        spreader_end = (
            0.290 * math.cos(angle),
            0.290 * math.sin(angle),
            0.455,
        )
        spreader_pose, spreader_length = _segment_pose((0.0, 0.0, 0.555), spreader_end)
        tripod_base.visual(
            Cylinder(radius=0.010, length=spreader_length),
            origin=spreader_pose,
            material=anodized,
            name=f"spreader_{index}",
        )
        tripod_base.visual(
            Cylinder(radius=0.030, length=0.012),
            origin=Origin(
                xyz=(
                    bottom[0],
                    bottom[1],
                    0.006,
                ),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=mount_gray,
            name=f"foot_{index}",
        )
    tripod_base.inertial = Inertial.from_geometry(
        Box((1.200, 1.200, 1.040)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
    )

    azimuth_mount = model.part("azimuth_mount")
    azimuth_mount.visual(
        Cylinder(radius=0.098, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=mount_gray,
        name="azimuth_base",
    )
    azimuth_mount.visual(
        Cylinder(radius=0.118, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=light_gray,
        name="azimuth_plate",
    )
    azimuth_mount.visual(
        Box((0.118, 0.170, 0.074)),
        origin=Origin(xyz=(-0.004, 0.000, 0.062)),
        material=mount_gray,
        name="azimuth_drive_housing",
    )
    azimuth_mount.visual(
        Box((0.095, 0.080, 0.126)),
        origin=Origin(xyz=(-0.010, 0.148, 0.112)),
        material=mount_gray,
        name="arm_gusset",
    )
    azimuth_mount.visual(
        Box((0.074, 0.112, 0.128)),
        origin=Origin(xyz=(-0.004, 0.122, 0.112)),
        material=mount_gray,
        name="arm_bridge",
    )
    azimuth_mount.visual(
        Box((0.060, 0.086, 0.390)),
        origin=Origin(xyz=(0.004, 0.206, 0.240)),
        material=mount_gray,
        name="arm_spine",
    )
    azimuth_mount.visual(
        arm_mesh,
        material=light_gray,
        name="arm_shell",
    )
    azimuth_mount.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.018, 0.165, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="altitude_bearing",
    )
    azimuth_mount.visual(
        Box((0.080, 0.090, 0.028)),
        origin=Origin(xyz=(0.042, 0.040, 0.090)),
        material=satin_black,
        name="motor_cover",
    )
    azimuth_mount.inertial = Inertial.from_geometry(
        Box((0.260, 0.280, 0.500)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.120, 0.250)),
    )

    optical_tube = model.part("optical_tube_assembly")
    tube_center_y = -0.165

    optical_tube.visual(
        tube_mesh,
        origin=Origin(xyz=(0.205, tube_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_white,
        name="tube_shell",
    )
    optical_tube.visual(
        front_rim_mesh,
        origin=Origin(xyz=(0.634, tube_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="front_rim",
    )
    optical_tube.visual(
        rear_cell_mesh,
        origin=Origin(xyz=(-0.205, tube_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rear_cell",
    )
    optical_tube.visual(
        Box((0.004, 0.238, 0.006)),
        origin=Origin(xyz=(0.540, tube_center_y, 0.0)),
        material=anodized,
        name="spider_vane_horizontal",
    )
    optical_tube.visual(
        Box((0.004, 0.006, 0.238)),
        origin=Origin(xyz=(0.540, tube_center_y, 0.0)),
        material=anodized,
        name="spider_vane_vertical",
    )
    optical_tube.visual(
        Box((0.036, 0.018, 0.026)),
        origin=Origin(xyz=(0.540, tube_center_y + 0.010, 0.0), rpy=(0.0, math.pi / 4.0, 0.0)),
        material=satin_black,
        name="secondary_holder",
    )
    optical_tube.visual(
        Box((0.068, 0.050, 0.040)),
        origin=Origin(xyz=(0.410, tube_center_y + 0.090, 0.095), rpy=(0.20, 0.0, 0.0)),
        material=satin_black,
        name="focuser_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(xyz=(0.410, tube_center_y + 0.122, 0.098), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="eyepiece_tube",
    )
    optical_tube.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=Origin(xyz=(0.410, tube_center_y + 0.176, 0.098), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="eyepiece",
    )
    optical_tube.visual(
        Cylinder(radius=0.020, length=0.180),
        origin=Origin(xyz=(0.335, tube_center_y - 0.008, 0.165), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="finder_scope",
    )
    optical_tube.visual(
        Box((0.020, 0.024, 0.040)),
        origin=Origin(xyz=(0.290, tube_center_y, 0.135)),
        material=anodized,
        name="finder_bracket_rear",
    )
    optical_tube.visual(
        Box((0.020, 0.024, 0.040)),
        origin=Origin(xyz=(0.380, tube_center_y, 0.135)),
        material=anodized,
        name="finder_bracket_front",
    )
    optical_tube.visual(
        cradle_ring_mesh,
        origin=Origin(xyz=(0.020, tube_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_gray,
        name="cradle_ring_rear",
    )
    optical_tube.visual(
        cradle_ring_mesh,
        origin=Origin(xyz=(0.180, tube_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_gray,
        name="cradle_ring_front",
    )
    optical_tube.visual(
        Box((0.248, 0.050, 0.116)),
        origin=Origin(xyz=(0.102, -0.045, 0.0)),
        material=mount_gray,
        name="cradle_rail",
    )
    optical_tube.visual(
        Box((0.122, 0.100, 0.148)),
        origin=Origin(xyz=(0.020, -0.070, 0.0)),
        material=mount_gray,
        name="trunnion_block",
    )
    optical_tube.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.000, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="trunnion_shaft",
    )
    optical_tube.inertial = Inertial.from_geometry(
        Box((0.900, 0.360, 0.360)),
        mass=6.4,
        origin=Origin(xyz=(0.205, -0.110, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=azimuth_mount,
        origin=Origin(xyz=(0.0, 0.0, head_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=azimuth_mount,
        child=optical_tube,
        origin=Origin(xyz=(0.018, 0.165, 0.455)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.9, lower=-0.15, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod = object_model.get_part("tripod_base")
    mount = object_model.get_part("azimuth_mount")
    tube = object_model.get_part("optical_tube_assembly")
    azimuth = object_model.get_articulation("azimuth_rotation")
    altitude = object_model.get_articulation("altitude_axis")

    ctx.check(
        "azimuth joint is continuous",
        azimuth.joint_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={azimuth.joint_type}",
    )
    altitude_limits = altitude.motion_limits
    ctx.check(
        "altitude joint opens upward through a realistic range",
        altitude_limits is not None
        and altitude_limits.lower is not None
        and altitude_limits.upper is not None
        and altitude.axis == (0.0, -1.0, 0.0)
        and altitude_limits.lower <= 0.0
        and altitude_limits.upper >= 1.2,
        details=f"axis={altitude.axis}, limits={altitude_limits}",
    )

    with ctx.pose({azimuth: 0.0, altitude: 0.0}):
        ctx.expect_gap(
            mount,
            tripod,
            axis="z",
            positive_elem="azimuth_base",
            negative_elem="tripod_head",
            max_gap=0.001,
            max_penetration=0.0,
            name="azimuth drive sits flush on tripod head",
        )
        ctx.expect_overlap(
            mount,
            tripod,
            axes="xy",
            elem_a="azimuth_base",
            elem_b="tripod_head",
            min_overlap=0.180,
            name="azimuth drive remains centered on the tripod head",
        )
        ctx.expect_within(
            tube,
            mount,
            axes="xz",
            inner_elem="trunnion_shaft",
            outer_elem="altitude_bearing",
            margin=0.004,
            name="trunnion shaft stays inside the altitude bearing envelope",
        )
        ctx.expect_overlap(
            tube,
            mount,
            axes="y",
            elem_a="trunnion_shaft",
            elem_b="altitude_bearing",
            min_overlap=0.014,
            name="trunnion shaft remains engaged along the bearing axis",
        )
        ctx.expect_gap(
            mount,
            tube,
            axis="y",
            positive_elem="arm_shell",
            negative_elem="tube_shell",
            min_gap=0.030,
            name="single fork arm clears the optical tube body",
        )
        ctx.expect_gap(
            tube,
            tripod,
            axis="z",
            positive_elem="tube_shell",
            negative_elem="tripod_head",
            min_gap=0.250,
            name="optical tube clears the tripod head at rest",
        )

        rest_front = _aabb_center(ctx.part_element_world_aabb(tube, elem="front_rim"))
        rest_bearing = _aabb_center(ctx.part_element_world_aabb(mount, elem="altitude_bearing"))

    with ctx.pose({altitude: 1.10}):
        raised_front = _aabb_center(ctx.part_element_world_aabb(tube, elem="front_rim"))
        ctx.expect_gap(
            tube,
            tripod,
            axis="z",
            positive_elem="front_rim",
            negative_elem="tripod_head",
            min_gap=0.700,
            name="raised telescope points well above the tripod head",
        )

    ctx.check(
        "positive altitude motion lifts the front aperture",
        rest_front is not None and raised_front is not None and raised_front[2] > rest_front[2] + 0.30,
        details=f"rest_front={rest_front}, raised_front={raised_front}",
    )

    with ctx.pose({azimuth: 0.90}):
        rotated_bearing = _aabb_center(ctx.part_element_world_aabb(mount, elem="altitude_bearing"))

    ctx.check(
        "azimuth rotation swings the fork arm around the tripod axis",
        rest_bearing is not None
        and rotated_bearing is not None
        and (
            abs(rotated_bearing[0] - rest_bearing[0]) > 0.08
            or abs(rotated_bearing[1] - rest_bearing[1]) > 0.08
        ),
        details=f"rest_bearing={rest_bearing}, rotated_bearing={rotated_bearing}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
