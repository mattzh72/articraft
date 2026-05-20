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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _vec_add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _cylindrical_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    axis: str = "z",
    segments: int = 48,
):
    geom = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -length * 0.5),
            (outer_radius, length * 0.5),
        ],
        [
            (inner_radius, -length * 0.5),
            (inner_radius, length * 0.5),
        ],
        segments=segments,
    )
    if axis == "x":
        geom.rotate_y(math.pi / 2.0)
    elif axis == "y":
        geom.rotate_x(math.pi / 2.0)
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="newtonian_reflector_equatorial_mount")

    tripod_black = model.material("tripod_black", rgba=(0.12, 0.12, 0.13, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    tube_white = model.material("tube_white", rgba=(0.91, 0.92, 0.93, 1.0))
    wrinkle_black = model.material("wrinkle_black", rgba=(0.07, 0.07, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.79, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    mirror_glass = model.material("mirror_glass", rgba=(0.78, 0.82, 0.86, 1.0))

    latitude = math.radians(42.0)
    polar_tilt = math.pi / 2.0 - latitude
    polar_axis = (0.0, math.sin(polar_tilt), math.cos(polar_tilt))
    ra_joint_xyz = (0.0, 0.0, 1.42)

    tube_length = 0.72
    tube_outer_radius = 0.095
    tube_inner_radius = 0.089
    tube_center_z = 0.155

    tube_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.098, -tube_length * 0.5 - 0.010),
            (0.096, -tube_length * 0.5),
            (tube_outer_radius, -tube_length * 0.5 + 0.006),
            (tube_outer_radius, tube_length * 0.5 - 0.006),
            (0.096, tube_length * 0.5),
            (0.098, tube_length * 0.5 + 0.010),
        ],
        [
            (tube_inner_radius, -tube_length * 0.5 + 0.004),
            (tube_inner_radius, tube_length * 0.5 - 0.004),
        ],
        segments=72,
    )
    tube_shell_geom.rotate_y(math.pi / 2.0)
    tube_shell_mesh = _save_mesh("newtonian_tube_shell", tube_shell_geom)
    ring_shell_mesh = _cylindrical_shell_mesh(
        "tube_ring_shell",
        outer_radius=0.105,
        inner_radius=0.096,
        length=0.020,
        axis="x",
        segments=56,
    )
    mirror_cell_mesh = _cylindrical_shell_mesh(
        "mirror_cell_shell",
        outer_radius=0.090,
        inner_radius=0.070,
        length=0.024,
        axis="x",
        segments=56,
    )
    focuser_sleeve_mesh = _cylindrical_shell_mesh(
        "focuser_sleeve_shell",
        outer_radius=0.028,
        inner_radius=0.024,
        length=0.090,
        axis="z",
        segments=48,
    )
    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.080, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        material=tripod_black,
        name="tripod_hub",
    )
    tripod_base.visual(
        Cylinder(radius=0.100, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.490)),
        material=mount_gray,
        name="spreader_tray",
    )
    tripod_base.visual(
        Cylinder(radius=0.055, length=0.480),
        origin=Origin(xyz=(0.0, 0.0, 1.070)),
        material=tripod_black,
        name="pier_column",
    )
    tripod_base.visual(
        Cylinder(radius=0.074, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.335)),
        material=mount_gray,
        name="top_collar",
    )
    tripod_base.visual(
        Box((0.220, 0.160, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 1.330)),
        material=mount_gray,
        name="latitude_block",
    )
    tripod_base.visual(
        Cylinder(radius=0.060, length=0.200),
        origin=Origin(
            xyz=_vec_add(ra_joint_xyz, _vec_scale(polar_axis, -0.100)),
            rpy=(-polar_tilt, 0.0, 0.0),
        ),
        material=mount_gray,
        name="polar_housing",
    )
    tripod_base.visual(
        Box((0.130, 0.080, 0.100)),
        origin=Origin(xyz=(0.0, -0.120, 1.315)),
        material=mount_gray,
        name="ra_drive_box",
    )
    tripod_base.visual(
        Cylinder(radius=0.026, length=0.080),
        origin=Origin(xyz=(0.0, -0.165, 1.315), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrinkle_black,
        name="ra_motor",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_geom = tube_from_spline_points(
            [
                (0.055 * c, 0.055 * s, 0.795),
                (0.185 * c, 0.185 * s, 0.565),
                (0.580 * c, 0.580 * s, 0.030),
            ],
            radius=0.017,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        tripod_base.visual(
            _save_mesh(f"tripod_leg_{index}", leg_geom),
            material=tripod_black,
            name=f"leg_{index}",
        )
        tripod_base.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(xyz=(0.580 * c, 0.580 * s, 0.023)),
            material=rubber,
            name=f"foot_{index}",
        )
        _add_member(
            tripod_base,
            (0.080 * c, 0.080 * s, 0.490),
            (0.260 * c, 0.260 * s, 0.420),
            radius=0.009,
            material=mount_gray,
            name=f"spreader_arm_{index}",
        )

    tripod_base.inertial = Inertial.from_geometry(
        Box((1.25, 1.25, 1.55)),
        mass=19.0,
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
    )

    ra_assembly = model.part("ra_assembly")
    ra_assembly.visual(
        Cylinder(radius=0.070, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=mount_gray,
        name="ra_base_cap",
    )
    ra_assembly.visual(
        Cylinder(radius=0.050, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=mount_gray,
        name="ra_axis_shaft",
    )
    ra_assembly.visual(
        Box((0.120, 0.180, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=mount_gray,
        name="dec_bridge",
    )
    ra_assembly.visual(
        Box((0.084, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.091, 0.154)),
        material=mount_gray,
        name="dec_yoke_left_lower",
    )
    ra_assembly.visual(
        Box((0.084, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, -0.091, 0.246)),
        material=mount_gray,
        name="dec_yoke_left_upper",
    )
    ra_assembly.visual(
        Box((0.084, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.091, 0.154)),
        material=mount_gray,
        name="dec_yoke_right_lower",
    )
    ra_assembly.visual(
        Box((0.084, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.091, 0.246)),
        material=mount_gray,
        name="dec_yoke_right_upper",
    )
    for side_sign in (-1.0, 1.0):
        for x_sign in (-1.0, 1.0):
            ra_assembly.visual(
                Box((0.016, 0.018, 0.136)),
                origin=Origin(xyz=(0.042 * x_sign, 0.091 * side_sign, 0.200)),
                material=mount_gray,
                name=f"fork_upright_{int(x_sign):+d}_{int(side_sign):+d}",
            )
    ra_assembly.visual(
        Box((0.090, 0.050, 0.070)),
        origin=Origin(xyz=(0.020, -0.110, 0.095)),
        material=wrinkle_black,
        name="dec_motor_box",
    )
    ra_assembly.inertial = Inertial.from_geometry(
        Box((0.260, 0.280, 0.320)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    dec_head = model.part("dec_head")
    dec_head.visual(
        Cylinder(radius=0.018, length=0.200),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="declination_shaft",
    )
    dec_head.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, -0.106, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mount_gray,
        name="left_trunnion",
    )
    dec_head.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.106, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mount_gray,
        name="right_trunnion",
    )
    dec_head.visual(
        Box((0.110, 0.090, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=mount_gray,
        name="dec_hub",
    )
    dec_head.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(-0.030, 0.0, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_gray,
        name="ota_side_collar",
    )
    dec_head.visual(
        Box((0.090, 0.080, 0.042)),
        origin=Origin(xyz=(0.100, 0.0, -0.039)),
        material=mount_gray,
        name="saddle_pedestal",
    )
    dec_head.visual(
        Box((0.180, 0.100, 0.018)),
        origin=Origin(xyz=(0.160, 0.0, -0.009)),
        material=brushed_aluminum,
        name="saddle_bed",
    )
    dec_head.visual(
        Box((0.180, 0.014, 0.028)),
        origin=Origin(xyz=(0.160, -0.036, 0.005)),
        material=mount_gray,
        name="saddle_clamp_left",
    )
    dec_head.visual(
        Box((0.180, 0.014, 0.028)),
        origin=Origin(xyz=(0.160, 0.036, 0.005)),
        material=mount_gray,
        name="saddle_clamp_right",
    )
    dec_head.visual(
        Cylinder(radius=0.006, length=0.038),
        origin=Origin(xyz=(0.250, 0.060, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrinkle_black,
        name="saddle_lock_knob",
    )
    dec_head.visual(
        Cylinder(radius=0.012, length=0.460),
        origin=Origin(xyz=(-0.240, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="counterweight_shaft",
    )
    dec_head.visual(
        Cylinder(radius=0.055, length=0.070),
        origin=Origin(xyz=(-0.300, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_gray,
        name="counterweight_1",
    )
    dec_head.visual(
        Cylinder(radius=0.055, length=0.070),
        origin=Origin(xyz=(-0.390, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_gray,
        name="counterweight_2",
    )
    dec_head.visual(
        Cylinder(radius=0.017, length=0.025),
        origin=Origin(xyz=(-0.478, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wrinkle_black,
        name="counterweight_stop",
    )
    dec_head.inertial = Inertial.from_geometry(
        Box((0.760, 0.260, 0.240)),
        mass=9.0,
        origin=Origin(xyz=(-0.140, 0.0, 0.0)),
    )

    ota_tube = model.part("ota_tube")
    ota_tube.visual(
        Box((0.300, 0.038, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=mount_gray,
        name="dovetail_bar",
    )
    ota_tube.visual(
        Box((0.300, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.022, 0.017), rpy=(0.45, 0.0, 0.0)),
        material=mount_gray,
        name="dovetail_left_flank",
    )
    ota_tube.visual(
        Box((0.300, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.022, 0.017), rpy=(-0.45, 0.0, 0.0)),
        material=mount_gray,
        name="dovetail_right_flank",
    )
    ota_tube.visual(
        Box((0.035, 0.050, 0.040)),
        origin=Origin(xyz=(-0.130, 0.0, 0.032)),
        material=mount_gray,
        name="rear_ring_foot",
    )
    ota_tube.visual(
        Box((0.035, 0.050, 0.040)),
        origin=Origin(xyz=(0.130, 0.0, 0.032)),
        material=mount_gray,
        name="front_ring_foot",
    )
    ota_tube.visual(
        ring_shell_mesh,
        origin=Origin(xyz=(-0.130, 0.0, tube_center_z)),
        material=mount_gray,
        name="rear_tube_ring",
    )
    ota_tube.visual(
        ring_shell_mesh,
        origin=Origin(xyz=(0.130, 0.0, tube_center_z)),
        material=mount_gray,
        name="front_tube_ring",
    )
    ota_tube.visual(
        tube_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, tube_center_z)),
        material=tube_white,
        name="tube_shell",
    )
    ota_tube.visual(
        mirror_cell_mesh,
        origin=Origin(xyz=(-0.300, 0.0, tube_center_z)),
        material=mount_gray,
        name="mirror_cell",
    )
    ota_tube.visual(
        Cylinder(radius=0.072, length=0.018),
        origin=Origin(xyz=(-0.300, 0.0, tube_center_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror_glass,
        name="primary_mirror",
    )
    ota_tube.visual(
        Box((0.004, 0.188, 0.004)),
        origin=Origin(xyz=(0.245, 0.0, tube_center_z)),
        material=steel,
        name="spider_vane_horizontal",
    )
    ota_tube.visual(
        Box((0.004, 0.004, 0.188)),
        origin=Origin(xyz=(0.245, 0.0, tube_center_z)),
        material=steel,
        name="spider_vane_vertical",
    )
    ota_tube.visual(
        Box((0.016, 0.020, 0.020)),
        origin=Origin(xyz=(0.245, 0.0, tube_center_z)),
        material=wrinkle_black,
        name="secondary_hub",
    )
    ota_tube.visual(
        Box((0.012, 0.032, 0.044)),
        origin=Origin(
            xyz=(0.245, 0.0, tube_center_z),
            rpy=(0.0, math.pi / 4.0, math.pi / 4.0),
        ),
        material=wrinkle_black,
        name="secondary_mirror",
    )
    ota_tube.visual(
        Box((0.080, 0.060, 0.024)),
        origin=Origin(xyz=(0.120, 0.0, 0.260)),
        material=wrinkle_black,
        name="focuser_base",
    )
    ota_tube.visual(
        focuser_sleeve_mesh,
        origin=Origin(xyz=(0.120, 0.0, 0.305)),
        material=wrinkle_black,
        name="focuser_sleeve",
    )
    ota_tube.visual(
        Box((0.050, 0.018, 0.018)),
        origin=Origin(xyz=(0.120, 0.037, 0.281)),
        material=wrinkle_black,
        name="pinion_housing",
    )
    ota_tube.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.106, 0.061, 0.281), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrinkle_black,
        name="focus_knob_left",
    )
    ota_tube.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.134, 0.056, 0.281), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrinkle_black,
        name="focus_knob_right",
    )
    ota_tube.inertial = Inertial.from_geometry(
        Box((0.820, 0.240, 0.320)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )

    focuser_drawtube = model.part("focuser_drawtube")
    focuser_drawtube.visual(
        Cylinder(radius=0.021, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=wrinkle_black,
        name="drawtube_shaft",
    )
    focuser_drawtube.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=wrinkle_black,
        name="drawtube_flange",
    )
    focuser_drawtube.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=wrinkle_black,
        name="eyepiece_holder",
    )
    focuser_drawtube.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=brushed_aluminum,
        name="compression_ring_cap",
    )
    focuser_drawtube.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.180)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    model.articulation(
        "right_ascension_axis",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=ra_assembly,
        origin=Origin(xyz=ra_joint_xyz, rpy=(-polar_tilt, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55),
    )
    model.articulation(
        "declination_axis",
        ArticulationType.REVOLUTE,
        parent=ra_assembly,
        child=dec_head,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.60,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "saddle_to_ota",
        ArticulationType.FIXED,
        parent=dec_head,
        child=ota_tube,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
    )
    model.articulation(
        "focuser_travel",
        ArticulationType.PRISMATIC,
        parent=ota_tube,
        child=focuser_drawtube,
        origin=Origin(xyz=(0.120, 0.0, 0.272)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.030,
            lower=0.0,
            upper=0.035,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tripod_base = object_model.get_part("tripod_base")
    ra_assembly = object_model.get_part("ra_assembly")
    dec_head = object_model.get_part("dec_head")
    ota_tube = object_model.get_part("ota_tube")
    focuser_drawtube = object_model.get_part("focuser_drawtube")

    right_ascension_axis = object_model.get_articulation("right_ascension_axis")
    declination_axis = object_model.get_articulation("declination_axis")
    focuser_travel = object_model.get_articulation("focuser_travel")

    ctx.expect_contact(
        ota_tube,
        dec_head,
        elem_a="dovetail_bar",
        elem_b="saddle_bed",
        name="dovetail bar seats on saddle bed",
    )
    ctx.expect_overlap(
        ota_tube,
        dec_head,
        axes="x",
        elem_a="dovetail_bar",
        elem_b="saddle_bed",
        min_overlap=0.16,
        name="dovetail bar has meaningful clamp engagement",
    )

    focus_upper = focuser_travel.motion_limits.upper or 0.0
    rest_focuser_pos = ctx.part_world_position(focuser_drawtube)
    with ctx.pose({focuser_travel: focus_upper}):
        ctx.expect_overlap(
            focuser_drawtube,
            ota_tube,
            axes="z",
            elem_a="drawtube_shaft",
            elem_b="focuser_sleeve",
            min_overlap=0.040,
            name="drawtube retains insertion at full extension",
        )
        extended_focuser_pos = ctx.part_world_position(focuser_drawtube)
    ctx.check(
        "focuser drawtube extends outward",
        rest_focuser_pos is not None
        and extended_focuser_pos is not None
        and extended_focuser_pos[2] > rest_focuser_pos[2] + 0.020,
        details=f"rest={rest_focuser_pos}, extended={extended_focuser_pos}",
    )

    rest_ota_pos = ctx.part_world_position(ota_tube)
    with ctx.pose({declination_axis: 0.85}):
        raised_ota_pos = ctx.part_world_position(ota_tube)
    ctx.check(
        "declination axis raises the optical tube",
        rest_ota_pos is not None
        and raised_ota_pos is not None
        and raised_ota_pos[2] > rest_ota_pos[2] + 0.050,
        details=f"rest={rest_ota_pos}, raised={raised_ota_pos}",
    )

    with ctx.pose({right_ascension_axis: math.pi / 2.0}):
        ra_rotated_ota_pos = ctx.part_world_position(ota_tube)
    ctx.check(
        "right ascension axis rotates tube around polar axis",
        rest_ota_pos is not None
        and ra_rotated_ota_pos is not None
        and (
            abs(ra_rotated_ota_pos[0] - rest_ota_pos[0]) > 0.060
            or abs(ra_rotated_ota_pos[1] - rest_ota_pos[1]) > 0.060
        ),
        details=f"rest={rest_ota_pos}, rotated={ra_rotated_ota_pos}",
    )

    ctx.expect_origin_distance(
        ra_assembly,
        tripod_base,
        axes="z",
        min_dist=1.20,
        max_dist=1.60,
        name="equatorial head sits above tripod crown",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
