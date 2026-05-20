from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

CEILING_Z = 0.38
CANOPY_HEIGHT = 0.050
DOWNROD_LENGTH = 0.120
JOINT_Z = CEILING_Z - CANOPY_HEIGHT - DOWNROD_LENGTH
BLADE_ANGLES = tuple(index * 2.0 * math.pi / 3.0 for index in range(3))


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_point(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (
        Origin(
            xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_segmented_hoop(
    part,
    *,
    radius: float,
    z: float,
    strand_radius: float,
    material,
    prefix: str,
    segments: int = 18,
) -> None:
    chord = 2.0 * radius * math.sin(math.pi / segments) * 1.02
    for index in range(segments):
        angle = index * math.tau / segments
        part.visual(
            Cylinder(radius=strand_radius, length=chord),
            origin=Origin(
                xyz=_circle_point(radius, angle, z),
                rpy=(0.0, math.pi / 2.0, angle + math.pi / 2.0),
            ),
            material=material,
            name=f"{prefix}_{index + 1}",
        )


def _add_polyline_strand(part, points, *, radius: float, material, prefix: str) -> None:
    for index, (start, end) in enumerate(zip(points, points[1:]), start=1):
        origin, length = _segment_origin(start, end)
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=origin,
            material=material,
            name=f"{prefix}_{index}",
        )


def _rotate_section_point(y: float, z: float, twist: float) -> tuple[float, float]:
    cos_t = math.cos(twist)
    sin_t = math.sin(twist)
    return (y * cos_t - z * sin_t, y * sin_t + z * cos_t)


def _blade_loop(
    x: float,
    width: float,
    thickness: float,
    droop: float,
    *,
    twist: float,
    camber: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    raw_loop = [
        (-half_w, 0.0),
        (-0.18 * width, thickness * 0.55 + camber),
        (0.18 * width, thickness * 0.48 + camber * 0.8),
        (half_w, 0.0),
        (0.14 * width, -thickness * 0.28),
        (-0.14 * width, -thickness * 0.32),
    ]
    loop: list[tuple[float, float, float]] = []
    for y, z in raw_loop:
        rot_y, rot_z = _rotate_section_point(y, z, twist)
        loop.append((x, rot_y, droop + rot_z))
    return loop


def _build_palm_blade_mesh():
    sections = [
        _blade_loop(0.040, 0.060, 0.012, -0.002, twist=0.06, camber=0.0020),
        _blade_loop(0.170, 0.158, 0.010, -0.012, twist=-0.02, camber=0.0032),
        _blade_loop(0.325, 0.178, 0.008, -0.024, twist=-0.10, camber=0.0030),
        _blade_loop(0.485, 0.148, 0.006, -0.040, twist=-0.18, camber=0.0020),
        _blade_loop(0.585, 0.050, 0.003, -0.056, twist=-0.28, camber=0.0008),
    ]
    return _save_mesh("palm_blade.obj", repair_loft(section_loft(sections)))


def _build_wicker_housing_mesh():
    outer_profile = [
        (0.028, -0.004),
        (0.044, -0.018),
        (0.086, -0.050),
        (0.101, -0.084),
        (0.088, -0.122),
        (0.060, -0.154),
        (0.040, -0.168),
    ]
    inner_profile = [
        (0.018, -0.004),
        (0.032, -0.020),
        (0.072, -0.052),
        (0.086, -0.084),
        (0.074, -0.120),
        (0.048, -0.150),
        (0.030, -0.162),
    ]
    return _save_mesh(
        "wicker_housing.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _build_globe_shell_mesh():
    outer_profile = [
        (0.021, -0.001),
        (0.034, -0.012),
        (0.074, -0.050),
        (0.088, -0.090),
        (0.078, -0.126),
        (0.050, -0.157),
        (0.010, -0.176),
    ]
    inner_profile = [
        (0.015, -0.001),
        (0.027, -0.012),
        (0.067, -0.050),
        (0.081, -0.090),
        (0.071, -0.124),
        (0.044, -0.154),
        (0.000, -0.170),
    ]
    return _save_mesh(
        "ribbed_globe_shell.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tropical_ceiling_fan", assets=ASSETS)

    bronze = model.material("bronze", rgba=(0.34, 0.26, 0.18, 1.0))
    dark_motor = model.material("dark_motor", rgba=(0.18, 0.15, 0.12, 1.0))
    wicker_shell = model.material("wicker_shell", rgba=(0.59, 0.42, 0.23, 1.0))
    wicker_weave = model.material("wicker_weave", rgba=(0.75, 0.61, 0.35, 1.0))
    palm_leaf = model.material("palm_leaf", rgba=(0.80, 0.70, 0.47, 1.0))
    palm_vein = model.material("palm_vein", rgba=(0.63, 0.49, 0.28, 1.0))
    glass = model.material("glass", rgba=(0.95, 0.97, 0.99, 0.30))
    glass_rib = model.material("glass_rib", rgba=(0.98, 0.99, 1.0, 0.48))

    blade_mesh = _build_palm_blade_mesh()
    housing_mesh = _build_wicker_housing_mesh()
    globe_shell_mesh = _build_globe_shell_mesh()

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.078, length=CANOPY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CEILING_Z - CANOPY_HEIGHT * 0.5)),
        material=bronze,
        name="canopy",
    )
    mount.visual(
        Cylinder(radius=0.011, length=DOWNROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z + DOWNROD_LENGTH * 0.5)),
        material=bronze,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.019, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z + 0.011)),
        material=bronze,
        name="downrod_collar",
    )
    mount.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, CANOPY_HEIGHT + DOWNROD_LENGTH)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z + (CANOPY_HEIGHT + DOWNROD_LENGTH) * 0.5)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=bronze,
        name="top_cap",
    )
    rotor.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=bronze,
        name="upper_hub",
    )
    rotor.visual(
        Cylinder(radius=0.050, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, -0.091)),
        material=dark_motor,
        name="motor_core",
    )
    rotor.visual(housing_mesh, material=wicker_shell, name="housing_shell")
    rotor.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=bronze,
        name="upper_trim_ring",
    )
    rotor.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.158)),
        material=bronze,
        name="lower_trim_ring",
    )
    rotor.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.172)),
        material=bronze,
        name="globe_fitter",
    )

    for index, (radius, z) in enumerate(
        (
            (0.037, -0.014),
            (0.053, -0.032),
            (0.083, -0.058),
            (0.098, -0.084),
            (0.087, -0.112),
            (0.061, -0.140),
            (0.043, -0.159),
        ),
        start=1,
    ):
        _add_segmented_hoop(
            rotor,
            radius=radius,
            z=z,
            strand_radius=0.0023,
            material=wicker_weave,
            prefix=f"hoop_{index}",
        )

    strand_radii = (0.036, 0.052, 0.074, 0.092, 0.095, 0.081, 0.058, 0.042)
    strand_zs = (-0.010, -0.028, -0.048, -0.070, -0.094, -0.118, -0.144, -0.160)
    for strand_index, base_angle in enumerate([i * math.pi / 4.0 for i in range(8)], start=1):
        forward_points = []
        reverse_points = []
        for point_index, (radius, z) in enumerate(zip(strand_radii, strand_zs)):
            fraction = point_index / (len(strand_zs) - 1)
            forward_points.append(_circle_point(radius, base_angle + 0.95 * fraction, z))
            reverse_points.append(_circle_point(radius, base_angle - 0.95 * fraction, z))
        _add_polyline_strand(
            rotor,
            forward_points,
            radius=0.0019,
            material=wicker_weave,
            prefix=f"forward_strand_{strand_index}",
        )
        _add_polyline_strand(
            rotor,
            reverse_points,
            radius=0.0019,
            material=wicker_weave,
            prefix=f"reverse_strand_{strand_index}",
        )

    for index, angle in enumerate(BLADE_ANGLES, start=1):
        rotor.visual(
            Box((0.074, 0.018, 0.012)),
            origin=Origin(
                xyz=(0.066 * math.cos(angle), 0.066 * math.sin(angle), -0.060),
                rpy=(0.0, 0.0, angle),
            ),
            material=bronze,
            name=f"blade_arm_{index}",
        )
        rotor.visual(
            Box((0.032, 0.050, 0.012)),
            origin=Origin(
                xyz=(0.104 * math.cos(angle), 0.104 * math.sin(angle), -0.060),
                rpy=(0.0, 0.0, angle),
            ),
            material=bronze,
            name=f"blade_seat_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.18),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    for index in range(1, 4):
        blade = model.part(f"blade_{index}")
        blade.visual(
            Box((0.044, 0.055, 0.008)),
            origin=Origin(xyz=(0.022, 0.0, -0.010)),
            material=bronze,
            name="root_block",
        )
        blade.visual(
            Box((0.076, 0.032, 0.008)),
            origin=Origin(xyz=(0.054, 0.0, -0.018), rpy=(0.0, -0.08, 0.0)),
            material=bronze,
            name="blade_iron",
        )
        blade.visual(
            blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=palm_leaf,
            name="leaf_panel",
        )
        blade.visual(
            Cylinder(radius=0.004, length=0.540),
            origin=Origin(xyz=(0.312, 0.0, -0.033), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=palm_vein,
            name="blade_midrib",
        )
        blade.inertial = Inertial.from_geometry(
            Box((0.59, 0.18, 0.07)),
            mass=0.55,
            origin=Origin(xyz=(0.305, 0.0, -0.034)),
        )

    globe = model.part("globe")
    globe.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=glass,
        name="globe_neck",
    )
    globe.visual(
        globe_shell_mesh,
        material=glass,
        name="globe_shell",
    )
    globe.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.176)),
        material=glass_rib,
        name="globe_finial_stem",
    )
    globe.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.0, 0.0, -0.186)),
        material=glass_rib,
        name="globe_finial",
    )
    rib_profile = (
        (0.021, -0.002),
        (0.031, -0.012),
        (0.050, -0.034),
        (0.072, -0.068),
        (0.082, -0.104),
        (0.069, -0.136),
        (0.043, -0.162),
        (0.015, -0.179),
    )
    for rib_index, angle in enumerate([i * math.tau / 12.0 for i in range(12)], start=1):
        rib_points = [_circle_point(radius, angle, z) for radius, z in rib_profile]
        _add_polyline_strand(
            globe,
            rib_points,
            radius=0.0018,
            material=glass_rib,
            prefix=f"globe_rib_{rib_index}",
        )
    globe.inertial = Inertial.from_geometry(
        Cylinder(radius=0.089, length=0.186),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, -0.093)),
    )

    model.articulation(
        "mount_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )

    for index, angle in enumerate(BLADE_ANGLES, start=1):
        model.articulation(
            f"rotor_to_blade_{index}",
            ArticulationType.FIXED,
            parent=rotor,
            child=f"blade_{index}",
            origin=Origin(
                xyz=(0.120 * math.cos(angle), 0.120 * math.sin(angle), -0.060),
                rpy=(0.0, 0.0, angle),
            ),
        )

    model.articulation(
        "rotor_to_globe",
        ArticulationType.FIXED,
        parent=rotor,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)

    mount = object_model.get_part("mount")
    rotor = object_model.get_part("rotor")
    globe = object_model.get_part("globe")
    blade_1 = object_model.get_part("blade_1")
    blade_2 = object_model.get_part("blade_2")
    blade_3 = object_model.get_part("blade_3")
    spin = object_model.get_articulation("mount_to_rotor")

    downrod = mount.get_visual("downrod")
    top_cap = rotor.get_visual("top_cap")
    fitter = rotor.get_visual("globe_fitter")
    housing_shell = rotor.get_visual("housing_shell")
    seat_1 = rotor.get_visual("blade_seat_1")
    seat_2 = rotor.get_visual("blade_seat_2")
    seat_3 = rotor.get_visual("blade_seat_3")
    globe_neck = globe.get_visual("globe_neck")
    globe_shell = globe.get_visual("globe_shell")
    blade_1_root = blade_1.get_visual("root_block")
    blade_2_root = blade_2.get_visual("root_block")
    blade_3_root = blade_3.get_visual("root_block")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(rotor, mount, axes="xy", max_dist=0.001, name="rotor_centered_on_downrod")
    ctx.expect_origin_distance(globe, rotor, axes="xy", max_dist=0.001, name="globe_centered_under_rotor")
    ctx.expect_contact(rotor, mount, elem_a=top_cap, elem_b=downrod, name="rotor_contacts_downrod")
    ctx.expect_within(
        globe,
        rotor,
        axes="xy",
        inner_elem=globe_neck,
        outer_elem=fitter,
        name="globe_neck_within_fitter",
    )
    ctx.expect_gap(
        rotor,
        globe,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=fitter,
        negative_elem=globe_neck,
        name="globe_hangs_from_fitter",
    )
    ctx.expect_contact(globe, rotor, elem_a=globe_neck, elem_b=fitter, name="globe_contacts_fitter")
    ctx.expect_contact(blade_1, rotor, elem_a=blade_1_root, elem_b=seat_1, name="blade_1_root_contact")
    ctx.expect_contact(blade_2, rotor, elem_a=blade_2_root, elem_b=seat_2, name="blade_2_root_contact")
    ctx.expect_contact(blade_3, rotor, elem_a=blade_3_root, elem_b=seat_3, name="blade_3_root_contact")

    blade_positions = [ctx.part_world_position(blade) for blade in (blade_1, blade_2, blade_3)]
    if all(position is not None for position in blade_positions):
        positions = blade_positions  # type: ignore[assignment]
        angles = sorted(math.atan2(position[1], position[0]) % (2.0 * math.pi) for position in positions)
        angle_gaps = [((angles[(index + 1) % 3] - angles[index]) % (2.0 * math.pi)) for index in range(3)]
        ctx.check(
            "blade_spacing_even",
            all(abs(gap - (2.0 * math.pi / 3.0)) < 0.12 for gap in angle_gaps),
            details=f"blade angle gaps={angle_gaps!r}",
        )
        root_radii = [math.hypot(position[0], position[1]) for position in positions]
        ctx.check(
            "blade_roots_at_realistic_radius",
            all(0.115 <= radius <= 0.125 for radius in root_radii),
            details=f"blade root radii={root_radii!r}",
        )

    blade_aabbs = [ctx.part_world_aabb(blade) for blade in (blade_1, blade_2, blade_3)]
    if all(aabb is not None for aabb in blade_aabbs):
        aabbs = blade_aabbs  # type: ignore[assignment]
        fan_span = max(
            max(aabb[1][0] for aabb in aabbs) - min(aabb[0][0] for aabb in aabbs),
            max(aabb[1][1] for aabb in aabbs) - min(aabb[0][1] for aabb in aabbs),
        )
        ctx.check(
            "fan_span_realistic",
            1.28 <= fan_span <= 1.45,
            details=f"overall span={fan_span:.3f} m",
        )

    housing_aabb = ctx.part_element_world_aabb(rotor, elem=housing_shell)
    if housing_aabb is not None:
        housing_diameter = max(
            housing_aabb[1][0] - housing_aabb[0][0],
            housing_aabb[1][1] - housing_aabb[0][1],
        )
        ctx.check(
            "wicker_housing_diameter_realistic",
            0.18 <= housing_diameter <= 0.22,
            details=f"housing diameter={housing_diameter:.3f} m",
        )

    globe_aabb = ctx.part_element_world_aabb(globe, elem=globe_shell)
    if globe_aabb is not None:
        globe_height = globe_aabb[1][2] - globe_aabb[0][2]
        globe_diameter = max(
            globe_aabb[1][0] - globe_aabb[0][0],
            globe_aabb[1][1] - globe_aabb[0][1],
        )
        ctx.check(
            "globe_proportions_realistic",
            0.16 <= globe_height <= 0.19 and 0.16 <= globe_diameter <= 0.19,
            details=f"globe height={globe_height:.3f} m diameter={globe_diameter:.3f} m",
        )

    blade_1_rest = ctx.part_world_position(blade_1)
    blade_1_spun = None
    with ctx.pose({spin: math.pi / 3.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="spin_pose_no_floating")
        ctx.expect_contact(rotor, mount, elem_a=top_cap, elem_b=downrod, name="spin_pose_rotor_contact")
        ctx.expect_contact(globe, rotor, elem_a=globe_neck, elem_b=fitter, name="spin_pose_globe_contact")
        ctx.expect_contact(blade_1, rotor, elem_a=blade_1_root, elem_b=seat_1, name="spin_pose_blade_1_contact")
        blade_1_spun = ctx.part_world_position(blade_1)

    if blade_1_rest is not None and blade_1_spun is not None:
        rest_radius = math.hypot(blade_1_rest[0], blade_1_rest[1])
        spun_radius = math.hypot(blade_1_spun[0], blade_1_spun[1])
        moved = math.hypot(blade_1_rest[0] - blade_1_spun[0], blade_1_rest[1] - blade_1_spun[1]) > 0.08
        ctx.check(
            "continuous_spin_moves_blade",
            moved and abs(rest_radius - spun_radius) < 0.002,
            details=(
                f"rest={blade_1_rest!r} spun={blade_1_spun!r} "
                f"rest_radius={rest_radius:.4f} spun_radius={spun_radius:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
