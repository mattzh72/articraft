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
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern(base_geometry: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _blade_section(
    radius: float,
    center_y: float,
    center_z: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, center_y - 0.95 * half_thickness, center_z - 0.52 * chord),
        (radius, center_y + 0.18 * half_thickness, center_z - 0.10 * chord),
        (radius, center_y + 1.00 * half_thickness, center_z + 0.48 * chord),
        (radius, center_y - 0.30 * half_thickness, center_z + 0.12 * chord),
    ]


def _soft_rect_loop(width: float, depth: float, z: float, corner: float) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    half_depth = depth * 0.5
    return [
        (-half_width + corner, -half_depth, z),
        (half_width - corner, -half_depth, z),
        (half_width, -half_depth + corner, z),
        (half_width, half_depth - corner, z),
        (half_width - corner, half_depth, z),
        (-half_width + corner, half_depth, z),
        (-half_width, half_depth - corner, z),
        (-half_width, -half_depth + corner, z),
    ]


def _nacelle_shell_geometry() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.58, -0.88),
            (0.66, -0.78),
            (0.67, -0.18),
            (0.66, 0.26),
            (0.63, 0.70),
            (0.57, 0.88),
        ],
        [
            (0.54, -0.92),
            (0.58, -0.80),
            (0.57, -0.16),
            (0.53, 0.28),
            (0.47, 0.70),
            (0.42, 0.88),
        ],
        segments=84,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )


def _spinner_geometry() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -0.30),
            (0.03, -0.27),
            (0.08, -0.21),
            (0.13, -0.12),
            (0.17, -0.02),
            (0.18, 0.05),
            (0.0, 0.05),
        ],
        segments=64,
    )


def _fan_blade_geometry() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _blade_section(0.16, -0.020, -0.04, 0.20, 0.034),
                _blade_section(0.33, 0.008, 0.02, 0.15, 0.022),
                _blade_section(0.50, 0.030, 0.09, 0.08, 0.010),
            ]
        )
    )
    return _radial_pattern(blade, 14, angle_offset=math.pi / 14.0)


def _exhaust_cone_geometry() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -0.02),
            (0.08, -0.02),
            (0.13, 0.06),
            (0.16, 0.18),
            (0.10, 0.32),
            (0.03, 0.42),
            (0.0, 0.46),
        ],
        segments=64,
    )


def _bearing_ring_geometry() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.090, -0.03),
            (0.090, 0.03),
        ],
        [
            (0.055, -0.03),
            (0.055, 0.03),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _support_fairing_geometry() -> MeshGeometry:
    return section_loft(
        [
            _soft_rect_loop(0.20, 0.16, 0.06, 0.035),
            _soft_rect_loop(0.22, 0.18, 0.18, 0.045),
            _soft_rect_loop(0.26, 0.20, 0.28, 0.050),
        ]
    )


def _base_cover_geometry() -> MeshGeometry:
    return section_loft(
        [
            _soft_rect_loop(1.30, 0.78, 0.00, 0.08),
            _soft_rect_loop(1.24, 0.72, 0.03, 0.08),
            _soft_rect_loop(1.18, 0.66, 0.06, 0.08),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turbofan_display_nacelle")

    nacelle_white = model.material("nacelle_white", rgba=(0.82, 0.84, 0.87, 1.0))
    inlet_gray = model.material("inlet_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.72, 0.75, 0.79, 1.0))
    hot_metal = model.material("hot_metal", rgba=(0.48, 0.42, 0.36, 1.0))
    stand_dark = model.material("stand_dark", rgba=(0.16, 0.18, 0.20, 1.0))

    base = model.part("stand_base")
    base.visual(
        Box((1.30, 0.78, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=stand_dark,
        name="base_plinth",
    )
    base.visual(
        mesh_from_geometry(_base_cover_geometry(), "base_cover"),
        material=stand_dark,
        name="base_cover",
    )
    base.inertial = Inertial.from_geometry(
        Box((1.30, 0.78, 0.06)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    support = model.part("display_support")
    support.visual(
        Box((0.16, 0.28, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=stand_dark,
        name="center_column",
    )
    support.visual(
        mesh_from_geometry(_support_fairing_geometry(), "support_fairing_lofted"),
        material=stand_dark,
        name="support_fairing",
    )
    support.visual(
        Box((0.30, 0.26, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        material=stand_dark,
        name="saddle_pad",
    )
    support.visual(
        Box((0.24, 0.05, 0.18)),
        origin=Origin(xyz=(0.0, 0.115, 0.18), rpy=(0.0, -0.62, 0.0)),
        material=stand_dark,
        name="left_gusset",
    )
    support.visual(
        Box((0.24, 0.05, 0.18)),
        origin=Origin(xyz=(0.0, -0.115, 0.18), rpy=(0.0, -0.62, 0.0)),
        material=stand_dark,
        name="right_gusset",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.30, 0.28, 0.31)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(_nacelle_shell_geometry(), "nacelle_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nacelle_white,
        name="nacelle_shell",
    )
    nacelle.visual(
        Box((0.24, 0.18, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.675)),
        material=inlet_gray,
        name="mount_pad",
    )
    nacelle.visual(
        mesh_from_geometry(_bearing_ring_geometry(), "bearing_ring"),
        origin=Origin(xyz=(-0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inlet_gray,
        name="bearing_ring",
    )
    nacelle.visual(
        Box((0.12, 0.04, 0.50)),
        origin=Origin(xyz=(-0.14, 0.0, 0.34)),
        material=inlet_gray,
        name="upper_guide_vane",
    )
    nacelle.visual(
        Box((0.12, 0.04, 0.50)),
        origin=Origin(xyz=(-0.14, 0.0, -0.34)),
        material=inlet_gray,
        name="lower_guide_vane",
    )
    nacelle.visual(
        Box((0.12, 0.50, 0.04)),
        origin=Origin(xyz=(-0.14, 0.34, 0.0)),
        material=inlet_gray,
        name="left_guide_vane",
    )
    nacelle.visual(
        Box((0.12, 0.50, 0.04)),
        origin=Origin(xyz=(-0.14, -0.34, 0.0)),
        material=inlet_gray,
        name="right_guide_vane",
    )
    nacelle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.66, length=1.76),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    exhaust = model.part("rear_exhaust")
    exhaust.visual(
        Cylinder(radius=0.06, length=0.10),
        origin=Origin(xyz=(0.27, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inlet_gray,
        name="front_collar",
    )
    exhaust.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.40, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inlet_gray,
        name="tail_fairing",
    )
    exhaust.visual(
        mesh_from_geometry(_exhaust_cone_geometry(), "exhaust_cone"),
        origin=Origin(xyz=(0.42, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hot_metal,
        name="exhaust_cone_body",
    )
    exhaust.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=0.78),
        mass=7.0,
        origin=Origin(xyz=(0.50, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rotor = model.part("fan_rotor")
    rotor.visual(
        mesh_from_geometry(_spinner_geometry(), "spinner"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_metal,
        name="spinner",
    )
    rotor.visual(
        Cylinder(radius=0.15, length=0.18),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inlet_gray,
        name="fan_hub",
    )
    rotor.visual(
        mesh_from_geometry(_fan_blade_geometry(), "fan_blades"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_metal,
        name="fan_blades",
    )
    rotor.visual(
        Cylinder(radius=0.045, length=0.62),
        origin=Origin(xyz=(0.31, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inlet_gray,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.085, length=0.06),
        origin=Origin(xyz=(0.32, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=inlet_gray,
        name="bearing_collar",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.50, length=0.95),
        mass=8.5,
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_support",
        ArticulationType.FIXED,
        parent=base,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )
    model.articulation(
        "support_to_nacelle",
        ArticulationType.FIXED,
        parent=support,
        child=nacelle,
        origin=Origin(xyz=(0.0, 0.0, 1.01)),
    )
    model.articulation(
        "nacelle_to_exhaust",
        ArticulationType.FIXED,
        parent=nacelle,
        child=exhaust,
        origin=Origin(),
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(-0.40, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("stand_base")
    support = object_model.get_part("display_support")
    nacelle = object_model.get_part("nacelle")
    exhaust = object_model.get_part("rear_exhaust")
    rotor = object_model.get_part("fan_rotor")

    base_plinth = base.get_visual("base_plinth")
    center_column = support.get_visual("center_column")
    saddle_pad = support.get_visual("saddle_pad")
    nacelle_shell = nacelle.get_visual("nacelle_shell")
    bearing_ring = nacelle.get_visual("bearing_ring")
    mount_pad = nacelle.get_visual("mount_pad")
    front_collar = exhaust.get_visual("front_collar")
    exhaust_cone_body = exhaust.get_visual("exhaust_cone_body")
    shaft = rotor.get_visual("shaft")
    bearing_collar = rotor.get_visual("bearing_collar")
    spinner = rotor.get_visual("spinner")
    fan_blades = rotor.get_visual("fan_blades")

    fan_spin = object_model.get_articulation("fan_spin")

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

    ctx.expect_contact(support, base, elem_a=center_column, elem_b=base_plinth, name="support_grounded_on_base")
    ctx.expect_contact(nacelle, support, elem_a=mount_pad, elem_b=saddle_pad, name="nacelle_mounted_on_support")
    ctx.expect_contact(rotor, nacelle, elem_a=bearing_collar, elem_b=bearing_ring, name="rotor_supported_by_nacelle_bearing")
    ctx.expect_contact(rotor, exhaust, elem_a=shaft, elem_b=front_collar, name="rotor_supported_by_center_bearing")
    ctx.expect_contact(exhaust, rotor, elem_a=front_collar, elem_b=shaft, name="rear_exhaust_supported_by_rotor")

    ctx.expect_within(rotor, nacelle, axes="yz", margin=0.01, name="rotor_within_nacelle_radius")
    ctx.expect_within(exhaust, nacelle, axes="yz", margin=0.01, name="exhaust_within_nacelle_radius")
    ctx.expect_overlap(rotor, nacelle, axes="yz", min_overlap=0.20, name="fan_stage_centered_in_intake")
    ctx.expect_overlap(exhaust, nacelle, axes="yz", min_overlap=0.10, name="exhaust_centered_in_nozzle")
    ctx.expect_gap(exhaust, rotor, axis="x", min_gap=-0.0005, max_gap=0.0005, positive_elem=front_collar, negative_elem=shaft, name="bearing_face_contact_gap")
    spinner_aabb = ctx.part_element_world_aabb(rotor, elem=spinner)
    nacelle_shell_aabb = ctx.part_element_world_aabb(nacelle, elem=nacelle_shell)
    assert spinner_aabb is not None
    assert nacelle_shell_aabb is not None
    spinner_setback = spinner_aabb[0][0] - nacelle_shell_aabb[0][0]
    ctx.check(
        "spinner_set_back_from_intake_lip",
        0.10 <= spinner_setback <= 0.30,
        details=f"spinner_setback={spinner_setback:.4f}",
    )

    rotor_axis_ok = all(abs(a - b) < 1e-9 for a, b in zip(fan_spin.axis, (1.0, 0.0, 0.0)))
    ctx.check("fan_spin_axis_is_longitudinal", rotor_axis_ok, details=f"axis={fan_spin.axis}")
    limits = fan_spin.motion_limits
    continuous_ok = (
        fan_spin.joint_type == ArticulationType.CONTINUOUS
        and limits is not None
        and limits.lower is None
        and limits.upper is None
    )
    ctx.check("fan_spin_is_continuous", continuous_ok, details="fan rotor should spin continuously about the nacelle centerline")

    nacelle_aabb = ctx.part_world_aabb(nacelle)
    assert nacelle_aabb is not None
    nacelle_length = nacelle_aabb[1][0] - nacelle_aabb[0][0]
    nacelle_diameter = nacelle_aabb[1][2] - nacelle_aabb[0][2]
    ctx.check(
        "nacelle_proportions_realistic",
        1.6 <= nacelle_length <= 2.0 and 1.2 <= nacelle_diameter <= 1.5,
        details=f"length={nacelle_length:.3f}, diameter={nacelle_diameter:.3f}",
    )

    for angle in (0.0, math.pi / 6.0, 2.0 * math.pi / 3.0):
        with ctx.pose({fan_spin: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"fan_spin_{angle:.2f}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"fan_spin_{angle:.2f}_no_floating")
            ctx.expect_contact(
                rotor,
                nacelle,
                elem_a=bearing_collar,
                elem_b=bearing_ring,
                name=f"fan_spin_{angle:.2f}_nacelle_bearing_contact",
            )
            ctx.expect_contact(
                rotor,
                exhaust,
                elem_a=shaft,
                elem_b=front_collar,
                name=f"fan_spin_{angle:.2f}_bearing_contact",
            )
            ctx.expect_within(
                rotor,
                nacelle,
                axes="yz",
                margin=0.01,
                name=f"fan_spin_{angle:.2f}_stays_within_nacelle",
            )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="fan_spin_sampled_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
