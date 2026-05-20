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


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern(
    base_geometry: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_z(angle_offset + index * math.tau / count))
    return patterned


def _blade_section(
    radius: float,
    tangential_offset: float,
    z_pos: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = 0.5 * thickness
    return [
        (radius, tangential_offset - 0.95 * half_thickness, z_pos - 0.55 * chord),
        (radius, tangential_offset + 0.18 * half_thickness, z_pos - 0.12 * chord),
        (radius, tangential_offset + 1.00 * half_thickness, z_pos + 0.44 * chord),
        (radius, tangential_offset - 0.24 * half_thickness, z_pos + 0.14 * chord),
    ]


def _build_front_cowl_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (1.18, -1.30),
            (1.28, -1.18),
            (1.31, -0.92),
            (1.30, -0.50),
            (1.29, 0.00),
            (1.27, 0.32),
            (1.22, 0.58),
        ],
        [
            (1.08, -1.34),
            (1.15, -1.20),
            (1.17, -0.92),
            (1.16, -0.50),
            (1.12, 0.00),
            (1.08, 0.32),
            (1.00, 0.58),
        ],
        segments=96,
        start_cap="round",
        end_cap="flat",
        lip_samples=12,
    )


def _build_mid_case_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (1.00, 0.58),
            (1.00, 1.20),
            (0.98, 1.70),
            (0.94, 2.46),
        ],
        [
            (0.86, 0.58),
            (0.86, 1.20),
            (0.86, 1.70),
            (0.82, 2.46),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def _build_core_mount_ring() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(0.98, 1.58), (0.98, 1.64)],
        [(0.86, 1.58), (0.86, 1.64)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_aft_nozzle_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (1.08, 2.46),
            (1.10, 2.70),
            (0.98, 3.00),
            (0.82, 3.28),
        ],
        [
            (0.82, 2.46),
            (0.84, 2.70),
            (0.78, 3.00),
            (0.70, 3.28),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def _build_core_plug() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.24),
            (0.12, 0.24),
            (0.20, 0.42),
            (0.38, 1.10),
            (0.42, 1.70),
            (0.34, 2.40),
            (0.22, 2.98),
            (0.10, 3.18),
            (0.0, 3.28),
        ],
        segments=80,
    )


def _build_mount_flange() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(0.52, 1.64), (0.52, 1.70)],
        [(0.34, 1.64), (0.34, 1.70)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_front_bearing_ring() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(0.32, -0.24), (0.32, -0.12)],
        [(0.24, -0.24), (0.24, -0.12)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_core_support_ring() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(0.56, 1.63), (0.56, 1.71)],
        [(0.38, 1.63), (0.38, 1.71)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_spinner() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, -1.18),
            (0.08, -1.12),
            (0.18, -0.98),
            (0.24, -0.78),
            (0.26, -0.52),
            (0.24, -0.24),
            (0.0, -0.24),
        ],
        segments=72,
    )


def _build_fan_blades() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _blade_section(0.32, -0.07, -0.16, 0.82, 0.12),
                _blade_section(0.52, -0.02, -0.06, 0.74, 0.10),
                _blade_section(0.74, 0.04, 0.08, 0.58, 0.08),
                _blade_section(0.96, 0.10, 0.22, 0.36, 0.05),
            ]
        ),
        repair="mesh",
    )
    return _radial_pattern(blade, 18, angle_offset=math.pi / 18.0)


def _part_origin_at_radius(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="airliner_turbofan_nacelle")

    nacelle_white = model.material("nacelle_white", rgba=(0.81, 0.83, 0.86, 1.0))
    titanium = model.material("titanium", rgba=(0.63, 0.66, 0.71, 1.0))
    dark_composite = model.material("dark_composite", rgba=(0.18, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.44, 0.47, 0.52, 1.0))

    nacelle_body = model.part("nacelle_body")
    nacelle_body.visual(
        _mesh(_build_front_cowl_shell(), "front_cowl_shell"),
        material=nacelle_white,
        name="front_cowl_shell",
    )
    nacelle_body.visual(
        _mesh(_build_mid_case_shell(), "mid_case_shell"),
        material=dark_composite,
        name="mid_case_shell",
    )
    nacelle_body.visual(
        _mesh(_build_core_mount_ring(), "core_mount_ring_v3"),
        material=steel,
        name="core_mount_ring",
    )
    nacelle_body.visual(
        _mesh(_build_aft_nozzle_shell(), "aft_nozzle_shell"),
        material=titanium,
        name="aft_nozzle_shell",
    )

    rail_z_center = 1.58
    rail_length = 2.20
    rail_radial_center = 1.00
    rail_radial_thickness = 0.06
    rail_tangential_width = 0.06
    rail_specs = (
        ("guide_rail_pos_x", Origin(xyz=(rail_radial_center, 0.0, rail_z_center))),
        ("guide_rail_neg_x", Origin(xyz=(-rail_radial_center, 0.0, rail_z_center))),
        (
            "guide_rail_pos_y",
            Origin(xyz=(0.0, rail_radial_center, rail_z_center), rpy=(0.0, 0.0, math.pi / 2.0)),
        ),
        (
            "guide_rail_neg_y",
            Origin(xyz=(0.0, -rail_radial_center, rail_z_center), rpy=(0.0, 0.0, math.pi / 2.0)),
        ),
    )
    for name, origin in rail_specs:
        nacelle_body.visual(
            Box((rail_radial_thickness, rail_tangential_width, rail_length)),
            origin=origin,
            material=steel,
            name=name,
        )

    cascade_radius = 1.03
    for index in range(8):
        angle = math.pi / 8.0 + index * math.pi / 4.0
        nacelle_body.visual(
            Box((0.08, 0.032, 0.26)),
            origin=Origin(
                xyz=_part_origin_at_radius(cascade_radius, angle, 1.04),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"cascade_vane_{index}",
        )

    for index in range(6):
        angle = index * math.tau / 6.0
        nacelle_body.visual(
            Box((0.08, 0.10, 0.08)),
            origin=Origin(
                xyz=_part_origin_at_radius(0.82, angle, 1.67),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"inner_mount_pad_{index}",
        )

    nacelle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=1.32, length=4.58),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
    )

    core = model.part("core")
    core.visual(
        _mesh(_build_core_plug(), "core_plug_v4"),
        material=dark_composite,
        name="core_plug",
    )
    core.visual(
        _mesh(_build_core_support_ring(), "core_support_ring_v1"),
        material=steel,
        name="core_support_ring",
    )
    core.visual(
        Cylinder(radius=0.20, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=steel,
        name="front_bearing",
    )
    for index in range(6):
        angle = index * math.tau / 6.0
        core.visual(
            Cylinder(radius=0.03, length=0.36),
            origin=Origin(
                xyz=_part_origin_at_radius(0.60, angle, 1.67),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=steel,
            name=f"core_frame_strut_{index}",
        )
    core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.86, length=3.70),
        mass=620.0,
        origin=Origin(xyz=(0.0, 0.0, 1.43)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        _mesh(_build_spinner(), "fan_spinner"),
        material=titanium,
        name="spinner",
    )
    fan_rotor.visual(
        Cylinder(radius=0.20, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
        material=dark_composite,
        name="hub_shaft",
    )
    fan_rotor.visual(
        Cylinder(radius=0.36, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_composite,
        name="hub_disk",
    )
    fan_rotor.visual(
        _mesh(_build_fan_blades(), "fan_blades"),
        material=titanium,
        name="fan_blades",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.00, length=1.00),
        mass=340.0,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    reverser_sleeve = model.part("reverser_sleeve")
    reverser_sleeve.visual(
        _mesh(
            LatheGeometry.from_shell_profiles(
                [
                    (1.25, 0.58),
                    (1.26, 1.15),
                    (1.24, 1.70),
                    (1.20, 2.18),
                ],
                [
                    (1.12, 0.58),
                    (1.12, 1.15),
                    (1.12, 1.70),
                    (1.12, 2.18),
                ],
                segments=88,
                start_cap="flat",
                end_cap="flat",
            ),
            "reverser_sleeve_shell",
        ),
        material=nacelle_white,
        name="sleeve_shell",
    )

    pad_z_center = 1.38
    pad_length = 1.45
    pad_radial_thickness = 0.09
    pad_tangential_width = 0.06
    pad_radial_center = 1.075
    pad_specs = (
        ("carriage_pad_pos_x", Origin(xyz=(pad_radial_center, 0.0, pad_z_center))),
        ("carriage_pad_neg_x", Origin(xyz=(-pad_radial_center, 0.0, pad_z_center))),
        (
            "carriage_pad_pos_y",
            Origin(xyz=(0.0, pad_radial_center, pad_z_center), rpy=(0.0, 0.0, math.pi / 2.0)),
        ),
        (
            "carriage_pad_neg_y",
            Origin(xyz=(0.0, -pad_radial_center, pad_z_center), rpy=(0.0, 0.0, math.pi / 2.0)),
        ),
    )
    for name, origin in pad_specs:
        reverser_sleeve.visual(
            Box((pad_radial_thickness, pad_tangential_width, pad_length)),
            origin=origin,
            material=steel,
            name=name,
        )

    reverser_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=1.26, length=1.60),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.38)),
    )

    model.articulation(
        "nacelle_to_core",
        ArticulationType.FIXED,
        parent=nacelle_body,
        child=core,
        origin=Origin(),
    )
    model.articulation(
        "core_to_fan_rotor",
        ArticulationType.CONTINUOUS,
        parent=core,
        child=fan_rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=80.0),
    )
    model.articulation(
        "nacelle_to_reverser_sleeve",
        ArticulationType.PRISMATIC,
        parent=nacelle_body,
        child=reverser_sleeve,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15000.0,
            velocity=0.60,
            lower=0.0,
            upper=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    nacelle_body = object_model.get_part("nacelle_body")
    core = object_model.get_part("core")
    fan_rotor = object_model.get_part("fan_rotor")
    reverser_sleeve = object_model.get_part("reverser_sleeve")

    fan_spin = object_model.get_articulation("core_to_fan_rotor")
    sleeve_slide = object_model.get_articulation("nacelle_to_reverser_sleeve")

    fan_axis = tuple(float(value) for value in fan_spin.axis)
    sleeve_axis = tuple(float(value) for value in sleeve_slide.axis)
    sleeve_limits = sleeve_slide.motion_limits

    ctx.check(
        "fan_rotor_joint_is_continuous_longitudinal",
        fan_spin.joint_type == ArticulationType.CONTINUOUS and fan_axis == (0.0, 0.0, 1.0),
        details=f"type={fan_spin.joint_type}, axis={fan_axis}",
    )
    ctx.check(
        "reverser_joint_is_prismatic_longitudinal",
        sleeve_slide.joint_type == ArticulationType.PRISMATIC and sleeve_axis == (0.0, 0.0, 1.0),
        details=f"type={sleeve_slide.joint_type}, axis={sleeve_axis}",
    )
    ctx.check(
        "reverser_has_realistic_translation_limits",
        sleeve_limits is not None
        and sleeve_limits.lower == 0.0
        and sleeve_limits.upper is not None
        and 0.20 <= sleeve_limits.upper <= 0.35,
        details=f"limits={sleeve_limits}",
    )

    ctx.expect_contact(core, nacelle_body, elem_a="core_frame_strut_0", elem_b="inner_mount_pad_0")
    ctx.expect_contact(fan_rotor, core, elem_a="hub_shaft", elem_b="front_bearing")
    ctx.expect_contact(
        reverser_sleeve,
        nacelle_body,
        elem_a="carriage_pad_pos_x",
        elem_b="guide_rail_pos_x",
    )
    ctx.expect_contact(
        reverser_sleeve,
        nacelle_body,
        elem_a="carriage_pad_pos_y",
        elem_b="guide_rail_pos_y",
    )
    ctx.expect_overlap(reverser_sleeve, nacelle_body, axes="xy", min_overlap=2.0)
    ctx.expect_overlap(fan_rotor, nacelle_body, axes="xy", min_overlap=1.8)

    sleeve_rest_aabb = ctx.part_world_aabb(reverser_sleeve)
    if sleeve_rest_aabb is None:
        ctx.fail("reverser_rest_aabb_available", "Could not resolve reverser sleeve AABB in rest pose.")
    else:
        with ctx.pose({sleeve_slide: 0.25}):
            sleeve_open_aabb = ctx.part_world_aabb(reverser_sleeve)
            if sleeve_open_aabb is None:
                ctx.fail("reverser_open_aabb_available", "Could not resolve reverser sleeve AABB in open pose.")
            else:
                ctx.check(
                    "reverser_translates_aft_on_engine_axis",
                    sleeve_open_aabb[0][2] > sleeve_rest_aabb[0][2] + 0.20,
                    details=f"rest_min_z={sleeve_rest_aabb[0][2]:.3f}, open_min_z={sleeve_open_aabb[0][2]:.3f}",
                )
            ctx.expect_contact(
                reverser_sleeve,
                nacelle_body,
                elem_a="carriage_pad_pos_x",
                elem_b="guide_rail_pos_x",
                name="reverser_pad_remains_on_rail_when_open",
            )
            ctx.fail_if_isolated_parts(name="no_isolated_parts_with_reverser_open")
            ctx.fail_if_parts_overlap_in_current_pose(name="no_part_overlaps_with_reverser_open")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
