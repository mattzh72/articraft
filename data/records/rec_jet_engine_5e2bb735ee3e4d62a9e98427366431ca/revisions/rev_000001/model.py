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


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _foil_section(
    radius: float,
    center_y: float,
    center_z: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, center_y - 0.94 * half_thickness, center_z - 0.52 * chord),
        (radius, center_y + 0.18 * half_thickness, center_z - 0.10 * chord),
        (radius, center_y + 1.00 * half_thickness, center_z + 0.48 * chord),
        (radius, center_y - 0.22 * half_thickness, center_z + 0.16 * chord),
    ]


def _radial_pattern(
    base_geometry: MeshGeometry,
    count: int,
    *,
    angle_offset: float = 0.0,
) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _lofted_mesh(sections: list[list[tuple[float, float, float]]]) -> MeshGeometry:
    return repair_loft(section_loft(sections))


def _build_casing_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.300, 0.000),
            (0.334, 0.040),
            (0.317, 0.110),
            (0.310, 0.620),
            (0.300, 0.760),
            (0.255, 0.930),
            (0.185, 1.060),
        ],
        [
            (0.236, 0.000),
            (0.252, 0.040),
            (0.245, 0.110),
            (0.245, 0.640),
            (0.236, 0.770),
            (0.176, 0.935),
            (0.118, 1.060),
        ],
        segments=88,
    )


def _build_centerbody_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.030),
            (0.016, 0.044),
            (0.036, 0.066),
            (0.058, 0.092),
            (0.072, 0.116),
            (0.074, 0.145),
            (0.072, 0.172),
            (0.056, 0.190),
            (0.0, 0.204),
        ],
        segments=72,
    )


def _build_front_spider_mesh() -> MeshGeometry:
    collar = LatheGeometry.from_shell_profiles(
        [(0.084, 0.112), (0.084, 0.146)],
        [(0.062, 0.112), (0.062, 0.146)],
        segments=64,
    )
    strut = _lofted_mesh(
        [
            _foil_section(0.062, -0.006, 0.114, 0.050, 0.018),
            _foil_section(0.146, 0.000, 0.124, 0.060, 0.016),
            _foil_section(0.194, 0.004, 0.130, 0.050, 0.014),
        ]
    )
    shoe = BoxGeometry((0.018, 0.022, 0.020)).translate(0.203, 0.0, 0.130)
    spider = MeshGeometry()
    spider.merge(collar)
    for index in range(4):
        arm = MeshGeometry()
        arm.merge(strut.copy())
        arm.merge(shoe.copy())
        spider.merge(arm.rotate_z(index * math.pi / 2.0))
    return spider


def _build_rotor_hub_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.000),
            (0.094, 0.000),
            (0.102, 0.012),
            (0.102, 0.052),
            (0.090, 0.066),
            (0.0, 0.066),
        ],
        segments=72,
    )


def _build_rotor_blades_mesh() -> MeshGeometry:
    blade = _lofted_mesh(
        [
            _foil_section(0.094, -0.012, 0.030, 0.066, 0.016),
            _foil_section(0.146, 0.000, 0.050, 0.056, 0.013),
            _foil_section(0.194, 0.014, 0.074, 0.040, 0.008),
        ]
    )
    return _radial_pattern(blade, 10, angle_offset=math.pi / 10.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_turbojet_module")

    casing_metal = model.material("casing_metal", rgba=(0.76, 0.79, 0.82, 1.0))
    support_steel = model.material("support_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    rotor_titanium = model.material("rotor_titanium", rgba=(0.56, 0.60, 0.66, 1.0))

    casing = model.part("casing")
    casing.visual(
        _save_mesh("casing_shell", _build_casing_shell_mesh()),
        material=casing_metal,
        name="casing_shell",
    )
    for index in range(4):
        angle = index * math.pi / 2.0
        casing.visual(
            Box((0.036, 0.024, 0.040)),
            origin=Origin(
                xyz=(0.230 * math.cos(angle), 0.230 * math.sin(angle), 0.130),
                rpy=(0.0, 0.0, angle),
            ),
            material=support_steel,
            name=f"intake_mount_pad_{index}",
        )
    casing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.334, length=1.060),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.530)),
    )

    support = model.part("support_structure")
    support.visual(
        _save_mesh("centerbody_core_v2", _build_centerbody_mesh()),
        material=support_steel,
        name="centerbody",
    )
    support.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=support_steel,
        name="support_shaft",
    )
    support.visual(
        _save_mesh("front_spider_v4", _build_front_spider_mesh()),
        material=support_steel,
        name="front_struts",
    )
    support.inertial = Inertial.from_geometry(
        Cylinder(radius=0.245, length=0.220),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    rotor = model.part("compressor_rotor")
    rotor.visual(
        _save_mesh("rotor_hub_v3", _build_rotor_hub_mesh()),
        material=rotor_titanium,
        name="rotor_hub",
    )
    rotor.visual(
        _save_mesh("rotor_blades_v3", _build_rotor_blades_mesh()),
        material=rotor_titanium,
        name="rotor_blades",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.194, length=0.085),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
    )

    model.articulation(
        "casing_to_support",
        ArticulationType.FIXED,
        parent=casing,
        child=support,
        origin=Origin(),
    )
    model.articulation(
        "compressor_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=90.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    casing = object_model.get_part("casing")
    support = object_model.get_part("support_structure")
    rotor = object_model.get_part("compressor_rotor")
    compressor_spin = object_model.get_articulation("compressor_spin")

    casing_shell = casing.get_visual("casing_shell")
    intake_mount_pad_0 = casing.get_visual("intake_mount_pad_0")
    centerbody = support.get_visual("centerbody")
    support_shaft = support.get_visual("support_shaft")
    front_struts = support.get_visual("front_struts")
    rotor_hub = rotor.get_visual("rotor_hub")
    rotor_blades = rotor.get_visual("rotor_blades")

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

    ctx.fail_if_isolated_parts(max_pose_samples=8, name="sampled_pose_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=16, name="compressor_spin_clearance")

    ctx.expect_contact(support, casing, elem_a=front_struts, elem_b=intake_mount_pad_0)
    ctx.expect_contact(rotor, support, elem_a=rotor_hub, elem_b=support_shaft)
    ctx.expect_gap(
        rotor,
        support,
        axis="z",
        positive_elem=rotor_blades,
        negative_elem=front_struts,
        min_gap=0.050,
        max_gap=0.090,
        name="rotor_blades_aft_of_front_struts",
    )
    ctx.expect_overlap(
        rotor,
        casing,
        axes="xy",
        elem_a=rotor_blades,
        elem_b=casing_shell,
        min_overlap=0.35,
        name="rotor_visible_inside_intake",
    )
    ctx.expect_within(
        support,
        casing,
        axes="xy",
        inner_elem=front_struts,
        outer_elem=casing_shell,
        margin=0.0,
        name="front_struts_within_casing_envelope",
    )

    ctx.check(
        "compressor_spin_axis_is_centerline",
        tuple(compressor_spin.axis) == (0.0, 0.0, 1.0),
        details=f"Expected rotor axis (0, 0, 1), got {compressor_spin.axis!r}",
    )

    casing_aabb = ctx.part_world_aabb(casing)
    if casing_aabb is None:
        ctx.fail("casing_aabb_available", "Casing world AABB was not available.")
    else:
        length = casing_aabb[1][2] - casing_aabb[0][2]
        diameter_x = casing_aabb[1][0] - casing_aabb[0][0]
        diameter_y = casing_aabb[1][1] - casing_aabb[0][1]
        diameter = max(diameter_x, diameter_y)
        ctx.check(
            "engine_length_realistic",
            0.95 <= length <= 1.12,
            details=f"Expected compact turbojet length in [0.95, 1.12], got {length:.4f}",
        )
        ctx.check(
            "engine_diameter_realistic",
            0.58 <= diameter <= 0.70,
            details=f"Expected engine diameter in [0.58, 0.70], got {diameter:.4f}",
        )

    rotor_rest = ctx.part_world_position(rotor)
    with ctx.pose({compressor_spin: math.pi / 5.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="compressor_spin_pose_no_overlap")
        ctx.expect_contact(support, casing, elem_a=front_struts, elem_b=intake_mount_pad_0)
        ctx.expect_contact(rotor, support, elem_a=rotor_hub, elem_b=support_shaft)
        ctx.expect_gap(
            rotor,
            support,
            axis="z",
            positive_elem=rotor_blades,
            negative_elem=front_struts,
            min_gap=0.050,
            max_gap=0.090,
            name="rotor_spin_pose_strut_clearance",
        )
        rotor_spun = ctx.part_world_position(rotor)
        if rotor_rest is None or rotor_spun is None:
            ctx.fail("rotor_pose_position_available", "Rotor world position was unavailable.")
        else:
            ctx.check(
                "rotor_stays_on_centerline_when_spun",
                all(abs(rest - spun) <= 1e-6 for rest, spun in zip(rotor_rest, rotor_spun)),
                details=f"Rotor center shifted from {rotor_rest!r} to {rotor_spun!r}",
            )

    with ctx.pose({compressor_spin: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="compressor_half_turn_no_overlap")
        ctx.expect_contact(support, casing, elem_a=front_struts, elem_b=intake_mount_pad_0)
        ctx.expect_contact(rotor, support, elem_a=rotor_hub, elem_b=support_shaft)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
