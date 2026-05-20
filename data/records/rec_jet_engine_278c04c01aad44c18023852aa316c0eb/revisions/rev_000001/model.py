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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern(base: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _ring_band(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    segments: int = 56,
) -> MeshGeometry:
    half_length = 0.5 * length
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_length), (outer_radius, half_length)],
        [(inner_radius, -half_length), (inner_radius, half_length)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _build_outer_case_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.204, -0.390),
            (0.220, -0.350),
            (0.218, -0.250),
            (0.214, -0.040),
            (0.206, 0.135),
            (0.188, 0.265),
            (0.156, 0.342),
            (0.124, 0.395),
        ],
        [
            (0.182, -0.392),
            (0.184, -0.344),
            (0.182, -0.248),
            (0.176, -0.040),
            (0.166, 0.132),
            (0.146, 0.262),
            (0.114, 0.338),
            (0.096, 0.402),
        ],
        segments=80,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )


def _build_rotor_blades() -> MeshGeometry:
    blade_profile = [
        (0.051, -0.011),
        (0.078, -0.018),
        (0.154, -0.010),
        (0.150, 0.004),
        (0.090, 0.018),
        (0.051, 0.010),
    ]
    blade = ExtrudeGeometry.centered(blade_profile, 0.004, cap=True)
    blade.rotate_y(math.radians(24.0))
    blade.translate(0.0, 0.0, 0.001)
    return _radial_pattern(blade, 12, angle_offset=math.pi / 12.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turbojet_module_on_stand")

    stand_gray = model.material("stand_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.10, 0.11, 0.12, 1.0))
    case_metal = model.material("case_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.28, 1.0))
    compressor_metal = model.material("compressor_metal", rgba=(0.63, 0.66, 0.70, 1.0))

    stand = model.part("stand")
    stand.visual(
        Box((0.82, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.20, 0.02)),
        material=stand_gray,
        name="left_base_rail",
    )
    stand.visual(
        Box((0.82, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.20, 0.02)),
        material=stand_gray,
        name="right_base_rail",
    )
    for x_pos, name in (
        (-0.35, "front_frame_crossbar"),
        (-0.18, "forward_support_crossbar"),
        (0.18, "aft_support_crossbar"),
        (0.35, "rear_frame_crossbar"),
    ):
        stand.visual(
            Box((0.06, 0.45, 0.04)),
            origin=Origin(xyz=(x_pos, 0.0, 0.02)),
            material=stand_gray,
            name=name,
        )
    for x_pos, name in ((-0.18, "forward_pedestal"), (0.18, "aft_pedestal")):
        stand.visual(
            Box((0.08, 0.14, 0.055)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0675)),
            material=stand_gray,
            name=name,
        )
    stand.visual(
        Box((0.16, 0.16, 0.02)),
        origin=Origin(xyz=(-0.18, 0.0, 0.105)),
        material=saddle_black,
        name="forward_saddle",
    )
    stand.visual(
        Box((0.16, 0.16, 0.02)),
        origin=Origin(xyz=(0.18, 0.0, 0.105)),
        material=saddle_black,
        name="aft_saddle",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.82, 0.45, 0.125)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
    )

    engine_case = model.part("engine_case")
    engine_case.visual(
        mesh_from_geometry(_build_outer_case_shell(), "outer_case_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=case_metal,
        name="outer_case_shell",
    )
    engine_case.visual(
        Cylinder(radius=0.215, length=0.018),
        origin=Origin(xyz=(-0.05, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_case_band",
    )
    engine_case.visual(
        Cylinder(radius=0.160, length=0.014),
        origin=Origin(xyz=(0.285, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="nozzle_band",
    )
    engine_case.visual(
        Cylinder(radius=0.015, length=0.120),
        origin=Origin(xyz=(-0.255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="center_shaft",
    )
    engine_case.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(-0.309, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_capture_collar",
    )
    engine_case.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(-0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_capture_collar",
    )
    engine_case.visual(
        mesh_from_geometry(
            _ring_band(inner_radius=0.156, outer_radius=0.186, length=0.014),
            "shaft_support_ring",
        ),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft_support_ring",
    )
    engine_case.visual(
        Box((0.016, 0.148, 0.020)),
        origin=Origin(xyz=(-0.205, 0.086, 0.0)),
        material=dark_steel,
        name="support_vane_pos_y",
    )
    engine_case.visual(
        Box((0.016, 0.148, 0.020)),
        origin=Origin(xyz=(-0.205, -0.086, 0.0)),
        material=dark_steel,
        name="support_vane_neg_y",
    )
    engine_case.visual(
        Box((0.016, 0.020, 0.148)),
        origin=Origin(xyz=(-0.205, 0.0, 0.086)),
        material=dark_steel,
        name="support_vane_pos_z",
    )
    engine_case.visual(
        Box((0.016, 0.020, 0.148)),
        origin=Origin(xyz=(-0.205, 0.0, -0.086)),
        material=dark_steel,
        name="support_vane_neg_z",
    )
    engine_case.visual(
        Box((0.03, 0.08, 0.08)),
        origin=Origin(xyz=(-0.18, 0.0, -0.245)),
        material=dark_steel,
        name="forward_mount_web",
    )
    engine_case.visual(
        Box((0.12, 0.10, 0.02)),
        origin=Origin(xyz=(-0.18, 0.0, -0.285)),
        material=dark_steel,
        name="forward_mount_shoe",
    )
    engine_case.visual(
        Box((0.03, 0.08, 0.087)),
        origin=Origin(xyz=(0.18, 0.0, -0.2415)),
        material=dark_steel,
        name="aft_mount_web",
    )
    engine_case.visual(
        Box((0.12, 0.10, 0.02)),
        origin=Origin(xyz=(0.18, 0.0, -0.285)),
        material=dark_steel,
        name="aft_mount_shoe",
    )
    engine_case.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.79),
        mass=34.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    compressor_rotor = model.part("compressor_rotor")
    compressor_rotor.visual(
        mesh_from_geometry(
            _ring_band(inner_radius=0.036, outer_radius=0.055, length=0.032),
            "compressor_hub_ring",
        ),
        origin=Origin(xyz=(-0.268, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_ring",
    )
    compressor_rotor.visual(
        Box((0.036, 0.042, 0.004)),
        origin=Origin(xyz=(-0.268, 0.036, 0.0)),
        material=dark_steel,
        name="hub_spoke_pos_y",
    )
    compressor_rotor.visual(
        Box((0.036, 0.042, 0.004)),
        origin=Origin(xyz=(-0.268, -0.036, 0.0)),
        material=dark_steel,
        name="hub_spoke_neg_y",
    )
    compressor_rotor.visual(
        Box((0.036, 0.004, 0.042)),
        origin=Origin(xyz=(-0.268, 0.0, 0.036)),
        material=dark_steel,
        name="hub_spoke_pos_z",
    )
    compressor_rotor.visual(
        Box((0.036, 0.004, 0.042)),
        origin=Origin(xyz=(-0.268, 0.0, -0.036)),
        material=dark_steel,
        name="hub_spoke_neg_z",
    )
    compressor_rotor.visual(
        Cylinder(radius=0.003, length=0.036),
        origin=Origin(xyz=(-0.268, 0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_clip_pos_y",
    )
    compressor_rotor.visual(
        Cylinder(radius=0.003, length=0.036),
        origin=Origin(xyz=(-0.268, -0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_clip_neg_y",
    )
    compressor_rotor.visual(
        Cylinder(radius=0.003, length=0.036),
        origin=Origin(xyz=(-0.268, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_clip_pos_z",
    )
    compressor_rotor.visual(
        Cylinder(radius=0.003, length=0.036),
        origin=Origin(xyz=(-0.268, 0.0, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hub_clip_neg_z",
    )
    compressor_rotor.visual(
        mesh_from_geometry(_build_rotor_blades(), "compressor_blades"),
        origin=Origin(xyz=(-0.268, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=compressor_metal,
        name="blade_ring",
    )
    compressor_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.04),
        mass=1.2,
        origin=Origin(xyz=(-0.268, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "stand_to_engine",
        ArticulationType.FIXED,
        parent=stand,
        child=engine_case,
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
    )
    model.articulation(
        "compressor_spin",
        ArticulationType.CONTINUOUS,
        parent=engine_case,
        child=compressor_rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    stand = object_model.get_part("stand")
    engine_case = object_model.get_part("engine_case")
    compressor_rotor = object_model.get_part("compressor_rotor")
    compressor_spin = object_model.get_articulation("compressor_spin")

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

    ctx.expect_contact(
        engine_case,
        stand,
        elem_a="forward_mount_shoe",
        elem_b="forward_saddle",
        name="forward_shoe_seats_on_forward_saddle",
    )
    ctx.expect_contact(
        engine_case,
        stand,
        elem_a="aft_mount_shoe",
        elem_b="aft_saddle",
        name="aft_shoe_seats_on_aft_saddle",
    )
    ctx.expect_overlap(
        engine_case,
        stand,
        axes="xy",
        min_overlap=0.09,
        elem_a="forward_mount_shoe",
        elem_b="forward_saddle",
        name="forward_shoe_centered_on_saddle",
    )
    ctx.expect_overlap(
        engine_case,
        stand,
        axes="xy",
        min_overlap=0.09,
        elem_a="aft_mount_shoe",
        elem_b="aft_saddle",
        name="aft_shoe_centered_on_saddle",
    )
    ctx.expect_contact(
        compressor_rotor,
        engine_case,
        elem_a="hub_clip_pos_y",
        elem_b="center_shaft",
        name="rotor_hub_captured_on_center_shaft",
    )
    ctx.expect_gap(
        compressor_rotor,
        engine_case,
        axis="x",
        positive_elem="hub_ring",
        negative_elem="front_capture_collar",
        min_gap=0.012,
        max_gap=0.028,
        name="front_capture_collar_keeps_rotor_from_walking_forward",
    )
    ctx.expect_gap(
        engine_case,
        compressor_rotor,
        axis="x",
        positive_elem="rear_capture_collar",
        negative_elem="hub_ring",
        min_gap=0.004,
        max_gap=0.018,
        name="rear_capture_collar_keeps_rotor_from_walking_aft",
    )
    ctx.expect_origin_distance(
        compressor_rotor,
        engine_case,
        axes="yz",
        max_dist=0.001,
        name="compressor_rotor_stays_on_engine_centerline",
    )
    ctx.expect_within(
        compressor_rotor,
        engine_case,
        axes="yz",
        inner_elem="blade_ring",
        outer_elem="outer_case_shell",
        margin=0.0,
        name="compressor_face_stays_inside_intake_diameter",
    )
    ctx.check(
        "compressor_spin_axis_is_main_centerline",
        tuple(compressor_spin.axis) == (1.0, 0.0, 0.0),
        details=f"Expected (1, 0, 0), got {compressor_spin.axis}",
    )

    with ctx.pose({compressor_spin: 1.7}):
        ctx.expect_contact(
            compressor_rotor,
            engine_case,
            elem_a="hub_clip_pos_y",
            elem_b="center_shaft",
            name="rotor_remains_captured_while_spinning",
        )
        ctx.expect_within(
            compressor_rotor,
            engine_case,
            axes="yz",
            inner_elem="blade_ring",
            outer_elem="outer_case_shell",
            margin=0.0,
            name="blade_ring_clearance_persists_in_spun_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
