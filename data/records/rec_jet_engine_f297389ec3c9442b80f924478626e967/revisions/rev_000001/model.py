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
)


def build_object_model() -> ArticulatedObject:
    def mesh(name: str, geometry: MeshGeometry):
        return mesh_from_geometry(geometry, name)

    def lathe_x(profile: list[tuple[float, float]], *, segments: int = 72) -> MeshGeometry:
        return LatheGeometry(profile, segments=segments).rotate_y(math.pi / 2.0)

    def lathe_shell_x(
        outer_profile: list[tuple[float, float]],
        inner_profile: list[tuple[float, float]],
        *,
        segments: int = 72,
        start_cap: str = "round",
        end_cap: str = "flat",
        lip_samples: int = 8,
    ) -> MeshGeometry:
        return (
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=segments,
                start_cap=start_cap,
                end_cap=end_cap,
                lip_samples=lip_samples,
            ).rotate_y(math.pi / 2.0)
        )

    def add_fan_blades(part, material) -> None:
        blade_count = 12
        radius = 0.192
        for index in range(blade_count):
            angle = 2.0 * math.pi * index / blade_count
            y_pos = radius * math.cos(angle)
            z_pos = radius * math.sin(angle)
            part.visual(
                Box((0.150, 0.160, 0.018)),
                origin=Origin(
                    xyz=(-0.118, y_pos, z_pos),
                    rpy=(angle, -0.22, 0.12),
                ),
                material=material,
                name=f"fan_blade_{index + 1}",
            )

    def add_bucket_visuals(part, side: str, material, hinge_material) -> None:
        mirror = 1.0 if side == "left" else -1.0
        part.visual(
            Cylinder(radius=0.007, length=0.368),
            origin=Origin(),
            material=hinge_material,
            name="hinge_rod",
        )
        part.visual(
            Box((0.340, 0.046, 0.334)),
            origin=Origin(xyz=(0.190, 0.056 * mirror, 0.0)),
            material=material,
            name="outer_skin",
        )
        part.visual(
            Box((0.280, 0.034, 0.052)),
            origin=Origin(xyz=(0.205, 0.056 * mirror, 0.142)),
            material=material,
            name="upper_lip",
        )
        part.visual(
            Box((0.280, 0.034, 0.052)),
            origin=Origin(xyz=(0.205, 0.056 * mirror, -0.142)),
            material=material,
            name="lower_lip",
        )
        part.visual(
            Box((0.110, 0.080, 0.320)),
            origin=Origin(xyz=(0.055, 0.040 * mirror, 0.0)),
            material=hinge_material,
            name="hinge_arm",
        )

    model = ArticulatedObject(name="business_jet_turbofan")

    nacelle_paint = model.material("nacelle_paint", rgba=(0.80, 0.82, 0.86, 1.0))
    intake_gray = model.material("intake_gray", rgba=(0.46, 0.49, 0.54, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.23, 0.24, 0.27, 1.0))
    compressor_metal = model.material("compressor_metal", rgba=(0.60, 0.63, 0.68, 1.0))
    hot_section = model.material("hot_section", rgba=(0.56, 0.45, 0.33, 1.0))

    nacelle = model.part("nacelle")
    nacelle_shell_mesh = lathe_shell_x(
        [
            (0.330, -0.090),
            (0.392, -0.030),
            (0.444, 0.160),
            (0.460, 0.420),
            (0.450, 0.900),
            (0.416, 1.180),
            (0.374, 1.400),
            (0.340, 1.620),
        ],
        [
            (0.312, -0.108),
            (0.350, -0.022),
            (0.392, 0.154),
            (0.406, 0.420),
            (0.396, 0.900),
            (0.366, 1.180),
            (0.352, 1.400),
            (0.322, 1.620),
        ],
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )
    nacelle.visual(mesh("nacelle_shell", nacelle_shell_mesh), material=nacelle_paint, name="nacelle_shell")
    nacelle.visual(
        Box((0.100, 0.190, 0.028)),
        origin=Origin(xyz=(0.390, 0.330, 0.0)),
        material=intake_gray,
        name="left_ogv_fairing",
    )
    nacelle.visual(
        Box((0.100, 0.190, 0.028)),
        origin=Origin(xyz=(0.390, -0.330, 0.0)),
        material=intake_gray,
        name="right_ogv_fairing",
    )
    nacelle.visual(
        Box((0.100, 0.028, 0.190)),
        origin=Origin(xyz=(0.390, 0.0, 0.330)),
        material=intake_gray,
        name="upper_ogv_fairing",
    )
    nacelle.visual(
        Box((0.100, 0.028, 0.190)),
        origin=Origin(xyz=(0.390, 0.0, -0.330)),
        material=intake_gray,
        name="lower_ogv_fairing",
    )
    nacelle.visual(
        Box((0.014, 0.022, 0.368)),
        origin=Origin(xyz=(1.426, 0.417, 0.0)),
        material=dark_metal,
        name="left_hinge_bracket",
    )
    nacelle.visual(
        Box((0.110, 0.070, 0.368)),
        origin=Origin(xyz=(1.378, 0.411, 0.0)),
        material=intake_gray,
        name="left_hinge_connector",
    )
    nacelle.visual(
        Box((0.014, 0.022, 0.368)),
        origin=Origin(xyz=(1.426, -0.417, 0.0)),
        material=dark_metal,
        name="right_hinge_bracket",
    )
    nacelle.visual(
        Box((0.110, 0.070, 0.368)),
        origin=Origin(xyz=(1.378, -0.411, 0.0)),
        material=intake_gray,
        name="right_hinge_connector",
    )
    nacelle.inertial = Inertial.from_geometry(
        Box((1.720, 1.020, 1.020)),
        mass=78.0,
        origin=Origin(xyz=(0.760, 0.0, 0.0)),
    )

    core = model.part("core")
    core.visual(
        mesh(
            "core_body",
            lathe_x(
                [
                    (0.070, 0.160),
                    (0.090, 0.230),
                    (0.120, 0.340),
                    (0.174, 0.340),
                    (0.188, 0.620),
                    (0.182, 0.920),
                    (0.152, 1.120),
                    (0.108, 1.290),
                    (0.066, 1.420),
                    (0.020, 1.540),
                    (0.000, 1.560),
                ],
                segments=64,
            ),
        ),
        material=dark_metal,
        name="core_body",
    )
    core.visual(
        Cylinder(radius=0.036, length=0.240),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_shaft",
    )
    core.visual(
        Cylinder(radius=0.220, length=0.100),
        origin=Origin(xyz=(0.330, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=compressor_metal,
        name="mount_ring",
    )
    core.visual(
        Cylinder(radius=0.115, length=0.055),
        origin=Origin(xyz=(0.250, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=compressor_metal,
        name="compressor_stage_1",
    )
    core.visual(
        Cylinder(radius=0.148, length=0.050),
        origin=Origin(xyz=(0.460, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=compressor_metal,
        name="compressor_stage_2",
    )
    core.visual(
        Cylinder(radius=0.166, length=0.050),
        origin=Origin(xyz=(0.720, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=compressor_metal,
        name="compressor_stage_3",
    )
    core.visual(
        Cylinder(radius=0.142, length=0.060),
        origin=Origin(xyz=(1.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hot_section,
        name="turbine_band",
    )
    core.visual(
        Box((0.100, 0.150, 0.024)),
        origin=Origin(xyz=(0.290, 0.160, 0.0)),
        material=dark_metal,
        name="left_support_strut",
    )
    core.visual(
        Box((0.100, 0.150, 0.024)),
        origin=Origin(xyz=(0.290, -0.160, 0.0)),
        material=dark_metal,
        name="right_support_strut",
    )
    core.visual(
        Box((0.100, 0.024, 0.150)),
        origin=Origin(xyz=(0.290, 0.0, 0.143)),
        material=dark_metal,
        name="upper_support_strut",
    )
    core.visual(
        Box((0.100, 0.024, 0.150)),
        origin=Origin(xyz=(0.290, 0.0, -0.143)),
        material=dark_metal,
        name="lower_support_strut",
    )
    core.inertial = Inertial.from_geometry(
        Box((1.560, 0.420, 0.420)),
        mass=28.0,
        origin=Origin(xyz=(0.780, 0.0, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        mesh(
            "fan_spinner",
            lathe_x(
                [
                    (0.000, -0.440),
                    (0.020, -0.405),
                    (0.072, -0.315),
                    (0.118, -0.205),
                    (0.148, -0.095),
                    (0.160, -0.018),
                    (0.082, 0.000),
                    (0.000, 0.000),
                ],
                segments=72,
            ),
        ),
        material=compressor_metal,
        name="spinner",
    )
    fan_rotor.visual(
        Cylinder(radius=0.118, length=0.104),
        origin=Origin(xyz=(-0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="fan_hub_shell",
    )
    fan_rotor.visual(
        Cylinder(radius=0.028, length=0.040),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="fan_axle_socket",
    )
    add_fan_blades(fan_rotor, compressor_metal)
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.360, length=0.440),
        mass=8.0,
        origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_bucket = model.part("left_bucket")
    add_bucket_visuals(left_bucket, "left", intake_gray, dark_metal)
    left_bucket.inertial = Inertial.from_geometry(
        Box((0.340, 0.120, 0.360)),
        mass=4.0,
        origin=Origin(xyz=(0.180, 0.055, 0.0)),
    )

    right_bucket = model.part("right_bucket")
    add_bucket_visuals(right_bucket, "right", intake_gray, dark_metal)
    right_bucket.inertial = Inertial.from_geometry(
        Box((0.340, 0.120, 0.360)),
        mass=4.0,
        origin=Origin(xyz=(0.180, -0.055, 0.0)),
    )

    model.articulation(
        "nacelle_to_core",
        ArticulationType.FIXED,
        parent=nacelle,
        child=core,
        origin=Origin(),
    )
    model.articulation(
        "core_to_fan",
        ArticulationType.CONTINUOUS,
        parent=core,
        child=fan_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=40.0),
    )
    model.articulation(
        "left_bucket_reverser",
        ArticulationType.REVOLUTE,
        parent=nacelle,
        child=left_bucket,
        origin=Origin(xyz=(1.440, 0.435, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.9,
            lower=0.0,
            upper=1.02,
        ),
    )
    model.articulation(
        "right_bucket_reverser",
        ArticulationType.REVOLUTE,
        parent=nacelle,
        child=right_bucket,
        origin=Origin(xyz=(1.440, -0.435, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.9,
            lower=0.0,
            upper=1.02,
        ),
    )

    return model


def run_tests() -> TestReport:
    def axis_close(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(actual, expected))

    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    core = object_model.get_part("core")
    fan_rotor = object_model.get_part("fan_rotor")
    left_bucket = object_model.get_part("left_bucket")
    right_bucket = object_model.get_part("right_bucket")
    left_hinge_arm = left_bucket.get_visual("hinge_arm")
    right_hinge_arm = right_bucket.get_visual("hinge_arm")
    left_hinge_connector = nacelle.get_visual("left_hinge_connector")
    right_hinge_connector = nacelle.get_visual("right_hinge_connector")

    fan_spin = object_model.get_articulation("core_to_fan")
    left_bucket_reverser = object_model.get_articulation("left_bucket_reverser")
    right_bucket_reverser = object_model.get_articulation("right_bucket_reverser")

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
    ctx.allow_overlap(
        left_bucket,
        nacelle,
        elem_a=left_hinge_arm,
        elem_b=left_hinge_connector,
        reason="Left reverser hinge arm nests into a hinge-pocket fairing during deployment.",
    )
    ctx.allow_overlap(
        right_bucket,
        nacelle,
        elem_a=right_hinge_arm,
        elem_b=right_hinge_connector,
        reason="Right reverser hinge arm nests into a hinge-pocket fairing during deployment.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "fan_spin_is_continuous_about_engine_axis",
        fan_spin.articulation_type == ArticulationType.CONTINUOUS
        and axis_close(fan_spin.axis, (1.0, 0.0, 0.0)),
        details=f"fan joint type={fan_spin.articulation_type}, axis={fan_spin.axis}",
    )
    ctx.check(
        "left_bucket_hinges_on_rear_side_axis",
        left_bucket_reverser.articulation_type == ArticulationType.REVOLUTE
        and axis_close(left_bucket_reverser.axis, (0.0, 0.0, 1.0)),
        details=f"left bucket axis={left_bucket_reverser.axis}",
    )
    ctx.check(
        "right_bucket_hinges_on_rear_side_axis",
        right_bucket_reverser.articulation_type == ArticulationType.REVOLUTE
        and axis_close(right_bucket_reverser.axis, (0.0, 0.0, -1.0)),
        details=f"right bucket axis={right_bucket_reverser.axis}",
    )

    ctx.expect_contact(core, nacelle, name="core_is_supported_inside_nacelle")
    ctx.expect_contact(
        fan_rotor,
        core,
        elem_a="fan_axle_socket",
        elem_b="front_shaft",
        name="fan_rotor_is_carried_by_core_front_hub",
    )
    ctx.expect_contact(
        left_bucket,
        nacelle,
        elem_b="left_hinge_bracket",
        contact_tol=0.0035,
        name="left_bucket_is_clipped_to_left_hinge_bracket",
    )
    ctx.expect_contact(
        right_bucket,
        nacelle,
        elem_b="right_hinge_bracket",
        contact_tol=0.0035,
        name="right_bucket_is_clipped_to_right_hinge_bracket",
    )
    ctx.expect_within(core, nacelle, axes="yz", margin=0.01, name="core_stays_on_engine_centerline")
    ctx.expect_within(fan_rotor, nacelle, axes="yz", margin=0.01, name="fan_stays_inside_inlet_diameter")

    left_rest_aabb = ctx.part_world_aabb(left_bucket)
    right_rest_aabb = ctx.part_world_aabb(right_bucket)
    assert left_rest_aabb is not None
    assert right_rest_aabb is not None

    with ctx.pose({left_bucket_reverser: 0.92, right_bucket_reverser: 0.92}):
        ctx.fail_if_parts_overlap_in_current_pose(name="deployed_reversers_clear_the_engine")
        ctx.expect_contact(
            left_bucket,
            nacelle,
            elem_a="hinge_rod",
            elem_b="left_hinge_bracket",
            contact_tol=0.0035,
            name="left_bucket_hinge_stays_clipped_when_deployed",
        )
        ctx.expect_contact(
            right_bucket,
            nacelle,
            elem_a="hinge_rod",
            elem_b="right_hinge_bracket",
            contact_tol=0.0035,
            name="right_bucket_hinge_stays_clipped_when_deployed",
        )

        left_open_aabb = ctx.part_world_aabb(left_bucket)
        right_open_aabb = ctx.part_world_aabb(right_bucket)
        assert left_open_aabb is not None
        assert right_open_aabb is not None

        ctx.check(
            "left_bucket_swings_outboard_when_deployed",
            left_open_aabb[1][1] > left_rest_aabb[1][1] + 0.08,
            details=f"rest max y={left_rest_aabb[1][1]:.3f}, open max y={left_open_aabb[1][1]:.3f}",
        )
        ctx.check(
            "right_bucket_swings_outboard_when_deployed",
            right_open_aabb[0][1] < right_rest_aabb[0][1] - 0.08,
            details=f"rest min y={right_rest_aabb[0][1]:.3f}, open min y={right_open_aabb[0][1]:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
