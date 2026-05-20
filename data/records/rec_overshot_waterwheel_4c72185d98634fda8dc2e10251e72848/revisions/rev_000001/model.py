from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _annular_mesh(
    name: str,
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
    ).rotate_x(pi / 2.0)
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel_with_drive_pulley")

    aged_oak = model.material("aged_oak", rgba=(0.50, 0.37, 0.23, 1.0))
    wet_oak = model.material("wet_oak", rgba=(0.40, 0.29, 0.19, 1.0))
    iron = model.material("iron", rgba=(0.35, 0.36, 0.38, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.19, 0.20, 0.22, 1.0))

    support_frame = model.part("support_frame")
    support_frame.inertial = Inertial.from_geometry(
        Box((1.46, 0.50, 2.42)),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
    )

    bent_y_positions = (-0.19, 0.19)
    for bent_y in bent_y_positions:
        for post_x in (-0.62, 0.62):
            support_frame.visual(
                Box((0.10, 0.08, 2.05)),
                origin=Origin(xyz=(post_x, bent_y, 1.025)),
                material=aged_oak,
            )
        support_frame.visual(
            Box((1.34, 0.08, 0.10)),
            origin=Origin(xyz=(0.0, bent_y, 0.10)),
            material=aged_oak,
        )
        support_frame.visual(
            Box((1.34, 0.08, 0.10)),
            origin=Origin(xyz=(0.0, bent_y, 2.10)),
            material=aged_oak,
        )
        for inner_x in (-0.18, 0.18):
            support_frame.visual(
                Box((0.08, 0.06, 1.19)),
                origin=Origin(xyz=(inner_x, bent_y, 0.595)),
                material=aged_oak,
            )
        support_frame.visual(
            Box((0.48, 0.08, 0.08)),
            origin=Origin(xyz=(0.0, bent_y, 1.00)),
            material=aged_oak,
        )
        support_frame.visual(
            Box((0.28, 0.08, 0.024)),
            origin=Origin(
                xyz=(0.0, bent_y, 1.193),
            ),
            material=wet_oak,
            name="bearing_pad_left" if bent_y < 0.0 else "bearing_pad_right",
        )
        for cheek_x in (-0.10, 0.10):
            support_frame.visual(
                Box((0.07, 0.08, 0.16)),
                origin=Origin(xyz=(cheek_x, bent_y, 1.115)),
                material=aged_oak,
            )

    for tie_x in (-0.62, 0.62):
        support_frame.visual(
            Box((0.08, 0.46, 0.08)),
            origin=Origin(xyz=(tie_x, 0.0, 0.10)),
            material=aged_oak,
        )

    support_frame.visual(
        Box((0.72, 0.36, 0.04)),
        origin=Origin(xyz=(-0.55, 0.0, 2.25)),
        material=wet_oak,
        name="flume_floor",
    )
    support_frame.visual(
        Box((0.72, 0.02, 0.12)),
        origin=Origin(xyz=(-0.55, -0.17, 2.31)),
        material=aged_oak,
    )
    support_frame.visual(
        Box((0.72, 0.02, 0.12)),
        origin=Origin(xyz=(-0.55, 0.17, 2.31)),
        material=aged_oak,
    )
    support_frame.visual(
        Box((0.08, 0.36, 0.20)),
        origin=Origin(xyz=(-0.90, 0.0, 2.25)),
        material=aged_oak,
    )
    for post_x in (-0.80, -0.30):
        for post_y in (-0.18, 0.18):
            support_frame.visual(
                Box((0.08, 0.04, 0.18)),
                origin=Origin(xyz=(post_x, post_y, 2.14)),
                material=aged_oak,
            )

    axle = model.part("axle")
    axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.86),
        mass=42.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    axle.visual(
        Cylinder(radius=0.045, length=0.86),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="shaft",
    )
    for collar_y in (-0.117, 0.117):
        axle.visual(
            Cylinder(radius=0.085, length=0.014),
            origin=Origin(xyz=(0.0, collar_y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=iron,
        )
    axle.visual(
        Cylinder(radius=0.070, length=0.014),
        origin=Origin(xyz=(0.0, 0.323, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pulley_inner_collar",
    )
    axle.visual(
        Cylinder(radius=0.075, length=0.014),
        origin=Origin(xyz=(0.0, 0.387, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="pulley_retainer",
    )
    axle.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(0.0, -0.34, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=iron,
    )

    waterwheel = model.part("waterwheel")
    waterwheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.98, length=0.20),
        mass=235.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rim_mesh = _annular_mesh(
        "waterwheel_side_rim",
        outer_profile=[
            (0.76, -0.020),
            (0.90, -0.020),
            (0.90, 0.020),
            (0.76, 0.020),
        ],
        inner_profile=[
            (0.78, -0.020),
            (0.78, 0.020),
        ],
    )
    waterwheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -0.09, 0.0)),
        material=wet_oak,
        name="left_rim",
    )
    waterwheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.09, 0.0)),
        material=wet_oak,
        name="right_rim",
    )

    hub_mesh = _annular_mesh(
        "waterwheel_hub_ring",
        outer_profile=[
            (0.17, -0.11),
            (0.20, -0.09),
            (0.20, 0.09),
            (0.17, 0.11),
        ],
        inner_profile=[
            (0.050, -0.11),
            (0.050, 0.11),
        ],
    )
    waterwheel.visual(hub_mesh, material=wet_oak, name="hub_ring")

    for spoke_index in range(8):
        angle = spoke_index * (pi / 4.0)
        waterwheel.visual(
            Box((0.88, 0.16, 0.060)),
            origin=Origin(
                xyz=(0.50 * cos(angle), 0.0, 0.50 * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=aged_oak,
            name=f"spoke_{spoke_index}",
        )

    bucket_count = 16
    bucket_phase = pi / bucket_count * 0.45
    for bucket_index in range(bucket_count):
        angle = bucket_index * (2.0 * pi / bucket_count)
        waterwheel.visual(
            Box((0.32, 0.19, 0.040)),
            origin=Origin(
                xyz=(0.75 * cos(angle), 0.0, 0.75 * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=wet_oak,
            name=f"bucket_divider_{bucket_index}",
        )

        floor_angle = angle + bucket_phase
        waterwheel.visual(
            Box((0.18, 0.18, 0.14)),
            origin=Origin(
                xyz=(0.86 * cos(floor_angle), 0.0, 0.86 * sin(floor_angle)),
                rpy=(0.0, floor_angle + pi / 2.0, 0.0),
            ),
            material=wet_oak,
            name=f"bucket_floor_{bucket_index}",
        )

    drive_pulley = model.part("drive_pulley")
    drive_pulley.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.05),
        mass=18.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    pulley_mesh = _annular_mesh(
        "drive_pulley_sheave",
        outer_profile=[
            (0.060, -0.025),
            (0.120, -0.025),
            (0.145, -0.016),
            (0.155, 0.000),
            (0.145, 0.016),
            (0.120, 0.025),
            (0.060, 0.025),
        ],
        inner_profile=[
            (0.050, -0.025),
            (0.050, 0.025),
        ],
    )
    drive_pulley.visual(pulley_mesh, material=iron, name="sheave")
    drive_pulley.visual(
        _annular_mesh(
            "drive_pulley_hub",
            outer_profile=[
                (0.080, -0.018),
                (0.090, -0.018),
                (0.090, 0.018),
                (0.080, 0.018),
            ],
            inner_profile=[
                (0.050, -0.018),
                (0.050, 0.018),
            ],
        ),
        material=dark_iron,
        name="hub_sleeve",
    )
    drive_pulley.visual(
        Box((0.035, 0.016, 0.028)),
        origin=Origin(xyz=(0.078, 0.0, 0.032)),
        material=dark_iron,
        name="pulley_boss",
    )

    model.articulation(
        "frame_to_axle",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=axle,
        origin=Origin(xyz=(0.0, 0.0, 1.25)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=4.0),
    )
    model.articulation(
        "axle_to_waterwheel",
        ArticulationType.FIXED,
        parent=axle,
        child=waterwheel,
        origin=Origin(),
    )
    model.articulation(
        "axle_to_drive_pulley",
        ArticulationType.FIXED,
        parent=axle,
        child=drive_pulley,
        origin=Origin(xyz=(0.0, 0.355, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    axle = object_model.get_part("axle")
    waterwheel = object_model.get_part("waterwheel")
    drive_pulley = object_model.get_part("drive_pulley")
    axle_spin = object_model.get_articulation("frame_to_axle")

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

    ctx.expect_contact(
        axle,
        support_frame,
        name="axle_seated_in_bearing_pads",
    )
    ctx.expect_contact(
        waterwheel,
        axle,
        name="wheel_clamped_to_axle",
    )
    ctx.expect_contact(
        drive_pulley,
        axle,
        name="pulley_clipped_to_axle",
    )
    ctx.expect_gap(
        drive_pulley,
        support_frame,
        axis="y",
        min_gap=0.08,
        name="pulley_sits_outboard_of_right_support",
    )
    ctx.expect_origin_distance(
        drive_pulley,
        waterwheel,
        axes="y",
        min_dist=0.33,
        max_dist=0.38,
        name="pulley_is_offset_from_wheel_face",
    )

    limits = axle_spin.motion_limits
    axis_ok = tuple(round(value, 6) for value in axle_spin.axis) == (0.0, 1.0, 0.0)
    type_ok = axle_spin.articulation_type == ArticulationType.CONTINUOUS
    limits_ok = limits is not None and limits.lower is None and limits.upper is None
    ctx.check(
        "main_axle_is_horizontal_continuous_spin",
        type_ok and axis_ok and limits_ok,
        details=(
            f"type={axle_spin.articulation_type}, axis={axle_spin.axis}, "
            f"limits=({None if limits is None else limits.lower}, "
            f"{None if limits is None else limits.upper})"
        ),
    )

    bucket_rest = ctx.part_element_world_aabb(waterwheel, elem="bucket_floor_0")
    boss_rest = ctx.part_element_world_aabb(drive_pulley, elem="pulley_boss")

    with ctx.pose({axle_spin: pi / 4.0}):
        ctx.expect_contact(
            waterwheel,
            axle,
            name="wheel_remains_seated_after_spin_pose",
        )
        ctx.expect_contact(
            drive_pulley,
            axle,
            name="pulley_remains_seated_after_spin_pose",
        )
        ctx.expect_gap(
            drive_pulley,
            support_frame,
            axis="y",
            min_gap=0.08,
            name="pulley_clear_of_support_after_spin_pose",
        )

        bucket_turned = ctx.part_element_world_aabb(waterwheel, elem="bucket_floor_0")
        boss_turned = ctx.part_element_world_aabb(drive_pulley, elem="pulley_boss")

    if bucket_rest is None or bucket_turned is None:
        ctx.fail(
            "named_bucket_floor_resolves",
            "Could not resolve bucket_floor_0 AABB for posed rotation check.",
        )
    else:
        bucket_rest_min_z = bucket_rest[0][2]
        bucket_turned_min_z = bucket_turned[0][2]
        ctx.check(
            "wheel_feature_moves_when_axle_spins",
            abs(bucket_turned_min_z - bucket_rest_min_z) > 0.20,
            details=(
                f"bucket floor min z changed from {bucket_rest_min_z:.4f} "
                f"to {bucket_turned_min_z:.4f}"
            ),
        )

    if boss_rest is None or boss_turned is None:
        ctx.fail(
            "named_pulley_boss_resolves",
            "Could not resolve pulley_boss AABB for shared-axle rotation check.",
        )
    else:
        boss_rest_min_z = boss_rest[0][2]
        boss_turned_min_z = boss_turned[0][2]
        ctx.check(
            "pulley_feature_moves_with_same_axle_spin",
            abs(boss_turned_min_z - boss_rest_min_z) > 0.03,
            details=(
                f"pulley boss min z changed from {boss_rest_min_z:.4f} "
                f"to {boss_turned_min_z:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
