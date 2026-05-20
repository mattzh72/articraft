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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_remote_weapon_station")

    def annular_ring(name: str, *, inner_radius: float, outer_radius: float, height: float):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(outer_radius, 0.0), (outer_radius, height)],
                [(inner_radius, 0.0), (inner_radius, height)],
                segments=56,
                start_cap="flat",
                end_cap="flat",
            ),
            name,
        )

    armor_dark = model.material("armor_dark", rgba=(0.27, 0.30, 0.28, 1.0))
    armor_mid = model.material("armor_mid", rgba=(0.34, 0.37, 0.35, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.10, 0.14, 0.16, 1.0))

    lower_ring_mesh = annular_ring(
        "lower_slew_ring_mesh_v2",
        inner_radius=0.15,
        outer_radius=0.25,
        height=0.04,
    )
    upper_ring_mesh = annular_ring(
        "upper_slew_ring_mesh_v2",
        inner_radius=0.16,
        outer_radius=0.26,
        height=0.05,
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.88, 0.88, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=armor_mid,
        name="base_plinth",
    )
    pedestal.visual(
        Box((0.56, 0.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=armor_dark,
        name="base_shoulder",
    )
    pedestal.visual(
        Cylinder(radius=0.17, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        material=armor_dark,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=metal_dark,
        name="bearing_spindle_lower",
    )
    pedestal.visual(
        lower_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=metal_dark,
        name="lower_slew_ring",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.88, 0.88, 1.15)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
    )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        upper_ring_mesh,
        material=metal_dark,
        name="upper_slew_ring",
    )
    azimuth_stage.visual(
        Box((0.56, 0.56, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=armor_dark,
        name="turret_deck",
    )
    azimuth_stage.visual(
        Box((0.32, 0.10, 0.58)),
        origin=Origin(xyz=(0.02, -0.24, 0.34)),
        material=armor_mid,
        name="left_cheek",
    )
    azimuth_stage.visual(
        Box((0.32, 0.10, 0.58)),
        origin=Origin(xyz=(0.02, 0.24, 0.34)),
        material=armor_mid,
        name="right_cheek",
    )
    azimuth_stage.visual(
        Box((0.16, 0.38, 0.10)),
        origin=Origin(xyz=(-0.04, 0.0, 0.16)),
        material=armor_dark,
        name="pivot_bridge",
    )
    azimuth_stage.visual(
        Box((0.26, 0.02, 0.34)),
        origin=Origin(xyz=(0.04, -0.30, 0.32)),
        material=armor_mid,
        name="left_shield_plate",
    )
    azimuth_stage.visual(
        Box((0.26, 0.02, 0.34)),
        origin=Origin(xyz=(0.04, 0.30, 0.32)),
        material=armor_mid,
        name="right_shield_plate",
    )
    azimuth_stage.inertial = Inertial.from_geometry(
        Box((0.60, 0.64, 0.68)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Cylinder(radius=0.05, length=0.22),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_dark,
        name="pivot_tube",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.0, -0.15, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_dark,
        name="left_trunnion",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.0, 0.15, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_dark,
        name="right_trunnion",
    )
    weapon_cradle.visual(
        Box((0.34, 0.22, 0.18)),
        origin=Origin(xyz=(0.03, 0.0, 0.00)),
        material=armor_mid,
        name="cradle_block",
    )
    weapon_cradle.visual(
        Box((0.56, 0.18, 0.18)),
        origin=Origin(xyz=(0.42, 0.0, 0.0)),
        material=armor_dark,
        name="receiver_body",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.07, length=0.28),
        origin=Origin(xyz=(0.84, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="barrel_shroud",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.045, length=0.88),
        origin=Origin(xyz=(1.42, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="barrel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.06, length=0.16),
        origin=Origin(xyz=(1.94, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_dark,
        name="muzzle_device",
    )
    weapon_cradle.visual(
        Box((0.28, 0.14, 0.18)),
        origin=Origin(xyz=(0.30, 0.0, -0.10)),
        material=armor_mid,
        name="ammo_box",
    )
    weapon_cradle.visual(
        Box((0.22, 0.14, 0.16)),
        origin=Origin(xyz=(0.26, 0.0, 0.17)),
        material=armor_mid,
        name="sensor_body",
    )
    weapon_cradle.visual(
        Box((0.16, 0.12, 0.10)),
        origin=Origin(xyz=(0.26, 0.0, 0.30)),
        material=armor_dark,
        name="sensor_head",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.38, 0.0, 0.17), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor_glass,
        name="sensor_lens",
    )
    weapon_cradle.visual(
        Box((0.16, 0.16, 0.16)),
        origin=Origin(xyz=(-0.10, 0.0, 0.09)),
        material=armor_mid,
        name="rear_counterweight",
    )
    weapon_cradle.inertial = Inertial.from_geometry(
        Box((2.25, 0.60, 0.74)),
        mass=310.0,
        origin=Origin(xyz=(0.92, 0.0, 0.10)),
    )

    model.articulation(
        "pedestal_to_azimuth",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28000.0, velocity=0.8),
    )
    model.articulation(
        "azimuth_to_cradle",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=weapon_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=0.9,
            lower=math.radians(-12.0),
            upper=math.radians(60.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    azimuth_stage = object_model.get_part("azimuth_stage")
    weapon_cradle = object_model.get_part("weapon_cradle")

    azimuth = object_model.get_articulation("pedestal_to_azimuth")
    elevation = object_model.get_articulation("azimuth_to_cradle")
    upper_ring = azimuth_stage.get_visual("upper_slew_ring")
    lower_ring = pedestal.get_visual("lower_slew_ring")
    left_cheek = azimuth_stage.get_visual("left_cheek")
    right_cheek = azimuth_stage.get_visual("right_cheek")
    left_shield_plate = azimuth_stage.get_visual("left_shield_plate")
    left_trunnion = weapon_cradle.get_visual("left_trunnion")
    right_trunnion = weapon_cradle.get_visual("right_trunnion")
    barrel = weapon_cradle.get_visual("barrel")
    receiver_body = weapon_cradle.get_visual("receiver_body")
    sensor_head = weapon_cradle.get_visual("sensor_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        weapon_cradle,
        azimuth_stage,
        elem_a=left_trunnion,
        elem_b=left_cheek,
        reason="Left trunnion journal intentionally sits inside the cheek bearing pocket.",
    )
    ctx.allow_overlap(
        weapon_cradle,
        azimuth_stage,
        elem_a=right_trunnion,
        elem_b=right_cheek,
        reason="Right trunnion journal intentionally sits inside the cheek bearing pocket.",
    )

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

    ctx.check(
        "azimuth_axis_vertical",
        azimuth.axis == (0.0, 0.0, 1.0),
        f"Expected vertical slew axis, got {azimuth.axis}.",
    )
    ctx.check(
        "elevation_axis_transverse",
        abs(elevation.axis[0]) < 1e-9
        and abs(abs(elevation.axis[1]) - 1.0) < 1e-9
        and abs(elevation.axis[2]) < 1e-9,
        f"Expected transverse elevation axis, got {elevation.axis}.",
    )

    elevation_limits = elevation.motion_limits
    ctx.check(
        "elevation_motion_window_realistic",
        elevation_limits is not None
        and elevation_limits.lower is not None
        and elevation_limits.upper is not None
        and elevation_limits.lower <= math.radians(-10.0)
        and elevation_limits.upper >= math.radians(55.0),
        "Cradle should depress slightly and elevate strongly like a heavy weapon station.",
    )

    ctx.expect_gap(
        azimuth_stage,
        pedestal,
        axis="z",
        positive_elem=upper_ring,
        negative_elem=lower_ring,
        max_gap=0.0,
        max_penetration=0.0,
        name="slew_ring_faces_seated",
    )
    ctx.expect_contact(
        azimuth_stage,
        pedestal,
        elem_a=upper_ring,
        elem_b=lower_ring,
        name="slew_ring_in_contact",
    )
    ctx.expect_overlap(
        azimuth_stage,
        pedestal,
        axes="xy",
        elem_a=upper_ring,
        elem_b=lower_ring,
        min_overlap=0.40,
        name="slew_ring_plan_overlap",
    )
    ctx.expect_contact(
        weapon_cradle,
        azimuth_stage,
        elem_a=left_trunnion,
        elem_b=left_cheek,
        name="left_trunnion_supported",
    )
    ctx.expect_contact(
        weapon_cradle,
        azimuth_stage,
        elem_a=right_trunnion,
        elem_b=right_cheek,
        name="right_trunnion_supported",
    )

    pedestal_aabb = ctx.part_world_aabb(pedestal)
    turret_aabb = ctx.part_world_aabb(azimuth_stage)
    cradle_aabb = ctx.part_world_aabb(weapon_cradle)
    if pedestal_aabb is None or turret_aabb is None or cradle_aabb is None:
        ctx.fail("part_aabbs_available", "Expected world AABBs for all major parts.")
    else:
        overall_min = (
            min(pedestal_aabb[0][0], turret_aabb[0][0], cradle_aabb[0][0]),
            min(pedestal_aabb[0][1], turret_aabb[0][1], cradle_aabb[0][1]),
            min(pedestal_aabb[0][2], turret_aabb[0][2], cradle_aabb[0][2]),
        )
        overall_max = (
            max(pedestal_aabb[1][0], turret_aabb[1][0], cradle_aabb[1][0]),
            max(pedestal_aabb[1][1], turret_aabb[1][1], cradle_aabb[1][1]),
            max(pedestal_aabb[1][2], turret_aabb[1][2], cradle_aabb[1][2]),
        )
        overall_dims = (
            overall_max[0] - overall_min[0],
            overall_max[1] - overall_min[1],
            overall_max[2] - overall_min[2],
        )
        ctx.check(
            "overall_station_scale",
            1.8 <= overall_dims[0] <= 2.6
            and 0.8 <= overall_dims[1] <= 1.3
            and 1.4 <= overall_dims[2] <= 2.1,
            f"Unexpected overall dimensions {overall_dims}.",
        )

    barrel_aabb = ctx.part_element_world_aabb(weapon_cradle, elem=barrel)
    shield_aabb = ctx.part_element_world_aabb(azimuth_stage, elem=left_shield_plate)
    receiver_aabb = ctx.part_element_world_aabb(weapon_cradle, elem=receiver_body)
    sensor_head_aabb = ctx.part_element_world_aabb(weapon_cradle, elem=sensor_head)
    if barrel_aabb is None or shield_aabb is None or receiver_aabb is None or sensor_head_aabb is None:
        ctx.fail("named_visual_measurements_available", "Expected named visual AABBs for silhouette checks.")
    else:
        ctx.check(
            "barrel_projects_well_ahead_of_shield",
            barrel_aabb[1][0] - shield_aabb[1][0] >= 1.2,
            f"Barrel projection ahead of shield too short: {barrel_aabb[1][0] - shield_aabb[1][0]:.3f} m.",
        )
        ctx.check(
            "sensor_package_above_receiver",
            sensor_head_aabb[0][2] >= receiver_aabb[1][2] - 0.01,
            "Sensor head should ride above the receiver body.",
        )

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_pose_support")

    with ctx.pose({azimuth: math.pi / 2.0}):
        ctx.expect_contact(
            azimuth_stage,
            pedestal,
            elem_a=upper_ring,
            elem_b=lower_ring,
            name="slew_ring_contact_quarter_turn",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="quarter_turn_no_floating")

    if elevation_limits is not None and elevation_limits.lower is not None and elevation_limits.upper is not None:
        with ctx.pose({elevation: elevation_limits.lower}):
            ctx.expect_contact(
                weapon_cradle,
                azimuth_stage,
                elem_a=left_trunnion,
                elem_b=left_cheek,
                name="left_trunnion_lower_pose",
            )
            ctx.expect_contact(
                weapon_cradle,
                azimuth_stage,
                elem_a=right_trunnion,
                elem_b=right_cheek,
                name="right_trunnion_lower_pose",
            )
            ctx.fail_if_isolated_parts(name="elevation_lower_no_floating")

        with ctx.pose({elevation: elevation_limits.upper}):
            ctx.expect_contact(
                weapon_cradle,
                azimuth_stage,
                elem_a=left_trunnion,
                elem_b=left_cheek,
                name="left_trunnion_upper_pose",
            )
            ctx.expect_contact(
                weapon_cradle,
                azimuth_stage,
                elem_a=right_trunnion,
                elem_b=right_cheek,
                name="right_trunnion_upper_pose",
            )
            ctx.fail_if_isolated_parts(name="elevation_upper_no_floating")

        with ctx.pose({azimuth: 1.1, elevation: elevation_limits.upper}):
            ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
