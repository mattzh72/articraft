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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_remote_weapon_station")

    pedestal_paint = model.material("pedestal_paint", rgba=(0.30, 0.32, 0.34, 1.0))
    armor_paint = model.material("armor_paint", rgba=(0.36, 0.39, 0.37, 1.0))
    weapon_black = model.material("weapon_black", rgba=(0.12, 0.12, 0.13, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.16, 0.28, 0.32, 0.55))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.24, 1.0))

    def armored_plate_mesh(
        profile_xz: list[tuple[float, float]],
        *,
        thickness: float,
        name: str,
    ):
        geom = ExtrudeGeometry(profile_xz, thickness, center=True, cap=True, closed=True)
        geom.rotate_x(math.pi / 2.0)
        return mesh_from_geometry(geom, name)

    side_plate = armored_plate_mesh(
        [
            (-0.02, -0.05),
            (0.10, -0.07),
            (0.25, -0.03),
            (0.34, 0.06),
            (0.28, 0.17),
            (0.10, 0.19),
            (0.00, 0.10),
        ],
        thickness=0.03,
        name="rws_side_plate_v3",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.58, 0.44, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=pedestal_paint,
        name="base_plinth",
    )
    pedestal.visual(
        Box((0.30, 0.24, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=pedestal_paint,
        name="lower_column",
    )
    pedestal.visual(
        Box((0.22, 0.18, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=pedestal_paint,
        name="upper_column",
    )
    pedestal.visual(
        Box((0.28, 0.24, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=armor_paint,
        name="top_cap",
    )
    pedestal.visual(
        Box((0.10, 0.004, 0.12)),
        origin=Origin(xyz=(0.0, 0.122, 0.18)),
        material=dark_metal,
        name="service_panel",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.58, 0.44, 0.40)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    turret_base = model.part("turret_base")
    turret_base.visual(
        Cylinder(radius=0.15, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="bearing_ring",
    )
    turret_base.visual(
        Box((0.26, 0.24, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=armor_paint,
        name="upper_deck",
    )
    turret_base.visual(
        Box((0.10, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=pedestal_paint,
        name="center_pedestal",
    )
    turret_base.visual(
        Box((0.12, 0.12, 0.06)),
        origin=Origin(xyz=(0.07, 0.0, 0.12)),
        material=armor_paint,
        name="forward_housing",
    )
    turret_base.visual(
        Box((0.10, 0.10, 0.06)),
        origin=Origin(xyz=(-0.05, 0.0, 0.12)),
        material=pedestal_paint,
        name="rear_counterbox",
    )
    turret_base.visual(
        Box((0.08, 0.06, 0.24)),
        origin=Origin(xyz=(0.01, 0.15, 0.19)),
        material=armor_paint,
        name="left_trunnion_support",
    )
    turret_base.visual(
        Box((0.08, 0.06, 0.24)),
        origin=Origin(xyz=(0.01, -0.15, 0.19)),
        material=armor_paint,
        name="right_trunnion_support",
    )
    turret_base.inertial = Inertial.from_geometry(
        Box((0.30, 0.32, 0.34)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.026, length=0.24),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_axle",
    )
    cradle.visual(
        side_plate,
        origin=Origin(xyz=(0.0, 0.09, 0.0)),
        material=armor_paint,
        name="left_armor_plate",
    )
    cradle.visual(
        side_plate,
        origin=Origin(xyz=(0.0, -0.09, 0.0)),
        material=armor_paint,
        name="right_armor_plate",
    )
    cradle.visual(
        Box((0.08, 0.18, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=armor_paint,
        name="rear_bridge",
    )
    cradle.visual(
        Box((0.10, 0.06, 0.04)),
        origin=Origin(xyz=(0.11, 0.06, -0.03)),
        material=armor_paint,
        name="left_lower_arm",
    )
    cradle.visual(
        Box((0.10, 0.06, 0.04)),
        origin=Origin(xyz=(0.11, -0.06, -0.03)),
        material=armor_paint,
        name="right_lower_arm",
    )
    cradle.visual(
        Box((0.14, 0.05, 0.04)),
        origin=Origin(xyz=(0.18, 0.10, 0.05)),
        material=armor_paint,
        name="left_payload_mount",
    )
    cradle.visual(
        Box((0.16, 0.05, 0.04)),
        origin=Origin(xyz=(0.20, -0.10, 0.05)),
        material=armor_paint,
        name="right_payload_mount",
    )
    cradle.visual(
        Box((0.12, 0.18, 0.04)),
        origin=Origin(xyz=(0.20, 0.0, 0.11)),
        material=pedestal_paint,
        name="top_brace",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.34, 0.26, 0.26)),
        mass=58.0,
        origin=Origin(xyz=(0.16, 0.0, 0.05)),
    )

    sensor_unit = model.part("sensor_unit")
    sensor_unit.visual(
        Box((0.06, 0.01, 0.05)),
        origin=Origin(xyz=(-0.01, 0.01, 0.0)),
        material=dark_metal,
        name="sensor_mount",
    )
    sensor_unit.visual(
        Box((0.12, 0.03, 0.04)),
        origin=Origin(xyz=(0.03, 0.03, 0.02)),
        material=dark_metal,
        name="sensor_standoff",
    )
    sensor_unit.visual(
        Box((0.16, 0.04, 0.10)),
        origin=Origin(xyz=(0.10, 0.065, 0.05)),
        material=armor_paint,
        name="sensor_body",
    )
    sensor_unit.visual(
        Box((0.12, 0.04, 0.02)),
        origin=Origin(xyz=(0.09, 0.065, 0.11)),
        material=pedestal_paint,
        name="sensor_roof",
    )
    sensor_unit.visual(
        Box((0.02, 0.035, 0.08)),
        origin=Origin(xyz=(0.17, 0.065, 0.05)),
        material=pedestal_paint,
        name="sensor_face",
    )
    sensor_unit.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.179, 0.074, 0.06), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor_glass,
        name="sensor_optic_primary",
    )
    sensor_unit.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.179, 0.056, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor_glass,
        name="sensor_optic_secondary",
    )
    sensor_unit.inertial = Inertial.from_geometry(
        Box((0.20, 0.06, 0.13)),
        mass=18.0,
        origin=Origin(xyz=(0.09, 0.055, 0.055)),
    )

    weapon_assembly = model.part("weapon_assembly")
    weapon_assembly.visual(
        Box((0.06, 0.01, 0.05)),
        origin=Origin(xyz=(-0.01, -0.01, 0.0)),
        material=dark_metal,
        name="weapon_mount",
    )
    weapon_assembly.visual(
        Box((0.12, 0.03, 0.04)),
        origin=Origin(xyz=(0.03, -0.025, 0.02)),
        material=dark_metal,
        name="weapon_standoff",
    )
    weapon_assembly.visual(
        Box((0.22, 0.05, 0.10)),
        origin=Origin(xyz=(0.10, -0.05, 0.05)),
        material=weapon_black,
        name="receiver",
    )
    weapon_assembly.visual(
        Box((0.08, 0.05, 0.05)),
        origin=Origin(xyz=(0.01, -0.05, 0.085)),
        material=weapon_black,
        name="breech_block",
    )
    weapon_assembly.visual(
        Cylinder(radius=0.026, length=0.34),
        origin=Origin(xyz=(0.33, -0.05, 0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weapon_black,
        name="barrel_shroud",
    )
    weapon_assembly.visual(
        Cylinder(radius=0.010, length=0.60),
        origin=Origin(xyz=(0.46, -0.05, 0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="barrel",
    )
    weapon_assembly.visual(
        Cylinder(radius=0.008, length=0.26),
        origin=Origin(xyz=(0.30, -0.05, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="gas_tube",
    )
    weapon_assembly.visual(
        Cylinder(radius=0.016, length=0.05),
        origin=Origin(xyz=(0.76, -0.05, 0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="muzzle_device",
    )
    weapon_assembly.visual(
        Box((0.10, 0.05, 0.03)),
        origin=Origin(xyz=(0.08, -0.05, 0.115)),
        material=weapon_black,
        name="feed_cover_block",
    )
    weapon_assembly.inertial = Inertial.from_geometry(
        Box((0.80, 0.07, 0.15)),
        mass=24.0,
        origin=Origin(xyz=(0.38, -0.045, 0.06)),
    )

    model.articulation(
        "pedestal_to_turret_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turret_base,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.2,
            lower=-math.radians(170.0),
            upper=math.radians(170.0),
        ),
    )
    model.articulation(
        "turret_to_cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=turret_base,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=1.0,
            lower=-math.radians(15.0),
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "cradle_to_sensor_unit",
        ArticulationType.FIXED,
        parent=cradle,
        child=sensor_unit,
        origin=Origin(xyz=(0.20, 0.12, 0.03)),
    )
    model.articulation(
        "cradle_to_weapon_assembly",
        ArticulationType.FIXED,
        parent=cradle,
        child=weapon_assembly,
        origin=Origin(xyz=(0.21, -0.12, 0.03)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turret_base = object_model.get_part("turret_base")
    cradle = object_model.get_part("cradle")
    sensor_unit = object_model.get_part("sensor_unit")
    weapon_assembly = object_model.get_part("weapon_assembly")
    sensor_optic = sensor_unit.get_visual("sensor_optic_primary")
    muzzle_device = weapon_assembly.get_visual("muzzle_device")

    yaw = object_model.get_articulation("pedestal_to_turret_yaw")
    pitch = object_model.get_articulation("turret_to_cradle_pitch")

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
        cradle,
        turret_base,
        elem_a="pivot_axle",
        elem_b="left_trunnion_support",
        reason="Cradle pivot axle is modeled as seating into the left trunnion bearing block.",
    )
    ctx.allow_overlap(
        cradle,
        turret_base,
        elem_a="pivot_axle",
        elem_b="right_trunnion_support",
        reason="Cradle pivot axle is modeled as seating into the right trunnion bearing block.",
    )
    ctx.allow_overlap(
        cradle,
        sensor_unit,
        elem_a="left_payload_mount",
        elem_b="sensor_mount",
        reason="The sensor package uses a captured mounting tongue seated inside the left armored saddle bracket.",
    )
    ctx.allow_overlap(
        cradle,
        weapon_assembly,
        elem_a="right_payload_mount",
        elem_b="weapon_mount",
        reason="The weapon package uses a captured mounting tongue seated inside the right armored saddle bracket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "yaw_axis_is_vertical",
        tuple(yaw.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical yaw axis, got {yaw.axis}",
    )
    ctx.check(
        "pitch_axis_is_lateral",
        tuple(abs(v) for v in pitch.axis) == (0.0, 1.0, 0.0),
        f"Expected horizontal pitch axis parallel to Y, got {pitch.axis}",
    )
    ctx.check(
        "yaw_limits_are_broad_but_bounded",
        yaw.motion_limits is not None
        and yaw.motion_limits.lower is not None
        and yaw.motion_limits.upper is not None
        and yaw.motion_limits.lower <= -2.8
        and yaw.motion_limits.upper >= 2.8,
        "Yaw should cover nearly full traversal without being unbounded.",
    )
    ctx.check(
        "pitch_limits_match_weapon_station_range",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower <= -0.25
        and pitch.motion_limits.upper >= 1.0,
        "Pitch should allow slight depression and strong elevation.",
    )

    ctx.expect_contact(turret_base, pedestal, name="turret_base_bears_on_pedestal")
    ctx.expect_contact(cradle, turret_base, name="cradle_supported_between_side_mounts")
    ctx.expect_contact(sensor_unit, cradle, name="sensor_unit_mounted_to_cradle")
    ctx.expect_contact(weapon_assembly, cradle, name="weapon_assembly_mounted_to_cradle")
    ctx.expect_origin_gap(
        sensor_unit,
        weapon_assembly,
        axis="y",
        min_gap=0.18,
        name="sensor_and_weapon_are_on_opposite_sides",
    )

    rest_weapon_pos = ctx.part_world_position(weapon_assembly)
    rest_sensor_pos = ctx.part_world_position(sensor_unit)
    rest_turret_pos = ctx.part_world_position(turret_base)
    rest_muzzle_aabb = ctx.part_element_world_aabb(weapon_assembly, elem=muzzle_device)
    rest_sensor_optic_aabb = ctx.part_element_world_aabb(sensor_unit, elem=sensor_optic)
    if rest_weapon_pos is not None and rest_sensor_pos is not None:
        ctx.check(
            "weapon_projects_farther_forward_than_sensor_head",
            rest_weapon_pos[0] > rest_sensor_pos[0] - 0.05,
            f"Weapon origin {rest_weapon_pos} should sit at least as far forward as the sensor unit {rest_sensor_pos}.",
        )

    yaw_limits = yaw.motion_limits
    if yaw_limits is not None and yaw_limits.lower is not None and yaw_limits.upper is not None:
        with ctx.pose({yaw: yaw_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="yaw_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="yaw_lower_no_floating")
        with ctx.pose({yaw: yaw_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="yaw_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="yaw_upper_no_floating")
            yawed_weapon_pos = ctx.part_world_position(weapon_assembly)
            if rest_weapon_pos is not None and yawed_weapon_pos is not None:
                ctx.check(
                    "yaw_sweeps_weapon_around_pedestal",
                    abs(yawed_weapon_pos[1] - rest_weapon_pos[1]) > 0.18,
                    f"Expected yaw motion to swing the weapon laterally: rest={rest_weapon_pos}, yawed={yawed_weapon_pos}",
                )

    pitch_limits = pitch.motion_limits
    if pitch_limits is not None and pitch_limits.lower is not None and pitch_limits.upper is not None:
        with ctx.pose({pitch: pitch_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pitch_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="pitch_lower_no_floating")
            ctx.expect_contact(cradle, turret_base, name="pitch_lower_cradle_stays_supported")
        with ctx.pose({pitch: pitch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pitch_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="pitch_upper_no_floating")
            ctx.expect_contact(cradle, turret_base, name="pitch_upper_cradle_stays_supported")
            elevated_weapon_pos = ctx.part_world_position(weapon_assembly)
            elevated_sensor_pos = ctx.part_world_position(sensor_unit)
            elevated_muzzle_aabb = ctx.part_element_world_aabb(weapon_assembly, elem=muzzle_device)
            elevated_sensor_optic_aabb = ctx.part_element_world_aabb(sensor_unit, elem=sensor_optic)
            if rest_muzzle_aabb is not None and elevated_muzzle_aabb is not None:
                rest_muzzle_z = (rest_muzzle_aabb[0][2] + rest_muzzle_aabb[1][2]) * 0.5
                elevated_muzzle_z = (elevated_muzzle_aabb[0][2] + elevated_muzzle_aabb[1][2]) * 0.5
                ctx.check(
                    "weapon_rises_when_cradle_elevates",
                    elevated_muzzle_z > rest_muzzle_z + 0.20,
                    f"Expected elevation to raise the muzzle: rest_z={rest_muzzle_z}, elevated_z={elevated_muzzle_z}",
                )
            elif rest_weapon_pos is not None and elevated_weapon_pos is not None:
                ctx.check(
                    "weapon_rises_when_cradle_elevates",
                    elevated_weapon_pos[2] > rest_weapon_pos[2] + 0.10,
                    f"Expected elevation to raise the weapon mount: rest={rest_weapon_pos}, elevated={elevated_weapon_pos}",
                )
            if rest_sensor_optic_aabb is not None and elevated_sensor_optic_aabb is not None:
                rest_sensor_optic_z = (rest_sensor_optic_aabb[0][2] + rest_sensor_optic_aabb[1][2]) * 0.5
                elevated_sensor_optic_z = (
                    elevated_sensor_optic_aabb[0][2] + elevated_sensor_optic_aabb[1][2]
                ) * 0.5
                ctx.check(
                    "sensor_rises_with_the_same_cradle_motion",
                    elevated_sensor_optic_z > rest_sensor_optic_z + 0.08,
                    f"Expected sensor optic to pitch upward with the cradle: rest_z={rest_sensor_optic_z}, elevated_z={elevated_sensor_optic_z}",
                )
            elif rest_sensor_pos is not None and elevated_sensor_pos is not None:
                ctx.check(
                    "sensor_rises_with_the_same_cradle_motion",
                    elevated_sensor_pos[2] > rest_sensor_pos[2] + 0.08,
                    f"Expected sensor to pitch upward with the cradle: rest={rest_sensor_pos}, elevated={elevated_sensor_pos}",
                )

    if rest_weapon_pos is not None and rest_turret_pos is not None:
        ctx.check(
            "weapon_sits_above_the_rotating_base",
            rest_weapon_pos[2] > rest_turret_pos[2] + 0.05,
            f"Weapon origin {rest_weapon_pos} should sit above turret base origin {rest_turret_pos}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
