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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_remote_weapon_station", assets=ASSETS)

    armor = model.material("armor", rgba=(0.43, 0.45, 0.39, 1.0))
    armor_dark = model.material("armor_dark", rgba=(0.26, 0.28, 0.25, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.18, 0.19, 0.21, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.39, 0.55, 0.61, 0.42))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))

    side_plate_profile = [
        (0.000, -0.030),
        (0.072, -0.030),
        (0.148, -0.006),
        (0.170, 0.050),
        (0.154, 0.134),
        (0.050, 0.134),
        (0.000, 0.090),
    ]
    side_plate_mesh = _save_mesh(
        "rws_side_plate.obj",
        ExtrudeGeometry(side_plate_profile, 0.008, center=True),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.420, 0.420, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=armor_dark,
        name="base_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.110, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=armor,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.150, 0.090, 0.070)),
        origin=Origin(xyz=(-0.055, 0.0, 0.065)),
        material=armor,
        name="service_box",
    )
    pedestal.visual(
        Cylinder(radius=0.140, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1425)),
        material=gunmetal,
        name="top_bearing",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.420, 0.420, 0.155)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
    )

    turret_body = model.part("turret_body")
    turret_body.visual(
        Cylinder(radius=0.145, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=gunmetal,
        name="turntable_drum",
    )
    turret_body.visual(
        Box((0.110, 0.090, 0.048)),
        origin=Origin(xyz=(-0.050, 0.0, 0.054)),
        material=armor_dark,
        name="turret_block",
    )
    turret_body.visual(
        Box((0.090, 0.080, 0.060)),
        origin=Origin(xyz=(-0.105, 0.0, 0.060)),
        material=armor_dark,
        name="rear_drive_box",
    )
    turret_body.visual(
        Box((0.060, 0.040, 0.125)),
        origin=Origin(xyz=(-0.005, 0.132, 0.0925)),
        material=armor,
        name="left_support",
    )
    turret_body.visual(
        Box((0.060, 0.040, 0.125)),
        origin=Origin(xyz=(-0.005, -0.132, 0.0925)),
        material=armor,
        name="right_support",
    )
    turret_body.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.119, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_bearing",
    )
    turret_body.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, -0.119, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_bearing",
    )
    turret_body.inertial = Inertial.from_geometry(
        Box((0.290, 0.270, 0.190)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    cradle_frame = model.part("cradle_frame")
    cradle_frame.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, 0.094, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor,
        name="left_plate",
    )
    cradle_frame.visual(
        side_plate_mesh,
        origin=Origin(xyz=(0.0, -0.094, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor,
        name="right_plate",
    )
    cradle_frame.visual(
        Box((0.108, 0.182, 0.020)),
        origin=Origin(xyz=(0.096, 0.0, 0.000)),
        material=armor_dark,
        name="lower_beam",
    )
    cradle_frame.visual(
        Box((0.036, 0.182, 0.024)),
        origin=Origin(xyz=(0.012, 0.0, 0.012)),
        material=armor_dark,
        name="trunnion_bridge",
    )
    cradle_frame.visual(
        Box((0.100, 0.182, 0.018)),
        origin=Origin(xyz=(0.110, 0.0, 0.120)),
        material=armor,
        name="top_shield",
    )
    cradle_frame.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.101, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="left_trunnion",
    )
    cradle_frame.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, -0.101, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="right_trunnion",
    )
    cradle_frame.inertial = Inertial.from_geometry(
        Box((0.190, 0.200, 0.170)),
        mass=42.0,
        origin=Origin(xyz=(0.085, 0.0, 0.055)),
    )

    sensor_pod = model.part("sensor_pod")
    sensor_pod.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.010, 0.005, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor,
        name="sensor_mount",
    )
    sensor_pod.visual(
        Box((0.100, 0.055, 0.070)),
        origin=Origin(xyz=(0.018, 0.0375, 0.045)),
        material=armor_dark,
        name="sensor_body",
    )
    sensor_pod.visual(
        Box((0.036, 0.055, 0.034)),
        origin=Origin(xyz=(0.084, 0.0375, 0.045)),
        material=armor,
        name="lens_hood",
    )
    sensor_pod.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.102, 0.0375, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor_glass,
        name="optic_aperture",
    )
    sensor_pod.visual(
        Box((0.032, 0.022, 0.022)),
        origin=Origin(xyz=(0.010, 0.0375, 0.090)),
        material=matte_black,
        name="rangefinder_cap",
    )
    sensor_pod.inertial = Inertial.from_geometry(
        Box((0.120, 0.065, 0.112)),
        mass=7.5,
        origin=Origin(xyz=(0.030, 0.0325, 0.056)),
    )

    weapon_module = model.part("weapon_module")
    weapon_module.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.010, -0.005, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=armor,
        name="receiver_mount",
    )
    weapon_module.visual(
        Box((0.125, 0.060, 0.055)),
        origin=Origin(xyz=(0.020, -0.040, 0.030)),
        material=armor_dark,
        name="receiver",
    )
    weapon_module.visual(
        Box((0.080, 0.042, 0.048)),
        origin=Origin(xyz=(-0.006, -0.031, 0.084)),
        material=armor,
        name="ammo_box",
    )
    weapon_module.visual(
        Cylinder(radius=0.015, length=0.190),
        origin=Origin(xyz=(0.150, -0.040, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_jacket",
    )
    weapon_module.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(xyz=(0.280, -0.040, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="barrel_core",
    )
    weapon_module.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(0.330, -0.040, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="flash_hider",
    )
    weapon_module.visual(
        Box((0.038, 0.026, 0.024)),
        origin=Origin(xyz=(0.050, -0.040, 0.060)),
        material=matte_black,
        name="feed_cover",
    )
    weapon_module.inertial = Inertial.from_geometry(
        Box((0.360, 0.080, 0.132)),
        mass=12.0,
        origin=Origin(xyz=(0.150, -0.040, 0.060)),
    )

    model.articulation(
        "pedestal_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turret_body,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=1.4),
    )
    model.articulation(
        "turret_pitch",
        ArticulationType.REVOLUTE,
        parent=turret_body,
        child=cradle_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.2,
            lower=-math.radians(20.0),
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "cradle_to_sensor",
        ArticulationType.FIXED,
        parent=cradle_frame,
        child=sensor_pod,
        origin=Origin(xyz=(0.092, 0.098, -0.004)),
    )
    model.articulation(
        "cradle_to_weapon",
        ArticulationType.FIXED,
        parent=cradle_frame,
        child=weapon_module,
        origin=Origin(xyz=(0.100, -0.098, -0.002)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    turret_body = object_model.get_part("turret_body")
    cradle_frame = object_model.get_part("cradle_frame")
    sensor_pod = object_model.get_part("sensor_pod")
    weapon_module = object_model.get_part("weapon_module")

    pedestal_yaw = object_model.get_articulation("pedestal_yaw")
    turret_pitch = object_model.get_articulation("turret_pitch")

    top_bearing = pedestal.get_visual("top_bearing")
    turntable_drum = turret_body.get_visual("turntable_drum")
    left_bearing = turret_body.get_visual("left_bearing")
    right_bearing = turret_body.get_visual("right_bearing")
    left_plate = cradle_frame.get_visual("left_plate")
    right_plate = cradle_frame.get_visual("right_plate")
    left_trunnion = cradle_frame.get_visual("left_trunnion")
    right_trunnion = cradle_frame.get_visual("right_trunnion")
    lower_beam = cradle_frame.get_visual("lower_beam")
    sensor_mount = sensor_pod.get_visual("sensor_mount")
    receiver_mount = weapon_module.get_visual("receiver_mount")
    sensor_body = sensor_pod.get_visual("sensor_body")
    receiver = weapon_module.get_visual("receiver")
    barrel_jacket = weapon_module.get_visual("barrel_jacket")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        sensor_pod,
        cradle_frame,
        elem_a=sensor_mount,
        elem_b=left_plate,
        reason="Sensor pod uses a small mounting spigot captured by the left armor plate aperture.",
    )
    ctx.allow_overlap(
        weapon_module,
        cradle_frame,
        elem_a=receiver_mount,
        elem_b=right_plate,
        reason="Weapon receiver uses a small mounting spigot captured by the right armor plate aperture.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(
        turret_body,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="yaw axis stays centered on pedestal",
    )
    ctx.expect_contact(
        turret_body,
        pedestal,
        elem_a=turntable_drum,
        elem_b=top_bearing,
        name="turntable sits on top bearing",
    )
    ctx.expect_overlap(
        turret_body,
        pedestal,
        axes="xy",
        elem_a=turntable_drum,
        elem_b=top_bearing,
        min_overlap=0.26,
        name="turntable fully covers bearing ring",
    )
    ctx.expect_origin_gap(
        cradle_frame,
        turret_body,
        axis="z",
        min_gap=0.11,
        max_gap=0.12,
        name="pitch cradle is carried above the yaw deck",
    )
    ctx.expect_contact(
        cradle_frame,
        turret_body,
        elem_a=left_trunnion,
        elem_b=left_bearing,
        name="left trunnion seats in left bearing",
    )
    ctx.expect_contact(
        cradle_frame,
        turret_body,
        elem_a=right_trunnion,
        elem_b=right_bearing,
        name="right trunnion seats in right bearing",
    )
    ctx.expect_contact(
        sensor_pod,
        cradle_frame,
        name="sensor pod is mounted to cradle",
    )
    ctx.expect_contact(
        weapon_module,
        cradle_frame,
        name="weapon module is mounted to cradle",
    )
    ctx.expect_contact(
        sensor_pod,
        cradle_frame,
        elem_a=sensor_mount,
        elem_b=left_plate,
        name="sensor pod bracket sits against left armor plate",
    )
    ctx.expect_contact(
        weapon_module,
        cradle_frame,
        elem_a=receiver_mount,
        elem_b=right_plate,
        name="weapon mount sits against right armor plate",
    )
    ctx.expect_origin_gap(
        sensor_pod,
        weapon_module,
        axis="y",
        min_gap=0.10,
        name="sensor pod stays on the left side of the cradle",
    )

    barrel_rest = ctx.part_element_world_aabb(weapon_module, elem=barrel_jacket)
    sensor_rest = ctx.part_element_world_aabb(sensor_pod, elem=sensor_body)
    weapon_rest = ctx.part_world_position(weapon_module)
    sensor_origin_rest = ctx.part_world_position(sensor_pod)
    assert barrel_rest is not None
    assert sensor_rest is not None
    assert weapon_rest is not None
    assert sensor_origin_rest is not None
    ctx.check(
        "barrel projects well ahead of sensor head",
        barrel_rest[1][0] > sensor_rest[1][0] + 0.15,
        details=f"barrel x_max={barrel_rest[1][0]:.4f}, sensor x_max={sensor_rest[1][0]:.4f}",
    )

    with ctx.pose({pedestal_yaw: math.pi / 2.0}):
        weapon_yaw = ctx.part_world_position(weapon_module)
        sensor_yaw = ctx.part_world_position(sensor_pod)
        assert weapon_yaw is not None
        assert sensor_yaw is not None
        ctx.expect_contact(
            turret_body,
            pedestal,
            elem_a=turntable_drum,
            elem_b=top_bearing,
            name="yawed turntable remains seated",
        )
        ctx.check(
            "yaw rotation swings weapon around vertical axis",
            abs(weapon_yaw[0] + weapon_rest[1]) < 0.01
            and abs(weapon_yaw[1] - weapon_rest[0]) < 0.01,
            details=f"rest={weapon_rest}, yawed={weapon_yaw}",
        )
        ctx.check(
            "yaw rotation carries sensor with the cradle assembly",
            abs(sensor_yaw[0] + sensor_origin_rest[1]) < 0.01
            and abs(sensor_yaw[1] - sensor_origin_rest[0]) < 0.01,
            details=f"rest={sensor_origin_rest}, yawed={sensor_yaw}",
        )

    with ctx.pose({turret_pitch: math.radians(50.0)}):
        barrel_up = ctx.part_element_world_aabb(weapon_module, elem=barrel_jacket)
        assert barrel_up is not None
        ctx.check(
            "pitch-up raises the barrel assembly",
            barrel_up[1][2] > barrel_rest[1][2] + 0.11,
            details=f"rest_zmax={barrel_rest[1][2]:.4f}, up_zmax={barrel_up[1][2]:.4f}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at high elevation")

    with ctx.pose({turret_pitch: -math.radians(15.0)}):
        barrel_down = ctx.part_element_world_aabb(weapon_module, elem=barrel_jacket)
        assert barrel_down is not None
        ctx.check(
            "pitch-down lowers the barrel assembly",
            barrel_down[1][2] < barrel_rest[1][2] - 0.04,
            details=f"rest_zmax={barrel_rest[1][2]:.4f}, down_zmax={barrel_down[1][2]:.4f}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at depressed elevation")

    with ctx.pose({pedestal_yaw: math.pi / 3.0, turret_pitch: math.radians(60.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in combined yaw and high pitch pose")
        ctx.expect_contact(
            cradle_frame,
            turret_body,
            elem_a=left_trunnion,
            elem_b=left_bearing,
            name="left trunnion stays seated at high pitch",
        )
        ctx.expect_contact(
            cradle_frame,
            turret_body,
            elem_a=right_trunnion,
            elem_b=right_bearing,
            name="right trunnion stays seated at high pitch",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
