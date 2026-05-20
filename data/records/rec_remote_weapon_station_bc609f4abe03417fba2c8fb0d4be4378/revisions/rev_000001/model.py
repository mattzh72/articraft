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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def _axis_matches(axis: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    base_gray = model.material("base_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    armor_gray = model.material("armor_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.22, 0.23, 0.25, 1.0))
    olive = model.material("olive", rgba=(0.34, 0.37, 0.27, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.22, 0.38, 0.46, 0.55))
    flat_black = model.material("flat_black", rgba=(0.07, 0.08, 0.09, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((1.10, 0.86, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_gray,
        name="roof_plate",
    )
    pedestal_base.visual(
        Box((0.58, 0.46, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=armor_gray,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Box((0.18, 0.26, 0.14)),
        origin=Origin(xyz=(-0.17, 0.0, 0.12)),
        material=base_gray,
        name="service_blister",
    )
    pedestal_base.visual(
        Cylinder(radius=0.25, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=dark_metal,
        name="bearing_track",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((1.10, 0.86, 0.35)),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
    )

    traverse_head = model.part("traverse_head")
    traverse_head.visual(
        Cylinder(radius=0.25, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="turntable_ring",
    )
    traverse_head.visual(
        Box((0.30, 0.36, 0.22)),
        origin=Origin(xyz=(-0.14, 0.0, 0.17)),
        material=armor_gray,
        name="central_housing",
    )
    traverse_head.visual(
        Box((0.18, 0.24, 0.18)),
        origin=Origin(xyz=(-0.38, 0.0, 0.19)),
        material=base_gray,
        name="rear_drive_box",
    )
    traverse_head.visual(
        Box((0.10, 0.08, 0.26)),
        origin=Origin(xyz=(0.02, -0.21, 0.33)),
        material=armor_gray,
        name="left_trunnion_cheek",
    )
    traverse_head.visual(
        Box((0.10, 0.08, 0.26)),
        origin=Origin(xyz=(0.02, 0.21, 0.33)),
        material=armor_gray,
        name="right_trunnion_cheek",
    )
    traverse_head.visual(
        Box((0.16, 0.10, 0.12)),
        origin=Origin(xyz=(-0.04, 0.23, 0.20)),
        material=base_gray,
        name="sensor_mount_pad",
    )
    traverse_head.inertial = Inertial.from_geometry(
        Box((0.64, 0.70, 0.54)),
        mass=190.0,
        origin=Origin(xyz=(-0.12, 0.0, 0.27)),
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Cylinder(radius=0.030, length=0.04),
        origin=Origin(xyz=(0.0, -0.16, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_pivot_stub",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.030, length=0.04),
        origin=Origin(xyz=(0.0, 0.16, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_pivot_stub",
    )
    weapon_cradle.visual(
        Box((0.12, 0.34, 0.12)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=base_gray,
        name="trunnion_block",
    )
    weapon_cradle.visual(
        Box((0.50, 0.02, 0.22)),
        origin=Origin(xyz=(0.30, -0.10, 0.03)),
        material=armor_gray,
        name="left_side_plate",
    )
    weapon_cradle.visual(
        Box((0.50, 0.02, 0.22)),
        origin=Origin(xyz=(0.30, 0.10, 0.03)),
        material=armor_gray,
        name="right_side_plate",
    )
    weapon_cradle.visual(
        Box((0.44, 0.18, 0.16)),
        origin=Origin(xyz=(0.34, 0.0, 0.04)),
        material=gunmetal,
        name="receiver_body",
    )
    weapon_cradle.visual(
        Box((0.18, 0.14, 0.12)),
        origin=Origin(xyz=(0.12, 0.0, 0.02)),
        material=base_gray,
        name="recoil_housing",
    )
    weapon_cradle.visual(
        Box((0.24, 0.16, 0.08)),
        origin=Origin(xyz=(0.30, 0.0, 0.16)),
        material=base_gray,
        name="top_cover",
    )
    weapon_cradle.visual(
        Box((0.22, 0.20, 0.16)),
        origin=Origin(xyz=(0.66, 0.0, 0.02)),
        material=armor_gray,
        name="front_shroud_block",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.050, length=0.30),
        origin=Origin(xyz=(0.92, 0.0, 0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel_shroud",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.022, length=0.96),
        origin=Origin(xyz=(1.55, 0.0, 0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="barrel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.028, length=0.10),
        origin=Origin(xyz=(2.08, 0.0, 0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="muzzle_device",
    )
    weapon_cradle.visual(
        Box((0.20, 0.12, 0.22)),
        origin=Origin(xyz=(0.30, -0.15, 0.02)),
        material=olive,
        name="ammo_box",
    )
    weapon_cradle.visual(
        Box((0.16, 0.10, 0.14)),
        origin=Origin(xyz=(0.24, 0.14, 0.03)),
        material=base_gray,
        name="elevation_actuator_housing",
    )
    weapon_cradle.inertial = Inertial.from_geometry(
        Box((2.14, 0.44, 0.30)),
        mass=120.0,
        origin=Origin(xyz=(0.92, 0.0, 0.06)),
    )

    sensor_mast = model.part("sensor_mast")
    sensor_mast.visual(
        Box((0.14, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=armor_gray,
        name="base_shoe",
    )
    sensor_mast.visual(
        Box((0.08, 0.06, 0.30)),
        origin=Origin(xyz=(0.02, 0.0, 0.27)),
        material=armor_gray,
        name="mast_post",
    )
    sensor_mast.visual(
        Box((0.10, 0.08, 0.08)),
        origin=Origin(xyz=(0.09, 0.0, 0.46)),
        material=base_gray,
        name="pod_bracket_root",
    )
    sensor_mast.visual(
        Box((0.04, 0.04, 0.10)),
        origin=Origin(xyz=(0.16, -0.06, 0.46)),
        material=base_gray,
        name="left_pod_ear",
    )
    sensor_mast.visual(
        Box((0.04, 0.04, 0.10)),
        origin=Origin(xyz=(0.16, 0.06, 0.46)),
        material=base_gray,
        name="right_pod_ear",
    )
    sensor_mast.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.58)),
        mass=34.0,
        origin=Origin(xyz=(0.08, 0.0, 0.28)),
    )

    sensor_pod = model.part("sensor_pod")
    sensor_pod.visual(
        Box((0.08, 0.06, 0.06)),
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material=base_gray,
        name="hinge_boss",
    )
    sensor_pod.visual(
        Box((0.18, 0.08, 0.10)),
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        material=armor_gray,
        name="pod_shell",
    )
    sensor_pod.visual(
        Box((0.08, 0.06, 0.08)),
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
        material=armor_gray,
        name="nose_cap",
    )
    sensor_pod.visual(
        Cylinder(radius=0.016, length=0.04),
        origin=Origin(xyz=(0.0, -0.02, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_tilt_stub",
    )
    sensor_pod.visual(
        Cylinder(radius=0.016, length=0.04),
        origin=Origin(xyz=(0.0, 0.02, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_tilt_stub",
    )
    sensor_pod.visual(
        Box((0.02, 0.05, 0.06)),
        origin=Origin(xyz=(0.33, 0.0, 0.0)),
        material=sensor_glass,
        name="window_panel",
    )
    sensor_pod.visual(
        Cylinder(radius=0.016, length=0.02),
        origin=Origin(xyz=(0.325, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flat_black,
        name="day_camera",
    )
    sensor_pod.visual(
        Cylinder(radius=0.022, length=0.02),
        origin=Origin(xyz=(0.325, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flat_black,
        name="thermal_imager",
    )
    sensor_pod.inertial = Inertial.from_geometry(
        Box((0.34, 0.08, 0.12)),
        mass=14.0,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal_base,
        child=traverse_head,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=1.0,
            lower=-3.05,
            upper=3.05,
        ),
    )
    model.articulation(
        "cradle_pitch",
        ArticulationType.REVOLUTE,
        parent=traverse_head,
        child=weapon_cradle,
        origin=Origin(xyz=(0.02, 0.0, 0.44)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.10,
        ),
    )
    model.articulation(
        "head_to_sensor_mast",
        ArticulationType.FIXED,
        parent=traverse_head,
        child=sensor_mast,
        origin=Origin(xyz=(-0.04, 0.32, 0.14)),
    )
    model.articulation(
        "sensor_tilt",
        ArticulationType.REVOLUTE,
        parent=sensor_mast,
        child=sensor_pod,
        origin=Origin(xyz=(0.16, 0.0, 0.46)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=-0.45,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    traverse_head = object_model.get_part("traverse_head")
    weapon_cradle = object_model.get_part("weapon_cradle")
    sensor_mast = object_model.get_part("sensor_mast")
    sensor_pod = object_model.get_part("sensor_pod")

    base_yaw = object_model.get_articulation("base_yaw")
    cradle_pitch = object_model.get_articulation("cradle_pitch")
    sensor_tilt = object_model.get_articulation("sensor_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        weapon_cradle,
        traverse_head,
        elem_a="left_pivot_stub",
        elem_b="left_trunnion_cheek",
        reason="left trunnion stub is intentionally captured inside the left yoke cheek bearing",
    )
    ctx.allow_overlap(
        weapon_cradle,
        traverse_head,
        elem_a="right_pivot_stub",
        elem_b="right_trunnion_cheek",
        reason="right trunnion stub is intentionally captured inside the right yoke cheek bearing",
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
        "yaw articulation is vertical revolute",
        base_yaw.articulation_type == ArticulationType.REVOLUTE and _axis_matches(base_yaw.axis, (0.0, 0.0, 1.0)),
        f"expected vertical yaw axis, got {base_yaw.axis}",
    )
    ctx.check(
        "weapon cradle pitches on transverse axis",
        cradle_pitch.articulation_type == ArticulationType.REVOLUTE and _axis_matches(cradle_pitch.axis, (0.0, 1.0, 0.0)),
        f"expected transverse cradle pitch axis, got {cradle_pitch.axis}",
    )
    ctx.check(
        "sensor pod tilts on separate transverse hinge",
        sensor_tilt.articulation_type == ArticulationType.REVOLUTE and _axis_matches(sensor_tilt.axis, (0.0, 1.0, 0.0)),
        f"expected transverse sensor tilt axis, got {sensor_tilt.axis}",
    )

    ctx.expect_contact(
        traverse_head,
        pedestal_base,
        elem_a="turntable_ring",
        elem_b="bearing_track",
        name="traverse head sits on base bearing",
    )
    ctx.expect_contact(
        weapon_cradle,
        traverse_head,
        elem_a="left_pivot_stub",
        elem_b="left_trunnion_cheek",
        name="left cradle trunnion seats in yoke cheek",
    )
    ctx.expect_contact(
        weapon_cradle,
        traverse_head,
        elem_a="right_pivot_stub",
        elem_b="right_trunnion_cheek",
        name="right cradle trunnion seats in yoke cheek",
    )
    ctx.expect_contact(
        sensor_mast,
        traverse_head,
        elem_a="base_shoe",
        elem_b="sensor_mount_pad",
        name="sensor mast mounts to traverse head side",
    )
    ctx.expect_contact(
        sensor_pod,
        sensor_mast,
        elem_a="left_tilt_stub",
        elem_b="left_pod_ear",
        name="left sensor tilt stub sits in bracket ear",
    )
    ctx.expect_contact(
        sensor_pod,
        sensor_mast,
        elem_a="right_tilt_stub",
        elem_b="right_pod_ear",
        name="right sensor tilt stub sits in bracket ear",
    )
    ctx.expect_gap(
        sensor_mast,
        weapon_cradle,
        axis="y",
        min_gap=0.05,
        name="sensor mast stands clear of weapon cradle",
    )
    ctx.expect_gap(
        sensor_pod,
        weapon_cradle,
        axis="y",
        min_gap=0.05,
        name="sensor pod remains outboard of gun cradle",
    )

    with ctx.pose({cradle_pitch: -0.30, sensor_tilt: 0.55}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in depressed surveillance pose")
        ctx.expect_gap(
            sensor_pod,
            weapon_cradle,
            axis="y",
            min_gap=0.04,
            name="sensor pod clears cradle when tilted up",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
