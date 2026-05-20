from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    armor_dark = model.material("armor_dark", rgba=(0.24, 0.26, 0.28, 1.0))
    armor_mid = model.material("armor_mid", rgba=(0.33, 0.35, 0.38, 1.0))
    weapon_black = model.material("weapon_black", rgba=(0.12, 0.13, 0.14, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.16, 0.17, 0.18, 1.0))
    optic_glass = model.material("optic_glass", rgba=(0.18, 0.38, 0.48, 0.85))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.34, 0.30, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=armor_dark,
        name="base_plate",
    )
    pedestal.visual(
        Box((0.24, 0.20, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=armor_mid,
        name="lower_body",
    )
    pedestal.visual(
        Box((0.18, 0.15, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=armor_mid,
        name="upper_body",
    )
    pedestal.visual(
        Cylinder(radius=0.11, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.152)),
        material=armor_dark,
        name="bearing_top",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=armor_dark,
        name="bearing_core",
    )

    turret_frame = model.part("turret_frame")
    turret_frame.visual(
        Cylinder(radius=0.108, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=armor_dark,
        name="bearing_flange",
    )
    turret_frame.visual(
        Box((0.25, 0.18, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=armor_mid,
        name="frame_deck",
    )
    turret_frame.visual(
        Box((0.12, 0.10, 0.05)),
        origin=Origin(xyz=(-0.04, 0.0, 0.071)),
        material=armor_mid,
        name="center_block",
    )
    turret_frame.visual(
        Box((0.19, 0.018, 0.15)),
        origin=Origin(xyz=(0.02, -0.086, 0.115)),
        material=armor_mid,
        name="left_cheek",
    )
    turret_frame.visual(
        Box((0.19, 0.018, 0.15)),
        origin=Origin(xyz=(0.02, 0.086, 0.115)),
        material=armor_mid,
        name="right_cheek",
    )
    turret_frame.visual(
        Box((0.035, 0.18, 0.022)),
        origin=Origin(xyz=(-0.075, 0.0, 0.182)),
        material=armor_dark,
        name="top_bridge",
    )
    turret_frame.visual(
        Box((0.05, 0.18, 0.014)),
        origin=Origin(xyz=(0.10, 0.0, 0.040)),
        material=armor_dark,
        name="front_brace",
    )
    turret_frame.visual(
        Box((0.04, 0.036, 0.04)),
        origin=Origin(xyz=(0.02, 0.111, 0.118)),
        material=armor_dark,
        name="sensor_arm",
    )
    turret_frame.visual(
        Box((0.018, 0.006, 0.06)),
        origin=Origin(xyz=(0.02, 0.132, 0.118)),
        material=armor_dark,
        name="sensor_clip_inner",
    )
    turret_frame.visual(
        Box((0.018, 0.006, 0.06)),
        origin=Origin(xyz=(0.02, 0.188, 0.118)),
        material=armor_dark,
        name="sensor_clip_outer",
    )
    turret_frame.visual(
        Box((0.018, 0.062, 0.006)),
        origin=Origin(xyz=(0.02, 0.160, 0.151)),
        material=armor_dark,
        name="sensor_clip_top",
    )
    turret_frame.visual(
        Box((0.018, 0.062, 0.006)),
        origin=Origin(xyz=(0.02, 0.160, 0.085)),
        material=armor_dark,
        name="sensor_clip_bottom",
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Cylinder(radius=0.012, length=0.154),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=weapon_black,
        name="trunnion_shaft",
    )
    weapon_cradle.visual(
        Box((0.18, 0.08, 0.05)),
        origin=Origin(xyz=(0.16, 0.0, -0.005)),
        material=weapon_black,
        name="receiver_body",
    )
    weapon_cradle.visual(
        Box((0.10, 0.075, 0.022)),
        origin=Origin(xyz=(0.10, 0.0, 0.028)),
        material=weapon_black,
        name="top_cover",
    )
    weapon_cradle.visual(
        Box((0.06, 0.055, 0.041)),
        origin=Origin(xyz=(0.09, 0.0, -0.049)),
        material=weapon_black,
        name="feed_box",
    )
    weapon_cradle.visual(
        Box((0.08, 0.075, 0.045)),
        origin=Origin(xyz=(0.03, 0.0, 0.020)),
        material=weapon_black,
        name="rear_pack",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.017, length=0.40),
        origin=Origin(xyz=(0.34, 0.0, -0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=weapon_black,
        name="barrel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.022, length=0.04),
        origin=Origin(xyz=(0.56, 0.0, -0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=weapon_black,
        name="muzzle_shroud",
    )

    sensor_pod = model.part("sensor_pod")
    sensor_pod.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=sensor_black,
        name="sensor_hub",
    )
    sensor_pod.visual(
        Box((0.05, 0.020, 0.020)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=sensor_black,
        name="sensor_neck",
    )
    sensor_pod.visual(
        Box((0.10, 0.040, 0.052)),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        material=sensor_black,
        name="pod_body",
    )
    sensor_pod.visual(
        Box((0.030, 0.034, 0.040)),
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        material=sensor_black,
        name="pod_nose",
    )
    sensor_pod.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.187, 0.0, 0.011), rpy=(0.0, pi / 2.0, 0.0)),
        material=optic_glass,
        name="primary_optic",
    )
    sensor_pod.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.185, 0.0, -0.014), rpy=(0.0, pi / 2.0, 0.0)),
        material=optic_glass,
        name="secondary_optic",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turret_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=turret_frame,
        child=weapon_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=-0.10,
            upper=0.75,
        ),
    )
    model.articulation(
        "sensor_tilt",
        ArticulationType.REVOLUTE,
        parent=turret_frame,
        child=sensor_pod,
        origin=Origin(xyz=(0.02, 0.160, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.4,
            lower=-0.40,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turret_frame = object_model.get_part("turret_frame")
    weapon_cradle = object_model.get_part("weapon_cradle")
    sensor_pod = object_model.get_part("sensor_pod")

    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")
    sensor_tilt = object_model.get_articulation("sensor_tilt")

    bearing_top = pedestal.get_visual("bearing_top")
    bearing_flange = turret_frame.get_visual("bearing_flange")
    frame_deck = turret_frame.get_visual("frame_deck")
    left_cheek = turret_frame.get_visual("left_cheek")
    right_cheek = turret_frame.get_visual("right_cheek")
    sensor_clip_inner = turret_frame.get_visual("sensor_clip_inner")
    sensor_clip_outer = turret_frame.get_visual("sensor_clip_outer")
    trunnion_shaft = weapon_cradle.get_visual("trunnion_shaft")
    barrel = weapon_cradle.get_visual("barrel")
    pod_body = sensor_pod.get_visual("pod_body")
    sensor_hub = sensor_pod.get_visual("sensor_hub")

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
        turret_frame,
        pedestal,
        elem_a=bearing_flange,
        elem_b=bearing_top,
        name="azimuth_ring_is_seated",
    )
    ctx.expect_overlap(
        turret_frame,
        pedestal,
        axes="xy",
        elem_a=bearing_flange,
        elem_b=bearing_top,
        min_overlap=0.20,
        name="azimuth_ring_has_bearing_overlap",
    )
    ctx.expect_contact(
        weapon_cradle,
        turret_frame,
        elem_a=trunnion_shaft,
        elem_b=left_cheek,
        name="left_trunnion_contacts_left_cheek",
    )
    ctx.expect_contact(
        weapon_cradle,
        turret_frame,
        elem_a=trunnion_shaft,
        elem_b=right_cheek,
        name="right_trunnion_contacts_right_cheek",
    )
    ctx.expect_gap(
        weapon_cradle,
        turret_frame,
        axis="z",
        positive_elem=barrel,
        negative_elem=frame_deck,
        min_gap=0.03,
        name="barrel_clears_frame_deck",
    )
    ctx.expect_gap(
        sensor_pod,
        turret_frame,
        axis="y",
        positive_elem=pod_body,
        negative_elem=right_cheek,
        min_gap=0.03,
        name="sensor_pod_sits_outboard_of_frame",
    )
    ctx.expect_contact(
        sensor_pod,
        turret_frame,
        elem_a=sensor_hub,
        elem_b=sensor_clip_inner,
        name="sensor_hub_clipped_to_inner_bracket",
    )
    ctx.expect_contact(
        sensor_pod,
        turret_frame,
        elem_a=sensor_hub,
        elem_b=sensor_clip_outer,
        name="sensor_hub_clipped_to_outer_keeper",
    )

    with ctx.pose({azimuth: 0.75, elevation: 0.45}):
        ctx.expect_contact(
            weapon_cradle,
            turret_frame,
            elem_a=trunnion_shaft,
            elem_b=left_cheek,
            name="left_trunnion_stays_seated_when_elevated",
        )
        ctx.expect_contact(
            weapon_cradle,
            turret_frame,
            elem_a=trunnion_shaft,
            elem_b=right_cheek,
            name="right_trunnion_stays_seated_when_elevated",
        )

    with ctx.pose({azimuth: -0.50, sensor_tilt: 0.35}):
        ctx.expect_contact(
            sensor_pod,
            turret_frame,
            elem_a=sensor_hub,
            elem_b=sensor_clip_inner,
            name="sensor_hub_stays_in_inner_clip_when_tilted",
        )
        ctx.expect_contact(
            sensor_pod,
            turret_frame,
            elem_a=sensor_hub,
            elem_b=sensor_clip_outer,
            name="sensor_hub_stays_in_outer_clip_when_tilted",
        )

    ctx.warn_if_articulation_overlaps(max_pose_samples=20)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
