from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PI = 3.141592653589793


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")

    armor_green = Material("mat_armor_green", rgba=(0.18, 0.23, 0.18, 1.0))
    dark_armor = Material("mat_dark_armor", rgba=(0.07, 0.08, 0.075, 1.0))
    gunmetal = Material("mat_gunmetal", rgba=(0.03, 0.034, 0.034, 1.0))
    rubber_black = Material("mat_rubber_black", rgba=(0.005, 0.006, 0.006, 1.0))
    optic_glass = Material("mat_blue_black_glass", rgba=(0.02, 0.06, 0.09, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.36, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_armor,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.20, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=armor_green,
        name="fixed_column",
    )
    pedestal.visual(
        Cylinder(radius=0.25, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_armor,
        name="lower_bearing_race",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.24, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_armor,
        name="yaw_bearing",
    )
    turret.visual(
        Box((0.78, 0.56, 0.08)),
        origin=Origin(xyz=(0.04, 0.0, 0.10)),
        material=armor_green,
        name="turret_deck",
    )
    turret.visual(
        Box((0.22, 0.22, 0.08)),
        origin=Origin(xyz=(-0.19, 0.39, 0.10)),
        material=armor_green,
        name="sensor_plinth",
    )
    turret.visual(
        Box((0.24, 0.36, 0.12)),
        origin=Origin(xyz=(-0.18, 0.0, 0.20)),
        material=armor_green,
        name="electronics_hump",
    )
    turret.visual(
        Box((0.42, 0.05, 0.32)),
        origin=Origin(xyz=(0.03, 0.23, 0.28)),
        material=armor_green,
        name="trunnion_cheek_0",
    )
    turret.visual(
        Cylinder(radius=0.075, length=0.05),
        origin=Origin(xyz=(0.05, 0.28, 0.28), rpy=(PI / 2, 0.0, 0.0)),
        material=dark_armor,
        name="outer_bearing_boss_0",
    )
    turret.visual(
        Box((0.42, 0.05, 0.32)),
        origin=Origin(xyz=(0.03, -0.23, 0.28)),
        material=armor_green,
        name="trunnion_cheek_1",
    )
    turret.visual(
        Cylinder(radius=0.075, length=0.05),
        origin=Origin(xyz=(0.05, -0.28, 0.28), rpy=(PI / 2, 0.0, 0.0)),
        material=dark_armor,
        name="outer_bearing_boss_1",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.2, lower=-PI, upper=PI),
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        Cylinder(radius=0.052, length=0.41),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(PI / 2, 0.0, 0.0)),
        material=dark_armor,
        name="trunnion_bar",
    )
    weapon_cradle.visual(
        Box((0.42, 0.16, 0.16)),
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=armor_green,
        name="receiver",
    )
    weapon_cradle.visual(
        Box((0.30, 0.14, 0.04)),
        origin=Origin(xyz=(0.26, 0.0, 0.10)),
        material=dark_armor,
        name="top_cover",
    )
    weapon_cradle.visual(
        Box((0.32, 0.10, 0.10)),
        origin=Origin(xyz=(0.52, 0.0, 0.02)),
        material=dark_armor,
        name="barrel_shroud",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.032, length=0.78),
        origin=Origin(xyz=(0.78, 0.0, 0.02), rpy=(0.0, PI / 2, 0.0)),
        material=gunmetal,
        name="barrel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.052, length=0.10),
        origin=Origin(xyz=(1.22, 0.0, 0.02), rpy=(0.0, PI / 2, 0.0)),
        material=gunmetal,
        name="muzzle_brake",
    )
    weapon_cradle.visual(
        Box((0.20, 0.12, 0.14)),
        origin=Origin(xyz=(0.18, -0.13, -0.05)),
        material=dark_armor,
        name="ammo_box",
    )
    weapon_cradle.visual(
        Box((0.18, 0.04, 0.05)),
        origin=Origin(xyz=(0.22, -0.075, -0.03)),
        material=dark_armor,
        name="feed_chute",
    )

    model.articulation(
        "weapon_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=weapon_cradle,
        origin=Origin(xyz=(0.05, 0.0, 0.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.9, lower=-0.25, upper=0.85),
    )

    sensor_mast = model.part("sensor_mast")
    sensor_mast.visual(
        Box((0.22, 0.16, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_armor,
        name="mast_foot",
    )
    sensor_mast.visual(
        Cylinder(radius=0.025, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=armor_green,
        name="vertical_post",
    )
    sensor_mast.visual(
        Box((0.25, 0.05, 0.05)),
        origin=Origin(xyz=(0.125, 0.0, 0.34)),
        material=armor_green,
        name="bracket_arm",
    )
    sensor_mast.visual(
        Box((0.04, 0.16, 0.06)),
        origin=Origin(xyz=(0.245, 0.0, 0.34)),
        material=armor_green,
        name="fork_bridge",
    )
    sensor_mast.visual(
        Box((0.09, 0.02, 0.11)),
        origin=Origin(xyz=(0.30, 0.07, 0.34)),
        material=armor_green,
        name="sensor_cheek_0",
    )
    sensor_mast.visual(
        Box((0.09, 0.02, 0.11)),
        origin=Origin(xyz=(0.30, -0.07, 0.34)),
        material=armor_green,
        name="sensor_cheek_1",
    )

    model.articulation(
        "mast_mount",
        ArticulationType.FIXED,
        parent=turret,
        child=sensor_mast,
        origin=Origin(xyz=(-0.19, 0.39, 0.14)),
    )

    sensor_pod = model.part("sensor_pod")
    sensor_pod.visual(
        Cylinder(radius=0.022, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(PI / 2, 0.0, 0.0)),
        material=dark_armor,
        name="tilt_pin",
    )
    sensor_pod.visual(
        Box((0.22, 0.10, 0.11)),
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
        material=dark_armor,
        name="sensor_housing",
    )
    sensor_pod.visual(
        Box((0.012, 0.072, 0.065)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=optic_glass,
        name="optical_window",
    )
    sensor_pod.visual(
        Box((0.15, 0.08, 0.025)),
        origin=Origin(xyz=(0.125, 0.0, 0.0675)),
        material=rubber_black,
        name="sunshade_cap",
    )

    model.articulation(
        "sensor_tilt",
        ArticulationType.REVOLUTE,
        parent=sensor_mast,
        child=sensor_pod,
        origin=Origin(xyz=(0.30, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-0.45, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turret = object_model.get_part("turret")
    weapon_cradle = object_model.get_part("weapon_cradle")
    sensor_mast = object_model.get_part("sensor_mast")
    sensor_pod = object_model.get_part("sensor_pod")
    yaw = object_model.get_articulation("base_yaw")
    pitch = object_model.get_articulation("weapon_pitch")
    sensor_tilt = object_model.get_articulation("sensor_tilt")

    ctx.expect_gap(
        turret,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="yaw_bearing",
        negative_elem="lower_bearing_race",
        name="rotating base sits on lower bearing",
    )
    ctx.expect_gap(
        sensor_mast,
        turret,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="mast_foot",
        negative_elem="sensor_plinth",
        name="sensor mast foot is mounted on turret side deck",
    )
    ctx.expect_contact(
        weapon_cradle,
        turret,
        elem_a="trunnion_bar",
        elem_b="trunnion_cheek_0",
        name="weapon trunnion reaches one cradle cheek",
    )
    ctx.expect_contact(
        weapon_cradle,
        turret,
        elem_a="trunnion_bar",
        elem_b="trunnion_cheek_1",
        name="weapon trunnion reaches opposite cradle cheek",
    )
    ctx.expect_contact(
        sensor_pod,
        sensor_mast,
        elem_a="tilt_pin",
        elem_b="sensor_cheek_0",
        name="sensor tilt pin reaches one bracket cheek",
    )
    ctx.expect_contact(
        sensor_pod,
        sensor_mast,
        elem_a="tilt_pin",
        elem_b="sensor_cheek_1",
        name="sensor tilt pin reaches opposite bracket cheek",
    )

    ctx.check(
        "yaw joint is vertical revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE and yaw.axis == (0.0, 0.0, 1.0),
        details=f"type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "weapon pitch is transverse",
        pitch.parent == "turret"
        and pitch.child == "weapon_cradle"
        and pitch.axis == (0.0, -1.0, 0.0),
        details=f"parent={pitch.parent}, child={pitch.child}, axis={pitch.axis}",
    )
    ctx.check(
        "sensor tilts from mast bracket",
        sensor_tilt.parent == "sensor_mast"
        and sensor_tilt.child == "sensor_pod"
        and sensor_tilt.axis == (0.0, -1.0, 0.0),
        details=f"parent={sensor_tilt.parent}, child={sensor_tilt.child}, axis={sensor_tilt.axis}",
    )

    rest_muzzle = ctx.part_element_world_aabb(weapon_cradle, elem="muzzle_brake")
    with ctx.pose({pitch: 0.60}):
        raised_muzzle = ctx.part_element_world_aabb(weapon_cradle, elem="muzzle_brake")
    ctx.check(
        "positive weapon pitch elevates muzzle",
        rest_muzzle is not None
        and raised_muzzle is not None
        and raised_muzzle[0][2] > rest_muzzle[0][2] + 0.18,
        details=f"rest={rest_muzzle}, raised={raised_muzzle}",
    )

    rest_window = ctx.part_element_world_aabb(sensor_pod, elem="optical_window")
    with ctx.pose({sensor_tilt: 0.45}):
        tilted_window = ctx.part_element_world_aabb(sensor_pod, elem="optical_window")
    ctx.check(
        "positive sensor tilt raises optic window",
        rest_window is not None
        and tilted_window is not None
        and tilted_window[0][2] > rest_window[0][2] + 0.05,
        details=f"rest={rest_window}, tilted={tilted_window}",
    )

    return ctx.report()


object_model = build_object_model()
