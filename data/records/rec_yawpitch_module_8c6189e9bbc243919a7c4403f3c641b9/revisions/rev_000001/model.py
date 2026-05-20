from __future__ import annotations

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
    model = ArticulatedObject(name="open_yaw_pitch_head")

    dark_anodized = model.material("dark_anodized_aluminum", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_aluminum = model.material("satin_machined_aluminum", rgba=(0.62, 0.64, 0.63, 1.0))
    black_hardware = model.material("black_oxide_hardware", rgba=(0.015, 0.014, 0.012, 1.0))
    bearing_steel = model.material("brushed_bearing_steel", rgba=(0.78, 0.76, 0.70, 1.0))

    lower = model.part("lower_support")
    lower.visual(
        Cylinder(radius=0.165, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_anodized,
        name="floor_plate",
    )
    lower.visual(
        Cylinder(radius=0.060, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=satin_aluminum,
        name="pedestal_column",
    )
    lower.visual(
        Cylinder(radius=0.094, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material=dark_anodized,
        name="bearing_collar",
    )
    for index, (x, y) in enumerate(
        (
            (0.112, 0.112),
            (-0.112, 0.112),
            (-0.112, -0.112),
            (0.112, -0.112),
        )
    ):
        lower.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, y, 0.029)),
            material=black_hardware,
            name=f"base_bolt_{index}",
        )

    yaw = model.part("yaw_stage")
    yaw.visual(
        Cylinder(radius=0.106, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_anodized,
        name="turntable",
    )
    yaw.visual(
        Box((0.040, 0.052, 0.020)),
        origin=Origin(xyz=(0.0, 0.115, 0.016)),
        material=black_hardware,
        name="front_index_tab",
    )
    yaw.visual(
        Cylinder(radius=0.042, length=0.146),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=satin_aluminum,
        name="riser_tube",
    )
    yaw.visual(
        Box((0.330, 0.064, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=dark_anodized,
        name="yoke_crosshead",
    )
    for side, x in (("neg", -0.155), ("pos", 0.155)):
        yaw.visual(
            Box((0.038, 0.064, 0.172)),
            origin=Origin(xyz=(x, 0.0, 0.264)),
            material=dark_anodized,
            name=f"yoke_cheek_{side}",
        )
        yaw.visual(
            Cylinder(radius=0.038, length=0.016),
            origin=Origin(xyz=(x * 0.8774193548, 0.0, 0.285), rpy=(0.0, pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"pitch_bearing_{side}",
        )

    cradle = model.part("pitch_cradle")
    cradle.visual(
        Box((0.214, 0.154, 0.022)),
        origin=Origin(xyz=(0.0, 0.020, -0.081)),
        material=dark_anodized,
        name="cradle_tray",
    )
    for side, x in (("neg", -0.092), ("pos", 0.092)):
        cradle.visual(
            Box((0.026, 0.154, 0.112)),
            origin=Origin(xyz=(x, 0.020, -0.033)),
            material=dark_anodized,
            name=f"cradle_side_{side}",
        )
        cradle.visual(
            Cylinder(radius=0.026, length=0.048),
            origin=Origin(xyz=(x * 1.1304347826, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=bearing_steel,
            name=f"trunnion_{side}",
        )
    cradle.visual(
        Box((0.166, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, -0.061, -0.044)),
        material=satin_aluminum,
        name="rear_tie_bar",
    )
    cradle.visual(
        Box((0.126, 0.018, 0.044)),
        origin=Origin(xyz=(0.0, 0.094, -0.051)),
        material=satin_aluminum,
        name="front_lip",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=yaw,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.2, lower=-pi, upper=pi),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=yaw,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.285)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=-0.95, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower = object_model.get_part("lower_support")
    yaw = object_model.get_part("yaw_stage")
    cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("yaw_axis")
    pitch_joint = object_model.get_articulation("pitch_axis")

    ctx.check(
        "three exposed structural stages",
        {part.name for part in object_model.parts} == {"lower_support", "yaw_stage", "pitch_cradle"},
        details="The pan-tilt head should be only the grounded lower support, yaw stage, and pitch cradle.",
    )
    ctx.check(
        "two revolute axes only",
        len(object_model.articulations) == 2
        and yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and pitch_joint.articulation_type == ArticulationType.REVOLUTE,
        details="The mechanism should contain exactly one yaw revolute and one pitch revolute.",
    )
    ctx.check(
        "yaw axis is vertical",
        tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"yaw axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch axis is horizontal",
        tuple(round(v, 6) for v in pitch_joint.axis) == (1.0, 0.0, 0.0),
        details=f"pitch axis={pitch_joint.axis}",
    )

    lower_aabb = ctx.part_world_aabb(lower)
    ctx.check(
        "lower support is grounded",
        lower_aabb is not None and abs(lower_aabb[0][2]) <= 0.001,
        details=f"lower_support aabb={lower_aabb}",
    )
    ctx.expect_gap(
        yaw,
        lower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="turntable",
        negative_elem="bearing_collar",
        name="yaw turntable seats on the lower bearing collar",
    )
    ctx.expect_within(
        cradle,
        yaw,
        axes="x",
        margin=0.002,
        inner_elem="cradle_tray",
        outer_elem="yoke_crosshead",
        name="pitch cradle fits between the yoke cheeks",
    )

    rest_tab = ctx.part_element_world_aabb(yaw, elem="front_index_tab")
    with ctx.pose({yaw_joint: 0.55}):
        turned_tab = ctx.part_element_world_aabb(yaw, elem="front_index_tab")
    rest_tab_center_x = (rest_tab[0][0] + rest_tab[1][0]) * 0.5 if rest_tab is not None else None
    turned_tab_center_x = (turned_tab[0][0] + turned_tab[1][0]) * 0.5 if turned_tab is not None else None
    ctx.check(
        "yaw stage visibly turns about the vertical axis",
        rest_tab_center_x is not None
        and turned_tab_center_x is not None
        and abs(turned_tab_center_x - rest_tab_center_x) > 0.045,
        details=f"front tab x at rest={rest_tab_center_x}, after yaw={turned_tab_center_x}",
    )

    rest_tray = ctx.part_element_world_aabb(cradle, elem="cradle_tray")
    with ctx.pose({pitch_joint: 0.55}):
        pitched_tray = ctx.part_element_world_aabb(cradle, elem="cradle_tray")
    rest_tray_center_z = (rest_tray[0][2] + rest_tray[1][2]) * 0.5 if rest_tray is not None else None
    pitched_tray_center_z = (pitched_tray[0][2] + pitched_tray[1][2]) * 0.5 if pitched_tray is not None else None
    ctx.check(
        "pitch cradle rotates about the horizontal axis",
        rest_tray_center_z is not None
        and pitched_tray_center_z is not None
        and pitched_tray_center_z > rest_tray_center_z + 0.015,
        details=f"tray z at rest={rest_tray_center_z}, after pitch={pitched_tray_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
