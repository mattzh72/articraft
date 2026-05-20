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
    model = ArticulatedObject(name="compact_three_joint_robot_arm")

    dark = model.material("dark_anodized", rgba=(0.05, 0.055, 0.06, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.14, 1.0))
    orange = model.material("safety_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    light_gray = model.material("powdercoat_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    blue = model.material("tool_plate_blue", rgba=(0.06, 0.24, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.135, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark,
        name="floor_flange",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.138),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material=charcoal,
        name="short_column",
    )
    base.visual(
        Box((0.130, 0.170, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=charcoal,
        name="shoulder_saddle",
    )
    for side, y in (("front", -0.080), ("rear", 0.080)):
        base.visual(
            Box((0.090, 0.028, 0.145)),
            origin=Origin(xyz=(0.0, y, 0.232)),
            material=charcoal,
            name=f"shoulder_cheek_{side}",
        )
        base.visual(
            Cylinder(radius=0.032, length=0.012),
            origin=Origin(xyz=(0.0, y * 1.15, 0.240), rpy=(-pi / 2, 0.0, 0.0)),
            material=steel,
            name=f"shoulder_pin_cap_{side}",
        )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        Cylinder(radius=0.065, length=0.132),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="shoulder_hub",
    )
    shoulder.visual(
        Box((0.285, 0.092, 0.078)),
        origin=Origin(xyz=(0.143, 0.0, 0.0)),
        material=orange,
        name="wide_link_body",
    )
    shoulder.visual(
        Box((0.075, 0.019, 0.036)),
        origin=Origin(xyz=(0.308, -0.055, 0.0)),
        material=orange,
        name="elbow_side_0",
    )
    shoulder.visual(
        Box((0.075, 0.019, 0.036)),
        origin=Origin(xyz=(0.308, 0.055, 0.0)),
        material=orange,
        name="elbow_side_1",
    )
    for i, y in enumerate((-0.058, 0.058)):
        shoulder.visual(
            Cylinder(radius=0.056, length=0.018),
            origin=Origin(xyz=(0.340, y, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
            material=steel,
            name=f"elbow_outer_boss_{i}",
        )
        shoulder.visual(
            Cylinder(radius=0.024, length=0.010),
            origin=Origin(xyz=(0.340, y * 1.17, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
            material=steel,
            name=f"elbow_pin_cap_{i}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.049, length=0.091),
        origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.235, 0.055, 0.052)),
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        material=light_gray,
        name="slim_link_body",
    )
    forearm.visual(
        Cylinder(radius=0.042, length=0.076),
        origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="wrist_hub",
    )
    forearm.visual(
        Box((0.075, 0.042, 0.030)),
        origin=Origin(xyz=(0.248, 0.0, 0.0)),
        material=light_gray,
        name="wrist_neck",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Box((0.085, 0.018, 0.092)),
        origin=Origin(xyz=(0.022, -0.047, 0.0)),
        material=charcoal,
        name="yoke_side_0",
    )
    wrist.visual(
        Box((0.085, 0.018, 0.092)),
        origin=Origin(xyz=(0.022, 0.047, 0.0)),
        material=charcoal,
        name="yoke_side_1",
    )
    wrist.visual(
        Box((0.038, 0.116, 0.022)),
        origin=Origin(xyz=(0.079, 0.0, 0.035)),
        material=charcoal,
        name="yoke_bridge",
    )
    for i, y in enumerate((-0.061, 0.061)):
        wrist.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
            material=steel,
            name=f"wrist_pin_cap_{i}",
        )
    wrist.visual(
        Box((0.058, 0.055, 0.055)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=charcoal,
        name="plate_neck",
    )
    wrist.visual(
        Box((0.018, 0.120, 0.120)),
        origin=Origin(xyz=(0.143, 0.0, 0.0)),
        material=blue,
        name="square_output_plate",
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.6, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "shoulder_to_forearm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forearm,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-1.35, upper=1.45),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.8, lower=-1.60, upper=1.60),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder_link")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forearm")
    wrist_joint = object_model.get_articulation("forearm_to_wrist")

    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.expect_overlap(
        base,
        shoulder,
        axes="xz",
        elem_a="shoulder_cheek_front",
        elem_b="shoulder_hub",
        min_overlap=0.040,
        name="shoulder hub sits between base cheeks",
    )
    ctx.expect_overlap(
        shoulder,
        forearm,
        axes="xz",
        elem_a="elbow_outer_boss_0",
        elem_b="elbow_hub",
        min_overlap=0.035,
        name="forearm hub is captured at elbow",
    )
    ctx.expect_overlap(
        forearm,
        wrist,
        axes="xz",
        elem_a="wrist_hub",
        elem_b="yoke_side_0",
        min_overlap=0.030,
        name="wrist yoke straddles forearm hub",
    )

    rest_plate = ctx.part_element_world_aabb(wrist, elem="square_output_plate")
    with ctx.pose({shoulder_joint: 0.45, elbow_joint: -0.60, wrist_joint: 0.75}):
        posed_plate = ctx.part_element_world_aabb(wrist, elem="square_output_plate")
    ctx.check(
        "serial joints move output plate",
        rest_plate is not None
        and posed_plate is not None
        and abs(posed_plate[0][2] - rest_plate[0][2]) > 0.020,
        details=f"rest={rest_plate}, posed={posed_plate}",
    )

    return ctx.report()


object_model = build_object_model()
