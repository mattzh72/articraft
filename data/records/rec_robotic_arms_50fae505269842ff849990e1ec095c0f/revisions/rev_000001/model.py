from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_joint_pick_place_arm")

    dark = Material("dark_powdercoat", color=(0.05, 0.055, 0.06, 1.0))
    graphite = Material("graphite_joint", color=(0.18, 0.19, 0.20, 1.0))
    light = Material("painted_light_grey", color=(0.78, 0.80, 0.78, 1.0))
    orange = Material("safety_orange", color=(1.0, 0.42, 0.08, 1.0))
    black = Material("rubber_black", color=(0.01, 0.01, 0.012, 1.0))

    cyl_x = (0.0, math.pi / 2.0, 0.0)
    cyl_y = (math.pi / 2.0, 0.0, 0.0)

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark,
        name="floor_base",
    )
    pedestal.visual(
        Cylinder(radius=0.13, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=light,
        name="base_column",
    )
    pedestal.visual(
        Cylinder(radius=0.20, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=graphite,
        name="top_bearing",
    )

    shoulder_housing = model.part("shoulder_housing")
    shoulder_housing.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=graphite,
        name="turntable",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.11, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=light,
        name="rotary_column",
    )
    shoulder_housing.visual(
        Box((0.24, 0.22, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=light,
        name="motor_housing",
    )
    shoulder_housing.visual(
        Cylinder(radius=0.07, length=0.08),
        origin=Origin(xyz=(0.08, 0.0, 0.31), rpy=cyl_x),
        material=graphite,
        name="shoulder_boss",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.06, 0.18, 0.16)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        material=graphite,
        name="root_mount",
    )
    upper_arm.visual(
        Box((0.61, 0.035, 0.055)),
        origin=Origin(xyz=(0.36, 0.09, 0.0)),
        material=orange,
        name="rail_0",
    )
    upper_arm.visual(
        Box((0.61, 0.035, 0.055)),
        origin=Origin(xyz=(0.36, -0.09, 0.0)),
        material=orange,
        name="rail_1",
    )
    upper_arm.visual(
        Box((0.08, 0.18, 0.11)),
        origin=Origin(xyz=(0.60, 0.0, 0.0)),
        material=graphite,
        name="elbow_spacer",
    )
    upper_arm.visual(
        Cylinder(radius=0.085, length=0.055),
        origin=Origin(xyz=(0.74, 0.0875, 0.0), rpy=cyl_y),
        material=graphite,
        name="elbow_lug_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.085, length=0.055),
        origin=Origin(xyz=(0.74, -0.0875, 0.0), rpy=cyl_y),
        material=graphite,
        name="elbow_lug_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.075, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y),
        material=graphite,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.48, 0.07, 0.06)),
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        material=light,
        name="forearm_beam",
    )
    forearm.visual(
        Cylinder(radius=0.062, length=0.10),
        origin=Origin(xyz=(0.53, 0.0, 0.0), rpy=cyl_x),
        material=graphite,
        name="wrist_socket",
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.065, length=0.14),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=cyl_x),
        material=graphite,
        name="roll_body",
    )
    wrist_head.visual(
        Box((0.06, 0.035, 0.025)),
        origin=Origin(xyz=(0.075, 0.0, 0.073)),
        material=orange,
        name="index_tab",
    )
    wrist_head.visual(
        Cylinder(radius=0.085, length=0.03),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=cyl_x),
        material=light,
        name="tool_flange",
    )
    wrist_head.visual(
        Cylinder(radius=0.045, length=0.06),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=cyl_x),
        material=black,
        name="suction_cup",
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=shoulder_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.8, lower=-2.95, upper=2.95),
    )
    model.articulation(
        "upper_arm_mount",
        ArticulationType.FIXED,
        parent=shoulder_housing,
        child=upper_arm,
        origin=Origin(xyz=(0.12, 0.0, 0.31)),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.74, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-1.1, upper=1.2),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    shoulder_housing = object_model.get_part("shoulder_housing")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")
    shoulder = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist_roll")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if getattr(joint, "articulation_type", None) == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "three driven revolute joints",
        len(revolute_joints) == 3,
        details=f"found {[joint.name for joint in revolute_joints]}",
    )
    ctx.check(
        "joint axes match requested mechanism",
        tuple(shoulder.axis) == (0.0, 0.0, 1.0)
        and tuple(elbow.axis) == (0.0, 1.0, 0.0)
        and tuple(wrist.axis) == (1.0, 0.0, 0.0),
        details=f"axes shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}",
    )

    ctx.expect_gap(
        shoulder_housing,
        pedestal,
        axis="z",
        positive_elem="turntable",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.000001,
        name="shoulder turntable sits on pedestal bearing",
    )
    ctx.expect_gap(
        upper_arm,
        shoulder_housing,
        axis="x",
        positive_elem="root_mount",
        negative_elem="motor_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper arm bolts to shoulder housing face",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem="elbow_lug_0",
        negative_elem="elbow_hub",
        max_gap=0.002,
        max_penetration=0.0,
        name="forearm hub seats against positive elbow lug",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_lug_1",
        max_gap=0.002,
        max_penetration=0.0,
        name="forearm hub seats against negative elbow lug",
    )
    ctx.expect_gap(
        wrist_head,
        forearm,
        axis="x",
        positive_elem="roll_body",
        negative_elem="wrist_socket",
        max_gap=0.001,
        max_penetration=0.000001,
        name="wrist roll body meets forearm socket",
    )
    ctx.expect_origin_gap(
        forearm,
        upper_arm,
        axis="x",
        min_gap=0.70,
        max_gap=0.78,
        name="upper arm is a long slender span to the elbow",
    )
    ctx.expect_origin_gap(
        wrist_head,
        forearm,
        axis="x",
        min_gap=0.54,
        max_gap=0.62,
        name="forearm keeps wrist clearly separated from elbow",
    )

    rest_wrist_position = ctx.part_world_position(wrist_head)
    with ctx.pose({shoulder: 1.0}):
        yawed_wrist_position = ctx.part_world_position(wrist_head)
    ctx.check(
        "shoulder yaw swings the whole chain about the pedestal",
        rest_wrist_position is not None
        and yawed_wrist_position is not None
        and yawed_wrist_position[1] > rest_wrist_position[1] + 0.55,
        details=f"rest={rest_wrist_position}, yawed={yawed_wrist_position}",
    )

    with ctx.pose({elbow: 0.8}):
        bent_wrist_position = ctx.part_world_position(wrist_head)
    ctx.check(
        "elbow bends forearm about a horizontal axis",
        rest_wrist_position is not None
        and bent_wrist_position is not None
        and bent_wrist_position[2] < rest_wrist_position[2] - 0.25,
        details=f"rest={rest_wrist_position}, bent={bent_wrist_position}",
    )

    rest_tab_aabb = ctx.part_element_world_aabb(wrist_head, elem="index_tab")
    with ctx.pose({wrist: 1.1}):
        rolled_tab_aabb = ctx.part_element_world_aabb(wrist_head, elem="index_tab")
    if rest_tab_aabb is not None and rolled_tab_aabb is not None:
        rest_tab_y = (rest_tab_aabb[0][1] + rest_tab_aabb[1][1]) / 2.0
        rolled_tab_y = (rolled_tab_aabb[0][1] + rolled_tab_aabb[1][1]) / 2.0
    else:
        rest_tab_y = None
        rolled_tab_y = None
    ctx.check(
        "wrist roll rotates the indexed wrist head about the forearm axis",
        rest_tab_y is not None
        and rolled_tab_y is not None
        and rolled_tab_y < rest_tab_y - 0.04,
        details=f"rest_tab_y={rest_tab_y}, rolled_tab_y={rolled_tab_y}",
    )

    return ctx.report()


object_model = build_object_model()
