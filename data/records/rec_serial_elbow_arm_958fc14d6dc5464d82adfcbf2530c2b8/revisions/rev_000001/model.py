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
    model = ArticulatedObject(name="industrial_pick_place_arm")

    painted = Material("painted_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    dark = Material("dark_graphite", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.56, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.12, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        material=steel,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=steel,
        name="shoulder_bearing",
    )
    base.visual(
        Box((0.28, 0.12, 0.20)),
        origin=Origin(xyz=(-0.18, -0.22, 0.16)),
        material=dark,
        name="controller_box",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.145, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=painted,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Cylinder(radius=0.115, length=0.08),
        origin=Origin(xyz=(0.68, 0.0, 0.040)),
        material=painted,
        name="elbow_hub",
    )
    upper_arm.visual(
        Box((0.58, 0.05, 0.055)),
        origin=Origin(xyz=(0.36, 0.085, 0.045)),
        material=painted,
        name="upper_link_0",
    )
    upper_arm.visual(
        Box((0.58, 0.05, 0.055)),
        origin=Origin(xyz=(0.36, -0.085, 0.045)),
        material=painted,
        name="upper_link_1",
    )
    upper_arm.visual(
        Box((0.40, 0.22, 0.030)),
        origin=Origin(xyz=(0.36, 0.0, 0.074)),
        material=steel,
        name="top_tie_bar",
    )
    for i, (x, y) in enumerate(
        ((0.055, 0.055), (0.055, -0.055), (-0.055, 0.055), (-0.055, -0.055))
    ):
        upper_arm.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(x, y, 0.096)),
            material=steel,
            name=f"shoulder_bolt_{i}",
        )
    for i, y in enumerate((0.045, -0.045)):
        upper_arm.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(0.68, y, 0.075)),
            material=steel,
            name=f"elbow_bolt_{i}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.105, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=steel,
        name="elbow_cap",
    )
    forearm.visual(
        Box((0.34, 0.09, 0.052)),
        origin=Origin(xyz=(0.225, 0.0, 0.126)),
        material=painted,
        name="forearm_link",
    )
    forearm.visual(
        Cylinder(radius=0.075, length=0.065),
        origin=Origin(xyz=(0.43, 0.0, 0.126)),
        material=steel,
        name="tool_flange",
    )
    forearm.visual(
        Cylinder(radius=0.020, length=0.23),
        origin=Origin(xyz=(0.43, 0.0, -0.020)),
        material=steel,
        name="vacuum_tube",
    )
    forearm.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.43, 0.0, -0.150)),
        material=rubber,
        name="suction_cup",
    )
    for i, (x, y) in enumerate(((0.035, 0.035), (0.035, -0.035), (-0.035, 0.035), (-0.035, -0.035))):
        forearm.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(x, y, 0.165)),
            material=dark,
            name=f"cap_bolt_{i}",
        )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.6, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "elbow_yaw",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.68, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=2.0, lower=-2.2, upper=2.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    shoulder = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow_yaw")

    revolute_count = sum(
        1
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check("two primary revolute joints", revolute_count == 2, details=f"found {revolute_count}")

    with ctx.pose({shoulder: 0.0, elbow: 0.0}):
        ctx.expect_gap(
            upper_arm,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="shoulder_hub",
            negative_elem="shoulder_bearing",
            name="shoulder hub sits on bearing",
        )
        ctx.expect_overlap(
            upper_arm,
            base,
            axes="xy",
            min_overlap=0.20,
            elem_a="shoulder_hub",
            elem_b="shoulder_bearing",
            name="shoulder hub centered over base bearing",
        )
        ctx.expect_gap(
            forearm,
            upper_arm,
            axis="z",
            max_gap=0.008,
            max_penetration=0.0,
            positive_elem="elbow_cap",
            negative_elem="elbow_hub",
            name="forearm elbow cap clears upper hub",
        )
        ctx.expect_overlap(
            forearm,
            upper_arm,
            axes="xy",
            min_overlap=0.16,
            elem_a="elbow_cap",
            elem_b="elbow_hub",
            name="elbow hubs share a vertical axis",
        )

        cup_aabb = ctx.part_element_world_aabb(forearm, elem="suction_cup")
        rest_cup = None if cup_aabb is None else tuple((cup_aabb[0][i] + cup_aabb[1][i]) * 0.5 for i in range(3))

    with ctx.pose({shoulder: 0.9, elbow: 0.0}):
        cup_aabb = ctx.part_element_world_aabb(forearm, elem="suction_cup")
        swept_cup = None if cup_aabb is None else tuple((cup_aabb[0][i] + cup_aabb[1][i]) * 0.5 for i in range(3))
    ctx.check(
        "shoulder yaw sweeps the tool laterally",
        rest_cup is not None and swept_cup is not None and swept_cup[1] > rest_cup[1] + 0.35,
        details=f"rest={rest_cup}, swept={swept_cup}",
    )

    with ctx.pose({shoulder: 0.0, elbow: math.radians(70.0)}):
        cup_aabb = ctx.part_element_world_aabb(forearm, elem="suction_cup")
        bent_cup = None if cup_aabb is None else tuple((cup_aabb[0][i] + cup_aabb[1][i]) * 0.5 for i in range(3))
    ctx.check(
        "elbow yaw bends the short forearm",
        rest_cup is not None and bent_cup is not None and bent_cup[1] > rest_cup[1] + 0.30,
        details=f"rest={rest_cup}, bent={bent_cup}",
    )

    return ctx.report()


object_model = build_object_model()
