from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small helper for compact cast/painted robotic housings."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_three_joint_pick_arm")

    painted_blue = model.material("painted_blue", rgba=(0.08, 0.28, 0.72, 1.0))
    dark = model.material("matte_graphite", rgba=(0.025, 0.028, 0.032, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    safety_orange = model.material("safety_orange", rgba=(1.0, 0.45, 0.05, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.24, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.11, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=painted_blue,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.145, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=steel,
        name="top_bearing",
    )
    for idx, (x, y) in enumerate(
        ((0.17, 0.10), (-0.17, 0.10), (0.17, -0.10), (-0.17, -0.10))
    ):
        pedestal.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, y, 0.064)),
            material=steel,
            name=f"base_bolt_{idx}",
        )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.135, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel,
        name="turntable",
    )
    shoulder.visual(
        mesh_from_cadquery(_rounded_box((0.26, 0.20, 0.20), 0.025), "shoulder_gearbox"),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=painted_blue,
        name="gearbox",
    )
    shoulder.visual(
        Cylinder(radius=0.075, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.16), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="side_bearing_band",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_rounded_box((0.35, 0.105, 0.085), 0.018), "upper_beam"),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        material=painted_blue,
        name="upper_beam",
    )
    upper_arm.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shoulder_collar",
    )
    for idx, y in enumerate((-0.0575, 0.0575)):
        upper_arm.visual(
            mesh_from_cadquery(_rounded_box((0.150, 0.040, 0.150), 0.012), f"elbow_yoke_{idx}"),
            origin=Origin(xyz=(0.420, y, 0.0)),
            material=painted_blue,
            name=f"elbow_yoke_{idx}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.057, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_hub",
    )
    forearm.visual(
        mesh_from_cadquery(_rounded_box((0.275, 0.075, 0.075), 0.015), "forearm_beam"),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=painted_blue,
        name="forearm_beam",
    )
    forearm.visual(
        Cylinder(radius=0.053, length=0.060),
        origin=Origin(xyz=(0.330, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wrist_socket",
    )

    wrist_head = model.part("wrist_head")
    wrist_head.visual(
        Cylinder(radius=0.055, length=0.095),
        origin=Origin(xyz=(0.0475, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="roll_body",
    )
    wrist_head.visual(
        Cylinder(radius=0.070, length=0.022),
        origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_orange,
        name="tool_flange",
    )
    wrist_head.visual(
        Box((0.045, 0.026, 0.026)),
        origin=Origin(xyz=(0.045, 0.0, 0.058)),
        material=steel,
        name="roll_marker",
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "shoulder_mount",
        ArticulationType.FIXED,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0.13, 0.0, 0.16)),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-1.15, upper=1.35),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_head,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=3.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    shoulder = object_model.get_part("shoulder")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_head = object_model.get_part("wrist_head")

    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow")
    wrist_roll = object_model.get_articulation("wrist_roll")

    actuated = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "exactly three powered joints",
        len(actuated) == 3,
        details=f"powered={[joint.name for joint in actuated]}",
    )
    ctx.check("shoulder yaw axis is vertical", tuple(shoulder_yaw.axis) == (0.0, 0.0, 1.0))
    ctx.check("elbow axis is horizontal", abs(elbow.axis[1]) == 1.0 and elbow.axis[0] == 0.0)
    ctx.check("wrist rolls along forearm", tuple(wrist_roll.axis) == (1.0, 0.0, 0.0))

    ctx.expect_gap(
        shoulder,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="turntable",
        negative_elem="top_bearing",
        name="turntable is seated on pedestal bearing",
    )
    ctx.expect_contact(
        upper_arm,
        shoulder,
        elem_a="upper_beam",
        elem_b="gearbox",
        contact_tol=0.003,
        name="upper arm mounts tightly to shoulder gearbox",
    )
    ctx.expect_gap(
        wrist_head,
        forearm,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="roll_body",
        negative_elem="wrist_socket",
        name="wrist roll body is seated in the forearm socket",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    wrist_rest = aabb_center(ctx.part_world_aabb(wrist_head))
    with ctx.pose({shoulder_yaw: 0.90}):
        wrist_yawed = aabb_center(ctx.part_world_aabb(wrist_head))
    ctx.check(
        "shoulder yaw sweeps arm around pedestal",
        wrist_rest is not None
        and wrist_yawed is not None
        and wrist_yawed[1] > wrist_rest[1] + 0.25,
        details=f"rest={wrist_rest}, yawed={wrist_yawed}",
    )

    with ctx.pose({elbow: 0.80}):
        wrist_raised = aabb_center(ctx.part_world_aabb(wrist_head))
    ctx.check(
        "positive elbow motion lifts the forearm",
        wrist_rest is not None
        and wrist_raised is not None
        and wrist_raised[2] > wrist_rest[2] + 0.18,
        details=f"rest={wrist_rest}, raised={wrist_raised}",
    )

    marker_rest = aabb_center(ctx.part_element_world_aabb(wrist_head, elem="roll_marker"))
    with ctx.pose({wrist_roll: math.pi / 2.0}):
        marker_rolled = aabb_center(ctx.part_element_world_aabb(wrist_head, elem="roll_marker"))
    ctx.check(
        "wrist roll turns the keyed head feature",
        marker_rest is not None
        and marker_rolled is not None
        and abs(marker_rolled[1] - marker_rest[1]) > 0.04,
        details=f"rest={marker_rest}, rolled={marker_rolled}",
    )

    return ctx.report()


object_model = build_object_model()
