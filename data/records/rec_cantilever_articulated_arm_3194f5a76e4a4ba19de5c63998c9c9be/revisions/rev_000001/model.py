from __future__ import annotations

import math

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
    model = ArticulatedObject(name="overhung_box_beam_mechanical_arm")

    yellow = model.material("industrial_yellow", rgba=(0.96, 0.68, 0.12, 1.0))
    dark = model.material("matte_dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    safety = model.material("safety_label_blue", rgba=(0.08, 0.22, 0.75, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.34, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.155, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=yellow,
        name="fixed_column",
    )
    pedestal.visual(
        Cylinder(radius=0.235, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=steel,
        name="top_bearing",
    )
    for i, (x, y) in enumerate(((0.23, 0.23), (-0.23, 0.23), (-0.23, -0.23), (0.23, -0.23))):
        pedestal.visual(
            Cylinder(radius=0.026, length=0.024),
            origin=Origin(xyz=(x, y, 0.086)),
            material=steel,
            name=f"anchor_bolt_{i}",
        )
    pedestal.visual(
        Box((0.39, 0.055, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=yellow,
        name="front_gusset",
    )
    pedestal.visual(
        Box((0.055, 0.39, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=yellow,
        name="side_gusset",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.215, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark,
        name="turntable",
    )
    turret.visual(
        Box((0.14, 0.18, 0.20)),
        origin=Origin(xyz=(0.04, 0.0, 0.15)),
        material=yellow,
        name="shoulder_stand",
    )
    turret.visual(
        Box((0.16, 0.25, 0.055)),
        origin=Origin(xyz=(0.145, 0.0, 0.117)),
        material=yellow,
        name="lower_bridge",
    )
    for side, y in enumerate((-0.15, 0.15)):
        turret.visual(
            Box((0.12, 0.055, 0.28)),
            origin=Origin(xyz=(0.23, y, 0.25)),
            material=yellow,
            name=f"shoulder_cheek_{side}",
        )
    turret.visual(
        Box((0.13, 0.36, 0.04)),
        origin=Origin(xyz=(0.23, 0.0, 0.395)),
        material=yellow,
        name="top_bridge",
    )
    turret.visual(
        Cylinder(radius=0.13, length=0.07),
        origin=Origin(xyz=(0.23, 0.215, 0.25), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="shoulder_motor",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.10, length=0.245),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.575, 0.10, 0.09)),
        origin=Origin(xyz=(0.3225, 0.0, 0.0)),
        material=yellow,
        name="upper_box_beam",
    )
    upper_arm.visual(
        Box((0.48, 0.012, 0.052)),
        origin=Origin(xyz=(0.31, 0.057, 0.0)),
        material=safety,
        name="upper_side_panel",
    )
    upper_arm.visual(
        Box((0.48, 0.012, 0.052)),
        origin=Origin(xyz=(0.31, -0.057, 0.0)),
        material=safety,
        name="upper_side_panel_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.112, length=0.26),
        origin=Origin(xyz=(0.72, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="elbow_housing",
    )
    upper_arm.visual(
        Cylinder(radius=0.123, length=0.055),
        origin=Origin(xyz=(0.72, -0.152, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="elbow_motor_cap",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.085, length=0.18),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.51, 0.085, 0.075)),
        origin=Origin(xyz=(0.285, 0.0, 0.0)),
        material=yellow,
        name="forearm_box_beam",
    )
    forearm.visual(
        Box((0.38, 0.011, 0.043)),
        origin=Origin(xyz=(0.34, 0.046, 0.0)),
        material=safety,
        name="forearm_side_panel",
    )
    forearm.visual(
        Box((0.38, 0.011, 0.043)),
        origin=Origin(xyz=(0.34, -0.046, 0.0)),
        material=safety,
        name="forearm_side_panel_1",
    )
    forearm.visual(
        Cylinder(radius=0.082, length=0.205),
        origin=Origin(xyz=(0.62, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="wrist_housing",
    )
    forearm.visual(
        Box((0.10, 0.062, 0.052)),
        origin=Origin(xyz=(0.50, 0.0, 0.066)),
        material=dark,
        name="wrist_drive_box",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.067, length=0.15),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wrist_hub",
    )
    wrist.visual(
        Box((0.22, 0.065, 0.065)),
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
        material=yellow,
        name="wrist_neck",
    )
    wrist.visual(
        Box((0.12, 0.075, 0.07)),
        origin=Origin(xyz=(0.15, 0.0, 0.065)),
        material=dark,
        name="wrist_motor_box",
    )
    wrist.visual(
        Cylinder(radius=0.062, length=0.04),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="roll_bearing",
    )

    flange = model.part("flange")
    flange.visual(
        Cylinder(radius=0.065, length=0.04),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="flange_disk",
    )
    flange.visual(
        Box((0.018, 0.16, 0.16)),
        origin=Origin(xyz=(0.049, 0.0, 0.0)),
        material=steel,
        name="tool_plate",
    )
    for i, (y, z) in enumerate(((0.052, 0.052), (-0.052, 0.052), (-0.052, -0.052), (0.052, -0.052))):
        flange.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.062, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"tool_bolt_{i}",
        )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper_arm,
        origin=Origin(xyz=(0.23, 0.0, 0.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.8, lower=-0.9, upper=1.2),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.72, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=1.0, lower=-2.2, upper=1.0),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.62, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.6, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "tool_roll",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=flange,
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    pedestal = object_model.get_part("pedestal")
    turret = object_model.get_part("turret")
    flange = object_model.get_part("flange")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")

    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_housing",
        elem_b="elbow_hub",
        reason="The elbow hub is intentionally seated inside the compact rotary actuator housing.",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="yz",
        inner_elem="elbow_hub",
        outer_elem="elbow_housing",
        margin=0.002,
        name="elbow hub is captured inside actuator housing",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="x",
        elem_a="elbow_hub",
        elem_b="elbow_housing",
        min_overlap=0.12,
        name="elbow hub has retained axial engagement",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_housing",
        elem_b="forearm_box_beam",
        reason="The boxed forearm root passes through a simplified uncut slot in the compact elbow housing.",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="x",
        positive_elem="forearm_box_beam",
        negative_elem="elbow_housing",
        max_penetration=0.09,
        name="forearm beam root only enters the elbow housing locally",
    )

    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_housing",
        elem_b="wrist_hub",
        reason="The wrist hub is intentionally nested inside the small wrist pitch bearing housing.",
    )
    ctx.expect_within(
        wrist,
        forearm,
        axes="yz",
        inner_elem="wrist_hub",
        outer_elem="wrist_housing",
        margin=0.002,
        name="wrist hub is captured inside wrist housing",
    )
    ctx.expect_overlap(
        wrist,
        forearm,
        axes="x",
        elem_a="wrist_hub",
        elem_b="wrist_housing",
        min_overlap=0.09,
        name="wrist hub has retained axial engagement",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_housing",
        elem_b="wrist_neck",
        reason="The wrist neck emerges through a simplified uncut slot in the compact wrist pitch housing.",
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="x",
        positive_elem="wrist_neck",
        negative_elem="wrist_housing",
        max_penetration=0.09,
        name="wrist neck only enters the wrist housing locally",
    )

    ctx.expect_contact(
        turret,
        pedestal,
        elem_a="turntable",
        elem_b="top_bearing",
        contact_tol=0.001,
        name="turntable sits on fixed pedestal bearing",
    )

    joints = [base_yaw, shoulder_pitch, elbow_pitch, object_model.get_articulation("wrist_pitch"), object_model.get_articulation("tool_roll")]
    ctx.check(
        "arm uses five rotary joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joint types={[j.articulation_type for j in joints]}",
    )

    rest_aabb = ctx.part_world_aabb(flange)
    ctx.check(
        "rest pose reaches well beyond pedestal",
        rest_aabb is not None and rest_aabb[1][0] > 1.85 and rest_aabb[0][2] > 0.60,
        details=f"flange_aabb={rest_aabb}",
    )

    rest_pos = ctx.part_world_position(flange)
    with ctx.pose({base_yaw: math.pi / 2.0}):
        yawed_pos = ctx.part_world_position(flange)
    ctx.check(
        "base yaw swings overhung reach sideways",
        rest_pos is not None
        and yawed_pos is not None
        and abs(yawed_pos[1] - rest_pos[0]) < 0.08
        and abs(yawed_pos[0]) < 0.12,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    shoulder_rest = ctx.part_world_aabb(flange)
    with ctx.pose({shoulder_pitch: 0.8, elbow_pitch: -0.6}):
        raised = ctx.part_world_aabb(flange)
    ctx.check(
        "pitch joints lift the outer wrist",
        shoulder_rest is not None and raised is not None and raised[0][2] > shoulder_rest[0][2] + 0.20,
        details=f"rest={shoulder_rest}, raised={raised}",
    )

    return ctx.report()


object_model = build_object_model()
