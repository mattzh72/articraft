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
    model = ArticulatedObject(name="wall_backed_three_joint_arm")

    powder_coat = Material("powder_coat", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    arm_finish = Material("arm_finish", rgba=(0.92, 0.48, 0.12, 1.0))
    worn_edge = Material("worn_edge", rgba=(0.20, 0.21, 0.22, 1.0))
    fastener = Material("fastener", rgba=(0.015, 0.015, 0.017, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.035, 0.320, 0.480)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0)),
        material=powder_coat,
        name="wall_plate",
    )
    backplate.visual(
        Box((0.026, 0.120, 0.405)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=dark_steel,
        name="center_spine",
    )
    backplate.visual(
        Box((0.020, 0.260, 0.040)),
        origin=Origin(xyz=(0.012, 0.0, 0.165)),
        material=dark_steel,
        name="upper_cross_rib",
    )
    backplate.visual(
        Box((0.020, 0.260, 0.040)),
        origin=Origin(xyz=(0.012, 0.0, -0.165)),
        material=dark_steel,
        name="lower_cross_rib",
    )
    backplate.visual(
        Box((0.180, 0.175, 0.070)),
        origin=Origin(xyz=(0.090, 0.0, -0.015)),
        material=dark_steel,
        name="shoulder_block",
    )
    backplate.visual(
        Box((0.115, 0.025, 0.135)),
        origin=Origin(xyz=(0.066, 0.0875, 0.020)),
        material=dark_steel,
        name="upper_side_gusset",
    )
    backplate.visual(
        Box((0.115, 0.025, 0.135)),
        origin=Origin(xyz=(0.066, -0.0875, 0.020)),
        material=dark_steel,
        name="lower_side_gusset",
    )
    backplate.visual(
        Cylinder(radius=0.083, length=0.052),
        origin=Origin(xyz=(0.180, 0.0, 0.010)),
        material=worn_edge,
        name="shoulder_bearing",
    )
    for i, (y, z) in enumerate(
        ((0.118, 0.178), (-0.118, 0.178), (0.118, -0.178), (-0.118, -0.178))
    ):
        backplate.visual(
            Cylinder(radius=0.015, length=0.012),
            origin=Origin(xyz=(0.004, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name=f"anchor_bolt_{i}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.070, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0485)),
        material=worn_edge,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.380, 0.095, 0.065)),
        origin=Origin(xyz=(0.245, 0.0, 0.050)),
        material=arm_finish,
        name="deep_upper_beam",
    )
    upper_arm.visual(
        Box((0.345, 0.020, 0.083)),
        origin=Origin(xyz=(0.245, 0.056, 0.050)),
        material=worn_edge,
        name="upper_side_rail_0",
    )
    upper_arm.visual(
        Box((0.345, 0.020, 0.083)),
        origin=Origin(xyz=(0.245, -0.056, 0.050)),
        material=worn_edge,
        name="upper_side_rail_1",
    )
    upper_arm.visual(
        Cylinder(radius=0.065, length=0.065),
        origin=Origin(xyz=(0.460, 0.0, 0.050)),
        material=worn_edge,
        name="elbow_bearing",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.052, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=worn_edge,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.305, 0.075, 0.055)),
        origin=Origin(xyz=(0.185, 0.0, 0.110)),
        material=arm_finish,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.270, 0.015, 0.055)),
        origin=Origin(xyz=(0.185, 0.044, 0.110)),
        material=worn_edge,
        name="forearm_side_rail_0",
    )
    forearm.visual(
        Box((0.270, 0.015, 0.055)),
        origin=Origin(xyz=(0.185, -0.044, 0.110)),
        material=worn_edge,
        name="forearm_side_rail_1",
    )
    forearm.visual(
        Cylinder(radius=0.048, length=0.055),
        origin=Origin(xyz=(0.360, 0.0, 0.110)),
        material=worn_edge,
        name="wrist_bearing",
    )

    wrist_flange = model.part("wrist_flange")
    wrist_flange.visual(
        Cylinder(radius=0.045, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=worn_edge,
        name="wrist_hub",
    )
    wrist_flange.visual(
        Box((0.115, 0.055, 0.040)),
        origin=Origin(xyz=(0.073, 0.0, 0.160)),
        material=arm_finish,
        name="flange_neck",
    )
    wrist_flange.visual(
        Cylinder(radius=0.052, length=0.020),
        origin=Origin(xyz=(0.140, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edge,
        name="tool_face",
    )
    for i, (y, z) in enumerate(
        ((0.026, 0.026), (-0.026, 0.026), (0.026, -0.026), (-0.026, -0.026))
    ):
        wrist_flange.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(
                xyz=(0.152, y, 0.160 + z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=fastener,
            name=f"face_bolt_{i}",
        )

    model.articulation(
        "shoulder_revolute",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=upper_arm,
        origin=Origin(xyz=(0.180, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "elbow_revolute",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.460, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-2.15, upper=2.15),
    )
    model.articulation(
        "wrist_revolute",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_flange,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-2.60, upper=2.60),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    backplate = object_model.get_part("backplate")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_flange = object_model.get_part("wrist_flange")
    shoulder = object_model.get_articulation("shoulder_revolute")
    elbow = object_model.get_articulation("elbow_revolute")
    wrist = object_model.get_articulation("wrist_revolute")

    ctx.check(
        "three named revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details="The arm should have shoulder, elbow, and wrist revolutes only.",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, wrist: 0.0}):
        ctx.expect_overlap(
            upper_arm,
            backplate,
            axes="xy",
            elem_a="shoulder_hub",
            elem_b="shoulder_bearing",
            min_overlap=0.045,
            name="shoulder hub is centered over backplate bearing",
        )
        ctx.expect_gap(
            upper_arm,
            backplate,
            axis="z",
            positive_elem="shoulder_hub",
            negative_elem="shoulder_bearing",
            min_gap=0.0,
            max_gap=0.004,
            name="shoulder bearing stack has a small running gap",
        )
        ctx.expect_overlap(
            forearm,
            upper_arm,
            axes="xy",
            elem_a="elbow_hub",
            elem_b="elbow_bearing",
            min_overlap=0.035,
            name="forearm hub is carried by upper arm elbow bearing",
        )
        ctx.expect_gap(
            forearm,
            upper_arm,
            axis="z",
            positive_elem="elbow_hub",
            negative_elem="elbow_bearing",
            min_gap=0.0,
            max_gap=0.008,
            name="elbow stack has a small running gap",
        )
        ctx.expect_overlap(
            wrist_flange,
            forearm,
            axes="xy",
            elem_a="wrist_hub",
            elem_b="wrist_bearing",
            min_overlap=0.030,
            name="wrist flange is carried by forearm wrist bearing",
        )
        ctx.expect_gap(
            wrist_flange,
            forearm,
            axis="z",
            positive_elem="wrist_hub",
            negative_elem="wrist_bearing",
            min_gap=0.0,
            max_gap=0.012,
            name="wrist stack has a small running gap",
        )

    rest_forearm = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.75, elbow: 0.0, wrist: 0.0}):
        swung_forearm = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder swing moves the downstream arm",
        rest_forearm is not None
        and swung_forearm is not None
        and abs(swung_forearm[1] - rest_forearm[1]) > 0.20,
        details=f"rest={rest_forearm}, swung={swung_forearm}",
    )

    rest_wrist = ctx.part_world_position(wrist_flange)
    with ctx.pose({shoulder: 0.0, elbow: 0.85, wrist: 0.0}):
        bent_wrist = ctx.part_world_position(wrist_flange)
    ctx.check(
        "elbow bend moves the wrist joint",
        rest_wrist is not None
        and bent_wrist is not None
        and abs(bent_wrist[1] - rest_wrist[1]) > 0.18,
        details=f"rest={rest_wrist}, bent={bent_wrist}",
    )

    return ctx.report()


object_model = build_object_model()
