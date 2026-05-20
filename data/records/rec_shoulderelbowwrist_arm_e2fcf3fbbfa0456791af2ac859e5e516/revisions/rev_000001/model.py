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
    model = ArticulatedObject(name="bench_articulated_arm")

    cast_iron = Material("mat_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    dark_steel = Material("mat_dark_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    brushed_aluminum = Material("mat_brushed_aluminum", rgba=(0.62, 0.66, 0.68, 1.0))
    safety_yellow = Material("mat_axis_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    rubber = Material("mat_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    root = model.part("root_bracket")
    root.visual(
        Box((0.52, 0.36, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_iron,
        name="bench_plate",
    )
    root.visual(
        Cylinder(radius=0.105, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=cast_iron,
        name="round_base_boss",
    )
    root.visual(
        Box((0.12, 0.22, 0.44)),
        origin=Origin(xyz=(-0.14, 0.0, 0.34)),
        material=cast_iron,
        name="rear_pedestal",
    )
    root.visual(
        Box((0.16, 0.035, 0.16)),
        origin=Origin(xyz=(-0.01, 0.095, 0.62)),
        material=cast_iron,
        name="shoulder_yoke_ear_0",
    )
    root.visual(
        Box((0.16, 0.035, 0.16)),
        origin=Origin(xyz=(-0.01, -0.095, 0.62)),
        material=cast_iron,
        name="shoulder_yoke_ear_1",
    )
    for y, suffix in ((0.115, "0"), (-0.115, "1")):
        root.visual(
            Cylinder(radius=0.058, length=0.020),
            origin=Origin(xyz=(0.0, y, 0.62), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
            name=f"shoulder_axis_cap_{suffix}",
        )
    for x, y, suffix in (
        (-0.18, 0.12, "0"),
        (0.18, 0.12, "1"),
        (-0.18, -0.12, "2"),
        (0.18, -0.12, "3"),
    ):
        root.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=(x, y, 0.063)),
            material=dark_steel,
            name=f"mount_bolt_{suffix}",
        )

    upper = model.part("upper_arm")
    upper.visual(
        Cylinder(radius=0.052, length=0.155),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_hub",
    )
    for y, suffix in ((0.045, "0"), (-0.045, "1")):
        upper.visual(
            Cylinder(radius=0.025, length=0.68),
            origin=Origin(xyz=(0.38, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_aluminum,
            name=f"upper_tube_{suffix}",
        )
    upper.visual(
        Box((0.075, 0.125, 0.040)),
        origin=Origin(xyz=(0.30, 0.0, 0.0)),
        material=brushed_aluminum,
        name="upper_tie_web",
    )
    upper.visual(
        Box((0.12, 0.030, 0.14)),
        origin=Origin(xyz=(0.78, 0.085, 0.0)),
        material=brushed_aluminum,
        name="elbow_yoke_ear_0",
    )
    upper.visual(
        Box((0.12, 0.030, 0.14)),
        origin=Origin(xyz=(0.78, -0.085, 0.0)),
        material=brushed_aluminum,
        name="elbow_yoke_ear_1",
    )
    for y, suffix in ((0.105, "0"), (-0.105, "1")):
        upper.visual(
            Cylinder(radius=0.050, length=0.020),
            origin=Origin(xyz=(0.78, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
            name=f"elbow_axis_cap_{suffix}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.047, length=0.140),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_hub",
    )
    for y, suffix in ((0.040, "0"), (-0.040, "1")):
        forearm.visual(
            Cylinder(radius=0.023, length=0.575),
            origin=Origin(xyz=(0.315, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_aluminum,
            name=f"forearm_tube_{suffix}",
        )
    forearm.visual(
        Box((0.065, 0.110, 0.034)),
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        material=brushed_aluminum,
        name="forearm_tie_web",
    )
    forearm.visual(
        Box((0.10, 0.026, 0.10)),
        origin=Origin(xyz=(0.65, 0.070, 0.0)),
        material=brushed_aluminum,
        name="wrist_yoke_ear_0",
    )
    forearm.visual(
        Box((0.10, 0.026, 0.10)),
        origin=Origin(xyz=(0.65, -0.070, 0.0)),
        material=brushed_aluminum,
        name="wrist_yoke_ear_1",
    )
    for y, suffix in ((0.088, "0"), (-0.088, "1")):
        forearm.visual(
            Cylinder(radius=0.039, length=0.018),
            origin=Origin(xyz=(0.65, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow,
            name=f"wrist_axis_cap_{suffix}",
        )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.038, length=0.114),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wrist_hub",
    )
    wrist.visual(
        Cylinder(radius=0.032, length=0.23),
        origin=Origin(xyz=(0.13, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="wrist_body",
    )
    wrist.visual(
        Cylinder(radius=0.046, length=0.026),
        origin=Origin(xyz=(0.255, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="tool_mount_face",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=root,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.6, lower=-0.85, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.78, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=1.8, lower=-1.6, upper=1.8),
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.65, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.4, lower=-1.4, upper=1.4),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_bracket")
    upper = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist_joint = object_model.get_articulation("wrist_joint")

    ctx.check(
        "three revolute joints in series",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.expect_gap(
        root,
        upper,
        axis="y",
        positive_elem="shoulder_yoke_ear_0",
        negative_elem="shoulder_hub",
        max_gap=0.002,
        max_penetration=0.0,
        name="shoulder positive ear supports hub",
    )
    ctx.expect_gap(
        upper,
        root,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_yoke_ear_1",
        max_gap=0.002,
        max_penetration=0.0,
        name="shoulder negative ear supports hub",
    )
    ctx.expect_gap(
        upper,
        forearm,
        axis="y",
        positive_elem="elbow_yoke_ear_0",
        negative_elem="elbow_hub",
        max_gap=0.002,
        max_penetration=0.0,
        name="elbow positive ear supports hub",
    )
    ctx.expect_gap(
        forearm,
        upper,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_yoke_ear_1",
        max_gap=0.002,
        max_penetration=0.0,
        name="elbow negative ear supports hub",
    )
    ctx.expect_gap(
        forearm,
        wrist,
        axis="y",
        positive_elem="wrist_yoke_ear_0",
        negative_elem="wrist_hub",
        max_gap=0.002,
        max_penetration=0.0,
        name="wrist positive ear supports hub",
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="y",
        positive_elem="wrist_hub",
        negative_elem="wrist_yoke_ear_1",
        max_gap=0.002,
        max_penetration=0.0,
        name="wrist negative ear supports hub",
    )
    ctx.expect_origin_distance(upper, forearm, axes="x", min_dist=0.75, max_dist=0.81, name="upper link span")
    ctx.expect_origin_distance(forearm, wrist, axes="x", min_dist=0.63, max_dist=0.67, name="forearm link span")

    rest_elbow_pos = ctx.part_world_position(forearm)
    rest_wrist_pos = ctx.part_world_position(wrist)
    with ctx.pose({shoulder: 0.65, elbow: 0.55, wrist_joint: -0.35}):
        raised_elbow_pos = ctx.part_world_position(forearm)
        raised_wrist_pos = ctx.part_world_position(wrist)
    ctx.check(
        "shoulder lift raises elbow axis",
        rest_elbow_pos is not None
        and raised_elbow_pos is not None
        and raised_elbow_pos[2] > rest_elbow_pos[2] + 0.30,
        details=f"rest={rest_elbow_pos}, raised={raised_elbow_pos}",
    )
    ctx.check(
        "series joints move wrist independently",
        rest_wrist_pos is not None
        and raised_wrist_pos is not None
        and abs(raised_wrist_pos[0] - rest_wrist_pos[0]) > 0.25
        and raised_wrist_pos[2] > rest_wrist_pos[2] + 0.55,
        details=f"rest={rest_wrist_pos}, posed={raised_wrist_pos}",
    )

    return ctx.report()


object_model = build_object_model()
