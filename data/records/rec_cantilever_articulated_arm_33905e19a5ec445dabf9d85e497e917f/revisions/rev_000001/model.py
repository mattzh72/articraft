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
    model = ArticulatedObject(name="cantilevered_industrial_arm")

    safety_yellow = Material("safety_yellow", rgba=(1.0, 0.66, 0.06, 1.0))
    dark_steel = Material("dark_powder_coat", rgba=(0.045, 0.050, 0.055, 1.0))
    cast_iron = Material("cast_iron", rgba=(0.18, 0.19, 0.20, 1.0))
    machined = Material("machined_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    black = Material("black_fasteners", rgba=(0.015, 0.015, 0.014, 1.0))

    support = model.part("side_support")
    support.visual(
        Box((0.70, 0.50, 0.08)),
        origin=Origin(xyz=(-0.08, 0.0, 0.04)),
        material=cast_iron,
        name="floor_foot",
    )
    support.visual(
        Box((0.24, 0.30, 0.96)),
        origin=Origin(xyz=(-0.16, 0.0, 0.56)),
        material=dark_steel,
        name="upright_column",
    )
    support.visual(
        Box((0.36, 0.38, 0.10)),
        origin=Origin(xyz=(-0.08, 0.0, 1.06)),
        material=cast_iron,
        name="top_cap",
    )
    support.visual(
        Cylinder(radius=0.23, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        material=machined,
        name="fixed_bearing",
    )
    # Box gussets are deliberately broad and overlapping with the column/foot:
    # they read as welded support ribs rather than as slender floating struts.
    support.visual(
        Box((0.34, 0.055, 0.44)),
        origin=Origin(xyz=(-0.02, 0.19, 0.28), rpy=(0.0, -0.38, 0.0)),
        material=dark_steel,
        name="upper_gusset_0",
    )
    support.visual(
        Box((0.34, 0.055, 0.44)),
        origin=Origin(xyz=(-0.02, -0.19, 0.28), rpy=(0.0, -0.38, 0.0)),
        material=dark_steel,
        name="upper_gusset_1",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.24, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=machined,
        name="turntable",
    )
    turret.visual(
        Box((0.28, 0.30, 0.10)),
        origin=Origin(xyz=(0.08, 0.0, 0.075)),
        material=dark_steel,
        name="yoke_base",
    )
    turret.visual(
        Box((0.18, 0.055, 0.36)),
        origin=Origin(xyz=(0.22, 0.17, 0.26)),
        material=dark_steel,
        name="yoke_cheek_0",
    )
    turret.visual(
        Box((0.18, 0.055, 0.36)),
        origin=Origin(xyz=(0.22, -0.17, 0.26)),
        material=dark_steel,
        name="yoke_cheek_1",
    )
    turret.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(0.22, 0.215, 0.26), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="shoulder_bolt_0",
    )
    turret.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(0.22, -0.215, 0.26), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="shoulder_bolt_1",
    )

    upper = model.part("upper_link")
    upper.visual(
        Cylinder(radius=0.12, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_yellow,
        name="shoulder_lug",
    )
    upper.visual(
        Box((0.55, 0.15, 0.14)),
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
        material=safety_yellow,
        name="upper_box_beam",
    )
    upper.visual(
        Box((0.50, 0.17, 0.045)),
        origin=Origin(xyz=(0.39, 0.0, 0.085)),
        material=safety_yellow,
        name="upper_top_flange",
    )
    upper.visual(
        Box((0.50, 0.17, 0.045)),
        origin=Origin(xyz=(0.39, 0.0, -0.085)),
        material=safety_yellow,
        name="upper_bottom_flange",
    )
    upper.visual(
        Cylinder(radius=0.14, length=0.24),
        origin=Origin(xyz=(0.78, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="elbow_housing",
    )
    upper.visual(
        Cylinder(radius=0.033, length=0.035),
        origin=Origin(xyz=(0.78, 0.1275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="elbow_bolt_0",
    )
    upper.visual(
        Cylinder(radius=0.033, length=0.035),
        origin=Origin(xyz=(0.78, -0.1275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="elbow_bolt_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.12, length=0.07),
        origin=Origin(xyz=(0.175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="elbow_output_flange",
    )
    forearm.visual(
        Box((0.46, 0.13, 0.12)),
        origin=Origin(xyz=(0.43, 0.0, 0.0)),
        material=safety_yellow,
        name="forearm_box_beam",
    )
    forearm.visual(
        Box((0.42, 0.15, 0.040)),
        origin=Origin(xyz=(0.43, 0.0, 0.074)),
        material=safety_yellow,
        name="forearm_top_flange",
    )
    forearm.visual(
        Box((0.42, 0.15, 0.040)),
        origin=Origin(xyz=(0.43, 0.0, -0.074)),
        material=safety_yellow,
        name="forearm_bottom_flange",
    )
    forearm.visual(
        Cylinder(radius=0.13, length=0.12),
        origin=Origin(xyz=(0.68, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="wrist_roll_housing",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.14, length=0.11),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="tool_flange",
    )
    wrist.visual(
        Box((0.05, 0.30, 0.24)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=dark_steel,
        name="mounting_plate",
    )
    for idx, (y, z) in enumerate(((-0.09, -0.07), (-0.09, 0.07), (0.09, -0.07), (0.09, 0.07))):
        wrist.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(0.209, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"plate_bolt_{idx}",
        )

    model.articulation(
        "support_to_turret",
        ArticulationType.REVOLUTE,
        parent=support,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.0, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "turret_to_upper_link",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=upper,
        origin=Origin(xyz=(0.22, 0.0, 0.26)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.8, lower=-0.9, upper=1.35),
    )
    model.articulation(
        "upper_link_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.78, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.0, lower=-1.45, upper=1.7),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.68, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=1.8, lower=-3.14, upper=3.14),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("support_to_turret"),
        object_model.get_articulation("turret_to_upper_link"),
        object_model.get_articulation("upper_link_to_forearm"),
        object_model.get_articulation("forearm_to_wrist"),
    ]

    ctx.check(
        "serial revolute chain",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joint_types={[j.articulation_type for j in joints]}",
    )
    ctx.check(
        "industrial joint torque limits",
        all(j.motion_limits is not None and j.motion_limits.effort >= 300.0 for j in joints),
        details=f"limits={[j.motion_limits for j in joints]}",
    )

    support = object_model.get_part("side_support")
    wrist = object_model.get_part("wrist")
    forearm = object_model.get_part("forearm")

    ctx.expect_origin_gap(
        wrist,
        support,
        axis="x",
        min_gap=1.35,
        name="cantilevered reach extends from side support",
    )

    shoulder = object_model.get_articulation("turret_to_upper_link")
    rest_forearm = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.9}):
        lifted_forearm = ctx.part_world_position(forearm)
    ctx.check(
        "shoulder pitch lifts distal chain",
        rest_forearm is not None
        and lifted_forearm is not None
        and lifted_forearm[2] > rest_forearm[2] + 0.25,
        details=f"rest={rest_forearm}, lifted={lifted_forearm}",
    )

    return ctx.report()


object_model = build_object_model()
