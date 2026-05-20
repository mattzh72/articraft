from __future__ import annotations

from math import radians

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
    model = ArticulatedObject(name="bench_cantilever_tool_arm")

    painted = Material("powder_coated_blue", rgba=(0.05, 0.18, 0.42, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark = Material("dark_bushings", rgba=(0.02, 0.025, 0.03, 1.0))
    plate = Material("machined_tool_plate", rgba=(0.78, 0.78, 0.72, 1.0))

    base = model.part("base_column")
    base.visual(Box((0.34, 0.28, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=painted, name="bench_foot")
    base.visual(Cylinder(radius=0.055, length=0.60), origin=Origin(xyz=(0.0, 0.0, 0.33)), material=painted, name="column_tube")
    base.visual(Box((0.18, 0.15, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.63)), material=painted, name="shoulder_saddle")
    for x in (-0.057, 0.057):
        base.visual(Box((0.024, 0.13, 0.16)), origin=Origin(xyz=(x, 0.0, 0.70)), material=painted, name=f"shoulder_cheek_{'n' if x < 0 else 'p'}")
        base.visual(
            Cylinder(radius=0.026, length=0.007),
            origin=Origin(xyz=(x + (-0.014 if x < 0 else 0.014), 0.0, 0.70), rpy=(0.0, radians(90), 0.0)),
            material=dark,
            name=f"shoulder_bore_cap_{'n' if x < 0 else 'p'}",
        )

    shoulder = model.part("shoulder_link")
    shoulder.visual(Cylinder(radius=0.047, length=0.090), origin=Origin(rpy=(0.0, radians(90), 0.0)), material=steel, name="shoulder_trunnion")
    shoulder.visual(Box((0.052, 0.41, 0.052)), origin=Origin(xyz=(0.0, 0.225, 0.0)), material=painted, name="upper_arm_tube")
    shoulder.visual(Box((0.13, 0.050, 0.080)), origin=Origin(xyz=(0.0, 0.455, 0.0)), material=painted, name="elbow_clevis_bridge")
    for x in (-0.052, 0.052):
        shoulder.visual(Box((0.024, 0.14, 0.13)), origin=Origin(xyz=(x, 0.54, 0.0)), material=painted, name=f"elbow_cheek_{'n' if x < 0 else 'p'}")
        shoulder.visual(
            Cylinder(radius=0.023, length=0.006),
            origin=Origin(xyz=(x + (-0.014 if x < 0 else 0.014), 0.54, 0.0), rpy=(0.0, radians(90), 0.0)),
            material=dark,
            name=f"elbow_bore_cap_{'n' if x < 0 else 'p'}",
        )

    elbow = model.part("elbow_link")
    elbow.visual(Cylinder(radius=0.042, length=0.080), origin=Origin(rpy=(0.0, radians(90), 0.0)), material=steel, name="elbow_trunnion")
    elbow.visual(Box((0.046, 0.27, 0.046)), origin=Origin(xyz=(0.0, 0.170, 0.0)), material=painted, name="forearm_tube")
    elbow.visual(Box((0.105, 0.035, 0.13)), origin=Origin(xyz=(0.0, 0.318, 0.0)), material=painted, name="wrist_clevis_bridge")
    for z in (-0.041, 0.041):
        elbow.visual(Box((0.105, 0.12, 0.018)), origin=Origin(xyz=(0.0, 0.38, z)), material=painted, name=f"wrist_cheek_{'low' if z < 0 else 'high'}")
        elbow.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(0.0, 0.38, z + (-0.012 if z < 0 else 0.012)), rpy=(0.0, 0.0, 0.0)),
            material=dark,
            name=f"wrist_bore_cap_{'low' if z < 0 else 'high'}",
        )

    tool = model.part("tool_plate")
    tool.visual(Cylinder(radius=0.032, length=0.12), origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(radians(-90), 0.0, 0.0)), material=steel, name="roll_shaft")
    tool.visual(Box((0.16, 0.018, 0.10)), origin=Origin(xyz=(0.0, 0.095, 0.0)), material=plate, name="mounting_plate")
    for x in (-0.050, 0.050):
        for z in (-0.030, 0.030):
            tool.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(x, 0.106, z), rpy=(radians(-90), 0.0, 0.0)),
                material=dark,
                name=f"tool_slot_{'n' if x < 0 else 'p'}_{'l' if z < 0 else 'h'}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=radians(-25), upper=radians(95)),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=elbow,
        origin=Origin(xyz=(0.0, 0.54, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.4, lower=radians(-45), upper=radians(90)),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=elbow,
        child=tool,
        origin=Origin(xyz=(0.0, 0.38, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=radians(-90), upper=radians(90)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist_roll")

    ctx.check("shoulder travel about 120 degrees", abs((shoulder.motion_limits.upper - shoulder.motion_limits.lower) - radians(120)) < 1e-6)
    ctx.check("elbow travel about 135 degrees", abs((elbow.motion_limits.upper - elbow.motion_limits.lower) - radians(135)) < 1e-6)
    ctx.check("wrist rolls ninety each way", wrist.motion_limits.lower == radians(-90) and wrist.motion_limits.upper == radians(90))
    ctx.check("shoulder and elbow axes are parallel", tuple(shoulder.axis) == tuple(elbow.axis) == (1.0, 0.0, 0.0))

    tool = object_model.get_part("tool_plate")
    rest_pos = ctx.part_world_position(tool)
    with ctx.pose({shoulder: radians(75), elbow: radians(60)}):
        raised_pos = ctx.part_world_position(tool)
    ctx.check(
        "arm raises tool plate at upper pose",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.25,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
