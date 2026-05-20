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
    model = ArticulatedObject(name="shoulder_elbow_wrist_arm")

    safety_yellow = Material("safety_yellow", color=(1.0, 0.72, 0.08, 1.0))
    dark_grey = Material("dark_grey", color=(0.08, 0.09, 0.10, 1.0))
    steel = Material("brushed_steel", color=(0.62, 0.65, 0.68, 1.0))
    rubber = Material("black_rubber", color=(0.015, 0.015, 0.018, 1.0))

    rot_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    rot_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    # Root pedestal: a floor-mounted base, a column, and a shoulder yoke.
    base = model.part("base")
    base.visual(Cylinder(0.24, 0.055), origin=Origin(xyz=(0.0, 0.0, 0.0275)), material=dark_grey, name="floor_plate")
    base.visual(Cylinder(0.14, 0.29), origin=Origin(xyz=(0.0, 0.0, 0.20)), material=safety_yellow, name="pedestal_column")
    base.visual(Box((0.20, 0.28, 0.08)), origin=Origin(xyz=(-0.045, 0.0, 0.295)), material=safety_yellow, name="shoulder_saddle")
    for y in (-0.115, 0.115):
        base.visual(Box((0.18, 0.045, 0.23)), origin=Origin(xyz=(0.0, y, 0.42)), material=safety_yellow, name=f"shoulder_yoke_{0 if y < 0 else 1}")
    base.visual(
        Cylinder(0.028, 0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.42), rpy=rot_y.rpy),
        material=steel,
        name="shoulder_shaft",
    )

    # Upper arm frame is at the shoulder axis; the arm reaches along local +X.
    upper = model.part("upper_arm")
    upper.visual(Cylinder(0.074, 0.16), origin=rot_y, material=steel, name="shoulder_barrel")
    upper.visual(Box((0.50, 0.10, 0.075)), origin=Origin(xyz=(0.32, 0.0, 0.0)), material=safety_yellow, name="upper_beam")
    upper.visual(Box((0.13, 0.25, 0.085)), origin=Origin(xyz=(0.525, 0.0, 0.0)), material=safety_yellow, name="elbow_bridge")
    for y in (-0.112, 0.112):
        upper.visual(Box((0.15, 0.04, 0.17)), origin=Origin(xyz=(0.655, y, 0.0)), material=safety_yellow, name=f"elbow_fork_{0 if y < 0 else 1}")
    upper.visual(Cylinder(0.026, 0.28), origin=Origin(xyz=(0.65, 0.0, 0.0), rpy=rot_y.rpy), material=steel, name="elbow_shaft")

    # Forearm frame is at the elbow axis; it repeats the fork-and-barrel construction.
    forearm = model.part("forearm")
    forearm.visual(Cylinder(0.062, 0.16), origin=rot_y, material=steel, name="elbow_barrel")
    forearm.visual(Box((0.42, 0.085, 0.065)), origin=Origin(xyz=(0.270, 0.0, 0.0)), material=safety_yellow, name="forearm_beam")
    forearm.visual(Box((0.10, 0.22, 0.075)), origin=Origin(xyz=(0.445, 0.0, 0.0)), material=safety_yellow, name="wrist_bridge")
    for y in (-0.098, 0.098):
        forearm.visual(Box((0.13, 0.035, 0.14)), origin=Origin(xyz=(0.555, y, 0.0)), material=safety_yellow, name=f"wrist_fork_{0 if y < 0 else 1}")
    forearm.visual(Cylinder(0.022, 0.25), origin=Origin(xyz=(0.55, 0.0, 0.0), rpy=rot_y.rpy), material=steel, name="wrist_shaft")

    # Short wrist link and tool flange.
    wrist = model.part("wrist")
    wrist.visual(Cylinder(0.052, 0.13), origin=rot_y, material=steel, name="wrist_barrel")
    wrist.visual(Box((0.20, 0.075, 0.06)), origin=Origin(xyz=(0.13, 0.0, 0.0)), material=safety_yellow, name="wrist_link")
    wrist.visual(Cylinder(0.075, 0.055), origin=Origin(xyz=(0.255, 0.0, 0.0), rpy=rot_x.rpy), material=steel, name="tool_flange")
    wrist.visual(Cylinder(0.052, 0.018), origin=Origin(xyz=(0.288, 0.0, 0.0), rpy=rot_x.rpy), material=rubber, name="tool_face")
    for i, (y, z) in enumerate(((0.038, 0.038), (-0.038, 0.038), (-0.038, -0.038), (0.038, -0.038))):
        wrist.visual(Cylinder(0.008, 0.014), origin=Origin(xyz=(0.301, y, z), rpy=rot_x.rpy), material=dark_grey, name=f"flange_bolt_{i}")

    shoulder_limits = MotionLimits(effort=180.0, velocity=1.5, lower=math.radians(-120.0), upper=math.radians(135.0))
    elbow_limits = MotionLimits(effort=120.0, velocity=1.8, lower=math.radians(-120.0), upper=math.radians(135.0))
    wrist_limits = MotionLimits(effort=45.0, velocity=2.5, lower=math.radians(-90.0), upper=math.radians(90.0))

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=shoulder_limits,
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.65, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=elbow_limits,
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.55, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=wrist_limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")
    base = object_model.get_part("base")
    upper = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_link = object_model.get_part("wrist")

    ctx.allow_overlap(
        base,
        upper,
        elem_a="shoulder_shaft",
        elem_b="shoulder_barrel",
        reason="The steel shoulder shaft is intentionally captured inside the upper-arm hinge barrel.",
    )
    ctx.allow_overlap(
        upper,
        forearm,
        elem_a="elbow_shaft",
        elem_b="elbow_barrel",
        reason="The elbow shaft is intentionally captured inside the forearm hinge barrel.",
    )
    ctx.allow_overlap(
        forearm,
        wrist_link,
        elem_a="wrist_shaft",
        elem_b="wrist_barrel",
        reason="The wrist shaft is intentionally captured inside the wrist hinge barrel.",
    )

    ctx.check(
        "shoulder and elbow have long forward travel",
        shoulder.motion_limits is not None
        and elbow.motion_limits is not None
        and shoulder.motion_limits.lower <= math.radians(-119.0)
        and elbow.motion_limits.lower <= math.radians(-119.0)
        and shoulder.motion_limits.upper >= math.radians(134.0)
        and elbow.motion_limits.upper >= math.radians(134.0),
    )
    ctx.check(
        "wrist has ninety degree each way travel",
        wrist.motion_limits is not None
        and wrist.motion_limits.lower <= math.radians(-89.0)
        and wrist.motion_limits.upper >= math.radians(89.0),
    )
    ctx.check(
        "revolute axes are parallel and horizontal",
        shoulder.axis == (0.0, -1.0, 0.0) and elbow.axis == (0.0, -1.0, 0.0) and wrist.axis == (0.0, -1.0, 0.0),
    )

    ctx.expect_within(base, upper, axes="xz", inner_elem="shoulder_shaft", outer_elem="shoulder_barrel", name="shoulder shaft sits inside barrel")
    ctx.expect_overlap(base, upper, axes="y", elem_a="shoulder_shaft", elem_b="shoulder_barrel", min_overlap=0.12, name="shoulder shaft spans barrel")
    ctx.expect_within(upper, forearm, axes="xz", inner_elem="elbow_shaft", outer_elem="elbow_barrel", name="elbow shaft sits inside barrel")
    ctx.expect_overlap(upper, forearm, axes="y", elem_a="elbow_shaft", elem_b="elbow_barrel", min_overlap=0.12, name="elbow shaft spans barrel")
    ctx.expect_within(forearm, wrist_link, axes="xz", inner_elem="wrist_shaft", outer_elem="wrist_barrel", name="wrist shaft sits inside barrel")
    ctx.expect_overlap(forearm, wrist_link, axes="y", elem_a="wrist_shaft", elem_b="wrist_barrel", min_overlap=0.10, name="wrist shaft spans barrel")

    rest = ctx.part_world_position(wrist_link)
    with ctx.pose({shoulder: math.radians(35.0), elbow: math.radians(25.0), wrist: math.radians(20.0)}):
        raised = ctx.part_world_position(wrist_link)
    ctx.check(
        "positive revolute motion raises the arm in the working plane",
        rest is not None and raised is not None and raised[2] > rest[2] + 0.25,
        details=f"rest={rest}, raised={raised}",
    )

    return ctx.report()


object_model = build_object_model()
