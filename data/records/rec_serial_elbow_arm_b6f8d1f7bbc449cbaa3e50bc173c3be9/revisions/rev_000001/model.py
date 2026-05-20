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
    model = ArticulatedObject(name="serial_elbow_arm_module")

    cast_gray = Material("cast_gray", rgba=(0.34, 0.37, 0.39, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    arm_orange = Material("arm_orange", rgba=(0.95, 0.43, 0.12, 1.0))
    end_plate_blue = Material("end_plate_blue", rgba=(0.10, 0.24, 0.55, 1.0))

    y_axis = (math.pi / 2.0, 0.0, 0.0)
    x_axis = (0.0, math.pi / 2.0, 0.0)

    base = model.part("pedestal")
    base.visual(
        Box((0.52, 0.34, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_gray,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.085, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=cast_gray,
        name="pedestal_column",
    )
    base.visual(
        Box((0.22, 0.24, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=cast_gray,
        name="shoulder_plinth",
    )
    for cheek_name, bearing_name, y in (
        ("shoulder_cheek_0", "shoulder_bearing_0", -0.095),
        ("shoulder_cheek_1", "shoulder_bearing_1", 0.095),
    ):
        base.visual(
            Box((0.160, 0.035, 0.260)),
            origin=Origin(xyz=(0.0, y, 0.565)),
            material=cast_gray,
            name=cheek_name,
        )
        base.visual(
            Cylinder(radius=0.070, length=0.024),
            origin=Origin(xyz=(0.0, y * 1.28, 0.600), rpy=y_axis),
            material=dark_steel,
            name=bearing_name,
        )
    base.visual(
        Cylinder(radius=0.032, length=0.255),
        origin=Origin(xyz=(0.0, 0.0, 0.600), rpy=y_axis),
        material=satin_steel,
        name="shoulder_shaft",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.082, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_axis),
        material=dark_steel,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.100, 0.130, 0.052)),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material=arm_orange,
        name="upper_neck",
    )
    for suffix, y in (("0", -0.078), ("1", 0.078)):
        upper_arm.visual(
            Box((0.570, 0.035, 0.060)),
            origin=Origin(xyz=(0.405, y, 0.0)),
            material=arm_orange,
            name=f"upper_rail_{suffix}",
        )
        upper_arm.visual(
            Box((0.150, 0.035, 0.205)),
            origin=Origin(xyz=(0.720, y * 1.35, 0.0)),
            material=arm_orange,
            name=f"elbow_cheek_{suffix}",
        )
    upper_arm.visual(
        Cylinder(radius=0.027, length=0.220),
        origin=Origin(xyz=(0.720, 0.0, 0.0), rpy=y_axis),
        material=satin_steel,
        name="elbow_shaft",
    )
    for suffix, y in (("0", -0.113), ("1", 0.113)):
        upper_arm.visual(
            Cylinder(radius=0.070, length=0.024),
            origin=Origin(xyz=(0.720, y, 0.0), rpy=y_axis),
            material=dark_steel,
            name=f"elbow_bearing_{suffix}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.073, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_axis),
        material=dark_steel,
        name="elbow_hub",
    )
    for suffix, y in (("0", -0.035), ("1", 0.035)):
        forearm.visual(
            Box((0.500, 0.026, 0.052)),
            origin=Origin(xyz=(0.300, y, 0.0)),
            material=arm_orange,
            name=f"forearm_rail_{suffix}",
        )
    forearm.visual(
        Box((0.070, 0.120, 0.074)),
        origin=Origin(xyz=(0.545, 0.0, 0.0)),
        material=arm_orange,
        name="wrist_block",
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Box((0.036, 0.205, 0.155)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=end_plate_blue,
        name="tool_plate",
    )
    for i, (y, z) in enumerate(((-0.067, -0.048), (-0.067, 0.048), (0.067, -0.048), (0.067, 0.048))):
        end_plate.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(0.040, y, z), rpy=x_axis),
            material=satin_steel,
            name=f"bolt_boss_{i}",
        )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.2, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.720, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.6, lower=-2.10, upper=2.10),
    )
    model.articulation(
        "wrist_mount",
        ArticulationType.FIXED,
        parent=forearm,
        child=end_plate,
        origin=Origin(xyz=(0.580, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    end_plate = object_model.get_part("end_plate")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.allow_overlap(
        pedestal,
        upper_arm,
        elem_a="shoulder_shaft",
        elem_b="shoulder_hub",
        reason="The fixed shoulder shaft is intentionally captured inside the rotating shoulder hub.",
    )
    ctx.expect_within(
        pedestal,
        upper_arm,
        axes="xz",
        inner_elem="shoulder_shaft",
        outer_elem="shoulder_hub",
        margin=0.0,
        name="shoulder shaft is radially inside the shoulder hub",
    )
    ctx.expect_overlap(
        pedestal,
        upper_arm,
        axes="y",
        elem_a="shoulder_shaft",
        elem_b="shoulder_hub",
        min_overlap=0.090,
        name="shoulder shaft spans the rotating hub",
    )

    ctx.allow_overlap(
        upper_arm,
        forearm,
        elem_a="elbow_shaft",
        elem_b="elbow_hub",
        reason="The elbow shaft is intentionally captured inside the rotating forearm hub.",
    )
    ctx.expect_within(
        upper_arm,
        forearm,
        axes="xz",
        inner_elem="elbow_shaft",
        outer_elem="elbow_hub",
        margin=0.0,
        name="elbow shaft is radially inside the elbow hub",
    )
    ctx.expect_overlap(
        upper_arm,
        forearm,
        axes="y",
        elem_a="elbow_shaft",
        elem_b="elbow_hub",
        min_overlap=0.080,
        name="elbow shaft spans the rotating hub",
    )

    ctx.expect_overlap(
        upper_arm,
        pedestal,
        axes="xz",
        min_overlap=0.120,
        elem_a="shoulder_hub",
        elem_b="shoulder_cheek_0",
        name="shoulder hub sits at the carried yoke height",
    )
    ctx.expect_contact(
        end_plate,
        forearm,
        elem_a="tool_plate",
        elem_b="wrist_block",
        contact_tol=0.002,
        name="end plate is mounted to the wrist block",
    )

    shoulder_axis = tuple(round(v, 6) for v in shoulder.axis)
    elbow_axis = tuple(round(v, 6) for v in elbow.axis)
    ctx.check(
        "shoulder and elbow use parallel horizontal axes",
        shoulder_axis == elbow_axis == (0.0, -1.0, 0.0),
        details=f"shoulder_axis={shoulder.axis}, elbow_axis={elbow.axis}",
    )

    rest_tip = ctx.part_world_position(end_plate)
    with ctx.pose({shoulder: 0.65, elbow: 0.0}):
        raised_by_shoulder = ctx.part_world_position(end_plate)
    with ctx.pose({shoulder: 0.0, elbow: 0.75}):
        raised_by_elbow = ctx.part_world_position(end_plate)
    ctx.check(
        "positive shoulder raises the serial arm",
        rest_tip is not None
        and raised_by_shoulder is not None
        and raised_by_shoulder[2] > rest_tip[2] + 0.20,
        details=f"rest={rest_tip}, shoulder_pose={raised_by_shoulder}",
    )
    ctx.check(
        "positive elbow folds the forearm upward",
        rest_tip is not None
        and raised_by_elbow is not None
        and raised_by_elbow[2] > rest_tip[2] + 0.12,
        details=f"rest={rest_tip}, elbow_pose={raised_by_elbow}",
    )

    return ctx.report()


object_model = build_object_model()
