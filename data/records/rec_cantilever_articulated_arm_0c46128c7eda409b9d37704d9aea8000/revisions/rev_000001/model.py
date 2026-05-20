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
    model = ArticulatedObject(name="floor_pedestal_service_arm")

    dark_cast = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.11, 1.0))
    arm_paint = model.material("warm_grey_paint", rgba=(0.62, 0.64, 0.60, 1.0))
    cover_black = model.material("trimmed_black_covers", rgba=(0.025, 0.028, 0.030, 1.0))
    machined = model.material("brushed_steel_caps", rgba=(0.72, 0.74, 0.70, 1.0))
    bolt_dark = model.material("blackened_bolts", rgba=(0.02, 0.02, 0.018, 1.0))

    cyl_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    cyl_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    shoulder_x = 0.34
    shoulder_z = 1.25

    root = model.part("root_column")
    root.visual(
        Cylinder(radius=0.36, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=dark_cast,
        name="floor_plate",
    )
    root.visual(
        Cylinder(radius=0.145, length=1.04),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=dark_cast,
        name="heavy_column",
    )
    root.visual(
        Cylinder(radius=0.19, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
        material=dark_cast,
        name="top_cast_cap",
    )
    root.visual(
        Box((0.34, 0.30, 0.13)),
        origin=Origin(xyz=(0.17, 0.0, 1.045)),
        material=dark_cast,
        name="shoulder_cantilever",
    )
    root.visual(
        Box((0.34, 0.42, 0.07)),
        origin=Origin(xyz=(shoulder_x - 0.04, 0.0, shoulder_z - 0.215)),
        material=dark_cast,
        name="shoulder_yoke_base",
    )
    root.visual(
        Box((0.24, 0.050, 0.42)),
        origin=Origin(xyz=(shoulder_x, -0.185, shoulder_z)),
        material=dark_cast,
        name="shoulder_cheek_0",
    )
    root.visual(
        Cylinder(radius=0.122, length=0.040),
        origin=Origin(xyz=(shoulder_x, -0.185 * 1.15, shoulder_z), rpy=cyl_y.rpy),
        material=machined,
        name="shoulder_cap_0",
    )
    root.visual(
        Box((0.24, 0.050, 0.42)),
        origin=Origin(xyz=(shoulder_x, 0.185, shoulder_z)),
        material=dark_cast,
        name="shoulder_cheek_1",
    )
    root.visual(
        Cylinder(radius=0.122, length=0.040),
        origin=Origin(xyz=(shoulder_x, 0.185 * 1.15, shoulder_z), rpy=cyl_y.rpy),
        material=machined,
        name="shoulder_cap_1",
    )
    for i, (x, y) in enumerate(((0.24, 0.24), (0.24, -0.24), (-0.24, 0.24), (-0.24, -0.24))):
        root.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(x, y, 0.068)),
            material=bolt_dark,
            name=f"floor_bolt_{i}",
        )

    upper = model.part("upper_link")
    upper.visual(
        Cylinder(radius=0.104, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="shoulder_hub",
    )
    upper.visual(
        Cylinder(radius=0.074, length=0.030),
        origin=Origin(xyz=(0.0, -0.145, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="shoulder_washer_0",
    )
    upper.visual(
        Cylinder(radius=0.074, length=0.030),
        origin=Origin(xyz=(0.0, 0.145, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="shoulder_washer_1",
    )
    upper.visual(
        Box((0.075, 0.150, 0.140)),
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        material=arm_paint,
        name="shoulder_neck",
    )
    upper.visual(
        Box((0.600, 0.180, 0.140)),
        origin=Origin(xyz=(0.445, 0.0, 0.0)),
        material=arm_paint,
        name="upper_box_link",
    )
    for i, z in enumerate((-0.082, 0.082)):
        upper.visual(
            Box((0.560, 0.205, 0.028)),
            origin=Origin(xyz=(0.460, 0.0, z)),
            material=arm_paint,
            name=f"upper_rib_{i}",
        )
    upper.visual(
        Box((0.430, 0.050, 0.030)),
        origin=Origin(xyz=(0.430, 0.0, 0.095)),
        material=cover_black,
        name="upper_cable_cover",
    )
    upper.visual(
        Box((0.125, 0.300, 0.155)),
        origin=Origin(xyz=(0.725, 0.0, 0.0)),
        material=arm_paint,
        name="elbow_clevis_bridge",
    )
    upper.visual(
        Box((0.230, 0.046, 0.360)),
        origin=Origin(xyz=(0.900, -0.148, 0.0)),
        material=arm_paint,
        name="elbow_cheek_0",
    )
    upper.visual(
        Cylinder(radius=0.095, length=0.034),
        origin=Origin(xyz=(0.900, -0.148 * 1.13, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="elbow_cap_0",
    )
    upper.visual(
        Box((0.230, 0.046, 0.360)),
        origin=Origin(xyz=(0.900, 0.148, 0.0)),
        material=arm_paint,
        name="elbow_cheek_1",
    )
    upper.visual(
        Cylinder(radius=0.095, length=0.034),
        origin=Origin(xyz=(0.900, 0.148 * 1.13, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="elbow_cap_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.084, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="elbow_hub",
    )
    forearm.visual(
        Cylinder(radius=0.075, length=0.020),
        origin=Origin(xyz=(0.0, -0.115, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="elbow_washer_0",
    )
    forearm.visual(
        Cylinder(radius=0.075, length=0.020),
        origin=Origin(xyz=(0.0, 0.115, 0.0), rpy=cyl_y.rpy),
        material=machined,
        name="elbow_washer_1",
    )
    forearm.visual(
        Box((0.110, 0.130, 0.112)),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material=arm_paint,
        name="forearm_neck",
    )
    forearm.visual(
        Box((0.420, 0.140, 0.112)),
        origin=Origin(xyz=(0.335, 0.0, 0.0)),
        material=arm_paint,
        name="forearm_box_link",
    )
    for i, z in enumerate((-0.065, 0.065)):
        forearm.visual(
            Box((0.390, 0.158, 0.022)),
            origin=Origin(xyz=(0.350, 0.0, z)),
            material=arm_paint,
            name=f"forearm_rib_{i}",
        )
    forearm.visual(
        Box((0.300, 0.045, 0.026)),
        origin=Origin(xyz=(0.360, 0.0, 0.078)),
        material=cover_black,
        name="forearm_cable_cover",
    )
    forearm.visual(
        Cylinder(radius=0.102, length=0.070),
        origin=Origin(xyz=(0.585, 0.0, 0.0), rpy=cyl_x.rpy),
        material=arm_paint,
        name="wrist_bearing",
    )
    forearm.visual(
        Box((0.060, 0.120, 0.100)),
        origin=Origin(xyz=(0.520, 0.0, 0.0)),
        material=arm_paint,
        name="wrist_neck",
    )

    flange = model.part("tool_flange")
    flange.visual(
        Cylinder(radius=0.043, length=0.080),
        origin=Origin(xyz=(-0.025, 0.0, 0.0), rpy=cyl_x.rpy),
        material=machined,
        name="flange_spindle",
    )
    flange.visual(
        Cylinder(radius=0.125, length=0.070),
        origin=Origin(xyz=(0.037, 0.0, 0.0), rpy=cyl_x.rpy),
        material=machined,
        name="flange_disk",
    )
    flange.visual(
        Cylinder(radius=0.050, length=0.040),
        origin=Origin(xyz=(0.092, 0.0, 0.0), rpy=cyl_x.rpy),
        material=machined,
        name="pilot_boss",
    )
    for i, (y, z) in enumerate(((0.080, 0.0), (-0.080, 0.0), (0.0, 0.080), (0.0, -0.080))):
        flange.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(0.070, y, z), rpy=cyl_x.rpy),
            material=bolt_dark,
            name=f"flange_bolt_{i}",
        )

    shoulder = model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=root,
        child=upper,
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.8, lower=-0.70, upper=0.45),
    )
    elbow = model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=forearm,
        origin=Origin(xyz=(0.900, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=170.0, velocity=0.95, lower=-1.10, upper=1.05),
    )
    wrist = model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=flange,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.6, lower=-math.pi, upper=math.pi),
    )

    # Name the joints in metadata so tests can refer to the same semantic chain.
    model.meta["primary_joints"] = (shoulder.name, elbow.name, wrist.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_column")
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    flange = object_model.get_part("tool_flange")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.allow_overlap(
        forearm,
        flange,
        elem_a="wrist_bearing",
        elem_b="flange_spindle",
        reason="The flange spindle is intentionally captured inside the wrist bearing proxy.",
    )

    ctx.check(
        "shoulder and elbow axes are parallel horizontal",
        tuple(shoulder.axis) == (0.0, 1.0, 0.0) and tuple(elbow.axis) == (0.0, 1.0, 0.0),
        details=f"shoulder={shoulder.axis}, elbow={elbow.axis}",
    )
    ctx.check(
        "wrist rotates about the forearm tool axis",
        tuple(wrist.axis) == (1.0, 0.0, 0.0),
        details=f"wrist={wrist.axis}",
    )

    ctx.expect_gap(
        root,
        upper,
        axis="y",
        positive_elem="shoulder_cheek_1",
        negative_elem="shoulder_hub",
        min_gap=0.018,
        name="shoulder positive cheek clears hub",
    )
    ctx.expect_gap(
        upper,
        root,
        axis="y",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_cheek_0",
        min_gap=0.018,
        name="shoulder negative cheek clears hub",
    )
    ctx.expect_gap(
        upper,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_1",
        negative_elem="elbow_hub",
        min_gap=0.018,
        name="elbow positive cheek clears hub",
    )
    ctx.expect_gap(
        forearm,
        upper,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_0",
        min_gap=0.018,
        name="elbow negative cheek clears hub",
    )
    ctx.expect_gap(
        flange,
        forearm,
        axis="x",
        positive_elem="flange_disk",
        negative_elem="wrist_bearing",
        min_gap=0.001,
        max_gap=0.003,
        name="flange seats against wrist bearing",
    )
    ctx.expect_within(
        flange,
        forearm,
        axes="yz",
        inner_elem="flange_spindle",
        outer_elem="wrist_bearing",
        margin=0.0,
        name="flange spindle is centered in wrist bearing",
    )
    ctx.expect_overlap(
        flange,
        forearm,
        axes="x",
        elem_a="flange_spindle",
        elem_b="wrist_bearing",
        min_overlap=0.060,
        name="flange spindle remains captured in wrist bearing",
    )

    for joint, value, check_name in (
        (shoulder, -0.70, "shoulder lower-limit cheek clearance"),
        (shoulder, 0.45, "shoulder upper-limit cheek clearance"),
    ):
        with ctx.pose({joint: value}):
            ctx.expect_gap(
                root,
                upper,
                axis="y",
                positive_elem="shoulder_cheek_1",
                negative_elem="shoulder_hub",
                min_gap=0.018,
                name=check_name,
            )

    for value, check_name in ((-1.10, "elbow lower-limit cheek clearance"), (1.05, "elbow upper-limit cheek clearance")):
        with ctx.pose({elbow: value}):
            ctx.expect_gap(
                upper,
                forearm,
                axis="y",
                positive_elem="elbow_cheek_1",
                negative_elem="elbow_hub",
                min_gap=0.018,
                name=check_name,
            )

    return ctx.report()


object_model = build_object_model()
