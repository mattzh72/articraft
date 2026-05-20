from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="bench_robot_arm")

    base_paint = Material("mat_deep_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    blue_paint = Material("mat_robot_blue", rgba=(0.05, 0.22, 0.72, 1.0))
    dark_joint = Material("mat_dark_joint", rgba=(0.015, 0.018, 0.022, 1.0))
    machined = Material("mat_machined_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    rubber = Material("mat_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.42, 0.34, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=base_paint,
        name="bench_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.125, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=base_paint,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.155, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=dark_joint,
        name="lower_bearing_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.135, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.288)),
        material=machined,
        name="pedestal_cap",
    )
    for index, (x, y) in enumerate(
        ((0.155, 0.115), (0.155, -0.115), (-0.155, 0.115), (-0.155, -0.115))
    ):
        pedestal.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(x, y, 0.034)),
            material=machined,
            name=f"base_bolt_{index}",
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.108, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=machined,
        name="shoulder_turntable",
    )
    upper_arm.visual(
        Cylinder(radius=0.082, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.095), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_joint,
        name="shoulder_cross_axis",
    )
    upper_arm.visual(
        Box((0.555, 0.036, 0.060)),
        origin=Origin(xyz=(0.310, 0.077, 0.095)),
        material=blue_paint,
        name="upper_side_0",
    )
    upper_arm.visual(
        Box((0.555, 0.036, 0.060)),
        origin=Origin(xyz=(0.310, -0.077, 0.095)),
        material=blue_paint,
        name="upper_side_1",
    )
    upper_arm.visual(
        Box((0.140, 0.155, 0.050)),
        origin=Origin(xyz=(0.125, 0.0, 0.135)),
        material=blue_paint,
        name="upper_root_bridge",
    )
    for index, y in enumerate((0.075, -0.075)):
        upper_arm.visual(
            Cylinder(radius=0.076, length=0.042),
            origin=Origin(xyz=(0.620, y, 0.095), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_joint,
            name=f"elbow_yoke_{index}",
        )
        upper_arm.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(0.620, y + (0.026 if y > 0.0 else -0.026), 0.095), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"elbow_cap_screw_{index}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.058, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="elbow_center_lug",
    )
    forearm.visual(
        Box((0.430, 0.064, 0.056)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=blue_paint,
        name="forearm_web",
    )
    forearm.visual(
        Box((0.330, 0.030, 0.040)),
        origin=Origin(xyz=(0.270, 0.047, 0.0)),
        material=blue_paint,
        name="forearm_rib_0",
    )
    forearm.visual(
        Box((0.330, 0.030, 0.040)),
        origin=Origin(xyz=(0.270, -0.047, 0.0)),
        material=blue_paint,
        name="forearm_rib_1",
    )
    forearm.visual(
        Cylinder(radius=0.063, length=0.080),
        origin=Origin(xyz=(0.460, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_joint,
        name="wrist_socket",
    )
    forearm.visual(
        Box((0.060, 0.135, 0.038)),
        origin=Origin(xyz=(0.430, 0.0, 0.000)),
        material=blue_paint,
        name="wrist_socket_bridge",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.052, length=0.150),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_joint,
        name="wrist_cartridge",
    )
    wrist.visual(
        Cylinder(radius=0.086, length=0.036),
        origin=Origin(xyz=(0.166, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=machined,
        name="tool_flange",
    )
    wrist.visual(
        Box((0.026, 0.018, 0.048)),
        origin=Origin(xyz=(0.189, 0.0, 0.056)),
        material=rubber,
        name="flange_index_tab",
    )
    for row, y in enumerate((-0.044, 0.044)):
        for col, z in enumerate((-0.044, 0.044)):
            wrist.visual(
                Cylinder(radius=0.010, length=0.014),
                origin=Origin(xyz=(0.188, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_joint,
                name=f"flange_bolt_{row}_{col}",
            )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-pi, upper=pi),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.620, 0.0, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.5, lower=-1.25, upper=1.35),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=3.5, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.check(
        "shoulder is vertical yaw",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in shoulder.axis) == (0.0, 0.0, 1.0),
        details=f"type={shoulder.articulation_type}, axis={shoulder.axis}",
    )
    ctx.check(
        "elbow is horizontal revolute axis",
        elbow.articulation_type == ArticulationType.REVOLUTE
        and abs(elbow.axis[1]) > 0.99
        and abs(elbow.axis[0]) < 1e-6
        and abs(elbow.axis[2]) < 1e-6,
        details=f"type={elbow.articulation_type}, axis={elbow.axis}",
    )
    ctx.check(
        "wrist rolls about forearm axis",
        wrist_roll.articulation_type == ArticulationType.REVOLUTE
        and wrist_roll.axis[0] > 0.99
        and abs(wrist_roll.axis[1]) < 1e-6
        and abs(wrist_roll.axis[2]) < 1e-6,
        details=f"type={wrist_roll.articulation_type}, axis={wrist_roll.axis}",
    )

    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="z",
        positive_elem="shoulder_turntable",
        negative_elem="pedestal_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on pedestal cap",
    )
    ctx.expect_overlap(
        upper_arm,
        pedestal,
        axes="xy",
        elem_a="shoulder_turntable",
        elem_b="pedestal_cap",
        min_overlap=0.18,
        name="yaw bearing footprint is centered on pedestal",
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="x",
        positive_elem="wrist_cartridge",
        negative_elem="wrist_socket",
        max_gap=0.001,
        max_penetration=0.0,
        name="wrist cartridge seats against forearm socket",
    )

    rest_wrist = ctx.part_world_position(wrist)
    with ctx.pose({elbow: 0.75}):
        raised_wrist = ctx.part_world_position(wrist)
    ctx.check(
        "positive elbow bend raises forearm",
        rest_wrist is not None
        and raised_wrist is not None
        and raised_wrist[2] > rest_wrist[2] + 0.20,
        details=f"rest={rest_wrist}, raised={raised_wrist}",
    )

    rest_yaw = ctx.part_world_position(wrist)
    with ctx.pose({shoulder: 0.80}):
        yawed = ctx.part_world_position(wrist)
    ctx.check(
        "shoulder yaw sweeps arm around base",
        rest_yaw is not None and yawed is not None and yawed[1] > rest_yaw[1] + 0.35,
        details=f"rest={rest_yaw}, yawed={yawed}",
    )

    tab_rest = ctx.part_element_world_aabb(wrist, elem="flange_index_tab")
    with ctx.pose({wrist_roll: 1.10}):
        tab_rolled = ctx.part_element_world_aabb(wrist, elem="flange_index_tab")
    if tab_rest is not None and tab_rolled is not None:
        rest_center_y = (tab_rest[0][1] + tab_rest[1][1]) / 2.0
        rolled_center_y = (tab_rolled[0][1] + tab_rolled[1][1]) / 2.0
        rest_center_z = (tab_rest[0][2] + tab_rest[1][2]) / 2.0
        rolled_center_z = (tab_rolled[0][2] + tab_rolled[1][2]) / 2.0
        tab_motion = abs(rolled_center_y - rest_center_y) + abs(rolled_center_z - rest_center_z)
    else:
        tab_motion = 0.0
    ctx.check(
        "wrist roll visibly turns flange index tab",
        tab_motion > 0.045,
        details=f"tab_rest={tab_rest}, tab_rolled={tab_rolled}, motion={tab_motion}",
    )

    return ctx.report()


object_model = build_object_model()
