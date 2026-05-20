from __future__ import annotations

from math import pi, radians

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
    model = ArticulatedObject(name="compact_service_manipulator")

    dark_steel = model.material("dark_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.66, 0.08, 1.0))
    blue_gray = model.material("blue_gray", rgba=(0.28, 0.36, 0.43, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    bolt = model.material("brushed_bolt", rgba=(0.58, 0.60, 0.58, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.44, 0.44, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Box((0.245, 0.245, 0.280)),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=graphite,
        name="square_column",
    )
    base.visual(
        Cylinder(radius=0.158, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=bolt,
        name="top_bearing",
    )
    for i, (x, y) in enumerate(
        ((-0.155, -0.155), (-0.155, 0.155), (0.155, -0.155), (0.155, 0.155))
    ):
        base.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(x, y, 0.070)),
            material=bolt,
            name=f"base_bolt_{i}",
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Box((0.285, 0.285, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=blue_gray,
        name="turntable_cap",
    )
    upper_link.visual(
        Box((0.175, 0.175, 0.145)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=blue_gray,
        name="shoulder_block",
    )
    upper_link.visual(
        Cylinder(radius=0.082, length=0.190),
        origin=Origin(xyz=(0.050, 0.0, 0.165), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_cross_tube",
    )

    beam_angle = radians(30.0)
    beam_length = 0.345
    beam_start = (0.035, 0.0, 0.155)
    beam_center = (
        beam_start[0] + 0.5 * beam_length * 0.8660254038,
        0.0,
        beam_start[2] + 0.5 * beam_length * 0.5,
    )
    upper_link.visual(
        Box((beam_length, 0.106, 0.074)),
        origin=Origin(xyz=beam_center, rpy=(0.0, -beam_angle, 0.0)),
        material=safety_yellow,
        name="raised_beam",
    )

    elbow_origin = (0.440, 0.0, 0.360)
    upper_link.visual(
        Box((0.047, 0.188, 0.135)),
        origin=Origin(xyz=(0.354, 0.0, elbow_origin[2])),
        material=blue_gray,
        name="elbow_bridge",
    )
    upper_link.visual(
        Box((0.130, 0.030, 0.165)),
        origin=Origin(xyz=(0.438, -0.075, elbow_origin[2])),
        material=blue_gray,
        name="elbow_cheek_0",
    )
    upper_link.visual(
        Box((0.130, 0.030, 0.165)),
        origin=Origin(xyz=(0.438, 0.075, elbow_origin[2])),
        material=blue_gray,
        name="elbow_cheek_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.055, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.300, 0.086, 0.066)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=safety_yellow,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.070, 0.112, 0.092)),
        origin=Origin(xyz=(0.285, 0.0, 0.0)),
        material=blue_gray,
        name="wrist_socket",
    )

    wrist_block = model.part("wrist_block")
    wrist_block.visual(
        Box((0.088, 0.140, 0.092)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material=blue_gray,
        name="wrist_case",
    )
    wrist_block.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=dark_steel,
        name="tool_flange",
    )
    wrist_block.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.105, 0.032, 0.030), rpy=(0.0, pi / 2, 0.0)),
        material=bolt,
        name="flange_bolt_0",
    )
    wrist_block.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.105, -0.032, 0.030), rpy=(0.0, pi / 2, 0.0)),
        material=bolt,
        name="flange_bolt_1",
    )
    wrist_block.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.105, 0.032, -0.030), rpy=(0.0, pi / 2, 0.0)),
        material=bolt,
        name="flange_bolt_2",
    )
    wrist_block.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.105, -0.032, -0.030), rpy=(0.0, pi / 2, 0.0)),
        material=bolt,
        name="flange_bolt_3",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.1, lower=-pi, upper=pi),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=elbow_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.4, lower=-1.05, upper=1.10),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_block,
        origin=Origin(xyz=(0.320, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.2, lower=-1.75, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    wrist_block = object_model.get_part("wrist_block")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.check("shoulder is vertical yaw", tuple(shoulder.axis) == (0.0, 0.0, 1.0))
    ctx.check("elbow is horizontal hinge", tuple(elbow.axis) == (0.0, 1.0, 0.0))
    ctx.check("wrist rolls on nose axis", tuple(wrist.axis) == (1.0, 0.0, 0.0))

    ctx.expect_gap(
        upper_link,
        base,
        axis="z",
        positive_elem="turntable_cap",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.000001,
        name="turntable cap seats on base bearing",
    )
    ctx.expect_gap(
        forearm,
        upper_link,
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_cheek_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="elbow hub bears on lower cheek",
    )
    ctx.expect_gap(
        upper_link,
        forearm,
        axis="y",
        positive_elem="elbow_cheek_1",
        negative_elem="elbow_hub",
        max_gap=0.002,
        max_penetration=0.0,
        name="elbow hub bears on upper cheek",
    )
    ctx.expect_gap(
        wrist_block,
        forearm,
        axis="x",
        positive_elem="wrist_case",
        negative_elem="forearm_beam",
        max_gap=0.001,
        max_penetration=0.0,
        name="wrist block is seated at forearm nose",
    )

    rest_wrist = ctx.part_world_position(wrist_block)
    with ctx.pose({shoulder: pi / 2}):
        yawed_wrist = ctx.part_world_position(wrist_block)
    ctx.check(
        "shoulder yaw swings the arm around the column",
        rest_wrist is not None
        and yawed_wrist is not None
        and yawed_wrist[1] > rest_wrist[1] + 0.25
        and abs(yawed_wrist[2] - rest_wrist[2]) < 0.01,
        details=f"rest={rest_wrist}, yawed={yawed_wrist}",
    )

    with ctx.pose({elbow: 0.65}):
        elbowed_wrist = ctx.part_world_position(wrist_block)
    ctx.check(
        "elbow hinge pitches the forearm",
        rest_wrist is not None
        and elbowed_wrist is not None
        and elbowed_wrist[2] < rest_wrist[2] - 0.12,
        details=f"rest={rest_wrist}, elbowed={elbowed_wrist}",
    )

    return ctx.report()


object_model = build_object_model()
