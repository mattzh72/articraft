from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


STEEL = Material("satin_blasted_steel", rgba=(0.64, 0.66, 0.66, 1.0))
GRAPHITE = Material("graphite_anodized_housings", rgba=(0.14, 0.15, 0.16, 1.0))
DARK = Material("dark_bearing_bores", rgba=(0.02, 0.025, 0.03, 1.0))
BLUE = Material("blue_axis_marks", rgba=(0.05, 0.24, 0.85, 1.0))


def _ring_z(outer: float, inner: float, length: float, z_center: float) -> cq.Workplane:
    outer_solid = (
        cq.Workplane("XY")
        .circle(outer)
        .extrude(length)
        .translate((0.0, 0.0, z_center - length / 2.0))
    )
    inner_solid = (
        cq.Workplane("XY")
        .circle(inner)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, z_center - length / 2.0 - 0.002))
    )
    return outer_solid.cut(inner_solid)


def _ring_x(outer: float, inner: float, length: float, x_center: float) -> cq.Workplane:
    outer_solid = (
        cq.Workplane("YZ")
        .circle(outer)
        .extrude(length)
        .translate((x_center - length / 2.0, 0.0, 0.0))
    )
    inner_solid = (
        cq.Workplane("YZ")
        .circle(inner)
        .extrude(length + 0.004)
        .translate((x_center - length / 2.0 - 0.002, 0.0, 0.0))
    )
    return outer_solid.cut(inner_solid)


def _cylinder_y(radius: float, length: float, y_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((0.0, y_center + length / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_yaw_pitch_roll_wrist")
    for material in (STEEL, GRAPHITE, DARK, BLUE):
        model.materials.append(material)

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.195, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=STEEL,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.135, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=GRAPHITE,
        name="fixed_pedestal",
    )
    pedestal.visual(
        mesh_from_cadquery(_ring_z(0.128, 0.078, 0.030, 0.095), "fixed_bearing_ring"),
        material=STEEL,
        name="fixed_bearing_ring",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        pedestal.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(
                xyz=(0.160 * math.cos(angle), 0.160 * math.sin(angle), 0.033)
            ),
            material=DARK,
            name=f"base_bolt_{i}",
        )

    yaw_housing = model.part("yaw_housing")
    yaw_housing.visual(
        Cylinder(radius=0.118, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=GRAPHITE,
        name="yaw_motor_can",
    )
    yaw_housing.visual(
        Cylinder(radius=0.104, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=STEEL,
        name="yaw_top_bearing",
    )
    yaw_housing.visual(
        Cylinder(radius=0.060, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=GRAPHITE,
        name="neck_column",
    )
    yaw_housing.visual(
        Box((0.190, 0.330, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=STEEL,
        name="yoke_bridge",
    )
    for side, y in (("near", -0.164), ("far", 0.164)):
        yaw_housing.visual(
            Box((0.176, 0.036, 0.205)),
            origin=Origin(xyz=(0.0, y, 0.288)),
            material=STEEL,
            name=f"{side}_pitch_cheek",
        )
        yaw_housing.visual(
            Cylinder(radius=0.073, length=0.018),
            origin=Origin(
                xyz=(0.0, y + (0.027 if y > 0 else -0.027), 0.292),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=GRAPHITE,
            name=f"{side}_pitch_boss",
        )
        yaw_housing.visual(
            Cylinder(radius=0.031, length=0.020),
            origin=Origin(
                xyz=(0.0, y + (0.038 if y > 0 else -0.038), 0.292),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=DARK,
            name=f"{side}_pitch_bore",
        )
    yaw_housing.visual(
        Box((0.045, 0.300, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.398)),
        material=STEEL,
        name="top_tie_bar",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_cylinder_y(0.083, 0.220, 0.0), "pitch_cross_housing"),
        material=GRAPHITE,
        name="pitch_cross_housing",
    )
    pitch_frame.visual(
        Cylinder(radius=0.062, length=0.244),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="pitch_shaft_caps",
    )
    pitch_frame.visual(
        Box((0.180, 0.140, 0.075)),
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
        material=GRAPHITE,
        name="roll_axis_arm",
    )
    pitch_frame.visual(
        mesh_from_cadquery(_ring_x(0.086, 0.066, 0.044, 0.207), "roll_bearing_ring"),
        material=STEEL,
        name="roll_bearing_ring",
    )
    for side, y in (("near", -0.074), ("far", 0.074)):
        pitch_frame.visual(
            Box((0.046, 0.016, 0.046)),
            origin=Origin(xyz=(0.192, y, 0.060)),
            material=STEEL,
            name=f"{side}_ring_web",
        )

    roll_head = model.part("roll_head")
    roll_head.visual(
        Cylinder(radius=0.066, length=0.036),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="rear_bearing_race",
    )
    roll_head.visual(
        Cylinder(radius=0.054, length=0.188),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GRAPHITE,
        name="roll_motor_can",
    )
    roll_head.visual(
        Cylinder(radius=0.076, length=0.024),
        origin=Origin(xyz=(0.191, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="tool_flange",
    )
    roll_head.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.207, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=DARK,
        name="center_socket",
    )
    for i in range(6):
        angle = i * math.tau / 6.0
        roll_head.visual(
            Cylinder(radius=0.0058, length=0.007),
            origin=Origin(
                xyz=(0.2065, 0.048 * math.cos(angle), 0.048 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=DARK,
            name=f"flange_bolt_{i}",
        )
    roll_head.visual(
        Box((0.012, 0.092, 0.006)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=BLUE,
        name="roll_index_mark",
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_housing,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=2.2, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_head,
        origin=Origin(xyz=(0.207, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=4.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw = object_model.get_part("yaw_housing")
    pitch = object_model.get_part("pitch_frame")
    roll = object_model.get_part("roll_head")
    yaw_axis = object_model.get_articulation("yaw_axis")
    pitch_axis = object_model.get_articulation("pitch_axis")
    roll_axis = object_model.get_articulation("roll_axis")

    ctx.expect_gap(
        yaw,
        pedestal,
        axis="z",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="yaw_motor_can",
        negative_elem="fixed_bearing_ring",
        name="yaw housing sits on the fixed bearing stack",
    )
    ctx.expect_within(
        pitch,
        yaw,
        axes="y",
        margin=0.003,
        inner_elem="pitch_cross_housing",
        outer_elem="yoke_bridge",
        name="pitch barrel is captured between yoke sides",
    )
    ctx.expect_within(
        roll,
        pitch,
        axes="yz",
        margin=0.010,
        inner_elem="roll_motor_can",
        outer_elem="roll_bearing_ring",
        name="roll cylinder is concentric with bearing ring",
    )

    rest_roll = ctx.part_world_position(roll)
    with ctx.pose({pitch_axis: 0.7}):
        pitched_roll = ctx.part_world_position(roll)
    ctx.check(
        "pitch axis swings the roll head",
        rest_roll is not None
        and pitched_roll is not None
        and pitched_roll[2] < rest_roll[2] - 0.070,
        details=f"rest={rest_roll}, pitched={pitched_roll}",
    )

    rest_yaw = ctx.part_world_position(roll)
    with ctx.pose({yaw_axis: math.pi / 2.0}):
        yawed_roll = ctx.part_world_position(roll)
    ctx.check(
        "yaw axis turns the wrist about vertical",
        rest_yaw is not None
        and yawed_roll is not None
        and yawed_roll[1] > rest_yaw[1] + 0.180,
        details=f"rest={rest_yaw}, yawed={yawed_roll}",
    )

    with ctx.pose({roll_axis: math.pi / 2.0}):
        ctx.expect_origin_distance(
            roll,
            pitch,
            axes="x",
            min_dist=0.205,
            max_dist=0.209,
            name="roll rotation preserves serial axis spacing",
        )

    return ctx.report()


object_model = build_object_model()
