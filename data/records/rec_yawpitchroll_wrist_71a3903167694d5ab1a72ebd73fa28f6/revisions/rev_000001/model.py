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


def _yaw_collar_mesh():
    """Broad root collar with an open bearing race, spokes, and a central hub."""
    ring = cq.Workplane("XY").circle(0.090).circle(0.055).extrude(0.055)
    spoke_x = cq.Workplane("XY").box(0.122, 0.024, 0.018).translate((0.0, 0.0, 0.055))
    spoke_y = cq.Workplane("XY").box(0.024, 0.122, 0.018).translate((0.0, 0.0, 0.055))
    hub = cq.Workplane("XY").circle(0.040).extrude(0.075).translate((0.0, 0.0, 0.055))
    return ring.union(spoke_x).union(spoke_y).union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tool_wrist")

    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    bearing_black = Material("bearing_black", rgba=(0.015, 0.016, 0.018, 1.0))
    yaw_blue = Material("yaw_blue", rgba=(0.08, 0.20, 0.40, 1.0))
    yoke_orange = Material("yoke_orange", rgba=(0.95, 0.47, 0.12, 1.0))
    spindle_steel = Material("spindle_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    marker_red = Material("marker_red", rgba=(0.85, 0.04, 0.02, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.34, 0.30, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="mount_plate",
    )
    for i, (x, y) in enumerate(((-0.115, -0.095), (-0.115, 0.095), (0.115, -0.095), (0.115, 0.095))):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, y, 0.038), rpy=(0.0, 0.0, 0.0)),
            material=bearing_black,
            name=f"base_bolt_{i}",
        )

    yaw_collar = model.part("yaw_collar")
    yaw_collar.visual(
        mesh_from_cadquery(_yaw_collar_mesh(), "yaw_collar_body", tolerance=0.0008),
        material=yaw_blue,
        name="collar_ring",
    )
    yaw_collar.visual(
        Box((0.046, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, -0.078, 0.090)),
        material=yaw_blue,
        name="pitch_side_pylon_0",
    )
    yaw_collar.visual(
        Cylinder(radius=0.035, length=0.026),
        origin=Origin(xyz=(0.0, -0.078, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yaw_blue,
        name="pitch_bearing_boss_0",
    )
    yaw_collar.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.0, -0.06435, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_black,
        name="pitch_bearing_face_0",
    )
    yaw_collar.visual(
        Box((0.046, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, 0.078, 0.090)),
        material=yaw_blue,
        name="pitch_side_pylon_1",
    )
    yaw_collar.visual(
        Cylinder(radius=0.035, length=0.026),
        origin=Origin(xyz=(0.0, 0.078, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yaw_blue,
        name="pitch_bearing_boss_1",
    )
    yaw_collar.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.0, 0.06435, 0.170), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_black,
        name="pitch_bearing_face_1",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Cylinder(radius=0.020, length=0.160),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="cross_axle",
    )
    pitch_yoke.visual(
        Box((0.140, 0.018, 0.070)),
        origin=Origin(xyz=(0.062, -0.039, 0.000)),
        material=yoke_orange,
        name="fork_arm_0",
    )
    pitch_yoke.visual(
        Box((0.140, 0.018, 0.070)),
        origin=Origin(xyz=(0.062, 0.039, 0.000)),
        material=yoke_orange,
        name="fork_arm_1",
    )
    pitch_yoke.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=yoke_orange,
        name="roll_bearing_shell",
    )
    pitch_yoke.visual(
        Box((0.034, 0.112, 0.074)),
        origin=Origin(xyz=(-0.018, 0.0, 0.055)),
        material=yoke_orange,
        name="rear_bridge",
    )
    pitch_yoke.visual(
        Box((0.034, 0.106, 0.018)),
        origin=Origin(xyz=(0.116, 0.0, 0.035)),
        material=yoke_orange,
        name="top_tie",
    )
    pitch_yoke.visual(
        Box((0.034, 0.106, 0.018)),
        origin=Origin(xyz=(0.116, 0.0, -0.035)),
        material=yoke_orange,
        name="bottom_tie",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="tool_shaft",
    )
    roll_spindle.visual(
        Cylinder(radius=0.045, length=0.022),
        origin=Origin(xyz=(0.153, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="nose_flange",
    )
    roll_spindle.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.173, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="pilot_lip",
    )
    for i, (y, z) in enumerate(((0.000, 0.033), (0.028, -0.016), (-0.028, -0.016))):
        roll_spindle.visual(
            Cylinder(radius=0.0055, length=0.004),
            origin=Origin(xyz=(0.1655, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_black,
            name=f"flange_bolt_{i}",
        )
    roll_spindle.visual(
        Cylinder(radius=0.0065, length=0.005),
        origin=Origin(xyz=(0.166, 0.0, 0.037), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=marker_red,
        name="roll_index",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_collar,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_spindle,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.check("yaw axis is vertical", tuple(yaw.axis) == (0.0, 0.0, 1.0))
    ctx.check("pitch axis is crosswise", tuple(pitch.axis) == (0.0, 1.0, 0.0))
    ctx.check("roll axis follows tool axis", tuple(roll.axis) == (1.0, 0.0, 0.0))

    base = object_model.get_part("base")
    yaw_collar = object_model.get_part("yaw_collar")
    pitch_yoke = object_model.get_part("pitch_yoke")
    roll_spindle = object_model.get_part("roll_spindle")

    ctx.allow_overlap(
        pitch_yoke,
        yaw_collar,
        elem_a="cross_axle",
        elem_b="pitch_bearing_boss_0",
        reason="The pitch axle is intentionally captured inside the side bearing boss.",
    )
    ctx.allow_overlap(
        pitch_yoke,
        yaw_collar,
        elem_a="cross_axle",
        elem_b="pitch_bearing_boss_1",
        reason="The pitch axle is intentionally captured inside the side bearing boss.",
    )
    ctx.allow_overlap(
        pitch_yoke,
        roll_spindle,
        elem_a="roll_bearing_shell",
        elem_b="tool_shaft",
        reason="The roll shaft is intentionally seated in the yoke bearing shell.",
    )

    ctx.expect_gap(
        yaw_collar,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="collar_ring",
        negative_elem="mount_plate",
        name="yaw collar seats on base plate",
    )
    ctx.expect_overlap(
        pitch_yoke,
        yaw_collar,
        axes="xz",
        elem_a="cross_axle",
        elem_b="pitch_bearing_boss_1",
        min_overlap=0.035,
        name="pitch axle lines up with side bearing",
    )
    ctx.expect_overlap(
        pitch_yoke,
        yaw_collar,
        axes="xz",
        elem_a="cross_axle",
        elem_b="pitch_bearing_boss_0",
        min_overlap=0.035,
        name="pitch axle lines up with opposite bearing",
    )
    ctx.expect_within(
        roll_spindle,
        pitch_yoke,
        axes="yz",
        inner_elem="tool_shaft",
        outer_elem="roll_bearing_shell",
        margin=0.0,
        name="roll shaft is centered in bearing shell",
    )
    ctx.expect_overlap(
        roll_spindle,
        pitch_yoke,
        axes="x",
        elem_a="tool_shaft",
        elem_b="roll_bearing_shell",
        min_overlap=0.010,
        name="roll shaft remains inserted in bearing shell",
    )
    ctx.expect_overlap(
        roll_spindle,
        pitch_yoke,
        axes="xz",
        elem_a="tool_shaft",
        elem_b="fork_arm_1",
        min_overlap=0.030,
        name="roll spindle runs through yoke fork",
    )

    flange_rest = ctx.part_element_world_aabb(roll_spindle, elem="nose_flange")
    with ctx.pose({yaw: 0.70}):
        flange_yawed = ctx.part_element_world_aabb(roll_spindle, elem="nose_flange")
    ctx.check(
        "yaw swings nose around vertical axis",
        flange_rest is not None
        and flange_yawed is not None
        and abs(((flange_yawed[0][1] + flange_yawed[1][1]) * 0.5) - ((flange_rest[0][1] + flange_rest[1][1]) * 0.5)) > 0.08,
        details=f"rest={flange_rest}, yawed={flange_yawed}",
    )

    with ctx.pose({pitch: 0.65}):
        flange_pitched = ctx.part_element_world_aabb(roll_spindle, elem="nose_flange")
    ctx.check(
        "pitch tips nose about cross-axis",
        flange_rest is not None
        and flange_pitched is not None
        and abs(((flange_pitched[0][2] + flange_pitched[1][2]) * 0.5) - ((flange_rest[0][2] + flange_rest[1][2]) * 0.5)) > 0.07,
        details=f"rest={flange_rest}, pitched={flange_pitched}",
    )

    mark_rest = ctx.part_element_world_aabb(roll_spindle, elem="roll_index")
    with ctx.pose({roll: math.pi / 2.0}):
        mark_rolled = ctx.part_element_world_aabb(roll_spindle, elem="roll_index")
    ctx.check(
        "roll rotates visible flange index",
        mark_rest is not None
        and mark_rolled is not None
        and abs(((mark_rolled[0][1] + mark_rolled[1][1]) * 0.5) - ((mark_rest[0][1] + mark_rest[1][1]) * 0.5)) > 0.025,
        details=f"rest={mark_rest}, rolled={mark_rolled}",
    )

    return ctx.report()


object_model = build_object_model()
