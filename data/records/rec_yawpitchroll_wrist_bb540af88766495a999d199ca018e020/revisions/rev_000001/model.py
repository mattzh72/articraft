from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.260
BASE_D = 0.200
BASE_H = 0.054
BASE_RING_R = 0.094
BASE_RING_H = 0.016

YAW_TABLE_R = 0.086
YAW_TABLE_T = 0.016
YAW_CORE_W = 0.150
YAW_CORE_D = 0.090
YAW_CORE_H = 0.040
REAR_BRIDGE_W = 0.210
REAR_BRIDGE_D = 0.040
REAR_BRIDGE_H = 0.040
TOWER_X = 0.110
TOWER_THK = 0.028
TOWER_DEPTH = 0.102
TOWER_H = 0.170
TOWER_BORE_R = 0.0210
PITCH_AXIS_Z = 0.145

PITCH_FRAME_LEN = 0.076
PITCH_FRAME_OUTER_R = 0.066
PITCH_FRAME_BORE_R = 0.058
TRUNNION_R = 0.0160
TRUNNION_FLANGE_R = 0.024
TRUNNION_FLANGE_T = 0.008
TRUNNION_SHAFT_LEN = TOWER_X + (TOWER_THK / 2.0) - PITCH_FRAME_OUTER_R + 0.001

SPINDLE_DRUM_R = 0.044
SPINDLE_DRUM_LEN = 0.084
SPINDLE_REAR_STUB_R = 0.032
SPINDLE_REAR_STUB_LEN = 0.018
SPINDLE_THRUST_R = 0.060
SPINDLE_THRUST_T = 0.006
FACEPLATE_R = 0.060
FACEPLATE_T = 0.010


def _base_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_W, BASE_D, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.014)
    )
    bearing_ring = cq.Workplane("XY").circle(BASE_RING_R).extrude(BASE_RING_H).translate((0.0, 0.0, BASE_H))
    return body.union(bearing_ring)


def _yaw_stage_shape() -> cq.Workplane:
    table = cq.Workplane("XY").circle(YAW_TABLE_R).extrude(YAW_TABLE_T)
    core = (
        cq.Workplane("XY")
        .box(YAW_CORE_W, YAW_CORE_D, YAW_CORE_H, centered=(True, True, False))
        .translate((0.0, 0.0, YAW_TABLE_T))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(REAR_BRIDGE_W, REAR_BRIDGE_D, REAR_BRIDGE_H, centered=(True, True, False))
        .translate((0.0, -0.030, YAW_TABLE_T))
    )
    left_tower = (
        cq.Workplane("XY")
        .box(TOWER_THK, TOWER_DEPTH, TOWER_H, centered=(True, True, False))
        .translate((-TOWER_X, 0.0, YAW_TABLE_T))
    )
    right_tower = (
        cq.Workplane("XY")
        .box(TOWER_THK, TOWER_DEPTH, TOWER_H, centered=(True, True, False))
        .translate((TOWER_X, 0.0, YAW_TABLE_T))
    )
    pitch_bore = (
        cq.Workplane("YZ")
        .circle(TOWER_BORE_R)
        .extrude(0.300, both=True)
        .translate((0.0, 0.0, PITCH_AXIS_Z))
    )
    return table.union(core).union(rear_bridge).union(left_tower).union(right_tower).cut(pitch_bore)


def _pitch_frame_shape() -> cq.Workplane:
    outer_shell = cq.Workplane("XZ").circle(PITCH_FRAME_OUTER_R).extrude(PITCH_FRAME_LEN / 2.0, both=True)
    inner_bore = (
        cq.Workplane("XZ")
        .circle(PITCH_FRAME_BORE_R)
        .extrude((PITCH_FRAME_LEN / 2.0) + 0.004, both=True)
    )
    right_trunnion = (
        cq.Workplane("YZ")
        .circle(TRUNNION_R)
        .extrude(TRUNNION_SHAFT_LEN)
        .translate((PITCH_FRAME_OUTER_R, 0.0, 0.0))
    )
    left_trunnion = (
        cq.Workplane("YZ")
        .circle(TRUNNION_R)
        .extrude(TRUNNION_SHAFT_LEN)
        .translate((-PITCH_FRAME_OUTER_R - TRUNNION_SHAFT_LEN, 0.0, 0.0))
    )
    right_boss = (
        cq.Workplane("XY")
        .box(0.020, 0.052, 0.052)
        .translate((PITCH_FRAME_OUTER_R + 0.008, 0.0, 0.0))
    )
    left_boss = (
        cq.Workplane("XY")
        .box(0.020, 0.052, 0.052)
        .translate((-PITCH_FRAME_OUTER_R - 0.008, 0.0, 0.0))
    )
    right_flange = (
        cq.Workplane("YZ")
        .circle(TRUNNION_FLANGE_R)
        .extrude(TRUNNION_FLANGE_T)
        .translate((PITCH_FRAME_OUTER_R + TRUNNION_SHAFT_LEN, 0.0, 0.0))
    )
    left_flange = (
        cq.Workplane("YZ")
        .circle(TRUNNION_FLANGE_R)
        .extrude(TRUNNION_FLANGE_T)
        .translate((-PITCH_FRAME_OUTER_R - TRUNNION_SHAFT_LEN - TRUNNION_FLANGE_T, 0.0, 0.0))
    )
    return (
        outer_shell.cut(inner_bore)
        .union(right_boss)
        .union(left_boss)
        .union(right_trunnion)
        .union(left_trunnion)
        .union(right_flange)
        .union(left_flange)
    )


def _roll_spindle_shape() -> cq.Workplane:
    drum = cq.Workplane("XZ").circle(SPINDLE_DRUM_R).extrude(SPINDLE_DRUM_LEN / 2.0, both=True)
    thrust_collar = (
        cq.Workplane("XZ")
        .circle(SPINDLE_THRUST_R)
        .extrude(SPINDLE_THRUST_T)
        .translate((0.0, (PITCH_FRAME_LEN / 2.0) + 0.001, 0.0))
    )
    rear_stub = (
        cq.Workplane("XZ")
        .circle(SPINDLE_REAR_STUB_R)
        .extrude(SPINDLE_REAR_STUB_LEN)
        .translate((0.0, -(SPINDLE_DRUM_LEN / 2.0) - SPINDLE_REAR_STUB_LEN + 0.001, 0.0))
    )
    return drum.union(thrust_collar).union(rear_stub)


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_yoke_spindle_wrist")

    model.material("base_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("bearing_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("dark_polymer", rgba=(0.14, 0.15, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_body"),
        material="base_dark",
        name="base_body",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_shape(), "yaw_stage_structure"),
        material="machined_aluminum",
        name="yaw_stage_structure",
    )
    yaw_stage.visual(
        Box((0.038, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, -0.058, 0.040)),
        material="dark_polymer",
        name="yaw_rear_pod",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_pitch_frame_shape(), "pitch_frame_body"),
        material="machined_aluminum",
        name="pitch_frame_body",
    )
    pitch_frame.visual(
        Cylinder(radius=TRUNNION_FLANGE_R, length=TRUNNION_FLANGE_T),
        origin=Origin(xyz=(TOWER_X + (TOWER_THK / 2.0) + (TRUNNION_FLANGE_T / 2.0), 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="bearing_steel",
        name="right_pitch_retain",
    )
    pitch_frame.visual(
        Cylinder(radius=TRUNNION_FLANGE_R, length=TRUNNION_FLANGE_T),
        origin=Origin(xyz=(-(TOWER_X + (TOWER_THK / 2.0) + (TRUNNION_FLANGE_T / 2.0)), 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="bearing_steel",
        name="left_pitch_retain",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=SPINDLE_DRUM_R, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="bearing_steel",
        name="roll_spindle_body",
    )
    roll_spindle.visual(
        Cylinder(radius=SPINDLE_REAR_STUB_R, length=0.020),
        origin=Origin(xyz=(0.0, -0.049, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="bearing_steel",
        name="rear_stub",
    )
    roll_spindle.visual(
        Cylinder(radius=SPINDLE_THRUST_R, length=0.008),
        origin=Origin(xyz=(0.0, 0.042, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="bearing_steel",
        name="front_thrust",
    )
    roll_spindle.visual(
        Cylinder(radius=FACEPLATE_R, length=FACEPLATE_T),
        origin=Origin(
            xyz=(0.0, 0.050, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="bearing_steel",
        name="faceplate",
    )
    roll_spindle.visual(
        Box((0.012, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, 0.044)),
        material="dark_polymer",
        name="roll_key",
    )

    model.articulation(
        "base_to_yaw_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + BASE_RING_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=30.0, velocity=2.5),
    )
    model.articulation(
        "yaw_stage_to_pitch_frame",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.05, upper=1.05, effort=20.0, velocity=2.0),
    )
    model.articulation(
        "pitch_frame_to_roll_spindle",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=12.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")

    yaw_joint = object_model.get_articulation("base_to_yaw_stage")
    pitch_joint = object_model.get_articulation("yaw_stage_to_pitch_frame")
    roll_joint = object_model.get_articulation("pitch_frame_to_roll_spindle")

    yaw_rear_pod = yaw_stage.get_visual("yaw_rear_pod")
    faceplate = roll_spindle.get_visual("faceplate")
    roll_key = roll_spindle.get_visual("roll_key")

    ctx.check(
        "yaw joint uses vertical axis",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch joint uses horizontal x axis",
        tuple(pitch_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )
    ctx.check(
        "roll joint follows spindle axis",
        tuple(roll_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={roll_joint.axis}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, roll_joint: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            base,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="yaw table seats on the grounded base",
        )
        ctx.expect_overlap(
            yaw_stage,
            base,
            axes="xy",
            min_overlap=0.120,
            name="yaw stage stays centered over the base footprint",
        )
        ctx.expect_overlap(
            pitch_frame,
            yaw_stage,
            axes="x",
            min_overlap=0.110,
            name="pitch frame trunnions stay captured within the yaw tower span",
        )
        ctx.expect_within(
            pitch_frame,
            yaw_stage,
            axes="y",
            margin=0.012,
            name="pitch frame stays centered between the yaw towers",
        )
        ctx.expect_within(
            roll_spindle,
            pitch_frame,
            axes="xz",
            margin=0.012,
            name="roll spindle stays centered inside the pitch frame bore",
        )
        ctx.expect_overlap(
            roll_spindle,
            pitch_frame,
            axes="y",
            min_overlap=0.070,
            name="roll spindle retains insertion through the pitch frame",
        )

        yaw_rest = _aabb_center(ctx.part_element_world_aabb(yaw_stage, elem=yaw_rear_pod))
        faceplate_rest = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=faceplate))
        key_rest = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=roll_key))

    with ctx.pose({yaw_joint: 1.0}):
        yaw_open = _aabb_center(ctx.part_element_world_aabb(yaw_stage, elem=yaw_rear_pod))
    ctx.check(
        "positive yaw swings the rear pod toward +x",
        yaw_rest is not None and yaw_open is not None and yaw_open[0] > yaw_rest[0] + 0.035,
        details=f"rest={yaw_rest}, moved={yaw_open}",
    )

    with ctx.pose({pitch_joint: 0.8}):
        faceplate_pitched = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=faceplate))
    ctx.check(
        "positive pitch raises the faceplate",
        faceplate_rest is not None
        and faceplate_pitched is not None
        and faceplate_pitched[2] > faceplate_rest[2] + 0.030,
        details=f"rest={faceplate_rest}, pitched={faceplate_pitched}",
    )

    with ctx.pose({roll_joint: 1.0}):
        key_rolled = _aabb_center(ctx.part_element_world_aabb(roll_spindle, elem=roll_key))
    ctx.check(
        "positive roll rotates the keyed feature toward +x",
        key_rest is not None and key_rolled is not None and key_rolled[0] > key_rest[0] + 0.010,
        details=f"rest={key_rest}, rolled={key_rolled}",
    )

    with ctx.pose({pitch_joint: -0.8}):
        ctx.expect_gap(
            roll_spindle,
            base,
            axis="z",
            min_gap=0.055,
            name="pitched spindle stays well above the grounded base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
