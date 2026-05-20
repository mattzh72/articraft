from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.082
BASE_HEIGHT = 0.018
BASE_PEDESTAL_RADIUS = 0.052
BASE_PEDESTAL_HEIGHT = 0.034
BASE_BEARING_RADIUS = 0.045
BASE_BEARING_HEIGHT = 0.010

YAW_ORIGIN_Z = BASE_HEIGHT + BASE_PEDESTAL_HEIGHT + BASE_BEARING_HEIGHT

YAW_STAGE_RADIUS = 0.053
YAW_STAGE_HEIGHT = 0.020
PITCH_MOUNT_X = 0.044
PITCH_AXIS_Z = 0.036
YAW_CHEEK_Y = 0.052
YAW_CHEEK_THICKNESS = 0.014

PITCH_SIDE_Y = 0.031
PITCH_SIDE_THICKNESS = 0.012
ROLL_AXIS_X = 0.066

ROLL_BODY_RADIUS = 0.021
ROLL_FRONT_RADIUS = 0.024


def _x_cylinder(radius: float, length: float, center_x: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x - length / 2.0, 0.0, 0.0))
    )


def _make_base_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT)
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(BASE_PEDESTAL_RADIUS)
        .extrude(BASE_PEDESTAL_HEIGHT)
    )
    bearing = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT + BASE_PEDESTAL_HEIGHT)
        .circle(BASE_BEARING_RADIUS)
        .extrude(BASE_BEARING_HEIGHT)
    )
    cable_boss = (
        cq.Workplane("XY")
        .box(0.040, 0.024, 0.018)
        .translate((-0.030, 0.0, BASE_HEIGHT + 0.009))
    )
    return flange.union(pedestal).union(bearing).union(cable_boss)


def _make_yaw_stage_shape() -> cq.Workplane:
    lower_drum = cq.Workplane("XY").circle(YAW_STAGE_RADIUS).extrude(YAW_STAGE_HEIGHT)
    spine = cq.Workplane("XY").box(0.058, 0.032, 0.020).translate((0.022, 0.000, 0.026))
    left_riser = cq.Workplane("XY").box(0.026, 0.024, 0.028).translate(
        (PITCH_MOUNT_X - 0.010, 0.034, 0.018)
    )
    right_riser = cq.Workplane("XY").box(0.026, 0.024, 0.028).translate(
        (PITCH_MOUNT_X - 0.010, -0.034, 0.018)
    )

    cheek_left = cq.Workplane("XY").box(0.026, YAW_CHEEK_THICKNESS, 0.048).translate(
        (PITCH_MOUNT_X, YAW_CHEEK_Y, PITCH_AXIS_Z)
    )
    cheek_right = cq.Workplane("XY").box(0.026, YAW_CHEEK_THICKNESS, 0.048).translate(
        (PITCH_MOUNT_X, -YAW_CHEEK_Y, PITCH_AXIS_Z)
    )

    boss_left = (
        cq.Workplane("XZ")
        .circle(0.022)
        .extrude(YAW_CHEEK_THICKNESS)
        .translate((PITCH_MOUNT_X, YAW_CHEEK_Y - YAW_CHEEK_THICKNESS / 2.0, PITCH_AXIS_Z))
    )
    boss_right = (
        cq.Workplane("XZ")
        .circle(0.022)
        .extrude(YAW_CHEEK_THICKNESS)
        .translate((PITCH_MOUNT_X, -YAW_CHEEK_Y - YAW_CHEEK_THICKNESS / 2.0, PITCH_AXIS_Z))
    )

    return (
        lower_drum.union(spine)
        .union(left_riser)
        .union(right_riser)
        .union(cheek_left)
        .union(cheek_right)
        .union(boss_left)
        .union(boss_right)
    )


def _make_pitch_side_plate(y_center: float) -> cq.Workplane:
    blank = (
        cq.Workplane("XZ")
        .workplane(offset=y_center - PITCH_SIDE_THICKNESS / 2.0)
        .rect(0.082, 0.070)
        .extrude(PITCH_SIDE_THICKNESS)
        .translate((0.026, 0.000, 0.000))
    )
    pitch_lobe = (
        cq.Workplane("XZ")
        .workplane(offset=y_center - PITCH_SIDE_THICKNESS / 2.0)
        .circle(0.028)
        .extrude(PITCH_SIDE_THICKNESS)
    )
    roll_lobe = (
        cq.Workplane("XZ")
        .workplane(offset=y_center - PITCH_SIDE_THICKNESS / 2.0)
        .center(ROLL_AXIS_X, 0.0)
        .circle(0.040)
        .extrude(PITCH_SIDE_THICKNESS)
    )

    pitch_bore = (
        cq.Workplane("XZ")
        .workplane(offset=y_center - PITCH_SIDE_THICKNESS / 2.0)
        .circle(0.015)
        .extrude(PITCH_SIDE_THICKNESS)
    )
    roll_window = (
        cq.Workplane("XZ")
        .workplane(offset=y_center - PITCH_SIDE_THICKNESS / 2.0)
        .center(ROLL_AXIS_X, 0.0)
        .slot2D(0.064, 0.050)
        .extrude(PITCH_SIDE_THICKNESS)
    )
    front_cut = cq.Workplane("XY").box(0.050, 0.020, 0.050).translate((0.096, y_center, 0.030))

    return blank.union(pitch_lobe).union(roll_lobe).cut(pitch_bore).cut(roll_window).cut(front_cut)


def _make_pitch_cradle_shape() -> cq.Workplane:
    left_plate = _make_pitch_side_plate(PITCH_SIDE_Y)
    right_plate = _make_pitch_side_plate(-PITCH_SIDE_Y)
    saddle_block = cq.Workplane("XY").box(0.060, 0.074, 0.024).translate(
        (ROLL_AXIS_X - 0.004, 0.000, -0.028)
    )
    saddle_cut = _x_cylinder(ROLL_BODY_RADIUS, 0.072, ROLL_AXIS_X).translate((0.0, 0.0, 0.0))
    lower_bridge = saddle_block.cut(saddle_cut)
    top_strap = cq.Workplane("XY").box(0.034, 0.074, 0.014).translate(
        (0.024, 0.000, 0.030)
    )
    return left_plate.union(right_plate).union(lower_bridge).union(top_strap)


def _make_roll_cartridge_shape() -> cq.Workplane:
    rear_cap = _x_cylinder(0.014, 0.014, -0.045)
    rear_motor = _x_cylinder(0.018, 0.022, -0.028)
    barrel = _x_cylinder(ROLL_BODY_RADIUS, 0.050, 0.008)
    front_flange = _x_cylinder(ROLL_FRONT_RADIUS, 0.014, 0.040)
    tool_nose = _x_cylinder(0.012, 0.022, 0.058)
    service_lug = cq.Workplane("XY").box(0.020, 0.010, 0.014).translate(
        (-0.016, 0.000, 0.027)
    )
    return (
        rear_cap.union(rear_motor)
        .union(barrel)
        .union(front_flange)
        .union(tool_nose)
        .union(service_lug)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cartridge_yoke_wrist")

    base_finish = model.material("base_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    yaw_finish = model.material("yaw_finish", rgba=(0.15, 0.16, 0.18, 1.0))
    cradle_finish = model.material("cradle_finish", rgba=(0.70, 0.73, 0.76, 1.0))
    roll_finish = model.material("roll_finish", rgba=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "wrist_base"),
        material=base_finish,
        name="base_shell",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_stage_shape(), "wrist_yaw_stage"),
        material=yaw_finish,
        name="yaw_stage_shell",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_make_pitch_cradle_shape(), "wrist_pitch_cradle"),
        material=cradle_finish,
        name="pitch_cradle_shell",
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        mesh_from_cadquery(_make_roll_cartridge_shape(), "wrist_roll_cartridge"),
        material=roll_finish,
        name="roll_cartridge_shell",
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.1,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(PITCH_MOUNT_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-1.05,
            upper=1.10,
        ),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_cartridge,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=4.5,
            lower=-3.0,
            upper=3.0,
        ),
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
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_cartridge = object_model.get_part("roll_cartridge")

    yaw_joint = object_model.get_articulation("base_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")
    roll_joint = object_model.get_articulation("pitch_to_roll")

    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        reason="The yaw clevis and pitch trunnion are represented as simplified solid bearing shells instead of thin hollow housings.",
    )
    ctx.allow_overlap(
        pitch_cradle,
        roll_cartridge,
        reason="The roll cartridge is shown nested inside the yoke as a compact cartridge-bearing fit, with the cradle modeled as a solid shell proxy.",
    )

    ctx.check(
        "wrist part count",
        len(object_model.parts) == 4,
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "wrist articulation count",
        len(object_model.articulations) == 3,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, roll_joint: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.0005,
            name="yaw stage seats on the grounded base bearing",
        )
        ctx.expect_within(
            pitch_cradle,
            yaw_stage,
            axes="y",
            margin=0.012,
            name="pitch cradle sits between the yaw cheeks",
        )
        ctx.expect_within(
            roll_cartridge,
            pitch_cradle,
            axes="yz",
            margin=0.020,
            name="roll cartridge stays captured within the cradle envelope",
        )
        ctx.expect_overlap(
            roll_cartridge,
            pitch_cradle,
            axes="yz",
            min_overlap=0.030,
            name="roll cartridge shares the cradle bearing window region",
        )

    rest_pos = ctx.part_world_position(roll_cartridge)
    with ctx.pose({pitch_joint: 0.85}):
        pitched_pos = ctx.part_world_position(roll_cartridge)
    ctx.check(
        "positive pitch raises the roll cartridge",
        rest_pos is not None
        and pitched_pos is not None
        and pitched_pos[2] > rest_pos[2] + 0.030,
        details=f"rest={rest_pos}, pitched={pitched_pos}",
    )

    with ctx.pose({yaw_joint: 0.70}):
        yawed_pos = ctx.part_world_position(roll_cartridge)
    ctx.check(
        "positive yaw swings the tool axis toward +y",
        rest_pos is not None
        and yawed_pos is not None
        and yawed_pos[1] > rest_pos[1] + 0.030,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    rest_aabb = ctx.part_element_world_aabb(roll_cartridge, elem="roll_cartridge_shell")
    with ctx.pose({roll_joint: pi / 2.0}):
        rolled_aabb = ctx.part_element_world_aabb(roll_cartridge, elem="roll_cartridge_shell")
    ctx.check(
        "positive roll rotates the service lug off the top of the cartridge",
        rest_aabb is not None
        and rolled_aabb is not None
        and rolled_aabb[1][2] < rest_aabb[1][2] - 0.008
        and rolled_aabb[0][1] < rest_aabb[0][1] - 0.008,
        details=f"rest_aabb={rest_aabb}, rolled_aabb={rolled_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
