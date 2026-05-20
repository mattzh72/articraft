from __future__ import annotations

from math import atan2

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


BASE_DEPTH = 0.28
BASE_WIDTH = 0.38
BASE_THICKNESS = 0.018

COUNTERWEIGHT_DEPTH = 0.12
COUNTERWEIGHT_WIDTH = 0.10
COUNTERWEIGHT_THICKNESS = 0.018

PIVOT_BOSS_RADIUS = 0.028
PIVOT_BOSS_HEIGHT = 0.020
PIVOT_X = -0.095
PIVOT_Y = 0.145
PIVOT_Z = BASE_THICKNESS + COUNTERWEIGHT_THICKNESS + PIVOT_BOSS_HEIGHT

LOWER_COLLAR_RADIUS = 0.024
LOWER_COLLAR_HEIGHT = 0.018
LOWER_COLUMN_SIZE = (0.028, 0.052, 0.182)
LOWER_HEAD_SIZE = (0.040, 0.058, 0.028)
ELBOW_ORIGIN = (0.030, 0.0, 0.214)

UPPER_BEAM_LENGTH = 0.102
UPPER_BEAM_SIZE = (UPPER_BEAM_LENGTH, 0.030, 0.024)
TILT_PLATE_SIZE = (0.036, 0.078, 0.010)
TRAY_HINGE_X = 0.120
TRAY_HINGE_Z = 0.005

TRAY_DEPTH = 0.240
TRAY_WIDTH = 0.220
TRAY_PANEL_THICKNESS = 0.008
TRAY_FRONT_LIP_THICKNESS = 0.012
TRAY_FRONT_LIP_HEIGHT = 0.020
TRAY_SIDE_STOP_LENGTH = 0.110
TRAY_SIDE_STOP_THICKNESS = 0.012
TRAY_SIDE_STOP_HEIGHT = 0.014

REST_YAW = atan2(-0.145, 0.038)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_laptop_stand")

    base_finish = model.material("base_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.23, 0.24, 0.26, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=base_finish,
        name="base_plate",
    )
    base.visual(
        Box((COUNTERWEIGHT_DEPTH, COUNTERWEIGHT_WIDTH, COUNTERWEIGHT_THICKNESS)),
        origin=Origin(
            xyz=(
                PIVOT_X - 0.004,
                PIVOT_Y - 0.012,
                BASE_THICKNESS + COUNTERWEIGHT_THICKNESS * 0.5,
            )
        ),
        material=base_finish,
        name="counterweight",
    )
    base.visual(
        Cylinder(radius=PIVOT_BOSS_RADIUS, length=PIVOT_BOSS_HEIGHT),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, PIVOT_Z - PIVOT_BOSS_HEIGHT * 0.5)),
        material=arm_finish,
        name="pivot_boss",
    )
    for foot_x in (-0.095, 0.095):
        for foot_y in (-0.125, 0.125):
            base.visual(
                Cylinder(radius=0.013, length=0.003),
                origin=Origin(xyz=(foot_x, foot_y, 0.0015)),
                material=rubber,
                name=f"foot_{int((foot_x + 0.095) / 0.19)}_{int((foot_y + 0.125) / 0.25)}",
            )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=LOWER_COLLAR_RADIUS, length=LOWER_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, LOWER_COLLAR_HEIGHT * 0.5)),
        material=arm_finish,
        name="base_collar",
    )
    lower_arm.visual(
        Box(LOWER_COLUMN_SIZE),
        origin=Origin(xyz=(0.0, 0.0, LOWER_COLLAR_HEIGHT + LOWER_COLUMN_SIZE[2] * 0.5)),
        material=arm_finish,
        name="column",
    )
    lower_arm.visual(
        Box(LOWER_HEAD_SIZE),
        origin=Origin(xyz=(ELBOW_ORIGIN[0] - LOWER_HEAD_SIZE[0] * 0.5, 0.0, ELBOW_ORIGIN[2])),
        material=arm_finish,
        name="head_block",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.016, 0.040, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, 0.020)),
        material=arm_finish,
        name="elbow_plate",
    )
    upper_arm.visual(
        Box((0.018, 0.024, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.008)),
        material=arm_finish,
        name="elbow_rib",
    )
    upper_arm.visual(
        Box(UPPER_BEAM_SIZE),
        origin=Origin(xyz=(0.010 + UPPER_BEAM_LENGTH * 0.5, 0.0, -0.009)),
        material=arm_finish,
        name="beam",
    )
    upper_arm.visual(
        Box(TILT_PLATE_SIZE),
        origin=Origin(xyz=(TRAY_HINGE_X, 0.0, 0.0)),
        material=tray_finish,
        name="tilt_plate",
    )
    upper_arm.visual(
        Box((0.030, 0.034, 0.020)),
        origin=Origin(xyz=(TRAY_HINGE_X - 0.010, 0.0, -0.015)),
        material=arm_finish,
        name="head_rib",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.036, 0.090, 0.012)),
        origin=Origin(xyz=(0.018, 0.0, 0.006)),
        material=arm_finish,
        name="hinge_pad",
    )
    tray.visual(
        Box((TRAY_DEPTH, TRAY_WIDTH, TRAY_PANEL_THICKNESS)),
        origin=Origin(xyz=(TRAY_DEPTH * 0.5, 0.0, 0.016)),
        material=tray_finish,
        name="tray_panel",
    )
    tray.visual(
        Box((TRAY_FRONT_LIP_THICKNESS, 0.190, TRAY_FRONT_LIP_HEIGHT)),
        origin=Origin(
            xyz=(
                TRAY_DEPTH - TRAY_FRONT_LIP_THICKNESS * 0.5,
                0.0,
                0.020 + TRAY_FRONT_LIP_HEIGHT * 0.5,
            )
        ),
        material=tray_finish,
        name="front_lip",
    )
    for stop_index, stop_y in enumerate((-0.104, 0.104)):
        tray.visual(
            Box((TRAY_SIDE_STOP_LENGTH, TRAY_SIDE_STOP_THICKNESS, TRAY_SIDE_STOP_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.095,
                    stop_y,
                    0.020 + TRAY_SIDE_STOP_HEIGHT * 0.5,
                )
            ),
            material=tray_finish,
            name=f"side_stop_{stop_index}",
        )
    for pad_index, pad_y in enumerate((-0.050, 0.050)):
        tray.visual(
            Box((0.095, 0.018, 0.004)),
            origin=Origin(xyz=(0.112, pad_y, 0.022)),
            material=rubber,
            name=f"pad_{pad_index}",
        )

    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, PIVOT_Z), rpy=(0.0, 0.0, REST_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.4, lower=-0.85, upper=0.65),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=ELBOW_ORIGIN),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=-0.45, upper=0.78),
    )
    model.articulation(
        "tray_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=tray,
        origin=Origin(xyz=(TRAY_HINGE_X, 0.0, TRAY_HINGE_Z), rpy=(0.0, 0.0, -REST_YAW)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-0.45, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    base_pivot = object_model.get_articulation("base_pivot")
    elbow = object_model.get_articulation("elbow")
    tray_tilt = object_model.get_articulation("tray_tilt")

    ctx.expect_within(
        tray,
        base,
        axes="xy",
        margin=0.05,
        name="tray stays centered over the base footprint",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.18,
        name="tray clears the desk base",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({base_pivot: 0.45}):
        swung_tray_pos = ctx.part_world_position(tray)
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.16,
            name="yawed tray still clears the base",
        )
    ctx.check(
        "base pivot swings the tray laterally",
        rest_tray_pos is not None
        and swung_tray_pos is not None
        and (
            ((swung_tray_pos[0] - rest_tray_pos[0]) ** 2 + (swung_tray_pos[1] - rest_tray_pos[1]) ** 2)
            ** 0.5
        )
        > 0.05,
        details=f"rest={rest_tray_pos}, swung={swung_tray_pos}",
    )

    with ctx.pose({elbow: 0.55}):
        raised_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "elbow raises the tray",
        rest_tray_pos is not None
        and raised_tray_pos is not None
        and raised_tray_pos[2] > rest_tray_pos[2] + 0.05,
        details=f"rest={rest_tray_pos}, raised={raised_tray_pos}",
    )

    def _center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_front = ctx.part_element_world_aabb(tray, elem="front_lip")
    rest_hinge = ctx.part_element_world_aabb(tray, elem="hinge_pad")
    with ctx.pose({tray_tilt: 0.35}):
        tilted_front = ctx.part_element_world_aabb(tray, elem="front_lip")
        tilted_hinge = ctx.part_element_world_aabb(tray, elem="hinge_pad")
    rest_front_z = _center_z(rest_front)
    rest_hinge_z = _center_z(rest_hinge)
    tilted_front_z = _center_z(tilted_front)
    tilted_hinge_z = _center_z(tilted_hinge)
    ctx.check(
        "tray hinge pitches the front edge upward",
        rest_front_z is not None
        and rest_hinge_z is not None
        and tilted_front_z is not None
        and tilted_hinge_z is not None
        and tilted_front_z > rest_front_z + 0.025
        and abs(tilted_hinge_z - rest_hinge_z) < 0.01,
        details=(
            f"rest_front_z={rest_front_z}, rest_hinge_z={rest_hinge_z}, "
            f"tilted_front_z={tilted_front_z}, tilted_hinge_z={tilted_hinge_z}"
        ),
    )

    with ctx.pose({base_pivot: 0.25, elbow: 0.35, tray_tilt: 0.20}):
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.14,
            name="combined articulated pose keeps the tray above the base",
        )

    return ctx.report()


object_model = build_object_model()
