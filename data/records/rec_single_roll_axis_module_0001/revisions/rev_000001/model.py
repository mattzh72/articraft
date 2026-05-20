from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BASE_LEN = 0.24
BASE_WIDTH = 0.09
BASE_THICKNESS = 0.012
SUPPORT_SPAN = 0.15
SUPPORT_THICKNESS = 0.018
SUPPORT_WIDTH = 0.055
SUPPORT_HEIGHT = 0.074
SUPPORT_TOP_Z = BASE_THICKNESS + SUPPORT_HEIGHT
AXIS_HEIGHT = 0.056
BORE_RADIUS = 0.0185
PHYSICAL_CLEARANCE = 0.019

SHAFT_RADIUS = 0.016
SHAFT_START_X = -0.012
SHAFT_LENGTH = 0.196
TUBE_RADIUS = 0.022
TUBE_INNER_RADIUS = 0.017
TUBE_START_X = 0.028
TUBE_LENGTH = 0.094
FLAG_PAD_CENTER_X = 0.078
FLAG_PAD_CENTER_Z = 0.026
FLAG_FIN_CENTER_X = 0.092
FLAG_FIN_CENTER_Z = 0.038

MOTOR_CAN_START_X = -0.14
MOTOR_CAN_LENGTH = 0.04
MOTOR_RADIUS = 0.02
MOTOR_FLANGE_CENTER_X = -0.095


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(
    radius: float,
    length: float,
    start_x: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("YZ", origin=(start_x, y, z)).circle(radius).extrude(length)


def _support_block(x_pos: float) -> cq.Workplane:
    support = _box(
        (SUPPORT_THICKNESS, SUPPORT_WIDTH, SUPPORT_HEIGHT),
        (x_pos, 0.0, BASE_THICKNESS + SUPPORT_HEIGHT / 2.0),
    )
    bore = _x_cylinder(
        BORE_RADIUS,
        SUPPORT_THICKNESS + 0.004,
        x_pos - (SUPPORT_THICKNESS + 0.004) / 2.0,
        z=AXIS_HEIGHT,
    )
    window = _box(
        (SUPPORT_THICKNESS + 0.002, SUPPORT_WIDTH * 0.42, SUPPORT_HEIGHT * 0.36),
        (x_pos, 0.0, BASE_THICKNESS + SUPPORT_HEIGHT * 0.24),
    )
    return support.cut(bore).cut(window)


def _build_base_frame_shape() -> cq.Workplane:
    frame = _box(
        (BASE_LEN, BASE_WIDTH, BASE_THICKNESS),
        (0.0, 0.0, BASE_THICKNESS / 2.0),
    )
    frame = frame.union(_support_block(-SUPPORT_SPAN / 2.0))
    frame = frame.union(_support_block(SUPPORT_SPAN / 2.0))
    return frame


def _build_motor_shape() -> cq.Workplane:
    motor_can = _x_cylinder(
        MOTOR_RADIUS,
        MOTOR_CAN_LENGTH,
        MOTOR_CAN_START_X,
        z=AXIS_HEIGHT,
    )
    flange = _box(
        (0.008, 0.046, 0.046),
        (MOTOR_FLANGE_CENTER_X, 0.0, AXIS_HEIGHT),
    )
    pedestal = _box(
        (0.016, 0.04, 0.024),
        (MOTOR_FLANGE_CENTER_X, 0.0, BASE_THICKNESS + 0.012),
    )
    brace = _box(
        (0.024, 0.028, 0.01),
        (MOTOR_FLANGE_CENTER_X + 0.008, 0.0, 0.04),
    )
    shaft_stub = _x_cylinder(
        0.0055,
        0.005,
        MOTOR_FLANGE_CENTER_X + 0.001,
        z=AXIS_HEIGHT,
    )
    return motor_can.union(flange).union(pedestal).union(brace).union(shaft_stub)


def _build_sensor_body_shape() -> cq.Workplane:
    shaft = _x_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, SHAFT_START_X)
    tube_outer = _x_cylinder(TUBE_RADIUS, TUBE_LENGTH, TUBE_START_X)
    tube_inner = _x_cylinder(TUBE_INNER_RADIUS, TUBE_LENGTH + 0.002, TUBE_START_X - 0.001)
    tube = tube_outer.cut(tube_inner)
    drive_collar = _x_cylinder(0.024, 0.014, -0.04)
    retaining_collar = _x_cylinder(0.02, 0.012, 0.172)
    return shaft.union(tube).union(drive_collar).union(retaining_collar)


def _build_sensor_flag_shape() -> cq.Workplane:
    pad = _box((0.036, 0.018, 0.008), (FLAG_PAD_CENTER_X, 0.0, FLAG_PAD_CENTER_Z))
    fin = _box((0.014, 0.006, 0.02), (FLAG_FIN_CENTER_X, 0.0, FLAG_FIN_CENTER_Z))
    return pad.union(fin)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motorized_roll_stage", assets=ASSETS)

    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("motor_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("tube_black", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("signal_amber", rgba=(0.87, 0.52, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_frame_shape(), "roll_stage_base_frame.obj", assets=ASSETS),
        material="machined_aluminum",
    )
    base.visual(
        mesh_from_cadquery(_build_motor_shape(), "roll_stage_motor.obj", assets=ASSETS),
        material="motor_black",
    )

    lower_support_height = AXIS_HEIGHT - PHYSICAL_CLEARANCE - BASE_THICKNESS
    upper_support_height = SUPPORT_TOP_Z - (AXIS_HEIGHT + PHYSICAL_CLEARANCE)
    for x_pos in (-SUPPORT_SPAN / 2.0, SUPPORT_SPAN / 2.0):
        pass

    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_WIDTH, SUPPORT_TOP_Z)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_TOP_Z / 2.0)),
    )

    sensor_tube = model.part("sensor_tube")
    sensor_tube.visual(
        mesh_from_cadquery(_build_sensor_body_shape(), "sensor_tube_body.obj", assets=ASSETS),
        material="tube_black",
    )
    sensor_tube.visual(
        mesh_from_cadquery(_build_sensor_flag_shape(), "sensor_tube_flag.obj", assets=ASSETS),
        material="signal_amber",
    )



    sensor_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_RADIUS, length=0.15),
        mass=0.42,
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "tube_roll",
        ArticulationType.REVOLUTE,
        parent=base,
        child=sensor_tube,
        origin=Origin(xyz=(-SUPPORT_SPAN / 2.0, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.2,
            upper=1.2,
            effort=6.0,
            velocity=2.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.expect_aabb_overlap("sensor_tube", "base", axes="xy", min_overlap=0.04)
    ctx.expect_joint_motion_axis(
        "tube_roll",
        "sensor_tube",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
