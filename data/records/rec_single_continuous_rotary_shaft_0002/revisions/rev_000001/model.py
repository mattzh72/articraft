from __future__ import annotations

import math
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
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
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
# In sdk_hybrid, author visual meshes with cadquery + mesh_from_cadquery.
PI = math.pi

BASE_SIZE = (0.34, 0.16, 0.012)
BASE_CENTER = (0.0, 0.0, 0.006)

MOTOR_FOOT_SIZE = (0.12, 0.072, 0.022)
MOTOR_FOOT_CENTER = (-0.03, -0.026, 0.023)
MOTOR_BODY_CENTER = (-0.03, -0.026, 0.060)
MOTOR_BODY_RADIUS = 0.044
MOTOR_BODY_LENGTH = 0.14
MOTOR_PULLEY_CENTER = (0.055, -0.026, 0.060)
MOTOR_PULLEY_RADIUS = 0.024
MOTOR_PULLEY_LENGTH = 0.018

PEDESTAL_SIZE = (0.05, 0.08, 0.108)
PEDESTAL_CENTER = (0.10, 0.018, 0.060)
SPINDLE_ORIGIN = (0.10, 0.018, 0.110)
BEARING_RADIUS = 0.028
BEARING_LENGTH = 0.09

SPINDLE_SHAFT_LENGTH = 0.19
SPINDLE_SHAFT_RADIUS = 0.012
DRIVEN_PULLEY_CENTER_LOCAL = (-0.045, 0.0, 0.0)
DRIVEN_PULLEY_RADIUS = 0.038
DRIVEN_PULLEY_LENGTH = 0.020
BUFF_WHEEL_CENTER_LOCAL = (0.055, 0.0, 0.0)
BUFF_WHEEL_RADIUS = 0.080
BUFF_WHEEL_LENGTH = 0.030
BUFF_SEAM_CENTER_LOCAL = (0.055, 0.0, 0.095)
BUFF_SEAM_RADIUS = 0.011
BUFF_SEAM_LENGTH = 0.012

MOTOR_TO_SPINDLE_DY = SPINDLE_ORIGIN[1] - MOTOR_PULLEY_CENTER[1]
MOTOR_TO_SPINDLE_DZ = SPINDLE_ORIGIN[2] - MOTOR_PULLEY_CENTER[2]
BELT_SPAN = math.hypot(MOTOR_TO_SPINDLE_DY, MOTOR_TO_SPINDLE_DZ)
BELT_ANGLE = math.atan2(MOTOR_TO_SPINDLE_DZ, MOTOR_TO_SPINDLE_DY)
BELT_NORMAL_Y = -MOTOR_TO_SPINDLE_DZ / BELT_SPAN
BELT_NORMAL_Z = MOTOR_TO_SPINDLE_DY / BELT_SPAN
BELT_OFFSET = 0.028


def _belt_run_center(sign: float) -> tuple[float, float, float]:
    return (
        MOTOR_PULLEY_CENTER[0],
        0.5 * (MOTOR_PULLEY_CENTER[1] + SPINDLE_ORIGIN[1]) + sign * BELT_NORMAL_Y * BELT_OFFSET,
        0.5 * (MOTOR_PULLEY_CENTER[2] + SPINDLE_ORIGIN[2]) + sign * BELT_NORMAL_Z * BELT_OFFSET,
    )


def _make_housing_body_mesh():
    import cadquery as cq

    base = cq.Workplane("XY").box(*BASE_SIZE).translate(BASE_CENTER).edges("|Z").fillet(0.01)
    foot = cq.Workplane("XY").box(*MOTOR_FOOT_SIZE).translate(MOTOR_FOOT_CENTER)
    motor = (
        cq.Workplane("YZ")
        .circle(MOTOR_BODY_RADIUS)
        .extrude(MOTOR_BODY_LENGTH)
        .translate(
            (
                MOTOR_BODY_CENTER[0] - MOTOR_BODY_LENGTH / 2.0,
                MOTOR_BODY_CENTER[1],
                MOTOR_BODY_CENTER[2],
            )
        )
    )
    end_bell = (
        cq.Workplane("YZ")
        .circle(0.047)
        .extrude(0.014)
        .translate((0.034, MOTOR_BODY_CENTER[1], MOTOR_BODY_CENTER[2]))
    )
    terminal_box = cq.Workplane("XY").box(0.028, 0.032, 0.022).translate((-0.052, -0.026, 0.109))
    pedestal = cq.Workplane("XY").box(*PEDESTAL_SIZE).translate(PEDESTAL_CENTER)
    brace = (
        cq.Workplane("XZ")
        .polyline([(0.032, 0.012), (0.032, 0.050), (0.082, 0.104), (0.102, 0.104), (0.102, 0.012)])
        .close()
        .extrude(0.048)
        .translate((0.0, -0.006, 0.0))
    )
    bearing = (
        cq.Workplane("YZ")
        .circle(BEARING_RADIUS)
        .extrude(BEARING_LENGTH)
        .translate((SPINDLE_ORIGIN[0] - BEARING_LENGTH / 2.0, SPINDLE_ORIGIN[1], SPINDLE_ORIGIN[2]))
    )
    belt_guard = (
        cq.Workplane("XY")
        .box(0.016, 0.118, 0.082)
        .translate((0.057, -0.004, 0.084))
        .edges("|X")
        .fillet(0.016)
    )
    return (
        base.union(foot)
        .union(motor)
        .union(end_bell)
        .union(terminal_box)
        .union(pedestal)
        .union(brace)
        .union(bearing)
        .union(belt_guard)
    )


def _make_spindle_steel_mesh():
    import cadquery as cq

    shaft = (
        cq.Workplane("YZ")
        .circle(SPINDLE_SHAFT_RADIUS)
        .extrude(SPINDLE_SHAFT_LENGTH)
        .translate((0.035 - SPINDLE_SHAFT_LENGTH / 2.0, 0.0, 0.0))
    )
    pulley = (
        cq.Workplane("YZ")
        .circle(DRIVEN_PULLEY_RADIUS)
        .extrude(DRIVEN_PULLEY_LENGTH)
        .translate(
            (
                DRIVEN_PULLEY_CENTER_LOCAL[0] - DRIVEN_PULLEY_LENGTH / 2.0,
                DRIVEN_PULLEY_CENTER_LOCAL[1],
                DRIVEN_PULLEY_CENTER_LOCAL[2],
            )
        )
    )
    hub = cq.Workplane("YZ").circle(0.024).extrude(0.026).translate((0.010, 0.0, 0.0))
    inner_flange = cq.Workplane("YZ").circle(0.034).extrude(0.004).translate((0.038, 0.0, 0.0))
    outer_flange = cq.Workplane("YZ").circle(0.034).extrude(0.004).translate((0.068, 0.0, 0.0))
    nut = cq.Workplane("YZ").circle(0.016).extrude(0.010).translate((0.085, 0.0, 0.0))
    knob = cq.Workplane("XY").circle(0.005).extrude(0.012).translate((-0.045, 0.0, 0.026))
    return (
        shaft.union(pulley)
        .union(hub)
        .union(inner_flange)
        .union(outer_flange)
        .union(nut)
        .union(knob)
    )


def _make_buff_wheel_mesh():
    import cadquery as cq

    wheel = (
        cq.Workplane("YZ")
        .circle(BUFF_WHEEL_RADIUS)
        .circle(0.015)
        .extrude(BUFF_WHEEL_LENGTH)
        .translate((BUFF_WHEEL_CENTER_LOCAL[0] - BUFF_WHEEL_LENGTH / 2.0, 0.0, 0.0))
    )
    seam = (
        cq.Workplane("YZ")
        .circle(BUFF_SEAM_RADIUS)
        .extrude(BUFF_SEAM_LENGTH)
        .translate(
            (BUFF_SEAM_CENTER_LOCAL[0] - BUFF_SEAM_LENGTH / 2.0, 0.0, BUFF_SEAM_CENTER_LOCAL[2])
        )
    )
    return wheel.union(seam)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="belt_driven_buffing_wheel", assets=ASSETS)

    model.material("machine_green", rgba=(0.18, 0.31, 0.23, 1.0))
    model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("buff_cloth", rgba=(0.88, 0.83, 0.67, 1.0))
    model.material("belt_black", rgba=(0.09, 0.09, 0.10, 1.0))

    housing = model.part("motor_housing")
    spindle = model.part("spindle")

    housing.visual(
        mesh_from_cadquery(_make_housing_body_mesh(), MESH_DIR / "motor_housing.obj"),
        material="machine_green",
    )
    spindle.visual(
        mesh_from_cadquery(_make_spindle_steel_mesh(), MESH_DIR / "spindle_steel.obj"),
        material="steel",
    )
    spindle.visual(
        mesh_from_cadquery(_make_buff_wheel_mesh(), MESH_DIR / "buff_wheel.obj"),
        material="buff_cloth",
    )

    housing.visual(
        Cylinder(radius=MOTOR_PULLEY_RADIUS, length=MOTOR_PULLEY_LENGTH),
        origin=Origin(xyz=MOTOR_PULLEY_CENTER, rpy=(0.0, PI / 2.0, 0.0)),
        material="steel",
    )
    housing.visual(
        Box((0.018, BELT_SPAN, 0.004)),
        origin=Origin(xyz=_belt_run_center(-1.0), rpy=(BELT_ANGLE, 0.0, 0.0)),
        material="belt_black",
    )
    housing.visual(
        Box((0.018, BELT_SPAN, 0.004)),
        origin=Origin(xyz=_belt_run_center(1.0), rpy=(BELT_ANGLE, 0.0, 0.0)),
        material="belt_black",
    )






    housing.inertial = Inertial.from_geometry(
        Box((0.34, 0.16, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )






    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.082, length=0.16),
        mass=1.9,
        origin=Origin(xyz=(0.03, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
    )

    model.articulation(
        "housing_to_spindle",
        ArticulationType.CONTINUOUS,
        parent="motor_housing",
        child="spindle",
        origin=Origin(xyz=SPINDLE_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "motor_housing",
        "spindle",
        reason="the spindle shaft and driven pulley intentionally occupy the pillow-block bearing envelope",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=64, overlap_tol=0.004, overlap_volume_tol=0.0)
    ctx.expect_joint_motion_axis(
        "housing_to_spindle",
        "spindle",
        world_axis="y",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_aabb_overlap("spindle", "motor_housing", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("spindle", "motor_housing", axes="xy", max_dist=0.16)
    ctx.expect_aabb_gap("spindle", "motor_housing", axis="z", max_gap=0.03, max_penetration=0.12)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
