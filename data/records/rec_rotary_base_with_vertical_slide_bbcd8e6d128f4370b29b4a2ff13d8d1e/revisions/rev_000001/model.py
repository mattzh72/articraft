from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.17
BASE_THICK = 0.018
BASE_COLLAR_RADIUS = 0.062
BASE_COLLAR_HEIGHT = 0.026
BASE_STACK_TOP = BASE_THICK + BASE_COLLAR_HEIGHT

YAW_JOINT_Z = BASE_STACK_TOP

TURNTABLE_RADIUS = 0.10
TURNTABLE_BOTTOM = 0.000
TURNTABLE_THICK = 0.018
TURNTABLE_HUB_RADIUS = 0.046
TURNTABLE_HUB_HEIGHT = 0.040

SUPPORT_Y = 0.030
PEDESTAL_W = 0.120
PEDESTAL_D = 0.085
PEDESTAL_BOTTOM = TURNTABLE_BOTTOM + TURNTABLE_THICK
PEDESTAL_H = 0.055

GUIDE_BASE_Z = PEDESTAL_BOTTOM + PEDESTAL_H - 0.010
GUIDE_BACK_W = 0.072
GUIDE_BACK_D = 0.032
GUIDE_BACK_H = 0.250
GUIDE_BACK_Y = SUPPORT_Y - 0.010

GUIDE_RAIL_W = 0.024
GUIDE_RAIL_D = 0.024
GUIDE_RAIL_H = 0.210
GUIDE_RAIL_X = 0.030
GUIDE_RAIL_Y = SUPPORT_Y + 0.014
GUIDE_RAIL_BOTTOM = PEDESTAL_BOTTOM + 0.018

TOP_BRIDGE_W = 0.074
TOP_BRIDGE_D = 0.020
TOP_BRIDGE_H = 0.018
TOP_BRIDGE_Y = SUPPORT_Y + 0.002
TOP_BRIDGE_BOTTOM = GUIDE_RAIL_BOTTOM + GUIDE_RAIL_H - 0.008

CARRIAGE_CENTER_Z = 0.195
CARRIAGE_TRAVEL = 0.085
CARRIAGE_FRONT_W = 0.110
CARRIAGE_FRONT_D = 0.012
CARRIAGE_FRONT_H = 0.072
CARRIAGE_FRONT_Y = 0.034
CARRIAGE_SHOE_W = 0.014
CARRIAGE_SHOE_D = 0.048
CARRIAGE_SHOE_H = 0.072
CARRIAGE_SHOE_X = 0.050
CARRIAGE_SHOE_Y = 0.006
CARRIAGE_FACE_PAD_W = 0.050
CARRIAGE_FACE_PAD_D = 0.010
CARRIAGE_FACE_PAD_H = 0.038
CARRIAGE_FACE_PAD_Y = 0.045

REST_PAD_W = 0.016
REST_PAD_D = 0.024
REST_PAD_H = 0.014
REST_PAD_X = CARRIAGE_SHOE_X
REST_PAD_Y = SUPPORT_Y + 0.002
REST_PAD_BOTTOM = CARRIAGE_CENTER_Z - (CARRIAGE_SHOE_H / 2.0) - REST_PAD_H


def _make_base_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICK)
    collar = cq.Workplane("XY").circle(BASE_COLLAR_RADIUS).extrude(BASE_STACK_TOP)
    bolt_cutters = (
        cq.Workplane("XY")
        .polarArray(0.115, 0.0, 360.0, 3)
        .circle(0.007)
        .extrude(BASE_STACK_TOP + 0.004)
    )
    return base_plate.union(collar).cut(bolt_cutters)


def _make_stage_shell() -> cq.Workplane:
    turntable = (
        cq.Workplane("XY")
        .workplane(offset=TURNTABLE_BOTTOM)
        .circle(TURNTABLE_RADIUS)
        .extrude(TURNTABLE_THICK)
    )
    hub = (
        cq.Workplane("XY")
        .workplane(offset=TURNTABLE_BOTTOM)
        .circle(TURNTABLE_HUB_RADIUS)
        .extrude(TURNTABLE_HUB_HEIGHT)
    )
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=PEDESTAL_BOTTOM)
        .center(0.0, SUPPORT_Y)
        .box(PEDESTAL_W, PEDESTAL_D, PEDESTAL_H, centered=(True, True, False))
    )
    return turntable.union(hub).union(pedestal)


def _make_guide_runner() -> cq.Workplane:
    backplate = (
        cq.Workplane("XY")
        .workplane(offset=GUIDE_BASE_Z)
        .center(0.0, GUIDE_BACK_Y)
        .box(GUIDE_BACK_W, GUIDE_BACK_D, GUIDE_BACK_H, centered=(True, True, False))
    )
    left_rail = (
        cq.Workplane("XY")
        .workplane(offset=GUIDE_RAIL_BOTTOM)
        .center(-GUIDE_RAIL_X, GUIDE_RAIL_Y)
        .box(GUIDE_RAIL_W, GUIDE_RAIL_D, GUIDE_RAIL_H, centered=(True, True, False))
    )
    right_rail = (
        cq.Workplane("XY")
        .workplane(offset=GUIDE_RAIL_BOTTOM)
        .center(GUIDE_RAIL_X, GUIDE_RAIL_Y)
        .box(GUIDE_RAIL_W, GUIDE_RAIL_D, GUIDE_RAIL_H, centered=(True, True, False))
    )
    top_bridge = (
        cq.Workplane("XY")
        .workplane(offset=TOP_BRIDGE_BOTTOM)
        .center(0.0, TOP_BRIDGE_Y)
        .box(TOP_BRIDGE_W, TOP_BRIDGE_D, TOP_BRIDGE_H, centered=(True, True, False))
    )
    left_rest_pad = (
        cq.Workplane("XY")
        .workplane(offset=REST_PAD_BOTTOM)
        .center(-REST_PAD_X, REST_PAD_Y)
        .box(REST_PAD_W, REST_PAD_D, REST_PAD_H, centered=(True, True, False))
    )
    right_rest_pad = (
        cq.Workplane("XY")
        .workplane(offset=REST_PAD_BOTTOM)
        .center(REST_PAD_X, REST_PAD_Y)
        .box(REST_PAD_W, REST_PAD_D, REST_PAD_H, centered=(True, True, False))
    )
    return (
        backplate.union(left_rail)
        .union(right_rail)
        .union(top_bridge)
        .union(left_rest_pad)
        .union(right_rest_pad)
    )


def _make_carriage_shape() -> cq.Workplane:
    front_plate = (
        cq.Workplane("XY")
        .center(0.0, CARRIAGE_FRONT_Y)
        .box(CARRIAGE_FRONT_W, CARRIAGE_FRONT_D, CARRIAGE_FRONT_H)
    )
    left_shoe = (
        cq.Workplane("XY")
        .center(-CARRIAGE_SHOE_X, CARRIAGE_SHOE_Y)
        .box(CARRIAGE_SHOE_W, CARRIAGE_SHOE_D, CARRIAGE_SHOE_H)
    )
    right_shoe = (
        cq.Workplane("XY")
        .center(CARRIAGE_SHOE_X, CARRIAGE_SHOE_Y)
        .box(CARRIAGE_SHOE_W, CARRIAGE_SHOE_D, CARRIAGE_SHOE_H)
    )
    face_pad = (
        cq.Workplane("XY")
        .center(0.0, CARRIAGE_FACE_PAD_Y)
        .box(CARRIAGE_FACE_PAD_W, CARRIAGE_FACE_PAD_D, CARRIAGE_FACE_PAD_H)
    )
    return front_plate.union(left_shoe).union(right_shoe).union(face_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_pedestal_runner")

    model.material("base_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("stage_gray", rgba=(0.39, 0.42, 0.45, 1.0))
    model.material("runner_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_orange", rgba=(0.84, 0.46, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_shell"),
        material="base_gray",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_RADIUS * 2.0, BASE_RADIUS * 2.0, BASE_STACK_TOP)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_STACK_TOP / 2.0)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_make_stage_shell(), "stage_shell"),
        material="stage_gray",
        name="stage_shell",
    )
    lower_stage.visual(
        mesh_from_cadquery(_make_guide_runner(), "guide_runner"),
        material="runner_steel",
        name="guide_runner",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Box((0.20, 0.17, 0.32)),
        mass=11.0,
        origin=Origin(xyz=(0.0, SUPPORT_Y * 0.5, 0.16)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage_shell"),
        material="carriage_orange",
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.11, 0.06, 0.072)),
        mass=2.2,
        origin=Origin(),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.7,
            upper=2.7,
            effort=45.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=lower_stage,
        child=carriage,
        origin=Origin(xyz=(0.0, SUPPORT_Y, CARRIAGE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=240.0,
            velocity=0.20,
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
    lower_stage = object_model.get_part("lower_stage")
    carriage = object_model.get_part("carriage")
    base_yaw = object_model.get_articulation("base_yaw")
    carriage_lift = object_model.get_articulation("carriage_lift")

    ctx.check(
        "base yaw uses a vertical revolute axis",
        base_yaw.articulation_type == ArticulationType.REVOLUTE and base_yaw.axis == (0.0, 0.0, 1.0),
        details=f"type={base_yaw.articulation_type}, axis={base_yaw.axis}",
    )
    ctx.check(
        "carriage lift uses a vertical prismatic axis",
        carriage_lift.articulation_type == ArticulationType.PRISMATIC and carriage_lift.axis == (0.0, 0.0, 1.0),
        details=f"type={carriage_lift.articulation_type}, axis={carriage_lift.axis}",
    )
    ctx.expect_origin_distance(
        carriage,
        base,
        axes="xy",
        min_dist=0.026,
        max_dist=0.034,
        name="runner carriage sits off the rotary centerline",
    )
    ctx.expect_overlap(
        carriage,
        lower_stage,
        axes="x",
        elem_a="carriage_shell",
        elem_b="guide_runner",
        min_overlap=0.060,
        name="carriage spans the upright runner width",
    )
    ctx.expect_overlap(
        carriage,
        lower_stage,
        axes="z",
        elem_a="carriage_shell",
        elem_b="guide_runner",
        min_overlap=0.060,
        name="carriage remains engaged with the runner at rest",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        min_gap=0.10,
        name="carriage clears the grounded base",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    lift_upper = carriage_lift.motion_limits.upper if carriage_lift.motion_limits is not None else 0.0
    with ctx.pose({carriage_lift: lift_upper}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            lower_stage,
            axes="z",
            elem_a="carriage_shell",
            elem_b="guide_runner",
            min_overlap=0.045,
            name="carriage retains runner overlap at full lift",
        )

    ctx.check(
        "carriage translates upward without lateral drift",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.075
        and abs(raised_carriage_pos[0] - rest_carriage_pos[0]) < 0.002
        and abs(raised_carriage_pos[1] - rest_carriage_pos[1]) < 0.002,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    yaw_test_angle = 0.9
    with ctx.pose({base_yaw: yaw_test_angle}):
        yawed_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "rotary lower stage yaws around the base axis",
        rest_carriage_pos is not None
        and yawed_carriage_pos is not None
        and abs(
            math.hypot(rest_carriage_pos[0], rest_carriage_pos[1])
            - math.hypot(yawed_carriage_pos[0], yawed_carriage_pos[1])
        )
        < 0.003
        and (
            abs(yawed_carriage_pos[0] - rest_carriage_pos[0]) > 0.015
            or abs(yawed_carriage_pos[1] - rest_carriage_pos[1]) > 0.015
        ),
        details=f"rest={rest_carriage_pos}, yawed={yawed_carriage_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
