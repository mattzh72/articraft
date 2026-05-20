from __future__ import annotations

import math

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


BASE_RADIUS = 0.18
BASE_THICKNESS = 0.028
COLUMN_RADIUS = 0.028
COLUMN_HEIGHT = 0.117
PAN_HEAD_RADIUS = 0.060
PAN_HEAD_THICKNESS = 0.028
PAN_AXIS_Z = BASE_THICKNESS + COLUMN_HEIGHT + PAN_HEAD_THICKNESS

YOKE_ARM_HALF_SPAN = 0.138
TILT_AXIS_Z = 0.160

CAN_OUTER_RADIUS = 0.110
CAN_INNER_RADIUS = 0.099
CAN_BODY_LENGTH = 0.290
CAN_FRONT_RING_LENGTH = 0.022
CAN_REAR_CAP_LENGTH = 0.026
CAN_FRONT_RING_RADIUS = 0.118

SERVICE_PAD_LENGTH = 0.150
SERVICE_PAD_DEPTH = 0.026
SERVICE_PAD_HEIGHT = 0.132
SERVICE_PAD_CENTER_X = -0.005
SERVICE_PAD_CENTER_Y = 0.104
SERVICE_PAD_CENTER_Z = 0.082

DOOR_WIDTH = 0.130
DOOR_HEIGHT = 0.114
DOOR_THICKNESS = 0.005
DOOR_HINGE_RADIUS = 0.006
DOOR_REAR_EDGE_X = SERVICE_PAD_CENTER_X - (DOOR_WIDTH / 2.0)
DOOR_HINGE_Y = SERVICE_PAD_CENTER_Y + (SERVICE_PAD_DEPTH / 2.0) + DOOR_HINGE_RADIUS


def _yoke_shape() -> cq.Workplane:
    pan_plate = cq.Workplane("XY", origin=(0.0, 0.0, -0.012)).cylinder(0.024, 0.090)
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.120, 0.240, 0.040)
        .translate((-0.020, 0.0, 0.006))
    )
    left_arm = (
        cq.Workplane("XY")
        .box(0.050, 0.020, 0.230)
        .translate((0.0, YOKE_ARM_HALF_SPAN, 0.105))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(0.050, 0.020, 0.230)
        .translate((0.0, -YOKE_ARM_HALF_SPAN, 0.105))
    )
    left_pad = (
        cq.Workplane("XZ", origin=(0.0, YOKE_ARM_HALF_SPAN - 0.010, TILT_AXIS_Z))
        .circle(0.032)
        .extrude(0.020)
    )
    right_pad = (
        cq.Workplane("XZ", origin=(0.0, -YOKE_ARM_HALF_SPAN + 0.010, TILT_AXIS_Z))
        .circle(0.032)
        .extrude(-0.020)
    )
    rear_spine = (
        cq.Workplane("XY")
        .box(0.075, 0.160, 0.055)
        .translate((-0.050, 0.0, 0.110))
    )
    yoke = pan_plate.union(lower_bridge)
    yoke = yoke.union(left_arm).union(right_arm)
    yoke = yoke.union(left_pad).union(right_pad).union(rear_spine)
    return yoke


def _lamp_can_shape() -> cq.Workplane:
    body_start_x = -CAN_BODY_LENGTH / 2.0
    shell = (
        cq.Workplane("YZ", origin=(body_start_x, 0.0, 0.0))
        .circle(CAN_OUTER_RADIUS)
        .extrude(CAN_BODY_LENGTH)
    )
    front_ring = (
        cq.Workplane("YZ", origin=(CAN_BODY_LENGTH / 2.0, 0.0, 0.0))
        .circle(CAN_FRONT_RING_RADIUS)
        .extrude(CAN_FRONT_RING_LENGTH)
    )
    rear_cap = (
        cq.Workplane("YZ", origin=(body_start_x - CAN_REAR_CAP_LENGTH, 0.0, 0.0))
        .circle(0.094)
        .extrude(CAN_REAR_CAP_LENGTH)
    )
    service_pad = (
        cq.Workplane("XY")
        .box(SERVICE_PAD_LENGTH, SERVICE_PAD_DEPTH, SERVICE_PAD_HEIGHT)
        .translate((SERVICE_PAD_CENTER_X, SERVICE_PAD_CENTER_Y, SERVICE_PAD_CENTER_Z))
    )

    shell = shell.union(front_ring).union(rear_cap).union(service_pad)

    cavity = (
        cq.Workplane("YZ", origin=(body_start_x + 0.020, 0.0, 0.0))
        .circle(CAN_INNER_RADIUS)
        .extrude(CAN_BODY_LENGTH + CAN_FRONT_RING_LENGTH - 0.030)
    )
    service_opening = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH, SERVICE_PAD_DEPTH + 0.030, DOOR_HEIGHT)
        .translate((SERVICE_PAD_CENTER_X, SERVICE_PAD_CENTER_Y, SERVICE_PAD_CENTER_Z))
    )

    shell = shell.cut(cavity).cut(service_opening)
    return shell


def _door_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(DOOR_WIDTH - 0.010, DOOR_THICKNESS, DOOR_HEIGHT - 0.004)
        .translate((0.071, 0.0035, 0.0))
    )
    hinge_barrel = cq.Workplane("XY", origin=(0.0, 0.0, -DOOR_HEIGHT / 2.0)).cylinder(
        DOOR_HEIGHT, DOOR_HINGE_RADIUS
    )
    latch_knob = (
        cq.Workplane("XZ", origin=(DOOR_WIDTH - 0.020, 0.006, 0.0))
        .circle(0.007)
        .extrude(0.010)
    )
    door = panel.union(hinge_barrel).union(latch_knob)
    return door


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight")

    stand_black = model.material("stand_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    lens_tint = model.material("lens_tint", rgba=(0.70, 0.79, 0.86, 0.55))
    trim_gray = model.material("trim_gray", rgba=(0.45, 0.46, 0.48, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=stand_black,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + (COLUMN_HEIGHT / 2.0))),
        material=stand_black,
        name="column",
    )
    stand.visual(
        Cylinder(radius=PAN_HEAD_RADIUS, length=PAN_HEAD_THICKNESS),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + (PAN_HEAD_THICKNESS / 2.0))
        ),
        material=dark_gray,
        name="pan_head",
    )
    stand.visual(
        Box((0.110, 0.110, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT + 0.005)),
        material=dark_gray,
        name="pan_cap",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.090, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=housing_gray,
        name="pan_plate",
    )
    yoke.visual(
        Box((0.130, 0.286, 0.028)),
        origin=Origin(xyz=(-0.012, 0.0, 0.022)),
        material=housing_gray,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.046, 0.020, 0.220)),
        origin=Origin(xyz=(0.0, YOKE_ARM_HALF_SPAN, 0.125)),
        material=housing_gray,
        name="arm_0",
    )
    yoke.visual(
        Box((0.046, 0.020, 0.220)),
        origin=Origin(xyz=(0.0, -YOKE_ARM_HALF_SPAN, 0.125)),
        material=housing_gray,
        name="arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(
            xyz=(0.0, YOKE_ARM_HALF_SPAN, TILT_AXIS_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_gray,
        name="pivot_pad_0",
    )
    yoke.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(
            xyz=(0.0, -YOKE_ARM_HALF_SPAN, TILT_AXIS_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_gray,
        name="pivot_pad_1",
    )

    lamp_can = model.part("lamp_can")
    lamp_can.visual(
        mesh_from_cadquery(_lamp_can_shape(), "lamp_can"),
        material=housing_gray,
        name="body_shell",
    )
    lamp_can.visual(
        Cylinder(radius=0.102, length=0.004),
        origin=Origin(
            xyz=(CAN_BODY_LENGTH / 2.0 + 0.003, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_tint,
        name="front_lens",
    )
    lamp_can.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(
            xyz=(0.0, CAN_OUTER_RADIUS + 0.004, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_gray,
        name="trunnion_0",
    )
    lamp_can.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(
            xyz=(0.0, -CAN_OUTER_RADIUS - 0.004, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=housing_gray,
        name="trunnion_1",
    )

    access_door = model.part("access_door")
    access_door.visual(
        Cylinder(radius=DOOR_HINGE_RADIUS, length=DOOR_HEIGHT),
        material=trim_gray,
        name="hinge_barrel",
    )
    access_door.visual(
        Box((DOOR_WIDTH - 0.010, DOOR_THICKNESS, DOOR_HEIGHT - 0.006)),
        origin=Origin(xyz=(0.065, -0.0035, 0.0)),
        material=trim_gray,
        name="door_panel",
    )
    access_door.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(
            xyz=(DOOR_WIDTH - 0.020, 0.004, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_gray,
        name="latch_knob",
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.6,
            lower=-2.5,
            upper=2.5,
        ),
    )
    model.articulation(
        "yoke_to_lamp_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_can,
        origin=Origin(xyz=(0.0, 0.0, TILT_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=-0.9,
            upper=1.1,
        ),
    )
    model.articulation(
        "lamp_can_to_access_door",
        ArticulationType.REVOLUTE,
        parent=lamp_can,
        child=access_door,
        origin=Origin(xyz=(DOOR_REAR_EDGE_X, DOOR_HINGE_Y, SERVICE_PAD_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.4,
            lower=0.0,
            upper=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    yoke = object_model.get_part("yoke")
    lamp_can = object_model.get_part("lamp_can")
    access_door = object_model.get_part("access_door")

    pan = object_model.get_articulation("stand_to_yoke")
    tilt = object_model.get_articulation("yoke_to_lamp_can")
    door = object_model.get_articulation("lamp_can_to_access_door")

    ctx.expect_overlap(
        access_door,
        lamp_can,
        axes="xz",
        elem_a="door_panel",
        elem_b="body_shell",
        min_overlap=0.09,
        name="closed access door stays inside the can side opening footprint",
    )
    ctx.expect_overlap(
        lamp_can,
        yoke,
        axes="yz",
        min_overlap=0.16,
        name="lamp can remains cradled by the yoke",
    )

    rest_front = _aabb_center(ctx.part_element_world_aabb(lamp_can, elem="front_lens"))
    with ctx.pose({pan: 1.0}):
        panned_front = _aabb_center(ctx.part_element_world_aabb(lamp_can, elem="front_lens"))
    ctx.check(
        "pan joint swings the lamp can laterally",
        rest_front is not None
        and panned_front is not None
        and panned_front[1] > rest_front[1] + 0.10,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    with ctx.pose({tilt: 0.7}):
        tilted_front = _aabb_center(ctx.part_element_world_aabb(lamp_can, elem="front_lens"))
    ctx.check(
        "tilt joint lifts the front lens upward",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.08,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    rest_door = _aabb_center(ctx.part_element_world_aabb(access_door, elem="door_panel"))
    with ctx.pose({door: 1.2}):
        open_door = _aabb_center(ctx.part_element_world_aabb(access_door, elem="door_panel"))
    ctx.check(
        "access door opens outward from the housing",
        rest_door is not None
        and open_door is not None
        and open_door[1] > rest_door[1] + 0.035,
        details=f"rest_door={rest_door}, open_door={open_door}",
    )

    return ctx.report()


object_model = build_object_model()
