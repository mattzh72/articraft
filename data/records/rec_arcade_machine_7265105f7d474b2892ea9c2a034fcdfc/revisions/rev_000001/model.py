from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.82
BODY_REAR_X = -0.74
SCREEN_PITCH = -0.55
SCREEN_CENTER = (0.39, 0.0, 1.21)
SCREEN_SIZE = (0.03, 0.52, 0.33)
SEAT_ORIGIN = (-0.06, 0.0, 0.12)
CONSOLE_ORIGIN = (0.26, 0.0, 0.59)
WHEEL_CENTER = (0.135, 0.0, 0.958)
SHIFTER_PIVOT = (0.20, -0.25, 0.70)
DOOR_WIDTH = 0.42
DOOR_HEIGHT = 0.36
DOOR_THICKNESS = 0.024
DOOR_CENTER_Z = 0.30
DOOR_HINGE_Y = -DOOR_WIDTH / 2.0


def _body_shape() -> cq.Workplane:
    outer_profile = [
        (BODY_REAR_X, 0.00),
        (0.66, 0.00),
        (0.66, 0.12),
        (0.58, 0.32),
        (0.49, 0.86),
        (0.42, 1.24),
        (0.54, 1.62),
        (0.30, 1.70),
        (-0.02, 1.55),
        (-0.26, 1.00),
        (-0.54, 0.68),
        (BODY_REAR_X, 0.54),
    ]
    shell = (
        cq.Workplane("XZ")
        .polyline(outer_profile)
        .close()
        .extrude(BODY_WIDTH / 2.0, both=True)
        .edges("|Y")
        .fillet(0.025)
    )

    cockpit_cut = (
        cq.Workplane("XY")
        .box(1.00, 0.58, 1.10, centered=(True, True, False))
        .translate((-0.08, 0.0, 0.12))
    )
    footwell_cut = (
        cq.Workplane("XY")
        .box(0.28, 0.34, 0.48, centered=(True, True, False))
        .translate((0.19, 0.0, 0.10))
    )
    door_cut = (
        cq.Workplane("XY")
        .box(0.10, DOOR_WIDTH + 0.002, DOOR_HEIGHT + 0.002)
        .translate((BODY_REAR_X + 0.03, 0.0, DOOR_CENTER_Z))
    )
    screen_recess = (
        cq.Workplane("XY")
        .box(0.14, SCREEN_SIZE[1] + 0.08, SCREEN_SIZE[2] + 0.08)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), math.degrees(SCREEN_PITCH))
        .translate(SCREEN_CENTER)
    )

    return shell.cut(cockpit_cut).cut(footwell_cut).cut(door_cut).cut(screen_recess)


def _control_console_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(0.32, 0.52, 0.06, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    dash_riser = (
        cq.Workplane("XY")
        .box(0.12, 0.46, 0.10, centered=(True, True, False))
        .translate((0.10, 0.0, 0.06))
    )
    steering_column = (
        cq.Workplane("XY")
        .box(0.18, 0.14, 0.16)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 25.0)
        .translate((-0.01, 0.0, 0.19))
    )
    shifter_boot = (
        cq.Workplane("XY")
        .circle(0.055)
        .extrude(0.04)
        .translate((SHIFTER_PIVOT[0] - CONSOLE_ORIGIN[0], SHIFTER_PIVOT[1], 0.06))
    )
    shifter_flange = (
        cq.Workplane("XY")
        .circle(0.075)
        .extrude(0.010)
        .translate((SHIFTER_PIVOT[0] - CONSOLE_ORIGIN[0], SHIFTER_PIVOT[1], 0.09))
    )
    return (
        deck.union(dash_riser)
        .union(steering_column)
        .union(shifter_boot)
        .union(shifter_flange)
        .edges("|Z")
        .fillet(0.012)
    )


def _seat_shape() -> cq.Workplane:
    pedestal = (
        cq.Workplane("XY")
        .box(0.20, 0.22, 0.25, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    cushion = (
        cq.Workplane("XY")
        .box(0.40, 0.46, 0.12, centered=(True, True, False))
        .translate((0.03, 0.0, 0.25))
    )
    back = (
        cq.Workplane("XY")
        .box(0.12, 0.40, 0.56)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
        .translate((-0.10, 0.0, 0.61))
    )
    left_support = (
        cq.Workplane("XY")
        .box(0.16, 0.06, 0.24)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
        .translate((-0.04, 0.20, 0.34))
    )
    right_support = (
        cq.Workplane("XY")
        .box(0.16, 0.06, 0.24)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
        .translate((-0.04, -0.20, 0.34))
    )
    return (
        pedestal.union(cushion)
        .union(back)
        .union(left_support)
        .union(right_support)
        .edges("|Z")
        .fillet(0.018)
    )


def _door_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(DOOR_THICKNESS, DOOR_WIDTH, DOOR_HEIGHT)
        .translate((0.0, DOOR_WIDTH / 2.0, 0.0))
    )
    hinge_barrel = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(DOOR_HEIGHT * 0.86)
        .translate((0.0, 0.0, -DOOR_HEIGHT * 0.43))
    )
    handle = (
        cq.Workplane("XY")
        .box(0.04, 0.10, 0.035)
        .translate((-DOOR_THICKNESS / 2.0 - 0.015, DOOR_WIDTH * 0.76, 0.0))
    )
    return panel.union(hinge_barrel).union(handle)


def _wheel_shape() -> cq.Workplane:
    rim = cq.Workplane("YZ").circle(0.18).circle(0.135).extrude(0.018, both=True)
    hub = cq.Workplane("YZ").circle(0.055).extrude(0.03, both=True)
    hub_sleeve = cq.Workplane("YZ").circle(0.040).extrude(0.09)
    top_spoke = cq.Workplane("XY").box(0.028, 0.04, 0.18).translate((0.0, 0.0, 0.09))
    lower_left_spoke = (
        cq.Workplane("XY")
        .box(0.028, 0.16, 0.04)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 34.0)
        .translate((0.0, -0.06, -0.05))
    )
    lower_right_spoke = (
        cq.Workplane("XY")
        .box(0.028, 0.16, 0.04)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -34.0)
        .translate((0.0, 0.06, -0.05))
    )
    return (
        rim.union(hub)
        .union(hub_sleeve)
        .union(top_spoke)
        .union(lower_left_spoke)
        .union(lower_right_spoke)
    )


def _shifter_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.022).extrude(0.05).translate((0.0, 0.0, -0.01))
    shaft = cq.Workplane("XY").circle(0.010).extrude(0.18).translate((0.0, 0.0, 0.02))
    knob = cq.Workplane("XY").sphere(0.030).translate((0.0, 0.0, 0.22))
    return base.union(shaft).union(knob).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -12.0)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sit_down_arcade_cabinet")

    body_finish = model.material("body_finish", rgba=(0.18, 0.20, 0.24, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.08, 0.09, 0.10, 1.0))
    seat_finish = model.material("seat_finish", rgba=(0.12, 0.14, 0.18, 1.0))
    screen_finish = model.material("screen_finish", rgba=(0.03, 0.03, 0.04, 1.0))

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        mesh_from_cadquery(_body_shape(), "cabinet_body"),
        material=body_finish,
        name="body_shell",
    )

    cabinet_body.visual(
        mesh_from_cadquery(_seat_shape(), "seat"),
        origin=Origin(xyz=SEAT_ORIGIN),
        material=seat_finish,
        name="seat_shell",
    )
    cabinet_body.visual(
        mesh_from_cadquery(_control_console_shape(), "control_console"),
        origin=Origin(xyz=CONSOLE_ORIGIN),
        material=body_finish,
        name="console_shell",
    )
    cabinet_body.visual(
        Box(SCREEN_SIZE),
        origin=Origin(xyz=SCREEN_CENTER, rpy=(0.0, SCREEN_PITCH, 0.0)),
        material=screen_finish,
        name="screen_panel",
    )
    cabinet_body.visual(
        Box((0.14, 0.42, 0.24)),
        origin=Origin(xyz=SCREEN_CENTER, rpy=(0.0, SCREEN_PITCH, 0.0)),
        material=trim_finish,
        name="screen_housing",
    )

    steering_wheel = model.part("steering_wheel")
    steering_wheel.visual(
        mesh_from_cadquery(_wheel_shape(), "steering_wheel"),
        material=trim_finish,
        name="wheel_rim",
    )

    gear_shifter = model.part("gear_shifter")
    gear_shifter.visual(
        mesh_from_cadquery(_shifter_shape(), "gear_shifter"),
        material=trim_finish,
        name="shifter_lever",
    )

    service_door = model.part("service_door")
    service_door.visual(
        mesh_from_cadquery(_door_shape(), "service_door"),
        material=body_finish,
        name="door_panel",
    )

    model.articulation(
        "body_to_steering_wheel",
        ArticulationType.CONTINUOUS,
        parent=cabinet_body,
        child=steering_wheel,
        origin=Origin(xyz=WHEEL_CENTER),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "body_to_gear_shifter",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=gear_shifter,
        origin=Origin(xyz=SHIFTER_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-0.30, upper=0.38),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=service_door,
        origin=Origin(xyz=(BODY_REAR_X - 0.004, DOOR_HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet_body = object_model.get_part("cabinet_body")
    service_door = object_model.get_part("service_door")
    shifter = object_model.get_part("gear_shifter")
    steering_wheel = object_model.get_part("steering_wheel")
    door_hinge = object_model.get_articulation("body_to_service_door")
    shifter_joint = object_model.get_articulation("body_to_gear_shifter")

    ctx.allow_overlap(
        cabinet_body,
        shifter,
        elem_a="console_shell",
        elem_b="shifter_lever",
        reason="The lever is intentionally authored as passing through the simplified console-top shifter boot at its pivot.",
    )
    ctx.allow_overlap(
        cabinet_body,
        service_door,
        elem_a="body_shell",
        elem_b="door_panel",
        reason="The rear service door includes an integral hinge barrel seated into the simplified jamb on the cabinet shell.",
    )
    ctx.allow_overlap(
        cabinet_body,
        steering_wheel,
        elem_a="console_shell",
        elem_b="wheel_rim",
        reason="The steering wheel hub is intentionally authored as engaging the simplified solid steering-column housing on the console.",
    )

    ctx.expect_origin_distance(
        shifter,
        steering_wheel,
        axes="y",
        min_dist=0.16,
        max_dist=0.30,
        name="gear shifter sits beside the steering wheel",
    )
    ctx.expect_origin_gap(
        steering_wheel,
        shifter,
        axis="z",
        min_gap=0.10,
        max_gap=0.30,
        name="steering wheel stays above the short shifter pivot",
    )

    closed_door_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")
    closed_door_center = _aabb_center(closed_door_aabb)
    ctx.check(
        "service door sits on the rear access plane",
        closed_door_center is not None and abs(closed_door_center[0] - BODY_REAR_X) < 0.03,
        details=f"door_center={closed_door_center}, expected_x={BODY_REAR_X}",
    )

    open_door_aabb = None
    open_shifter_aabb = None
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(service_door, elem="door_panel")

    ctx.check(
        "service door swings outward behind the seat pod",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][0] < closed_door_aabb[0][0] - 0.08,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_shifter_aabb = ctx.part_world_aabb(shifter)
    with ctx.pose({shifter_joint: 0.32}):
        open_shifter_aabb = ctx.part_world_aabb(shifter)

    ctx.check(
        "gear shifter tilts forward from its console pivot",
        rest_shifter_aabb is not None
        and open_shifter_aabb is not None
        and open_shifter_aabb[1][0] > rest_shifter_aabb[1][0] + 0.02,
        details=f"rest={rest_shifter_aabb}, shifted={open_shifter_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
