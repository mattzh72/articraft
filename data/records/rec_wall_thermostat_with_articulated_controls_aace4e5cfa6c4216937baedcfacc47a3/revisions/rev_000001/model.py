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


BODY_RADIUS = 0.0355
BODY_DEPTH = 0.0160
CENTER_SUPPORT_RADIUS = 0.0294
CENTER_SUPPORT_DEPTH = 0.0240
DISPLAY_RADIUS = 0.0278
DISPLAY_GLASS_DEPTH = 0.0020
BACK_PLATE_RADIUS = 0.0220
BACK_PLATE_DEPTH = 0.0034

RING_OUTER_RADIUS = 0.0420
RING_REAR_BORE_RADIUS = 0.0367
RING_FRONT_OPENING_RADIUS = 0.0306
RING_DEPTH = 0.0230
RING_FRONT_LIP_DEPTH = 0.0065
RING_Z = 0.0020
RING_TRACK_RADIUS = 0.0367
RING_TRACK_LENGTH = 0.0036
RING_TRACK_Z = 0.0038

DOOR_WIDTH = 0.0380
DOOR_HEIGHT = 0.0280
DOOR_THICKNESS = 0.0032
DOOR_CLEARANCE = 0.0006
DOOR_RECESS_DEPTH = 0.0040
DOOR_AXIS_X = DOOR_WIDTH * 0.5
DOOR_AXIS_Z = 0.0024
DOOR_BARREL_RADIUS = 0.0032
DOOR_BARREL_LENGTH = 0.0100
DOOR_BARREL_X = -0.0026
BODY_EAR_RADIUS = 0.0031
BODY_EAR_LENGTH = 0.0080
BODY_EAR_Y = 0.0120


def _annulus(outer_radius: float, inner_radius: float, height: float):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.0020)
        .translate((0.0, 0.0, -0.0010))
    )
    return outer.cut(inner)


def _build_body_shell():
    shell = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_DEPTH)
    shell = shell.union(cq.Workplane("XY").circle(CENTER_SUPPORT_RADIUS).extrude(CENTER_SUPPORT_DEPTH))
    shell = shell.cut(
        cq.Workplane("XY")
        .rect(DOOR_WIDTH + 2.0 * DOOR_CLEARANCE, DOOR_HEIGHT + 2.0 * DOOR_CLEARANCE)
        .extrude(DOOR_RECESS_DEPTH)
    )
    shell = shell.cut(
        cq.Workplane("XZ")
        .center(DOOR_AXIS_X + DOOR_BARREL_X, DOOR_AXIS_Z)
        .circle(DOOR_BARREL_RADIUS + 0.0005)
        .extrude(DOOR_BARREL_LENGTH + 0.0020)
        .translate((0.0, -(DOOR_BARREL_LENGTH + 0.0020) * 0.5, 0.0))
    )
    shell = shell.cut(
        cq.Workplane("XY")
        .circle(DISPLAY_RADIUS - 0.0045)
        .extrude(0.0012)
        .translate((0.0, 0.0, CENTER_SUPPORT_DEPTH - 0.0012))
    )
    return shell


def _build_ring():
    rear_body = _annulus(RING_OUTER_RADIUS, RING_REAR_BORE_RADIUS, RING_DEPTH)
    front_lip = _annulus(
        RING_OUTER_RADIUS,
        RING_FRONT_OPENING_RADIUS,
        RING_FRONT_LIP_DEPTH,
    ).translate((0.0, 0.0, RING_DEPTH - RING_FRONT_LIP_DEPTH))
    return rear_body.union(front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_thermostat")

    shell_white = model.material("shell_white", rgba=(0.90, 0.91, 0.92, 1.0))
    ring_graphite = model.material("ring_graphite", rgba=(0.16, 0.17, 0.19, 1.0))
    glass_black = model.material("glass_black", rgba=(0.05, 0.06, 0.07, 1.0))
    wall_gray = model.material("wall_gray", rgba=(0.78, 0.79, 0.80, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "thermostat_body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=BACK_PLATE_RADIUS, length=BACK_PLATE_DEPTH),
        origin=Origin(xyz=(0.0, 0.0, BACK_PLATE_DEPTH * 0.5)),
        material=wall_gray,
        name="wall_plate",
    )
    body.visual(
        Cylinder(radius=DISPLAY_RADIUS, length=DISPLAY_GLASS_DEPTH),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CENTER_SUPPORT_DEPTH - DISPLAY_GLASS_DEPTH * 0.5,
            )
        ),
        material=glass_black,
        name="display_glass",
    )
    body.visual(
        Cylinder(radius=RING_TRACK_RADIUS, length=RING_TRACK_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, RING_TRACK_Z), rpy=(0.0, 0.0, 0.0)),
        material=shell_white,
        name="ring_track",
    )
    for index, y_pos in enumerate((-BODY_EAR_Y, BODY_EAR_Y)):
        body.visual(
            Cylinder(radius=BODY_EAR_RADIUS, length=BODY_EAR_LENGTH),
            origin=Origin(
                xyz=(DOOR_AXIS_X, y_pos, DOOR_AXIS_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=shell_white,
            name=f"door_ear_{index}",
        )

    ring = model.part("ring")
    ring.visual(
        mesh_from_cadquery(_build_ring(), "thermostat_ring"),
        origin=Origin(xyz=(0.0, 0.0, RING_Z)),
        material=ring_graphite,
        name="outer_ring",
    )

    model.articulation(
        "body_to_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((DOOR_WIDTH, DOOR_HEIGHT, DOOR_THICKNESS)),
        origin=Origin(xyz=(-DOOR_WIDTH * 0.5, 0.0, 0.0)),
        material=shell_white,
        name="door_panel",
    )
    battery_door.visual(
        Cylinder(radius=DOOR_BARREL_RADIUS, length=DOOR_BARREL_LENGTH),
        origin=Origin(xyz=(DOOR_BARREL_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_white,
        name="door_barrel",
    )
    battery_door.visual(
        Box((0.0035, 0.010, 0.0018)),
        origin=Origin(
            xyz=(-DOOR_WIDTH + 0.0018, 0.0, -0.0010),
        ),
        material=glass_black,
        name="door_notch",
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(DOOR_AXIS_X, 0.0, DOOR_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=1.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    ring = object_model.get_part("ring")
    battery_door = object_model.get_part("battery_door")
    ring_joint = object_model.get_articulation("body_to_ring")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.allow_overlap(
        body,
        ring,
        elem_a="ring_track",
        elem_b="outer_ring",
        reason="The thermostat ring is represented as a close-fitting rotating sleeve on a thin rear bearing track.",
    )
    ctx.allow_overlap(
        battery_door,
        body,
        elem_a="door_panel",
        elem_b="body_shell",
        reason="The rear battery hatch sits in a recessed pocket that is visually cut into the shell mesh but represented as a simplified solid shell for QC.",
    )
    ctx.allow_overlap(
        battery_door,
        body,
        elem_a="door_barrel",
        elem_b="body_shell",
        reason="The battery-door hinge barrel nests into a simplified hinge-side recess on the rear shell.",
    )

    ctx.expect_origin_distance(
        body,
        ring,
        axes="xy",
        max_dist=1e-6,
        name="ring stays centered on thermostat body",
    )
    limits = ring_joint.motion_limits
    ctx.check(
        "ring uses continuous motion limits",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"motion_limits={limits!r}",
    )
    ctx.expect_overlap(
        battery_door,
        body,
        axes="xy",
        elem_a="door_panel",
        elem_b="body_shell",
        min_overlap=0.024,
        name="battery door covers the rear access opening when closed",
    )

    closed_aabb = ctx.part_world_aabb(battery_door)
    with ctx.pose({door_joint: 1.2}):
        open_aabb = ctx.part_world_aabb(battery_door)
    ctx.check(
        "battery door opens backward from the rear shell",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.010,
        details=f"closed={closed_aabb!r}, open={open_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
