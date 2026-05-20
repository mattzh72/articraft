from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.045
BODY_LENGTH = 0.115
BODY_HEIGHT = 0.014
BODY_CORNER_FILLET = 0.007
BODY_TOP_EDGE_FILLET = 0.0022
BODY_BOTTOM_EDGE_FILLET = 0.0014

CONTROL_Y = 0.018
RING_OUTER_DIAMETER = 0.030
RING_INNER_DIAMETER = 0.019
RING_HEIGHT = 0.0024
RING_POCKET_DIAMETER = 0.0318
RING_POCKET_DEPTH = 0.0012

BUTTON_DIAMETER = 0.0142
BUTTON_HEIGHT = 0.0022
BUTTON_CAP_HEIGHT = 0.0017
BUTTON_CAP_CENTER_Z = 0.00155
BUTTON_STEM_DIAMETER = 0.0080
BUTTON_STEM_BOTTOM_Z = -0.0030
BUTTON_WELL_DIAMETER = 0.0154
BUTTON_WELL_DEPTH = 0.0032
BUTTON_BORE_DIAMETER = 0.0088
BUTTON_BORE_DEPTH = 0.0065

HINGE_RADIUS = 0.0015
HINGE_Y = -BODY_LENGTH / 2.0 + 0.0032
HINGE_Z = -BODY_HEIGHT / 2.0 + 0.00125
BODY_KNUCKLE_LENGTH = 0.0080
BODY_KNUCKLE_X = 0.0118
COVER_KNUCKLE_LENGTH = 0.0150

COVER_WIDTH = 0.032
COVER_BODY_LENGTH = 0.058
COVER_THICKNESS = 0.0017
COVER_FRONT_RADIUS = 0.0022
COVER_START_Y = 0.0025
COVER_CENTER_Z = -0.00210
COVER_CLEARANCE = 0.00030
CAVITY_DEPTH = COVER_THICKNESS + 0.00045
CAVITY_WIDTH = COVER_WIDTH + 2.0 * COVER_CLEARANCE
CAVITY_LENGTH = COVER_BODY_LENGTH + 2.0 * COVER_CLEARANCE
CAVITY_CENTER_Y = HINGE_Y + COVER_START_Y + COVER_BODY_LENGTH / 2.0

RING_FLOOR_Z = BODY_HEIGHT / 2.0 - RING_POCKET_DEPTH
BUTTON_HOME_Z = BODY_HEIGHT / 2.0 - BUTTON_HEIGHT - 0.00015

BUTTON_STEM_TOP_Z = BUTTON_CAP_CENTER_Z - BUTTON_CAP_HEIGHT / 2.0
BUTTON_STEM_LENGTH = BUTTON_STEM_TOP_Z - BUTTON_STEM_BOTTOM_Z
BUTTON_STEM_CENTER_Z = (BUTTON_STEM_TOP_Z + BUTTON_STEM_BOTTOM_Z) / 2.0


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
    )


def _rounded_plate(width: float, length: float, thickness: float, corner_radius: float) -> cq.Workplane:
    plate = cq.Workplane("XY").box(width, length, thickness, centered=(True, False, False))
    max_radius = min(corner_radius, width * 0.25, length * 0.12)
    if max_radius > 0.0:
        plate = plate.edges("|Z").fillet(max_radius)
    return plate


def _body_shape() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_WIDTH, BODY_LENGTH, BODY_HEIGHT)
    shell = shell.edges("|Z").fillet(BODY_CORNER_FILLET)
    shell = shell.edges(">Z").fillet(BODY_TOP_EDGE_FILLET)
    shell = shell.edges("<Z").fillet(BODY_BOTTOM_EDGE_FILLET)

    ring_pocket = (
        cq.Workplane("XY")
        .center(0.0, CONTROL_Y)
        .circle(RING_POCKET_DIAMETER / 2.0)
        .extrude(RING_POCKET_DEPTH)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0 - RING_POCKET_DEPTH))
    )
    button_well = (
        cq.Workplane("XY")
        .center(0.0, CONTROL_Y)
        .circle(BUTTON_WELL_DIAMETER / 2.0)
        .extrude(BUTTON_WELL_DEPTH)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0 - BUTTON_WELL_DEPTH))
    )
    button_bore = (
        cq.Workplane("XY")
        .center(0.0, CONTROL_Y)
        .circle(BUTTON_BORE_DIAMETER / 2.0)
        .extrude(BUTTON_BORE_DEPTH)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0 - BUTTON_WELL_DEPTH - BUTTON_BORE_DEPTH))
    )

    cavity = _rounded_plate(CAVITY_WIDTH, CAVITY_LENGTH, CAVITY_DEPTH + 0.0002, 0.0030).translate(
        (0.0, CAVITY_CENTER_Y - CAVITY_LENGTH / 2.0, -BODY_HEIGHT / 2.0 - 0.0001)
    )

    shell = shell.cut(ring_pocket).cut(button_well).cut(button_bore).cut(cavity)

    for direction in (-1.0, 1.0):
        knuckle = _x_cylinder(HINGE_RADIUS, BODY_KNUCKLE_LENGTH).translate(
            (direction * BODY_KNUCKLE_X, HINGE_Y, HINGE_Z)
        )
        support = cq.Workplane("XY").box(
            BODY_KNUCKLE_LENGTH,
            0.0032,
            0.0018,
            centered=(True, True, True),
        ).translate((direction * BODY_KNUCKLE_X, HINGE_Y - 0.0010, HINGE_Z - 0.0011))
        shell = shell.union(knuckle).union(support)

    return shell


def _ring_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(RING_OUTER_DIAMETER / 2.0)
        .circle(RING_INNER_DIAMETER / 2.0)
        .extrude(RING_HEIGHT)
    )
    ring = ring.edges(">Z").fillet(0.00055)
    ring = ring.edges("<Z").chamfer(0.00020)
    return ring


def _battery_cover_shape() -> cq.Workplane:
    cover_plate = _rounded_plate(COVER_WIDTH, COVER_BODY_LENGTH, COVER_THICKNESS, COVER_FRONT_RADIUS).translate(
        (0.0, COVER_START_Y, COVER_CENTER_Z)
    )
    front_lip = cq.Workplane("XY").box(
        COVER_WIDTH * 0.55,
        0.0020,
        0.0007,
        centered=(True, True, True),
    ).translate((0.0, COVER_START_Y + COVER_BODY_LENGTH - 0.0010, COVER_CENTER_Z + COVER_THICKNESS / 2.0 - 0.00035))
    center_knuckle = _x_cylinder(HINGE_RADIUS, COVER_KNUCKLE_LENGTH).translate((0.0, 0.0, 0.0))
    center_web = cq.Workplane("XY").box(
        COVER_KNUCKLE_LENGTH,
        COVER_START_Y + 0.0007,
        0.0017,
        centered=(True, False, True),
    ).translate((0.0, 0.0, COVER_CENTER_Z / 2.0))
    return cover_plate.union(front_lip).union(center_knuckle).union(center_web)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_speaker_remote")

    shell_mat = model.material("shell_graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    ring_mat = model.material("ring_black", rgba=(0.07, 0.07, 0.08, 1.0))
    button_mat = model.material("button_matte", rgba=(0.11, 0.12, 0.13, 1.0))
    cover_mat = model.material("cover_graphite", rgba=(0.18, 0.19, 0.21, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "remote_body"),
        material=shell_mat,
        name="shell",
    )

    ring = model.part("ring")
    ring.visual(
        mesh_from_cadquery(_ring_shape(), "remote_ring"),
        material=ring_mat,
        name="ring",
    )

    button = model.part("button")
    button.visual(
        Cylinder(radius=BUTTON_DIAMETER / 2.0, length=BUTTON_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_CAP_CENTER_Z)),
        material=button_mat,
        name="cap",
    )
    button.visual(
        Cylinder(radius=BUTTON_STEM_DIAMETER / 2.0, length=BUTTON_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, BUTTON_STEM_CENTER_Z)),
        material=button_mat,
        name="stem",
    )

    battery_cover = model.part("battery_cover")
    battery_cover.visual(
        mesh_from_cadquery(_battery_cover_shape(), "remote_battery_cover"),
        material=cover_mat,
        name="cover",
    )

    model.articulation(
        "body_to_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=ring,
        origin=Origin(xyz=(0.0, CONTROL_Y, RING_FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=9.0),
    )

    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.0, CONTROL_Y, BUTTON_HOME_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0014,
        ),
    )

    model.articulation(
        "body_to_battery_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    ring = object_model.get_part("ring")
    button = object_model.get_part("button")
    battery_cover = object_model.get_part("battery_cover")

    ring_joint = object_model.get_articulation("body_to_ring")
    button_joint = object_model.get_articulation("body_to_button")
    cover_joint = object_model.get_articulation("body_to_battery_cover")

    ctx.allow_isolated_part(
        button,
        reason="The center key is a spring-loaded plunger authored with guide clearance inside the housing bore.",
    )
    ctx.allow_overlap(
        body,
        button,
        elem_a="shell",
        elem_b="stem",
        reason="The hidden plunger stem is intentionally represented inside the body's simplified opaque guide bore.",
    )

    ctx.check(
        "ring joint is continuous",
        ring_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={ring_joint.articulation_type}",
    )
    ctx.check(
        "button joint travels downward",
        tuple(button_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={button_joint.axis}",
    )
    ctx.check(
        "battery cover hinge opens from rear edge",
        tuple(cover_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={cover_joint.axis}",
    )

    with ctx.pose({button_joint: 0.0, cover_joint: 0.0}):
        ctx.expect_overlap(
            ring,
            body,
            axes="xy",
            min_overlap=0.028,
            name="ring stays centered on the control recess",
        )
        ctx.expect_within(
            button,
            ring,
            axes="xy",
            margin=0.0008,
            name="button sits inside the rotating ring opening",
        )

    rest_button_aabb = ctx.part_world_aabb(button)
    with ctx.pose({button_joint: 0.0014}):
        pressed_button_aabb = ctx.part_world_aabb(button)
    ctx.check(
        "center button depresses into the housing",
        rest_button_aabb is not None
        and pressed_button_aabb is not None
        and pressed_button_aabb[1][2] < rest_button_aabb[1][2] - 0.0010,
        details=f"rest={rest_button_aabb}, pressed={pressed_button_aabb}",
    )

    body_aabb = ctx.part_world_aabb(body)
    closed_cover_aabb = ctx.part_world_aabb(battery_cover)
    ctx.check(
        "battery cover closes near the body underside",
        body_aabb is not None
        and closed_cover_aabb is not None
        and abs(closed_cover_aabb[0][2] - body_aabb[0][2]) <= 0.0005,
        details=f"body={body_aabb}, cover={closed_cover_aabb}",
    )

    with ctx.pose({cover_joint: 1.35}):
        opened_cover_aabb = ctx.part_world_aabb(battery_cover)
    ctx.check(
        "battery cover free edge drops when opened",
        closed_cover_aabb is not None
        and opened_cover_aabb is not None
        and opened_cover_aabb[0][2] < closed_cover_aabb[0][2] - 0.010,
        details=f"closed={closed_cover_aabb}, opened={opened_cover_aabb}",
    )

    with ctx.pose({ring_joint: 2.1}):
        ctx.expect_overlap(
            ring,
            body,
            axes="xy",
            min_overlap=0.028,
            name="ring remains seated while rotated",
        )

    return ctx.report()


object_model = build_object_model()
