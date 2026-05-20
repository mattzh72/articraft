from __future__ import annotations

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


BODY_WIDTH = 0.56
BODY_DEPTH = 0.42
BODY_BOTTOM_Z = 0.09
BODY_HEIGHT = 0.68
WALL = 0.018

DOOR_WIDTH = 0.47
DOOR_HEIGHT = 0.34
DOOR_THICKNESS = 0.020
DOOR_BOTTOM_Z = BODY_BOTTOM_Z + 0.09

SHELF_BOTTOM_Z = DOOR_BOTTOM_Z + DOOR_HEIGHT

LID_WIDTH = BODY_WIDTH - 0.012
LID_DEPTH = BODY_DEPTH - 0.010
LID_THICKNESS = 0.028

GUIDE_X = 0.18
GUIDE_Y = BODY_DEPTH / 2.0 + 0.012
GUIDE_TOP_Z = BODY_BOTTOM_Z + 0.60
GUIDE_LENGTH = 0.32
GUIDE_OUTER_RADIUS = 0.018

HANDLE_ROD_RADIUS = 0.011
HANDLE_ROD_LENGTH = 0.60
HANDLE_TRAVEL = 0.18
GUIDE_INNER_RADIUS = HANDLE_ROD_RADIUS

WHEEL_RADIUS = 0.095
WHEEL_WIDTH = 0.055
WHEEL_Y = BODY_DEPTH / 2.0 - 0.052
WHEEL_X = BODY_WIDTH / 2.0 + WHEEL_WIDTH / 2.0

DOOR_OPEN_ANGLE = 1.45
LID_OPEN_ANGLE = 1.22


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _tube_at(
    *,
    x: float,
    y: float,
    z0: float,
    length: float,
    outer_radius: float,
    inner_radius: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").center(x, y).circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("XY")
        .center(x, y)
        .circle(inner_radius)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner).translate((0.0, 0.0, z0))


def _build_body_shape() -> cq.Workplane:
    outer = _box_at(
        (BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT),
        (0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0),
    )
    inner = _box_at(
        (
            BODY_WIDTH - 2.0 * WALL,
            BODY_DEPTH - 2.0 * WALL,
            BODY_HEIGHT - WALL + 0.010,
        ),
        (0.0, 0.0, BODY_BOTTOM_Z + WALL + (BODY_HEIGHT - WALL + 0.010) / 2.0),
    )
    body = outer.cut(inner)

    lower_opening = _box_at(
        (DOOR_WIDTH, 0.085, DOOR_HEIGHT),
        (0.0, -BODY_DEPTH / 2.0 + 0.040, DOOR_BOTTOM_Z + DOOR_HEIGHT / 2.0),
    )
    body = body.cut(lower_opening)

    shelf = _box_at(
        (BODY_WIDTH - 2.0 * WALL - 0.006, BODY_DEPTH - 2.0 * WALL - 0.008, WALL),
        (0.0, 0.0, SHELF_BOTTOM_Z + WALL / 2.0),
    )
    body = body.union(shelf)

    axle_beam = _box_at(
        (BODY_WIDTH - 0.18, 0.052, 0.052),
        (0.0, BODY_DEPTH / 2.0 - 0.027, WHEEL_RADIUS + 0.018),
    )
    body = body.union(axle_beam)

    for x in (-0.16, 0.16):
        foot = _box_at(
            (0.08, 0.09, BODY_BOTTOM_Z),
            (x, -BODY_DEPTH / 2.0 + 0.055, BODY_BOTTOM_Z / 2.0),
        )
        body = body.union(foot)

    for x in (-GUIDE_X, GUIDE_X):
        guide = _tube_at(
            x=x,
            y=GUIDE_Y,
            z0=GUIDE_TOP_Z - GUIDE_LENGTH,
            length=GUIDE_LENGTH,
            outer_radius=GUIDE_OUTER_RADIUS,
            inner_radius=GUIDE_INNER_RADIUS,
        )
        bridge = _box_at(
            (0.020, 0.046, 0.22),
            (
                x - 0.022 * (1.0 if x > 0.0 else -1.0),
                GUIDE_Y - 0.012,
                GUIDE_TOP_Z - GUIDE_LENGTH / 2.0,
            ),
        )
        body = body.union(guide).union(bridge)

    rear_bumper = _box_at(
        (BODY_WIDTH - 0.10, 0.026, 0.040),
        (0.0, BODY_DEPTH / 2.0 - 0.002, BODY_BOTTOM_Z + 0.12),
    )
    body = body.union(rear_bumper)

    return body


def _build_door_shape() -> cq.Workplane:
    panel = _box_at(
        (DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT),
        (0.0, -DOOR_THICKNESS / 2.0, DOOR_HEIGHT / 2.0),
    )
    recess = _box_at(
        (DOOR_WIDTH - 0.12, DOOR_THICKNESS * 0.62, DOOR_HEIGHT - 0.10),
        (0.0, -DOOR_THICKNESS * 0.20, DOOR_HEIGHT / 2.0 + 0.005),
    )
    pull = _box_at(
        (0.11, 0.014, 0.030),
        (0.0, -DOOR_THICKNESS / 2.0 - 0.004, DOOR_HEIGHT * 0.80),
    )
    kick = _box_at(
        (DOOR_WIDTH - 0.04, 0.012, 0.035),
        (0.0, -DOOR_THICKNESS / 2.0 + 0.003, 0.028),
    )
    return panel.cut(recess).union(pull).union(kick)


def _build_lid_shape() -> cq.Workplane:
    lid = _box_at(
        (LID_WIDTH, LID_DEPTH, LID_THICKNESS),
        (0.0, -LID_DEPTH / 2.0, LID_THICKNESS / 2.0),
    )
    top_pad = _box_at(
        (LID_WIDTH - 0.12, LID_DEPTH * 0.48, LID_THICKNESS * 0.45),
        (0.0, -LID_DEPTH * 0.50, LID_THICKNESS * 0.72),
    )
    front_lip = _box_at(
        (0.16, 0.012, 0.020),
        (0.0, -LID_DEPTH + 0.010, LID_THICKNESS * 0.42),
    )
    return lid.union(top_pad).union(front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="site_toolbox")

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    lid_dark = model.material("lid_dark", rgba=(0.20, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.65, 0.68, 0.72, 1.0))
    tire_black = model.material("tire_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material=body_dark,
        name="shell",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_shape(), "door_panel"),
        material=body_dark,
        name="panel",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "lid_panel"),
        material=lid_dark,
        name="panel",
    )

    handle = model.part("handle")
    for sign, name in ((-1.0, "rod_0"), (1.0, "rod_1")):
        handle.visual(
            Cylinder(radius=HANDLE_ROD_RADIUS, length=HANDLE_ROD_LENGTH),
            origin=Origin(xyz=(sign * GUIDE_X, GUIDE_Y, 0.0), rpy=(0.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )
    handle.visual(
        Cylinder(radius=0.014, length=2.0 * GUIDE_X + 0.04),
        origin=Origin(xyz=(0.0, GUIDE_Y, 0.31), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="crossbar",
    )
    handle.visual(
        Box((0.26, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, GUIDE_Y, 0.31)),
        material=body_dark,
        name="grip",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=tire_black,
        name="tire",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=tire_black,
        name="tire",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0, DOOR_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=DOOR_OPEN_ANGLE),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - 0.004, BODY_BOTTOM_Z + BODY_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=LID_OPEN_ANGLE),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=HANDLE_TRAVEL),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-WHEEL_X, WHEEL_Y, WHEEL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_X, WHEEL_Y, WHEEL_RADIUS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    door_hinge = object_model.get_articulation("door_hinge")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_slide = object_model.get_articulation("handle_slide")

    with ctx.pose({door_hinge: 0.0, lid_hinge: 0.0, handle_slide: 0.0}):
        ctx.allow_overlap(
            body,
            handle,
            elem_a="shell",
            elem_b="rod_0",
            reason="The telescoping tow handle rods are intentionally represented as sliding inside the integrated rear guide housing mesh.",
        )
        ctx.allow_overlap(
            body,
            handle,
            elem_a="shell",
            elem_b="rod_1",
            reason="The telescoping tow handle rods are intentionally represented as sliding inside the integrated rear guide housing mesh.",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            min_overlap=0.42,
            name="door spans the lower front opening width",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.30,
            name="lid covers the upper tray footprint",
        )

    handle_rest = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        handle_extended = ctx.part_world_position(handle)
    ctx.check(
        "tow handle extends upward",
        handle_rest is not None
        and handle_extended is not None
        and handle_extended[2] > handle_rest[2] + 0.15,
        details=f"rest={handle_rest}, extended={handle_extended}",
    )

    door_closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE}):
        door_open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door folds down and away from the body",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.14
        and door_open_aabb[0][1] < door_closed_aabb[0][1] - 0.10,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}",
    )

    lid_closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: LID_OPEN_ANGLE}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward from the rear hinge",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.14,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    left_pos = ctx.part_world_position(left_wheel)
    right_pos = ctx.part_world_position(right_wheel)
    body_pos = ctx.part_world_position(body)
    ctx.check(
        "rear wheels straddle the body",
        left_pos is not None
        and right_pos is not None
        and body_pos is not None
        and left_pos[0] < body_pos[0] - 0.20
        and right_pos[0] > body_pos[0] + 0.20
        and left_pos[1] > body_pos[1] + 0.10
        and right_pos[1] > body_pos[1] + 0.10,
        details=f"body={body_pos}, left={left_pos}, right={right_pos}",
    )

    return ctx.report()


object_model = build_object_model()
