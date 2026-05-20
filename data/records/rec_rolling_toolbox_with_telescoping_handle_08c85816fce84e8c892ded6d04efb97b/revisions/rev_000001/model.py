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


BODY_WIDTH = 0.34
BODY_DEPTH = 0.22
BODY_HEIGHT = 0.62
BODY_WALL = 0.014
BODY_FLOOR = 0.020
BODY_ROOF = 0.020

SPINE_WIDTH = 0.11
SPINE_EXTRA_DEPTH = 0.04
SPINE_FRONT_WALL = 0.012
SPINE_CHANNEL_WIDTH = 0.078
SPINE_CHANNEL_BOTTOM = 0.11
SPINE_CHANNEL_TOP_MARGIN = 0.032

DOOR_THICKNESS = 0.016
DOOR_SIDE_GAP = 0.003
DOOR_TOP_GAP = 0.004
DOOR_BOTTOM_GAP = 0.004
DOOR_FRONT_GAP = 0.0015

WHEEL_RADIUS = 0.055
WHEEL_WIDTH = 0.026
WHEEL_CLEARANCE = 0.004

HANDLE_BAR_RADIUS = 0.006
HANDLE_BAR_SPACING = 0.058
HANDLE_GRIP_WIDTH = 0.090
HANDLE_GRIP_DEPTH = 0.018
HANDLE_GRIP_HEIGHT = 0.016
HANDLE_INSERTION = 0.31
HANDLE_BAR_UPPER = 0.030
HANDLE_TRAVEL = 0.22


def _body_mesh():
    outer = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT).translate(
        (0.0, 0.0, BODY_HEIGHT / 2.0)
    )
    spine = cq.Workplane("XY").box(SPINE_WIDTH, SPINE_EXTRA_DEPTH, BODY_HEIGHT).translate(
        (0.0, -(BODY_DEPTH / 2.0 + SPINE_EXTRA_DEPTH / 2.0), BODY_HEIGHT / 2.0)
    )
    body = outer.union(spine)

    cavity_depth = BODY_DEPTH - BODY_WALL + 0.030
    cavity_center_y = -BODY_DEPTH / 2.0 + BODY_WALL + cavity_depth / 2.0
    cavity_height = BODY_HEIGHT - BODY_FLOOR - BODY_ROOF
    cavity_center_z = BODY_FLOOR + cavity_height / 2.0
    cavity = cq.Workplane("XY").box(
        BODY_WIDTH - 2.0 * BODY_WALL,
        cavity_depth,
        cavity_height,
    ).translate((0.0, cavity_center_y, cavity_center_z))
    body = body.cut(cavity)

    channel_front = -BODY_DEPTH / 2.0 - SPINE_FRONT_WALL
    channel_rear = -BODY_DEPTH / 2.0 - SPINE_EXTRA_DEPTH - 0.010
    channel_depth = channel_front - channel_rear
    channel_center_y = (channel_front + channel_rear) / 2.0
    channel_height = BODY_HEIGHT - SPINE_CHANNEL_BOTTOM - SPINE_CHANNEL_TOP_MARGIN
    channel_center_z = SPINE_CHANNEL_BOTTOM + channel_height / 2.0
    channel = cq.Workplane("XY").box(
        SPINE_CHANNEL_WIDTH,
        channel_depth,
        channel_height,
    ).translate((0.0, channel_center_y, channel_center_z))
    body = body.cut(channel)

    top_pocket = cq.Workplane("XY").box(
        SPINE_CHANNEL_WIDTH + 0.020,
        SPINE_EXTRA_DEPTH + 0.010,
        0.034,
    ).translate(
        (
            0.0,
            -(BODY_DEPTH / 2.0 + SPINE_EXTRA_DEPTH / 2.0),
            BODY_HEIGHT - 0.025,
        )
    )
    body = body.cut(top_pocket)

    recess_y = -BODY_DEPTH / 2.0 + 0.020
    recess_z = WHEEL_RADIUS + 0.012
    wheel_x = BODY_WIDTH / 2.0 - WHEEL_WIDTH / 2.0
    for sign in (-1.0, 1.0):
        recess = cq.Workplane("XY").box(0.062, 0.136, 0.134).translate(
            (sign * wheel_x, recess_y, recess_z)
        )
        body = body.cut(recess)

    toe_foot = cq.Workplane("XY").box(0.090, 0.040, 0.020).translate(
        (0.0, BODY_DEPTH / 2.0 - 0.020, 0.010)
    )
    body = body.union(toe_foot)

    return body


def _door_mesh(door_width: float, door_height: float):
    door = cq.Workplane("XY").box(door_width, DOOR_THICKNESS, door_height).translate(
        (door_width / 2.0, DOOR_THICKNESS / 2.0, door_height / 2.0)
    )
    panel_recess = cq.Workplane("XY").box(
        door_width - 0.056,
        0.005,
        door_height - 0.120,
    ).translate((door_width / 2.0, DOOR_THICKNESS - 0.0025, door_height / 2.0))
    door = door.cut(panel_recess)

    latch = cq.Workplane("XY").box(0.018, 0.018, 0.150).translate(
        (door_width - 0.026, DOOR_THICKNESS / 2.0 + 0.004, door_height * 0.56)
    )
    door = door.union(latch)

    return door


def _handle_mesh():
    bar_length = HANDLE_INSERTION + HANDLE_BAR_UPPER
    left_bar = (
        cq.Workplane("XY")
        .center(-HANDLE_BAR_SPACING / 2.0, 0.0)
        .circle(HANDLE_BAR_RADIUS)
        .extrude(bar_length)
        .translate((0.0, 0.0, -HANDLE_INSERTION))
    )
    right_bar = (
        cq.Workplane("XY")
        .center(HANDLE_BAR_SPACING / 2.0, 0.0)
        .circle(HANDLE_BAR_RADIUS)
        .extrude(bar_length)
        .translate((0.0, 0.0, -HANDLE_INSERTION))
    )
    grip = cq.Workplane("XY").box(
        HANDLE_GRIP_WIDTH,
        HANDLE_GRIP_DEPTH,
        HANDLE_GRIP_HEIGHT,
    ).translate((0.0, 0.0, HANDLE_BAR_UPPER + HANDLE_GRIP_HEIGHT / 2.0 - 0.002))

    return left_bar.union(right_bar).union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_toolbox")

    shell_color = model.material("shell_color", rgba=(0.18, 0.20, 0.23, 1.0))
    door_color = model.material("door_color", rgba=(0.80, 0.16, 0.11, 1.0))
    trim_color = model.material("trim_color", rgba=(0.10, 0.11, 0.12, 1.0))
    metal_color = model.material("metal_color", rgba=(0.78, 0.80, 0.83, 1.0))
    rubber_color = model.material("rubber_color", rgba=(0.08, 0.08, 0.09, 1.0))

    door_width = BODY_WIDTH - 2.0 * (BODY_WALL + DOOR_SIDE_GAP)
    door_height = BODY_HEIGHT - BODY_FLOOR - BODY_ROOF - DOOR_TOP_GAP - DOOR_BOTTOM_GAP
    door_bottom_z = BODY_FLOOR + DOOR_BOTTOM_GAP
    door_hinge_x = -door_width / 2.0
    door_origin_y = BODY_DEPTH / 2.0 + DOOR_FRONT_GAP

    channel_front = -BODY_DEPTH / 2.0 - SPINE_FRONT_WALL
    channel_rear = -BODY_DEPTH / 2.0 - SPINE_EXTRA_DEPTH - 0.010
    channel_center_y = (channel_front + channel_rear) / 2.0

    handle_joint_z = BODY_HEIGHT - 0.070
    wheel_axle_y = -BODY_DEPTH / 2.0 + 0.020
    wheel_axle_z = WHEEL_RADIUS + 0.012
    wheel_axle_x = BODY_WIDTH / 2.0 - WHEEL_WIDTH / 2.0

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_mesh(), "toolbox_body"),
        material=shell_color,
        name="shell",
    )

    axle_stub_x = wheel_axle_x - WHEEL_WIDTH / 2.0 - 0.009

    for idx, sign in enumerate((-1.0, 1.0)):
        body.visual(
            Cylinder(radius=0.011, length=0.018),
            origin=Origin(
                xyz=(sign * axle_stub_x, wheel_axle_y, wheel_axle_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=metal_color,
            name=f"axle_{idx}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_mesh(door_width, door_height), "toolbox_door"),
        material=door_color,
        name="panel",
    )
    for idx, barrel_z in enumerate((0.085, door_height - 0.085)):
        door.visual(
            Cylinder(radius=0.003, length=0.110),
            origin=Origin(xyz=(0.0, -0.0015, barrel_z)),
            material=trim_color,
            name=f"barrel_{idx}",
        )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_mesh(), "toolbox_handle"),
        material=metal_color,
        name="frame",
        origin=Origin(xyz=(0.0, channel_center_y, 0.0)),
    )

    for idx, sign in enumerate((-1.0, 1.0)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber_color,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.034, length=0.008),
            origin=Origin(
                xyz=(sign * (WHEEL_WIDTH / 2.0 - 0.004), 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=metal_color,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.028, length=0.008),
            origin=Origin(
                xyz=(-sign * (WHEEL_WIDTH / 2.0 - 0.004), 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=metal_color,
            name="inner_hub",
        )

        model.articulation(
            f"body_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(sign * wheel_axle_x, wheel_axle_y, wheel_axle_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=30.0),
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(door_hinge_x, door_origin_y, door_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=12.0, velocity=1.5),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, handle_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=HANDLE_TRAVEL, effort=45.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    door_hinge = object_model.get_articulation("body_to_door")
    handle_slide = object_model.get_articulation("body_to_handle")

    with ctx.pose({door_hinge: 0.0, handle_slide: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="panel",
            negative_elem="shell",
            min_gap=0.0005,
            max_gap=0.006,
            name="door sits just ahead of the body front frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="panel",
            elem_b="shell",
            min_overlap=0.20,
            name="closed door covers the front opening",
        )
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            elem_a="frame",
            elem_b="shell",
            margin=0.010,
            name="retracted handle stays within the rear spine footprint",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="frame",
            elem_b="shell",
            min_overlap=0.24,
            name="retracted handle remains deeply inserted in the body",
        )

    door_closed_aabb = ctx.part_world_aabb(door)
    body_aabb = ctx.part_world_aabb(body)
    with ctx.pose({door_hinge: 1.20}):
        door_open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "door swings outward from the front face",
        door_closed_aabb is not None
        and door_open_aabb is not None
        and body_aabb is not None
        and door_open_aabb[1][1] > body_aabb[1][1] + 0.08
        and door_open_aabb[0][0] < door_closed_aabb[0][0] + 0.02,
        details=f"closed={door_closed_aabb}, open={door_open_aabb}, body={body_aabb}",
    )

    handle_rest = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: HANDLE_TRAVEL}):
        handle_extended = ctx.part_world_position(handle)
        ctx.expect_within(
            handle,
            body,
            axes="xy",
            elem_a="frame",
            elem_b="shell",
            margin=0.012,
            name="extended handle stays centered in the rear spine channel",
        )
        ctx.expect_overlap(
            handle,
            body,
            axes="z",
            elem_a="frame",
            elem_b="shell",
            min_overlap=0.08,
            name="extended handle keeps retained insertion",
        )

    ctx.check(
        "handle extends upward",
        handle_rest is not None
        and handle_extended is not None
        and handle_extended[2] > handle_rest[2] + 0.10,
        details=f"rest={handle_rest}, extended={handle_extended}",
    )

    wheel_0_pos = ctx.part_world_position(wheel_0)
    wheel_1_pos = ctx.part_world_position(wheel_1)
    ctx.check(
        "wheels sit low and wide for transport",
        wheel_0_pos is not None
        and wheel_1_pos is not None
        and wheel_0_pos[2] < 0.09
        and wheel_1_pos[2] < 0.09
        and abs(wheel_0_pos[0] - wheel_1_pos[0]) > 0.20,
        details=f"wheel_0={wheel_0_pos}, wheel_1={wheel_1_pos}",
    )

    return ctx.report()


object_model = build_object_model()
