from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.112
BODY_WIDTH = 0.036
BODY_THICKNESS = 0.014
BODY_TOP_Z = BODY_THICKNESS * 0.5

TAIL_COLLAR_LENGTH = 0.008
TAIL_COLLAR_RADIUS = 0.007
BATTERY_CAP_LENGTH = 0.013

MAIN_BUTTON_X = 0.024
MAIN_BUTTON_SIZE = (0.020, 0.012)
MAIN_BUTTON_TRAVEL = 0.0015

FRONT_BUTTON_X = -0.004
FRONT_BUTTON_Y = 0.009
FRONT_BUTTON_SIZE = (0.011, 0.008)
FRONT_BUTTON_TRAVEL = 0.0012

GUARD_LENGTH = 0.028
GUARD_WIDTH = 0.017
GUARD_THICKNESS = 0.0018
GUARD_BARREL_RADIUS = 0.0012
GUARD_HINGE_X = MAIN_BUTTON_X
GUARD_HINGE_Y = 0.0092
GUARD_HINGE_Z = BODY_TOP_Z + 0.005


def _button_shape(length: float, width: float, cap_height: float, stem_height: float) -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .box(length, width, cap_height, centered=(True, True, False))
        .edges("|Z")
        .fillet(min(width * 0.22, cap_height * 0.45, 0.0018))
    )
    stem = (
        cq.Workplane("XY")
        .box(length * 0.72, width * 0.72, stem_height, centered=(True, True, False))
        .edges("|Z")
        .fillet(min(width * 0.14, stem_height * 0.35, 0.0010))
        .translate((0.0, 0.0, -stem_height))
    )
    return cap.union(stem)


def _thumb_guard_shape() -> cq.Workplane:
    flap_center_y = -GUARD_WIDTH * 0.5 + GUARD_BARREL_RADIUS
    flap_center_z = -GUARD_BARREL_RADIUS - GUARD_THICKNESS * 0.5 + 0.00025
    flap = (
        cq.Workplane("XY")
        .box(GUARD_LENGTH, GUARD_WIDTH, GUARD_THICKNESS, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0012)
        .translate((0.0, flap_center_y, flap_center_z))
    )
    barrel = (
        cq.Workplane("YZ")
        .circle(GUARD_BARREL_RADIUS)
        .extrude(GUARD_LENGTH)
        .translate((-GUARD_LENGTH * 0.5, 0.0, 0.0))
    )
    return barrel.union(flap)


def _battery_cap_shape() -> cq.Workplane:
    core = (
        cq.Workplane("YZ")
        .circle(TAIL_COLLAR_RADIUS)
        .extrude(BATTERY_CAP_LENGTH)
        .translate((-BATTERY_CAP_LENGTH * 0.5, 0.0, 0.0))
    )
    grip_ring = (
        cq.Workplane("YZ")
        .circle(TAIL_COLLAR_RADIUS * 1.08)
        .extrude(BATTERY_CAP_LENGTH * 0.34)
        .translate((-BATTERY_CAP_LENGTH * 0.5, 0.0, 0.0))
    )
    slot = (
        cq.Workplane("YZ")
        .box(BATTERY_CAP_LENGTH * 0.40, 0.0012, TAIL_COLLAR_RADIUS * 1.25, centered=(True, True, True))
        .translate((-BATTERY_CAP_LENGTH + 0.0012, 0.0, 0.0))
    )
    return core.union(grip_ring).cut(slot)


def _body_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_THICKNESS, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0043)
    )
    shell = (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(MAIN_BUTTON_X, 0.0)
        .rect(MAIN_BUTTON_SIZE[0], MAIN_BUTTON_SIZE[1])
        .cutBlind(-0.0042)
    )
    shell = (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(FRONT_BUTTON_X, -FRONT_BUTTON_Y)
        .rect(FRONT_BUTTON_SIZE[0], FRONT_BUTTON_SIZE[1])
        .cutBlind(-0.0038)
    )
    shell = (
        shell.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(FRONT_BUTTON_X, FRONT_BUTTON_Y)
        .rect(FRONT_BUTTON_SIZE[0], FRONT_BUTTON_SIZE[1])
        .cutBlind(-0.0038)
    )

    collar = (
        cq.Workplane("YZ")
        .circle(TAIL_COLLAR_RADIUS)
        .extrude(TAIL_COLLAR_LENGTH)
        .translate((-BODY_LENGTH * 0.5 - TAIL_COLLAR_LENGTH, 0.0, 0.0))
    )

    front_bumper = (
        cq.Workplane("YZ")
        .circle(0.0046)
        .extrude(0.004)
        .translate((BODY_LENGTH * 0.5 - 0.002, 0.0, 0.0))
    )

    hinge_post_offset = GUARD_LENGTH * 0.5 - 0.0038
    post_height = GUARD_HINGE_Z - BODY_TOP_Z
    hinge_posts = (
        cq.Workplane("XY")
        .box(0.0032, 0.0036, post_height, centered=(True, True, False))
        .translate((GUARD_HINGE_X - hinge_post_offset, GUARD_HINGE_Y, BODY_TOP_Z))
    )
    hinge_posts = hinge_posts.union(
        cq.Workplane("XY")
        .box(0.0032, 0.0036, post_height, centered=(True, True, False))
        .translate((GUARD_HINGE_X + hinge_post_offset, GUARD_HINGE_Y, BODY_TOP_Z))
    )

    return shell.union(collar).union(front_bumper).union(hinge_posts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="presentation_clicker")

    body_black = model.material("body_black", rgba=(0.11, 0.12, 0.13, 1.0))
    guard_black = model.material("guard_black", rgba=(0.16, 0.17, 0.18, 1.0))
    button_black = model.material("button_black", rgba=(0.18, 0.18, 0.19, 1.0))
    laser_red = model.material("laser_red", rgba=(0.65, 0.10, 0.10, 1.0))
    cap_metal = model.material("cap_metal", rgba=(0.66, 0.67, 0.69, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "clicker_body"),
        material=body_black,
        name="body_shell",
    )

    thumb_guard = model.part("thumb_guard")
    thumb_guard.visual(
        mesh_from_cadquery(_thumb_guard_shape(), "thumb_guard"),
        material=guard_black,
        name="guard_shell",
    )

    main_button = model.part("main_button")
    main_button.visual(
        mesh_from_cadquery(_button_shape(MAIN_BUTTON_SIZE[0], MAIN_BUTTON_SIZE[1], 0.0018, 0.0020), "main_button"),
        material=laser_red,
        name="button_cap",
    )

    front_button_0 = model.part("front_button_0")
    front_button_0.visual(
        mesh_from_cadquery(_button_shape(FRONT_BUTTON_SIZE[0], FRONT_BUTTON_SIZE[1], 0.0016, 0.0018), "front_button_0"),
        material=button_black,
        name="button_cap",
    )

    front_button_1 = model.part("front_button_1")
    front_button_1.visual(
        mesh_from_cadquery(_button_shape(FRONT_BUTTON_SIZE[0], FRONT_BUTTON_SIZE[1], 0.0016, 0.0018), "front_button_1"),
        material=button_black,
        name="button_cap",
    )

    battery_cap = model.part("battery_cap")
    battery_cap.visual(
        mesh_from_cadquery(_battery_cap_shape(), "battery_cap"),
        material=cap_metal,
        name="cap_shell",
        origin=Origin(xyz=(-BATTERY_CAP_LENGTH * 0.5, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_thumb_guard",
        ArticulationType.REVOLUTE,
        parent=body,
        child=thumb_guard,
        origin=Origin(xyz=(GUARD_HINGE_X, GUARD_HINGE_Y, GUARD_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    model.articulation(
        "body_to_main_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=main_button,
        origin=Origin(xyz=(MAIN_BUTTON_X, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=MAIN_BUTTON_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_front_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_button_0,
        origin=Origin(xyz=(FRONT_BUTTON_X, -FRONT_BUTTON_Y, BODY_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=FRONT_BUTTON_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_front_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_button_1,
        origin=Origin(xyz=(FRONT_BUTTON_X, FRONT_BUTTON_Y, BODY_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.05,
            lower=0.0,
            upper=FRONT_BUTTON_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_battery_cap",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=battery_cap,
        origin=Origin(xyz=(-BODY_LENGTH * 0.5 - TAIL_COLLAR_LENGTH, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    thumb_guard = object_model.get_part("thumb_guard")
    main_button = object_model.get_part("main_button")
    front_button_0 = object_model.get_part("front_button_0")
    front_button_1 = object_model.get_part("front_button_1")
    battery_cap = object_model.get_part("battery_cap")

    guard_joint = object_model.get_articulation("body_to_thumb_guard")
    main_button_joint = object_model.get_articulation("body_to_main_button")
    front_button_0_joint = object_model.get_articulation("body_to_front_button_0")
    front_button_1_joint = object_model.get_articulation("body_to_front_button_1")
    cap_joint = object_model.get_articulation("body_to_battery_cap")

    ctx.expect_gap(
        thumb_guard,
        main_button,
        axis="z",
        min_gap=0.0002,
        max_gap=0.0030,
        name="guard hovers just above the main button when closed",
    )
    ctx.expect_overlap(
        thumb_guard,
        main_button,
        axes="xy",
        min_overlap=0.010,
        name="guard covers the main button footprint",
    )

    main_rest = ctx.part_world_position(main_button)
    with ctx.pose({main_button_joint: MAIN_BUTTON_TRAVEL}):
        main_pressed = ctx.part_world_position(main_button)
    ctx.check(
        "main button depresses downward",
        main_rest is not None
        and main_pressed is not None
        and main_pressed[2] < main_rest[2] - 0.0010,
        details=f"rest={main_rest}, pressed={main_pressed}",
    )

    front_0_rest = ctx.part_world_position(front_button_0)
    front_1_rest_for_0 = ctx.part_world_position(front_button_1)
    with ctx.pose({front_button_0_joint: FRONT_BUTTON_TRAVEL}):
        front_0_pressed = ctx.part_world_position(front_button_0)
        front_1_during_0 = ctx.part_world_position(front_button_1)
    ctx.check(
        "front button 0 depresses independently",
        front_0_rest is not None
        and front_0_pressed is not None
        and front_1_rest_for_0 is not None
        and front_1_during_0 is not None
        and front_0_pressed[2] < front_0_rest[2] - 0.0008
        and abs(front_1_during_0[2] - front_1_rest_for_0[2]) < 1e-6,
        details=(
            f"front_0_rest={front_0_rest}, front_0_pressed={front_0_pressed}, "
            f"front_1_rest={front_1_rest_for_0}, front_1_during_press={front_1_during_0}"
        ),
    )

    front_1_rest = ctx.part_world_position(front_button_1)
    front_0_rest_for_1 = ctx.part_world_position(front_button_0)
    with ctx.pose({front_button_1_joint: FRONT_BUTTON_TRAVEL}):
        front_1_pressed = ctx.part_world_position(front_button_1)
        front_0_during_1 = ctx.part_world_position(front_button_0)
    ctx.check(
        "front button 1 depresses independently",
        front_1_rest is not None
        and front_1_pressed is not None
        and front_0_rest_for_1 is not None
        and front_0_during_1 is not None
        and front_1_pressed[2] < front_1_rest[2] - 0.0008
        and abs(front_0_during_1[2] - front_0_rest_for_1[2]) < 1e-6,
        details=(
            f"front_1_rest={front_1_rest}, front_1_pressed={front_1_pressed}, "
            f"front_0_rest={front_0_rest_for_1}, front_0_during_press={front_0_during_1}"
        ),
    )

    guard_closed = ctx.part_world_aabb(thumb_guard)
    with ctx.pose({guard_joint: math.radians(90.0)}):
        guard_open = ctx.part_world_aabb(thumb_guard)
    ctx.check(
        "thumb guard opens upward",
        guard_closed is not None
        and guard_open is not None
        and guard_open[1][2] > guard_closed[1][2] + 0.010,
        details=f"closed={guard_closed}, open={guard_open}",
    )

    ctx.expect_origin_distance(
        battery_cap,
        body,
        axes="yz",
        max_dist=0.0001,
        name="battery cap stays centered on the clicker axis",
    )
    cap_rest = ctx.part_world_position(battery_cap)
    with ctx.pose({cap_joint: 1.7}):
        cap_rotated = ctx.part_world_position(battery_cap)
    ctx.check(
        "battery cap rotates in place",
        cap_rest is not None
        and cap_rotated is not None
        and abs(cap_rest[0] - cap_rotated[0]) < 1e-6
        and abs(cap_rest[1] - cap_rotated[1]) < 1e-6
        and abs(cap_rest[2] - cap_rotated[2]) < 1e-6,
        details=f"rest={cap_rest}, rotated={cap_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
