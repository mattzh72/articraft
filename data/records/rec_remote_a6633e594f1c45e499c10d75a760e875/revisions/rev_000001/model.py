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


BODY_LEN = 0.060
BODY_WID = 0.028
BODY_THK = 0.010
BODY_FILLET = 0.0024

SLOT_CENTER_X = 0.016
SLOT_CENTER_Z = 0.0016
SLOT_LEN = 0.013
SLOT_DEPTH = 0.0042
SLOT_HEIGHT = 0.0036

SLIDER_TRAVEL = 0.006
SLIDER_TONGUE_LEN = 0.0105
SLIDER_TONGUE_DEPTH = 0.0034
SLIDER_TONGUE_HEIGHT = 0.0028
SLIDER_PAD_LEN = 0.0085
SLIDER_PAD_DEPTH = 0.0034
SLIDER_PAD_HEIGHT = 0.0052
SLIDER_PAD_CENTER_Y = 0.0027

DOOR_LEN = 0.024
DOOR_WID = 0.012
DOOR_THK = 0.0011
DOOR_CLEAR = 0.0003
BATTERY_CAVITY_DEPTH = 0.0056

DOOR_AXIS_X = -0.012
DOOR_AXIS_Z = -0.0038
DOOR_KNUCKLE_RADIUS = 0.0009
DOOR_KNUCKLE_LEN = 0.0104
HINGE_EAR_RADIUS = 0.0011
HINGE_EAR_LEN = 0.0030
HINGE_EAR_CENTER_Y = 0.0070
DOOR_PANEL_BOTTOM_Z = -0.0012


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_LEN, BODY_WID, BODY_THK)
    body = body.edges().fillet(BODY_FILLET)

    slider_slot = (
        cq.Workplane("XY")
        .box(SLOT_LEN, SLOT_DEPTH + 0.0004, SLOT_HEIGHT, centered=(True, True, True))
        .translate(
            (
                SLOT_CENTER_X,
                BODY_WID / 2.0 - SLOT_DEPTH / 2.0 + 0.0002,
                SLOT_CENTER_Z,
            )
        )
    )

    battery_opening = (
        cq.Workplane("XY")
        .box(
            DOOR_LEN + 2.0 * DOOR_CLEAR,
            DOOR_WID + 2.0 * DOOR_CLEAR,
            BATTERY_CAVITY_DEPTH,
            centered=(False, True, False),
        )
        .translate((DOOR_AXIS_X, 0.0, -BODY_THK / 2.0))
    )

    finger_notch = (
        cq.Workplane("YZ")
        .center(0.0, -BODY_THK / 2.0 + 0.0019)
        .circle(0.0025)
        .extrude(0.0032)
        .translate((DOOR_AXIS_X + DOOR_LEN - 0.0012, 0.0, 0.0))
    )

    body = body.cut(slider_slot)
    body = body.cut(battery_opening)
    body = body.cut(finger_notch)

    for sign in (-1.0, 1.0):
        hinge_ear = (
            cq.Workplane("XZ")
            .center(DOOR_AXIS_X, DOOR_AXIS_Z)
            .circle(HINGE_EAR_RADIUS)
            .extrude(HINGE_EAR_LEN, both=True)
            .translate((0.0, sign * HINGE_EAR_CENTER_Y, 0.0))
        )
        body = body.union(hinge_ear)

    return body


def _slider_shape() -> cq.Workplane:
    tongue = cq.Workplane("XY").box(
        SLIDER_TONGUE_LEN,
        SLIDER_TONGUE_DEPTH,
        SLIDER_TONGUE_HEIGHT,
    )

    pad = (
        cq.Workplane("XY")
        .box(
            SLIDER_PAD_LEN,
            SLIDER_PAD_DEPTH,
            SLIDER_PAD_HEIGHT,
        )
        .edges()
        .fillet(0.00055)
        .translate((0.0, SLIDER_PAD_CENTER_Y, 0.0))
    )

    return tongue.union(pad)


def _door_panel_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            DOOR_LEN,
            DOOR_WID,
            DOOR_THK,
            centered=(False, True, False),
        )
        .edges("|Z")
        .fillet(0.0007)
        .translate((0.0, 0.0, DOOR_PANEL_BOTTOM_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_remote")

    model.material("body_plastic", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("trim_plastic", rgba=(0.20, 0.20, 0.21, 1.0))
    model.material("slider_plastic", rgba=(0.27, 0.28, 0.30, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "camera_remote_body"),
        material="body_plastic",
        name="shell",
    )

    shutter_slider = model.part("shutter_slider")
    shutter_slider.visual(
        mesh_from_cadquery(_slider_shape(), "camera_remote_slider"),
        material="slider_plastic",
        name="thumb",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        mesh_from_cadquery(_door_panel_shape(), "camera_remote_battery_panel"),
        material="trim_plastic",
        name="panel",
    )
    battery_door.visual(
        Cylinder(radius=DOOR_KNUCKLE_RADIUS, length=DOOR_KNUCKLE_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="trim_plastic",
        name="knuckle",
    )
    battery_door.visual(
        Box((0.0016, DOOR_WID * 0.55, DOOR_THK * 0.9)),
        origin=Origin(
            xyz=(
                DOOR_LEN - 0.0008,
                0.0,
                DOOR_PANEL_BOTTOM_Z + DOOR_THK / 2.0,
            )
        ),
        material="trim_plastic",
        name="latch_edge",
    )

    model.articulation(
        "side_shutter",
        ArticulationType.PRISMATIC,
        parent=body,
        child=shutter_slider,
        origin=Origin(
            xyz=(
                SLOT_CENTER_X,
                BODY_WID / 2.0 - SLOT_DEPTH / 2.0,
                SLOT_CENTER_Z,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDER_TRAVEL / 2.0,
            upper=SLIDER_TRAVEL / 2.0,
            effort=4.0,
            velocity=0.08,
        ),
    )

    model.articulation(
        "battery_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(DOOR_AXIS_X, 0.0, DOOR_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.55,
            effort=1.5,
            velocity=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shutter_slider = object_model.get_part("shutter_slider")
    battery_door = object_model.get_part("battery_door")
    side_shutter = object_model.get_articulation("side_shutter")
    battery_hinge = object_model.get_articulation("battery_hinge")

    slider_limits = side_shutter.motion_limits
    door_limits = battery_hinge.motion_limits

    ctx.expect_overlap(
        shutter_slider,
        body,
        axes="xz",
        min_overlap=0.003,
        name="slider stays aligned on the side of the remote",
    )
    ctx.expect_within(
        battery_door,
        body,
        axes="xy",
        margin=0.003,
        inner_elem="panel",
        outer_elem="shell",
        name="closed battery door stays within the remote footprint",
    )

    rest_slider_pos = ctx.part_world_position(shutter_slider)
    extended_slider_pos = None
    with ctx.pose({side_shutter: slider_limits.upper}):
        ctx.expect_overlap(
            shutter_slider,
            body,
            axes="z",
            min_overlap=0.002,
            name="slider remains guided at full travel",
        )
        extended_slider_pos = ctx.part_world_position(shutter_slider)

    ctx.check(
        "slider moves forward along the slot",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + 0.0025,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    closed_door_aabb = ctx.part_world_aabb(battery_door)
    open_door_aabb = None
    with ctx.pose({battery_hinge: door_limits.upper}):
        ctx.expect_gap(
            body,
            battery_door,
            axis="z",
            min_gap=0.006,
            positive_elem="shell",
            negative_elem="latch_edge",
            name="opened battery door free edge swings clear below the body",
        )
        open_door_aabb = ctx.part_world_aabb(battery_door)

    ctx.check(
        "battery door rotates downward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][2] < closed_door_aabb[0][2] - 0.008,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
