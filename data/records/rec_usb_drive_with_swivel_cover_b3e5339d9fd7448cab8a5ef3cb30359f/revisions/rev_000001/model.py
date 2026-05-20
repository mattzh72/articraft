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


BODY_LENGTH = 0.045
BODY_WIDTH = 0.016
BODY_THICKNESS = 0.007
BODY_CENTER_X = 0.0025
BODY_REAR_X = BODY_CENTER_X - BODY_LENGTH / 2.0
BODY_FRONT_X = BODY_CENTER_X + BODY_LENGTH / 2.0

PIVOT_X = BODY_REAR_X + 0.0010
USB_LENGTH = 0.014
USB_WIDTH = 0.0122
USB_THICKNESS = 0.0046
USB_WALL = 0.00075
USB_CENTER_X = BODY_FRONT_X + USB_LENGTH / 2.0 - 0.00025

COVER_CLEARANCE = 0.0010
COVER_RAIL_Y = 0.0016
COVER_ARM_Y = BODY_WIDTH / 2.0 + COVER_CLEARANCE + COVER_RAIL_Y / 2.0
COVER_HEIGHT = BODY_THICKNESS + 0.0022
COVER_ARM_LENGTH = 0.063
COVER_BRIDGE_X = 0.0020

PIN_RADIUS = 0.0022
PIN_CLEARANCE_RADIUS = 0.00275
PIVOT_LUG_RADIUS = 0.0043
PIVOT_LUG_LENGTH = 0.0030


def _rounded_body_shape() -> cq.Workplane:
    """Compact plastic USB body with softly radiused edges."""
    return (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_THICKNESS)
        .edges()
        .fillet(0.00115)
        .translate((BODY_CENTER_X, 0.0, 0.0))
    )


def _usb_shell_shape() -> cq.Workplane:
    """Hollow USB-A-style metal connector shell."""
    outer = cq.Workplane("XY").box(USB_LENGTH, USB_WIDTH, USB_THICKNESS)
    inner = cq.Workplane("XY").box(
        USB_LENGTH + 0.0020,
        USB_WIDTH - 2.0 * USB_WALL,
        USB_THICKNESS - 2.0 * USB_WALL,
    )
    return outer.cut(inner)


def _y_axis_cylinder(length: float, radius: float) -> cq.Workplane:
    """CadQuery cylinder whose axis is local Y instead of local Z."""
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _cover_lug(y: float) -> cq.Workplane:
    outer = _y_axis_cylinder(PIVOT_LUG_LENGTH, PIVOT_LUG_RADIUS).translate(
        (0.0, y, 0.0)
    )
    bore = _y_axis_cylinder(PIVOT_LUG_LENGTH + 0.0015, PIN_CLEARANCE_RADIUS).translate(
        (0.0, y, 0.0)
    )
    return outer.cut(bore)


def _cover_shape() -> cq.Workplane:
    """One connected U-shaped swivel cover, modeled in its pivot frame."""
    upper_arm = cq.Workplane("XY").box(
        COVER_ARM_LENGTH, COVER_RAIL_Y, COVER_HEIGHT
    ).translate((COVER_ARM_LENGTH / 2.0, COVER_ARM_Y, 0.0))
    lower_arm = cq.Workplane("XY").box(
        COVER_ARM_LENGTH, COVER_RAIL_Y, COVER_HEIGHT
    ).translate((COVER_ARM_LENGTH / 2.0, -COVER_ARM_Y, 0.0))
    front_bridge = cq.Workplane("XY").box(
        COVER_BRIDGE_X,
        2.0 * COVER_ARM_Y + COVER_RAIL_Y,
        COVER_HEIGHT,
    ).translate((COVER_ARM_LENGTH + COVER_BRIDGE_X / 2.0, 0.0, 0.0))

    return (
        upper_arm.union(lower_arm)
        .union(front_bridge)
        .union(_cover_lug(COVER_ARM_Y))
        .union(_cover_lug(-COVER_ARM_Y))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_drive_swivel_cover")

    matte_graphite = model.material("matte_graphite", rgba=(0.015, 0.017, 0.020, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.040, 0.045, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.36, 1.0))
    blue_plastic = model.material("blue_plastic", rgba=(0.04, 0.18, 0.72, 1.0))
    gold = model.material("gold_contacts", rgba=(1.0, 0.73, 0.22, 1.0))
    red_window = model.material("red_led_window", rgba=(0.95, 0.05, 0.02, 0.85))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shape(), "rounded_body", tolerance=0.00035),
        material=matte_graphite,
        name="body_shell",
    )
    body.visual(
        Box((0.010, 0.012, 0.0010)),
        origin=Origin(xyz=(BODY_FRONT_X - 0.0020, 0.0, 0.0)),
        material=satin_black,
        name="front_collar",
    )
    body.visual(
        mesh_from_cadquery(_usb_shell_shape(), "usb_shell", tolerance=0.00025),
        origin=Origin(xyz=(USB_CENTER_X, 0.0, 0.0)),
        material=brushed_metal,
        name="connector_shell",
    )
    body.visual(
        Box((USB_LENGTH + 0.0008, 0.0078, 0.0008)),
        origin=Origin(xyz=(USB_CENTER_X - 0.0003, 0.0, -0.00085)),
        material=blue_plastic,
        name="connector_tongue",
    )
    for index, y in enumerate((-0.0033, -0.0011, 0.0011, 0.0033)):
        body.visual(
            Box((0.0042, 0.00105, 0.00016)),
            origin=Origin(xyz=(USB_CENTER_X + 0.0015, y, -0.00037)),
            material=gold,
            name=f"contact_{index}",
        )
    for index, x in enumerate((-0.006, 0.001, 0.008)):
        body.visual(
            Box((0.0020, 0.0105, 0.00022)),
            origin=Origin(xyz=(x, 0.0, BODY_THICKNESS / 2.0 + 0.00008)),
            material=satin_black,
            name=f"grip_rib_{index}",
        )
    body.visual(
        Box((0.0032, 0.0055, 0.00028)),
        origin=Origin(xyz=(0.0155, 0.0, BODY_THICKNESS / 2.0 + 0.0001)),
        material=red_window,
        name="status_window",
    )
    body.visual(
        Cylinder(radius=PIN_RADIUS, length=2.0 * COVER_ARM_Y + 0.008),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_pin",
    )
    for index, y in enumerate((-COVER_ARM_Y - 0.0022, COVER_ARM_Y + 0.0022)):
        body.visual(
            Cylinder(radius=0.0034, length=0.0012),
            origin=Origin(xyz=(PIVOT_X, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"pin_head_{index}",
        )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_shape(), "swivel_cover", tolerance=0.00035),
        material=brushed_metal,
        name="cover_yoke",
    )

    model.articulation(
        "cover_pivot",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    pivot = object_model.get_articulation("cover_pivot")

    ctx.check(
        "cover joint is continuous",
        pivot.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={pivot.articulation_type}",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="x",
        min_overlap=0.040,
        name="swivel cover closely spans the USB body",
    )
    ctx.expect_within(
        body,
        cover,
        axes="y",
        margin=0.0015,
        inner_elem="body_shell",
        outer_elem="cover_yoke",
        name="body is nested between the U-shaped cover arms",
    )
    ctx.expect_contact(
        body,
        cover,
        elem_a="pivot_pin",
        elem_b="cover_yoke",
        contact_tol=0.0007,
        name="cover lugs ride on the side pivot pin",
    )

    rest_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({pivot: math.pi}):
        spun_aabb = ctx.part_world_aabb(cover)

    rest_front = rest_aabb[1][0] if rest_aabb is not None else None
    spun_rear = spun_aabb[0][0] if spun_aabb is not None else None
    ctx.check(
        "continuous cover can swing behind the body",
        rest_front is not None
        and spun_rear is not None
        and rest_front > BODY_FRONT_X + USB_LENGTH
        and spun_rear < BODY_REAR_X - 0.035,
        details=f"rest_front={rest_front}, spun_rear={spun_rear}",
    )

    return ctx.report()


object_model = build_object_model()
