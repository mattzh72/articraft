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


BODY_LENGTH = 0.046
BODY_WIDTH = 0.018
BODY_HEIGHT = 0.008
BODY_FRONT_X = BODY_LENGTH / 2.0
PIVOT_X = -0.017
PIVOT_Y = -0.010


def _rounded_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
    return shell.edges().fillet(0.0018)


def _usb_a_shell() -> cq.Workplane:
    """Thin rectangular USB-A metal sleeve, open along the insertion axis."""
    outer = cq.Workplane("XY").box(0.018, 0.0124, 0.0046)
    inner = cq.Workplane("XY").box(0.021, 0.0106, 0.0032)
    shell = outer.cut(inner)
    return shell.edges("|X").fillet(0.00035)


def _swivel_cover_frame() -> cq.Workplane:
    """A flat U-shaped stainless cover with a washer-like pivot eye."""
    sheet_z = BODY_HEIGHT / 2.0 + 0.0020
    sheet_t = 0.0012
    rail_len = 0.064
    rail_w = 0.0030
    near_y = -0.0040
    far_y = 0.0220
    bridge_x = 0.0610

    near_rail = (
        cq.Workplane("XY")
        .box(rail_len, rail_w, sheet_t)
        .translate((0.0305, near_y, sheet_z))
    )
    far_rail = (
        cq.Workplane("XY")
        .box(rail_len, rail_w, sheet_t)
        .translate((0.0305, far_y, sheet_z))
    )
    front_bridge = (
        cq.Workplane("XY")
        .box(0.0030, far_y - near_y + rail_w, sheet_t)
        .translate((bridge_x, (near_y + far_y) / 2.0, sheet_z))
    )
    pivot_tab = (
        cq.Workplane("XY")
        .box(0.0100, 0.0050, sheet_t)
        .translate((0.0028, near_y * 0.58, sheet_z))
    )
    eye_outer = (
        cq.Workplane("XY")
        .circle(0.0044)
        .extrude(sheet_t)
        .translate((0.0, 0.0, sheet_z - sheet_t / 2.0))
    )
    eye_hole = (
        cq.Workplane("XY")
        .circle(0.0021)
        .extrude(sheet_t * 1.8)
        .translate((0.0, 0.0, sheet_z - sheet_t * 0.9))
    )
    pivot_eye = eye_outer.cut(eye_hole)
    cover = near_rail.union(far_rail).union(front_bridge).union(pivot_tab).union(pivot_eye)
    return cover.edges().fillet(0.00025)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    plastic = model.material("matte_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.055, 0.058, 0.063, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.74, 0.76, 0.77, 1.0))
    dark_steel = model.material("recessed_axis_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    gold = model.material("contact_gold", rgba=(1.0, 0.72, 0.22, 1.0))
    led_green = model.material("tiny_green_led", rgba=(0.10, 0.85, 0.34, 0.85))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shell(), "rounded_body_shell"),
        material=plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.008, BODY_WIDTH + 0.0015, BODY_HEIGHT + 0.0010)),
        origin=Origin(xyz=(BODY_FRONT_X - 0.002, 0.0, 0.0)),
        material=rubber,
        name="connector_collar",
    )
    body.visual(
        mesh_from_cadquery(_usb_a_shell(), "usb_a_metal_shell"),
        origin=Origin(xyz=(BODY_FRONT_X + 0.008, 0.0, 0.0)),
        material=steel,
        name="connector_shell",
    )
    body.visual(
        Box((0.018, 0.0062, 0.0012)),
        origin=Origin(xyz=(BODY_FRONT_X + 0.007, 0.0, -0.0007)),
        material=rubber,
        name="connector_tongue",
    )
    for index, y in enumerate((-0.0027, -0.0009, 0.0009, 0.0027)):
        body.visual(
            Box((0.0032, 0.00105, 0.00025)),
            origin=Origin(xyz=(BODY_FRONT_X + 0.014, y, -0.00008)),
            material=gold,
            name=f"contact_pad_{index}",
        )
    body.visual(
        Cylinder(radius=0.0030, length=0.0012),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, BODY_HEIGHT / 2.0 + 0.0006)),
        material=dark_steel,
        name="pivot_boss",
    )
    body.visual(
        Cylinder(radius=0.00135, length=0.0032),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, BODY_HEIGHT / 2.0 + 0.0022)),
        material=steel,
        name="pivot_pin_head",
    )
    body.visual(
        Box((0.0040, 0.0013, 0.0005)),
        origin=Origin(xyz=(-0.001, 0.0054, BODY_HEIGHT / 2.0 + 0.00012)),
        material=led_green,
        name="status_led",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_swivel_cover_frame(), "u_shaped_swivel_cover"),
        material=steel,
        name="cover_frame",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.18, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("body_to_cover")

    ctx.check(
        "cover_joint_is_continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={swivel.articulation_type!r}",
    )
    ctx.check("cover_axis_is_vertical_pin", tuple(swivel.axis) == (0.0, 0.0, 1.0), details=f"axis={swivel.axis!r}")

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_frame",
        elem_b="body_shell",
        min_overlap=0.012,
        name="u_cover_wraps_body_footprint",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_frame",
        negative_elem="body_shell",
        min_gap=0.0007,
        max_gap=0.0025,
        name="cover_sheet_clears_body_top",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="xy",
        elem_a="pivot_boss",
        elem_b="cover_frame",
        min_overlap=0.003,
        name="axis_boss_sits_under_cover_eye",
    )

    rest_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(cover)
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="cover_frame",
            negative_elem="body_shell",
            min_gap=0.0007,
            name="turned_cover_still_clears_body_top",
        )

    if rest_aabb is not None and turned_aabb is not None:
        rest_center_y = (float(rest_aabb[0][1]) + float(rest_aabb[1][1])) / 2.0
        turned_center_y = (float(turned_aabb[0][1]) + float(turned_aabb[1][1])) / 2.0
        ctx.check(
            "cover_swivels_around_side_pin",
            turned_center_y > rest_center_y + 0.015,
            details=f"rest_center_y={rest_center_y:.4f}, turned_center_y={turned_center_y:.4f}",
        )
    else:
        ctx.fail("cover_swivels_around_side_pin", "Missing cover AABB in rest or turned pose.")

    return ctx.report()


object_model = build_object_model()
