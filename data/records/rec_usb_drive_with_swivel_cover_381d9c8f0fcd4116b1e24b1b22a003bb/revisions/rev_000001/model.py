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


BODY_LEN = 0.046
BODY_W = 0.018
BODY_H = 0.008
PIVOT_X = -0.018
PIVOT_Z = 0.010


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]):
    """CadQuery cylinder whose axis is the local Y direction."""
    x, y, z = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((x, y - length / 2.0, z))
    )


def _rounded_body_shape():
    return cq.Workplane("XY").box(BODY_LEN, BODY_W, BODY_H).edges().fillet(0.0013)


def _usb_connector_shell():
    body_front_x = BODY_LEN / 2.0
    plug_len = 0.016
    plug_w = 0.012
    plug_h = 0.0045
    wall = 0.00055
    center_x = body_front_x + plug_len / 2.0

    top = _cq_box((plug_len, plug_w, wall), (center_x, 0.0, plug_h / 2.0 - wall / 2.0))
    bottom = _cq_box((plug_len, plug_w, wall), (center_x, 0.0, -plug_h / 2.0 + wall / 2.0))
    side_a = _cq_box((plug_len, wall, plug_h), (center_x, plug_w / 2.0 - wall / 2.0, 0.0))
    side_b = _cq_box((plug_len, wall, plug_h), (center_x, -plug_w / 2.0 + wall / 2.0, 0.0))
    rear_lip = _cq_box((0.0010, plug_w, plug_h), (body_front_x + 0.0005, 0.0, 0.0))
    return top.union(bottom).union(side_a).union(side_b).union(rear_lip)


def _swivel_cover_shape():
    rail_len = 0.065
    rail_start = -0.003
    rail_center_x = rail_start + rail_len / 2.0
    rail_y = 0.0122
    rail_t = 0.0022
    cover_h = 0.0074
    bridge_t = 0.0042

    side_a = _cq_box((rail_len, rail_t, cover_h), (rail_center_x, rail_y, 0.0))
    side_b = _cq_box((rail_len, rail_t, cover_h), (rail_center_x, -rail_y, 0.0))
    bridge = _cq_box((bridge_t, 2.0 * rail_y + rail_t, cover_h), (0.062, 0.0, 0.0))

    collar_r = 0.0046
    collar_len = 0.0040
    collar_a = _cylinder_y(collar_r, collar_len, (0.0, rail_y, 0.0))
    collar_b = _cylinder_y(collar_r, collar_len, (0.0, -rail_y, 0.0))

    frame = side_a.union(side_b).union(bridge).union(collar_a).union(collar_b)
    pivot_bore = _cylinder_y(0.00245, 0.036, (0.0, 0.0, 0.0))
    return frame.cut(pivot_bore)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_drive_swivel_cover")

    body_plastic = model.material("satin_black_plastic", color=(0.015, 0.018, 0.022, 1.0))
    rubber = model.material("black_insert", color=(0.002, 0.002, 0.002, 1.0))
    metal = model.material("brushed_silver_metal", color=(0.72, 0.74, 0.74, 1.0))
    dark_metal = model.material("dark_pin_metal", color=(0.20, 0.21, 0.22, 1.0))
    gold = model.material("gold_contacts", color=(0.96, 0.69, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shape(), "rounded_usb_body", tolerance=0.0004),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_usb_connector_shell(), "hollow_usb_connector", tolerance=0.00025),
        material=metal,
        name="connector_shell",
    )
    body.visual(
        Box((0.0165, 0.0064, 0.0009)),
        origin=Origin(xyz=(BODY_LEN / 2.0 + 0.0080, 0.0, 0.0)),
        material=rubber,
        name="connector_tongue",
    )
    for i, y in enumerate((-0.0027, -0.0009, 0.0009, 0.0027)):
        body.visual(
            Box((0.0048, 0.0010, 0.00022)),
            origin=Origin(xyz=(BODY_LEN / 2.0 + 0.0122, y, 0.00056)),
            material=gold,
            name=f"contact_{i}",
        )

    # A single fixed sleeve and coaxial pin make the side pivot visibly supported.
    body.visual(
        Box((0.010, 0.014, 0.0042)),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0054)),
        material=body_plastic,
        name="sleeve_pedestal",
    )
    body.visual(
        Cylinder(radius=0.0030, length=0.019),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="axis_sleeve",
    )
    body.visual(
        Cylinder(radius=0.00250, length=0.033),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_pin",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_swivel_cover_shape(), "u_shaped_swivel_cover", tolerance=0.00035),
        material=metal,
        name="cover_frame",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    cover_joint = object_model.get_articulation("body_to_cover")

    ctx.allow_overlap(
        body,
        cover,
        elem_a="pivot_pin",
        elem_b="cover_frame",
        reason="The side pin is intentionally captured in the cover bore; the tiny radial interference represents a fitted swivel joint.",
    )

    ctx.check(
        "cover joint is continuous",
        getattr(cover_joint, "articulation_type", None) == ArticulationType.CONTINUOUS,
        details=f"joint type is {getattr(cover_joint, 'articulation_type', None)!r}",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="x",
        elem_a="cover_frame",
        elem_b="body_shell",
        min_overlap=0.040,
        name="cover spans the compact body",
    )
    ctx.expect_within(
        body,
        cover,
        axes="y",
        inner_elem="body_shell",
        outer_elem="cover_frame",
        margin=0.0,
        name="body sits between the cover cheeks",
    )
    ctx.expect_within(
        body,
        cover,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="cover_frame",
        margin=0.0025,
        name="pivot pin passes through the cover bore",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="y",
        elem_a="pivot_pin",
        elem_b="cover_frame",
        min_overlap=0.026,
        name="cover is retained on the side pin",
    )

    rest_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_joint: math.pi}):
        flipped_aabb = ctx.part_world_aabb(cover)

    ctx.check(
        "cover rotates to the opposite side",
        rest_aabb is not None
        and flipped_aabb is not None
        and rest_aabb[1][0] > 0.040
        and flipped_aabb[0][0] < -0.078,
        details=f"rest_aabb={rest_aabb}, flipped_aabb={flipped_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
