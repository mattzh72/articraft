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


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    """Small filleted rectangular solid, authored in meters."""
    return cq.Workplane("XY").box(length, width, height).edges("|X").fillet(radius)


def _usb_connector(length: float, width: float, height: float, wall: float) -> cq.Workplane:
    """A thin-walled USB-A shell with an actual rectangular opening."""
    outer = cq.Workplane("XY").box(length, width, height)
    inner = cq.Workplane("XY").box(length + 0.004, width - 2.0 * wall, height - 2.0 * wall)
    return outer.cut(inner)


def _cover_frame() -> cq.Workplane:
    """U-shaped stamped cover frame, local origin on the side pivot axis."""
    plate_t = 0.0012
    zc = 0.0065

    # World-space design targets at q=0 are converted to the child frame whose
    # origin is the pivot at (-0.021, -0.0105, 0).
    x0, x1 = -0.031, 0.024
    pivot_x, pivot_y = -0.021, -0.0105
    rail_len = x1 - x0
    rail_cx = (x0 + x1) * 0.5 - pivot_x

    near_rail = cq.Workplane("XY").box(rail_len, 0.0030, plate_t).translate(
        (rail_cx, -0.0120 - pivot_y, zc)
    )
    far_rail = cq.Workplane("XY").box(rail_len, 0.0030, plate_t).translate(
        (rail_cx, 0.0120 - pivot_y, zc)
    )
    rear_bridge = cq.Workplane("XY").box(0.0040, 0.0270, plate_t).translate(
        (-0.0295 - pivot_x, 0.0 - pivot_y, zc)
    )
    return near_rail.union(far_rail).union(rear_bridge)


def _pivot_eye() -> cq.Workplane:
    """Flat washer around the side pin, with clearance through the center."""
    plate_t = 0.0012
    z_bottom = 0.0065 - plate_t * 0.5
    outer = cq.Workplane("XY").circle(0.0054).extrude(plate_t)
    inner = cq.Workplane("XY").circle(0.0028).extrude(plate_t + 0.0004).translate((0.0, 0.0, -0.0002))
    return outer.cut(inner).translate((0.0, 0.0, z_bottom))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_usb_drive")

    soft_black = model.material("soft_black", color=(0.015, 0.017, 0.020, 1.0))
    graphite = model.material("graphite_plastic", color=(0.055, 0.060, 0.066, 1.0))
    satin_metal = model.material("satin_metal", color=(0.70, 0.72, 0.70, 1.0))
    blue_insert = model.material("blue_insert", color=(0.04, 0.22, 0.65, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box(0.046, 0.018, 0.008, 0.0024), "main_shell"),
        origin=Origin(xyz=(-0.0045, 0.0, 0.0)),
        material=graphite,
        name="main_shell",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box(0.0065, 0.018, 0.008, 0.0022), "rear_cap"),
        origin=Origin(xyz=(-0.0305, 0.0, 0.0)),
        material=soft_black,
        name="rear_cap",
    )
    body.visual(
        mesh_from_cadquery(_rounded_box(0.0105, 0.016, 0.0072, 0.0018), "front_collar"),
        origin=Origin(xyz=(0.0220, 0.0, 0.0)),
        material=soft_black,
        name="front_collar",
    )
    body.visual(
        mesh_from_cadquery(_usb_connector(0.020, 0.012, 0.0045, 0.00065), "connector_shell"),
        origin=Origin(xyz=(0.0368, 0.0, 0.0)),
        material=satin_metal,
        name="connector_shell",
    )
    body.visual(
        Box((0.018, 0.0052, 0.0010)),
        origin=Origin(xyz=(0.0355, 0.0, -0.0002)),
        material=blue_insert,
        name="connector_tongue",
    )
    body.visual(
        Box((0.016, 0.0045, 0.00065)),
        origin=Origin(xyz=(-0.005, 0.0, 0.00432)),
        material=soft_black,
        name="top_grip",
    )

    pivot_xyz = (-0.021, -0.0105, 0.0)
    body.visual(
        Cylinder(radius=0.0021, length=0.0124),
        origin=Origin(xyz=(pivot_xyz[0], pivot_xyz[1], 0.0011)),
        material=satin_metal,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.0013),
        origin=Origin(xyz=(pivot_xyz[0], pivot_xyz[1], 0.00785)),
        material=satin_metal,
        name="pin_cap",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_frame(), "cover_frame"),
        material=satin_metal,
        name="cover_frame",
    )
    cover.visual(
        mesh_from_cadquery(_pivot_eye(), "pivot_eye"),
        material=satin_metal,
        name="pivot_eye",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=pivot_xyz),
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
        "cover joint is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type is {swivel.articulation_type}",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="x",
        elem_a="cover_frame",
        elem_b="main_shell",
        min_overlap=0.038,
        name="U cover spans the body length",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="y",
        elem_a="cover_frame",
        elem_b="main_shell",
        min_overlap=0.017,
        name="U cover wraps across body width",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_frame",
        negative_elem="main_shell",
        min_gap=0.001,
        max_gap=0.003,
        name="cover clears top of body",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="pin_cap",
        negative_elem="pivot_eye",
        min_gap=0.0,
        max_gap=0.0004,
        name="pin cap captures pivot eye",
    )

    # The fixed body is visibly stepped but coaxial: rear cap, main shell, and
    # front collar all share the same long centerline and nest inside the same
    # Y/Z envelope.
    body_sections = ("rear_cap", "main_shell", "front_collar")
    centers = []
    for elem in body_sections:
        aabb = ctx.part_element_world_aabb(body, elem=elem)
        if aabb is not None:
            lo, hi = aabb
            centers.append((elem, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5))
    ctx.check(
        "fixed body sections are coaxial",
        len(centers) == 3
        and max(abs(y) for _, y, _ in centers) < 0.0008
        and max(abs(z) for _, _, z in centers) < 0.0008,
        details=f"section centers={centers}",
    )

    closed_aabb = ctx.part_element_world_aabb(cover, elem="cover_frame")
    with ctx.pose({swivel: math.pi / 2.0}):
        rotated_aabb = ctx.part_element_world_aabb(cover, elem="cover_frame")
    moved = False
    if closed_aabb is not None and rotated_aabb is not None:
        c_lo, c_hi = closed_aabb
        r_lo, r_hi = rotated_aabb
        closed_center = ((c_lo[0] + c_hi[0]) * 0.5, (c_lo[1] + c_hi[1]) * 0.5)
        rotated_center = ((r_lo[0] + r_hi[0]) * 0.5, (r_lo[1] + r_hi[1]) * 0.5)
        closed_size = (c_hi[0] - c_lo[0], c_hi[1] - c_lo[1])
        rotated_size = (r_hi[0] - r_lo[0], r_hi[1] - r_lo[1])
        moved = (
            abs(rotated_center[0] - closed_center[0]) > 0.018
            and rotated_size[1] > closed_size[1] + 0.020
            and rotated_size[0] < closed_size[0] - 0.020
        )
    ctx.check(
        "cover sweeps around side pivot",
        moved,
        details=f"closed={closed_aabb}, rotated={rotated_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
