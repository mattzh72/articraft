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


def _rounded_body_shell() -> cq.Workplane:
    """Plastic thumb-drive body, authored around the side pivot at the origin."""
    return (
        cq.Workplane("XY")
        .box(0.044, 0.017, 0.0075)
        .translate((0.018, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.0030)
    )


def _swivel_cover_shell() -> cq.Workplane:
    """One continuous stamped-metal U cover with downturned side lips and a pin hole."""
    thickness = 0.0012
    lip_height = 0.0042

    cover = (
        cq.Workplane("XY")
        .box(0.062, 0.0032, thickness)
        .translate((0.031, 0.0118, 0.0))
    )
    cover = cover.union(
        cq.Workplane("XY")
        .box(0.062, 0.0032, thickness)
        .translate((0.031, -0.0118, 0.0))
    )
    cover = cover.union(
        cq.Workplane("XY")
        .box(0.0050, 0.0270, thickness)
        .translate((0.0625, 0.0, 0.0))
    )
    cover = cover.union(cq.Workplane("XY").cylinder(thickness, 0.0065))
    cover = cover.union(
        cq.Workplane("XY")
        .box(0.014, 0.0050, thickness)
        .translate((0.006, 0.0083, 0.0))
    )
    cover = cover.union(
        cq.Workplane("XY")
        .box(0.014, 0.0050, thickness)
        .translate((0.006, -0.0083, 0.0))
    )

    # Thin side and nose lips make the cover read as a channel wrapping around the drive.
    cover = cover.union(
        cq.Workplane("XY")
        .box(0.060, 0.0010, lip_height)
        .translate((0.031, 0.0138, -lip_height / 2.0))
    )
    cover = cover.union(
        cq.Workplane("XY")
        .box(0.060, 0.0010, lip_height)
        .translate((0.031, -0.0138, -lip_height / 2.0))
    )
    cover = cover.union(
        cq.Workplane("XY")
        .box(0.0010, 0.0270, lip_height)
        .translate((0.0648, 0.0, -lip_height / 2.0))
    )

    pin_clearance = cq.Workplane("XY").cylinder(thickness * 8.0, 0.00325)
    return cover.cut(pin_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_drive_swivel_cover")

    black_plastic = model.material("black_plastic", color=(0.015, 0.016, 0.018, 1.0))
    dark_rubber = model.material("dark_rubber", color=(0.002, 0.002, 0.002, 1.0))
    connector_metal = model.material("connector_metal", color=(0.70, 0.72, 0.70, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shell(), "rounded_usb_body", tolerance=0.00025),
        material=black_plastic,
        name="body_shell",
    )

    # Type-A connector shell: four metal walls leave the front visibly hollow.
    body.visual(
        Box((0.019, 0.0120, 0.0006)),
        origin=Origin(xyz=(0.0490, 0.0, 0.0020)),
        material=connector_metal,
        name="connector_top",
    )
    body.visual(
        Box((0.019, 0.0120, 0.0006)),
        origin=Origin(xyz=(0.0490, 0.0, -0.0020)),
        material=connector_metal,
        name="connector_bottom",
    )
    body.visual(
        Box((0.019, 0.0006, 0.0046)),
        origin=Origin(xyz=(0.0490, 0.0057, 0.0)),
        material=connector_metal,
        name="connector_side_0",
    )
    body.visual(
        Box((0.019, 0.0006, 0.0046)),
        origin=Origin(xyz=(0.0490, -0.0057, 0.0)),
        material=connector_metal,
        name="connector_side_1",
    )
    body.visual(
        Box((0.018, 0.0066, 0.0011)),
        origin=Origin(xyz=(0.0490, 0.0, -0.0007)),
        material=dark_rubber,
        name="contact_tongue",
    )

    # Raised molded grip ribs on the compact body.
    for i, y in enumerate((-0.0042, 0.0, 0.0042)):
        body.visual(
            Box((0.023, 0.00075, 0.00045)),
            origin=Origin(xyz=(0.019, y, 0.00375)),
            material=dark_rubber,
            name=f"grip_rib_{i}",
        )

    # The visible side pivot is fixed to the body; the cover rotates around it.
    body.visual(
        Cylinder(radius=0.0026, length=0.0120),
        origin=Origin(xyz=(0.0, 0.0, 0.0004)),
        material=connector_metal,
        name="pivot_pin",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.0006),
        origin=Origin(xyz=(0.0, 0.0, 0.00610)),
        material=connector_metal,
        name="pivot_head",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_swivel_cover_shell(), "swivel_cover_shell", tolerance=0.00020),
        origin=Origin(xyz=(0.0, 0.0, 0.0052)),
        material=brushed_steel,
        name="cover_frame",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    joint = object_model.get_articulation("body_to_cover")

    ctx.check(
        "cover joint is continuous",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={joint.articulation_type}",
    )

    body_aabb = ctx.part_world_aabb(body)
    cover_aabb = ctx.part_world_aabb(cover)
    if body_aabb is not None and cover_aabb is not None:
        body_len = body_aabb[1][0] - body_aabb[0][0]
        cover_len = cover_aabb[1][0] - cover_aabb[0][0]
        cover_width = cover_aabb[1][1] - cover_aabb[0][1]
        ctx.check(
            "cover is long and narrow",
            cover_len > body_len and cover_len > 2.2 * cover_width,
            details=f"cover_len={cover_len:.4f}, body_len={body_len:.4f}, cover_width={cover_width:.4f}",
        )

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        min_overlap=0.012,
        name="closed cover wraps over body footprint",
    )

    with ctx.pose({joint: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(cover)

    if cover_aabb is not None and turned_aabb is not None:
        rest_center = (
            (cover_aabb[0][0] + cover_aabb[1][0]) / 2.0,
            (cover_aabb[0][1] + cover_aabb[1][1]) / 2.0,
        )
        turned_center = (
            (turned_aabb[0][0] + turned_aabb[1][0]) / 2.0,
            (turned_aabb[0][1] + turned_aabb[1][1]) / 2.0,
        )
        ctx.check(
            "cover swings around side pivot",
            rest_center[0] > 0.025 and turned_center[1] > 0.025,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )

    return ctx.report()


object_model = build_object_model()
