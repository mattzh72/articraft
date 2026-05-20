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


def _rounded_usb_body() -> cq.Workplane:
    """One injection-molded half-shell silhouette with long draft-rounded edges."""
    return (
        cq.Workplane("XY")
        .box(0.046, 0.018, 0.008)
        .edges("|X")
        .fillet(0.0032)
        .edges(">X or <X")
        .fillet(0.0012)
        .translate((-0.019, 0.0, 0.0))
    )


def _tube_y(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Hollow cylindrical sleeve with its axis along local/world Y."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )


def _box_solid(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _swivel_cover_shell() -> cq.Workplane:
    """Single stamped cover: one hinge sleeve, web, U-channel hood, and front lip."""
    hinge = _tube_y(0.00235, 0.00145, 0.0112)
    web = _box_solid((0.0068, 0.0100, 0.0016), (0.0030, 0.0, -0.0019))
    top = _box_solid((0.0220, 0.0170, 0.0009), (0.0160, 0.0, -0.00295))
    side_a = _box_solid((0.0220, 0.0009, 0.0067), (0.0160, 0.00845, -0.00605))
    side_b = _box_solid((0.0220, 0.0009, 0.0067), (0.0160, -0.00845, -0.00605))
    front_lip = _box_solid((0.0009, 0.0170, 0.0067), (0.02655, 0.0, -0.00605))
    return (
        hinge.union(web)
        .union(top)
        .union(side_a)
        .union(side_b)
        .union(front_lip)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_usb_swivel_drive")

    black_pp = model.material("molded_black_pp", rgba=(0.015, 0.015, 0.018, 1.0))
    seam_black = model.material("shadow_seam", rgba=(0.0, 0.0, 0.0, 1.0))
    satin_metal = model.material("satin_stamped_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_tongue = model.material("usb_contact_tongue", rgba=(0.01, 0.01, 0.012, 1.0))
    contact_gold = model.material("contact_gold_flash", rgba=(0.95, 0.68, 0.20, 1.0))

    body = model.part(
        "body",
        meta={
            "manufacturing_note": (
                "Two-shot visible detail is avoided: the body is one injection molded "
                "plastic shell with molded snap windows, a simple insert-molded USB "
                "connector, and a side pin for the stamped swivel cover."
            )
        },
    )
    body.visual(
        mesh_from_cadquery(_rounded_usb_body(), "rounded_usb_body", tolerance=0.00035),
        material=black_pp,
        name="body_shell",
    )
    body.visual(
        Box((0.0046, 0.0140, 0.0060)),
        origin=Origin(xyz=(0.0059, 0.0, 0.0)),
        material=black_pp,
        name="molded_connector_collar",
    )

    # Mold split and snap-window cues are shallow molded features, not extra parts.
    body.visual(
        Box((0.038, 0.00035, 0.00045)),
        origin=Origin(xyz=(-0.019, 0.00895, 0.0002)),
        material=seam_black,
        name="side_seam_0",
    )
    body.visual(
        Box((0.038, 0.00035, 0.00045)),
        origin=Origin(xyz=(-0.019, -0.00895, 0.0002)),
        material=seam_black,
        name="side_seam_1",
    )
    for i, y in enumerate((-0.0060, 0.0060)):
        body.visual(
            Box((0.0045, 0.0020, 0.00035)),
            origin=Origin(xyz=(-0.026, y, 0.00415)),
            material=seam_black,
            name=f"snap_window_{i}",
        )

    # USB-A shell is a thin folded stamping represented by four walls.
    body.visual(
        Box((0.0170, 0.0126, 0.00050)),
        origin=Origin(xyz=(0.0165, 0.0, 0.00255)),
        material=satin_metal,
        name="connector_top_wall",
    )
    body.visual(
        Box((0.0170, 0.0126, 0.00050)),
        origin=Origin(xyz=(0.0165, 0.0, -0.00255)),
        material=satin_metal,
        name="connector_bottom_wall",
    )
    body.visual(
        Box((0.0170, 0.00055, 0.0049)),
        origin=Origin(xyz=(0.0165, 0.0063, 0.0)),
        material=satin_metal,
        name="connector_side_wall_0",
    )
    body.visual(
        Box((0.0170, 0.00055, 0.0049)),
        origin=Origin(xyz=(0.0165, -0.0063, 0.0)),
        material=satin_metal,
        name="connector_side_wall_1",
    )
    body.visual(
        Box((0.0185, 0.0066, 0.0010)),
        origin=Origin(xyz=(0.0142, 0.0, -0.00055)),
        material=dark_tongue,
        name="connector_tongue",
    )
    for i, y in enumerate((-0.00225, 0.00225)):
        body.visual(
            Box((0.0045, 0.00125, 0.00018)),
            origin=Origin(xyz=(0.0132, y, 0.00286)),
            material=seam_black,
            name=f"shell_lock_hole_{i}",
        )
        body.visual(
            Box((0.0023, 0.0011, 0.00020)),
            origin=Origin(xyz=(0.0223, y, -0.00002)),
            material=contact_gold,
            name=f"contact_pad_{i}",
        )

    pivot_xyz = (0.0060, 0.0, 0.00685)
    for i, y in enumerate((-0.00855, 0.00855)):
        body.visual(
            Box((0.0052, 0.0042, 0.0017)),
            origin=Origin(xyz=(pivot_xyz[0], y, 0.00455)),
            material=black_pp,
            name=f"pivot_web_{i}",
        )
        body.visual(
            Cylinder(radius=0.00275, length=0.0041),
            origin=Origin(xyz=(pivot_xyz[0], y, pivot_xyz[2] - 0.00025), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black_pp,
            name=f"pivot_lug_{i}",
        )
    body.visual(
        Cylinder(radius=0.00110, length=0.0255),
        origin=Origin(xyz=pivot_xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="pivot_pin",
    )
    for i, y in enumerate((-0.0132, 0.0132)):
        body.visual(
            Cylinder(radius=0.00205, length=0.0009),
            origin=Origin(xyz=(pivot_xyz[0], y * 0.992, pivot_xyz[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name=f"pin_head_{i}",
        )

    cover = model.part(
        "swivel_cover",
        meta={
            "manufacturing_note": (
                "One stamped U-channel cover with an integral rolled sleeve; the "
                "only moving interface is the pinned sleeve."
            )
        },
    )
    cover.visual(
        mesh_from_cadquery(_swivel_cover_shell(), "swivel_cover_shell", tolerance=0.00025),
        material=satin_metal,
        name="cover_shell",
    )

    model.articulation(
        "body_to_swivel_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=pivot_xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=5.0, lower=0.0, upper=2.45),
        meta={
            "stop_note": (
                "Closed and service positions are set by molded stops in the body "
                "and by the same visible transverse pivot pin."
            )
        },
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("swivel_cover")
    hinge = object_model.get_articulation("body_to_swivel_cover")

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        min_overlap=0.010,
        elem_a="cover_shell",
        elem_b="connector_top_wall",
        name="closed cover projects over USB connector",
    )
    ctx.expect_within(
        body,
        cover,
        axes="y",
        margin=0.001,
        inner_elem="connector_top_wall",
        outer_elem="cover_shell",
        name="closed cover side skirts straddle connector width",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="y",
        min_overlap=0.009,
        elem_a="cover_shell",
        elem_b="pivot_pin",
        name="cover sleeve is coaxial with the visible pin",
    )

    closed_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({hinge: 2.10}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            min_gap=0.0002,
            positive_elem="cover_shell",
            negative_elem="body_shell",
            name="opened cover clears molded body shell",
        )
        open_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "swivel cover lifts away on the pin",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.010
        and open_aabb[0][0] < closed_aabb[0][0] - 0.004,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
