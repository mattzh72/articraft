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


BODY_WIDTH = 0.240
BODY_DEPTH = 0.160
BODY_HEIGHT = 0.030
BODY_CORNER_RADIUS = 0.022

HINGE_Y = BODY_DEPTH / 2.0 + 0.014
HINGE_Z = BODY_HEIGHT + 0.018
ANTENNA_XS = (-0.066, 0.066)
ANTENNA_WIDTH = 0.042
ANTENNA_LENGTH = 0.120
ANTENNA_THICKNESS = 0.006
ANTENNA_SWING = math.radians(100.0)


def _router_body_geometry() -> cq.Workplane:
    """Rounded slab with real top vent slots cut through the shell."""
    body = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(BODY_CORNER_RADIUS)
        .edges(">Z")
        .fillet(0.003)
        .edges("<Z")
        .fillet(0.0015)
    )

    slot_length = 0.040
    slot_width = 0.0048
    # Two vent fields leave an unperforated center spine, as on many flat routers.
    x_centers = [-0.078, -0.052, -0.026, 0.026, 0.052, 0.078]
    y_centers = [-0.036, -0.018, 0.000, 0.018, 0.036]
    for x in x_centers:
        for y in y_centers:
            cutter = (
                cq.Workplane("XY")
                .slot2D(slot_length, slot_width, angle=0.0)
                .extrude(BODY_HEIGHT + 0.012)
                .translate((x, y, -BODY_HEIGHT / 2.0 - 0.006))
            )
            body = body.cut(cutter)

    # A shallow bevel-like front inset gives the otherwise flat face scale and orientation.
    front_recess = (
        cq.Workplane("XY")
        .box(0.090, 0.003, 0.006)
        .translate((0.0, -BODY_DEPTH / 2.0 - 0.001, 0.002))
    )
    body = body.union(front_recess)
    return body


def _antenna_geometry() -> cq.Workplane:
    """One connected folding paddle: hinge knuckle, neck, and thin rounded panel."""
    root_radius = 0.006
    hinge_barrel = cq.Workplane("YZ").cylinder(ANTENNA_WIDTH * 0.82, root_radius)
    neck = (
        cq.Workplane("XY")
        .rect(ANTENNA_WIDTH * 0.48, root_radius * 1.7)
        .extrude(ANTENNA_THICKNESS)
        .translate((0.0, root_radius * 0.72, -ANTENNA_THICKNESS / 2.0))
    )
    panel = (
        cq.Workplane("XY")
        .rect(ANTENNA_WIDTH, ANTENNA_LENGTH)
        .extrude(ANTENNA_THICKNESS)
        .translate((0.0, root_radius + ANTENNA_LENGTH / 2.0, -ANTENNA_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.001)
    )
    return hinge_barrel.union(neck).union(panel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flat_home_router")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.075, 0.080, 0.086, 1.0))
    dark_grey = model.material("dark_grey_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    green_led = model.material("green_led_lenses", rgba=(0.10, 0.85, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_router_body_geometry(), "rounded_router_body", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=charcoal,
        name="vented_slab",
    )

    # Fixed yoke-style pivot bases on the back edge.  The two ears flank, but do
    # not intersect, each moving antenna barrel.
    for idx, x in enumerate(ANTENNA_XS):
        body.visual(
            Box((0.056, 0.026, 0.012)),
            origin=Origin(xyz=(x, BODY_DEPTH / 2.0 + 0.004, BODY_HEIGHT + 0.006)),
            material=dark_grey,
            name=f"pivot_base_{idx}",
        )
        for side, sx in enumerate((-1.0, 1.0)):
            body.visual(
                Box((0.006, 0.017, 0.030)),
                origin=Origin(
                    xyz=(
                        x + sx * 0.0225,
                        HINGE_Y,
                        BODY_HEIGHT + 0.022,
                    )
                ),
                material=dark_grey,
                name=f"pivot_ear_{idx}_{side}",
            )

    # Small front status windows make the slab read as a home network appliance.
    for idx, x in enumerate((-0.030, -0.015, 0.0, 0.015, 0.030)):
        body.visual(
            Box((0.006, 0.0025, 0.003)),
            origin=Origin(xyz=(x, -BODY_DEPTH / 2.0 - 0.0025, BODY_HEIGHT * 0.52)),
            material=green_led if idx in (1, 3) else matte_black,
            name=f"status_window_{idx}",
        )

    antenna_mesh = _antenna_geometry()
    for idx, x in enumerate(ANTENNA_XS):
        antenna = model.part(f"antenna_{idx}")
        antenna.visual(
            mesh_from_cadquery(antenna_mesh, f"paddle_antenna_{idx}", tolerance=0.0006),
            material=matte_black,
            name="paddle",
        )
        model.articulation(
            f"body_to_antenna_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.0,
                lower=0.0,
                upper=ANTENNA_SWING,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    ctx.check(
        "two paddle antennas",
        all(object_model.get_part(f"antenna_{idx}") is not None for idx in range(2)),
        "Both rear paddle antennas should be separate articulated parts.",
    )

    for idx in range(2):
        antenna = object_model.get_part(f"antenna_{idx}")
        hinge = object_model.get_articulation(f"body_to_antenna_{idx}")
        limits = hinge.motion_limits
        ctx.check(
            f"antenna_{idx} hinge range",
            limits is not None
            and abs((limits.upper or 0.0) - ANTENNA_SWING) < 1.0e-6
            and abs(limits.lower or 0.0) < 1.0e-6,
            "Each antenna should rotate from folded flat to about 100 degrees upright.",
        )

        with ctx.pose({hinge: 0.0}):
            ctx.expect_gap(
                antenna,
                body,
                axis="z",
                min_gap=0.010,
                negative_elem="vented_slab",
                name=f"antenna_{idx} folded above rear edge",
            )
            folded_aabb = ctx.part_world_aabb(antenna)

        with ctx.pose({hinge: ANTENNA_SWING}):
            raised_aabb = ctx.part_world_aabb(antenna)
            ctx.expect_gap(
                antenna,
                body,
                axis="z",
                min_gap=0.010,
                negative_elem="vented_slab",
                name=f"antenna_{idx} upright clears slab",
            )

        ctx.check(
            f"antenna_{idx} rises when deployed",
            folded_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > folded_aabb[1][2] + 0.080,
            details=f"folded_aabb={folded_aabb}, raised_aabb={raised_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
