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


WIDTH = 0.90
HEIGHT = 1.20
DEPTH = 0.055
STILE_WIDTH = 0.075
RAIL_HEIGHT = 0.080
INNER_HALF_WIDTH = WIDTH * 0.5 - STILE_WIDTH
SLAT_COUNT = 9
SLAT_SPACING = 0.110
SLAT_Z0 = -0.440
SLAT_LENGTH = 0.680
SLAT_HEIGHT = 0.088
SLAT_THICKNESS = 0.014
SLAT_REST_ANGLE = -0.48
PIN_RADIUS = 0.008
HOLE_RADIUS = PIN_RADIUS


def _slat_z(index: int) -> float:
    return SLAT_Z0 + index * SLAT_SPACING


def _make_frame_shape():
    """One continuous painted shutter frame with drilled pivot holes."""
    side_x = WIDTH * 0.5 - STILE_WIDTH * 0.5
    rail_z = HEIGHT * 0.5 - RAIL_HEIGHT * 0.5

    def box(size, xyz):
        return cq.Workplane("XY").box(*size).translate(xyz)

    body = box((STILE_WIDTH, DEPTH, HEIGHT), (-side_x, 0.0, 0.0))
    body = body.union(box((STILE_WIDTH, DEPTH, HEIGHT), (side_x, 0.0, 0.0)))
    body = body.union(box((WIDTH, DEPTH, RAIL_HEIGHT), (0.0, 0.0, rail_z)))
    body = body.union(box((WIDTH, DEPTH, RAIL_HEIGHT), (0.0, 0.0, -rail_z)))

    # A slight bevel keeps the outer rectangular stiles from reading as raw blocks.
    body = body.edges("|Z").fillet(0.006).edges("|X").fillet(0.004)

    for i in range(SLAT_COUNT):
        # Fixed YZ workplane avoids drift while cutting the repeated axle bores.
        cutter = (
            cq.Workplane("YZ")
            .center(0.0, _slat_z(i))
            .circle(HOLE_RADIUS)
            .extrude(WIDTH + 0.16, both=True)
        )
        body = body.cut(cutter)

    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_louvered_shutter")

    painted_frame = model.material("painted_frame", rgba=(0.82, 0.84, 0.80, 1.0))
    painted_slat = model.material("painted_slat", rgba=(0.88, 0.90, 0.86, 1.0))
    worn_edge = model.material("worn_edge", rgba=(0.70, 0.72, 0.68, 1.0))
    steel_pin = model.material("steel_pin", rgba=(0.55, 0.57, 0.58, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_shape(), "drilled_shutter_frame"),
        material=painted_frame,
        name="frame_body",
    )
    # Thin front lips on the side stiles make the two rails read as the bearing supports
    # for the pin stack, while remaining part of the fixed frame.
    lip_x = WIDTH * 0.5 - STILE_WIDTH + 0.005
    for sign, name in [(-1.0, "bearing_lip_0"), (1.0, "bearing_lip_1")]:
        frame.visual(
            Box((0.018, 0.012, HEIGHT - 2.0 * RAIL_HEIGHT)),
            origin=Origin(xyz=(sign * lip_x, -DEPTH * 0.5 - 0.006, 0.0)),
            material=worn_edge,
            name=name,
        )

    pin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    pin_length = 0.122
    pin_center = WIDTH * 0.5 - pin_length * 0.5 - 0.004

    for i in range(SLAT_COUNT):
        slat = model.part(f"slat_{i}")
        slat.visual(
            Box((SLAT_LENGTH, SLAT_THICKNESS, SLAT_HEIGHT)),
            origin=Origin(rpy=(SLAT_REST_ANGLE, 0.0, 0.0)),
            material=painted_slat,
            name="blade",
        )
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=pin_length),
            origin=Origin(xyz=(-pin_center, 0.0, 0.0), rpy=pin_origin.rpy),
            material=steel_pin,
            name="pin_0",
        )
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=pin_length),
            origin=Origin(xyz=(pin_center, 0.0, 0.0), rpy=pin_origin.rpy),
            material=steel_pin,
            name="pin_1",
        )
        model.articulation(
            f"slat_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, _slat_z(i))),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=1.8, lower=0.0, upper=1.20),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    slats = [object_model.get_part(f"slat_{i}") for i in range(SLAT_COUNT)]
    pivots = [object_model.get_articulation(f"slat_{i}_pivot") for i in range(SLAT_COUNT)]

    ctx.check("one pivot per slat", len(slats) == SLAT_COUNT and len(pivots) == SLAT_COUNT)

    for i, slat in enumerate(slats):
        ctx.allow_overlap(
            frame,
            slat,
            elem_a="frame_body",
            elem_b="pin_0",
            reason="The left pivot pin is intentionally captured through the side stile bearing bore.",
        )
        ctx.allow_overlap(
            frame,
            slat,
            elem_a="frame_body",
            elem_b="pin_1",
            reason="The right pivot pin is intentionally captured through the side stile bearing bore.",
        )
        ctx.expect_within(
            slat,
            frame,
            axes="xz",
            inner_elem="blade",
            margin=0.003,
            name=f"slat_{i} fits inside the rectangular frame outline",
        )
        ctx.expect_overlap(
            slat,
            frame,
            axes="x",
            min_overlap=0.030,
            elem_a="pin_0",
            elem_b="frame_body",
            name=f"slat_{i} left pin enters a side stile",
        )
        ctx.expect_overlap(
            slat,
            frame,
            axes="x",
            min_overlap=0.030,
            elem_a="pin_1",
            elem_b="frame_body",
            name=f"slat_{i} right pin enters a side stile",
        )

    for i in range(SLAT_COUNT - 1):
        ctx.expect_gap(
            slats[i + 1],
            slats[i],
            axis="z",
            min_gap=0.010,
            name=f"slat_{i}_to_slat_{i + 1} has louver spacing",
        )

    mid = slats[SLAT_COUNT // 2]
    pivot = pivots[SLAT_COUNT // 2]
    rest_aabb = ctx.part_element_world_aabb(mid, elem="blade")
    rest_depth = rest_aabb[1][1] - rest_aabb[0][1] if rest_aabb is not None else None
    with ctx.pose({pivot: 1.20}):
        open_aabb = ctx.part_element_world_aabb(mid, elem="blade")
        open_depth = open_aabb[1][1] - open_aabb[0][1] if open_aabb is not None else None
        ctx.expect_within(
            mid,
            frame,
            axes="xz",
            inner_elem="blade",
            margin=0.003,
            name="opened middle slat remains retained by the frame",
        )
    ctx.check(
        "middle slat rotates about its longitudinal pin axis",
        rest_depth is not None and open_depth is not None and open_depth > rest_depth + 0.010,
        details=f"rest_depth={rest_depth}, open_depth={open_depth}",
    )

    return ctx.report()


object_model = build_object_model()
