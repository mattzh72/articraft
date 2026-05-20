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


BODY_LEN = 0.108
BODY_DEPTH = 0.046
BODY_HEIGHT = 0.066
LENS_Z = 0.004

GRIP_LEN = 0.088
GRIP_DEPTH = 0.020
GRIP_HEIGHT = 0.052
GRIP_CENTER = (0.004, -0.028, -0.003)

BATTERY_LEN = 0.028
BATTERY_DEPTH = 0.036
BATTERY_HEIGHT = 0.056
BATTERY_CENTER = (-0.058, 0.0, 0.002)

TOP_RIDGE_LEN = 0.056
TOP_RIDGE_DEPTH = 0.028
TOP_RIDGE_HEIGHT = 0.014
TOP_RIDGE_CENTER = (-0.008, 0.0, 0.034)

SCREEN_WIDTH = 0.070
SCREEN_THICKNESS = 0.008
SCREEN_HEIGHT = 0.046
SCREEN_HINGE_X = -0.034
SCREEN_HINGE_Y = BODY_DEPTH * 0.5
SCREEN_HINGE_Z = 0.004

RING_OUTER_RADIUS = 0.0245
RING_INNER_RADIUS = 0.0186
RING_LENGTH = 0.012
RING_CENTER_X = 0.068

HATCH_WIDTH = 0.026
HATCH_THICKNESS = 0.003
HATCH_HEIGHT = 0.020
HATCH_HINGE_X = 0.016
HATCH_HINGE_Y = GRIP_CENTER[1] - (GRIP_DEPTH * 0.5)
HATCH_HINGE_Z = -0.022


def _rounded_box(size: tuple[float, float, float], radius: float, center: tuple[float, float, float]):
    solid = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid.translate(center)


def _rounded_xz_panel(
    width: float,
    thickness: float,
    height: float,
    radius: float,
    center: tuple[float, float, float],
):
    solid = cq.Workplane("XY").box(width, thickness, height)
    if radius > 0.0:
        solid = solid.edges("|Y").fillet(radius)
    return solid.translate(center)


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]):
    x, y, z = center
    return cq.Workplane("XY").circle(radius).extrude(length).translate((x, y, z - length * 0.5))


def _x_annulus(outer_radius: float, inner_radius: float, length: float, center_x: float = 0.0):
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((center_x - length * 0.5, 0.0, 0.0))
    )


def _body_shell_mesh():
    shell = _rounded_box((BODY_LEN, BODY_DEPTH, BODY_HEIGHT), 0.010, (0.0, 0.0, 0.0))
    grip = _rounded_box((GRIP_LEN, GRIP_DEPTH, GRIP_HEIGHT), 0.007, GRIP_CENTER)
    battery = _rounded_box((BATTERY_LEN, BATTERY_DEPTH, BATTERY_HEIGHT), 0.006, BATTERY_CENTER)
    ridge = _rounded_box((TOP_RIDGE_LEN, TOP_RIDGE_DEPTH, TOP_RIDGE_HEIGHT), 0.004, TOP_RIDGE_CENTER)
    shell = shell.union(grip).union(battery).union(ridge)
    return mesh_from_cadquery(shell, "camcorder_body_shell")


def _screen_frame_mesh():
    frame = _rounded_xz_panel(
        SCREEN_WIDTH,
        SCREEN_THICKNESS,
        SCREEN_HEIGHT,
        0.0045,
        (SCREEN_WIDTH * 0.5, SCREEN_THICKNESS * 0.5, 0.0),
    )
    pocket = cq.Workplane("XY").box(
        SCREEN_WIDTH * 0.78,
        SCREEN_THICKNESS * 0.26,
        SCREEN_HEIGHT * 0.72,
    ).translate((SCREEN_WIDTH * 0.54, SCREEN_THICKNESS * 0.13, 0.0))
    hinge_spine = _z_cylinder(0.0028, SCREEN_HEIGHT * 0.92, (0.0018, 0.0030, 0.0))
    frame = frame.cut(pocket).union(hinge_spine)
    return mesh_from_cadquery(frame, "camcorder_screen_frame")


def _focus_ring_mesh():
    ring = _x_annulus(RING_OUTER_RADIUS, RING_INNER_RADIUS, RING_LENGTH)
    rib = _x_annulus(RING_OUTER_RADIUS + 0.0008, RING_INNER_RADIUS, 0.0012)
    ring = ring.union(rib.translate((-0.0032, 0.0, 0.0)))
    ring = ring.union(rib)
    ring = ring.union(rib.translate((0.0032, 0.0, 0.0)))
    return mesh_from_cadquery(ring, "camcorder_focus_ring")


def _hatch_mesh():
    panel = _rounded_xz_panel(
        HATCH_WIDTH,
        HATCH_THICKNESS,
        HATCH_HEIGHT,
        0.0024,
        (0.0, -HATCH_THICKNESS * 0.5, HATCH_HEIGHT * 0.5),
    )
    hinge = (
        cq.Workplane("YZ")
        .circle(0.0015)
        .extrude(HATCH_WIDTH * 0.92)
        .translate((-HATCH_WIDTH * 0.46, -HATCH_THICKNESS * 0.5, 0.0015))
    )
    return mesh_from_cadquery(panel.union(hinge), "camcorder_card_hatch")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_camcorder")

    body_black = model.material("body_black", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    ring_rubber = model.material("ring_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = model.material("glass", rgba=(0.20, 0.30, 0.36, 0.42))
    display_glass = model.material("display_glass", rgba=(0.16, 0.26, 0.31, 0.55))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_black, name="shell")
    body.visual(
        Cylinder(radius=0.017, length=0.039),
        origin=Origin(xyz=(0.0685, 0.0, LENS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="lens_core",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.014),
        origin=Origin(xyz=(0.055, 0.0, LENS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_black,
        name="rear_shroud",
    )
    body.visual(
        Cylinder(radius=0.0215, length=0.012),
        origin=Origin(xyz=(0.082, 0.0, LENS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_black,
        name="front_hood",
    )
    body.visual(
        Cylinder(radius=0.0155, length=0.0012),
        origin=Origin(xyz=(0.0886, 0.0, LENS_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_glass",
    )
    body.visual(
        Box((0.020, 0.004, 0.012)),
        origin=Origin(xyz=(0.012, -0.039, 0.000)),
        material=trim_gray,
        name="grip_strap_pad",
    )

    screen = model.part("screen")
    screen.visual(_screen_frame_mesh(), material=body_black, name="screen_frame")
    screen.visual(
        Box((SCREEN_WIDTH * 0.70, 0.0012, SCREEN_HEIGHT * 0.62)),
        origin=Origin(xyz=(SCREEN_WIDTH * 0.56, 0.00148, 0.0)),
        material=display_glass,
        name="display",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(_focus_ring_mesh(), material=ring_rubber, name="ring_shell")

    card_hatch = model.part("card_hatch")
    card_hatch.visual(_hatch_mesh(), material=trim_gray, name="hatch_panel")

    model.articulation(
        "screen_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(SCREEN_HINGE_X, SCREEN_HINGE_Y, SCREEN_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=2.5,
            lower=0.0,
            upper=2.1,
        ),
    )
    model.articulation(
        "focus_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(RING_CENTER_X, 0.0, LENS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=12.0,
        ),
    )
    model.articulation(
        "card_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=card_hatch,
        origin=Origin(xyz=(HATCH_HINGE_X, HATCH_HINGE_Y, HATCH_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    screen = object_model.get_part("screen")
    focus_ring = object_model.get_part("focus_ring")
    card_hatch = object_model.get_part("card_hatch")

    screen_hinge = object_model.get_articulation("screen_hinge")
    card_hatch_hinge = object_model.get_articulation("card_hatch_hinge")

    ctx.expect_gap(
        screen,
        body,
        axis="y",
        max_gap=0.003,
        max_penetration=0.0,
        name="screen parks close to the left body flank",
    )
    ctx.expect_overlap(
        screen,
        body,
        axes="xz",
        min_overlap=0.040,
        name="screen covers the body side when closed",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="yz",
        elem_a="ring_shell",
        elem_b="lens_core",
        min_overlap=0.032,
        name="focus ring stays concentric around the lens core",
    )
    ctx.expect_gap(
        body,
        card_hatch,
        axis="y",
        positive_elem="shell",
        negative_elem="hatch_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="media hatch sits flush against the handgrip side",
    )
    ctx.expect_overlap(
        card_hatch,
        body,
        axes="xz",
        min_overlap=0.016,
        name="media hatch stays within the handgrip panel area",
    )

    closed_screen_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({screen_hinge: 1.35}):
        open_screen_aabb = ctx.part_world_aabb(screen)
    ctx.check(
        "screen swings outward from the body",
        closed_screen_aabb is not None
        and open_screen_aabb is not None
        and open_screen_aabb[1][1] > closed_screen_aabb[1][1] + 0.030,
        details=f"closed={closed_screen_aabb}, open={open_screen_aabb}",
    )

    closed_hatch_aabb = ctx.part_world_aabb(card_hatch)
    with ctx.pose({card_hatch_hinge: 1.20}):
        open_hatch_aabb = ctx.part_world_aabb(card_hatch)
    ctx.check(
        "media hatch folds downward and outward",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[0][1] < closed_hatch_aabb[0][1] - 0.010
        and open_hatch_aabb[1][2] < closed_hatch_aabb[1][2] - 0.006,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
