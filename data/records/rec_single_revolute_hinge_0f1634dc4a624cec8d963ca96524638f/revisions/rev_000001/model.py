from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_LENGTH = 0.120
LEAF_WIDTH = 0.040
LEAF_THICKNESS = 0.0024
BARREL_RADIUS = 0.0060
PIN_RADIUS = 0.00318
BORE_RADIUS = 0.00318
BARREL_GAP = 0.0015
PLATE_CLEARANCE = 0.0012
TAB_OVERLAP = 0.0010


def _cylinder(radius: float, length: float, z_min: float, *, axis=(0.0, 0.0, 1.0)) -> cq.Workplane:
    solid = cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(0.0, 0.0, z_min),
        cq.Vector(*axis),
    )
    return cq.Workplane("XY").add(solid)


def _tube_segment(z_min: float, z_max: float) -> cq.Workplane:
    length = z_max - z_min
    outer = _cylinder(BARREL_RADIUS, length, z_min)
    inner = _cylinder(BORE_RADIUS, length + 0.002, z_min - 0.001)
    return outer.cut(inner)


def _cut_screw_holes(body: cq.Workplane, *, side: float) -> cq.Workplane:
    hole_x = side * (BARREL_RADIUS + PLATE_CLEARANCE + LEAF_WIDTH * 0.58)
    cutter_len = LEAF_THICKNESS * 4.0
    for z in (-0.039, 0.0, 0.039):
        cutter = cq.Solid.makeCylinder(
            0.0031,
            cutter_len,
            cq.Vector(hole_x, -cutter_len / 2.0, z),
            cq.Vector(0.0, 1.0, 0.0),
        )
        body = body.cut(cq.Workplane("XY").add(cutter))
    return body


def _knuckle_intervals(indices: tuple[int, ...]) -> list[tuple[float, float]]:
    knuckle_len = (HINGE_LENGTH - 6.0 * BARREL_GAP) / 5.0
    z = -HINGE_LENGTH / 2.0 + BARREL_GAP
    intervals: list[tuple[float, float]] = []
    for index in range(5):
        z_min = z + index * (knuckle_len + BARREL_GAP)
        z_max = z_min + knuckle_len
        if index in indices:
            intervals.append((z_min, z_max))
    return intervals


def _leaf_with_knuckles(*, side: float, intervals: list[tuple[float, float]]) -> cq.Workplane:
    plate_center_x = side * (BARREL_RADIUS + PLATE_CLEARANCE + LEAF_WIDTH / 2.0)
    leaf = (
        cq.Workplane("XY")
        .box(LEAF_WIDTH, LEAF_THICKNESS, HINGE_LENGTH, centered=(True, True, True))
        .translate((plate_center_x, 0.0, 0.0))
        .edges("|Y")
        .fillet(0.003)
    )
    leaf = _cut_screw_holes(leaf, side=side)

    tab_inner = BARREL_RADIUS - TAB_OVERLAP
    tab_outer = BARREL_RADIUS + PLATE_CLEARANCE + 0.0010
    tab_width = tab_outer - tab_inner
    tab_center_x = side * (tab_inner + tab_width / 2.0)
    for z_min, z_max in intervals:
        z_center = (z_min + z_max) / 2.0
        tab_len = max(0.001, (z_max - z_min) - 0.0010)
        tab = (
            cq.Workplane("XY")
            .box(tab_width, LEAF_THICKNESS, tab_len, centered=(True, True, True))
            .translate((tab_center_x, 0.0, z_center))
        )
        leaf = leaf.union(tab)
        leaf = leaf.union(_tube_segment(z_min, z_max))
    return leaf


def _pin_with_heads() -> cq.Workplane:
    shaft = _cylinder(PIN_RADIUS, HINGE_LENGTH + 0.006, -HINGE_LENGTH / 2.0 - 0.003)
    head_radius = 0.0048
    head_thickness = 0.0024
    top = _cylinder(head_radius, head_thickness, HINGE_LENGTH / 2.0 - 0.0020)
    bottom = _cylinder(head_radius, head_thickness, -HINGE_LENGTH / 2.0 - head_thickness + 0.0020)
    return shaft.union(top).union(bottom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barrel_and_leaf_hinge")

    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    moving_steel = model.material("moving_leaf_steel", rgba=(0.78, 0.79, 0.76, 1.0))
    pin_steel = model.material("pin_burnished_steel", rgba=(0.45, 0.46, 0.44, 1.0))

    grounded_leaf = model.part("grounded_leaf")
    grounded_leaf.visual(
        mesh_from_cadquery(
            _leaf_with_knuckles(side=-1.0, intervals=_knuckle_intervals((0, 2, 4))),
            "grounded_leaf_shell",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        material=satin_steel,
        name="grounded_leaf_shell",
    )
    grounded_leaf.visual(
        mesh_from_cadquery(
            _pin_with_heads(),
            "hinge_pin",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        material=pin_steel,
        name="pin",
    )
    grounded_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH + BARREL_RADIUS * 2.0, BARREL_RADIUS * 2.0, HINGE_LENGTH)),
        mass=0.16,
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(
            _leaf_with_knuckles(side=1.0, intervals=_knuckle_intervals((1, 3))),
            "moving_leaf_shell",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        material=moving_steel,
        name="moving_leaf_shell",
    )
    moving_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH + BARREL_RADIUS * 2.0, BARREL_RADIUS * 2.0, HINGE_LENGTH)),
        mass=0.10,
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=grounded_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=math.radians(115.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grounded_leaf = object_model.get_part("grounded_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.check("grounded leaf exists", grounded_leaf is not None, "Expected a grounded hinge leaf.")
    ctx.check("moving leaf exists", moving_leaf is not None, "Expected a moving hinge leaf.")
    ctx.check("single revolute hinge exists", hinge is not None, "Expected one hinge articulation.")
    if grounded_leaf is None or moving_leaf is None or hinge is None:
        return ctx.report()

    ctx.allow_overlap(
        grounded_leaf,
        moving_leaf,
        elem_a="pin",
        elem_b="moving_leaf_shell",
        reason="The central pin is intentionally captured inside the moving leaf's knuckle bores.",
    )
    ctx.expect_overlap(
        grounded_leaf,
        moving_leaf,
        axes="z",
        elem_a="pin",
        elem_b="moving_leaf_shell",
        min_overlap=0.080,
        name="central pin spans moving knuckles",
    )
    ctx.expect_contact(
        grounded_leaf,
        moving_leaf,
        elem_a="pin",
        elem_b="moving_leaf_shell",
        contact_tol=0.0002,
        name="moving knuckles ride on central pin",
    )

    rest_aabb = ctx.part_world_aabb(moving_leaf)
    with ctx.pose({hinge: math.radians(90.0)}):
        swung_aabb = ctx.part_world_aabb(moving_leaf)
        ctx.expect_overlap(
            grounded_leaf,
            moving_leaf,
            axes="z",
            elem_a="pin",
            elem_b="moving_leaf_shell",
            min_overlap=0.080,
            name="pin still spans knuckles while swung",
        )

    ctx.check(
        "moving leaf rotates about pin axis",
        rest_aabb is not None
        and swung_aabb is not None
        and float(swung_aabb[1][1]) > float(rest_aabb[1][1]) + 0.030,
        details=f"rest_aabb={rest_aabb}, swung_aabb={swung_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
