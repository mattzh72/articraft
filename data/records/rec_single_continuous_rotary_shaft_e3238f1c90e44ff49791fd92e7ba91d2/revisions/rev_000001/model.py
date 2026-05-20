from __future__ import annotations

from math import pi

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


BASE_LENGTH = 0.50
BASE_WIDTH = 0.18
BASE_HEIGHT = 0.032
CHEEK_SPACING = 0.32
CHEEK_THICKNESS = 0.036
CHEEK_WIDTH = 0.150
CHEEK_HEIGHT = 0.205
SHAFT_Z = 0.128
SHAFT_RADIUS = 0.032
SHAFT_LENGTH = 0.48
BORE_RADIUS = 0.036


def _fixed_support_shape() -> cq.Workplane:
    """One fixed casting: a base plate and two bored support cheeks."""

    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
    )

    support = base
    for x in (-CHEEK_SPACING / 2.0, CHEEK_SPACING / 2.0):
        cheek = (
            cq.Workplane("XY")
            .box(CHEEK_THICKNESS, CHEEK_WIDTH, CHEEK_HEIGHT)
            # Let the cheek sink very slightly into the base so the exported
            # fixed support is one fused casting rather than tangent shells.
            .translate((x, 0.0, BASE_HEIGHT + CHEEK_HEIGHT / 2.0 - 0.001))
        )
        bore = (
            cq.Workplane("YZ")
            .center(0.0, SHAFT_Z)
            .circle(BORE_RADIUS)
            .extrude(CHEEK_THICKNESS * 2.4, both=True)
            .translate((x, 0.0, 0.0))
        )
        support = support.union(cheek).cut(bore)

        bearing_land = (
            cq.Workplane("XY")
            .box(CHEEK_THICKNESS + 0.008, 0.040, 0.012)
            .translate((x, 0.0, SHAFT_Z - SHAFT_RADIUS - 0.006))
        )
        support = support.union(bearing_land)

    for x in (-0.205, 0.205):
        for y in (-0.055, 0.055):
            bolt_hole = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(0.010)
                .extrude(BASE_HEIGHT * 3.0, both=True)
                .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
            )
            support = support.cut(bolt_hole)

    return support


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cheek_supported_rotary_shaft")
    cast_finish = model.material("painted_cast_iron_blue", rgba=(0.10, 0.18, 0.30, 1.0))
    shaft_finish = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_fixed_support_shape(), "bored_support_cheeks", tolerance=0.0006),
        material=cast_finish,
        name="support_frame",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=shaft_finish,
        name="shaft_body",
    )

    model.articulation(
        "support_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    shaft = object_model.get_part("shaft")
    joint = object_model.get_articulation("support_to_shaft")

    ctx.check(
        "continuous shaft joint",
        joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS,
        "The output shaft should have one unbounded rotary degree of freedom.",
    )
    ctx.check(
        "joint axis follows shaft",
        joint is not None and tuple(float(v) for v in joint.axis) == (1.0, 0.0, 0.0),
        f"axis={None if joint is None else joint.axis!r}",
    )

    ctx.expect_within(
        shaft,
        support,
        axes="yz",
        inner_elem="shaft_body",
        outer_elem="support_frame",
        margin=0.002,
        name="shaft is centered in bored cheek openings",
    )
    ctx.expect_overlap(
        shaft,
        support,
        axes="x",
        elem_a="shaft_body",
        elem_b="support_frame",
        min_overlap=CHEEK_SPACING,
        name="shaft spans both support cheeks",
    )

    at_zero = ctx.part_world_position(shaft)
    with ctx.pose({joint: pi}):
        at_half_turn = ctx.part_world_position(shaft)
    ctx.check(
        "continuous spin does not translate shaft",
        at_zero is not None
        and at_half_turn is not None
        and max(abs(at_zero[i] - at_half_turn[i]) for i in range(3)) < 1e-6,
        f"rest={at_zero}, half_turn={at_half_turn}",
    )

    return ctx.report()


object_model = build_object_model()
