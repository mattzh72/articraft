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


AXIS_Z = 0.092
BEARING_HALF_LENGTH = 0.0575
JOINT_X = BEARING_HALF_LENGTH + 0.012


def _make_base_body() -> cq.Workplane:
    """Low cast cartridge body: a flat foot and one bored pillow-block stage."""
    base = (
        cq.Workplane("XY")
        .box(0.320, 0.160, 0.032)
        .translate((0.0, 0.0, 0.016))
    )

    pedestal = (
        cq.Workplane("XY")
        .box(0.125, 0.092, 0.060)
        .translate((0.0, 0.0, 0.032 + 0.030))
    )
    sleeve = (
        cq.Workplane("YZ")
        .center(0.0, AXIS_Z)
        .circle(0.044)
        .extrude(BEARING_HALF_LENGTH, both=True)
    )
    body = base.union(pedestal).union(sleeve)

    # The stationary bearing support is bored through so the roll-axis journal
    # reads as captured by the cartridge body rather than floating in front.
    bore = (
        cq.Workplane("YZ")
        .center(0.0, AXIS_Z)
        .circle(0.024)
        .extrude(BEARING_HALF_LENGTH + 0.010, both=True)
    )
    body = body.cut(bore)

    # Four plain counterbored mounting holes make the base read as grounded.
    mounting_points = [
        (-0.112, -0.054),
        (-0.112, 0.054),
        (0.112, -0.054),
        (0.112, 0.054),
    ]
    through_holes = (
        cq.Workplane("XY")
        .pushPoints(mounting_points)
        .circle(0.008)
        .extrude(0.070)
        .translate((0.0, 0.0, -0.018))
    )
    counterbores = (
        cq.Workplane("XY")
        .pushPoints(mounting_points)
        .circle(0.015)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.024))
    )
    body = body.cut(through_holes).cut(counterbores)

    # Low longitudinal ribs visually tie the bearing stage back into the foot.
    for y in (-0.035, 0.035):
        rib = (
            cq.Workplane("XY")
            .box(0.172, 0.014, 0.018)
            .translate((0.0, y, 0.032 + 0.009))
        )
        body = body.union(rib)

    return body


def _make_output_rotor() -> cq.Workplane:
    """One rotating spindle member: plain flange and a short output nose."""
    def x_cylinder(radius: float, length: float, x_center: float) -> cq.Workplane:
        return (
            cq.Workplane("YZ")
            .circle(radius)
            .extrude(length / 2.0, both=True)
            .translate((x_center, 0.0, 0.0))
        )

    flange = x_cylinder(0.043, 0.014, 0.008)
    hub = x_cylinder(0.026, 0.018, 0.017)
    nose = x_cylinder(0.018, 0.072, 0.047)
    rotor = flange.union(hub).union(nose)

    # A shallow milled flat on the nose gives the otherwise round spindle a
    # visible rotational cue while keeping the flange itself plain.
    flat_cutter = (
        cq.Workplane("XY")
        .box(0.054, 0.040, 0.014)
        .translate((0.048, 0.0, 0.018))
    )
    return rotor.cut(flat_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_roll_cartridge")
    cast = model.material("dark_cast_iron", rgba=(0.16, 0.18, 0.19, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.72, 0.66, 1.0))
    paint = model.material("black_index_paint", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_body(), "base_body", tolerance=0.0008),
        material=cast,
        name="base_body",
    )

    output_nose = model.part("output_nose")
    output_nose.visual(
        Cylinder(radius=0.024, length=0.056),
        origin=Origin(xyz=(-0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="journal",
    )
    output_nose.visual(
        mesh_from_cadquery(_make_output_rotor(), "output_rotor", tolerance=0.0006),
        material=steel,
        name="output_rotor",
    )
    # A very thin painted witness mark on the flange face makes the spin legible
    # without adding a second mechanical articulation.
    output_nose.visual(
        Box((0.0012, 0.006, 0.030)),
        origin=Origin(xyz=(0.015, 0.0, 0.027)),
        material=paint,
        name="index_mark",
    )

    model.articulation(
        "spindle_spin",
        ArticulationType.REVOLUTE,
        parent=base,
        child=output_nose,
        origin=Origin(xyz=(JOINT_X, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=8.0,
            velocity=12.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    output_nose = object_model.get_part("output_nose")
    spin = object_model.get_articulation("spindle_spin")

    ctx.allow_overlap(
        base,
        output_nose,
        elem_a="base_body",
        elem_b="journal",
        reason=(
            "The bearing journal is intentionally captured inside the bored "
            "spindle support; the stationary housing is represented as a "
            "single cartridge body."
        ),
    )

    ctx.check(
        "single revolute spindle joint",
        len(object_model.articulations) == 1
        and spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.check(
        "spindle axis follows the supported roll axis",
        tuple(round(v, 6) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_overlap(
        output_nose,
        base,
        axes="yz",
        min_overlap=0.030,
        elem_a="output_rotor",
        elem_b="base_body",
        name="rotor is centered in the bearing support",
    )
    ctx.expect_within(
        output_nose,
        base,
        axes="yz",
        inner_elem="journal",
        outer_elem="base_body",
        margin=0.0,
        name="journal stays within the bearing envelope",
    )
    ctx.expect_overlap(
        output_nose,
        base,
        axes="x",
        min_overlap=0.020,
        elem_a="journal",
        elem_b="base_body",
        name="journal is retained in the support",
    )

    mark_rest = ctx.part_element_world_aabb(output_nose, elem="index_mark")
    with ctx.pose({spin: math.pi / 2.0}):
        mark_quarter = ctx.part_element_world_aabb(output_nose, elem="index_mark")
    ctx.check(
        "flange witness mark revolves about roll axis",
        mark_rest is not None
        and mark_quarter is not None
        and mark_rest[0][2] > mark_quarter[0][2] + 0.010,
        details=f"rest={mark_rest}, quarter_turn={mark_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
