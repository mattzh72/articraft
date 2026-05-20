from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_THICKNESS = 0.032
JOINT_Z = 0.137


def _fixed_housing_mesh() -> cq.Workplane:
    """Low, broad fixed casting: mounting plate, bearing housing, and gussets."""

    plate = (
        cq.Workplane("XY")
        .box(0.46, 0.30, PLATE_THICKNESS)
        .translate((0.0, 0.0, PLATE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.018)
    )

    hole_points = [(-0.175, -0.105), (-0.175, 0.105), (0.175, -0.105), (0.175, 0.105)]
    bolt_holes = (
        cq.Workplane("XY")
        .pushPoints(hole_points)
        .circle(0.015)
        .extrude(PLATE_THICKNESS + 0.04)
        .translate((0.0, 0.0, -0.02))
    )
    fixed = plate.cut(bolt_holes)

    lower_housing = cq.Workplane("XY").circle(0.105).extrude(0.065).translate((0.0, 0.0, PLATE_THICKNESS))
    central_column = cq.Workplane("XY").circle(0.055).extrude(0.105).translate((0.0, 0.0, PLATE_THICKNESS))
    top_race = cq.Workplane("XY").circle(0.083).extrude(0.016).translate((0.0, 0.0, JOINT_Z - 0.016))
    fixed = fixed.union(lower_housing).union(central_column).union(top_race)

    rib = (
        cq.Workplane("XZ")
        .polyline([(0.050, PLATE_THICKNESS), (0.185, PLATE_THICKNESS), (0.050, 0.122)])
        .close()
        .extrude(0.024, both=True)
    )
    for angle in (0, 90, 180, 270):
        fixed = fixed.union(rib.rotate((0, 0, 0), (0, 0, 1), angle))

    return fixed


def _rotating_head_mesh() -> cq.Workplane:
    """Single rotating link: annular flange fused to a shallow cradle."""

    flange = cq.Workplane("XY").circle(0.095).extrude(0.026)
    bearing_clearance = cq.Workplane("XY").circle(0.058).extrude(0.052).translate((0.0, 0.0, -0.010))
    head = flange.cut(bearing_clearance)

    saddle = cq.Workplane("XY").box(0.185, 0.108, 0.016).translate((0.0, 0.0, 0.033))
    cheek_a = cq.Workplane("XY").box(0.170, 0.018, 0.046).translate((0.0, -0.054, 0.060))
    cheek_b = cq.Workplane("XY").box(0.170, 0.018, 0.046).translate((0.0, 0.054, 0.060))
    nose_radius = cq.Workplane("XY").circle(0.042).extrude(0.014).translate((0.0, 0.0, 0.038))

    return head.union(saddle).union(cheek_a).union(cheek_b).union(nose_radius)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_rotary_module")

    cast_iron = model.material("charcoal_cast_iron", rgba=(0.11, 0.12, 0.13, 1.0))
    anodized_head = model.material("orange_anodized_aluminum", rgba=(0.92, 0.44, 0.10, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_fixed_housing_mesh(), "fixed_housing"),
        material=cast_iron,
        name="fixed_housing",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_rotating_head_mesh(), "rotating_head"),
        material=anodized_head,
        name="rotating_head",
    )

    model.articulation(
        "housing_to_head",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    head = object_model.get_part("head")
    turntable = object_model.get_articulation("housing_to_head")

    ctx.check(
        "one vertical revolute joint",
        len(object_model.articulations) == 1
        and turntable.articulation_type == ArticulationType.REVOLUTE
        and tuple(turntable.axis) == (0.0, 0.0, 1.0),
        details=f"joints={object_model.articulations}",
    )
    ctx.expect_gap(
        head,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating head sits just above fixed race",
    )
    ctx.expect_overlap(
        head,
        housing,
        axes="xy",
        min_overlap=0.10,
        name="rotating head is centered on the fixed column footprint",
    )

    rest_pos = ctx.part_world_position(head)
    with ctx.pose({turntable: pi / 2.0}):
        turned_pos = ctx.part_world_position(head)
        ctx.expect_gap(
            head,
            housing,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="rotated head keeps bearing clearance",
        )
    ctx.check(
        "revolute motion keeps head on vertical axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
