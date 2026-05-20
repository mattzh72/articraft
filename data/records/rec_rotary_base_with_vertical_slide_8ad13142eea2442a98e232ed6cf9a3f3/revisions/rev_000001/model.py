from __future__ import annotations

import cadquery as cq
from math import pi

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


def _sleeve_mesh() -> object:
    """Hollow sliding collar centered on its part frame."""
    return (
        cq.Workplane("XY")
        .circle(0.062)
        .circle(0.045)
        .extrude(0.160)
        .translate((0.0, 0.0, -0.080))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_lifting_pedestal")

    dark_cast = model.material("dark_cast_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    turntable_blue = model.material("machined_blue", rgba=(0.05, 0.18, 0.33, 1.0))
    steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.68, 1.0))
    black = model.material("blackened_fasteners", rgba=(0.01, 0.01, 0.012, 1.0))
    tooling = model.material("tooling_plate_steel", rgba=(0.48, 0.50, 0.52, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.270, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_cast,
        name="floor_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=dark_cast,
        name="bearing_puck",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.205, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=turntable_blue,
        name="turntable_disc",
    )
    turntable.visual(
        Cylinder(radius=0.145, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=steel,
        name="rotary_top_ring",
    )
    turntable.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=turntable_blue,
        name="column_socket",
    )
    turntable.visual(
        Cylinder(radius=0.028, length=0.750),
        origin=Origin(xyz=(0.0, 0.0, 0.421)),
        material=steel,
        name="guide_column",
    )
    turntable.visual(
        Cylinder(radius=0.034, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.808)),
        material=black,
        name="column_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_sleeve_mesh(), "hollow_carriage_sleeve", tolerance=0.0008),
        material=turntable_blue,
        name="hollow_sleeve",
    )
    carriage.visual(
        Box((0.180, 0.070, 0.055)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=turntable_blue,
        name="tool_arm",
    )
    for index, (x, y) in enumerate(((0.0365, 0.0), (-0.0365, 0.0), (0.0, 0.0365), (0.0, -0.0365))):
        carriage.visual(
            Cylinder(radius=0.0085, length=0.130),
            origin=Origin(xyz=(x, y, 0.0)),
            material=black,
            name=f"guide_roller_{index}",
        )
    carriage.visual(
        Box((0.025, 0.180, 0.130)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=tooling,
        name="tooling_plate",
    )
    for index, y in enumerate((-0.060, 0.060)):
        for z in (-0.040, 0.040):
            carriage.visual(
                Cylinder(radius=0.010, length=0.007),
                origin=Origin(xyz=(0.241, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=black,
                name=f"plate_bolt_{index}_{0 if z < 0 else 1}",
            )
    carriage.visual(
        Cylinder(radius=0.026, length=0.005),
        origin=Origin(xyz=(0.240, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="tool_socket",
    )

    model.articulation(
        "pedestal_to_turntable",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-pi, upper=pi),
    )
    model.articulation(
        "turntable_to_carriage",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.25, lower=0.0, upper=0.420),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable = object_model.get_part("turntable")
    carriage = object_model.get_part("carriage")
    rotary = object_model.get_articulation("pedestal_to_turntable")
    lift = object_model.get_articulation("turntable_to_carriage")

    ctx.expect_gap(
        turntable,
        pedestal,
        axis="z",
        positive_elem="turntable_disc",
        negative_elem="bearing_puck",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on bearing puck",
    )
    ctx.expect_origin_distance(
        carriage,
        turntable,
        axes="xy",
        max_dist=0.001,
        name="carriage lift axis is coaxial with guide column",
    )
    ctx.expect_overlap(
        carriage,
        turntable,
        axes="z",
        elem_a="hollow_sleeve",
        elem_b="guide_column",
        min_overlap=0.140,
        name="lower carriage remains engaged on guide column",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.420}):
        ctx.expect_overlap(
            carriage,
            turntable,
            axes="z",
            elem_a="hollow_sleeve",
            elem_b="guide_column",
            min_overlap=0.140,
            name="raised carriage remains engaged on guide column",
        )
        raised_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage raises along column",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.400,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    closed_plate = ctx.part_element_world_aabb(carriage, elem="tooling_plate")
    with ctx.pose({rotary: pi / 2.0}):
        turned_plate = ctx.part_element_world_aabb(carriage, elem="tooling_plate")
    if closed_plate is not None and turned_plate is not None:
        closed_center = (
            (closed_plate[0][0] + closed_plate[1][0]) / 2.0,
            (closed_plate[0][1] + closed_plate[1][1]) / 2.0,
        )
        turned_center = (
            (turned_plate[0][0] + turned_plate[1][0]) / 2.0,
            (turned_plate[0][1] + turned_plate[1][1]) / 2.0,
        )
        rotary_ok = closed_center[0] > 0.20 and abs(closed_center[1]) < 0.02 and turned_center[1] > 0.20
    else:
        rotary_ok = False
        closed_center = None
        turned_center = None
    ctx.check(
        "turntable carries tooling head around vertical axis",
        rotary_ok,
        details=f"closed_center={closed_center}, turned_center={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
