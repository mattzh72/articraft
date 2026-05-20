from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _guide_rail_shape() -> cq.Workplane:
    """A low, chamfered linear guide rail with flat bearing land."""
    rail = cq.Workplane("XY").box(0.90, 0.070, 0.040).translate((0.0, 0.0, 0.060))
    return rail.edges("|X").chamfer(0.004)


def _carriage_saddle_shape() -> cq.Workplane:
    """Compact moving saddle with an open underside channel around the rail."""
    body = cq.Workplane("XY").box(0.180, 0.200, 0.110).translate((0.0, 0.0, 0.020))
    rail_clearance = cq.Workplane("XY").box(0.230, 0.088, 0.058).translate((0.0, 0.0, -0.019))
    body = body.cut(rail_clearance)
    return body.edges("|Z").chamfer(0.004)


def _tooling_plate_shape() -> cq.Workplane:
    """Flat tooling surface with two long shallow fixture slots."""
    plate = cq.Workplane("XY").box(0.160, 0.160, 0.012).translate((0.0, 0.0, 0.081))
    for y in (-0.040, 0.040):
        slot = cq.Workplane("XY").box(0.128, 0.014, 0.018).translate((0.0, y, 0.081))
        plate = plate.cut(slot)
    return plate.edges("|Z").chamfer(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="base_plate_slide_table")

    cast_iron = Material("painted_cast_iron", rgba=(0.22, 0.25, 0.27, 1.0))
    ground_steel = Material("ground_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = Material("dark_oxide_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    satin_plate = Material("satin_tooling_plate", rgba=(0.72, 0.74, 0.72, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.10, 0.42, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_guide_rail_shape(), "guide_rail"),
        material=ground_steel,
        name="guide_rail",
    )
    base.visual(
        Box((0.050, 0.160, 0.060)),
        origin=Origin(xyz=(-0.485, 0.0, 0.070)),
        material=dark_steel,
        name="end_stop_0",
    )
    base.visual(
        Box((0.050, 0.160, 0.060)),
        origin=Origin(xyz=(0.485, 0.0, 0.070)),
        material=dark_steel,
        name="end_stop_1",
    )
    for i, x in enumerate((-0.34, -0.20, -0.06, 0.08, 0.22, 0.36)):
        base.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.082)),
            material=dark_steel,
            name=f"rail_screw_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_saddle_shape(), "carriage_saddle"),
        material=cast_iron,
        name="saddle",
    )
    carriage.visual(
        mesh_from_cadquery(_tooling_plate_shape(), "tooling_plate"),
        material=satin_plate,
        name="tooling_plate",
    )
    carriage.visual(
        Box((0.130, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.020, 0.005)),
        material=dark_steel,
        name="bearing_pad_0",
    )
    carriage.visual(
        Box((0.130, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, 0.005)),
        material=dark_steel,
        name="bearing_pad_1",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.30, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("base_to_carriage")

    ctx.check(
        "single prismatic carriage slide",
        len(object_model.articulations) == 1
        and object_model.articulations[0].articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            carriage,
            base,
            axis="z",
            positive_elem="bearing_pad_0",
            negative_elem="guide_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="carriage bearing pad sits on guide",
        )
        ctx.expect_within(
            carriage,
            base,
            axes="y",
            inner_elem="bearing_pad_0",
            outer_elem="guide_rail",
            margin=0.0,
            name="bearing pad is centered on guide width",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="bearing_pad_0",
            elem_b="guide_rail",
            min_overlap=0.12,
            name="compact carriage overlaps the guide at rest",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.30}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="bearing_pad_0",
            elem_b="guide_rail",
            min_overlap=0.12,
            name="carriage remains on guide at positive travel",
        )
        ctx.expect_gap(
            base,
            carriage,
            axis="x",
            positive_elem="end_stop_1",
            negative_elem="saddle",
            min_gap=0.035,
            name="positive travel stops before end block",
        )
        extended_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: -0.30}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="bearing_pad_0",
            elem_b="guide_rail",
            min_overlap=0.12,
            name="carriage remains on guide at negative travel",
        )
        ctx.expect_gap(
            carriage,
            base,
            axis="x",
            positive_elem="saddle",
            negative_elem="end_stop_0",
            min_gap=0.035,
            name="negative travel stops before end block",
        )

    ctx.check(
        "joint motion follows guide direction",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
