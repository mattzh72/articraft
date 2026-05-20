from __future__ import annotations

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
import cadquery as cq


GUIDE_X = 0.38


def _carriage_shape() -> cq.Workplane:
    """One connected carriage mesh with guide shoes tangent to the columns."""

    shoe_width = 0.10
    shoe_center = GUIDE_X - 0.035 - 0.5 * shoe_width

    left_shoe = cq.Workplane("XY").box(shoe_width, 0.16, 0.36).translate((-shoe_center, 0.0, 0.0))
    right_shoe = cq.Workplane("XY").box(shoe_width, 0.16, 0.36).translate((shoe_center, 0.0, 0.0))
    crosshead = cq.Workplane("XY").box(0.59, 0.12, 0.16).translate((0.0, 0.0, -0.02))
    hanging_web = cq.Workplane("XY").box(0.18, 0.10, 0.46).translate((0.0, 0.0, -0.30))
    lower_lug = cq.Workplane("XY").box(0.30, 0.12, 0.10).translate((0.0, 0.0, -0.57))

    return (
        left_shoe
        .union(right_shoe)
        .union(crosshead)
        .union(hanging_web)
        .union(lower_lug)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_bridge_lift_carriage")

    painted_steel = model.material("painted_steel", color=(0.18, 0.22, 0.25, 1.0))
    polished_rod = model.material("polished_rod", color=(0.74, 0.77, 0.76, 1.0))
    safety_yellow = model.material("safety_yellow", color=(1.0, 0.66, 0.04, 1.0))

    guide_frame = model.part("guide_frame")
    guide_frame.visual(
        Box((1.10, 0.42, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=painted_steel,
        name="lower_base",
    )
    guide_frame.visual(
        Box((1.05, 0.28, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 1.98)),
        material=painted_steel,
        name="upper_tie",
    )
    for index, x in enumerate((-GUIDE_X, GUIDE_X)):
        guide_frame.visual(
            Cylinder(radius=0.035, length=1.90),
            origin=Origin(xyz=(x, 0.0, 1.00)),
            material=polished_rod,
            name=f"guide_column_{index}",
        )
        guide_frame.visual(
            Cylinder(radius=0.066, length=0.065),
            origin=Origin(xyz=(x, 0.0, 0.13)),
            material=painted_steel,
            name=f"lower_column_socket_{index}",
        )
        guide_frame.visual(
            Cylinder(radius=0.066, length=0.065),
            origin=Origin(xyz=(x, 0.0, 1.89)),
            material=painted_steel,
            name=f"upper_column_socket_{index}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_bearing_frame", tolerance=0.0008),
        material=safety_yellow,
        name="bearing_frame",
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=0.25, lower=0.0, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("guide_frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("guide_to_carriage")

    limits = slide.motion_limits
    ctx.check(
        "one vertical prismatic carriage joint",
        len(object_model.articulations) == 1
        and slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, -1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper == 0.75,
        details=f"joint={slide.articulation_type}, axis={slide.axis}, limits={limits}",
    )

    ctx.expect_gap(
        frame,
        carriage,
        axis="z",
        positive_elem="upper_tie",
        negative_elem="bearing_frame",
        min_gap=0.10,
        max_gap=0.18,
        name="carriage hangs below upper tie",
    )
    for column in ("guide_column_0", "guide_column_1"):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="bearing_frame",
            elem_b=column,
            contact_tol=0.002,
            name=f"guide shoe bears on {column}",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="bearing_frame",
            elem_b=column,
            min_overlap=0.30,
            name=f"upper pose retains {column}",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.75}):
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="bearing_frame",
            negative_elem="lower_base",
            min_gap=0.10,
            name="lowered carriage stays above base",
        )
        for column in ("guide_column_0", "guide_column_1"):
            ctx.expect_overlap(
                carriage,
                frame,
                axes="z",
                elem_a="bearing_frame",
                elem_b=column,
                min_overlap=0.30,
                name=f"lower pose retains {column}",
            )
        lowered_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive travel lowers carriage",
        rest_pos is not None
        and lowered_pos is not None
        and rest_pos[2] - lowered_pos[2] > 0.70,
        details=f"rest={rest_pos}, lowered={lowered_pos}",
    )

    return ctx.report()


object_model = build_object_model()
