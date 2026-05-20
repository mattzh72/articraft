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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_section_telescoping_slide")

    dark_zinc = model.material("dark_zinc", rgba=(0.18, 0.19, 0.19, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    cut_edge = model.material("dark_cut_edges", rgba=(0.07, 0.075, 0.075, 1.0))
    white_nylon = model.material("white_nylon", rgba=(0.86, 0.86, 0.80, 1.0))
    bolt_black = model.material("black_oxide_bolts", rgba=(0.02, 0.02, 0.018, 1.0))

    outer_length = 0.70
    outer_width = 0.080
    outer_height = 0.056
    wall = 0.006

    # The fixed section is a true rectangular box tube rather than a solid block.
    # It runs from x=-outer_length to x=0, with long top windows so the nested
    # runner and its retained insertion remain visible.
    outer_tube = cq.Workplane("XY").box(outer_length, outer_width, outer_height)
    inner_void = cq.Workplane("XY").box(
        outer_length + 0.030,
        outer_width - 2.0 * wall,
        outer_height - 2.0 * wall,
    )
    outer_tube = outer_tube.cut(inner_void)
    for slot_x, slot_len in ((-0.49, 0.18), (-0.22, 0.20)):
        top_slot = (
            cq.Workplane("XY")
            .box(slot_len, outer_width - 0.030, outer_height)
            .translate((slot_x, 0.0, outer_height * 0.43))
        )
        outer_tube = outer_tube.cut(top_slot)
    outer_tube = outer_tube.edges("|X").fillet(0.002)
    outer_tube = outer_tube.translate((-outer_length / 2.0, 0.0, outer_height / 2.0))

    outer_section = model.part("outer_section")
    outer_section.visual(
        mesh_from_cadquery(outer_tube, "outer_box_tube", tolerance=0.0007),
        material=dark_zinc,
        name="outer_sleeve",
    )
    # Low-friction liners fill the running clearance and make actual sliding
    # contact with the nested member instead of leaving the runner floating in a
    # purely visual cavity.
    for i, y in enumerate((-0.032, 0.032)):
        outer_section.visual(
            Box((0.64, 0.006, 0.020)),
            origin=Origin(xyz=(-0.34, y, outer_height / 2.0)),
            material=white_nylon,
            name=f"side_guide_{i}",
        )
    outer_section.visual(
        Box((0.64, 0.040, 0.005)),
        origin=Origin(xyz=(-0.34, 0.0, wall + 0.0025)),
        material=white_nylon,
        name="bottom_guide",
    )
    # A continuous mounting foot makes the root section read as grounded and
    # keeps all visible mounting hardware physically tied into the sleeve.
    outer_section.visual(
        Box((0.66, 0.115, 0.008)),
        origin=Origin(xyz=(-0.35, 0.0, -0.004)),
        material=dark_zinc,
        name="mounting_foot",
    )
    for i, x in enumerate((-0.60, -0.42, -0.24, -0.08)):
        outer_section.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(x, -0.043, 0.002)),
            material=bolt_black,
            name=f"bolt_{i}_a",
        )
        outer_section.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(x, 0.043, 0.002)),
            material=bolt_black,
            name=f"bolt_{i}_b",
        )

    inner_runner = model.part("inner_runner")
    # The child frame is at the front lip of the fixed sleeve.  The runner has a
    # long hidden tail (negative X) and a plain protruding face at positive X.
    inner_runner.visual(
        Box((0.620, 0.058, 0.034)),
        origin=Origin(xyz=(-0.130, 0.0, outer_height / 2.0)),
        material=brushed_steel,
        name="runner_body",
    )
    inner_runner.visual(
        Box((0.008, 0.064, 0.040)),
        origin=Origin(xyz=(0.184, 0.0, outer_height / 2.0)),
        material=cut_edge,
        name="front_face",
    )

    model.articulation(
        "outer_to_runner",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=inner_runner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.280, effort=120.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_section")
    runner = object_model.get_part("inner_runner")
    slide = object_model.get_articulation("outer_to_runner")

    ctx.expect_within(
        runner,
        outer,
        axes="yz",
        inner_elem="runner_body",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="runner fits inside box section cross section",
    )
    ctx.expect_overlap(
        runner,
        outer,
        axes="x",
        elem_a="runner_body",
        elem_b="outer_sleeve",
        min_overlap=0.40,
        name="runner has clear retained overlap at rest",
    )
    ctx.expect_gap(
        runner,
        outer,
        axis="x",
        positive_elem="front_face",
        negative_elem="outer_sleeve",
        min_gap=0.175,
        max_gap=0.195,
        name="plain front face protrudes from fixed section",
    )
    ctx.expect_contact(
        runner,
        outer,
        elem_a="runner_body",
        elem_b="bottom_guide",
        contact_tol=0.001,
        name="runner is supported by bottom guide",
    )

    rest_pos = ctx.part_world_position(runner)
    with ctx.pose({slide: 0.280}):
        ctx.expect_overlap(
            runner,
            outer,
            axes="x",
            elem_a="runner_body",
            elem_b="outer_sleeve",
            min_overlap=0.12,
            name="runner remains inserted at full extension",
        )
        extended_pos = ctx.part_world_position(runner)

    ctx.check(
        "prismatic joint extends along slide axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.25,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
