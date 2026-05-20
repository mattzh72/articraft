from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_three_stage_extension_slide")

    galvanized = model.material("galvanized_steel", color=(0.68, 0.70, 0.72, 1.0))
    dark = model.material("dark_inner_steel", color=(0.18, 0.20, 0.22, 1.0))
    wall_paint = model.material("painted_wall_back", color=(0.86, 0.84, 0.78, 1.0))
    screw = model.material("brushed_screw_heads", color=(0.46, 0.47, 0.48, 1.0))

    outer = model.part("outer_sleeve")

    # The root part is the fixed wall-backed stage: a wall plate, stand-off
    # brackets, and a folded C-channel outer sleeve.
    outer.visual(
        Box((0.040, 0.240, 0.300)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=wall_paint,
        name="wall_plate",
    )
    outer.visual(
        Box((0.030, 0.095, 0.085)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=galvanized,
        name="rear_socket",
    )
    outer.visual(
        Box((0.080, 0.026, 0.030)),
        origin=Origin(xyz=(0.040, -0.055, 0.075)),
        material=galvanized,
        name="upper_standoff",
    )
    outer.visual(
        Box((0.080, 0.026, 0.030)),
        origin=Origin(xyz=(0.040, -0.055, -0.075)),
        material=galvanized,
        name="lower_standoff",
    )

    outer.visual(
        Box((0.560, 0.006, 0.070)),
        origin=Origin(xyz=(0.280, -0.039, 0.0)),
        material=galvanized,
        name="outer_web",
    )
    outer.visual(
        Box((0.560, 0.084, 0.006)),
        origin=Origin(xyz=(0.280, 0.0, 0.032)),
        material=galvanized,
        name="outer_top_flange",
    )
    outer.visual(
        Box((0.560, 0.084, 0.006)),
        origin=Origin(xyz=(0.280, 0.0, -0.032)),
        material=galvanized,
        name="outer_bottom_flange",
    )
    outer.visual(
        Box((0.560, 0.006, 0.018)),
        origin=Origin(xyz=(0.280, 0.039, 0.023)),
        material=galvanized,
        name="outer_upper_lip",
    )
    outer.visual(
        Box((0.560, 0.006, 0.018)),
        origin=Origin(xyz=(0.280, 0.039, -0.023)),
        material=galvanized,
        name="outer_lower_lip",
    )
    for idx, z in enumerate((-0.105, 0.105)):
        outer.visual(
            Cylinder(radius=0.015, length=0.008),
            origin=Origin(xyz=(0.004, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw,
            name=f"wall_screw_{idx}",
        )

    middle = model.part("middle_section")
    middle.visual(
        Box((0.620, 0.044, 0.018)),
        origin=Origin(xyz=(0.270, 0.003, 0.0)),
        material=dark,
        name="middle_spine",
    )
    middle.visual(
        Box((0.620, 0.006, 0.052)),
        origin=Origin(xyz=(0.270, -0.022, 0.0)),
        material=dark,
        name="middle_web",
    )
    middle.visual(
        Box((0.620, 0.030, 0.006)),
        origin=Origin(xyz=(0.270, -0.005, 0.026)),
        material=dark,
        name="top_glide",
    )
    middle.visual(
        Box((0.620, 0.030, 0.006)),
        origin=Origin(xyz=(0.270, -0.005, -0.026)),
        material=dark,
        name="bottom_glide",
    )
    middle.visual(
        Box((0.018, 0.052, 0.048)),
        origin=Origin(xyz=(-0.040, 0.003, 0.0)),
        material=dark,
        name="rear_stop_tab",
    )

    inner = model.part("inner_section")
    inner.visual(
        Box((0.580, 0.016, 0.012)),
        origin=Origin(xyz=(0.300, 0.033, 0.0)),
        material=galvanized,
        name="inner_blade",
    )
    inner.visual(
        Box((0.030, 0.040, 0.040)),
        origin=Origin(xyz=(0.605, 0.035, 0.0)),
        material=galvanized,
        name="terminal_end",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.260),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer = object_model.get_part("outer_sleeve")
    middle = object_model.get_part("middle_section")
    inner = object_model.get_part("inner_section")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "slide joints share extension axis",
        tuple(outer_slide.axis) == (1.0, 0.0, 0.0) and tuple(inner_slide.axis) == (1.0, 0.0, 0.0),
        details=f"outer axis={outer_slide.axis}, inner axis={inner_slide.axis}",
    )

    ctx.expect_contact(
        middle,
        outer,
        elem_a="top_glide",
        elem_b="outer_top_flange",
        name="middle rides outer top flange",
    )
    ctx.expect_contact(
        middle,
        outer,
        elem_a="bottom_glide",
        elem_b="outer_bottom_flange",
        name="middle rides outer bottom flange",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="inner_blade",
        elem_b="middle_spine",
        name="inner blade is supported by middle spine",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="middle_spine",
        elem_b="outer_web",
        min_overlap=0.30,
        name="middle retained in outer sleeve at rest",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_blade",
        elem_b="middle_spine",
        min_overlap=0.25,
        name="inner retained in middle section at rest",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_inner = ctx.part_world_position(inner)
    with ctx.pose({outer_slide: 0.260, inner_slide: 0.220}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_spine",
            elem_b="outer_web",
            min_overlap=0.10,
            name="middle retained in outer sleeve when extended",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_blade",
            elem_b="middle_spine",
            min_overlap=0.10,
            name="inner retained in middle section when extended",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "middle section extends outward",
        rest_middle is not None
        and extended_middle is not None
        and extended_middle[0] > rest_middle[0] + 0.250,
        details=f"rest={rest_middle}, extended={extended_middle}",
    )
    ctx.check(
        "terminal inner section projects furthest",
        rest_inner is not None
        and extended_inner is not None
        and extended_inner[0] > rest_inner[0] + 0.470,
        details=f"rest={rest_inner}, extended={extended_inner}",
    )

    return ctx.report()


object_model = build_object_model()
