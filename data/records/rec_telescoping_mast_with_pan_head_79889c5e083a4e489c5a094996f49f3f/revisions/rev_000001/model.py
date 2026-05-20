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


def _annular_tube(outer_radius: float, inner_radius: float, z_min: float, z_max: float):
    """CadQuery annular tube whose local axis is the mast +Z centerline."""
    length = z_max - z_min
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(length)
    return outer.cut(bore).translate((0.0, 0.0, z_min))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_telescoping_mast_head")

    dark = model.material("black_powder_coat", rgba=(0.02, 0.023, 0.026, 1.0))
    rubber = model.material("black_polymer_bushing", rgba=(0.005, 0.005, 0.006, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.75, 0.76, 1.0))
    anodized = model.material("dark_anodized_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    safety = model.material("orange_index_mark", rgba=(1.0, 0.42, 0.06, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        Cylinder(radius=0.125, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark,
        name="ground_flange",
    )
    outer_tube.visual(
        mesh_from_cadquery(_annular_tube(0.060, 0.050, 0.030, 0.630), "outer_tube_wall"),
        material=dark,
        name="outer_tube_wall",
    )
    outer_tube.visual(
        mesh_from_cadquery(_annular_tube(0.070, 0.050, 0.585, 0.655), "outer_top_collar"),
        material=dark,
        name="outer_top_collar",
    )
    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        outer_tube.visual(
            Box((0.125, 0.012, 0.165)),
            origin=Origin(xyz=(0.063, 0.0, 0.112), rpy=(0.0, 0.0, yaw)),
            material=dark,
            name=f"base_gusset_{index}",
        )

    lower_tube = model.part("lower_tube")
    lower_tube.visual(
        mesh_from_cadquery(_annular_tube(0.042, 0.033, -0.320, 0.430), "lower_tube_wall"),
        material=aluminum,
        name="lower_tube_wall",
    )
    lower_tube.visual(
        mesh_from_cadquery(_annular_tube(0.050, 0.033, -0.315, -0.270), "lower_guide_band"),
        material=rubber,
        name="lower_guide_band",
    )
    lower_tube.visual(
        mesh_from_cadquery(_annular_tube(0.050, 0.033, 0.372, 0.430), "lower_top_collar"),
        material=rubber,
        name="lower_top_collar",
    )

    top_tube = model.part("top_tube")
    top_tube.visual(
        mesh_from_cadquery(_annular_tube(0.026, 0.018, -0.260, 0.360), "top_tube_wall"),
        material=aluminum,
        name="top_tube_wall",
    )
    top_tube.visual(
        mesh_from_cadquery(_annular_tube(0.033, 0.018, -0.255, -0.215), "top_guide_band"),
        material=rubber,
        name="top_guide_band",
    )
    top_tube.visual(
        mesh_from_cadquery(_annular_tube(0.034, 0.018, 0.312, 0.360), "top_platform_collar"),
        material=rubber,
        name="top_platform_collar",
    )

    platform = model.part("platform")
    platform.visual(
        Cylinder(radius=0.078, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=anodized,
        name="platform_disk",
    )
    platform.visual(
        Cylinder(radius=0.038, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=anodized,
        name="raised_hub",
    )
    platform.visual(
        Box((0.078, 0.024, 0.014)),
        origin=Origin(xyz=(0.092, 0.0, 0.030)),
        material=safety,
        name="index_tab",
    )
    for index, (x, y) in enumerate(((0.045, 0.045), (-0.045, 0.045), (-0.045, -0.045), (0.045, -0.045))):
        platform.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(x, y, 0.035)),
            material=rubber,
            name=f"bolt_head_{index}",
        )

    model.articulation(
        "outer_to_lower",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=lower_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.250),
    )
    model.articulation(
        "lower_to_top",
        ArticulationType.PRISMATIC,
        parent=lower_tube,
        child=top_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=0.0, upper=0.200),
    )
    model.articulation(
        "top_to_platform",
        ArticulationType.REVOLUTE,
        parent=top_tube,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_tube")
    lower = object_model.get_part("lower_tube")
    top = object_model.get_part("top_tube")
    platform = object_model.get_part("platform")
    lower_slide = object_model.get_articulation("outer_to_lower")
    top_slide = object_model.get_articulation("lower_to_top")
    pan = object_model.get_articulation("top_to_platform")

    ctx.allow_overlap(
        lower,
        outer,
        elem_a="lower_guide_band",
        elem_b="outer_tube_wall",
        reason="The lower lift tube guide bushing is intentionally captured in sliding contact with the outer sleeve bore.",
    )
    ctx.allow_overlap(
        top,
        lower,
        elem_a="top_guide_band",
        elem_b="lower_tube_wall",
        reason="The top lift tube guide bushing is intentionally captured in sliding contact with the lower lift tube bore.",
    )

    ctx.check(
        "two vertical prismatic mast stages",
        lower_slide.articulation_type == ArticulationType.PRISMATIC
        and top_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(lower_slide.axis) == (0.0, 0.0, 1.0)
        and tuple(top_slide.axis) == (0.0, 0.0, 1.0),
        details=f"lower={lower_slide.articulation_type}, axis={lower_slide.axis}; top={top_slide.articulation_type}, axis={top_slide.axis}",
    )
    ctx.check(
        "platform pans about mast centerline",
        pan.articulation_type == ArticulationType.REVOLUTE and tuple(pan.axis) == (0.0, 0.0, 1.0),
        details=f"type={pan.articulation_type}, axis={pan.axis}",
    )

    ctx.expect_contact(
        lower,
        outer,
        elem_a="lower_guide_band",
        elem_b="outer_tube_wall",
        contact_tol=0.001,
        name="lower guide is seated in outer sleeve",
    )
    ctx.expect_contact(
        top,
        lower,
        elem_a="top_guide_band",
        elem_b="lower_tube_wall",
        contact_tol=0.001,
        name="top guide is seated in lower sleeve",
    )
    ctx.expect_overlap(
        lower,
        outer,
        axes="z",
        elem_a="lower_tube_wall",
        elem_b="outer_tube_wall",
        min_overlap=0.25,
        name="lower stage retained in outer tube at rest",
    )
    ctx.expect_overlap(
        top,
        lower,
        axes="z",
        elem_a="top_tube_wall",
        elem_b="lower_tube_wall",
        min_overlap=0.18,
        name="top stage retained in lower tube at rest",
    )
    ctx.expect_gap(
        platform,
        top,
        axis="z",
        positive_elem="platform_disk",
        negative_elem="top_platform_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotary platform sits on top tube",
    )

    rest_top_position = ctx.part_world_position(top)
    with ctx.pose({lower_slide: 0.250, top_slide: 0.200}):
        ctx.expect_overlap(
            lower,
            outer,
            axes="z",
            elem_a="lower_tube_wall",
            elem_b="outer_tube_wall",
            min_overlap=0.06,
            name="lower stage retained at full travel",
        )
        ctx.expect_overlap(
            top,
            lower,
            axes="z",
            elem_a="top_tube_wall",
            elem_b="lower_tube_wall",
            min_overlap=0.05,
            name="top stage retained at full travel",
        )
        extended_top_position = ctx.part_world_position(top)

    ctx.check(
        "serial stages extend upward",
        rest_top_position is not None
        and extended_top_position is not None
        and extended_top_position[2] > rest_top_position[2] + 0.40,
        details=f"rest={rest_top_position}, extended={extended_top_position}",
    )

    return ctx.report()


object_model = build_object_model()
