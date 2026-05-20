from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _add_square_tube(
    part,
    *,
    x_min: float,
    x_max: float,
    outside: float,
    wall: float,
    material,
    collar_material=None,
    collar_length: float = 0.0,
) -> None:
    """Four connected wall plates forming an open-ended square tube along +X."""

    length = x_max - x_min
    x_center = (x_min + x_max) * 0.5
    half = outside * 0.5
    wall_center = half - wall * 0.5
    side_height = outside - 2.0 * wall

    part.visual(
        Box((length, outside, wall)),
        origin=Origin(xyz=(x_center, 0.0, wall_center)),
        material=material,
        name="top_wall",
    )
    part.visual(
        Box((length, outside, wall)),
        origin=Origin(xyz=(x_center, 0.0, -wall_center)),
        material=material,
        name="bottom_wall",
    )
    part.visual(
        Box((length, wall, side_height)),
        origin=Origin(xyz=(x_center, wall_center, 0.0)),
        material=material,
        name="side_y_pos",
    )
    part.visual(
        Box((length, wall, side_height)),
        origin=Origin(xyz=(x_center, -wall_center, 0.0)),
        material=material,
        name="side_y_neg",
    )

    if collar_material is not None and collar_length > 0.0:
        collar_x = x_max - collar_length * 0.5
        collar_proud = wall * 0.65
        collar_thickness = wall * 0.9
        collar_outer = outside + 2.0 * collar_proud
        collar_offset = half + collar_thickness * 0.5
        part.visual(
            Box((collar_length, collar_outer, collar_thickness)),
            origin=Origin(xyz=(collar_x, 0.0, collar_offset)),
            material=collar_material,
            name="front_collar_top",
        )
        part.visual(
            Box((collar_length, collar_outer, collar_thickness)),
            origin=Origin(xyz=(collar_x, 0.0, -collar_offset)),
            material=collar_material,
            name="front_collar_bottom",
        )
        part.visual(
            Box((collar_length, collar_thickness, collar_outer)),
            origin=Origin(xyz=(collar_x, collar_offset, 0.0)),
            material=collar_material,
            name="front_collar_y_pos",
        )
        part.visual(
            Box((collar_length, collar_thickness, collar_outer)),
            origin=Origin(xyz=(collar_x, -collar_offset, 0.0)),
            material=collar_material,
            name="front_collar_y_neg",
        )


def _add_guide_pads(
    part,
    *,
    x_center: float,
    pad_length: float,
    child_outside: float,
    parent_inside_half: float,
    material,
) -> None:
    """Low-friction bearing shoes that bridge the clearance to the parent sleeve."""

    child_half = child_outside * 0.5
    radial_gap = parent_inside_half - child_half
    pad_face = child_outside * 0.34
    pad_center = child_half + radial_gap * 0.5

    part.visual(
        Box((pad_length, pad_face, radial_gap)),
        origin=Origin(xyz=(x_center, 0.0, pad_center)),
        material=material,
        name="guide_pad_top",
    )
    part.visual(
        Box((pad_length, pad_face, radial_gap)),
        origin=Origin(xyz=(x_center, 0.0, -pad_center)),
        material=material,
        name="guide_pad_bottom",
    )
    part.visual(
        Box((pad_length, radial_gap, pad_face)),
        origin=Origin(xyz=(x_center, pad_center, 0.0)),
        material=material,
        name="guide_pad_y_pos",
    )
    part.visual(
        Box((pad_length, radial_gap, pad_face)),
        origin=Origin(xyz=(x_center, -pad_center, 0.0)),
        material=material,
        name="guide_pad_y_neg",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_service_boom")

    bracket_paint = model.material("bracket_dark_blue", rgba=(0.03, 0.07, 0.12, 1.0))
    bracket_edge = model.material("worn_black_edge", rgba=(0.015, 0.015, 0.014, 1.0))
    outer_paint = model.material("safety_yellow", rgba=(0.95, 0.68, 0.07, 1.0))
    mid_paint = model.material("painted_steel_orange", rgba=(0.86, 0.38, 0.08, 1.0))
    inner_paint = model.material("galvanized_steel", rgba=(0.48, 0.52, 0.54, 1.0))
    tip_paint = model.material("dark_telescoping_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    collar_paint = model.material("black_wear_collars", rgba=(0.02, 0.02, 0.018, 1.0))

    root = model.part("root_bracket")
    root.visual(
        Box((0.70, 0.68, 0.07)),
        origin=Origin(xyz=(-0.22, 0.0, 0.035)),
        material=bracket_paint,
        name="ground_foot",
    )
    root.visual(
        Box((0.14, 0.18, 0.555)),
        origin=Origin(xyz=(-0.22, 0.0, 0.3475)),
        material=bracket_paint,
        name="vertical_post",
    )
    root.visual(
        Box((0.32, 0.50, 0.09)),
        origin=Origin(xyz=(-0.11, 0.0, 0.58)),
        material=bracket_paint,
        name="under_saddle_crossbar",
    )
    root.visual(
        Box((0.45, 0.05, 0.43)),
        origin=Origin(xyz=(0.125, 0.205, 0.84)),
        material=bracket_paint,
        name="cradle_y_pos",
    )
    root.visual(
        Box((0.45, 0.05, 0.43)),
        origin=Origin(xyz=(0.125, -0.205, 0.84)),
        material=bracket_paint,
        name="cradle_y_neg",
    )
    root.visual(
        Box((0.24, 0.08, 0.44)),
        origin=Origin(xyz=(-0.06, 0.0, 0.50), rpy=(0.0, -0.55, 0.0)),
        material=bracket_edge,
        name="rear_diagonal_web",
    )
    for ix, x in enumerate((-0.45, 0.02)):
        for iy, y in enumerate((-0.24, 0.24)):
            root.visual(
                Cylinder(radius=0.035, length=0.018),
                origin=Origin(xyz=(x, y, 0.079)),
                material=bracket_edge,
                name=f"anchor_bolt_{ix}_{iy}",
            )

    outer = model.part("outer_box")
    _add_square_tube(
        outer,
        x_min=0.0,
        x_max=1.25,
        outside=0.36,
        wall=0.035,
        material=outer_paint,
        collar_material=collar_paint,
        collar_length=0.095,
    )

    middle = model.part("middle_tube")
    _add_square_tube(
        middle,
        x_min=-0.78,
        x_max=0.45,
        outside=0.26,
        wall=0.028,
        material=mid_paint,
        collar_material=collar_paint,
        collar_length=0.075,
    )
    _add_guide_pads(
        middle,
        x_center=-0.60,
        pad_length=0.18,
        child_outside=0.26,
        parent_inside_half=0.145,
        material=collar_paint,
    )

    inner = model.part("inner_tube")
    _add_square_tube(
        inner,
        x_min=-0.55,
        x_max=0.38,
        outside=0.18,
        wall=0.022,
        material=inner_paint,
        collar_material=collar_paint,
        collar_length=0.060,
    )
    _add_guide_pads(
        inner,
        x_center=-0.38,
        pad_length=0.14,
        child_outside=0.18,
        parent_inside_half=0.102,
        material=collar_paint,
    )

    tip = model.part("tip_tube")
    _add_square_tube(
        tip,
        x_min=-0.38,
        x_max=0.32,
        outside=0.12,
        wall=0.018,
        material=tip_paint,
        collar_material=collar_paint,
        collar_length=0.050,
    )
    _add_guide_pads(
        tip,
        x_center=-0.26,
        pad_length=0.11,
        child_outside=0.12,
        parent_inside_half=0.068,
        material=collar_paint,
    )
    tip.visual(
        Box((0.08, 0.16, 0.16)),
        origin=Origin(xyz=(0.36, 0.0, 0.0)),
        material=collar_paint,
        name="front_tool_plate",
    )

    model.articulation(
        "bracket_to_outer",
        ArticulationType.FIXED,
        parent=root,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(1.25, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.20, lower=0.0, upper=0.50),
        motion_properties=MotionProperties(damping=35.0, friction=12.0),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.22, lower=0.0, upper=0.35),
        motion_properties=MotionProperties(damping=28.0, friction=9.0),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=tip,
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.25, lower=0.0, upper=0.24),
        motion_properties=MotionProperties(damping=20.0, friction=7.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_bracket")
    outer = object_model.get_part("outer_box")
    middle = object_model.get_part("middle_tube")
    inner = object_model.get_part("inner_tube")
    tip = object_model.get_part("tip_tube")
    outer_slide = object_model.get_articulation("outer_to_middle")
    middle_slide = object_model.get_articulation("middle_to_inner")
    inner_slide = object_model.get_articulation("inner_to_tip")

    ctx.expect_contact(
        root,
        outer,
        elem_a="cradle_y_pos",
        elem_b="side_y_pos",
        contact_tol=0.001,
        name="outer box is seated in the root cradle",
    )

    def check_square_clearance(parent, child, label: str) -> None:
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            positive_elem="side_y_pos",
            negative_elem="side_y_pos",
            min_gap=0.004,
            max_gap=0.025,
            name=f"{label} has clearance on +Y wall",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            positive_elem="side_y_neg",
            negative_elem="side_y_neg",
            min_gap=0.004,
            max_gap=0.025,
            name=f"{label} has clearance on -Y wall",
        )
        ctx.expect_gap(
            parent,
            child,
            axis="z",
            positive_elem="top_wall",
            negative_elem="top_wall",
            min_gap=0.004,
            max_gap=0.025,
            name=f"{label} has clearance under top wall",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="z",
            positive_elem="bottom_wall",
            negative_elem="bottom_wall",
            min_gap=0.004,
            max_gap=0.025,
            name=f"{label} has clearance over bottom wall",
        )

    check_square_clearance(outer, middle, "middle tube in outer box")
    check_square_clearance(middle, inner, "inner tube in middle tube")
    check_square_clearance(inner, tip, "tip tube in inner tube")

    ctx.expect_contact(
        middle,
        outer,
        elem_a="guide_pad_y_pos",
        elem_b="side_y_pos",
        contact_tol=0.001,
        name="middle tube guide shoe bears on outer box",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="guide_pad_y_pos",
        elem_b="side_y_pos",
        contact_tol=0.001,
        name="inner tube guide shoe bears on middle tube",
    )
    ctx.expect_contact(
        tip,
        inner,
        elem_a="guide_pad_y_pos",
        elem_b="side_y_pos",
        contact_tol=0.001,
        name="tip tube guide shoe bears on inner tube",
    )

    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="top_wall",
        elem_b="top_wall",
        min_overlap=0.70,
        name="middle tube has long retained insertion at rest",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="top_wall",
        elem_b="top_wall",
        min_overlap=0.50,
        name="inner tube has retained insertion at rest",
    )
    ctx.expect_overlap(
        tip,
        inner,
        axes="x",
        elem_a="top_wall",
        elem_b="top_wall",
        min_overlap=0.35,
        name="tip tube has retained insertion at rest",
    )

    mid_rest = ctx.part_world_position(middle)
    with ctx.pose({outer_slide: 0.50}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="top_wall",
            elem_b="top_wall",
            min_overlap=0.25,
            name="middle tube remains captured at full extension",
        )
        mid_extended = ctx.part_world_position(middle)
    ctx.check(
        "middle tube slides forward",
        mid_rest is not None and mid_extended is not None and mid_extended[0] > mid_rest[0] + 0.45,
        details=f"rest={mid_rest}, extended={mid_extended}",
    )

    inner_rest = ctx.part_world_position(inner)
    with ctx.pose({middle_slide: 0.35}):
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="top_wall",
            elem_b="top_wall",
            min_overlap=0.18,
            name="inner tube remains captured at full extension",
        )
        inner_extended = ctx.part_world_position(inner)
    ctx.check(
        "inner tube slides forward",
        inner_rest is not None
        and inner_extended is not None
        and inner_extended[0] > inner_rest[0] + 0.30,
        details=f"rest={inner_rest}, extended={inner_extended}",
    )

    tip_rest = ctx.part_world_position(tip)
    with ctx.pose({inner_slide: 0.24}):
        ctx.expect_overlap(
            tip,
            inner,
            axes="x",
            elem_a="top_wall",
            elem_b="top_wall",
            min_overlap=0.12,
            name="tip tube remains captured at full extension",
        )
        tip_extended = ctx.part_world_position(tip)
    ctx.check(
        "tip tube slides forward",
        tip_rest is not None and tip_extended is not None and tip_extended[0] > tip_rest[0] + 0.20,
        details=f"rest={tip_rest}, extended={tip_extended}",
    )

    return ctx.report()


object_model = build_object_model()
