from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_tube_visual(
    part,
    *,
    visual_name: str,
    mesh_name: str,
    wall_names: tuple[str, str, str, str],
    x_min: float,
    x_max: float,
    width_y: float,
    height_z: float,
    wall: float,
    material,
):
    """Add a connected open rectangular tube from four overlapping wall boxes."""

    length = x_max - x_min
    center_x = (x_min + x_max) * 0.5
    top_z = height_z * 0.5 - wall * 0.5
    side_y = width_y * 0.5 - wall * 0.5

    part.visual(
        Box((length, width_y, wall)),
        origin=Origin(xyz=(center_x, 0.0, top_z)),
        material=material,
        name=wall_names[0],
    )
    part.visual(
        Box((length, width_y, wall)),
        origin=Origin(xyz=(center_x, 0.0, -top_z)),
        material=material,
        name=wall_names[1],
    )
    part.visual(
        Box((length, wall, height_z)),
        origin=Origin(xyz=(center_x, -side_y, 0.0)),
        material=material,
        name=wall_names[2],
    )
    part.visual(
        Box((length, wall, height_z)),
        origin=Origin(xyz=(center_x, side_y, 0.0)),
        material=material,
        name=wall_names[3],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_telescoping_boom")

    painted = model.material("painted_dark_blue", rgba=(0.08, 0.12, 0.16, 1.0))
    outer_mat = model.material("powder_coated_outer", rgba=(0.88, 0.57, 0.12, 1.0))
    mid_mat = model.material("anodized_middle", rgba=(0.70, 0.72, 0.74, 1.0))
    inner_mat = model.material("brushed_inner", rgba=(0.52, 0.55, 0.58, 1.0))
    tip_mat = model.material("dark_inner", rgba=(0.28, 0.30, 0.32, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    label_mat = model.material("white_labels", rgba=(0.92, 0.92, 0.86, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.90, 0.72, 0.08)),
        origin=Origin(xyz=(0.38, 0.0, 0.04)),
        material=painted,
        name="floor_plate",
    )
    support.visual(
        Box((0.16, 0.08, 0.95)),
        origin=Origin(xyz=(0.25, -0.23, 0.555)),
        material=painted,
        name="side_post_0",
    )
    support.visual(
        Box((0.16, 0.08, 0.95)),
        origin=Origin(xyz=(0.25, 0.23, 0.555)),
        material=painted,
        name="side_post_1",
    )
    support.visual(
        Box((0.70, 0.46, 0.05)),
        origin=Origin(xyz=(0.48, 0.0, 1.005)),
        material=painted,
        name="saddle_plate",
    )
    support.visual(
        Box((0.38, 0.08, 0.56)),
        origin=Origin(xyz=(0.45, -0.23, 0.37), rpy=(0.0, 0.55, 0.0)),
        material=painted,
        name="diagonal_brace_0",
    )
    support.visual(
        Box((0.38, 0.08, 0.56)),
        origin=Origin(xyz=(0.45, 0.23, 0.37), rpy=(0.0, 0.55, 0.0)),
        material=painted,
        name="diagonal_brace_1",
    )
    support.visual(
        Box((0.07, 0.58, 0.22)),
        origin=Origin(xyz=(0.114, 0.0, 1.14)),
        material=painted,
        name="rear_stop",
    )

    outer = model.part("outer_box")
    _add_tube_visual(
        outer,
        visual_name="outer_sleeve",
        mesh_name="outer_sleeve",
        wall_names=(
            "outer_sleeve_top_wall",
            "outer_sleeve_bottom_wall",
            "outer_sleeve_side_neg",
            "outer_sleeve_side_pos",
        ),
        x_min=0.0,
        x_max=1.20,
        width_y=0.34,
        height_z=0.26,
        wall=0.025,
        material=outer_mat,
    )
    _add_tube_visual(
        outer,
        visual_name="outer_mouth_collar",
        mesh_name="outer_mouth_collar",
        wall_names=(
            "outer_mouth_collar_top_wall",
            "outer_mouth_collar_bottom_wall",
            "outer_mouth_collar_side_neg",
            "outer_mouth_collar_side_pos",
        ),
        x_min=1.14,
        x_max=1.22,
        width_y=0.37,
        height_z=0.29,
        wall=0.055,
        material=painted,
    )
    outer.visual(
        Box((0.014, 0.006, 0.145)),
        origin=Origin(xyz=(0.26, -0.172, 0.0)),
        material=label_mat,
        name="scale_mark_0",
    )
    outer.visual(
        Box((0.014, 0.006, 0.115)),
        origin=Origin(xyz=(0.56, -0.172, 0.0)),
        material=label_mat,
        name="scale_mark_1",
    )

    middle = model.part("middle_box")
    _add_tube_visual(
        middle,
        visual_name="middle_section",
        mesh_name="middle_section",
        wall_names=(
            "middle_section_top_wall",
            "middle_section_bottom_wall",
            "middle_section_side_neg",
            "middle_section_side_pos",
        ),
        x_min=-0.75,
        x_max=0.45,
        width_y=0.250,
        height_z=0.170,
        wall=0.020,
        material=mid_mat,
    )
    middle.visual(
        Box((0.13, 0.075, 0.024)),
        origin=Origin(xyz=(-0.68, 0.0, 0.093)),
        material=rubber,
        name="middle_glide_pad",
    )
    _add_tube_visual(
        middle,
        visual_name="middle_mouth_collar",
        mesh_name="middle_mouth_collar",
        wall_names=(
            "middle_mouth_collar_top_wall",
            "middle_mouth_collar_bottom_wall",
            "middle_mouth_collar_side_neg",
            "middle_mouth_collar_side_pos",
        ),
        x_min=0.39,
        x_max=0.47,
        width_y=0.285,
        height_z=0.215,
        wall=0.045,
        material=tip_mat,
    )

    inner = model.part("inner_box")
    _add_tube_visual(
        inner,
        visual_name="inner_section",
        mesh_name="inner_section",
        wall_names=(
            "inner_section_top_wall",
            "inner_section_bottom_wall",
            "inner_section_side_neg",
            "inner_section_side_pos",
        ),
        x_min=-0.65,
        x_max=0.35,
        width_y=0.170,
        height_z=0.120,
        wall=0.016,
        material=inner_mat,
    )
    inner.visual(
        Box((0.12, 0.058, 0.012)),
        origin=Origin(xyz=(-0.58, 0.0, 0.059)),
        material=rubber,
        name="inner_glide_pad",
    )
    _add_tube_visual(
        inner,
        visual_name="inner_mouth_collar",
        mesh_name="inner_mouth_collar",
        wall_names=(
            "inner_mouth_collar_top_wall",
            "inner_mouth_collar_bottom_wall",
            "inner_mouth_collar_side_neg",
            "inner_mouth_collar_side_pos",
        ),
        x_min=0.30,
        x_max=0.37,
        width_y=0.215,
        height_z=0.160,
        wall=0.038,
        material=tip_mat,
    )

    tip = model.part("tip_box")
    _add_tube_visual(
        tip,
        visual_name="tip_section",
        mesh_name="tip_section",
        wall_names=(
            "tip_section_top_wall",
            "tip_section_bottom_wall",
            "tip_section_side_neg",
            "tip_section_side_pos",
        ),
        x_min=-0.55,
        x_max=0.30,
        width_y=0.115,
        height_z=0.070,
        wall=0.012,
        material=tip_mat,
    )
    tip.visual(
        Box((0.11, 0.044, 0.014)),
        origin=Origin(xyz=(-0.50, 0.0, 0.037)),
        material=rubber,
        name="tip_glide_pad",
    )
    tip.visual(
        Box((0.055, 0.145, 0.105)),
        origin=Origin(xyz=(0.327, 0.0, 0.0)),
        material=rubber,
        name="end_cap",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=support,
        child=outer,
        origin=Origin(xyz=(0.15, 0.0, 1.16)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(1.20, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.55, effort=180.0, velocity=0.35),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.45, effort=140.0, velocity=0.32),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=tip,
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.35, effort=100.0, velocity=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_box")
    middle = object_model.get_part("middle_box")
    inner = object_model.get_part("inner_box")
    tip = object_model.get_part("tip_box")
    outer_slide = object_model.get_articulation("outer_to_middle")
    middle_slide = object_model.get_articulation("middle_to_inner")
    inner_slide = object_model.get_articulation("inner_to_tip")

    ctx.expect_gap(
        outer,
        object_model.get_part("support"),
        axis="z",
        positive_elem="outer_sleeve_bottom_wall",
        negative_elem="saddle_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="outer sleeve rests on saddle",
    )

    def expect_nested(parent, child, parent_prefix: str, child_prefix: str, min_x: float, label: str) -> None:
        ctx.expect_gap(
            parent,
            child,
            axis="z",
            positive_elem=f"{parent_prefix}_top_wall",
            negative_elem=f"{child_prefix}_top_wall",
            min_gap=0.004,
            name=f"{label} top clearance",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="z",
            positive_elem=f"{child_prefix}_bottom_wall",
            negative_elem=f"{parent_prefix}_bottom_wall",
            min_gap=0.004,
            name=f"{label} bottom clearance",
        )
        ctx.expect_gap(
            parent,
            child,
            axis="y",
            positive_elem=f"{parent_prefix}_side_pos",
            negative_elem=f"{child_prefix}_side_pos",
            min_gap=0.006,
            name=f"{label} positive side clearance",
        )
        ctx.expect_gap(
            child,
            parent,
            axis="y",
            positive_elem=f"{child_prefix}_side_neg",
            negative_elem=f"{parent_prefix}_side_neg",
            min_gap=0.006,
            name=f"{label} negative side clearance",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="x",
            elem_a=f"{child_prefix}_top_wall",
            elem_b=f"{parent_prefix}_top_wall",
            min_overlap=min_x,
            name=f"{label} retained insertion",
        )

    expect_nested(outer, middle, "outer_sleeve", "middle_section", 0.55, "middle in outer at rest")
    expect_nested(middle, inner, "middle_section", "inner_section", 0.50, "inner in middle at rest")
    expect_nested(inner, tip, "inner_section", "tip_section", 0.40, "tip in inner at rest")
    ctx.expect_contact(
        middle,
        outer,
        elem_a="middle_glide_pad",
        elem_b="outer_sleeve_top_wall",
        name="middle glide bears on outer sleeve",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="inner_glide_pad",
        elem_b="middle_section_top_wall",
        name="inner glide bears on middle section",
    )
    ctx.expect_contact(
        tip,
        inner,
        elem_a="tip_glide_pad",
        elem_b="inner_section_top_wall",
        name="tip glide bears on inner section",
    )

    rest_tip = ctx.part_world_position(tip)
    with ctx.pose({outer_slide: 0.55, middle_slide: 0.45, inner_slide: 0.35}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_section_top_wall",
            elem_b="outer_sleeve_top_wall",
            min_overlap=0.18,
            name="middle section retained when extended",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_section_top_wall",
            elem_b="middle_section_top_wall",
            min_overlap=0.18,
            name="inner section retained when extended",
        )
        ctx.expect_overlap(
            tip,
            inner,
            axes="x",
            elem_a="tip_section_top_wall",
            elem_b="inner_section_top_wall",
            min_overlap=0.18,
            name="tip section retained when extended",
        )
        extended_tip = ctx.part_world_position(tip)

    ctx.check(
        "prismatic stages extend along boom axis",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[0] > rest_tip[0] + 1.20,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
