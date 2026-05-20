from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _rectangular_sleeve(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    rear_wall: float,
    center_x: float,
) -> cq.Workplane:
    """Closed rectangular tube with a thin rear stop and open front mouth."""
    outer = cq.Workplane("XY").box(length, width, height).translate((center_x, 0.0, 0.0))
    cavity_length = length - rear_wall + 0.006
    cavity_center = center_x - length / 2.0 + rear_wall + cavity_length / 2.0
    cavity = (
        cq.Workplane("XY")
        .box(cavity_length, width - 2.0 * wall, height - 2.0 * wall)
        .translate((cavity_center, 0.0, 0.0))
    )
    return outer.cut(cavity).edges("|X").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    satin_steel = model.material("satin_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.24, 0.25, 0.26, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.014, 0.013, 1.0))

    outer = model.part("outer_sleeve")
    outer_shell = _rectangular_sleeve(
        length=0.260,
        width=0.060,
        height=0.036,
        wall=0.004,
        rear_wall=0.018,
        center_x=0.130,
    )
    outer.visual(
        mesh_from_cadquery(outer_shell, "outer_sleeve_shell", tolerance=0.0007),
        material=satin_steel,
        name="outer_sleeve_shell",
    )
    outer.visual(
        Box((0.030, 0.088, 0.004)),
        origin=Origin(xyz=(0.055, 0.0, -0.020)),
        material=dark_steel,
        name="rear_mount_ear",
    )
    outer.visual(
        Box((0.038, 0.088, 0.004)),
        origin=Origin(xyz=(0.210, 0.0, -0.020)),
        material=dark_steel,
        name="front_mount_ear",
    )

    middle = model.part("middle_member")
    middle_shell = _rectangular_sleeve(
        length=0.280,
        width=0.045,
        height=0.024,
        wall=0.004,
        rear_wall=0.015,
        center_x=-0.040,
    )
    middle.visual(
        mesh_from_cadquery(middle_shell, "middle_member_shell", tolerance=0.0007),
        material=dark_steel,
        name="middle_member_shell",
    )
    middle.visual(
        Box((0.060, 0.0035, 0.010)),
        origin=Origin(xyz=(-0.150, 0.02425, 0.0)),
        material=black_plastic,
        name="middle_side_glide",
    )
    middle.visual(
        Box((0.060, 0.0035, 0.010)),
        origin=Origin(xyz=(-0.150, -0.02425, 0.0)),
        material=black_plastic,
        name="middle_side_glide_1",
    )
    middle.visual(
        Box((0.060, 0.020, 0.0020)),
        origin=Origin(xyz=(-0.150, 0.0, 0.0130)),
        material=black_plastic,
        name="middle_top_glide",
    )
    middle.visual(
        Box((0.060, 0.020, 0.0020)),
        origin=Origin(xyz=(-0.150, 0.0, -0.0130)),
        material=black_plastic,
        name="middle_bottom_glide",
    )
    middle.visual(
        Box((0.022, 0.049, 0.003)),
        origin=Origin(xyz=(0.088, 0.0, -0.0135)),
        material=black_plastic,
        name="middle_wiper",
    )

    inner = model.part("inner_member")
    inner.visual(
        Box((0.285, 0.032, 0.012)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.0)),
        material=satin_steel,
        name="inner_member_bar",
    )
    inner.visual(
        Box((0.100, 0.0025, 0.006)),
        origin=Origin(xyz=(-0.075, 0.01725, 0.0)),
        material=black_plastic,
        name="inner_side_glide",
    )
    inner.visual(
        Box((0.100, 0.0025, 0.006)),
        origin=Origin(xyz=(-0.075, -0.01725, 0.0)),
        material=black_plastic,
        name="inner_side_glide_1",
    )
    inner.visual(
        Box((0.100, 0.014, 0.0020)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0070)),
        material=black_plastic,
        name="inner_top_glide",
    )
    inner.visual(
        Box((0.100, 0.014, 0.0020)),
        origin=Origin(xyz=(-0.075, 0.0, -0.0070)),
        material=black_plastic,
        name="inner_bottom_glide",
    )
    inner.visual(
        Box((0.012, 0.055, 0.035)),
        origin=Origin(xyz=(0.131, 0.0, 0.0)),
        material=black_plastic,
        name="end_plate",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.260, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_sleeve")
    middle = object_model.get_part("middle_member")
    inner = object_model.get_part("inner_member")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.check(
        "middle stage has 120 mm travel",
        abs(outer_slide.motion_limits.upper - 0.120) < 1e-9,
        details=f"upper={outer_slide.motion_limits.upper}",
    )
    ctx.check(
        "inner stage has 120 mm travel",
        abs(inner_slide.motion_limits.upper - 0.120) < 1e-9,
        details=f"upper={inner_slide.motion_limits.upper}",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        inner_elem="middle_member_shell",
        outer_elem="outer_sleeve_shell",
        margin=0.0,
        name="middle member is centered inside outer sleeve",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        inner_elem="inner_member_bar",
        outer_elem="middle_member_shell",
        margin=0.0,
        name="inner member is centered inside middle member",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="middle_member_shell",
        elem_b="outer_sleeve_shell",
        min_overlap=0.12,
        name="collapsed middle remains deeply inserted",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_member_bar",
        elem_b="middle_member_shell",
        min_overlap=0.10,
        name="collapsed inner remains deeply inserted",
    )

    rest_end = ctx.part_element_world_aabb(inner, elem="end_plate")
    with ctx.pose({outer_slide: 0.120, inner_slide: 0.120}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            inner_elem="middle_member_shell",
            outer_elem="outer_sleeve_shell",
            margin=0.0,
            name="extended middle stays aligned in sleeve",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            inner_elem="inner_member_bar",
            outer_elem="middle_member_shell",
            margin=0.0,
            name="extended inner stays aligned in middle member",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_member_shell",
            elem_b="outer_sleeve_shell",
            min_overlap=0.055,
            name="extended middle retains insertion",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_member_bar",
            elem_b="middle_member_shell",
            min_overlap=0.035,
            name="extended inner retains insertion",
        )
        extended_end = ctx.part_element_world_aabb(inner, elem="end_plate")

    ctx.check(
        "end plate moves outward by both slide travels",
        rest_end is not None
        and extended_end is not None
        and extended_end[1][0] > rest_end[1][0] + 0.235,
        details=f"rest={rest_end}, extended={extended_end}",
    )

    return ctx.report()


object_model = build_object_model()
