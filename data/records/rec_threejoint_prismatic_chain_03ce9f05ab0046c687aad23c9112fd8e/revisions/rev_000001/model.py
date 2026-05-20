from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rect_ring_mesh(
    name: str,
    *,
    outer_w: float,
    outer_d: float,
    hole_w: float,
    hole_d: float,
    height: float,
    radius: float,
    hole_radius: float | None = None,
):
    """Mesh for a rounded rectangular hollow extrusion centered on local Z."""

    outer = rounded_rect_profile(outer_w, outer_d, radius, corner_segments=6)
    hole = rounded_rect_profile(
        hole_w,
        hole_d,
        hole_radius if hole_radius is not None else max(radius * 0.55, 0.001),
        corner_segments=6,
    )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [hole], height, cap=True, center=True, closed=True),
        name,
    )


def _add_rect_collar(
    part,
    *,
    prefix: str,
    outer_w: float,
    outer_d: float,
    hole_w: float,
    hole_d: float,
    height: float,
    z: float,
    material: Material,
) -> None:
    """Add a four-piece rectangular collar whose aperture stays open for sliders."""

    side_x = (outer_w - hole_w) / 2.0
    face_y = (outer_d - hole_d) / 2.0
    part.visual(
        Box((outer_w, face_y, height)),
        origin=Origin(xyz=(0.0, hole_d / 2.0 + face_y / 2.0, z)),
        material=material,
        name=f"{prefix}_front",
    )
    part.visual(
        Box((outer_w, face_y, height)),
        origin=Origin(xyz=(0.0, -(hole_d / 2.0 + face_y / 2.0), z)),
        material=material,
        name=f"{prefix}_rear",
    )
    part.visual(
        Box((side_x, outer_d, height)),
        origin=Origin(xyz=(hole_w / 2.0 + side_x / 2.0, 0.0, z)),
        material=material,
        name=f"{prefix}_side_0",
    )
    part.visual(
        Box((side_x, outer_d, height)),
        origin=Origin(xyz=(-(hole_w / 2.0 + side_x / 2.0), 0.0, z)),
        material=material,
        name=f"{prefix}_side_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_antenna_mast")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.74, 1.0))
    pale_aluminum = model.material("pale_aluminum", rgba=(0.86, 0.88, 0.86, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.16, 0.18, 0.20, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.02, 0.02, 0.018, 1.0))
    safety_red = model.material("red_height_marks", rgba=(0.80, 0.06, 0.03, 1.0))

    # Realistic mast proportions in meters.  Each child section extends below
    # its joint frame so that it remains captured inside the parent at full travel.
    outer_len = 1.05
    outer_bottom_z = 0.06
    outer_top_z = outer_bottom_z + outer_len

    middle_len = 1.25
    middle_lower = -0.88
    middle_upper = middle_lower + middle_len
    middle_travel = 0.72

    inner_len = 1.15
    inner_lower = -0.80
    inner_upper = inner_lower + inner_len
    inner_travel = 0.68

    outer_stage = model.part("outer_stage")
    outer_stage.visual(
        Box((0.28, 0.20, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_anodized,
        name="base_plate",
    )
    outer_stage.visual(
        _rect_ring_mesh(
            "outer_tube_mesh",
            outer_w=0.120,
            outer_d=0.085,
            hole_w=0.108,
            hole_d=0.073,
            height=outer_len,
            radius=0.010,
            hole_radius=0.006,
        ),
        origin=Origin(xyz=(0.0, 0.0, outer_bottom_z + outer_len / 2.0)),
        material=brushed_aluminum,
        name="outer_tube",
    )
    outer_stage.visual(
        _rect_ring_mesh(
            "base_socket_mesh",
            outer_w=0.154,
            outer_d=0.118,
            hole_w=0.114,
            hole_d=0.079,
            height=0.090,
            radius=0.014,
            hole_radius=0.008,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_anodized,
        name="base_socket",
    )
    _add_rect_collar(
        outer_stage,
        prefix="outer_collar",
        outer_w=0.142,
        outer_d=0.106,
        hole_w=0.094,
        hole_d=0.066,
        height=0.080,
        z=outer_top_z,
        material=black_plastic,
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        _rect_ring_mesh(
            "middle_tube_mesh",
            outer_w=0.088,
            outer_d=0.060,
            hole_w=0.078,
            hole_d=0.050,
            height=middle_len,
            radius=0.008,
            hole_radius=0.004,
        ),
        origin=Origin(xyz=(0.0, 0.0, (middle_lower + middle_upper) / 2.0)),
        material=pale_aluminum,
        name="middle_tube",
    )
    _add_rect_collar(
        middle_stage,
        prefix="middle_collar",
        outer_w=0.108,
        outer_d=0.080,
        hole_w=0.068,
        hole_d=0.046,
        height=0.075,
        z=middle_upper - 0.020,
        material=black_plastic,
    )
    # Thin bonded height marks make the sliding member read as a measured mast.
    for index, z in enumerate((-0.08, 0.08, 0.24)):
        middle_stage.visual(
            Box((0.050, 0.002, 0.018)),
            origin=Origin(xyz=(0.0, -0.031, z)),
            material=safety_red,
            name=f"middle_mark_{index}",
        )

    inner_stage = model.part("inner_stage")
    inner_stage.visual(
        _rect_ring_mesh(
            "inner_tube_mesh",
            outer_w=0.062,
            outer_d=0.040,
            hole_w=0.054,
            hole_d=0.032,
            height=inner_len,
            radius=0.006,
            hole_radius=0.003,
        ),
        origin=Origin(xyz=(0.0, 0.0, (inner_lower + inner_upper) / 2.0)),
        material=brushed_aluminum,
        name="inner_tube",
    )
    inner_stage.visual(
        Box((0.070, 0.048, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, inner_upper + 0.008)),
        material=dark_anodized,
        name="top_cap",
    )
    inner_stage.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, inner_upper + 0.0435)),
        material=black_plastic,
        name="antenna_stud",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, outer_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=middle_travel),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, middle_upper)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=inner_travel),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_stage")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    outer_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.allow_overlap(
        outer,
        middle,
        elem_a="outer_tube",
        elem_b="middle_tube",
        reason=(
            "The middle rectangular section is intentionally nested inside the "
            "hollow outer sleeve; the mesh collision proxy reports the captured "
            "sliding fit as overlap."
        ),
    )
    ctx.allow_overlap(
        middle,
        inner,
        elem_a="middle_tube",
        elem_b="inner_tube",
        reason=(
            "The inner rectangular section is intentionally nested inside the "
            "hollow middle sleeve; the mesh collision proxy reports the captured "
            "sliding fit as overlap."
        ),
    )
    ctx.allow_overlap(
        outer,
        inner,
        elem_a="outer_tube",
        elem_b="inner_tube",
        reason=(
            "In the collapsed pose the innermost mast section passes down through "
            "the outer sleeve as part of the three-stage nested stack."
        ),
    )

    ctx.expect_within(
        middle,
        outer,
        axes="xy",
        inner_elem="middle_tube",
        outer_elem="outer_tube",
        margin=0.0,
        name="middle rectangular section nests inside the outer section footprint",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="middle_tube",
        margin=0.0,
        name="inner rectangular section nests inside the middle section footprint",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_tube",
        margin=0.0,
        name="inner section also fits through the outer sleeve footprint",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="z",
        elem_a="middle_tube",
        elem_b="outer_tube",
        min_overlap=0.70,
        name="collapsed middle section remains deeply inserted",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="z",
        elem_a="inner_tube",
        elem_b="middle_tube",
        min_overlap=0.65,
        name="collapsed inner section remains deeply inserted",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_tube",
        min_overlap=0.35,
        name="collapsed inner section passes through the outer sleeve",
    )

    rest_middle_pos = ctx.part_world_position(middle)
    rest_inner_pos = ctx.part_world_position(inner)
    with ctx.pose({outer_slide: 0.72, inner_slide: 0.68}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="z",
            elem_a="middle_tube",
            elem_b="outer_tube",
            min_overlap=0.12,
            name="extended middle section is still retained in the outer sleeve",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="z",
            elem_a="inner_tube",
            elem_b="middle_tube",
            min_overlap=0.10,
            name="extended inner section is still retained in the middle sleeve",
        )
        extended_middle_pos = ctx.part_world_position(middle)
        extended_inner_pos = ctx.part_world_position(inner)

    ctx.check(
        "slides extend along the mast axis",
        rest_middle_pos is not None
        and rest_inner_pos is not None
        and extended_middle_pos is not None
        and extended_inner_pos is not None
        and extended_middle_pos[2] > rest_middle_pos[2] + 0.70
        and extended_inner_pos[2] > rest_inner_pos[2] + 1.35,
        details=(
            f"middle rest={rest_middle_pos}, middle extended={extended_middle_pos}, "
            f"inner rest={rest_inner_pos}, inner extended={extended_inner_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
