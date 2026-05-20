from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


POST_RADIUS = 0.09
POST_HEIGHT = 1.95
POST_HINGE_AXIS_X = 0.12

SECTION_WIDTH = 1.55
SECTION_HEIGHT = 1.25
SECTION_DEPTH = 0.045
FRAME_TUBE = 0.055
FRAME_STANDOFF = 0.017
BOTTOM_CLEARANCE = 0.12

HINGE_RADIUS = 0.018
POST_KNUCKLE_LEN = 0.20
LEAF_POST_KNUCKLE_LEN = 0.50
CENTER_OUTER_KNUCKLE_LEN = 0.21
CENTER_INNER_KNUCKLE_LEN = 0.60

CENTER_OFFSET_Y = 0.058
INNER_FRAME_Y = -CENTER_OFFSET_Y

LOWER_KNUCKLE_Z = 0.22
UPPER_KNUCKLE_Z = SECTION_HEIGHT - 0.22
MID_KNUCKLE_Z = SECTION_HEIGHT * 0.50


def _add_diagonal_brace(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(
                (start[0] + end[0]) * 0.5,
                (start[1] + end[1]) * 0.5,
                (start[2] + end[2]) * 0.5,
            ),
            rpy=(0.0, atan2(dx, dz), 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bifold_vehicle_access_gate")

    galvanized = model.material("galvanized", rgba=(0.74, 0.75, 0.77, 1.0))
    gate_powder = model.material("gate_powder", rgba=(0.28, 0.31, 0.34, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    footing = model.material("footing", rgba=(0.34, 0.34, 0.35, 1.0))

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT * 0.5)),
        material=galvanized,
        name="post_shaft",
    )
    post.visual(
        Cylinder(radius=POST_RADIUS * 1.28, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=footing,
        name="post_base_collar",
    )
    post.visual(
        Cylinder(radius=POST_RADIUS * 1.03, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT + 0.006)),
        material=galvanized,
        name="post_cap",
    )

    bracket_span_x = POST_HINGE_AXIS_X - POST_RADIUS + 0.006
    bracket_center_x = POST_RADIUS - 0.003 + bracket_span_x * 0.5
    for z_center, barrel_name, bracket_name in (
        (BOTTOM_CLEARANCE + LOWER_KNUCKLE_Z, "post_lower_knuckle", "post_lower_bracket"),
        (BOTTOM_CLEARANCE + UPPER_KNUCKLE_Z, "post_upper_knuckle", "post_upper_bracket"),
    ):
        post.visual(
            Box((bracket_span_x, SECTION_DEPTH * 0.95, 0.10)),
            origin=Origin(xyz=(bracket_center_x, 0.0, z_center)),
            material=hinge_steel,
            name=bracket_name,
        )
        post.visual(
            Cylinder(radius=HINGE_RADIUS, length=POST_KNUCKLE_LEN),
            origin=Origin(xyz=(POST_HINGE_AXIS_X, 0.0, z_center)),
            material=hinge_steel,
            name=barrel_name,
        )

    outer_section = model.part("outer_section")
    outer_section.visual(
        Box((FRAME_TUBE, SECTION_DEPTH, SECTION_HEIGHT)),
        origin=Origin(
            xyz=(FRAME_STANDOFF + FRAME_TUBE * 0.5, 0.0, SECTION_HEIGHT * 0.5),
        ),
        material=gate_powder,
        name="outer_leading_stile",
    )
    outer_section.visual(
        Box((FRAME_TUBE, SECTION_DEPTH, SECTION_HEIGHT)),
        origin=Origin(
            xyz=(SECTION_WIDTH - FRAME_STANDOFF - FRAME_TUBE * 0.5, 0.0, SECTION_HEIGHT * 0.5),
        ),
        material=gate_powder,
        name="outer_trailing_stile",
    )
    outer_section.visual(
        Box((SECTION_WIDTH - 2.0 * FRAME_STANDOFF, SECTION_DEPTH, FRAME_TUBE)),
        origin=Origin(xyz=(SECTION_WIDTH * 0.5, 0.0, FRAME_TUBE * 0.5)),
        material=gate_powder,
        name="outer_bottom_rail",
    )
    outer_section.visual(
        Box((SECTION_WIDTH - 2.0 * FRAME_STANDOFF, SECTION_DEPTH, FRAME_TUBE)),
        origin=Origin(xyz=(SECTION_WIDTH * 0.5, 0.0, SECTION_HEIGHT - FRAME_TUBE * 0.5)),
        material=gate_powder,
        name="outer_top_rail",
    )
    outer_section.visual(
        Box((SECTION_WIDTH - 2.0 * FRAME_STANDOFF - 0.12, SECTION_DEPTH * 0.92, FRAME_TUBE * 0.72)),
        origin=Origin(xyz=(SECTION_WIDTH * 0.5, 0.0, SECTION_HEIGHT * 0.52)),
        material=gate_powder,
        name="outer_mid_rail",
    )
    _add_diagonal_brace(
        outer_section,
        start=(FRAME_STANDOFF + FRAME_TUBE * 0.75, 0.0, FRAME_TUBE * 0.72),
        end=(SECTION_WIDTH - FRAME_STANDOFF - FRAME_TUBE * 0.78, 0.0, SECTION_HEIGHT - FRAME_TUBE * 0.72),
        radius=0.015,
        material=gate_powder,
        name="outer_diagonal_brace",
    )
    outer_section.visual(
        Box((FRAME_STANDOFF, SECTION_DEPTH * 0.88, 0.30)),
        origin=Origin(xyz=(FRAME_STANDOFF * 0.5, 0.0, MID_KNUCKLE_Z)),
        material=hinge_steel,
        name="outer_post_mount",
    )
    outer_section.visual(
        Cylinder(radius=HINGE_RADIUS, length=LEAF_POST_KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, MID_KNUCKLE_Z)),
        material=hinge_steel,
        name="outer_post_knuckle",
    )
    for z_center, ear_name, barrel_name in (
        (LOWER_KNUCKLE_Z, "outer_center_lower_ear", "outer_center_lower_knuckle"),
        (UPPER_KNUCKLE_Z, "outer_center_upper_ear", "outer_center_upper_knuckle"),
    ):
        outer_section.visual(
            Box((FRAME_STANDOFF, CENTER_OFFSET_Y + SECTION_DEPTH * 0.5, 0.09)),
            origin=Origin(
                xyz=(SECTION_WIDTH - FRAME_STANDOFF * 0.5, CENTER_OFFSET_Y * 0.5, z_center),
            ),
            material=hinge_steel,
            name=ear_name,
        )
        outer_section.visual(
            Cylinder(radius=HINGE_RADIUS, length=CENTER_OUTER_KNUCKLE_LEN),
            origin=Origin(xyz=(SECTION_WIDTH, CENTER_OFFSET_Y, z_center)),
            material=hinge_steel,
            name=barrel_name,
        )

    inner_section = model.part("inner_section")
    inner_section.visual(
        Box((FRAME_TUBE, SECTION_DEPTH, SECTION_HEIGHT)),
        origin=Origin(
            xyz=(FRAME_STANDOFF + FRAME_TUBE * 0.5, INNER_FRAME_Y, SECTION_HEIGHT * 0.5),
        ),
        material=gate_powder,
        name="inner_leading_stile",
    )
    inner_section.visual(
        Box((FRAME_TUBE, SECTION_DEPTH, SECTION_HEIGHT)),
        origin=Origin(
            xyz=(SECTION_WIDTH - FRAME_STANDOFF - FRAME_TUBE * 0.5, INNER_FRAME_Y, SECTION_HEIGHT * 0.5),
        ),
        material=gate_powder,
        name="inner_trailing_stile",
    )
    inner_section.visual(
        Box((SECTION_WIDTH - 2.0 * FRAME_STANDOFF, SECTION_DEPTH, FRAME_TUBE)),
        origin=Origin(xyz=(SECTION_WIDTH * 0.5, INNER_FRAME_Y, FRAME_TUBE * 0.5)),
        material=gate_powder,
        name="inner_bottom_rail",
    )
    inner_section.visual(
        Box((SECTION_WIDTH - 2.0 * FRAME_STANDOFF, SECTION_DEPTH, FRAME_TUBE)),
        origin=Origin(xyz=(SECTION_WIDTH * 0.5, INNER_FRAME_Y, SECTION_HEIGHT - FRAME_TUBE * 0.5)),
        material=gate_powder,
        name="inner_top_rail",
    )
    inner_section.visual(
        Box((SECTION_WIDTH - 2.0 * FRAME_STANDOFF - 0.12, SECTION_DEPTH * 0.92, FRAME_TUBE * 0.72)),
        origin=Origin(xyz=(SECTION_WIDTH * 0.5, INNER_FRAME_Y, SECTION_HEIGHT * 0.48)),
        material=gate_powder,
        name="inner_mid_rail",
    )
    _add_diagonal_brace(
        inner_section,
        start=(FRAME_STANDOFF + FRAME_TUBE * 0.78, INNER_FRAME_Y, SECTION_HEIGHT - FRAME_TUBE * 0.72),
        end=(SECTION_WIDTH - FRAME_STANDOFF - FRAME_TUBE * 0.75, INNER_FRAME_Y, FRAME_TUBE * 0.72),
        radius=0.015,
        material=gate_powder,
        name="inner_diagonal_brace",
    )
    inner_section.visual(
        Box((FRAME_STANDOFF, CENTER_OFFSET_Y + SECTION_DEPTH * 0.88, 0.30)),
        origin=Origin(xyz=(FRAME_STANDOFF * 0.5, INNER_FRAME_Y * 0.5, MID_KNUCKLE_Z)),
        material=hinge_steel,
        name="inner_center_mount",
    )
    inner_section.visual(
        Cylinder(radius=HINGE_RADIUS, length=CENTER_INNER_KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, MID_KNUCKLE_Z)),
        material=hinge_steel,
        name="inner_center_knuckle",
    )

    model.articulation(
        "post_to_outer",
        ArticulationType.REVOLUTE,
        parent=post,
        child=outer_section,
        origin=Origin(xyz=(POST_HINGE_AXIS_X, 0.0, BOTTOM_CLEARANCE)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.65),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_section,
        child=inner_section,
        origin=Origin(xyz=(SECTION_WIDTH, CENTER_OFFSET_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=3.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    post = object_model.get_part("post")
    outer = object_model.get_part("outer_section")
    inner = object_model.get_part("inner_section")
    outer_hinge = object_model.get_articulation("post_to_outer")
    inner_hinge = object_model.get_articulation("outer_to_inner")

    ctx.check(
        "outer hinge axis is vertical",
        tuple(outer_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={outer_hinge.axis}",
    )
    ctx.check(
        "inner hinge axis is vertical",
        tuple(inner_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={inner_hinge.axis}",
    )

    with ctx.pose({outer_hinge: 0.0, inner_hinge: 0.0}):
        ctx.expect_gap(
            outer,
            post,
            axis="x",
            positive_elem="outer_leading_stile",
            negative_elem="post_shaft",
            min_gap=0.03,
            max_gap=0.06,
            name="outer frame clears the round post in the closed pose",
        )
        ctx.expect_overlap(
            post,
            outer,
            axes="xy",
            elem_a="post_upper_knuckle",
            elem_b="outer_post_knuckle",
            min_overlap=0.02,
            name="outer section shares the post hinge axis",
        )
        ctx.expect_overlap(
            outer,
            inner,
            axes="xy",
            elem_a="outer_center_upper_knuckle",
            elem_b="inner_center_knuckle",
            min_overlap=0.02,
            name="inner section shares the center knuckle axis",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="x",
            positive_elem="inner_leading_stile",
            negative_elem="outer_trailing_stile",
            min_gap=0.02,
            max_gap=0.05,
            name="closed leaves meet with a realistic center hinge gap",
        )

    closed_outer_aabb = ctx.part_world_aabb(outer)
    closed_inner_aabb = ctx.part_world_aabb(inner)
    with ctx.pose({outer_hinge: 1.05, inner_hinge: 2.85}):
        opened_outer_aabb = ctx.part_world_aabb(outer)
        folded_inner_aabb = ctx.part_world_aabb(inner)
        ctx.check(
            "outer section swings away from the post",
            closed_outer_aabb is not None
            and opened_outer_aabb is not None
            and opened_outer_aabb[1][1] > closed_outer_aabb[1][1] + 0.9,
            details=f"closed={closed_outer_aabb}, opened={opened_outer_aabb}",
        )
        ctx.check(
            "inner section folds back toward the outer section",
            closed_inner_aabb is not None
            and folded_inner_aabb is not None
            and folded_inner_aabb[1][0] < closed_inner_aabb[1][0] - 1.0,
            details=f"closed={closed_inner_aabb}, folded={folded_inner_aabb}",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="y",
            positive_elem="inner_leading_stile",
            negative_elem="outer_trailing_stile",
            min_gap=0.006,
            name="folded inner leaf sits to the side of the outer leaf",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
