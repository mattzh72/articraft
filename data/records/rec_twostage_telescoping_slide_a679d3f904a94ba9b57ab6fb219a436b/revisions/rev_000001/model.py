from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.42
OUTER_WIDTH = 0.034
OUTER_HEIGHT = 0.044
OUTER_WALL = 0.003

INNER_LENGTH = 0.32
INNER_BODY_WIDTH = 0.022
INNER_BODY_HEIGHT = 0.03
INNER_WALL = 0.0025
GUIDE_PAD_WIDTH = 0.012
GUIDE_PAD_HEIGHT = ((OUTER_HEIGHT - 2.0 * OUTER_WALL) - INNER_BODY_HEIGHT) / 2.0
FRONT_CAP_THICKNESS = 0.004

CLOSED_INSERT = 0.12
SLIDE_TRAVEL = 0.12


def _add_box_section_visuals(
    part,
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    material: str,
    prefix: str,
    x0: float = 0.0,
) -> None:
    span_width = width - 2.0 * wall

    part.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(x0 + length / 2.0, width / 2.0 - wall / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_right_wall",
    )
    part.visual(
        Box((length, wall, height)),
        origin=Origin(xyz=(x0 + length / 2.0, -(width / 2.0 - wall / 2.0), 0.0)),
        material=material,
        name=f"{prefix}_left_wall",
    )
    part.visual(
        Box((length, span_width, wall)),
        origin=Origin(xyz=(x0 + length / 2.0, 0.0, height / 2.0 - wall / 2.0)),
        material=material,
        name=f"{prefix}_top_wall",
    )
    part.visual(
        Box((length, span_width, wall)),
        origin=Origin(xyz=(x0 + length / 2.0, 0.0, -(height / 2.0 - wall / 2.0))),
        material=material,
        name=f"{prefix}_bottom_wall",
    )


def _add_inner_runner_visuals(part, *, material: str) -> None:
    body_length = INNER_LENGTH - FRONT_CAP_THICKNESS
    _add_box_section_visuals(
        part,
        length=body_length,
        width=INNER_BODY_WIDTH,
        height=INNER_BODY_HEIGHT,
        wall=INNER_WALL,
        material=material,
        prefix="runner",
    )

    part.visual(
        Box((body_length, GUIDE_PAD_WIDTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(
            xyz=(
                body_length / 2.0,
                0.0,
                INNER_BODY_HEIGHT / 2.0 + GUIDE_PAD_HEIGHT / 2.0,
            )
        ),
        material=material,
        name="top_guide_pad",
    )
    part.visual(
        Box((body_length, GUIDE_PAD_WIDTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(
            xyz=(
                body_length / 2.0,
                0.0,
                -(INNER_BODY_HEIGHT / 2.0 + GUIDE_PAD_HEIGHT / 2.0),
            )
        ),
        material=material,
        name="bottom_guide_pad",
    )
    part.visual(
        Box((FRONT_CAP_THICKNESS, INNER_BODY_WIDTH, INNER_BODY_HEIGHT)),
        origin=Origin(
            xyz=(INNER_LENGTH - FRONT_CAP_THICKNESS / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="front_face",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide")

    model.material("outer_steel", rgba=(0.34, 0.36, 0.40, 1.0))
    model.material("inner_steel", rgba=(0.69, 0.71, 0.74, 1.0))

    outer = model.part("outer_section")
    _add_box_section_visuals(
        outer,
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        material="outer_steel",
        prefix="outer",
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=1.3,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, 0.0)),
    )

    inner = model.part("inner_runner")
    _add_inner_runner_visuals(inner, material="inner_steel")
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_BODY_WIDTH, INNER_BODY_HEIGHT + 2.0 * GUIDE_PAD_HEIGHT)),
        mass=0.7,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(CLOSED_INSERT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_section")
    inner = object_model.get_part("inner_runner")
    slide = object_model.get_articulation("outer_to_inner")
    limits = slide.motion_limits

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "slide_joint_configuration",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper == SLIDE_TRAVEL,
        f"type={slide.articulation_type}, axis={slide.axis}, limits={limits}",
    )

    ctx.expect_contact(
        inner,
        outer,
        contact_tol=5e-5,
        name="runner_supported_in_closed_pose",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        min_overlap=0.29,
        name="closed_pose_has_clear_stage_overlap",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="yz",
        margin=0.0,
        name="runner_stays_nested_within_outer_profile",
    )
    ctx.expect_gap(
        inner,
        outer,
        axis="x",
        positive_elem="front_face",
        min_gap=0.01,
        max_gap=0.03,
        name="front_face_projects_past_outer_in_closed_pose",
    )

    closed_x = ctx.part_world_position(inner)[0]
    with ctx.pose({slide: SLIDE_TRAVEL}):
        open_x = ctx.part_world_position(inner)[0]
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_extension")
        ctx.expect_contact(
            inner,
            outer,
            contact_tol=5e-5,
            name="runner_supported_in_extended_pose",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            min_overlap=0.17,
            name="extended_pose_retains_engagement_overlap",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="x",
            positive_elem="front_face",
            min_gap=0.13,
            name="front_face_advances_forward_at_full_extension",
        )

    ctx.check(
        "runner_moves_forward_along_slide_axis",
        open_x > closed_x + 0.95 * SLIDE_TRAVEL,
        f"closed_x={closed_x:.4f}, open_x={open_x:.4f}, travel={SLIDE_TRAVEL:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
