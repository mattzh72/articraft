from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.45
OUTER_HEIGHT = 0.044
OUTER_BACK_THICKNESS = 0.0016
OUTER_LIP_THICKNESS = 0.0016
OUTER_BACK_Y_MIN = -0.0090
OUTER_FRONT_Y_MAX = 0.0030
OUTER_LIP_DEPTH = OUTER_FRONT_Y_MAX - (OUTER_BACK_Y_MIN + OUTER_BACK_THICKNESS)
OUTER_BACK_CENTER_Y = OUTER_BACK_Y_MIN + (OUTER_BACK_THICKNESS / 2.0)
OUTER_LIP_CENTER_Y = (OUTER_FRONT_Y_MAX + OUTER_BACK_Y_MIN + OUTER_BACK_THICKNESS) / 2.0

INTERMEDIATE_LENGTH = 0.38
INTERMEDIATE_CORE_WIDTH = 0.0064
INTERMEDIATE_CORE_HEIGHT = 0.0360
INTERMEDIATE_RUNNER_HEIGHT = (
    OUTER_HEIGHT - (2.0 * OUTER_LIP_THICKNESS) - INTERMEDIATE_CORE_HEIGHT
) / 2.0
INTERMEDIATE_RUNNER_WIDTH = 0.0032
INTERMEDIATE_CENTER_Y = -0.0014

SLIDE_TRAVEL = 0.22


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide")

    model.material("outer_steel", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("inner_zinc", rgba=(0.75, 0.77, 0.80, 1.0))

    outer_slide = model.part("outer_slide")
    outer_slide.visual(
        Box((OUTER_LENGTH, OUTER_BACK_THICKNESS, OUTER_HEIGHT)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, OUTER_BACK_CENTER_Y, 0.0)),
        material="outer_steel",
        name="back_web",
    )
    outer_slide.visual(
        Box((OUTER_LENGTH, OUTER_LIP_DEPTH, OUTER_LIP_THICKNESS)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                OUTER_LIP_CENTER_Y,
                OUTER_HEIGHT / 2.0 - OUTER_LIP_THICKNESS / 2.0,
            )
        ),
        material="outer_steel",
        name="top_lip",
    )
    outer_slide.visual(
        Box((OUTER_LENGTH, OUTER_LIP_DEPTH, OUTER_LIP_THICKNESS)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                OUTER_LIP_CENTER_Y,
                -OUTER_HEIGHT / 2.0 + OUTER_LIP_THICKNESS / 2.0,
            )
        ),
        material="outer_steel",
        name="bottom_lip",
    )

    intermediate_member = model.part("intermediate_member")
    intermediate_member.visual(
        Box((INTERMEDIATE_LENGTH, INTERMEDIATE_CORE_WIDTH, INTERMEDIATE_CORE_HEIGHT)),
        origin=Origin(xyz=(INTERMEDIATE_LENGTH / 2.0, INTERMEDIATE_CENTER_Y, 0.0)),
        material="inner_zinc",
        name="core",
    )
    intermediate_member.visual(
        Box((INTERMEDIATE_LENGTH, INTERMEDIATE_RUNNER_WIDTH, INTERMEDIATE_RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(
                INTERMEDIATE_LENGTH / 2.0,
                INTERMEDIATE_CENTER_Y,
                INTERMEDIATE_CORE_HEIGHT / 2.0 + INTERMEDIATE_RUNNER_HEIGHT / 2.0,
            )
        ),
        material="inner_zinc",
        name="top_runner",
    )
    intermediate_member.visual(
        Box((INTERMEDIATE_LENGTH, INTERMEDIATE_RUNNER_WIDTH, INTERMEDIATE_RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(
                INTERMEDIATE_LENGTH / 2.0,
                INTERMEDIATE_CENTER_Y,
                -INTERMEDIATE_CORE_HEIGHT / 2.0 - INTERMEDIATE_RUNNER_HEIGHT / 2.0,
            )
        ),
        material="inner_zinc",
        name="bottom_runner",
    )

    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer_slide,
        child=intermediate_member,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_slide = object_model.get_part("outer_slide")
    intermediate_member = object_model.get_part("intermediate_member")
    slide = object_model.get_articulation("outer_to_intermediate")
    top_lip = outer_slide.get_visual("top_lip")
    bottom_lip = outer_slide.get_visual("bottom_lip")
    top_runner = intermediate_member.get_visual("top_runner")
    bottom_runner = intermediate_member.get_visual("bottom_runner")

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

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            intermediate_member,
            outer_slide,
            elem_a=top_runner,
            elem_b=top_lip,
            name="closed top runner bears on top guide lip",
        )
        ctx.expect_contact(
            intermediate_member,
            outer_slide,
            elem_a=bottom_runner,
            elem_b=bottom_lip,
            name="closed bottom runner bears on bottom guide lip",
        )
        ctx.expect_within(
            intermediate_member,
            outer_slide,
            axes="yz",
            margin=0.0,
            name="closed intermediate remains nested within outer section",
        )
        ctx.expect_overlap(
            intermediate_member,
            outer_slide,
            axes="x",
            min_overlap=0.36,
            name="closed slide keeps deep longitudinal overlap",
        )

        outer_aabb = ctx.part_world_aabb(outer_slide)
        intermediate_aabb = ctx.part_world_aabb(intermediate_member)
        if outer_aabb is None or intermediate_aabb is None:
            ctx.fail("closed pose bounds available", "missing world AABB for slide parts")
        else:
            remaining_front_margin = outer_aabb[1][0] - intermediate_aabb[1][0]
            ctx.check(
                "closed intermediate stays behind outer front edge",
                remaining_front_margin >= 0.05,
                (
                    "expected at least 0.05 m of front retention when closed; "
                    f"measured {remaining_front_margin:.4f} m"
                ),
            )

    full_extension = slide.motion_limits.upper
    with ctx.pose({slide: full_extension}):
        ctx.expect_contact(
            intermediate_member,
            outer_slide,
            elem_a=top_runner,
            elem_b=top_lip,
            name="extended top runner stays in contact with top guide lip",
        )
        ctx.expect_contact(
            intermediate_member,
            outer_slide,
            elem_a=bottom_runner,
            elem_b=bottom_lip,
            name="extended bottom runner stays in contact with bottom guide lip",
        )
        ctx.expect_within(
            intermediate_member,
            outer_slide,
            axes="yz",
            margin=0.0,
            name="extended intermediate stays laterally guided by outer section",
        )
        ctx.expect_overlap(
            intermediate_member,
            outer_slide,
            axes="x",
            min_overlap=0.22,
            name="extended slide keeps clear member overlap",
        )
        ctx.expect_origin_gap(
            intermediate_member,
            outer_slide,
            axis="x",
            min_gap=SLIDE_TRAVEL - 0.001,
            max_gap=SLIDE_TRAVEL + 0.001,
            name="prismatic joint translates intermediate along slide axis",
        )
        ctx.expect_origin_distance(
            intermediate_member,
            outer_slide,
            axes="yz",
            max_dist=1e-6,
            name="prismatic extension does not drift off axis",
        )

        outer_aabb = ctx.part_world_aabb(outer_slide)
        intermediate_aabb = ctx.part_world_aabb(intermediate_member)
        if outer_aabb is None or intermediate_aabb is None:
            ctx.fail("extended pose bounds available", "missing world AABB for slide parts")
        else:
            front_protrusion = intermediate_aabb[1][0] - outer_aabb[1][0]
            ctx.check(
                "extended intermediate protrudes from the front",
                front_protrusion >= 0.12,
                (
                    "expected the intermediate member to extend visibly beyond the "
                    f"outer body; measured protrusion {front_protrusion:.4f} m"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
