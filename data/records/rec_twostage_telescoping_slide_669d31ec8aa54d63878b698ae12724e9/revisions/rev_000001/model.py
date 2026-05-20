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


OUTER_LEN = 0.34
OUTER_W = 0.14
OUTER_H = 0.062
BASE_T = 0.010
WALL_T = 0.008
LIP_T = 0.006
LIP_DEPTH = 0.022
BACK_T = 0.012

STAGE_BLOCK_LEN = 0.26
STAGE_BLOCK_W = 0.104
STAGE_BLOCK_H = 0.026
STAGE_NECK_X0 = 0.012
STAGE_NECK_LEN = 0.236
STAGE_NECK_W = 0.070
STAGE_PLATE_LEN = 0.30
STAGE_PLATE_W = 0.118
STAGE_PLATE_T = 0.012
STAGE_PLATE_Z0 = OUTER_H + 0.002 - BASE_T
SLIDE_HOME_X = -OUTER_LEN / 2.0 + BACK_T + 0.004
SLIDE_TRAVEL = 0.22
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_service_fixture")
    model.material("powder_coat_graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        Box((OUTER_LEN, OUTER_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="powder_coat_graphite",
        name="base_pan",
    )
    outer_sleeve.visual(
        Box((OUTER_LEN, WALL_T, OUTER_H - BASE_T)),
        origin=Origin(xyz=(0.0, OUTER_W / 2.0 - WALL_T / 2.0, BASE_T + (OUTER_H - BASE_T) / 2.0)),
        material="powder_coat_graphite",
        name="left_wall",
    )
    outer_sleeve.visual(
        Box((OUTER_LEN, WALL_T, OUTER_H - BASE_T)),
        origin=Origin(xyz=(0.0, -OUTER_W / 2.0 + WALL_T / 2.0, BASE_T + (OUTER_H - BASE_T) / 2.0)),
        material="powder_coat_graphite",
        name="right_wall",
    )
    outer_sleeve.visual(
        Box((OUTER_LEN, LIP_DEPTH, LIP_T)),
        origin=Origin(
            xyz=(
                0.0,
                OUTER_W / 2.0 - WALL_T - LIP_DEPTH / 2.0,
                OUTER_H - LIP_T / 2.0,
            )
        ),
        material="powder_coat_graphite",
        name="left_lip",
    )
    outer_sleeve.visual(
        Box((OUTER_LEN, LIP_DEPTH, LIP_T)),
        origin=Origin(
            xyz=(
                0.0,
                -OUTER_W / 2.0 + WALL_T + LIP_DEPTH / 2.0,
                OUTER_H - LIP_T / 2.0,
            )
        ),
        material="powder_coat_graphite",
        name="right_lip",
    )
    outer_sleeve.visual(
        Box((BACK_T, OUTER_W, OUTER_H)),
        origin=Origin(xyz=(-OUTER_LEN / 2.0 + BACK_T / 2.0, 0.0, OUTER_H / 2.0)),
        material="powder_coat_graphite",
        name="back_stop",
    )
    outer_sleeve.inertial = Inertial.from_geometry(
        Box((OUTER_LEN, OUTER_W, OUTER_H)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, OUTER_H / 2.0)),
    )

    inner_stage = model.part("inner_stage")
    inner_stage.visual(
        Box((STAGE_BLOCK_LEN, STAGE_BLOCK_W, STAGE_BLOCK_H)),
        origin=Origin(xyz=(STAGE_BLOCK_LEN / 2.0, 0.0, STAGE_BLOCK_H / 2.0)),
        material="machined_aluminum",
        name="stage_block",
    )
    inner_stage.visual(
        Box((STAGE_NECK_LEN, STAGE_NECK_W, STAGE_PLATE_Z0 - STAGE_BLOCK_H)),
        origin=Origin(
            xyz=(
                STAGE_NECK_X0 + STAGE_NECK_LEN / 2.0,
                0.0,
                STAGE_BLOCK_H + (STAGE_PLATE_Z0 - STAGE_BLOCK_H) / 2.0,
            )
        ),
        material="machined_aluminum",
        name="stage_neck",
    )
    inner_stage.visual(
        Box((STAGE_PLATE_LEN, STAGE_PLATE_W, STAGE_PLATE_T)),
        origin=Origin(
            xyz=(
                STAGE_PLATE_LEN / 2.0,
                0.0,
                STAGE_PLATE_Z0 + STAGE_PLATE_T / 2.0,
            )
        ),
        material="machined_aluminum",
        name="stage_plate",
    )
    inner_stage.inertial = Inertial.from_geometry(
        Box((STAGE_PLATE_LEN, STAGE_PLATE_W, STAGE_PLATE_Z0 + STAGE_PLATE_T)),
        mass=1.8,
        origin=Origin(
            xyz=(
                STAGE_PLATE_LEN / 2.0,
                0.0,
                (STAGE_PLATE_Z0 + STAGE_PLATE_T) / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_stage_slide",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_stage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, BASE_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_stage = object_model.get_part("inner_stage")
    slide = object_model.get_articulation("outer_to_stage_slide")

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
        "slide_is_prismatic_on_x",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected PRISMATIC axis (1, 0, 0), got {slide.articulation_type} axis={slide.axis}",
    )

    ctx.expect_contact(
        inner_stage,
        outer_sleeve,
        elem_a="stage_block",
        elem_b="base_pan",
        name="stage_is_supported_by_sleeve_at_home",
    )
    ctx.expect_within(
        inner_stage,
        outer_sleeve,
        axes="y",
        margin=0.0,
        name="stage_stays_captured_laterally_at_home",
    )
    ctx.expect_overlap(
        inner_stage,
        outer_sleeve,
        axes="x",
        min_overlap=0.24,
        name="home_pose_has_deep_sleeve_engagement",
    )
    ctx.expect_gap(
        inner_stage,
        outer_sleeve,
        axis="z",
        positive_elem="stage_plate",
        negative_elem="left_lip",
        min_gap=0.0015,
        max_gap=0.0035,
        name="stage_plate_runs_just_above_sleeve_lips",
    )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            inner_stage,
            outer_sleeve,
            elem_a="stage_block",
            elem_b="base_pan",
            name="stage_remains_supported_at_full_extension",
        )
        ctx.expect_within(
            inner_stage,
            outer_sleeve,
            axes="y",
            margin=0.0,
            name="stage_stays_captured_laterally_at_full_extension",
        )
        ctx.expect_overlap(
            inner_stage,
            outer_sleeve,
            axes="x",
            min_overlap=0.10,
            name="full_extension_retains_visible_overlap",
        )
        ctx.expect_origin_gap(
            inner_stage,
            outer_sleeve,
            axis="x",
            min_gap=0.06,
            name="full_extension_moves_stage_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
