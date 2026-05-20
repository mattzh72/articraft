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


BASE_W = 0.74
BASE_D = 0.46
BASE_T = 0.12

PEDESTAL_W = 0.16
PEDESTAL_D = 0.16
PEDESTAL_T = 0.04

POST_SIZE = 0.07
POST_H = 0.92
POST_SPACING = 0.38
POST_X = POST_SPACING / 2.0
POST_BASE_Z = BASE_T + PEDESTAL_T

TABLE_W = 0.54
TABLE_D = 0.24
TABLE_T = 0.03
TABLE_Y = 0.17

GUIDE_H = 0.085
GUIDE_WALL_T = 0.035
GUIDE_BEAM_T = 0.045
GUIDE_OPENING = POST_SIZE
GUIDE_FRAME_W = POST_SPACING + POST_SIZE + 2.0 * GUIDE_WALL_T

PLATEN_LOWER_Z = 0.18
PLATEN_TRAVEL = 0.55

BASE_COLOR = (0.18, 0.19, 0.21, 1.0)
POST_COLOR = (0.73, 0.75, 0.78, 1.0)
PLATEN_COLOR = (0.92, 0.48, 0.14, 1.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_post_platen_lift")

    base_mat = model.material("base_paint", rgba=BASE_COLOR)
    post_mat = model.material("post_steel", rgba=POST_COLOR)
    platen_mat = model.material("platen_orange", rgba=PLATEN_COLOR)

    base = model.part("base")
    base.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material=base_mat,
        name="base_slab",
    )
    for sign, name in ((-1.0, "left"), (1.0, "right")):
        x = sign * POST_X
        base.visual(
            Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_T)),
            origin=Origin(xyz=(x, 0.0, BASE_T + PEDESTAL_T / 2.0)),
            material=base_mat,
            name=f"{name}_pedestal",
        )
        base.visual(
            Box((POST_SIZE, POST_SIZE, POST_H)),
            origin=Origin(xyz=(x, 0.0, POST_BASE_Z + POST_H / 2.0)),
            material=post_mat,
            name=f"{name}_post",
        )

    platen = model.part("platen")
    platen.visual(
        Box((TABLE_W, TABLE_D, TABLE_T)),
        origin=Origin(xyz=(0.0, TABLE_Y, TABLE_T / 2.0)),
        material=platen_mat,
        name="table",
    )
    platen.visual(
        Box((GUIDE_FRAME_W, GUIDE_BEAM_T, GUIDE_H)),
        origin=Origin(
            xyz=(
                0.0,
                GUIDE_OPENING / 2.0 + GUIDE_BEAM_T / 2.0,
                TABLE_T + GUIDE_H / 2.0,
            )
        ),
        material=platen_mat,
        name="front_beam",
    )
    platen.visual(
        Box((GUIDE_FRAME_W, GUIDE_BEAM_T, GUIDE_H)),
        origin=Origin(
            xyz=(
                0.0,
                -(GUIDE_OPENING / 2.0 + GUIDE_BEAM_T / 2.0),
                TABLE_T + GUIDE_H / 2.0,
            )
        ),
        material=platen_mat,
        name="rear_beam",
    )
    for sign, side in ((-1.0, "left"), (1.0, "right")):
        x = sign * POST_X
        platen.visual(
            Box((GUIDE_WALL_T, GUIDE_OPENING, GUIDE_H)),
            origin=Origin(
                xyz=(
                    x - (POST_SIZE / 2.0 + GUIDE_WALL_T / 2.0),
                    0.0,
                    TABLE_T + GUIDE_H / 2.0,
                )
            ),
            material=platen_mat,
            name=f"{side}_outer_wall",
        )
        platen.visual(
            Box((GUIDE_WALL_T, GUIDE_OPENING, GUIDE_H)),
            origin=Origin(
                xyz=(
                    x + (POST_SIZE / 2.0 + GUIDE_WALL_T / 2.0),
                    0.0,
                    TABLE_T + GUIDE_H / 2.0,
                )
            ),
            material=platen_mat,
            name=f"{side}_inner_wall",
        )

    model.articulation(
        "base_to_platen",
        ArticulationType.PRISMATIC,
        parent=base,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, PLATEN_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.22,
            lower=0.0,
            upper=PLATEN_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platen = object_model.get_part("platen")
    lift = object_model.get_articulation("base_to_platen")

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

    limits = lift.motion_limits
    ctx.check(
        "platen_lift_joint_is_vertical_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )
    ctx.check(
        "platen_lift_motion_limits_match_design",
        limits is not None
        and limits.lower == 0.0
        and limits.upper == PLATEN_TRAVEL
        and limits.velocity > 0.0
        and limits.effort > 0.0,
        details=f"limits={limits}",
    )

    lower_pos = ctx.part_world_position(platen)
    with ctx.pose({lift: 0.0}):
        ctx.expect_gap(
            platen,
            base,
            axis="z",
            positive_elem="table",
            negative_elem="base_slab",
            min_gap=PLATEN_LOWER_Z - BASE_T - 1e-6,
            max_gap=PLATEN_LOWER_Z - BASE_T + 1e-6,
            name="lowered_platen_table_sits_above_base",
        )
        ctx.expect_contact(
            platen,
            base,
            elem_b="left_post",
            name="lowered_platen_contacts_left_post",
        )
        ctx.expect_contact(
            platen,
            base,
            elem_b="right_post",
            name="lowered_platen_contacts_right_post",
        )

    with ctx.pose({lift: PLATEN_TRAVEL}):
        upper_pos = ctx.part_world_position(platen)
        ctx.expect_gap(
            platen,
            base,
            axis="z",
            positive_elem="table",
            negative_elem="base_slab",
            min_gap=PLATEN_LOWER_Z + PLATEN_TRAVEL - BASE_T - 1e-6,
            max_gap=PLATEN_LOWER_Z + PLATEN_TRAVEL - BASE_T + 1e-6,
            name="raised_platen_table_travel_matches_joint",
        )
        ctx.expect_contact(
            platen,
            base,
            elem_b="left_post",
            name="raised_platen_contacts_left_post",
        )
        ctx.expect_contact(
            platen,
            base,
            elem_b="right_post",
            name="raised_platen_contacts_right_post",
        )

    ctx.check(
        "platen_origin_moves_up_by_prismatic_travel",
        lower_pos is not None
        and upper_pos is not None
        and abs((upper_pos[2] - lower_pos[2]) - PLATEN_TRAVEL) <= 1e-6,
        details=f"lower={lower_pos}, upper={upper_pos}, travel={PLATEN_TRAVEL}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
