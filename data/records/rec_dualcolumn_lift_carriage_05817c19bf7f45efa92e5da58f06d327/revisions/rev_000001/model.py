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


BASE_W = 0.38
BASE_D = 0.26
BASE_T = 0.035

UPRIGHT_W = 0.05
UPRIGHT_D = 0.08
UPRIGHT_H = 1.12
UPRIGHT_Y = -0.01
UPRIGHT_X = 0.165

TOP_BEAM_H = 0.055

RAIL_SIZE = 0.026
RAIL_X = 0.11
RAIL_Y = 0.043
RAIL_H = UPRIGHT_H

SADDLE_H = 0.18
SHOE_OUTER_W = 0.058
SHOE_OUTER_D = 0.052
SHOE_CHANNEL_W = RAIL_SIZE
SHOE_CHANNEL_D = RAIL_SIZE
SHOE_SIDE_T = (SHOE_OUTER_W - SHOE_CHANNEL_W) / 2.0
SHOE_END_T = (SHOE_OUTER_D - SHOE_CHANNEL_D) / 2.0
CROSSHEAD_W = (2.0 * RAIL_X) + SHOE_OUTER_W
CROSSHEAD_D = 0.018
CROSSHEAD_H = 0.14
WORK_FACE_W = 0.18
WORK_FACE_D = 0.024
WORK_FACE_H = 0.18
WORK_FACE_Y = (SHOE_OUTER_D / 2.0) + CROSSHEAD_D + (WORK_FACE_D / 2.0)

HOME_Z = 0.18
TRAVEL = 0.72

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_rail_service_lift")

    model.material("frame_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("guide_steel", rgba=(0.71, 0.74, 0.78, 1.0))
    model.material("saddle_gray", rgba=(0.63, 0.65, 0.68, 1.0))
    model.material("face_blue", rgba=(0.23, 0.41, 0.67, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="frame_gray",
        name="base_foot",
    )
    frame.visual(
        Box((UPRIGHT_W, UPRIGHT_D, UPRIGHT_H)),
        origin=Origin(xyz=(-UPRIGHT_X, UPRIGHT_Y, BASE_T + (UPRIGHT_H / 2.0))),
        material="frame_gray",
        name="left_upright",
    )
    frame.visual(
        Box((UPRIGHT_W, UPRIGHT_D, UPRIGHT_H)),
        origin=Origin(xyz=(UPRIGHT_X, UPRIGHT_Y, BASE_T + (UPRIGHT_H / 2.0))),
        material="frame_gray",
        name="right_upright",
    )
    frame.visual(
        Box((BASE_W, UPRIGHT_D, TOP_BEAM_H)),
        origin=Origin(
            xyz=(0.0, UPRIGHT_Y, BASE_T + UPRIGHT_H + (TOP_BEAM_H / 2.0))
        ),
        material="frame_gray",
        name="top_beam",
    )
    frame.visual(
        Box((BASE_W - 0.08, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.0675, BASE_T + 0.11)),
        material="frame_gray",
        name="rear_tie",
    )
    frame.visual(
        Box((RAIL_SIZE, RAIL_SIZE, RAIL_H)),
        origin=Origin(xyz=(-RAIL_X, RAIL_Y, BASE_T + (RAIL_H / 2.0))),
        material="guide_steel",
        name="left_guide",
    )
    frame.visual(
        Box((RAIL_SIZE, RAIL_SIZE, RAIL_H)),
        origin=Origin(xyz=(RAIL_X, RAIL_Y, BASE_T + (RAIL_H / 2.0))),
        material="guide_steel",
        name="right_guide",
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, BASE_T + UPRIGHT_H + TOP_BEAM_H)),
        mass=24.0,
        origin=Origin(
            xyz=(0.0, 0.0, (BASE_T + UPRIGHT_H + TOP_BEAM_H) / 2.0)
        ),
    )

    saddle = model.part("saddle")
    saddle.visual(
        Box((SHOE_SIDE_T, SHOE_CHANNEL_D, SADDLE_H)),
        origin=Origin(
            xyz=(-RAIL_X - (SHOE_CHANNEL_W / 2.0) - (SHOE_SIDE_T / 2.0), 0.0, 0.0)
        ),
        material="saddle_gray",
        name="left_outboard_wall",
    )
    saddle.visual(
        Box((SHOE_SIDE_T, SHOE_CHANNEL_D, SADDLE_H)),
        origin=Origin(
            xyz=(-RAIL_X + (SHOE_CHANNEL_W / 2.0) + (SHOE_SIDE_T / 2.0), 0.0, 0.0)
        ),
        material="saddle_gray",
        name="left_inboard_wall",
    )
    saddle.visual(
        Box((SHOE_OUTER_W, SHOE_END_T, SADDLE_H)),
        origin=Origin(
            xyz=(-RAIL_X, (SHOE_CHANNEL_D / 2.0) + (SHOE_END_T / 2.0), 0.0)
        ),
        material="saddle_gray",
        name="left_front_bridge",
    )
    saddle.visual(
        Box((SHOE_OUTER_W, SHOE_END_T, SADDLE_H)),
        origin=Origin(
            xyz=(-RAIL_X, -(SHOE_CHANNEL_D / 2.0) - (SHOE_END_T / 2.0), 0.0)
        ),
        material="saddle_gray",
        name="left_rear_bridge",
    )
    saddle.visual(
        Box((SHOE_SIDE_T, SHOE_CHANNEL_D, SADDLE_H)),
        origin=Origin(
            xyz=(RAIL_X - (SHOE_CHANNEL_W / 2.0) - (SHOE_SIDE_T / 2.0), 0.0, 0.0)
        ),
        material="saddle_gray",
        name="right_inboard_wall",
    )
    saddle.visual(
        Box((SHOE_SIDE_T, SHOE_CHANNEL_D, SADDLE_H)),
        origin=Origin(
            xyz=(RAIL_X + (SHOE_CHANNEL_W / 2.0) + (SHOE_SIDE_T / 2.0), 0.0, 0.0)
        ),
        material="saddle_gray",
        name="right_outboard_wall",
    )
    saddle.visual(
        Box((SHOE_OUTER_W, SHOE_END_T, SADDLE_H)),
        origin=Origin(
            xyz=(RAIL_X, (SHOE_CHANNEL_D / 2.0) + (SHOE_END_T / 2.0), 0.0)
        ),
        material="saddle_gray",
        name="right_front_bridge",
    )
    saddle.visual(
        Box((SHOE_OUTER_W, SHOE_END_T, SADDLE_H)),
        origin=Origin(
            xyz=(RAIL_X, -(SHOE_CHANNEL_D / 2.0) - (SHOE_END_T / 2.0), 0.0)
        ),
        material="saddle_gray",
        name="right_rear_bridge",
    )
    saddle.visual(
        Box((CROSSHEAD_W, CROSSHEAD_D, CROSSHEAD_H)),
        origin=Origin(
            xyz=(0.0, (SHOE_OUTER_D / 2.0) + (CROSSHEAD_D / 2.0), 0.0)
        ),
        material="saddle_gray",
        name="crosshead",
    )
    saddle.visual(
        Box((WORK_FACE_W, WORK_FACE_D, WORK_FACE_H)),
        origin=Origin(xyz=(0.0, WORK_FACE_Y, 0.0)),
        material="face_blue",
        name="work_face",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((WORK_FACE_W + 0.08, WORK_FACE_Y + WORK_FACE_D, SADDLE_H)),
        mass=6.5,
        origin=Origin(xyz=(0.0, WORK_FACE_Y / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_saddle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle,
        origin=Origin(xyz=(0.0, RAIL_Y, HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.18,
            lower=0.0,
            upper=TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    saddle = object_model.get_part("saddle")
    stage = object_model.get_articulation("frame_to_saddle")

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

    ctx.expect_contact(
        saddle,
        frame,
        elem_a="left_inboard_wall",
        elem_b="left_guide",
        name="left_saddle_contacts_left_guide",
    )
    ctx.expect_contact(
        saddle,
        frame,
        elem_a="left_outboard_wall",
        elem_b="left_guide",
        name="left_saddle_wraps_both_sides_of_left_guide",
    )
    ctx.expect_contact(
        saddle,
        frame,
        elem_a="right_inboard_wall",
        elem_b="right_guide",
        name="right_saddle_contacts_right_guide",
    )
    ctx.expect_contact(
        saddle,
        frame,
        elem_a="right_outboard_wall",
        elem_b="right_guide",
        name="right_saddle_wraps_both_sides_of_right_guide",
    )
    ctx.expect_overlap(
        saddle,
        frame,
        axes="z",
        elem_a="left_inboard_wall",
        elem_b="left_guide",
        min_overlap=0.16,
        name="left_guide_has_clear_vertical_overlap",
    )
    ctx.expect_overlap(
        saddle,
        frame,
        axes="z",
        elem_a="right_inboard_wall",
        elem_b="right_guide",
        min_overlap=0.16,
        name="right_guide_has_clear_vertical_overlap",
    )
    ctx.expect_gap(
        saddle,
        frame,
        axis="z",
        positive_elem="work_face",
        negative_elem="base_foot",
        min_gap=0.045,
        name="work_face_clears_base_in_home_pose",
    )

    rest_position = ctx.part_world_position(saddle)
    upper_position = None
    upper_limit = stage.motion_limits.upper if stage.motion_limits is not None else None
    with ctx.pose({stage: TRAVEL}):
        upper_position = ctx.part_world_position(saddle)
        ctx.expect_contact(
            saddle,
            frame,
            elem_a="left_inboard_wall",
            elem_b="left_guide",
            name="left_guide_stays_engaged_at_top_pose",
        )
        ctx.expect_contact(
            saddle,
            frame,
            elem_a="right_inboard_wall",
            elem_b="right_guide",
            name="right_guide_stays_engaged_at_top_pose",
        )
        ctx.expect_gap(
            frame,
            saddle,
            axis="z",
            positive_elem="top_beam",
            negative_elem="work_face",
            min_gap=0.14,
            name="work_face_clears_top_beam_at_top_pose",
        )

    ctx.check(
        "prismatic_stage_moves_upward",
        rest_position is not None
        and upper_position is not None
        and upper_limit is not None
        and upper_position[2] > rest_position[2] + 0.70,
        details=(
            f"rest_z={None if rest_position is None else round(rest_position[2], 3)}, "
            f"upper_z={None if upper_position is None else round(upper_position[2], 3)}, "
            f"upper_limit={upper_limit}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
