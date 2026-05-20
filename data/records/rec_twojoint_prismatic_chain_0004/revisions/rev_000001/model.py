from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_LEN = 0.260
FRAME_WIDTH = 0.070
FRAME_BASE_THK = 0.004
FRAME_RAIL_THK = 0.007
FRAME_RAIL_H = 0.016

STAGE1_LEN = 0.180
STAGE1_BODY_W = 0.054
STAGE1_BODY_H = 0.015
STAGE1_DECK_W = 0.084
STAGE1_DECK_H = 0.006
STAGE1_TOP_RAIL_W = 0.012
STAGE1_TOP_RAIL_H = 0.010
STAGE1_TOP_RAIL_Y = 0.023

STAGE2_LEN = 0.120
STAGE2_BODY_W = 0.032
STAGE2_BODY_H = 0.009
STAGE2_PLATE_W = 0.060
STAGE2_PLATE_H = 0.006

TRAVEL = 0.100
SLIDING_CLEARANCE = 0.001


def _box_inertia(part, size: tuple[float, float, float], mass: float, xyz: tuple[float, float, float]) -> None:
    part.inertial = Inertial.from_geometry(
        Box(size),
        mass=mass,
        origin=Origin(xyz=xyz),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_prismatic_chain", assets=ASSETS)

    frame_color = model.material("frame_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    stage1_color = model.material("stage1_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    stage2_color = model.material("stage2_bluegray", rgba=(0.48, 0.56, 0.66, 1.0))

    frame = model.part("outer_frame")
    frame.visual(
        Box((FRAME_LEN, FRAME_WIDTH, FRAME_BASE_THK)),
        origin=Origin(xyz=(FRAME_LEN / 2.0, 0.0, FRAME_BASE_THK / 2.0)),
        material=frame_color,
        name="base",
    )
    rail_z = FRAME_BASE_THK + FRAME_RAIL_H / 2.0
    rail_y = FRAME_WIDTH / 2.0 - FRAME_RAIL_THK / 2.0
    frame.visual(
        Box((FRAME_LEN, FRAME_RAIL_THK, FRAME_RAIL_H)),
        origin=Origin(xyz=(FRAME_LEN / 2.0, rail_y, rail_z)),
        material=frame_color,
        name="left_rail",
    )
    frame.visual(
        Box((FRAME_LEN, FRAME_RAIL_THK, FRAME_RAIL_H)),
        origin=Origin(xyz=(FRAME_LEN / 2.0, -rail_y, rail_z)),
        material=frame_color,
        name="right_rail",
    )
    _box_inertia(
        frame,
        (FRAME_LEN, FRAME_WIDTH, FRAME_BASE_THK + FRAME_RAIL_H),
        mass=2.4,
        xyz=(FRAME_LEN / 2.0, 0.0, (FRAME_BASE_THK + FRAME_RAIL_H) / 2.0),
    )

    stage1 = model.part("stage_1")
    stage1.visual(
        Box((STAGE1_LEN, STAGE1_BODY_W, STAGE1_BODY_H)),
        origin=Origin(xyz=(STAGE1_LEN / 2.0, 0.0, STAGE1_BODY_H / 2.0)),
        material=stage1_color,
        name="body",
    )
    stage1.visual(
        Box((STAGE1_LEN, STAGE1_DECK_W, STAGE1_DECK_H)),
        origin=Origin(
            xyz=(STAGE1_LEN / 2.0, 0.0, STAGE1_BODY_H + STAGE1_DECK_H / 2.0),
        ),
        material=stage1_color,
        name="deck",
    )
    stage1.visual(
        Box((STAGE1_LEN, STAGE1_TOP_RAIL_W, STAGE1_TOP_RAIL_H)),
        origin=Origin(
            xyz=(
                STAGE1_LEN / 2.0,
                STAGE1_TOP_RAIL_Y,
                STAGE1_BODY_H + STAGE1_DECK_H + STAGE1_TOP_RAIL_H / 2.0,
            ),
        ),
        material=stage1_color,
        name="top_left_rail",
    )
    stage1.visual(
        Box((STAGE1_LEN, STAGE1_TOP_RAIL_W, STAGE1_TOP_RAIL_H)),
        origin=Origin(
            xyz=(
                STAGE1_LEN / 2.0,
                -STAGE1_TOP_RAIL_Y,
                STAGE1_BODY_H + STAGE1_DECK_H + STAGE1_TOP_RAIL_H / 2.0,
            ),
        ),
        material=stage1_color,
        name="top_right_rail",
    )
    _box_inertia(
        stage1,
        (STAGE1_LEN, STAGE1_DECK_W, STAGE1_BODY_H + STAGE1_DECK_H + STAGE1_TOP_RAIL_H),
        mass=1.1,
        xyz=(STAGE1_LEN / 2.0, 0.0, (STAGE1_BODY_H + STAGE1_DECK_H + STAGE1_TOP_RAIL_H) / 2.0),
    )

    stage2 = model.part("stage_2")
    stage2.visual(
        Box((STAGE2_LEN, STAGE2_BODY_W, STAGE2_BODY_H)),
        origin=Origin(xyz=(STAGE2_LEN / 2.0, 0.0, STAGE2_BODY_H / 2.0)),
        material=stage2_color,
        name="body",
    )
    stage2.visual(
        Box((STAGE2_LEN, STAGE2_PLATE_W, STAGE2_PLATE_H)),
        origin=Origin(
            xyz=(STAGE2_LEN / 2.0, 0.0, STAGE2_BODY_H + STAGE2_PLATE_H / 2.0),
        ),
        material=stage2_color,
        name="plate",
    )
    _box_inertia(
        stage2,
        (STAGE2_LEN, STAGE2_PLATE_W, STAGE2_BODY_H + STAGE2_PLATE_H),
        mass=0.55,
        xyz=(STAGE2_LEN / 2.0, 0.0, (STAGE2_BODY_H + STAGE2_PLATE_H) / 2.0),
    )

    model.articulation(
        "frame_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, FRAME_BASE_THK + SLIDING_CLEARANCE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=TRAVEL,
        ),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                STAGE1_BODY_H + STAGE1_DECK_H + SLIDING_CLEARANCE,
            ),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("outer_frame")
    stage1 = object_model.get_part("stage_1")
    stage2 = object_model.get_part("stage_2")
    joint1 = object_model.get_articulation("frame_to_stage_1")
    joint2 = object_model.get_articulation("stage_1_to_stage_2")

    frame_base = frame.get_visual("base")
    frame_left_rail = frame.get_visual("left_rail")
    frame_right_rail = frame.get_visual("right_rail")
    stage1_body = stage1.get_visual("body")
    stage1_deck = stage1.get_visual("deck")
    stage1_top_left = stage1.get_visual("top_left_rail")
    stage1_top_right = stage1.get_visual("top_right_rail")
    stage2_body = stage2.get_visual("body")
    stage2_plate = stage2.get_visual("plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.check("outer_frame_present", frame is not None, "missing outer_frame part")
    ctx.check("stage_1_present", stage1 is not None, "missing stage_1 part")
    ctx.check("stage_2_present", stage2 is not None, "missing stage_2 part")

    ctx.check(
        "joint_axes_are_collinear",
        tuple(joint1.axis) == (1.0, 0.0, 0.0) and tuple(joint2.axis) == (1.0, 0.0, 0.0),
        f"expected both prismatic axes along +x, got {joint1.axis} and {joint2.axis}",
    )
    ctx.check(
        "joint_travels_are_100mm",
        abs(joint1.motion_limits.upper - TRAVEL) < 1e-9
        and abs(joint2.motion_limits.upper - TRAVEL) < 1e-9,
        "both prismatic stages should provide 100 mm of travel",
    )

    ctx.expect_gap(
        stage1,
        frame,
        axis="z",
        positive_elem=stage1_body,
        negative_elem=frame_base,
        min_gap=SLIDING_CLEARANCE,
        max_gap=SLIDING_CLEARANCE + 1e-5,
        name="stage_1_body_clears_frame_base",
    )
    ctx.expect_within(
        stage1,
        frame,
        axes="y",
        inner_elem=stage1_body,
        outer_elem=frame_base,
        margin=0.0,
        name="stage_1_body_stays_between_frame_rails",
    )
    ctx.expect_contact(
        stage1,
        frame,
        elem_a=stage1_deck,
        elem_b=frame_left_rail,
        name="stage_1_deck_contacts_left_frame_rail",
    )
    ctx.expect_contact(
        stage1,
        frame,
        elem_a=stage1_deck,
        elem_b=frame_right_rail,
        name="stage_1_deck_contacts_right_frame_rail",
    )
    ctx.expect_overlap(
        stage1,
        frame,
        axes="x",
        elem_a=stage1_deck,
        elem_b=frame_base,
        min_overlap=0.16,
        name="stage_1_has_substantial_x_overlap_with_frame",
    )

    ctx.expect_gap(
        stage2,
        stage1,
        axis="z",
        positive_elem=stage2_body,
        negative_elem=stage1_deck,
        min_gap=SLIDING_CLEARANCE,
        max_gap=SLIDING_CLEARANCE + 1e-5,
        name="stage_2_body_clears_stage_1_deck",
    )
    ctx.expect_within(
        stage2,
        stage1,
        axes="y",
        inner_elem=stage2_body,
        outer_elem=stage1_deck,
        margin=0.0,
        name="stage_2_body_stays_between_stage_1_guides",
    )
    ctx.expect_contact(
        stage2,
        stage1,
        elem_a=stage2_plate,
        elem_b=stage1_top_left,
        name="stage_2_plate_contacts_left_stage_1_rail",
    )
    ctx.expect_contact(
        stage2,
        stage1,
        elem_a=stage2_plate,
        elem_b=stage1_top_right,
        name="stage_2_plate_contacts_right_stage_1_rail",
    )
    ctx.expect_overlap(
        stage2,
        stage1,
        axes="x",
        elem_a=stage2_plate,
        elem_b=stage1_deck,
        min_overlap=0.12,
        name="stage_2_has_substantial_x_overlap_with_stage_1",
    )

    stage1_rest_x = ctx.part_world_position(stage1)[0]
    stage2_rest_x = ctx.part_world_position(stage2)[0]
    with ctx.pose({joint1: TRAVEL}):
        stage1_extended_x = ctx.part_world_position(stage1)[0]
        ctx.check(
            "stage_1_translates_100mm",
            abs(stage1_extended_x - stage1_rest_x - TRAVEL) < 1e-9,
            f"expected stage_1 x shift of {TRAVEL}, got {stage1_extended_x - stage1_rest_x}",
        )
        ctx.expect_contact(
            stage1,
            frame,
            elem_a=stage1_deck,
            elem_b=frame_left_rail,
            name="stage_1_remains_supported_at_full_extension_left",
        )
        ctx.expect_overlap(
            stage1,
            frame,
            axes="x",
            elem_a=stage1_deck,
            elem_b=frame_base,
            min_overlap=0.08,
            name="stage_1_retains_x_overlap_at_full_extension",
        )

    with ctx.pose({joint1: TRAVEL, joint2: TRAVEL}):
        stage2_extended_x = ctx.part_world_position(stage2)[0]
        ctx.check(
            "stage_2_total_translation_is_200mm_when_both_extend",
            abs(stage2_extended_x - stage2_rest_x - 2.0 * TRAVEL) < 1e-9,
            (
                "expected stage_2 world x shift of 0.2 m when both prismatic joints are at "
                "their upper limits"
            ),
        )
        ctx.expect_contact(
            stage2,
            stage1,
            elem_a=stage2_plate,
            elem_b=stage1_top_left,
            name="stage_2_remains_supported_at_full_extension_left",
        )
        ctx.expect_overlap(
            stage2,
            stage1,
            axes="x",
            elem_a=stage2_plate,
            elem_b=stage1_deck,
            min_overlap=0.08,
            name="stage_2_retains_x_overlap_at_full_extension",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
