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


FRAME_W = 0.34
FRAME_H = 0.62
WALL_T = 0.012

GUIDE_X = 0.105
GUIDE_D = 0.014
GUIDE_W = 0.024
GUIDE_Y = 0.028
GUIDE_Z0 = 0.06
GUIDE_LEN = 0.50

SUPPORT_W = 0.29
SUPPORT_D = 0.026
SUPPORT_H = 0.036
SUPPORT_Y = WALL_T + (SUPPORT_D / 2.0)
BOTTOM_SUPPORT_Z = 0.07
TOP_SUPPORT_Z = 0.55

CARRIAGE_Z0 = 0.21
SLIDE_TRAVEL = 0.22
CHEEK_W = 0.016
SHOE_H = 0.18
SHOE_D = 0.040
SHOE_Y = 0.0
CHEEK_OFFSET_X = (GUIDE_W / 2.0) + (CHEEK_W / 2.0)
FRONT_WALL_W = GUIDE_W + (2.0 * CHEEK_W)
FRONT_WALL_T = 0.020
FRONT_WALL_Y = (GUIDE_D / 2.0) + (FRONT_WALL_T / 2.0)
PLATEN_W = 0.26
PLATEN_H = 0.30
PLATEN_T = 0.016
PLATEN_Y = 0.060
BRIDGE_W = 0.22
BRIDGE_H = 0.22
BRIDGE_D = 0.040
BRIDGE_Y = 0.047


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_carriage")
    model.material("frame_finish", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("carriage_finish", rgba=(0.70, 0.72, 0.75, 1.0))

    back_frame = model.part("back_frame")
    back_frame.visual(
        Box((FRAME_W, WALL_T, FRAME_H)),
        origin=Origin(xyz=(0.0, WALL_T / 2.0, FRAME_H / 2.0)),
        material="frame_finish",
        name="wall_plate",
    )
    back_frame.visual(
        Box((SUPPORT_W, SUPPORT_D, SUPPORT_H)),
        origin=Origin(xyz=(0.0, SUPPORT_Y, BOTTOM_SUPPORT_Z)),
        material="frame_finish",
        name="bottom_support",
    )
    back_frame.visual(
        Box((SUPPORT_W, SUPPORT_D, SUPPORT_H)),
        origin=Origin(xyz=(0.0, SUPPORT_Y, TOP_SUPPORT_Z)),
        material="frame_finish",
        name="top_support",
    )
    back_frame.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_LEN)),
        origin=Origin(xyz=(-GUIDE_X, GUIDE_Y, GUIDE_Z0 + (GUIDE_LEN / 2.0))),
        material="frame_finish",
        name="left_guide",
    )
    back_frame.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_LEN)),
        origin=Origin(xyz=(GUIDE_X, GUIDE_Y, GUIDE_Z0 + (GUIDE_LEN / 2.0))),
        material="frame_finish",
        name="right_guide",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CHEEK_W, SHOE_D, SHOE_H)),
        origin=Origin(xyz=(-GUIDE_X - CHEEK_OFFSET_X, SHOE_Y, 0.0)),
        material="carriage_finish",
        name="left_outer_cheek",
    )
    carriage.visual(
        Box((CHEEK_W, SHOE_D, SHOE_H)),
        origin=Origin(xyz=(-GUIDE_X + CHEEK_OFFSET_X, SHOE_Y, 0.0)),
        material="carriage_finish",
        name="left_inner_cheek",
    )
    carriage.visual(
        Box((FRONT_WALL_W, FRONT_WALL_T, SHOE_H)),
        origin=Origin(xyz=(-GUIDE_X, FRONT_WALL_Y, 0.0)),
        material="carriage_finish",
        name="left_front_wall",
    )
    carriage.visual(
        Box((CHEEK_W, SHOE_D, SHOE_H)),
        origin=Origin(xyz=(GUIDE_X - CHEEK_OFFSET_X, SHOE_Y, 0.0)),
        material="carriage_finish",
        name="right_inner_cheek",
    )
    carriage.visual(
        Box((CHEEK_W, SHOE_D, SHOE_H)),
        origin=Origin(xyz=(GUIDE_X + CHEEK_OFFSET_X, SHOE_Y, 0.0)),
        material="carriage_finish",
        name="right_outer_cheek",
    )
    carriage.visual(
        Box((FRONT_WALL_W, FRONT_WALL_T, SHOE_H)),
        origin=Origin(xyz=(GUIDE_X, FRONT_WALL_Y, 0.0)),
        material="carriage_finish",
        name="right_front_wall",
    )
    carriage.visual(
        Box((BRIDGE_W, BRIDGE_D, BRIDGE_H)),
        origin=Origin(xyz=(0.0, BRIDGE_Y, 0.0)),
        material="carriage_finish",
        name="bridge_body",
    )
    carriage.visual(
        Box((PLATEN_W, PLATEN_T, PLATEN_H)),
        origin=Origin(xyz=(0.0, PLATEN_Y, 0.0)),
        material="carriage_finish",
        name="front_platen",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=back_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, GUIDE_Y, CARRIAGE_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.22,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_frame = object_model.get_part("back_frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")

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

    limits = slide.motion_limits
    lower = 0.0 if limits is None or limits.lower is None else limits.lower
    upper = 0.0 if limits is None or limits.upper is None else limits.upper

    ctx.check(
        "parts present",
        back_frame is not None and carriage is not None,
        details="Expected back_frame and carriage parts.",
    )
    ctx.check(
        "vertical prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"Expected PRISMATIC axis +Z, got type={slide.articulation_type} axis={slide.axis}.",
    )
    ctx.check(
        "slide travel limits",
        abs(lower - 0.0) < 1e-9 and abs(upper - SLIDE_TRAVEL) < 1e-9,
        details=f"Expected travel [0, {SLIDE_TRAVEL}], got [{lower}, {upper}].",
    )

    with ctx.pose({slide: lower}):
        ctx.expect_contact(
            carriage,
            back_frame,
            contact_tol=0.0015,
            name="carriage supported on guides at bottom stroke",
        )
        ctx.expect_within(
            carriage,
            back_frame,
            axes="x",
            margin=0.02,
            name="carriage stays between frame sides",
        )

    with ctx.pose({slide: upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps at top stroke")

    with ctx.pose({slide: lower}):
        low_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: upper}):
        high_pos = ctx.part_world_position(carriage)

    moved_straight_up = (
        low_pos is not None
        and high_pos is not None
        and abs(high_pos[0] - low_pos[0]) < 1e-6
        and abs(high_pos[1] - low_pos[1]) < 1e-6
        and high_pos[2] > low_pos[2] + 0.20
    )
    ctx.check(
        "positive stroke raises platen vertically",
        moved_straight_up,
        details=f"Expected pure +Z translation, got low={low_pos} high={high_pos}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
