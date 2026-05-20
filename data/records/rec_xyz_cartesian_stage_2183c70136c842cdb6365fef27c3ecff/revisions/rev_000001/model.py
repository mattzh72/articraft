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


FRAME_RAIL_Y = 0.07
FRAME_BACK_THICKNESS = 0.014
FRAME_RAIL_THICKNESS = 0.012
FRAME_RAIL_CENTER_X = FRAME_BACK_THICKNESS + (FRAME_RAIL_THICKNESS / 2.0)
FRAME_RAIL_FRONT_X = FRAME_BACK_THICKNESS + FRAME_RAIL_THICKNESS
FRAME_TO_FIRST_ORIGIN = (FRAME_RAIL_FRONT_X, 0.0, 0.34)

FIRST_BEAM_CENTER_X = 0.11
FIRST_TO_SECOND_ORIGIN = (0.07, -0.37, -0.05)

SECOND_TO_THIRD_ORIGIN = (0.02, 0.03, 0.27)

FIRST_TRAVEL_Z = 0.62
SECOND_TRAVEL_Y = 0.44
THIRD_TRAVEL_X = 0.15


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_xyz_positioning_module")

    model.material("frame_charcoal", rgba=(0.24, 0.26, 0.28, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("stage_blue", rgba=(0.24, 0.39, 0.63, 1.0))

    frame = model.part("side_frame")
    _add_box(frame, (FRAME_BACK_THICKNESS, 0.34, 1.18), (FRAME_BACK_THICKNESS / 2.0, 0.0, 0.59), "frame_charcoal", "back_plate")
    _add_box(frame, (0.09, 0.24, 0.06), (0.045, 0.0, 0.03), "frame_charcoal", "base_block")
    _add_box(frame, (0.07, 0.18, 0.06), (0.035, 0.0, 1.15), "frame_charcoal", "head_block")
    for rail_index, rail_y in enumerate((-FRAME_RAIL_Y, FRAME_RAIL_Y), start=1):
        _add_box(
            frame,
            (FRAME_RAIL_THICKNESS, 0.032, 0.84),
            (FRAME_RAIL_CENTER_X, rail_y, 0.60),
            "machined_aluminum",
            f"vertical_rail_{rail_index}",
        )

    first = model.part("first_carriage")
    _add_box(first, (0.026, 0.22, 0.32), (0.013, 0.0, 0.0), "machined_aluminum", "z_slide_plate")
    _add_box(first, (0.05, 0.18, 0.22), (0.051, 0.0, 0.0), "machined_aluminum", "beam_support")
    _add_box(first, (0.04, 0.74, 0.08), (0.09, 0.0, -0.09), "machined_aluminum", "y_beam")
    _add_box(first, (0.04, 0.12, 0.10), (0.07, 0.0, 0.11), "carriage_gray", "motor_cap")

    second = model.part("second_carriage")
    _add_box(second, (0.04, 0.12, 0.04), (0.03, 0.06, 0.02), "carriage_gray", "y_shoe")
    _add_box(second, (0.06, 0.10, 0.18), (0.05, 0.06, 0.11), "carriage_gray", "z_upright")
    _add_box(second, (0.08, 0.08, 0.06), (0.10, 0.06, 0.22), "carriage_gray", "rail_mount")
    _add_box(second, (0.22, 0.06, 0.04), (0.19, 0.06, 0.25), "carriage_gray", "x_rail")

    third = model.part("third_stage")
    _add_box(third, (0.08, 0.06, 0.03), (0.04, 0.03, 0.015), "stage_blue", "x_shoe")
    _add_box(third, (0.08, 0.08, 0.24), (0.06, 0.04, 0.15), "stage_blue", "vertical_column")
    _add_box(third, (0.05, 0.08, 0.04), (0.085, 0.04, 0.29), "stage_blue", "top_cap")
    _add_box(third, (0.03, 0.10, 0.10), (0.105, 0.04, 0.11), "machined_aluminum", "tool_plate")
    _add_box(third, (0.05, 0.08, 0.06), (0.13, 0.04, 0.05), "machined_aluminum", "tool_nose")

    model.articulation(
        "frame_to_first",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=first,
        origin=Origin(xyz=FRAME_TO_FIRST_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.30,
            lower=0.0,
            upper=FIRST_TRAVEL_Z,
        ),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.PRISMATIC,
        parent=first,
        child=second,
        origin=Origin(xyz=FIRST_TO_SECOND_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.35,
            lower=0.0,
            upper=SECOND_TRAVEL_Y,
        ),
    )
    model.articulation(
        "second_to_third",
        ArticulationType.PRISMATIC,
        parent=second,
        child=third,
        origin=Origin(xyz=SECOND_TO_THIRD_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.22,
            lower=0.0,
            upper=THIRD_TRAVEL_X,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("side_frame")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_carriage")
    third = object_model.get_part("third_stage")

    first_slide = object_model.get_articulation("frame_to_first")
    second_slide = object_model.get_articulation("first_to_second")
    third_slide = object_model.get_articulation("second_to_third")

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

    ctx.expect_contact(first, frame, name="first_carriage_guided_on_frame_at_home")
    ctx.expect_contact(second, first, name="second_carriage_guided_on_first_at_home")
    ctx.expect_contact(third, second, name="third_stage_guided_on_second_at_home")

    with ctx.pose({first_slide: first_slide.motion_limits.upper}):
        ctx.expect_contact(first, frame, name="first_carriage_stays_guided_at_top")
        ctx.expect_origin_gap(first, frame, axis="z", min_gap=0.90, name="first_axis_moves_upward")

    with ctx.pose({second_slide: second_slide.motion_limits.upper}):
        ctx.expect_contact(second, first, name="second_carriage_stays_guided_at_far_travel")
        ctx.expect_origin_gap(second, frame, axis="y", min_gap=0.06, name="second_axis_moves_along_beam")

    with ctx.pose({third_slide: third_slide.motion_limits.upper}):
        ctx.expect_contact(third, second, name="third_stage_stays_guided_extended")
        ctx.expect_origin_gap(third, second, axis="x", min_gap=0.12, name="third_axis_moves_forward")

    with ctx.pose(
        {
            first_slide: first_slide.motion_limits.upper,
            second_slide: second_slide.motion_limits.upper,
            third_slide: third_slide.motion_limits.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_maximum_reach")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
