from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


LEG_CENTER_X = 0.78
FOOT_SIZE = (0.24, 0.92, 0.08)
BASE_RAIL_SIZE = (1.44, 0.12, 0.06)
UPRIGHT_SIZE = (0.14, 0.16, 1.28)
BEAM_SIZE = (1.72, 0.18, 0.16)
RAIL_SIZE = (1.56, 0.03, 0.014)
BEAM_CENTER_Z = FOOT_SIZE[2] + UPRIGHT_SIZE[2] + (BEAM_SIZE[2] / 2.0)
BEAM_TOP_Z = BEAM_CENTER_Z + (BEAM_SIZE[2] / 2.0)

BEAM_CARRIAGE_SIZE_X = 0.28
BEAM_TRAVEL = 0.50
SLIDE_TRAVEL = 0.38


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="beam_rider_portal_gantry")

    model.material("frame_gray", rgba=(0.41, 0.44, 0.48, 1.0))
    model.material("carriage_gray", rgba=(0.79, 0.80, 0.82, 1.0))
    model.material("slide_dark", rgba=(0.23, 0.24, 0.27, 1.0))

    frame = model.part("portal_frame")
    frame.visual(
        Box(FOOT_SIZE),
        origin=Origin(xyz=(-LEG_CENTER_X, 0.0, FOOT_SIZE[2] / 2.0)),
        material="frame_gray",
        name="left_foot",
    )
    frame.visual(
        Box(FOOT_SIZE),
        origin=Origin(xyz=(LEG_CENTER_X, 0.0, FOOT_SIZE[2] / 2.0)),
        material="frame_gray",
        name="right_foot",
    )
    frame.visual(
        Box(BASE_RAIL_SIZE),
        origin=Origin(xyz=(0.0, 0.30, BASE_RAIL_SIZE[2] / 2.0)),
        material="frame_gray",
        name="front_tie",
    )
    frame.visual(
        Box(BASE_RAIL_SIZE),
        origin=Origin(xyz=(0.0, -0.30, BASE_RAIL_SIZE[2] / 2.0)),
        material="frame_gray",
        name="rear_tie",
    )
    frame.visual(
        Box(UPRIGHT_SIZE),
        origin=Origin(xyz=(-LEG_CENTER_X, 0.0, FOOT_SIZE[2] + (UPRIGHT_SIZE[2] / 2.0))),
        material="frame_gray",
        name="left_upright",
    )
    frame.visual(
        Box(UPRIGHT_SIZE),
        origin=Origin(xyz=(LEG_CENTER_X, 0.0, FOOT_SIZE[2] + (UPRIGHT_SIZE[2] / 2.0))),
        material="frame_gray",
        name="right_upright",
    )
    frame.visual(
        Box(BEAM_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BEAM_CENTER_Z)),
        material="frame_gray",
        name="cross_beam",
    )
    frame.visual(
        Box((0.14, 0.18, 0.34)),
        origin=Origin(xyz=(-0.66, 0.0, 1.19)),
        material="frame_gray",
        name="left_brace",
    )
    frame.visual(
        Box((0.14, 0.18, 0.34)),
        origin=Origin(xyz=(0.66, 0.0, 1.19)),
        material="frame_gray",
        name="right_brace",
    )

    beam_carriage = model.part("beam_carriage")
    beam_carriage.visual(
        Box((0.34, 0.24, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="carriage_gray",
        name="carriage_pad",
    )
    beam_carriage.visual(
        Box((0.34, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.115, 0.030)),
        material="carriage_gray",
        name="front_guide",
    )
    beam_carriage.visual(
        Box((0.34, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, -0.115, 0.030)),
        material="carriage_gray",
        name="rear_guide",
    )
    beam_carriage.visual(
        Box((0.06, 0.03, 0.274)),
        origin=Origin(xyz=(0.0, 0.115, -0.113)),
        material="carriage_gray",
        name="front_side_plate",
    )
    beam_carriage.visual(
        Box((0.06, 0.03, 0.274)),
        origin=Origin(xyz=(0.0, -0.115, -0.113)),
        material="carriage_gray",
        name="rear_side_plate",
    )
    beam_carriage.visual(
        Box((0.16, 0.22, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material="carriage_gray",
        name="slide_mount",
    )

    vertical_slide = model.part("vertical_slide")
    vertical_slide.visual(
        Box((0.14, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material="slide_dark",
        name="top_plate",
    )
    vertical_slide.visual(
        Box((0.10, 0.08, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material="slide_dark",
        name="slide_body",
    )
    vertical_slide.visual(
        Box((0.11, 0.09, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.23)),
        material="slide_dark",
        name="tool_flange",
    )
    vertical_slide.visual(
        Cylinder(radius=0.03, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material="slide_dark",
        name="tool_nose",
    )

    model.articulation(
        "frame_to_beam_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=beam_carriage,
        origin=Origin(xyz=(0.0, 0.0, BEAM_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.45,
            lower=-BEAM_TRAVEL,
            upper=BEAM_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_vertical_slide",
        ArticulationType.PRISMATIC,
        parent=beam_carriage,
        child=vertical_slide,
        origin=Origin(xyz=(0.0, 0.0, -0.25)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.30,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    portal_frame = object_model.get_part("portal_frame")
    beam_carriage = object_model.get_part("beam_carriage")
    vertical_slide = object_model.get_part("vertical_slide")
    x_slide = object_model.get_articulation("frame_to_beam_carriage")
    z_slide = object_model.get_articulation("carriage_to_vertical_slide")

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

    x_limits = x_slide.motion_limits
    z_limits = z_slide.motion_limits

    ctx.check(
        "beam_carriage_joint_is_horizontal_prismatic",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and x_slide.axis == (1.0, 0.0, 0.0)
        and x_limits is not None
        and x_limits.lower is not None
        and x_limits.upper is not None
        and (x_limits.upper - x_limits.lower) >= 0.95,
        f"type={x_slide.articulation_type}, axis={x_slide.axis}, limits={x_limits}",
    )
    ctx.check(
        "vertical_slide_joint_is_downward_prismatic",
        z_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.axis == (0.0, 0.0, -1.0)
        and z_limits is not None
        and z_limits.lower is not None
        and z_limits.upper is not None
        and (z_limits.upper - z_limits.lower) >= 0.35,
        f"type={z_slide.articulation_type}, axis={z_slide.axis}, limits={z_limits}",
    )

    ctx.expect_origin_distance(
        beam_carriage,
        portal_frame,
        axes="y",
        max_dist=0.001,
        name="beam_carriage_stays_centered_front_to_back",
    )
    ctx.expect_origin_distance(
        vertical_slide,
        beam_carriage,
        axes="xy",
        max_dist=0.001,
        name="vertical_slide_hangs_directly_below_carriage_at_home",
    )
    ctx.expect_origin_gap(
        beam_carriage,
        vertical_slide,
        axis="z",
        min_gap=0.22,
        max_gap=0.25,
        name="vertical_slide_starts_compact_beneath_carriage",
    )

    with ctx.pose({x_slide: x_limits.lower}):
        left_pos = ctx.part_world_position(beam_carriage)
    with ctx.pose({x_slide: x_limits.upper}):
        right_pos = ctx.part_world_position(beam_carriage)
    ctx.check(
        "beam_carriage_translates_along_x_without_vertical_drift",
        left_pos is not None
        and right_pos is not None
        and (right_pos[0] - left_pos[0]) >= 0.95
        and abs(right_pos[1] - left_pos[1]) <= 1e-5
        and abs(right_pos[2] - left_pos[2]) <= 1e-5,
        f"left={left_pos}, right={right_pos}",
    )

    with ctx.pose({z_slide: z_limits.lower}):
        retracted_pos = ctx.part_world_position(vertical_slide)
    with ctx.pose({z_slide: z_limits.upper}):
        extended_pos = ctx.part_world_position(vertical_slide)
    ctx.check(
        "vertical_slide_translates_downward_without_lateral_drift",
        retracted_pos is not None
        and extended_pos is not None
        and (retracted_pos[2] - extended_pos[2]) >= 0.35
        and abs(retracted_pos[0] - extended_pos[0]) <= 1e-5
        and abs(retracted_pos[1] - extended_pos[1]) <= 1e-5,
        f"retracted={retracted_pos}, extended={extended_pos}",
    )

    with ctx.pose({x_slide: x_limits.upper, z_slide: z_limits.upper}):
        ctx.expect_origin_distance(
            vertical_slide,
            beam_carriage,
            axes="xy",
            max_dist=0.001,
            name="vertical_slide_stays_under_carriage_at_full_extension",
        )
        ctx.expect_origin_gap(
            beam_carriage,
            vertical_slide,
            axis="z",
            min_gap=0.60,
            max_gap=0.63,
            name="vertical_slide_reaches_full_vertical_stroke",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_full_reach_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
