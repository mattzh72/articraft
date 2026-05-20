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


BEAM_LENGTH = 1.20
BEAM_DEPTH = 0.16
BEAM_HEIGHT = 0.14
BEAM_TOP_HEIGHT = 0.08
BEAM_RAIL_DEPTH = 0.04
BEAM_RAIL_HEIGHT = 0.06
BEAM_RAIL_CENTER_Y = 0.06
BEAM_TOP_CENTER_Z = 0.03
BEAM_RAIL_CENTER_Z = -0.04

RIDER_LENGTH = 0.24
RIDER_DEPTH = 0.19
RIDER_BODY_LENGTH = 0.24
RIDER_BODY_DEPTH = 0.16
RIDER_BODY_HEIGHT = 0.07
RIDER_BODY_CENTER_Z = -0.095
RIDER_CHEEK_THICKNESS = 0.024
RIDER_CHEEK_CENTER_Y = 0.068
RIDER_CHEEK_HEIGHT = 0.15
RIDER_CHEEK_CENTER_Z = -0.025
RIDER_SHOE_LENGTH = 0.18
RIDER_SHOE_DEPTH = 0.034
RIDER_SHOE_CENTER_Y = 0.058
RIDER_SHOE_THICKNESS = 0.02
RIDER_SHOE_CENTER_Z = 0.06
RIDER_GUIDE_ROOF_LENGTH = 0.16
RIDER_GUIDE_ROOF_DEPTH = 0.12
RIDER_GUIDE_ROOF_HEIGHT = 0.03
RIDER_GUIDE_ROOF_CENTER_Z = -0.145
RIDER_GUIDE_WALL_LENGTH = 0.16
RIDER_GUIDE_WALL_THICKNESS = 0.02
RIDER_GUIDE_WALL_CENTER_Y = 0.05
RIDER_GUIDE_WALL_HEIGHT = 0.15
RIDER_GUIDE_WALL_CENTER_Z = -0.235

CARRIAGE_GUIDE_BLOCK_X = 0.10
CARRIAGE_GUIDE_BLOCK_Y = 0.08
CARRIAGE_GUIDE_BLOCK_Z = 0.12
CARRIAGE_BODY_X = 0.11
CARRIAGE_BODY_Y = 0.08
CARRIAGE_BODY_Z = 0.12
CARRIAGE_BODY_CENTER_Z = -0.11
CARRIAGE_PLATE_X = 0.14
CARRIAGE_PLATE_Y = 0.11
CARRIAGE_PLATE_Z = 0.03
CARRIAGE_PLATE_CENTER_Z = -0.185

RIDER_HOME_Z = -(BEAM_HEIGHT / 2.0 + RIDER_SHOE_CENTER_Z + RIDER_SHOE_THICKNESS / 2.0)
CARRIAGE_HOME_Z = (
    RIDER_GUIDE_ROOF_CENTER_Z
    - RIDER_GUIDE_ROOF_HEIGHT / 2.0
    - CARRIAGE_GUIDE_BLOCK_Z / 2.0
)

RIDER_TRAVEL = 0.36
CARRIAGE_TRAVEL = 0.10


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_beam_portal_axis")

    model.material("beam_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("rider_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("carriage_gray", rgba=(0.46, 0.48, 0.50, 1.0))

    top_beam = model.part("top_beam")
    top_beam.visual(
        Box((BEAM_LENGTH, BEAM_DEPTH, BEAM_TOP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BEAM_TOP_CENTER_Z)),
        material="beam_aluminum",
        name="beam_top",
    )
    top_beam.visual(
        Box((BEAM_LENGTH, BEAM_RAIL_DEPTH, BEAM_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, BEAM_RAIL_CENTER_Y, BEAM_RAIL_CENTER_Z)),
        material="beam_aluminum",
        name="left_rail",
    )
    top_beam.visual(
        Box((BEAM_LENGTH, BEAM_RAIL_DEPTH, BEAM_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -BEAM_RAIL_CENTER_Y, BEAM_RAIL_CENTER_Z)),
        material="beam_aluminum",
        name="right_rail",
    )
    top_beam.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)),
        mass=22.0,
    )

    beam_rider = model.part("beam_rider")
    beam_rider.visual(
        Box((RIDER_BODY_LENGTH, RIDER_BODY_DEPTH, RIDER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RIDER_BODY_CENTER_Z)),
        material="rider_charcoal",
        name="body",
    )
    beam_rider.visual(
        Box((RIDER_LENGTH, RIDER_CHEEK_THICKNESS, RIDER_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, RIDER_CHEEK_CENTER_Y, RIDER_CHEEK_CENTER_Z)),
        material="rider_charcoal",
        name="left_cheek",
    )
    beam_rider.visual(
        Box((RIDER_LENGTH, RIDER_CHEEK_THICKNESS, RIDER_CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, -RIDER_CHEEK_CENTER_Y, RIDER_CHEEK_CENTER_Z)),
        material="rider_charcoal",
        name="right_cheek",
    )
    beam_rider.visual(
        Box((RIDER_SHOE_LENGTH, RIDER_SHOE_DEPTH, RIDER_SHOE_THICKNESS)),
        origin=Origin(xyz=(0.0, RIDER_SHOE_CENTER_Y, RIDER_SHOE_CENTER_Z)),
        material="carriage_gray",
        name="left_shoe",
    )
    beam_rider.visual(
        Box((RIDER_SHOE_LENGTH, RIDER_SHOE_DEPTH, RIDER_SHOE_THICKNESS)),
        origin=Origin(xyz=(0.0, -RIDER_SHOE_CENTER_Y, RIDER_SHOE_CENTER_Z)),
        material="carriage_gray",
        name="right_shoe",
    )
    beam_rider.visual(
        Box((RIDER_GUIDE_ROOF_LENGTH, RIDER_GUIDE_ROOF_DEPTH, RIDER_GUIDE_ROOF_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RIDER_GUIDE_ROOF_CENTER_Z)),
        material="rider_charcoal",
        name="guide_roof",
    )
    beam_rider.visual(
        Box((RIDER_GUIDE_WALL_LENGTH, RIDER_GUIDE_WALL_THICKNESS, RIDER_GUIDE_WALL_HEIGHT)),
        origin=Origin(xyz=(0.0, RIDER_GUIDE_WALL_CENTER_Y, RIDER_GUIDE_WALL_CENTER_Z)),
        material="rider_charcoal",
        name="left_wall",
    )
    beam_rider.visual(
        Box((RIDER_GUIDE_WALL_LENGTH, RIDER_GUIDE_WALL_THICKNESS, RIDER_GUIDE_WALL_HEIGHT)),
        origin=Origin(xyz=(0.0, -RIDER_GUIDE_WALL_CENTER_Y, RIDER_GUIDE_WALL_CENTER_Z)),
        material="rider_charcoal",
        name="right_wall",
    )
    beam_rider.inertial = Inertial.from_geometry(
        Box((RIDER_LENGTH, RIDER_DEPTH, 0.38)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
    )

    vertical_carriage = model.part("vertical_carriage")
    vertical_carriage.visual(
        Box((CARRIAGE_GUIDE_BLOCK_X, CARRIAGE_GUIDE_BLOCK_Y, CARRIAGE_GUIDE_BLOCK_Z)),
        material="carriage_gray",
        name="guide_block",
    )
    vertical_carriage.visual(
        Box((CARRIAGE_BODY_X, CARRIAGE_BODY_Y, CARRIAGE_BODY_Z)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BODY_CENTER_Z)),
        material="carriage_gray",
        name="carriage_body",
    )
    vertical_carriage.visual(
        Box((CARRIAGE_PLATE_X, CARRIAGE_PLATE_Y, CARRIAGE_PLATE_Z)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_PLATE_CENTER_Z)),
        material="carriage_gray",
        name="tool_plate",
    )
    vertical_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_PLATE_X, CARRIAGE_PLATE_Y, 0.24)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
    )

    model.articulation(
        "beam_to_rider",
        ArticulationType.PRISMATIC,
        parent=top_beam,
        child=beam_rider,
        origin=Origin(xyz=(0.0, 0.0, RIDER_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=650.0,
            velocity=0.65,
            lower=-RIDER_TRAVEL,
            upper=RIDER_TRAVEL,
        ),
    )

    model.articulation(
        "rider_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam_rider,
        child=vertical_carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=0.30,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_beam = object_model.get_part("top_beam")
    beam_rider = object_model.get_part("beam_rider")
    vertical_carriage = object_model.get_part("vertical_carriage")
    beam_to_rider = object_model.get_articulation("beam_to_rider")
    rider_to_carriage = object_model.get_articulation("rider_to_carriage")

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
        "beam_rider_joint_is_prismatic",
        beam_to_rider.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type was {beam_to_rider.articulation_type}",
    )
    ctx.check(
        "carriage_joint_is_prismatic",
        rider_to_carriage.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type was {rider_to_carriage.articulation_type}",
    )
    ctx.check(
        "beam_rider_joint_axis_along_x",
        tuple(beam_to_rider.axis) == (1.0, 0.0, 0.0),
        details=f"axis was {beam_to_rider.axis}",
    )
    ctx.check(
        "carriage_joint_axis_points_down",
        tuple(rider_to_carriage.axis) == (0.0, 0.0, -1.0),
        details=f"axis was {rider_to_carriage.axis}",
    )

    ctx.expect_contact(top_beam, beam_rider, name="rider_is_supported_by_beam")
    ctx.expect_contact(beam_rider, vertical_carriage, name="carriage_is_supported_by_rider")
    ctx.expect_origin_gap(
        top_beam,
        beam_rider,
        axis="z",
        min_gap=0.13,
        max_gap=0.15,
        name="rider_hangs_below_beam",
    )
    ctx.expect_origin_gap(
        beam_rider,
        vertical_carriage,
        axis="z",
        min_gap=0.21,
        max_gap=0.23,
        name="carriage_starts_below_rider_center",
    )

    rider_home_pos = ctx.part_world_position(beam_rider)
    carriage_home_pos = ctx.part_world_position(vertical_carriage)
    if rider_home_pos is not None:
        with ctx.pose({beam_to_rider: 0.30}):
            shifted_rider_pos = ctx.part_world_position(beam_rider)
            ctx.check(
                "positive_beam_travel_moves_rider_positive_x",
                shifted_rider_pos is not None and shifted_rider_pos[0] > rider_home_pos[0] + 0.29,
                details=f"home={rider_home_pos}, shifted={shifted_rider_pos}",
            )
            ctx.expect_contact(top_beam, beam_rider, name="rider_stays_supported_when_shifted")

    if carriage_home_pos is not None:
        with ctx.pose({rider_to_carriage: 0.08}):
            shifted_carriage_pos = ctx.part_world_position(vertical_carriage)
            ctx.check(
                "positive_vertical_travel_moves_carriage_down",
                shifted_carriage_pos is not None
                and shifted_carriage_pos[2] < carriage_home_pos[2] - 0.07,
                details=f"home={carriage_home_pos}, shifted={shifted_carriage_pos}",
            )
            ctx.expect_contact(
                beam_rider,
                vertical_carriage,
                name="carriage_stays_guided_when_lowered",
            )

    with ctx.pose({beam_to_rider: 0.34, rider_to_carriage: 0.08}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_combined_travel")
        ctx.expect_contact(top_beam, beam_rider, name="rider_supported_at_combined_travel")
        ctx.expect_contact(
            beam_rider,
            vertical_carriage,
            name="carriage_guided_at_combined_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
