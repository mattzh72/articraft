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


FRAME_SKID_LENGTH = 0.76
FRAME_SKID_WIDTH = 0.08
FRAME_SKID_HEIGHT = 0.05
FRAME_SKID_OFFSET_Y = 0.10

COLUMN_WIDTH = 0.08
COLUMN_DEPTH = 0.14
COLUMN_HEIGHT = 0.58
COLUMN_CENTER_X = 0.33
COLUMN_CENTER_Z = FRAME_SKID_HEIGHT + COLUMN_HEIGHT / 2.0

TOP_BEAM_LENGTH = 0.74
TOP_BEAM_DEPTH = 0.14
TOP_BEAM_HEIGHT = 0.08
TOP_BEAM_CENTER_Z = FRAME_SKID_HEIGHT + COLUMN_HEIGHT + TOP_BEAM_HEIGHT / 2.0

LOWER_TIE_LENGTH = 0.62
LOWER_TIE_DEPTH = 0.08
LOWER_TIE_HEIGHT = 0.05
LOWER_TIE_CENTER_Y = -0.02
LOWER_TIE_CENTER_Z = 0.19

RAIL_LENGTH = 0.60
RAIL_DEPTH = 0.016
RAIL_HEIGHT = 0.014
RAIL_CENTER_Y = 0.078
UPPER_RAIL_CENTER_Z = 0.595
LOWER_RAIL_CENTER_Z = 0.535

CARRIAGE_WIDTH = 0.36
CARRIAGE_DEPTH = 0.048
CARRIAGE_HEIGHT = 0.20
CARRIAGE_JOINT_ORIGIN = (0.0, 0.086, 0.565)
CARRIAGE_TRAVEL = 0.09

SHOE_WIDTH = 0.22
SHOE_DEPTH = 0.016
SHOE_HEIGHT = 0.020
SHOE_LOCAL_CENTER_Y = 0.007
UPPER_SHOE_LOCAL_CENTER_Z = 0.030
LOWER_SHOE_LOCAL_CENTER_Z = -0.030

GUIDE_WIDTH = 0.012
GUIDE_DEPTH = 0.016
GUIDE_HEIGHT = 0.17
GUIDE_CENTER_X = 0.076
GUIDE_CENTER_Y = 0.034
GUIDE_CENTER_Z = -0.010

STAGE_WIDTH = 0.14
STAGE_DEPTH = 0.018
STAGE_HEIGHT = 0.14
STAGE_TRAVEL = 0.10
STAGE_JOINT_ORIGIN = (0.0, 0.047, -0.020)

STAGE_PLATFORM_WIDTH = 0.16
STAGE_PLATFORM_DEPTH = 0.024
STAGE_PLATFORM_HEIGHT = 0.022
STAGE_PLATFORM_CENTER_Y = 0.012
STAGE_PLATFORM_CENTER_Z = -0.100

STAGE_SHOE_WIDTH = 0.012
STAGE_SHOE_DEPTH = 0.010
STAGE_SHOE_HEIGHT = 0.15
STAGE_SHOE_CENTER_X = 0.076
STAGE_SHOE_CENTER_Y = 0.0
STAGE_SHOE_CENTER_Z = -0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_portal_lift_stage")

    model.material("frame_charcoal", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("carriage_blue", rgba=(0.22, 0.40, 0.69, 1.0))
    model.material("stage_silver", rgba=(0.84, 0.85, 0.87, 1.0))
    model.material("guide_black", rgba=(0.10, 0.11, 0.13, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((FRAME_SKID_LENGTH, FRAME_SKID_WIDTH, FRAME_SKID_HEIGHT)),
        origin=Origin(xyz=(0.0, FRAME_SKID_OFFSET_Y, FRAME_SKID_HEIGHT / 2.0)),
        material="frame_charcoal",
        name="skid_front",
    )
    frame.visual(
        Box((FRAME_SKID_LENGTH, FRAME_SKID_WIDTH, FRAME_SKID_HEIGHT)),
        origin=Origin(xyz=(0.0, -FRAME_SKID_OFFSET_Y, FRAME_SKID_HEIGHT / 2.0)),
        material="frame_charcoal",
        name="skid_rear",
    )
    frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, 0.0, COLUMN_CENTER_Z)),
        material="frame_charcoal",
        name="column_left",
    )
    frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(COLUMN_CENTER_X, 0.0, COLUMN_CENTER_Z)),
        material="frame_charcoal",
        name="column_right",
    )
    frame.visual(
        Box((TOP_BEAM_LENGTH, TOP_BEAM_DEPTH, TOP_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TOP_BEAM_CENTER_Z)),
        material="frame_charcoal",
        name="top_beam",
    )
    frame.visual(
        Box((LOWER_TIE_LENGTH, LOWER_TIE_DEPTH, LOWER_TIE_HEIGHT)),
        origin=Origin(xyz=(0.0, LOWER_TIE_CENTER_Y, LOWER_TIE_CENTER_Z)),
        material="frame_charcoal",
        name="lower_tie",
    )
    frame.visual(
        Box((0.18, 0.06, 0.18)),
        origin=Origin(xyz=(0.0, LOWER_TIE_CENTER_Y, 0.305)),
        material="frame_charcoal",
        name="service_box",
    )
    frame.visual(
        Box((RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, UPPER_RAIL_CENTER_Z)),
        material="rail_steel",
        name="rail_upper",
    )
    frame.visual(
        Box((RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, LOWER_RAIL_CENTER_Z)),
        material="rail_steel",
        name="rail_lower",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_SKID_LENGTH, 0.28, TOP_BEAM_CENTER_Z + TOP_BEAM_HEIGHT / 2.0)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, (TOP_BEAM_CENTER_Z + TOP_BEAM_HEIGHT / 2.0) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.34, 0.030, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material="carriage_blue",
        name="backplate",
    )
    carriage.visual(
        Box((0.038, 0.022, 0.060)),
        origin=Origin(xyz=(-0.136, 0.026, 0.068)),
        material="carriage_blue",
        name="left_cap",
    )
    carriage.visual(
        Box((0.038, 0.022, 0.060)),
        origin=Origin(xyz=(0.136, 0.026, 0.068)),
        material="carriage_blue",
        name="right_cap",
    )
    carriage.visual(
        Box((0.032, 0.026, 0.090)),
        origin=Origin(xyz=(-0.076, 0.030, -0.055)),
        material="carriage_blue",
        name="guide_mount_left",
    )
    carriage.visual(
        Box((0.032, 0.026, 0.090)),
        origin=Origin(xyz=(0.076, 0.030, -0.055)),
        material="carriage_blue",
        name="guide_mount_right",
    )
    carriage.visual(
        Box((0.16, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, 0.028, -0.096)),
        material="carriage_blue",
        name="bottom_bridge",
    )
    carriage.visual(
        Box((SHOE_WIDTH, SHOE_DEPTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, SHOE_LOCAL_CENTER_Y, UPPER_SHOE_LOCAL_CENTER_Z)),
        material="guide_black",
        name="shoe_upper",
    )
    carriage.visual(
        Box((SHOE_WIDTH, SHOE_DEPTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(0.0, SHOE_LOCAL_CENTER_Y, LOWER_SHOE_LOCAL_CENTER_Z)),
        material="guide_black",
        name="shoe_lower",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_CENTER_X, GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        material="guide_black",
        name="guide_left",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, GUIDE_CENTER_Y, GUIDE_CENTER_Z)),
        material="guide_black",
        name="guide_right",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, CARRIAGE_DEPTH + 0.02, CARRIAGE_HEIGHT)),
        mass=7.5,
    )

    stage = model.part("stage")
    stage.visual(
        Box((STAGE_WIDTH, STAGE_DEPTH, STAGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material="stage_silver",
        name="main_plate",
    )
    stage.visual(
        Box((0.024, 0.018, 0.150)),
        origin=Origin(xyz=(-0.050, 0.008, -0.010)),
        material="stage_silver",
        name="left_rib",
    )
    stage.visual(
        Box((0.024, 0.018, 0.150)),
        origin=Origin(xyz=(0.050, 0.008, -0.010)),
        material="stage_silver",
        name="right_rib",
    )
    stage.visual(
        Box((STAGE_SHOE_WIDTH, STAGE_SHOE_DEPTH, STAGE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(-STAGE_SHOE_CENTER_X, STAGE_SHOE_CENTER_Y, STAGE_SHOE_CENTER_Z)
        ),
        material="guide_black",
        name="left_shoe",
    )
    stage.visual(
        Box((STAGE_SHOE_WIDTH, STAGE_SHOE_DEPTH, STAGE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(STAGE_SHOE_CENTER_X, STAGE_SHOE_CENTER_Y, STAGE_SHOE_CENTER_Z)
        ),
        material="guide_black",
        name="right_shoe",
    )
    stage.visual(
        Box((STAGE_PLATFORM_WIDTH, STAGE_PLATFORM_DEPTH, STAGE_PLATFORM_HEIGHT)),
        origin=Origin(xyz=(0.0, STAGE_PLATFORM_CENTER_Y, STAGE_PLATFORM_CENTER_Z)),
        material="stage_silver",
        name="platform",
    )
    stage.visual(
        Box((0.050, 0.016, 0.058)),
        origin=Origin(xyz=(0.0, 0.010, -0.068)),
        material="stage_silver",
        name="platform_stem",
    )
    stage.inertial = Inertial.from_geometry(
        Box((STAGE_PLATFORM_WIDTH, STAGE_PLATFORM_DEPTH, STAGE_HEIGHT + 0.03)),
        mass=3.2,
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=CARRIAGE_JOINT_ORIGIN),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
            effort=650.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_to_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=stage,
        origin=Origin(xyz=STAGE_JOINT_ORIGIN),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE_TRAVEL,
            effort=320.0,
            velocity=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    stage = object_model.get_part("stage")
    frame_to_carriage = object_model.get_articulation("frame_to_carriage")
    carriage_to_stage = object_model.get_articulation("carriage_to_stage")

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
        frame,
        carriage,
        elem_a="rail_upper",
        elem_b="shoe_upper",
        name="upper_carriage_shoe_on_upper_rail",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a="rail_lower",
        elem_b="shoe_lower",
        name="lower_carriage_shoe_on_lower_rail",
    )
    ctx.expect_contact(
        stage,
        carriage,
        elem_a="left_shoe",
        elem_b="guide_left",
        name="left_stage_shoe_on_left_guide",
    )
    ctx.expect_contact(
        stage,
        carriage,
        elem_a="right_shoe",
        elem_b="guide_right",
        name="right_stage_shoe_on_right_guide",
    )

    with ctx.pose({carriage_to_stage: 0.0}):
        ctx.expect_within(
            stage,
            carriage,
            axes="x",
            margin=0.0,
            name="stage_within_carriage_width",
        )
        ctx.expect_overlap(
            stage,
            carriage,
            axes="xz",
            min_overlap=0.14,
            name="stage_nested_in_carriage_face",
        )

    with ctx.pose({frame_to_carriage: frame_to_carriage.motion_limits.lower}):
        carriage_left = ctx.part_world_position(carriage)
    with ctx.pose({frame_to_carriage: frame_to_carriage.motion_limits.upper}):
        carriage_right = ctx.part_world_position(carriage)

    carriage_motion_ok = (
        carriage_left is not None
        and carriage_right is not None
        and carriage_right[0] - carriage_left[0] > 0.17
        and abs(carriage_right[2] - carriage_left[2]) < 1e-6
    )
    ctx.check(
        "carriage_moves_horizontally",
        carriage_motion_ok,
        details=(
            f"expected strong +X travel with stable height, got left={carriage_left}, "
            f"right={carriage_right}"
        ),
    )

    with ctx.pose({carriage_to_stage: 0.0}):
        stage_rest = ctx.part_world_position(stage)
    with ctx.pose({carriage_to_stage: carriage_to_stage.motion_limits.upper}):
        stage_lowered = ctx.part_world_position(stage)

    stage_motion_ok = (
        stage_rest is not None
        and stage_lowered is not None
        and stage_lowered[2] < stage_rest[2] - 0.09
        and abs(stage_lowered[0] - stage_rest[0]) < 1e-6
    )
    ctx.check(
        "stage_descends_vertically",
        stage_motion_ok,
        details=(
            f"expected stage to lower along -Z with stable X, got rest={stage_rest}, "
            f"lowered={stage_lowered}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
