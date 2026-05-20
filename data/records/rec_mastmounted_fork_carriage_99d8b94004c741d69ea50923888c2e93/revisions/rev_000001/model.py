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


FRAME_HEIGHT = 2.34
FRAME_OUTER_WIDTH = 0.86
COLUMN_WIDTH = 0.10
COLUMN_DEPTH = 0.14
COLUMN_CENTER_X = (FRAME_OUTER_WIDTH - COLUMN_WIDTH) / 2.0
COLUMN_BACK_WEB = 0.03
COLUMN_FLANGE = 0.02
COLUMN_END_CAP = 0.10

BOTTOM_CROSS_HEIGHT = 0.12
BOTTOM_CROSS_DEPTH = 0.12
TOP_CROSS_HEIGHT = 0.10
TOP_CROSS_DEPTH = 0.10

FRAME_GUIDE_STRIP_WIDTH = 0.07
FRAME_GUIDE_STRIP_DEPTH = 0.012
FRAME_GUIDE_STRIP_HEIGHT = 2.18
FRAME_GUIDE_STRIP_BOTTOM = 0.08
FRAME_GUIDE_STRIP_CENTER_Y = 0.069

CARRIAGE_HOME_Z = 0.09
CARRIAGE_TRAVEL = 1.20

CARRIAGE_PLATE_WIDTH = 0.58
CARRIAGE_PLATE_DEPTH = 0.04
CARRIAGE_PLATE_HEIGHT = 0.66
CARRIAGE_PLATE_BOTTOM = 0.18

LOWER_BAR_DEPTH = 0.08
LOWER_BAR_HEIGHT = 0.10
LOWER_BAR_BOTTOM = 0.08

BACKREST_BAR_WIDTH = 0.05
BACKREST_BAR_DEPTH = 0.04
BACKREST_BAR_HEIGHT = 0.38
BACKREST_BAR_BOTTOM = CARRIAGE_PLATE_BOTTOM + CARRIAGE_PLATE_HEIGHT

SIDE_MOUNT_WIDTH = 0.045
SIDE_MOUNT_DEPTH = 0.08
SIDE_MOUNT_HEIGHT = 0.84
SIDE_MOUNT_BOTTOM = 0.08
SIDE_MOUNT_CENTER_Y = 0.125
SIDE_MOUNT_CENTER_X = 0.300

GUIDE_SHOE_WIDTH = 0.06
GUIDE_SHOE_DEPTH = 0.10
GUIDE_SHOE_HEIGHT = 0.18
GUIDE_SHOE_CENTER_Y = 0.125
GUIDE_SHOE_CENTER_X = 0.350
GUIDE_LOWER_BOTTOM = 0.08
GUIDE_UPPER_BOTTOM = 0.72

FORK_WIDTH = 0.11
FORK_HEEL_DEPTH = 0.18
FORK_HEEL_HEIGHT = 0.16
FORK_HEEL_BOTTOM = 0.02
FORK_HEEL_REAR_Y = 0.10
FORK_BLADE_LENGTH = 0.92
FORK_BLADE_THICKNESS = 0.04
FORK_BLADE_REAR_Y = FORK_HEEL_REAR_Y + FORK_HEEL_DEPTH
FORK_CENTER_X = 0.18
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_frame_fork_mast")
    model.material("mast_gray", rgba=(0.41, 0.43, 0.46, 1.0))
    model.material("carriage_black", rgba=(0.18, 0.19, 0.21, 1.0))

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, 0.0, FRAME_HEIGHT / 2.0)),
        material="mast_gray",
        name="left_column",
    )
    rear_frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(COLUMN_CENTER_X, 0.0, FRAME_HEIGHT / 2.0)),
        material="mast_gray",
        name="right_column",
    )
    rear_frame.visual(
        Box((FRAME_OUTER_WIDTH - 0.04, BOTTOM_CROSS_DEPTH, BOTTOM_CROSS_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.02, 0.10)),
        material="mast_gray",
        name="bottom_cross",
    )
    rear_frame.visual(
        Box((FRAME_OUTER_WIDTH - 0.10, TOP_CROSS_DEPTH, TOP_CROSS_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.01, FRAME_HEIGHT - 0.11)),
        material="mast_gray",
        name="top_cross",
    )
    rear_frame.visual(
        Box((0.16, 0.08, 1.34)),
        origin=Origin(xyz=(0.0, -0.05, 0.83)),
        material="mast_gray",
        name="rear_spine",
    )
    rear_frame.visual(
        Box((0.20, 0.22, 0.03)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, -0.06, 0.015)),
        material="mast_gray",
        name="left_foot",
    )
    rear_frame.visual(
        Box((0.20, 0.22, 0.03)),
        origin=Origin(xyz=(COLUMN_CENTER_X, -0.06, 0.015)),
        material="mast_gray",
        name="right_foot",
    )
    rear_frame.visual(
        Box((FRAME_GUIDE_STRIP_WIDTH, FRAME_GUIDE_STRIP_DEPTH, FRAME_GUIDE_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                -COLUMN_CENTER_X,
                FRAME_GUIDE_STRIP_CENTER_Y,
                FRAME_GUIDE_STRIP_BOTTOM + (FRAME_GUIDE_STRIP_HEIGHT / 2.0),
            )
        ),
        material="mast_gray",
        name="left_guide_strip",
    )
    rear_frame.visual(
        Box((FRAME_GUIDE_STRIP_WIDTH, FRAME_GUIDE_STRIP_DEPTH, FRAME_GUIDE_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                COLUMN_CENTER_X,
                FRAME_GUIDE_STRIP_CENTER_Y,
                FRAME_GUIDE_STRIP_BOTTOM + (FRAME_GUIDE_STRIP_HEIGHT / 2.0),
            )
        ),
        material="mast_gray",
        name="right_guide_strip",
    )

    carriage_forks = model.part("carriage_forks")
    carriage_forks.visual(
        Box((CARRIAGE_PLATE_WIDTH, CARRIAGE_PLATE_DEPTH, CARRIAGE_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.14,
                CARRIAGE_PLATE_BOTTOM + (CARRIAGE_PLATE_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="front_plate",
    )
    carriage_forks.visual(
        Box((CARRIAGE_PLATE_WIDTH, LOWER_BAR_DEPTH, LOWER_BAR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.12,
                LOWER_BAR_BOTTOM + (LOWER_BAR_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="lower_bar",
    )
    carriage_forks.visual(
        Box((SIDE_MOUNT_WIDTH, SIDE_MOUNT_DEPTH, SIDE_MOUNT_HEIGHT)),
        origin=Origin(
            xyz=(
                -SIDE_MOUNT_CENTER_X,
                SIDE_MOUNT_CENTER_Y,
                SIDE_MOUNT_BOTTOM + (SIDE_MOUNT_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="left_side_mount",
    )
    carriage_forks.visual(
        Box((SIDE_MOUNT_WIDTH, SIDE_MOUNT_DEPTH, SIDE_MOUNT_HEIGHT)),
        origin=Origin(
            xyz=(
                SIDE_MOUNT_CENTER_X,
                SIDE_MOUNT_CENTER_Y,
                SIDE_MOUNT_BOTTOM + (SIDE_MOUNT_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="right_side_mount",
    )
    carriage_forks.visual(
        Box((BACKREST_BAR_WIDTH, BACKREST_BAR_DEPTH, BACKREST_BAR_HEIGHT)),
        origin=Origin(
            xyz=(
                -0.24,
                0.14,
                BACKREST_BAR_BOTTOM + (BACKREST_BAR_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="left_backrest_bar",
    )
    carriage_forks.visual(
        Box((BACKREST_BAR_WIDTH, BACKREST_BAR_DEPTH, BACKREST_BAR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.14,
                BACKREST_BAR_BOTTOM + (BACKREST_BAR_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="center_backrest_bar",
    )
    carriage_forks.visual(
        Box((BACKREST_BAR_WIDTH, BACKREST_BAR_DEPTH, BACKREST_BAR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.24,
                0.14,
                BACKREST_BAR_BOTTOM + (BACKREST_BAR_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="right_backrest_bar",
    )
    carriage_forks.visual(
        Box((CARRIAGE_PLATE_WIDTH, BACKREST_BAR_DEPTH, 0.05)),
        origin=Origin(
            xyz=(
                0.0,
                0.14,
                BACKREST_BAR_BOTTOM + BACKREST_BAR_HEIGHT + 0.015,
            )
        ),
        material="carriage_black",
        name="top_guard_bar",
    )
    carriage_forks.visual(
        Box((FORK_WIDTH, FORK_HEEL_DEPTH, FORK_HEEL_HEIGHT)),
        origin=Origin(
            xyz=(
                -FORK_CENTER_X,
                FORK_HEEL_REAR_Y + (FORK_HEEL_DEPTH / 2.0),
                FORK_HEEL_BOTTOM + (FORK_HEEL_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="left_fork_heel",
    )
    carriage_forks.visual(
        Box((FORK_WIDTH, FORK_BLADE_LENGTH, FORK_BLADE_THICKNESS)),
        origin=Origin(
            xyz=(
                -FORK_CENTER_X,
                FORK_BLADE_REAR_Y + (FORK_BLADE_LENGTH / 2.0),
                FORK_BLADE_THICKNESS / 2.0,
            )
        ),
        material="carriage_black",
        name="left_fork_blade",
    )
    carriage_forks.visual(
        Box((FORK_WIDTH, FORK_HEEL_DEPTH, FORK_HEEL_HEIGHT)),
        origin=Origin(
            xyz=(
                FORK_CENTER_X,
                FORK_HEEL_REAR_Y + (FORK_HEEL_DEPTH / 2.0),
                FORK_HEEL_BOTTOM + (FORK_HEEL_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="right_fork_heel",
    )
    carriage_forks.visual(
        Box((FORK_WIDTH, FORK_BLADE_LENGTH, FORK_BLADE_THICKNESS)),
        origin=Origin(
            xyz=(
                FORK_CENTER_X,
                FORK_BLADE_REAR_Y + (FORK_BLADE_LENGTH / 2.0),
                FORK_BLADE_THICKNESS / 2.0,
            )
        ),
        material="carriage_black",
        name="right_fork_blade",
    )
    carriage_forks.visual(
        Box((GUIDE_SHOE_WIDTH, GUIDE_SHOE_DEPTH, GUIDE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(
                -GUIDE_SHOE_CENTER_X,
                GUIDE_SHOE_CENTER_Y,
                GUIDE_LOWER_BOTTOM + (GUIDE_SHOE_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="left_lower_shoe",
    )
    carriage_forks.visual(
        Box((GUIDE_SHOE_WIDTH, GUIDE_SHOE_DEPTH, GUIDE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(
                -GUIDE_SHOE_CENTER_X,
                GUIDE_SHOE_CENTER_Y,
                GUIDE_UPPER_BOTTOM + (GUIDE_SHOE_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="left_upper_shoe",
    )
    carriage_forks.visual(
        Box((GUIDE_SHOE_WIDTH, GUIDE_SHOE_DEPTH, GUIDE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(
                GUIDE_SHOE_CENTER_X,
                GUIDE_SHOE_CENTER_Y,
                GUIDE_LOWER_BOTTOM + (GUIDE_SHOE_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="right_lower_shoe",
    )
    carriage_forks.visual(
        Box((GUIDE_SHOE_WIDTH, GUIDE_SHOE_DEPTH, GUIDE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(
                GUIDE_SHOE_CENTER_X,
                GUIDE_SHOE_CENTER_Y,
                GUIDE_UPPER_BOTTOM + (GUIDE_SHOE_HEIGHT / 2.0),
            )
        ),
        material="carriage_black",
        name="right_upper_shoe",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_frame,
        child=carriage_forks,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=18000.0,
            velocity=0.35,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    carriage_forks = object_model.get_part("carriage_forks")
    lift = object_model.get_articulation("frame_to_carriage")
    left_column = rear_frame.get_visual("left_column")
    right_column = rear_frame.get_visual("right_column")
    left_guide_strip = rear_frame.get_visual("left_guide_strip")
    right_guide_strip = rear_frame.get_visual("right_guide_strip")
    front_plate = carriage_forks.get_visual("front_plate")
    left_fork_blade = carriage_forks.get_visual("left_fork_blade")
    right_fork_blade = carriage_forks.get_visual("right_fork_blade")
    left_lower_shoe = carriage_forks.get_visual("left_lower_shoe")
    right_lower_shoe = carriage_forks.get_visual("right_lower_shoe")
    left_upper_shoe = carriage_forks.get_visual("left_upper_shoe")
    right_upper_shoe = carriage_forks.get_visual("right_upper_shoe")

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
        "lift_is_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"expected PRISMATIC articulation, got {lift.articulation_type}",
    )
    ctx.expect_contact(
        carriage_forks,
        rear_frame,
        elem_a=left_lower_shoe,
        elem_b=left_guide_strip,
        name="left_lower_shoe_contacts_left_rail",
    )
    ctx.expect_contact(
        carriage_forks,
        rear_frame,
        elem_a=right_lower_shoe,
        elem_b=right_guide_strip,
        name="right_lower_shoe_contacts_right_rail",
    )
    ctx.expect_gap(
        carriage_forks,
        rear_frame,
        axis="y",
        min_gap=0.045,
        positive_elem=front_plate,
        negative_elem=left_column,
        name="front_plate_runs_in_front_of_left_column",
    )
    ctx.expect_gap(
        carriage_forks,
        rear_frame,
        axis="y",
        min_gap=0.045,
        positive_elem=front_plate,
        negative_elem=right_column,
        name="front_plate_runs_in_front_of_right_column",
    )
    ctx.expect_gap(
        carriage_forks,
        rear_frame,
        axis="y",
        min_gap=0.20,
        positive_elem=left_fork_blade,
        negative_elem=left_column,
        name="left_fork_projects_forward_of_frame",
    )
    ctx.expect_gap(
        carriage_forks,
        rear_frame,
        axis="y",
        min_gap=0.20,
        positive_elem=right_fork_blade,
        negative_elem=right_column,
        name="right_fork_projects_forward_of_frame",
    )
    ctx.expect_within(
        carriage_forks,
        rear_frame,
        axes="x",
        margin=0.02,
        name="carriage_stays_between_side_frames",
    )

    home_position = ctx.part_world_position(carriage_forks)
    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        raised_position = ctx.part_world_position(carriage_forks)
        ctx.expect_contact(
            carriage_forks,
            rear_frame,
            elem_a=left_upper_shoe,
            elem_b=left_guide_strip,
            name="left_upper_shoe_stays_guided_at_full_raise",
        )
        ctx.expect_contact(
            carriage_forks,
            rear_frame,
            elem_a=right_upper_shoe,
            elem_b=right_guide_strip,
            name="right_upper_shoe_stays_guided_at_full_raise",
        )
        ctx.expect_gap(
            carriage_forks,
            rear_frame,
            axis="y",
            min_gap=0.045,
            positive_elem=front_plate,
            negative_elem=left_column,
            name="front_plate_stays_in_front_of_left_column_at_full_raise",
        )
        ctx.expect_gap(
            carriage_forks,
            rear_frame,
            axis="y",
            min_gap=0.045,
            positive_elem=front_plate,
            negative_elem=right_column,
            name="front_plate_stays_in_front_of_right_column_at_full_raise",
        )

    lift_ok = (
        home_position is not None
        and raised_position is not None
        and abs(raised_position[0] - home_position[0]) <= 1e-6
        and abs(raised_position[1] - home_position[1]) <= 1e-6
        and raised_position[2] >= home_position[2] + (CARRIAGE_TRAVEL - 1e-6)
    )
    ctx.check(
        "carriage_translates_vertically",
        lift_ok,
        details=(
            f"home={home_position}, raised={raised_position}, "
            f"expected +Z travel of {CARRIAGE_TRAVEL:.3f} m"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
