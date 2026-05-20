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

MAST_HEIGHT = 0.88
MAST_WIDTH = 0.52
RAIL_WIDTH = 0.08
RAIL_DEPTH = 0.06
RAIL_WALL = 0.008
TOP_CROSSHEAD_HEIGHT = 0.05
BOTTOM_TIE_HEIGHT = 0.05
BOTTOM_TIE_DEPTH = 0.03
RAIL_CENTER_X = MAST_WIDTH / 2.0 - RAIL_WIDTH / 2.0

CARRIAGE_ORIGIN_Z = 0.24
CARRIAGE_TRAVEL = 0.25
CARRIAGE_WIDTH = 0.32
CARRIAGE_DEPTH = 0.022
CARRIAGE_HEIGHT = 0.20
GUIDE_WIDTH = 0.03
GUIDE_DEPTH = 0.044
GUIDE_HEIGHT = 0.06
GUIDE_CENTER_X = 0.204
GUIDE_UPPER_Z = 0.07
GUIDE_LOWER_Z = -0.07

FORK_SHANK_WIDTH = 0.04
FORK_SHANK_DEPTH = 0.022
FORK_SHANK_HEIGHT = 0.08
FORK_TINE_LENGTH = 0.26
FORK_TINE_THICKNESS = 0.025
FORK_CENTER_X = 0.12


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="forklift_mast_module", assets=ASSETS)

    mast_gray = model.material("mast_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    carriage_black = model.material("carriage_black", rgba=(0.16, 0.17, 0.18, 1.0))
    fork_steel = model.material("fork_steel", rgba=(0.42, 0.44, 0.46, 1.0))

    mast = model.part("mast")

    # Left upright channel, open toward the carriage.
    mast.visual(
        Box((RAIL_WALL, RAIL_DEPTH, MAST_HEIGHT)),
        origin=Origin(xyz=(-MAST_WIDTH / 2.0 + RAIL_WALL / 2.0, 0.0, MAST_HEIGHT / 2.0)),
        material=mast_gray,
        name="left_web",
    )
    mast.visual(
        Box((RAIL_WIDTH - RAIL_WALL, RAIL_WALL, MAST_HEIGHT)),
        origin=Origin(
            xyz=(
                -RAIL_CENTER_X + RAIL_WALL / 2.0,
                RAIL_DEPTH / 2.0 - RAIL_WALL / 2.0,
                MAST_HEIGHT / 2.0,
            )
        ),
        material=mast_gray,
        name="left_front_flange",
    )
    mast.visual(
        Box((RAIL_WIDTH - RAIL_WALL, RAIL_WALL, MAST_HEIGHT)),
        origin=Origin(
            xyz=(
                -RAIL_CENTER_X + RAIL_WALL / 2.0,
                -RAIL_DEPTH / 2.0 + RAIL_WALL / 2.0,
                MAST_HEIGHT / 2.0,
            )
        ),
        material=mast_gray,
        name="left_rear_flange",
    )

    # Right upright channel, mirrored.
    mast.visual(
        Box((RAIL_WALL, RAIL_DEPTH, MAST_HEIGHT)),
        origin=Origin(xyz=(MAST_WIDTH / 2.0 - RAIL_WALL / 2.0, 0.0, MAST_HEIGHT / 2.0)),
        material=mast_gray,
        name="right_web",
    )
    mast.visual(
        Box((RAIL_WIDTH - RAIL_WALL, RAIL_WALL, MAST_HEIGHT)),
        origin=Origin(
            xyz=(
                RAIL_CENTER_X - RAIL_WALL / 2.0,
                RAIL_DEPTH / 2.0 - RAIL_WALL / 2.0,
                MAST_HEIGHT / 2.0,
            )
        ),
        material=mast_gray,
        name="right_front_flange",
    )
    mast.visual(
        Box((RAIL_WIDTH - RAIL_WALL, RAIL_WALL, MAST_HEIGHT)),
        origin=Origin(
            xyz=(
                RAIL_CENTER_X - RAIL_WALL / 2.0,
                -RAIL_DEPTH / 2.0 + RAIL_WALL / 2.0,
                MAST_HEIGHT / 2.0,
            )
        ),
        material=mast_gray,
        name="right_rear_flange",
    )

    mast.visual(
        Box((MAST_WIDTH, RAIL_DEPTH, TOP_CROSSHEAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT - TOP_CROSSHEAD_HEIGHT / 2.0)),
        material=mast_gray,
        name="top_crosshead",
    )
    mast.visual(
        Box((MAST_WIDTH, BOTTOM_TIE_DEPTH, BOTTOM_TIE_HEIGHT)),
        origin=Origin(xyz=(0.0, -BOTTOM_TIE_DEPTH / 2.0, BOTTOM_TIE_HEIGHT / 2.0)),
        material=mast_gray,
        name="bottom_tie",
    )
    mast.inertial = Inertial.from_geometry(
        Box((MAST_WIDTH, RAIL_DEPTH, MAST_HEIGHT)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_WIDTH, CARRIAGE_DEPTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=carriage_black,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.029, 0.018, 0.18)),
        origin=Origin(xyz=(-0.1745, 0.010, 0.0)),
        material=carriage_black,
        name="left_side_arm",
    )
    carriage.visual(
        Box((0.029, 0.018, 0.18)),
        origin=Origin(xyz=(0.1745, 0.010, 0.0)),
        material=carriage_black,
        name="right_side_arm",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_CENTER_X, 0.0, GUIDE_UPPER_Z)),
        material=fork_steel,
        name="left_upper_shoe",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_CENTER_X, 0.0, GUIDE_LOWER_Z)),
        material=fork_steel,
        name="left_lower_shoe",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, 0.0, GUIDE_UPPER_Z)),
        material=fork_steel,
        name="right_upper_shoe",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_CENTER_X, 0.0, GUIDE_LOWER_Z)),
        material=fork_steel,
        name="right_lower_shoe",
    )

    # Load backrest above the carriage plate.
    carriage.visual(
        Box((0.02, 0.018, 0.20)),
        origin=Origin(xyz=(-0.10, 0.018, 0.20)),
        material=carriage_black,
        name="backrest_left_post",
    )
    carriage.visual(
        Box((0.02, 0.018, 0.20)),
        origin=Origin(xyz=(0.10, 0.018, 0.20)),
        material=carriage_black,
        name="backrest_right_post",
    )
    carriage.visual(
        Box((0.24, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.018, 0.215)),
        material=carriage_black,
        name="backrest_mid_rail",
    )
    carriage.visual(
        Box((0.24, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.018, 0.27)),
        material=carriage_black,
        name="backrest_top",
    )
    carriage.visual(
        Box((0.018, 0.018, 0.18)),
        origin=Origin(xyz=(-0.033, 0.018, 0.20)),
        material=carriage_black,
        name="backrest_inner_left",
    )
    carriage.visual(
        Box((0.018, 0.018, 0.18)),
        origin=Origin(xyz=(0.033, 0.018, 0.20)),
        material=carriage_black,
        name="backrest_inner_right",
    )

    # Fork pair: vertical shanks and forward tines.
    carriage.visual(
        Box((FORK_SHANK_WIDTH, FORK_SHANK_DEPTH, FORK_SHANK_HEIGHT)),
        origin=Origin(xyz=(-FORK_CENTER_X, 0.018, -0.14)),
        material=fork_steel,
        name="fork_left_shank",
    )
    carriage.visual(
        Box((FORK_SHANK_WIDTH, FORK_SHANK_DEPTH, FORK_SHANK_HEIGHT)),
        origin=Origin(xyz=(FORK_CENTER_X, 0.018, -0.14)),
        material=fork_steel,
        name="fork_right_shank",
    )
    carriage.visual(
        Box((FORK_SHANK_WIDTH, FORK_TINE_LENGTH, FORK_TINE_THICKNESS)),
        origin=Origin(xyz=(-FORK_CENTER_X, 0.158, -0.1925)),
        material=fork_steel,
        name="fork_left_tine",
    )
    carriage.visual(
        Box((FORK_SHANK_WIDTH, FORK_TINE_LENGTH, FORK_TINE_THICKNESS)),
        origin=Origin(xyz=(FORK_CENTER_X, 0.158, -0.1925)),
        material=fork_steel,
        name="fork_right_tine",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.40, 0.32, 0.52)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.13, 0.03)),
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_lift")

    top_crosshead = mast.get_visual("top_crosshead")
    bottom_tie = mast.get_visual("bottom_tie")
    left_front_flange = mast.get_visual("left_front_flange")
    left_rear_flange = mast.get_visual("left_rear_flange")
    right_front_flange = mast.get_visual("right_front_flange")
    right_rear_flange = mast.get_visual("right_rear_flange")
    left_upper_shoe = carriage.get_visual("left_upper_shoe")
    left_lower_shoe = carriage.get_visual("left_lower_shoe")
    right_upper_shoe = carriage.get_visual("right_upper_shoe")
    right_lower_shoe = carriage.get_visual("right_lower_shoe")
    carriage_plate = carriage.get_visual("carriage_plate")
    backrest_top = carriage.get_visual("backrest_top")
    fork_left_tine = carriage.get_visual("fork_left_tine")
    fork_right_tine = carriage.get_visual("fork_right_tine")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_origin_distance(
        carriage,
        mast,
        axes="xy",
        max_dist=0.001,
        name="carriage_centered_between_mast_rails",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_origin_gap(
            carriage,
            mast,
            axis="z",
            min_gap=CARRIAGE_ORIGIN_Z - 0.001,
            max_gap=CARRIAGE_ORIGIN_Z + 0.001,
            name="carriage_rest_height",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            inner_elem=left_upper_shoe,
            margin=0.0,
            name="left_upper_shoe_captured_by_mast",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            inner_elem=left_lower_shoe,
            margin=0.0,
            name="left_lower_shoe_captured_by_mast",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            inner_elem=right_upper_shoe,
            margin=0.0,
            name="right_upper_shoe_captured_by_mast",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            inner_elem=right_lower_shoe,
            margin=0.0,
            name="right_lower_shoe_captured_by_mast",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem=carriage_plate,
            negative_elem=bottom_tie,
            min_gap=0.004,
            name="carriage_plate_sits_forward_of_bottom_tie",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=left_upper_shoe,
            elem_b=left_front_flange,
            name="left_upper_shoe_contacts_front_flange",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=left_upper_shoe,
            elem_b=left_rear_flange,
            name="left_upper_shoe_contacts_rear_flange",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=right_upper_shoe,
            elem_b=right_front_flange,
            name="right_upper_shoe_contacts_front_flange",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=right_upper_shoe,
            elem_b=right_rear_flange,
            name="right_upper_shoe_contacts_rear_flange",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem=top_crosshead,
            negative_elem=backrest_top,
            min_gap=0.05,
            name="backrest_clears_crosshead_at_rest",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="x",
            min_overlap=0.39,
            name="carriage_tracks_between_uprights",
        )

    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        ctx.expect_origin_gap(
            carriage,
            mast,
            axis="z",
            min_gap=CARRIAGE_ORIGIN_Z + CARRIAGE_TRAVEL - 0.001,
            max_gap=CARRIAGE_ORIGIN_Z + CARRIAGE_TRAVEL + 0.001,
            name="carriage_raises_full_travel",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem=top_crosshead,
            negative_elem=backrest_top,
            min_gap=0.05,
            name="backrest_clears_crosshead_at_full_raise",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            inner_elem=left_upper_shoe,
            margin=0.0,
            name="left_upper_shoe_stays_captured_when_raised",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            inner_elem=left_lower_shoe,
            margin=0.0,
            name="left_lower_shoe_stays_captured_when_raised",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            inner_elem=right_upper_shoe,
            margin=0.0,
            name="right_upper_shoe_stays_captured_when_raised",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xy",
            inner_elem=right_lower_shoe,
            margin=0.0,
            name="right_lower_shoe_stays_captured_when_raised",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=left_lower_shoe,
            elem_b=left_front_flange,
            name="left_lower_shoe_contacts_front_flange_when_raised",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=left_lower_shoe,
            elem_b=left_rear_flange,
            name="left_lower_shoe_contacts_rear_flange_when_raised",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=right_lower_shoe,
            elem_b=right_front_flange,
            name="right_lower_shoe_contacts_front_flange_when_raised",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=right_lower_shoe,
            elem_b=right_rear_flange,
            name="right_lower_shoe_contacts_rear_flange_when_raised",
        )

    plate_bb = ctx.part_element_world_aabb(carriage, elem="carriage_plate")
    left_fork_bb = ctx.part_element_world_aabb(carriage, elem="fork_left_tine")
    right_fork_bb = ctx.part_element_world_aabb(carriage, elem="fork_right_tine")
    if plate_bb and left_fork_bb and right_fork_bb:
        plate_min, plate_max = plate_bb
        left_min, left_max = left_fork_bb
        right_min, right_max = right_fork_bb
        ctx.check(
            "forks_project_forward_of_carriage",
            left_max[1] > plate_max[1] + 0.20 and right_max[1] > plate_max[1] + 0.20,
            details=(
                f"plate_max_y={plate_max[1]:.4f}, "
                f"left_fork_max_y={left_max[1]:.4f}, right_fork_max_y={right_max[1]:.4f}"
            ),
        )
        ctx.check(
            "fork_pair_is_evenly_spaced",
            abs((left_min[0] + left_max[0]) / 2.0 + (right_min[0] + right_max[0]) / 2.0) < 1e-6
            and ((right_min[0] + right_max[0]) / 2.0 - (left_min[0] + left_max[0]) / 2.0) > 0.20,
            details=(
                f"left_center_x={(left_min[0] + left_max[0]) / 2.0:.4f}, "
                f"right_center_x={(right_min[0] + right_max[0]) / 2.0:.4f}"
            ),
        )
        ctx.check(
            "fork_tips_are_level",
            abs(left_min[2] - right_min[2]) < 1e-6 and abs(left_max[2] - right_max[2]) < 1e-6,
            details=(
                f"left_z=({left_min[2]:.4f}, {left_max[2]:.4f}), "
                f"right_z=({right_min[2]:.4f}, {right_max[2]:.4f})"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
