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


MAST_HEIGHT = 1.85
RAIL_WIDTH = 0.10
RAIL_DEPTH = 0.08
RAIL_CENTER_X = 0.28
FRAME_WIDTH = (2.0 * RAIL_CENTER_X) + RAIL_WIDTH

TOP_CROSS_HEIGHT = 0.11
BOTTOM_CROSS_HEIGHT = 0.14
CROSS_DEPTH = 0.07
REAR_BRACE_HEIGHT = 0.08
REAR_BRACE_DEPTH = 0.03
REAR_BRACE_Z = 0.82

CARRIAGE_HEIGHT = 0.44
GUIDE_WALL = 0.014
GUIDE_SLEEVE_WIDTH = RAIL_WIDTH + (2.0 * GUIDE_WALL)
GUIDE_SLEEVE_DEPTH = RAIL_DEPTH + (2.0 * GUIDE_WALL)

TOP_BEAM_HEIGHT = 0.06
BOTTOM_BEAM_HEIGHT = 0.06
INNER_BEAM_WIDTH = (2.0 * (RAIL_CENTER_X - (RAIL_WIDTH / 2.0)))
INNER_CLEAR_WIDTH = INNER_BEAM_WIDTH - (2.0 * GUIDE_WALL)

LOAD_PLATE_WIDTH = 0.42
LOAD_PLATE_HEIGHT = 0.24
LOAD_PLATE_DEPTH = 0.02

FORK_BAR_WIDTH = 0.44
FORK_BAR_HEIGHT = 0.05
FORK_BAR_DEPTH = 0.05

FORK_WIDTH = 0.09
FORK_SHANK_HEIGHT = 0.15
FORK_SHANK_DEPTH = 0.05
FORK_TINE_LENGTH = 0.85
FORK_TINE_THICKNESS = 0.04
FORK_CENTER_X = 0.155

HOME_CARRIAGE_Z = 0.50
LIFT_TRAVEL = 0.94


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_rail_forklift_mast")

    model.material("mast_steel", rgba=(0.29, 0.30, 0.33, 1.0))
    model.material("carriage_steel", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("fork_steel", rgba=(0.17, 0.18, 0.20, 1.0))

    mast = model.part("mast_frame")
    mast.visual(
        Box((RAIL_WIDTH, RAIL_DEPTH, MAST_HEIGHT)),
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, MAST_HEIGHT / 2.0)),
        material="mast_steel",
        name="left_rail",
    )
    mast.visual(
        Box((RAIL_WIDTH, RAIL_DEPTH, MAST_HEIGHT)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, MAST_HEIGHT / 2.0)),
        material="mast_steel",
        name="right_rail",
    )
    mast.visual(
        Box((FRAME_WIDTH, CROSS_DEPTH, TOP_CROSS_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.005, MAST_HEIGHT - (TOP_CROSS_HEIGHT / 2.0))),
        material="mast_steel",
        name="top_cross",
    )
    mast.visual(
        Box((FRAME_WIDTH, CROSS_DEPTH, BOTTOM_CROSS_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.005, BOTTOM_CROSS_HEIGHT / 2.0)),
        material="mast_steel",
        name="bottom_cross",
    )
    mast.visual(
        Box((FRAME_WIDTH - 0.10, REAR_BRACE_DEPTH, REAR_BRACE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -(RAIL_DEPTH / 2.0) + (REAR_BRACE_DEPTH / 2.0), REAR_BRACE_Z)
        ),
        material="mast_steel",
        name="rear_brace",
    )
    mast.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, RAIL_DEPTH, MAST_HEIGHT)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")

    # Left guide sleeve wrapping the left rail.
    mast_left_outer_face = -RAIL_CENTER_X - (RAIL_WIDTH / 2.0)
    mast_left_inner_face = -RAIL_CENTER_X + (RAIL_WIDTH / 2.0)
    mast_right_inner_face = RAIL_CENTER_X - (RAIL_WIDTH / 2.0)
    mast_right_outer_face = RAIL_CENTER_X + (RAIL_WIDTH / 2.0)
    mast_front_face = RAIL_DEPTH / 2.0
    mast_rear_face = -(RAIL_DEPTH / 2.0)

    left_outer_x = mast_left_outer_face - (GUIDE_WALL / 2.0)
    left_inner_x = mast_left_inner_face + (GUIDE_WALL / 2.0)
    right_inner_x = mast_right_inner_face - (GUIDE_WALL / 2.0)
    right_outer_x = mast_right_outer_face + (GUIDE_WALL / 2.0)
    front_y = mast_front_face + (GUIDE_WALL / 2.0)
    rear_y = mast_rear_face - (GUIDE_WALL / 2.0)

    sleeve_wall_material = "carriage_steel"

    carriage.visual(
        Box((GUIDE_WALL, GUIDE_SLEEVE_DEPTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(left_outer_x, 0.0, 0.0)),
        material=sleeve_wall_material,
        name="left_outer_guide",
    )
    carriage.visual(
        Box((GUIDE_WALL, GUIDE_SLEEVE_DEPTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(left_inner_x, 0.0, 0.0)),
        material=sleeve_wall_material,
        name="left_inner_guide",
    )
    carriage.visual(
        Box((GUIDE_SLEEVE_WIDTH, GUIDE_WALL, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(-RAIL_CENTER_X, front_y, 0.0)),
        material=sleeve_wall_material,
        name="left_front_guide",
    )
    carriage.visual(
        Box((GUIDE_SLEEVE_WIDTH, GUIDE_WALL, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(-RAIL_CENTER_X, rear_y, 0.0)),
        material=sleeve_wall_material,
        name="left_rear_guide",
    )

    carriage.visual(
        Box((GUIDE_WALL, GUIDE_SLEEVE_DEPTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(right_outer_x, 0.0, 0.0)),
        material=sleeve_wall_material,
        name="right_outer_guide",
    )
    carriage.visual(
        Box((GUIDE_WALL, GUIDE_SLEEVE_DEPTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(right_inner_x, 0.0, 0.0)),
        material=sleeve_wall_material,
        name="right_inner_guide",
    )
    carriage.visual(
        Box((GUIDE_SLEEVE_WIDTH, GUIDE_WALL, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(RAIL_CENTER_X, front_y, 0.0)),
        material=sleeve_wall_material,
        name="right_front_guide",
    )
    carriage.visual(
        Box((GUIDE_SLEEVE_WIDTH, GUIDE_WALL, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(RAIL_CENTER_X, rear_y, 0.0)),
        material=sleeve_wall_material,
        name="right_rear_guide",
    )

    carriage.visual(
        Box((INNER_BEAM_WIDTH, 0.10, TOP_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material="carriage_steel",
        name="top_beam",
    )
    carriage.visual(
        Box((INNER_BEAM_WIDTH, 0.10, BOTTOM_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material="carriage_steel",
        name="bottom_beam",
    )
    carriage.visual(
        Box((LOAD_PLATE_WIDTH, LOAD_PLATE_DEPTH, LOAD_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.05, 0.01)),
        material="carriage_steel",
        name="load_plate",
    )
    carriage.visual(
        Box((FORK_BAR_WIDTH, FORK_BAR_DEPTH, FORK_BAR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.07, -0.18)),
        material="carriage_steel",
        name="fork_bar",
    )

    carriage.visual(
        Box((FORK_WIDTH, FORK_SHANK_DEPTH, FORK_SHANK_HEIGHT)),
        origin=Origin(xyz=(-FORK_CENTER_X, 0.095, -0.23)),
        material="fork_steel",
        name="left_fork_shank",
    )
    carriage.visual(
        Box((FORK_WIDTH, FORK_TINE_LENGTH, FORK_TINE_THICKNESS)),
        origin=Origin(
            xyz=(
                -FORK_CENTER_X,
                0.07 + (FORK_TINE_LENGTH / 2.0),
                -0.225,
            )
        ),
        material="fork_steel",
        name="left_fork_tine",
    )
    carriage.visual(
        Box((FORK_WIDTH, FORK_SHANK_DEPTH, FORK_SHANK_HEIGHT)),
        origin=Origin(xyz=(FORK_CENTER_X, 0.095, -0.23)),
        material="fork_steel",
        name="right_fork_shank",
    )
    carriage.visual(
        Box((FORK_WIDTH, FORK_TINE_LENGTH, FORK_TINE_THICKNESS)),
        origin=Origin(
            xyz=(
                FORK_CENTER_X,
                0.07 + (FORK_TINE_LENGTH / 2.0),
                -0.225,
            )
        ),
        material="fork_steel",
        name="right_fork_tine",
    )

    carriage.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH + (2.0 * GUIDE_WALL), 0.97, 0.53)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.43, -0.045)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, HOME_CARRIAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3000.0,
            velocity=0.35,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast_frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

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

    with ctx.pose({lift: 0.0}):
        ctx.expect_contact(
            carriage,
            mast,
            elem_a="left_inner_guide",
            elem_b="left_rail",
            name="left guide sleeve is mounted on left rail",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a="right_inner_guide",
            elem_b="right_rail",
            name="right guide sleeve is mounted on right rail",
        )
        ctx.expect_gap(
            carriage,
            mast,
            axis="z",
            positive_elem="bottom_beam",
            negative_elem="bottom_cross",
            min_gap=0.18,
            name="carriage clears the bottom crossmember at rest",
        )
        ctx.expect_origin_gap(
            carriage,
            mast,
            axis="z",
            min_gap=HOME_CARRIAGE_Z - 1e-6,
            max_gap=HOME_CARRIAGE_Z + 1e-6,
            name="carriage home position sits on the mast centerline",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="x",
            min_overlap=0.60,
            name="carriage spans across both mast rails",
        )

    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.expect_origin_gap(
            carriage,
            mast,
            axis="z",
            min_gap=(HOME_CARRIAGE_Z + LIFT_TRAVEL) - 1e-6,
            max_gap=(HOME_CARRIAGE_Z + LIFT_TRAVEL) + 1e-6,
            name="prismatic joint lifts the carriage upward",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem="top_cross",
            negative_elem="top_beam",
            min_gap=0.05,
            name="upper travel limit leaves headroom below the mast cap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
