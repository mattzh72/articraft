from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


COLUMN_WIDTH = 0.18
COLUMN_DEPTH = 0.16
COLUMN_CLEAR_HEIGHT = 2.90
COLUMN_CENTER_SPACING = 1.72

FOOT_HEIGHT = 0.18
FOOT_WIDTH = 2.16
FOOT_DEPTH = 0.38
FOOT_CENTER_Y = -0.02

BRIDGE_HEIGHT = 0.20
BRIDGE_WIDTH = 1.96
BRIDGE_DEPTH = 0.18

GUIDE_CLEARANCE = 0.002
SLEEVE_SIDE_WALL = 0.022
SLEEVE_FRONT_WALL = 0.038
SLEEVE_HEIGHT = 0.62
SLEEVE_INNER_WIDTH = COLUMN_WIDTH + (2.0 * GUIDE_CLEARANCE)
SLEEVE_INNER_DEPTH = COLUMN_DEPTH + (2.0 * GUIDE_CLEARANCE)
SLEEVE_OUTER_WIDTH = SLEEVE_INNER_WIDTH + (2.0 * SLEEVE_SIDE_WALL)
SLEEVE_OUTER_DEPTH = SLEEVE_INNER_DEPTH + SLEEVE_FRONT_WALL
SLEEVE_FRONT_CENTER_Y = (COLUMN_DEPTH / 2.0) + (SLEEVE_FRONT_WALL / 2.0)
SLEEVE_WALL_OFFSET_X = (COLUMN_WIDTH / 2.0) + GUIDE_CLEARANCE + (SLEEVE_SIDE_WALL / 2.0)

FRAME_SPAN = COLUMN_CENTER_SPACING - SLEEVE_OUTER_WIDTH
PLATFORM_WIDTH = FRAME_SPAN - 0.10

CARRIAGE_HOME_Z = 0.82
CARRIAGE_TRAVEL = 1.88


def _sleeve_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(SLEEVE_OUTER_WIDTH, SLEEVE_OUTER_DEPTH, SLEEVE_HEIGHT)
        .translate((0.0, SLEEVE_CENTER_Y, 0.0))
    )
    cavity = cq.Workplane("XY").box(
        SLEEVE_INNER_WIDTH,
        SLEEVE_INNER_DEPTH,
        SLEEVE_HEIGHT + 0.01,
    )

    front_pad = cq.Workplane("XY").box(
        COLUMN_WIDTH * 0.62,
        GUIDE_CLEARANCE,
        SLEEVE_HEIGHT * 0.72,
    ).translate((0.0, (COLUMN_DEPTH / 2.0) + (GUIDE_CLEARANCE / 2.0), 0.0))

    side_pad_depth = 0.09
    side_pad_y = 0.02
    left_pad = cq.Workplane("XY").box(
        GUIDE_CLEARANCE,
        side_pad_depth,
        SLEEVE_HEIGHT * 0.72,
    ).translate(
        (
            -(COLUMN_WIDTH / 2.0) - (GUIDE_CLEARANCE / 2.0),
            side_pad_y,
            0.0,
        )
    )
    right_pad = cq.Workplane("XY").box(
        GUIDE_CLEARANCE,
        side_pad_depth,
        SLEEVE_HEIGHT * 0.72,
    ).translate(
        (
            (COLUMN_WIDTH / 2.0) + (GUIDE_CLEARANCE / 2.0),
            side_pad_y,
            0.0,
        )
    )

    return outer.cut(cavity).union(front_pad).union(left_pad).union(right_pad)


def _carriage_frame_shape() -> cq.Workplane:
    lower_beam = cq.Workplane("XY").box(FRAME_SPAN, 0.12, 0.16).translate((0.0, 0.14, -0.18))
    upper_beam = cq.Workplane("XY").box(FRAME_SPAN, 0.09, 0.09).translate((0.0, 0.11, 0.23))
    front_panel = cq.Workplane("XY").box(FRAME_SPAN - 0.16, 0.03, 0.34).translate((0.0, 0.17, 0.04))
    return lower_beam.union(upper_beam).union(front_panel)


def _platform_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").box(PLATFORM_WIDTH, 0.36, 0.06).translate((0.0, 0.36, -0.29))
    lip = cq.Workplane("XY").box(PLATFORM_WIDTH, 0.03, 0.07).translate((0.0, 0.525, -0.245))
    return deck.union(lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_tied_dual_column_lift")

    model.material("frame_gray", rgba=(0.27, 0.30, 0.34, 1.0))
    model.material("carriage_yellow", rgba=(0.92, 0.74, 0.10, 1.0))
    model.material("deck_charcoal", rgba=(0.22, 0.23, 0.25, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_CLEAR_HEIGHT)),
        origin=Origin(
            xyz=(
                -(COLUMN_CENTER_SPACING / 2.0),
                0.0,
                FOOT_HEIGHT + (COLUMN_CLEAR_HEIGHT / 2.0),
            )
        ),
        material="frame_gray",
        name="left_column",
    )
    frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_CLEAR_HEIGHT)),
        origin=Origin(
            xyz=(
                COLUMN_CENTER_SPACING / 2.0,
                0.0,
                FOOT_HEIGHT + (COLUMN_CLEAR_HEIGHT / 2.0),
            )
        ),
        material="frame_gray",
        name="right_column",
    )
    frame.visual(
        Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(0.0, FOOT_CENTER_Y, FOOT_HEIGHT / 2.0)),
        material="frame_gray",
        name="lower_foot",
    )
    frame.visual(
        Box((BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                FOOT_HEIGHT + COLUMN_CLEAR_HEIGHT + (BRIDGE_HEIGHT / 2.0),
            )
        ),
        material="frame_gray",
        name="upper_bridge",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((SLEEVE_OUTER_WIDTH, SLEEVE_FRONT_WALL, SLEEVE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(COLUMN_CENTER_SPACING / 2.0),
                SLEEVE_FRONT_CENTER_Y,
                0.0,
            )
        ),
        material="carriage_yellow",
        name="left_sleeve_front",
    )
    carriage.visual(
        Box((SLEEVE_SIDE_WALL, SLEEVE_INNER_DEPTH, SLEEVE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(COLUMN_CENTER_SPACING / 2.0) + SLEEVE_WALL_OFFSET_X,
                0.0,
                0.0,
            )
        ),
        material="carriage_yellow",
        name="left_sleeve_inboard",
    )
    carriage.visual(
        Box((SLEEVE_SIDE_WALL, SLEEVE_INNER_DEPTH, SLEEVE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(COLUMN_CENTER_SPACING / 2.0) - SLEEVE_WALL_OFFSET_X,
                0.0,
                0.0,
            )
        ),
        material="carriage_yellow",
        name="left_sleeve_outboard",
    )
    carriage.visual(
        Box((SLEEVE_OUTER_WIDTH, SLEEVE_FRONT_WALL, SLEEVE_HEIGHT)),
        origin=Origin(
            xyz=(
                COLUMN_CENTER_SPACING / 2.0,
                SLEEVE_FRONT_CENTER_Y,
                0.0,
            )
        ),
        material="carriage_yellow",
        name="right_sleeve_front",
    )
    carriage.visual(
        Box((SLEEVE_SIDE_WALL, SLEEVE_INNER_DEPTH, SLEEVE_HEIGHT)),
        origin=Origin(
            xyz=(
                (COLUMN_CENTER_SPACING / 2.0) - SLEEVE_WALL_OFFSET_X,
                0.0,
                0.0,
            )
        ),
        material="carriage_yellow",
        name="right_sleeve_inboard",
    )
    carriage.visual(
        Box((SLEEVE_SIDE_WALL, SLEEVE_INNER_DEPTH, SLEEVE_HEIGHT)),
        origin=Origin(
            xyz=(
                (COLUMN_CENTER_SPACING / 2.0) + SLEEVE_WALL_OFFSET_X,
                0.0,
                0.0,
            )
        ),
        material="carriage_yellow",
        name="right_sleeve_outboard",
    )
    carriage.visual(
        mesh_from_cadquery(_carriage_frame_shape(), "carriage_frame"),
        material="carriage_yellow",
        name="carriage_frame",
    )
    carriage.visual(
        mesh_from_cadquery(_platform_shape(), "platform_deck"),
        material="deck_charcoal",
        name="platform_deck",
    )

    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("carriage_lift")

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
            frame,
            elem_a="left_sleeve_front",
            elem_b="left_column",
            contact_tol=0.003,
            name="left_sleeve_guides_left_column_at_home",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="right_sleeve_front",
            elem_b="right_column",
            contact_tol=0.003,
            name="right_sleeve_guides_right_column_at_home",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="platform_deck",
            negative_elem="lower_foot",
            min_gap=0.25,
            name="platform_starts_above_lower_foot",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="x",
            min_overlap=1.90,
            name="carriage_spans_between_columns",
        )

    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="left_sleeve_front",
            elem_b="left_column",
            contact_tol=0.003,
            name="left_sleeve_guides_left_column_at_top",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="right_sleeve_front",
            elem_b="right_column",
            contact_tol=0.003,
            name="right_sleeve_guides_right_column_at_top",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="upper_bridge",
            negative_elem="left_sleeve_front",
            min_gap=0.03,
            name="upper_bridge_clears_carriage_at_top",
        )

    with ctx.pose({lift: 0.0}):
        home_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        top_pos = ctx.part_world_position(carriage)

    move_ok = (
        home_pos is not None
        and top_pos is not None
        and abs(top_pos[0] - home_pos[0]) <= 1e-4
        and abs(top_pos[1] - home_pos[1]) <= 1e-4
        and abs((top_pos[2] - home_pos[2]) - CARRIAGE_TRAVEL) <= 1e-4
    )
    ctx.check(
        "carriage_moves_only_vertically",
        move_ok,
        details=f"home={home_pos}, top={top_pos}, expected_dz={CARRIAGE_TRAVEL}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
