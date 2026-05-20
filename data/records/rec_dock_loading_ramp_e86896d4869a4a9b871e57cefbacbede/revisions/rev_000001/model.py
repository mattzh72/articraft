from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


PIT_LENGTH = 2.35
OUTER_WIDTH = 2.20
CURB_THICKNESS = 0.08
PIT_DEPTH = 0.45

DECK_LENGTH = 2.10
DECK_WIDTH = 1.96
DECK_DEPTH = 0.14
DECK_SKIN = 0.014

LIP_LENGTH = 0.36
LIP_WIDTH = 1.80
LIP_THICKNESS = 0.04
LIP_STANDOFF = 0.016

WORKING_DECK_ANGLE = pi / 2.0
WORKING_LIP_ANGLE = 2.75


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_storing_dock_leveler")

    frame_color = model.material("frame_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    deck_color = model.material("deck_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    lip_color = model.material("safety_yellow", rgba=(0.86, 0.73, 0.16, 1.0))

    frame = model.part("pit_frame")
    frame.visual(
        Box((PIT_LENGTH, OUTER_WIDTH, 0.05)),
        origin=Origin(xyz=(PIT_LENGTH / 2.0, 0.0, -PIT_DEPTH + 0.025)),
        material=frame_color,
        name="pit_floor",
    )
    frame.visual(
        Box((PIT_LENGTH, CURB_THICKNESS, PIT_DEPTH + 0.04)),
        origin=Origin(
            xyz=(
                PIT_LENGTH / 2.0,
                (OUTER_WIDTH - CURB_THICKNESS) / 2.0,
                -(PIT_DEPTH + 0.04) / 2.0 + 0.02,
            )
        ),
        material=frame_color,
        name="left_curb",
    )
    frame.visual(
        Box((PIT_LENGTH, CURB_THICKNESS, PIT_DEPTH + 0.04)),
        origin=Origin(
            xyz=(
                PIT_LENGTH / 2.0,
                -(OUTER_WIDTH - CURB_THICKNESS) / 2.0,
                -(PIT_DEPTH + 0.04) / 2.0 + 0.02,
            )
        ),
        material=frame_color,
        name="right_curb",
    )
    frame.visual(
        Box((0.34, OUTER_WIDTH, 0.12)),
        origin=Origin(xyz=(-0.17, 0.0, -0.06)),
        material=frame_color,
        name="dock_sill",
    )
    frame.visual(
        Box((0.18, OUTER_WIDTH, 0.22)),
        origin=Origin(xyz=(-0.09, 0.0, -0.11)),
        material=frame_color,
        name="rear_header",
    )
    frame.visual(
        Box((0.14, 0.16, 0.28)),
        origin=Origin(xyz=(-0.07, 0.82, -0.14)),
        material=frame_color,
        name="left_hinge_pedestal",
    )
    frame.visual(
        Box((0.14, 0.16, 0.28)),
        origin=Origin(xyz=(-0.07, -0.82, -0.14)),
        material=frame_color,
        name="right_hinge_pedestal",
    )
    frame.inertial = Inertial.from_geometry(
        Box((PIT_LENGTH, OUTER_WIDTH, PIT_DEPTH)),
        mass=650.0,
        origin=Origin(xyz=(PIT_LENGTH / 2.0, 0.0, -PIT_DEPTH / 2.0)),
    )

    deck = model.part("deck")
    deck.visual(
        Box((DECK_SKIN, DECK_WIDTH, DECK_LENGTH)),
        origin=Origin(xyz=(DECK_SKIN / 2.0, 0.0, DECK_LENGTH / 2.0)),
        material=deck_color,
        name="deck_skin",
    )
    deck.visual(
        Box((DECK_DEPTH - 0.008, 0.06, DECK_LENGTH)),
        origin=Origin(xyz=((DECK_DEPTH + 0.008) / 2.0, 0.95, DECK_LENGTH / 2.0)),
        material=deck_color,
        name="left_side_girder",
    )
    deck.visual(
        Box((DECK_DEPTH - 0.008, 0.06, DECK_LENGTH)),
        origin=Origin(xyz=((DECK_DEPTH + 0.008) / 2.0, -0.95, DECK_LENGTH / 2.0)),
        material=deck_color,
        name="right_side_girder",
    )
    deck.visual(
        Box((DECK_DEPTH - 0.010, 0.10, 1.68)),
        origin=Origin(xyz=((DECK_DEPTH + 0.010) / 2.0, -0.42, 0.84)),
        material=deck_color,
        name="left_center_beam",
    )
    deck.visual(
        Box((DECK_DEPTH - 0.010, 0.10, 1.68)),
        origin=Origin(xyz=((DECK_DEPTH + 0.010) / 2.0, 0.0, 0.84)),
        material=deck_color,
        name="center_beam",
    )
    deck.visual(
        Box((DECK_DEPTH - 0.010, 0.10, 1.68)),
        origin=Origin(xyz=((DECK_DEPTH + 0.010) / 2.0, 0.42, 0.84)),
        material=deck_color,
        name="right_center_beam",
    )
    deck.visual(
        Box((DECK_DEPTH - 0.010, DECK_WIDTH, 0.14)),
        origin=Origin(xyz=((DECK_DEPTH + 0.010) / 2.0, 0.0, 0.07)),
        material=deck_color,
        name="rear_cross_beam",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_DEPTH, DECK_WIDTH, DECK_LENGTH)),
        mass=240.0,
        origin=Origin(xyz=(DECK_DEPTH / 2.0, 0.0, DECK_LENGTH / 2.0)),
    )

    lip = model.part("lip")
    lip.visual(
        Box((LIP_THICKNESS, LIP_WIDTH, LIP_LENGTH)),
        origin=Origin(
            xyz=(LIP_STANDOFF + LIP_THICKNESS / 2.0, 0.0, -LIP_LENGTH / 2.0)
        ),
        material=lip_color,
        name="lip_plate",
    )
    lip.visual(
        Box((0.012, LIP_WIDTH, 0.04)),
        origin=Origin(xyz=(0.020, 0.0, -0.020)),
        material=lip_color,
        name="lip_hinge_leaf",
    )
    lip.visual(
        Box((0.06, LIP_WIDTH, 0.05)),
        origin=Origin(xyz=(LIP_STANDOFF + 0.03, 0.0, -LIP_LENGTH + 0.025)),
        material=lip_color,
        name="lip_toe_bar",
    )
    lip.inertial = Inertial.from_geometry(
        Box((LIP_THICKNESS + LIP_STANDOFF, LIP_WIDTH, LIP_LENGTH)),
        mass=45.0,
        origin=Origin(
            xyz=((LIP_THICKNESS + LIP_STANDOFF) / 2.0, 0.0, -LIP_LENGTH / 2.0)
        ),
    )

    model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.8,
            lower=0.0,
            upper=WORKING_DECK_ANGLE,
        ),
    )
    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(0.0, 0.0, DECK_LENGTH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2000.0,
            velocity=1.4,
            lower=0.0,
            upper=3.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    frame = object_model.get_part("pit_frame")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    deck_hinge = object_model.get_articulation("frame_to_deck")
    lip_hinge = object_model.get_articulation("deck_to_lip")

    with ctx.pose({deck_hinge: 0.0, lip_hinge: 0.0}):
        ctx.expect_origin_gap(
            lip,
            deck,
            axis="z",
            min_gap=DECK_LENGTH - 0.001,
            max_gap=DECK_LENGTH + 0.001,
            name="stored lip hinge sits at the top of the upright deck",
        )
        stored_deck_aabb = ctx.part_world_aabb(deck)

    with ctx.pose({deck_hinge: WORKING_DECK_ANGLE, lip_hinge: WORKING_LIP_ANGLE}):
        ctx.expect_origin_gap(
            lip,
            deck,
            axis="x",
            min_gap=DECK_LENGTH - 0.001,
            max_gap=DECK_LENGTH + 0.001,
            name="lowered deck carries the lip hinge out to the front edge",
        )
        ctx.expect_within(
            lip,
            deck,
            axes="y",
            margin=0.09,
            name="deployed lip stays inside the deck side curbs",
        )
        ctx.expect_gap(
            deck,
            frame,
            axis="z",
            positive_elem="deck_skin",
            negative_elem="pit_floor",
            min_gap=0.32,
            name="working deck skin spans well above the pit floor",
        )
        working_deck_aabb = ctx.part_world_aabb(deck)
        working_lip_aabb = ctx.part_world_aabb(lip)

    stored_upright = (
        stored_deck_aabb is not None
        and (stored_deck_aabb[1][2] - stored_deck_aabb[0][2]) > 2.0
        and (stored_deck_aabb[1][0] - stored_deck_aabb[0][0]) < 0.18
    )
    working_flat = (
        working_deck_aabb is not None
        and (working_deck_aabb[1][0] - working_deck_aabb[0][0]) > 2.0
        and (working_deck_aabb[1][2] - working_deck_aabb[0][2]) < 0.18
    )
    ctx.check(
        "deck pivots from upright storage to a nearly flat service surface",
        stored_upright and working_flat,
        details=f"stored={stored_deck_aabb}, working={working_deck_aabb}",
    )

    lip_bridges_forward = (
        working_lip_aabb is not None
        and working_deck_aabb is not None
        and working_lip_aabb[1][0] > working_deck_aabb[1][0] + 0.20
        and working_lip_aabb[0][2] < -0.02
        and working_lip_aabb[1][2] < 0.08
    )
    ctx.check(
        "lip projects forward and slightly downward in the working pose",
        lip_bridges_forward,
        details=f"deck={working_deck_aabb}, lip={working_lip_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
