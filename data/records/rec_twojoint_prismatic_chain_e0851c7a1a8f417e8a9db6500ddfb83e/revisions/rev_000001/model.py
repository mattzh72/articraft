from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.52
BASE_WIDTH = 0.18
BASE_THICKNESS = 0.014

OUTER_RAIL_START = 0.05
OUTER_RAIL_LENGTH = 0.42
OUTER_RAIL_WIDTH = 0.024
OUTER_RAIL_HEIGHT = 0.016
OUTER_RAIL_Y = 0.055
OUTER_RAIL_TOP_Z = BASE_THICKNESS + OUTER_RAIL_HEIGHT

MIDDLE_BODY_LENGTH = 0.26
MIDDLE_BODY_WIDTH = 0.14
MIDDLE_SHOE_START = 0.02
MIDDLE_SHOE_LENGTH = 0.19
MIDDLE_SHOE_WIDTH = 0.028
MIDDLE_SHOE_HEIGHT = 0.014
MIDDLE_TOP_DECK_HEIGHT = 0.016
MIDDLE_TOP_RAIL_START = 0.06
MIDDLE_TOP_RAIL_LENGTH = 0.19
MIDDLE_TOP_RAIL_WIDTH = 0.018
MIDDLE_TOP_RAIL_HEIGHT = 0.010
MIDDLE_TOP_RAIL_Y = 0.032
MIDDLE_TOP_RAIL_TOP_Z = (
    MIDDLE_SHOE_HEIGHT + MIDDLE_TOP_DECK_HEIGHT + MIDDLE_TOP_RAIL_HEIGHT
)

INNER_BODY_LENGTH = 0.15
INNER_BODY_WIDTH = 0.10
INNER_SHOE_START = 0.015
INNER_SHOE_LENGTH = 0.11
INNER_SHOE_WIDTH = 0.022
INNER_SHOE_HEIGHT = 0.012
INNER_TOP_PAD_START = 0.025
INNER_TOP_PAD_LENGTH = 0.10
INNER_TOP_PAD_WIDTH = 0.060
INNER_TOP_PAD_HEIGHT = 0.010
INNER_TOTAL_HEIGHT = INNER_SHOE_HEIGHT + 0.014 + INNER_TOP_PAD_HEIGHT

OUTER_TO_MIDDLE_HOME_X = 0.06
OUTER_TO_MIDDLE_TRAVEL = 0.18
MIDDLE_TO_INNER_HOME_X = 0.055
MIDDLE_TO_INNER_TRAVEL = 0.07


def _outer_guide_shape():
    base = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(False, True, False),
    )

    rail_proto = (
        cq.Workplane("XY")
        .box(
            OUTER_RAIL_LENGTH,
            OUTER_RAIL_WIDTH,
            OUTER_RAIL_HEIGHT,
            centered=(False, True, False),
        )
        .edges(">Z and |X")
        .fillet(0.002)
    )
    left_rail = rail_proto.translate(
        (OUTER_RAIL_START, OUTER_RAIL_Y, BASE_THICKNESS)
    )
    right_rail = rail_proto.translate(
        (OUTER_RAIL_START, -OUTER_RAIL_Y, BASE_THICKNESS)
    )

    front_keeper = cq.Workplane("XY").box(
        0.02,
        0.065,
        0.008,
        centered=(False, True, False),
    ).translate((0.012, 0.0, BASE_THICKNESS))
    rear_keeper = cq.Workplane("XY").box(
        0.02,
        0.065,
        0.008,
        centered=(False, True, False),
    ).translate((BASE_LENGTH - 0.032, 0.0, BASE_THICKNESS))

    body = base.union(left_rail).union(right_rail).union(front_keeper).union(rear_keeper)

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (0.04, -0.07),
                (0.04, 0.07),
                (BASE_LENGTH - 0.04, -0.07),
                (BASE_LENGTH - 0.04, 0.07),
            ]
        )
        .circle(0.006)
        .extrude(BASE_THICKNESS + 0.004)
    )
    return body.cut(mount_holes)


def _middle_carriage_shape():
    left_shoe = cq.Workplane("XY").box(
        MIDDLE_SHOE_LENGTH,
        MIDDLE_SHOE_WIDTH,
        MIDDLE_SHOE_HEIGHT,
        centered=(False, True, False),
    ).translate((MIDDLE_SHOE_START, OUTER_RAIL_Y, 0.0))
    right_shoe = cq.Workplane("XY").box(
        MIDDLE_SHOE_LENGTH,
        MIDDLE_SHOE_WIDTH,
        MIDDLE_SHOE_HEIGHT,
        centered=(False, True, False),
    ).translate((MIDDLE_SHOE_START, -OUTER_RAIL_Y, 0.0))

    deck = cq.Workplane("XY").box(
        MIDDLE_BODY_LENGTH,
        MIDDLE_BODY_WIDTH,
        MIDDLE_TOP_DECK_HEIGHT,
        centered=(False, True, False),
    ).translate((0.0, 0.0, MIDDLE_SHOE_HEIGHT))

    top_rail_proto = (
        cq.Workplane("XY")
        .box(
            MIDDLE_TOP_RAIL_LENGTH,
            MIDDLE_TOP_RAIL_WIDTH,
            MIDDLE_TOP_RAIL_HEIGHT,
            centered=(False, True, False),
        )
        .edges(">Z and |X")
        .fillet(0.0015)
    )
    left_top_rail = top_rail_proto.translate(
        (MIDDLE_TOP_RAIL_START, MIDDLE_TOP_RAIL_Y, MIDDLE_SHOE_HEIGHT + MIDDLE_TOP_DECK_HEIGHT)
    )
    right_top_rail = top_rail_proto.translate(
        (MIDDLE_TOP_RAIL_START, -MIDDLE_TOP_RAIL_Y, MIDDLE_SHOE_HEIGHT + MIDDLE_TOP_DECK_HEIGHT)
    )

    body = deck.union(left_shoe).union(right_shoe).union(left_top_rail).union(right_top_rail)

    pocket = cq.Workplane("XY").box(
        0.11,
        0.034,
        0.006,
        centered=(False, True, False),
    ).translate((0.10, 0.0, MIDDLE_SHOE_HEIGHT + MIDDLE_TOP_DECK_HEIGHT - 0.006))
    return body.cut(pocket)


def _inner_carriage_shape():
    left_shoe = cq.Workplane("XY").box(
        INNER_SHOE_LENGTH,
        INNER_SHOE_WIDTH,
        INNER_SHOE_HEIGHT,
        centered=(False, True, False),
    ).translate((INNER_SHOE_START, MIDDLE_TOP_RAIL_Y, 0.0))
    right_shoe = cq.Workplane("XY").box(
        INNER_SHOE_LENGTH,
        INNER_SHOE_WIDTH,
        INNER_SHOE_HEIGHT,
        centered=(False, True, False),
    ).translate((INNER_SHOE_START, -MIDDLE_TOP_RAIL_Y, 0.0))

    plate = cq.Workplane("XY").box(
        INNER_BODY_LENGTH,
        INNER_BODY_WIDTH,
        0.014,
        centered=(False, True, False),
    ).translate((0.0, 0.0, INNER_SHOE_HEIGHT))

    top_pad = cq.Workplane("XY").box(
        INNER_TOP_PAD_LENGTH,
        INNER_TOP_PAD_WIDTH,
        INNER_TOP_PAD_HEIGHT,
        centered=(False, True, False),
    ).translate((INNER_TOP_PAD_START, 0.0, INNER_SHOE_HEIGHT + 0.014))

    body = plate.union(left_shoe).union(right_shoe).union(top_pad)

    tool_pocket = cq.Workplane("XY").box(
        0.055,
        0.026,
        0.004,
        centered=(False, True, False),
    ).translate((0.0475, 0.0, INNER_TOTAL_HEIGHT - 0.004))
    return body.cut(tool_pocket)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_linear_stack")

    model.material("grounded_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("carriage_dark", rgba=(0.27, 0.30, 0.34, 1.0))
    model.material("stage_silver", rgba=(0.66, 0.69, 0.73, 1.0))

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        mesh_from_cadquery(_outer_guide_shape(), "outer_guide"),
        material="grounded_aluminum",
        name="outer_guide_shell",
    )
    outer_guide.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, OUTER_RAIL_TOP_Z)),
        mass=7.5,
        origin=Origin(xyz=(BASE_LENGTH / 2.0, 0.0, OUTER_RAIL_TOP_Z / 2.0)),
    )

    middle_carriage = model.part("middle_carriage")
    middle_carriage.visual(
        mesh_from_cadquery(_middle_carriage_shape(), "middle_carriage"),
        material="carriage_dark",
        name="middle_carriage_shell",
    )
    middle_carriage.inertial = Inertial.from_geometry(
        Box((MIDDLE_BODY_LENGTH, MIDDLE_BODY_WIDTH, MIDDLE_TOP_RAIL_TOP_Z)),
        mass=2.4,
        origin=Origin(
            xyz=(
                MIDDLE_BODY_LENGTH / 2.0,
                0.0,
                MIDDLE_TOP_RAIL_TOP_Z / 2.0,
            )
        ),
    )

    inner_carriage = model.part("inner_carriage")
    inner_carriage.visual(
        mesh_from_cadquery(_inner_carriage_shape(), "inner_carriage"),
        material="stage_silver",
        name="inner_carriage_shell",
    )
    inner_carriage.inertial = Inertial.from_geometry(
        Box((INNER_BODY_LENGTH, INNER_BODY_WIDTH, INNER_TOTAL_HEIGHT)),
        mass=1.0,
        origin=Origin(xyz=(INNER_BODY_LENGTH / 2.0, 0.0, INNER_TOTAL_HEIGHT / 2.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=middle_carriage,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME_X, 0.0, OUTER_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.30,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_carriage,
        child=inner_carriage,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME_X, 0.0, MIDDLE_TOP_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.25,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_guide = object_model.get_part("outer_guide")
    middle_carriage = object_model.get_part("middle_carriage")
    inner_carriage = object_model.get_part("inner_carriage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        "serial joints are prismatic +x slides",
        outer_to_middle.axis == (1.0, 0.0, 0.0)
        and middle_to_inner.axis == (1.0, 0.0, 0.0)
        and outer_to_middle.motion_limits is not None
        and middle_to_inner.motion_limits is not None
        and outer_to_middle.motion_limits.lower == 0.0
        and middle_to_inner.motion_limits.lower == 0.0,
        details="Both moving stages should translate forward along +X from their home pose.",
    )

    ctx.expect_contact(
        middle_carriage,
        outer_guide,
        contact_tol=1e-6,
        name="middle carriage bears on grounded outer guide",
    )
    ctx.expect_contact(
        inner_carriage,
        middle_carriage,
        contact_tol=1e-6,
        name="inner carriage bears on middle carriage guide",
    )
    ctx.expect_within(
        middle_carriage,
        outer_guide,
        axes="y",
        margin=0.0,
        name="middle carriage stays laterally inside outer guide footprint",
    )
    ctx.expect_within(
        inner_carriage,
        middle_carriage,
        axes="y",
        margin=0.0,
        name="inner carriage stays laterally inside middle carriage footprint",
    )
    ctx.expect_gap(
        inner_carriage,
        outer_guide,
        axis="z",
        min_gap=0.035,
        name="inner carriage is stacked above grounded guide",
    )

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL, middle_to_inner: MIDDLE_TO_INNER_TRAVEL}):
        ctx.expect_contact(
            middle_carriage,
            outer_guide,
            contact_tol=1e-6,
            name="outer stage stays supported at full extension",
        )
        ctx.expect_contact(
            inner_carriage,
            middle_carriage,
            contact_tol=1e-6,
            name="inner stage stays supported at full extension",
        )
        ctx.expect_origin_gap(
            middle_carriage,
            outer_guide,
            axis="x",
            min_gap=OUTER_TO_MIDDLE_HOME_X + OUTER_TO_MIDDLE_TRAVEL - 1e-6,
            name="middle carriage extends forward in +x",
        )
        ctx.expect_origin_gap(
            inner_carriage,
            outer_guide,
            axis="x",
            min_gap=OUTER_TO_MIDDLE_HOME_X
            + OUTER_TO_MIDDLE_TRAVEL
            + MIDDLE_TO_INNER_HOME_X
            + MIDDLE_TO_INNER_TRAVEL
            - 1e-6,
            name="inner carriage extends farther forward than the middle stage",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
