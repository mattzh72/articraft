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


BODY_LENGTH = 0.46
BODY_WIDTH = 0.24
BODY_PLATE_THICKNESS = 0.022
BODY_SPINE_THICKNESS = 0.026
BODY_HEIGHT = BODY_PLATE_THICKNESS + BODY_SPINE_THICKNESS
BASE_RAIL_LENGTH = 0.36
BASE_RAIL_WIDTH = 0.024
BASE_RAIL_HEIGHT = 0.012
BASE_RAIL_Y = 0.075

MAIN_CARRIAGE_LENGTH = 0.18
MAIN_CARRIAGE_WIDTH = 0.17
MAIN_CARRIAGE_BODY_HEIGHT = 0.040
MAIN_STAGE_DECK_LENGTH = 0.126
MAIN_STAGE_DECK_WIDTH = 0.108
MAIN_STAGE_DECK_HEIGHT = 0.010
MAIN_UPPER_RAIL_LENGTH = 0.112
MAIN_UPPER_RAIL_WIDTH = 0.016
MAIN_UPPER_RAIL_HEIGHT = 0.010
MAIN_UPPER_RAIL_Y = 0.036

TERMINAL_CARRIAGE_LENGTH = 0.10
TERMINAL_CARRIAGE_WIDTH = 0.105
TERMINAL_CARRIAGE_BODY_HEIGHT = 0.028
TERMINAL_CARRIAGE_TOP_HEIGHT = 0.008

BODY_TO_MAIN_HOME_X = -0.07
BODY_TO_MAIN_TRAVEL = 0.16
MAIN_TO_TERMINAL_HOME_X = 0.010
MAIN_TO_TERMINAL_TRAVEL = 0.08
BASE_RAIL_TOP_Z = BODY_HEIGHT + BASE_RAIL_HEIGHT
MAIN_UPPER_RAIL_TOP_Z = (
    MAIN_CARRIAGE_BODY_HEIGHT + MAIN_STAGE_DECK_HEIGHT + MAIN_UPPER_RAIL_HEIGHT
)


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        BODY_LENGTH,
        BODY_WIDTH,
        BODY_PLATE_THICKNESS,
        centered=(True, True, False),
    )
    body = body.union(
        cq.Workplane("XY")
        .box(
            BODY_LENGTH - 0.040,
            BODY_WIDTH - 0.040,
            BODY_SPINE_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_PLATE_THICKNESS))
    )
    body = body.union(
        cq.Workplane("XY")
        .box(0.080, BODY_WIDTH - 0.050, BASE_RAIL_HEIGHT, centered=(True, True, False))
        .translate((-0.165, 0.0, BODY_HEIGHT))
    )
    body = body.cut(
        cq.Workplane("XY")
        .box(0.295, 0.082, 0.018, centered=(True, True, False))
        .translate((0.025, 0.0, BODY_HEIGHT - 0.018))
    )
    body = body.cut(
        cq.Workplane("XY")
        .box(BODY_LENGTH - 0.050, BODY_WIDTH - 0.090, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    return body


def _main_carriage_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        MAIN_CARRIAGE_LENGTH,
        MAIN_CARRIAGE_WIDTH,
        MAIN_CARRIAGE_BODY_HEIGHT,
        centered=(True, True, False),
    )
    carriage = carriage.cut(
        cq.Workplane("XY")
        .box(
            MAIN_CARRIAGE_LENGTH - 0.020,
            0.104,
            0.022,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.0))
    )
    carriage = carriage.cut(
        cq.Workplane("XY")
        .box(
            MAIN_CARRIAGE_LENGTH - 0.042,
            MAIN_CARRIAGE_WIDTH - 0.022,
            0.010,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.014))
    )
    carriage = carriage.union(
        cq.Workplane("XY")
        .box(
            MAIN_STAGE_DECK_LENGTH,
            MAIN_STAGE_DECK_WIDTH,
            MAIN_STAGE_DECK_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.015, 0.0, MAIN_CARRIAGE_BODY_HEIGHT))
    )
    carriage = carriage.union(
        cq.Workplane("XY")
        .box(0.028, MAIN_CARRIAGE_WIDTH * 0.82, 0.008, centered=(True, True, False))
        .translate((0.072, 0.0, 0.018))
    )
    return carriage


def _terminal_carriage_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        TERMINAL_CARRIAGE_LENGTH,
        TERMINAL_CARRIAGE_WIDTH,
        TERMINAL_CARRIAGE_BODY_HEIGHT,
        centered=(True, True, False),
    )
    carriage = carriage.cut(
        cq.Workplane("XY")
        .box(
            TERMINAL_CARRIAGE_LENGTH - 0.014,
            0.050,
            0.018,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.0))
    )
    carriage = carriage.cut(
        cq.Workplane("XY")
        .box(
            TERMINAL_CARRIAGE_LENGTH - 0.024,
            TERMINAL_CARRIAGE_WIDTH - 0.018,
            0.008,
            centered=(True, True, False),
        )
        .translate((-0.006, 0.0, 0.010))
    )
    carriage = carriage.union(
        cq.Workplane("XY")
        .box(
            TERMINAL_CARRIAGE_LENGTH - 0.014,
            TERMINAL_CARRIAGE_WIDTH - 0.020,
            TERMINAL_CARRIAGE_TOP_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.004, 0.0, TERMINAL_CARRIAGE_BODY_HEIGHT))
    )
    carriage = carriage.union(
        cq.Workplane("XY")
        .box(0.020, TERMINAL_CARRIAGE_WIDTH * 0.84, 0.010, centered=(True, True, False))
        .translate((0.040, 0.0, 0.014))
    )
    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_two_stage_axis")

    model.material("body_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("rail_steel", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("carriage_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("terminal_gray", rgba=(0.62, 0.65, 0.69, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material="body_black",
        name="body_shell",
    )
    body.visual(
        Box((BASE_RAIL_LENGTH, BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -BASE_RAIL_Y, BODY_HEIGHT + (BASE_RAIL_HEIGHT / 2.0))),
        material="rail_steel",
        name="left_rail",
    )
    body.visual(
        Box((BASE_RAIL_LENGTH, BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, BASE_RAIL_Y, BODY_HEIGHT + (BASE_RAIL_HEIGHT / 2.0))),
        material="rail_steel",
        name="right_rail",
    )

    main_carriage = model.part("main_carriage")
    main_carriage.visual(
        mesh_from_cadquery(_main_carriage_shape(), "main_carriage_shell"),
        material="carriage_aluminum",
        name="main_carriage_shell",
    )
    main_carriage.visual(
        Box((MAIN_UPPER_RAIL_LENGTH, MAIN_UPPER_RAIL_WIDTH, MAIN_UPPER_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.015,
                -MAIN_UPPER_RAIL_Y,
                MAIN_CARRIAGE_BODY_HEIGHT
                + MAIN_STAGE_DECK_HEIGHT
                + (MAIN_UPPER_RAIL_HEIGHT / 2.0),
            )
        ),
        material="rail_steel",
        name="upper_left_rail",
    )
    main_carriage.visual(
        Box((MAIN_UPPER_RAIL_LENGTH, MAIN_UPPER_RAIL_WIDTH, MAIN_UPPER_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.015,
                MAIN_UPPER_RAIL_Y,
                MAIN_CARRIAGE_BODY_HEIGHT
                + MAIN_STAGE_DECK_HEIGHT
                + (MAIN_UPPER_RAIL_HEIGHT / 2.0),
            )
        ),
        material="rail_steel",
        name="upper_right_rail",
    )

    terminal_carriage = model.part("terminal_carriage")
    terminal_carriage.visual(
        mesh_from_cadquery(_terminal_carriage_shape(), "terminal_carriage_shell"),
        material="terminal_gray",
        name="terminal_carriage_shell",
    )

    model.articulation(
        "body_to_main",
        ArticulationType.PRISMATIC,
        parent=body,
        child=main_carriage,
        origin=Origin(xyz=(BODY_TO_MAIN_HOME_X, 0.0, BASE_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=BODY_TO_MAIN_TRAVEL,
            effort=700.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "main_to_terminal",
        ArticulationType.PRISMATIC,
        parent=main_carriage,
        child=terminal_carriage,
        origin=Origin(xyz=(MAIN_TO_TERMINAL_HOME_X, 0.0, MAIN_UPPER_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MAIN_TO_TERMINAL_TRAVEL,
            effort=250.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    main_carriage = object_model.get_part("main_carriage")
    terminal_carriage = object_model.get_part("terminal_carriage")
    body_to_main = object_model.get_articulation("body_to_main")
    main_to_terminal = object_model.get_articulation("main_to_terminal")

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
        "body_to_main_uses_x_prismatic_axis",
        body_to_main.axis == (1.0, 0.0, 0.0),
        details=f"axis={body_to_main.axis}",
    )
    ctx.check(
        "main_to_terminal_uses_x_prismatic_axis",
        main_to_terminal.axis == (1.0, 0.0, 0.0),
        details=f"axis={main_to_terminal.axis}",
    )

    ctx.expect_contact(
        main_carriage,
        body,
        elem_b="left_rail",
        name="main_carriage_contacts_left_body_rail",
    )
    ctx.expect_contact(
        main_carriage,
        body,
        elem_b="right_rail",
        name="main_carriage_contacts_right_body_rail",
    )
    ctx.expect_contact(
        terminal_carriage,
        main_carriage,
        elem_b="upper_left_rail",
        name="terminal_carriage_contacts_left_upper_rail",
    )
    ctx.expect_contact(
        terminal_carriage,
        main_carriage,
        elem_b="upper_right_rail",
        name="terminal_carriage_contacts_right_upper_rail",
    )

    with ctx.pose({body_to_main: 0.0, main_to_terminal: 0.0}):
        ctx.expect_overlap(
            main_carriage,
            body,
            axes="xy",
            min_overlap=0.12,
            name="main_carriage_overlaps_body_planform_at_home",
        )
        ctx.expect_overlap(
            terminal_carriage,
            main_carriage,
            axes="xy",
            min_overlap=0.08,
            name="terminal_carriage_overlaps_main_planform_at_home",
        )
        ctx.expect_within(
            main_carriage,
            body,
            axes="y",
            margin=0.0,
            name="main_carriage_stays_within_body_width",
        )
        ctx.expect_within(
            terminal_carriage,
            main_carriage,
            axes="y",
            margin=0.0,
            name="terminal_carriage_stays_within_main_width",
        )

        main_home = ctx.part_world_position(main_carriage)
        terminal_home = ctx.part_world_position(terminal_carriage)

    with ctx.pose({body_to_main: BODY_TO_MAIN_TRAVEL, main_to_terminal: 0.0}):
        main_extended = ctx.part_world_position(main_carriage)
        terminal_with_stage1_extended = ctx.part_world_position(terminal_carriage)

    with ctx.pose(
        {body_to_main: BODY_TO_MAIN_TRAVEL, main_to_terminal: MAIN_TO_TERMINAL_TRAVEL}
    ):
        terminal_fully_extended = ctx.part_world_position(terminal_carriage)

    main_stage_ok = (
        main_home is not None
        and main_extended is not None
        and (main_extended[0] - main_home[0]) > 0.140
        and abs(main_extended[1] - main_home[1]) < 1e-6
        and abs(main_extended[2] - main_home[2]) < 1e-6
    )
    ctx.check(
        "main_stage_extends_forward_without_drifting",
        main_stage_ok,
        details=(
            f"home={main_home}, extended={main_extended}, "
            f"expected strong +x motion with fixed y/z"
        ),
    )

    terminal_stage_ok = (
        terminal_home is not None
        and terminal_with_stage1_extended is not None
        and terminal_fully_extended is not None
        and (terminal_fully_extended[0] - terminal_with_stage1_extended[0]) > 0.070
        and abs(terminal_fully_extended[1] - terminal_with_stage1_extended[1]) < 1e-6
        and abs(terminal_fully_extended[2] - terminal_with_stage1_extended[2]) < 1e-6
    )
    ctx.check(
        "terminal_stage_extends_forward_without_drifting",
        terminal_stage_ok,
        details=(
            f"home={terminal_home}, stage1_extended={terminal_with_stage1_extended}, "
            f"fully_extended={terminal_fully_extended}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
