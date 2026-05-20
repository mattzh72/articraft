from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WALL_PLATE_T = 0.010
WALL_PLATE_W = 0.120
WALL_PLATE_H = 0.220

BRACKET_ARM_L = 0.048
BRACKET_ARM_W = 0.040
BRACKET_ARM_H = 0.040

PIVOT_CENTER_X = WALL_PLATE_T + BRACKET_ARM_L + 0.007
PIVOT_SEAT_R = 0.020
PIVOT_SEAT_T = 0.010
PIVOT_SEAT_CENTER_Z = BRACKET_ARM_H / 2.0 + PIVOT_SEAT_T / 2.0
PIVOT_PLATE_R = 0.026
PIVOT_PLATE_T = 0.007
PIVOT_PLATE_CENTER_Z = (
    PIVOT_SEAT_CENTER_Z + PIVOT_SEAT_T / 2.0 + PIVOT_PLATE_T / 2.0
)

MAIN_BEAM_L = 0.520
MAIN_BEAM_W = 0.036
MAIN_BEAM_H = 0.022
MAIN_BEAM_START_X = 0.020
MAIN_BEAM_CENTER_Z = (
    PIVOT_PLATE_CENTER_Z + PIVOT_PLATE_T / 2.0 + MAIN_BEAM_H / 2.0 - 0.001
)

CARRIAGE_BODY_CENTER_X = 0.330
CARRIAGE_BODY_L = 0.150
CARRIAGE_BODY_W = 0.078
CARRIAGE_BODY_H = 0.048

RUNNER_L = 0.078
RUNNER_W = 0.014
RUNNER_H = 0.012
RUNNER_Y = 0.011
RUNNER_CENTER_Z = MAIN_BEAM_CENTER_Z + MAIN_BEAM_H / 2.0 + RUNNER_H / 2.0

CARRIAGE_BODY_CENTER_Z = RUNNER_CENTER_Z + RUNNER_H / 2.0 + CARRIAGE_BODY_H / 2.0

FRONT_PLATE_T = 0.010
FRONT_PLATE_W = 0.102
FRONT_PLATE_H = 0.070
FRONT_PLATE_CENTER_Z = CARRIAGE_BODY_CENTER_Z + 0.010

SWING_LIMIT = 1.35
EXTENSION_MAX = 0.160


def _wall_bracket_body() -> cq.Workplane:
    wall_plate = (
        cq.Workplane("XY")
        .box(WALL_PLATE_T, WALL_PLATE_W, WALL_PLATE_H)
        .translate((WALL_PLATE_T / 2.0, 0.0, 0.0))
    )
    wall_plate = (
        wall_plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.035, -0.070),
                (-0.035, 0.070),
                (0.035, -0.070),
                (0.035, 0.070),
            ]
        )
        .hole(0.012)
    )

    root_arm = (
        cq.Workplane("XY")
        .box(BRACKET_ARM_L, BRACKET_ARM_W, BRACKET_ARM_H)
        .translate((WALL_PLATE_T + BRACKET_ARM_L / 2.0 - 0.001, 0.0, 0.0))
    )

    return wall_plate.union(root_arm)


def _pivot_seat() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(PIVOT_SEAT_R)
        .extrude(PIVOT_SEAT_T)
        .translate((PIVOT_CENTER_X, 0.0, PIVOT_SEAT_CENTER_Z - PIVOT_SEAT_T / 2.0))
    )


def _pivot_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(PIVOT_PLATE_R)
        .extrude(PIVOT_PLATE_T)
        .translate((0.0, 0.0, PIVOT_PLATE_CENTER_Z - PIVOT_PLATE_T / 2.0))
    )


def _main_beam() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(MAIN_BEAM_L, MAIN_BEAM_W, MAIN_BEAM_H)
        .translate((MAIN_BEAM_START_X + MAIN_BEAM_L / 2.0, 0.0, MAIN_BEAM_CENTER_Z))
    )


def _carriage_body() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(CARRIAGE_BODY_L, CARRIAGE_BODY_W, CARRIAGE_BODY_H)
        .translate((CARRIAGE_BODY_CENTER_X, 0.0, CARRIAGE_BODY_CENTER_Z))
    )


def _runner_pad(y_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(RUNNER_L, RUNNER_W, RUNNER_H)
        .translate((CARRIAGE_BODY_CENTER_X, y_center, RUNNER_CENTER_Z))
    )


def _front_plate() -> cq.Workplane:
    front_plate_center_x = (
        CARRIAGE_BODY_CENTER_X + CARRIAGE_BODY_L / 2.0 + FRONT_PLATE_T / 2.0
    )
    return (
        cq.Workplane("XY")
        .box(FRONT_PLATE_T, FRONT_PLATE_W, FRONT_PLATE_H)
        .translate((front_plate_center_x, 0.0, FRONT_PLATE_CENTER_Z))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_hinged_support_arm")

    bracket_steel = model.material("bracket_steel", color=(0.22, 0.23, 0.25))
    arm_steel = model.material("arm_steel", color=(0.33, 0.35, 0.38))
    carriage_finish = model.material("carriage_finish", color=(0.45, 0.47, 0.50))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        mesh_from_cadquery(_wall_bracket_body(), "wall_bracket_body"),
        material=bracket_steel,
        name="wall_plate",
    )
    wall_bracket.visual(
        mesh_from_cadquery(_pivot_seat(), "wall_bracket_pivot_seat"),
        material=arm_steel,
        name="pivot_seat",
    )

    main_link = model.part("main_link")
    main_link.visual(
        mesh_from_cadquery(_pivot_plate(), "main_link_pivot_plate"),
        material=arm_steel,
        name="pivot_plate",
    )
    main_link.visual(
        mesh_from_cadquery(_main_beam(), "main_link_beam"),
        material=arm_steel,
        name="main_beam",
    )

    tip_carriage = model.part("tip_carriage")
    tip_carriage.visual(
        mesh_from_cadquery(_carriage_body(), "tip_carriage_body"),
        material=carriage_finish,
        name="carriage_body",
    )
    tip_carriage.visual(
        mesh_from_cadquery(_runner_pad(-RUNNER_Y), "tip_carriage_left_runner"),
        material=arm_steel,
        name="left_runner",
    )
    tip_carriage.visual(
        mesh_from_cadquery(_runner_pad(RUNNER_Y), "tip_carriage_right_runner"),
        material=arm_steel,
        name="right_runner",
    )
    tip_carriage.visual(
        mesh_from_cadquery(_front_plate(), "tip_carriage_front_plate"),
        material=carriage_finish,
        name="front_plate",
    )

    model.articulation(
        "bracket_to_main_link",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=main_link,
        origin=Origin(xyz=(PIVOT_CENTER_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=-SWING_LIMIT,
            upper=SWING_LIMIT,
        ),
    )

    model.articulation(
        "main_link_to_tip_carriage",
        ArticulationType.PRISMATIC,
        parent=main_link,
        child=tip_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=0.0,
            upper=EXTENSION_MAX,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_bracket = object_model.get_part("wall_bracket")
    main_link = object_model.get_part("main_link")
    tip_carriage = object_model.get_part("tip_carriage")
    swing = object_model.get_articulation("bracket_to_main_link")
    extension = object_model.get_articulation("main_link_to_tip_carriage")

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

    swing_ok = (
        swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(swing.axis) == (0.0, 0.0, 1.0)
        and swing.motion_limits is not None
        and swing.motion_limits.lower == -SWING_LIMIT
        and swing.motion_limits.upper == SWING_LIMIT
    )
    extension_ok = (
        extension.articulation_type == ArticulationType.PRISMATIC
        and tuple(extension.axis) == (1.0, 0.0, 0.0)
        and extension.motion_limits is not None
        and extension.motion_limits.lower == 0.0
        and extension.motion_limits.upper == EXTENSION_MAX
    )
    ctx.check(
        "motion_stack_reads_swing_then_extension",
        swing_ok and extension_ok,
        "Expected a wall-bracket revolute joint followed by a main-link prismatic carriage axis.",
    )

    ctx.expect_contact(
        main_link,
        wall_bracket,
        elem_a="pivot_plate",
        elem_b="pivot_seat",
        name="hinge_plate_is_seated_on_bracket",
    )
    ctx.expect_contact(
        main_link,
        tip_carriage,
        elem_a="main_beam",
        elem_b="left_runner",
        name="left_runner_contacts_beam_retracted",
    )
    ctx.expect_contact(
        main_link,
        tip_carriage,
        elem_a="main_beam",
        elem_b="right_runner",
        name="right_runner_contacts_beam_retracted",
    )

    with ctx.pose({extension: EXTENSION_MAX}):
        ctx.expect_contact(
            main_link,
            tip_carriage,
            elem_a="main_beam",
            elem_b="left_runner",
            name="left_runner_contacts_beam_extended",
        )
        ctx.expect_contact(
            main_link,
            tip_carriage,
            elem_a="main_beam",
            elem_b="right_runner",
            name="right_runner_contacts_beam_extended",
        )
        ctx.expect_overlap(
            main_link,
            tip_carriage,
            axes="xy",
            min_overlap=0.030,
            elem_a="main_beam",
            elem_b="carriage_body",
            name="carriage_body_stays_over_beam_when_extended",
        )
        ctx.expect_gap(
            tip_carriage,
            main_link,
            axis="x",
            min_gap=0.020,
            max_gap=0.050,
            positive_elem="front_plate",
            negative_elem="main_beam",
            name="front_plate_projects_beyond_beam_tip_when_extended",
        )

    with ctx.pose({swing: 0.9}):
        ctx.expect_contact(
            main_link,
            wall_bracket,
            elem_a="pivot_plate",
            elem_b="pivot_seat",
            name="hinge_plate_stays_seated_when_swung",
        )

    with ctx.pose({swing: 0.9, extension: EXTENSION_MAX}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no_part_overlap_in_swung_and_extended_pose"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
