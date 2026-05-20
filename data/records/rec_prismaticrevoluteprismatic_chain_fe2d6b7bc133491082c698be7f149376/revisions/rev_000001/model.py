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


BASE_LENGTH = 0.60
BASE_WIDTH = 0.18
BASE_THICKNESS = 0.016
RAIL_LENGTH = 0.48
RAIL_WIDTH = 0.03
RAIL_HEIGHT = 0.02
TRACK_Y_OFFSET = 0.04

CARRIAGE_FOOT_LENGTH = 0.09
CARRIAGE_FOOT_WIDTH = 0.022
CARRIAGE_FOOT_HEIGHT = 0.012
CARRIAGE_BRIDGE_LENGTH = 0.09
CARRIAGE_BRIDGE_WIDTH = 0.11
CARRIAGE_BRIDGE_HEIGHT = 0.014
CARRIAGE_EAR_LENGTH = 0.018
CARRIAGE_EAR_WIDTH = 0.008
CARRIAGE_EAR_HEIGHT = 0.04
CARRIAGE_EAR_OFFSET_Y = 0.013
HINGE_X = 0.0
HINGE_Z = 0.0
PIN_RADIUS = 0.006
PIN_LENGTH = 0.012
CARRIAGE_HINGE_HEIGHT = BASE_THICKNESS + RAIL_HEIGHT + 0.026

BRACKET_EYE_OUTER_RADIUS = 0.009
BRACKET_NECK_LENGTH = 0.024
BRACKET_NECK_WIDTH = 0.010
BRACKET_NECK_HEIGHT = 0.016
BRACKET_BRACE_LENGTH = 0.014
BRACKET_BRACE_WIDTH = 0.012
BRACKET_BRACE_HEIGHT = 0.010
BRACKET_REAR_BLOCK_LENGTH = 0.020
BRACKET_REAR_BLOCK_WIDTH = 0.014
BRACKET_REAR_BLOCK_HEIGHT = 0.012
GUIDE_LENGTH = 0.11
GUIDE_CENTER_X = 0.105
GUIDE_FLOOR_WIDTH = 0.016
GUIDE_FLOOR_HEIGHT = 0.004
GUIDE_FLOOR_Z = 0.008
GUIDE_RAIL_WIDTH = 0.004
GUIDE_RAIL_HEIGHT = 0.018
GUIDE_RAIL_OFFSET_Y = 0.008
GUIDE_RAIL_CENTER_Z = 0.019
SLIDER_ORIGIN_X = 0.07
SLIDER_ORIGIN_Z = 0.015

SLIDER_ROD_LENGTH = 0.085
SLIDER_ROD_WIDTH = 0.010
SLIDER_ROD_HEIGHT = 0.01
SLIDER_PAD_LENGTH = 0.020
SLIDER_PAD_WIDTH = 0.012
SLIDER_PAD_HEIGHT = 0.012

BASE_SLIDE_LIMIT = 0.26
HINGE_LIMIT = 1.15
TERMINAL_SLIDE_LIMIT = 0.07


def _make_base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
    ).translate((0.0, 0.0, BASE_THICKNESS / 2.0))

    left_track = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_WIDTH,
        RAIL_HEIGHT,
    ).translate((0.0, TRACK_Y_OFFSET, BASE_THICKNESS + RAIL_HEIGHT / 2.0))
    right_track = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_WIDTH,
        RAIL_HEIGHT,
    ).translate((0.0, -TRACK_Y_OFFSET, BASE_THICKNESS + RAIL_HEIGHT / 2.0))

    return plate.union(left_track).union(right_track)


def _make_carriage_shape() -> cq.Workplane:
    left_foot = cq.Workplane("XY").box(
        CARRIAGE_FOOT_LENGTH,
        CARRIAGE_FOOT_WIDTH,
        CARRIAGE_FOOT_HEIGHT,
    ).translate((-CARRIAGE_FOOT_LENGTH / 2.0, TRACK_Y_OFFSET, -0.020))
    right_foot = cq.Workplane("XY").box(
        CARRIAGE_FOOT_LENGTH,
        CARRIAGE_FOOT_WIDTH,
        CARRIAGE_FOOT_HEIGHT,
    ).translate((-CARRIAGE_FOOT_LENGTH / 2.0, -TRACK_Y_OFFSET, -0.020))

    bridge = cq.Workplane("XY").box(
        CARRIAGE_BRIDGE_LENGTH,
        CARRIAGE_BRIDGE_WIDTH,
        CARRIAGE_BRIDGE_HEIGHT,
    ).translate((-CARRIAGE_BRIDGE_LENGTH / 2.0, 0.0, -0.009))

    left_ear = cq.Workplane("XY").box(
        CARRIAGE_EAR_LENGTH,
        CARRIAGE_EAR_WIDTH,
        CARRIAGE_EAR_HEIGHT,
    ).translate((-CARRIAGE_EAR_LENGTH / 2.0, CARRIAGE_EAR_OFFSET_Y, -0.010))
    right_ear = cq.Workplane("XY").box(
        CARRIAGE_EAR_LENGTH,
        CARRIAGE_EAR_WIDTH,
        CARRIAGE_EAR_HEIGHT,
    ).translate((-CARRIAGE_EAR_LENGTH / 2.0, -CARRIAGE_EAR_OFFSET_Y, -0.010))

    return (
        left_foot.union(right_foot)
        .union(bridge)
        .union(left_ear)
        .union(right_ear)
    )


def _make_bracket_shape() -> cq.Workplane:
    eye_outer = (
        cq.Workplane("XZ")
        .circle(BRACKET_EYE_OUTER_RADIUS)
        .extrude(PIN_LENGTH / 2.0, both=True)
    )
    eye_hole = (
        cq.Workplane("XZ")
        .circle(PIN_RADIUS)
        .extrude((PIN_LENGTH + 0.004) / 2.0, both=True)
    )
    eye = eye_outer.cut(eye_hole)

    neck = cq.Workplane("XY").box(
        BRACKET_NECK_LENGTH,
        BRACKET_NECK_WIDTH,
        BRACKET_NECK_HEIGHT,
    ).translate((BRACKET_NECK_LENGTH / 2.0, 0.0, BRACKET_NECK_HEIGHT / 2.0))

    brace = cq.Workplane("XY").box(
        BRACKET_BRACE_LENGTH,
        BRACKET_BRACE_WIDTH,
        BRACKET_BRACE_HEIGHT,
    ).translate((0.027, 0.0, 0.008))

    rear_block = cq.Workplane("XY").box(
        BRACKET_REAR_BLOCK_LENGTH,
        BRACKET_REAR_BLOCK_WIDTH,
        BRACKET_REAR_BLOCK_HEIGHT,
    ).translate((0.040, 0.0, 0.008))

    guide_floor = cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_FLOOR_WIDTH,
        GUIDE_FLOOR_HEIGHT,
    ).translate((GUIDE_CENTER_X, 0.0, GUIDE_FLOOR_Z))

    left_rail = cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_RAIL_WIDTH,
        GUIDE_RAIL_HEIGHT,
    ).translate((GUIDE_CENTER_X, GUIDE_RAIL_OFFSET_Y, GUIDE_RAIL_CENTER_Z))
    right_rail = cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_RAIL_WIDTH,
        GUIDE_RAIL_HEIGHT,
    ).translate((GUIDE_CENTER_X, -GUIDE_RAIL_OFFSET_Y, GUIDE_RAIL_CENTER_Z))

    return (
        eye.union(neck)
        .union(brace)
        .union(rear_block)
        .union(guide_floor)
        .union(left_rail)
        .union(right_rail)
    )


def _make_slider_shape() -> cq.Workplane:
    rod = cq.Workplane("XY").box(
        SLIDER_ROD_LENGTH,
        SLIDER_ROD_WIDTH,
        SLIDER_ROD_HEIGHT,
    ).translate((SLIDER_ROD_LENGTH / 2.0, 0.0, 0.0))

    pad = cq.Workplane("XY").box(
        SLIDER_PAD_LENGTH,
        SLIDER_PAD_WIDTH,
        SLIDER_PAD_HEIGHT,
    ).translate((SLIDER_ROD_LENGTH + SLIDER_PAD_LENGTH / 2.0, 0.0, 0.001))

    return rod.union(pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_hinge_slide_chain")

    base_mat = model.material("base_mat", rgba=(0.20, 0.22, 0.24, 1.0))
    carriage_mat = model.material("carriage_mat", rgba=(0.45, 0.49, 0.54, 1.0))
    bracket_mat = model.material("bracket_mat", rgba=(0.84, 0.49, 0.18, 1.0))
    slider_mat = model.material("slider_mat", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base"),
        material=base_mat,
        name="base_structure",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(
            _make_carriage_shape(),
            "carriage",
            tolerance=0.0002,
            angular_tolerance=0.05,
        ),
        material=carriage_mat,
        name="carriage_body",
    )

    pivot_bracket = model.part("pivot_bracket")
    pivot_bracket.visual(
        mesh_from_cadquery(
            _make_bracket_shape(),
            "pivot_bracket",
            tolerance=0.0002,
            angular_tolerance=0.05,
        ),
        material=bracket_mat,
        name="pivot_bracket_body",
    )

    terminal_slider = model.part("terminal_slider")
    terminal_slider.visual(
        mesh_from_cadquery(
            _make_slider_shape(),
            "terminal_slider",
            tolerance=0.0002,
            angular_tolerance=0.05,
        ),
        material=slider_mat,
        name="terminal_slider_body",
    )

    model.articulation(
        "base_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.12, 0.0, CARRIAGE_HINGE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=BASE_SLIDE_LIMIT,
        ),
    )

    model.articulation(
        "carriage_to_bracket",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pivot_bracket,
        origin=Origin(xyz=(BRACKET_EYE_OUTER_RADIUS, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
            lower=0.0,
            upper=HINGE_LIMIT,
        ),
    )

    model.articulation(
        "bracket_to_terminal",
        ArticulationType.PRISMATIC,
        parent=pivot_bracket,
        child=terminal_slider,
        origin=Origin(xyz=(SLIDER_ORIGIN_X, 0.0, SLIDER_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=TERMINAL_SLIDE_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    pivot_bracket = object_model.get_part("pivot_bracket")
    terminal_slider = object_model.get_part("terminal_slider")

    base_slide = object_model.get_articulation("base_slide")
    carriage_to_bracket = object_model.get_articulation("carriage_to_bracket")
    bracket_to_terminal = object_model.get_articulation("bracket_to_terminal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        pivot_bracket,
        terminal_slider,
        reason="Terminal slider is intentionally captured inside a close-tolerance guide channel on the pivot bracket.",
    )

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0005)
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
        "all mechanism parts exist",
        all(part is not None for part in (base, carriage, pivot_bracket, terminal_slider)),
        "Expected base, carriage, pivot_bracket, and terminal_slider parts.",
    )
    ctx.check(
        "articulation types match slide-hinge-slide chain",
        base_slide.articulation_type == ArticulationType.PRISMATIC
        and carriage_to_bracket.articulation_type == ArticulationType.REVOLUTE
        and bracket_to_terminal.articulation_type == ArticulationType.PRISMATIC,
        "Expected PRISMATIC -> REVOLUTE -> PRISMATIC articulation sequence.",
    )
    ctx.check(
        "terminal prismatic axis follows bracket output direction",
        tuple(bracket_to_terminal.axis) == (1.0, 0.0, 0.0),
        f"Expected terminal slider axis (1, 0, 0), got {bracket_to_terminal.axis!r}.",
    )

    ctx.expect_contact(
        carriage,
        base,
        name="carriage remains physically supported by the grounded base rail",
    )
    ctx.expect_contact(
        pivot_bracket,
        carriage,
        contact_tol=0.0005,
        name="pivot bracket is carried by the carriage hinge pin",
    )
    ctx.expect_contact(
        terminal_slider,
        pivot_bracket,
        name="terminal slider remains engaged in the bracket guide",
    )

    with ctx.pose({base_slide: BASE_SLIDE_LIMIT}):
        ctx.expect_contact(
            carriage,
            base,
            name="carriage stays on the base rail at full slide extension",
        )

    with ctx.pose({carriage_to_bracket: HINGE_LIMIT * 0.8}):
        ctx.expect_contact(
            pivot_bracket,
            carriage,
            contact_tol=0.0005,
            name="hinge contact is preserved while the bracket pitches upward",
        )

    with ctx.pose(
        {
            base_slide: 0.12,
            carriage_to_bracket: HINGE_LIMIT * 0.75,
            bracket_to_terminal: TERMINAL_SLIDE_LIMIT * 0.8,
        }
    ):
        ctx.expect_contact(
            terminal_slider,
            pivot_bracket,
            name="terminal slide stays captured when extended along the pitched bracket",
        )

    with ctx.pose({base_slide: 0.0}):
        carriage_home = ctx.part_world_position(carriage)
    with ctx.pose({base_slide: BASE_SLIDE_LIMIT}):
        carriage_far = ctx.part_world_position(carriage)
    carriage_advances = (
        carriage_home is not None
        and carriage_far is not None
        and carriage_far[0] > carriage_home[0] + 0.22
    )
    ctx.check(
        "root prismatic joint advances the carriage along +X",
        carriage_advances,
        f"Expected carriage X advance above 0.22 m, got home={carriage_home}, far={carriage_far}.",
    )

    with ctx.pose({carriage_to_bracket: 0.0}):
        slider_low = ctx.part_world_position(terminal_slider)
    with ctx.pose({carriage_to_bracket: HINGE_LIMIT * 0.8}):
        slider_high = ctx.part_world_position(terminal_slider)
    hinge_lifts = (
        slider_low is not None
        and slider_high is not None
        and slider_high[2] > slider_low[2] + 0.045
    )
    ctx.check(
        "revolute joint pitches the output stage upward",
        hinge_lifts,
        f"Expected terminal slider Z increase above 0.045 m, got low={slider_low}, high={slider_high}.",
    )

    with ctx.pose(
        {
            base_slide: 0.10,
            carriage_to_bracket: HINGE_LIMIT * 0.75,
            bracket_to_terminal: 0.0,
        }
    ):
        slider_retracted = ctx.part_world_position(terminal_slider)
    with ctx.pose(
        {
            base_slide: 0.10,
            carriage_to_bracket: HINGE_LIMIT * 0.75,
            bracket_to_terminal: TERMINAL_SLIDE_LIMIT,
        }
    ):
        slider_extended = ctx.part_world_position(terminal_slider)
    terminal_follows_bracket = (
        slider_retracted is not None
        and slider_extended is not None
        and slider_extended[0] > slider_retracted[0] + 0.02
        and slider_extended[2] > slider_retracted[2] + 0.04
    )
    ctx.check(
        "terminal prismatic joint extends along the bracket's pitched output axis",
        terminal_follows_bracket,
        "Expected the terminal slider to move outward and upward when extended with the bracket pitched.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
