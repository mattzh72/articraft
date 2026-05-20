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


BASE_LENGTH = 0.86
BASE_WIDTH = 0.34
BASE_THICKNESS = 0.020
BASE_RAIL_LENGTH = 0.78
BASE_RAIL_WIDTH = 0.050
BASE_RAIL_HEIGHT = 0.012
BASE_RAIL_Y = 0.110

FIRST_STAGE_LENGTH = 0.46
FIRST_STAGE_WIDTH = 0.28
FIRST_STAGE_BODY_THICKNESS = 0.022
FIRST_STAGE_RUNNER_LENGTH = 0.44
FIRST_STAGE_RUNNER_WIDTH = 0.036
FIRST_STAGE_RUNNER_DEPTH = 0.010
FIRST_STAGE_RUNNER_Y = BASE_RAIL_Y
FIRST_STAGE_TOP_RAIL_LENGTH = 0.40
FIRST_STAGE_TOP_RAIL_WIDTH = 0.028
FIRST_STAGE_TOP_RAIL_HEIGHT = 0.010
FIRST_STAGE_TOP_RAIL_Y = 0.070
FIRST_STAGE_HOME_X = -0.10
FIRST_STAGE_STROKE = 0.30

TERMINAL_STAGE_LENGTH = 0.20
TERMINAL_STAGE_WIDTH = 0.18
TERMINAL_STAGE_BODY_THICKNESS = 0.020
TERMINAL_STAGE_RUNNER_LENGTH = 0.18
TERMINAL_STAGE_RUNNER_WIDTH = 0.026
TERMINAL_STAGE_RUNNER_DEPTH = 0.010
TERMINAL_STAGE_RUNNER_Y = FIRST_STAGE_TOP_RAIL_Y
TERMINAL_STAGE_HOME_X = 0.04
TERMINAL_STAGE_STROKE = 0.15
END_PLATE_THICKNESS = 0.010
END_PLATE_WIDTH = 0.150
END_PLATE_HEIGHT = 0.090


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.010)
    )
    rear_pad = cq.Workplane("XY").box(0.13, 0.26, 0.010).translate(
        (-0.29, 0.0, -0.005)
    )
    front_pad = cq.Workplane("XY").box(0.13, 0.26, 0.010).translate(
        (0.29, 0.0, -0.005)
    )
    left_rail = cq.Workplane("XY").box(
        BASE_RAIL_LENGTH,
        BASE_RAIL_WIDTH,
        BASE_RAIL_HEIGHT,
    ).translate((0.0, BASE_RAIL_Y, BASE_THICKNESS / 2 + BASE_RAIL_HEIGHT / 2))
    right_rail = cq.Workplane("XY").box(
        BASE_RAIL_LENGTH,
        BASE_RAIL_WIDTH,
        BASE_RAIL_HEIGHT,
    ).translate((0.0, -BASE_RAIL_Y, BASE_THICKNESS / 2 + BASE_RAIL_HEIGHT / 2))

    return (
        plate.union(rear_pad)
        .union(front_pad)
        .union(left_rail)
        .union(right_rail)
        .faces(">Z")
        .edges("|Y")
        .fillet(0.004)
    )


def _first_stage_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(FIRST_STAGE_LENGTH, FIRST_STAGE_WIDTH, FIRST_STAGE_BODY_THICKNESS)
        .edges("|Z")
        .fillet(0.008)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(-0.03, 0.0)
        .rect(0.27, 0.10)
        .cutBlind(0.008)
    )

    left_runner = cq.Workplane("XY").box(
        FIRST_STAGE_RUNNER_LENGTH,
        FIRST_STAGE_RUNNER_WIDTH,
        FIRST_STAGE_RUNNER_DEPTH,
    ).translate(
        (
            0.0,
            FIRST_STAGE_RUNNER_Y,
            -(FIRST_STAGE_BODY_THICKNESS + FIRST_STAGE_RUNNER_DEPTH) / 2,
        )
    )
    right_runner = cq.Workplane("XY").box(
        FIRST_STAGE_RUNNER_LENGTH,
        FIRST_STAGE_RUNNER_WIDTH,
        FIRST_STAGE_RUNNER_DEPTH,
    ).translate(
        (
            0.0,
            -FIRST_STAGE_RUNNER_Y,
            -(FIRST_STAGE_BODY_THICKNESS + FIRST_STAGE_RUNNER_DEPTH) / 2,
        )
    )
    left_top_rail = cq.Workplane("XY").box(
        FIRST_STAGE_TOP_RAIL_LENGTH,
        FIRST_STAGE_TOP_RAIL_WIDTH,
        FIRST_STAGE_TOP_RAIL_HEIGHT,
    ).translate(
        (
            0.03,
            FIRST_STAGE_TOP_RAIL_Y,
            FIRST_STAGE_BODY_THICKNESS / 2 + FIRST_STAGE_TOP_RAIL_HEIGHT / 2,
        )
    )
    right_top_rail = cq.Workplane("XY").box(
        FIRST_STAGE_TOP_RAIL_LENGTH,
        FIRST_STAGE_TOP_RAIL_WIDTH,
        FIRST_STAGE_TOP_RAIL_HEIGHT,
    ).translate(
        (
            0.03,
            -FIRST_STAGE_TOP_RAIL_Y,
            FIRST_STAGE_BODY_THICKNESS / 2 + FIRST_STAGE_TOP_RAIL_HEIGHT / 2,
        )
    )

    return (
        body.union(left_runner)
        .union(right_runner)
        .union(left_top_rail)
        .union(right_top_rail)
    )


def _terminal_stage_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            TERMINAL_STAGE_LENGTH,
            TERMINAL_STAGE_WIDTH,
            TERMINAL_STAGE_BODY_THICKNESS,
        )
        .edges("|Z")
        .fillet(0.006)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(-0.015, 0.0)
        .rect(0.10, 0.06)
        .cutBlind(0.005)
    )
    left_runner = cq.Workplane("XY").box(
        TERMINAL_STAGE_RUNNER_LENGTH,
        TERMINAL_STAGE_RUNNER_WIDTH,
        TERMINAL_STAGE_RUNNER_DEPTH,
    ).translate(
        (
            0.0,
            TERMINAL_STAGE_RUNNER_Y,
            -(
                TERMINAL_STAGE_BODY_THICKNESS + TERMINAL_STAGE_RUNNER_DEPTH
            )
            / 2,
        )
    )
    right_runner = cq.Workplane("XY").box(
        TERMINAL_STAGE_RUNNER_LENGTH,
        TERMINAL_STAGE_RUNNER_WIDTH,
        TERMINAL_STAGE_RUNNER_DEPTH,
    ).translate(
        (
            0.0,
            -TERMINAL_STAGE_RUNNER_Y,
            -(
                TERMINAL_STAGE_BODY_THICKNESS + TERMINAL_STAGE_RUNNER_DEPTH
            )
            / 2,
        )
    )
    return body.union(left_runner).union(right_runner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_transfer_axis")

    model.material("anodized_gray", color=(0.63, 0.65, 0.68, 1.0))
    model.material("plate_gray", color=(0.77, 0.79, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base"),
        material="anodized_gray",
        name="base_body",
    )

    first_slide = model.part("first_slide")
    first_slide.visual(
        mesh_from_cadquery(_first_stage_shape(), "first_slide"),
        material="anodized_gray",
        name="first_slide_body",
    )

    terminal_slide = model.part("terminal_slide")
    terminal_slide.visual(
        mesh_from_cadquery(_terminal_stage_body_shape(), "terminal_slide"),
        material="anodized_gray",
        name="terminal_slide_body",
    )
    terminal_slide.visual(
        Box((END_PLATE_THICKNESS, END_PLATE_WIDTH, END_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(
                TERMINAL_STAGE_LENGTH / 2 + END_PLATE_THICKNESS / 2 - 0.002,
                0.0,
                0.053,
            )
        ),
        material="plate_gray",
        name="end_plate",
    )

    base_to_first_z = (
        BASE_THICKNESS / 2
        + BASE_RAIL_HEIGHT
        + FIRST_STAGE_BODY_THICKNESS / 2
        + FIRST_STAGE_RUNNER_DEPTH
    )
    first_to_terminal_z = (
        FIRST_STAGE_BODY_THICKNESS / 2
        + FIRST_STAGE_TOP_RAIL_HEIGHT
        + TERMINAL_STAGE_BODY_THICKNESS / 2
        + TERMINAL_STAGE_RUNNER_DEPTH
    )

    model.articulation(
        "base_to_first_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first_slide,
        origin=Origin(xyz=(FIRST_STAGE_HOME_X, 0.0, base_to_first_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.65,
            lower=0.0,
            upper=FIRST_STAGE_STROKE,
        ),
    )
    model.articulation(
        "first_slide_to_terminal_slide",
        ArticulationType.PRISMATIC,
        parent=first_slide,
        child=terminal_slide,
        origin=Origin(xyz=(TERMINAL_STAGE_HOME_X, 0.0, first_to_terminal_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.60,
            lower=0.0,
            upper=TERMINAL_STAGE_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    first_slide = object_model.get_part("first_slide")
    terminal_slide = object_model.get_part("terminal_slide")
    base_to_first = object_model.get_articulation("base_to_first_slide")
    first_to_terminal = object_model.get_articulation("first_slide_to_terminal_slide")
    end_plate = terminal_slide.get_visual("end_plate")

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
        "serial_prismatic_joint_chain",
        (
            base_to_first.articulation_type == ArticulationType.PRISMATIC
            and first_to_terminal.articulation_type == ArticulationType.PRISMATIC
            and base_to_first.parent == "base"
            and base_to_first.child == "first_slide"
            and first_to_terminal.parent == "first_slide"
            and first_to_terminal.child == "terminal_slide"
        ),
        "expected base -> first_slide -> terminal_slide serial prismatic chain",
    )
    ctx.check(
        "prismatic_axes_point_forward",
        base_to_first.axis == (1.0, 0.0, 0.0)
        and first_to_terminal.axis == (1.0, 0.0, 0.0),
        "both transfer stages should translate along +X",
    )
    ctx.check(
        "stroke_lengths_are_staged",
        (
            base_to_first.motion_limits is not None
            and first_to_terminal.motion_limits is not None
            and base_to_first.motion_limits.upper == FIRST_STAGE_STROKE
            and first_to_terminal.motion_limits.upper == TERMINAL_STAGE_STROKE
            and base_to_first.motion_limits.upper > first_to_terminal.motion_limits.upper
        ),
        "expected a longer first stroke and a shorter terminal stroke",
    )
    ctx.check(
        "plain_end_plate_present",
        end_plate.geometry == Box((END_PLATE_THICKNESS, END_PLATE_WIDTH, END_PLATE_HEIGHT)),
        "terminal slide should carry a plain rectangular end plate",
    )

    with ctx.pose({base_to_first: 0.0, first_to_terminal: 0.0}):
        ctx.expect_contact(
            first_slide,
            base,
            name="first_slide_supported_by_base",
        )
        ctx.expect_contact(
            terminal_slide,
            first_slide,
            name="terminal_slide_supported_by_first_slide",
        )
        ctx.expect_overlap(
            first_slide,
            base,
            axes="y",
            min_overlap=0.20,
            name="first_slide_tracks_within_base_width",
        )
        ctx.expect_overlap(
            terminal_slide,
            first_slide,
            axes="y",
            min_overlap=0.12,
            name="terminal_slide_tracks_within_first_slide_width",
        )

    with ctx.pose({base_to_first: FIRST_STAGE_STROKE, first_to_terminal: 0.0}):
        ctx.expect_contact(
            first_slide,
            base,
            name="first_slide_remains_supported_at_full_extension",
        )
        ctx.expect_origin_gap(
            first_slide,
            base,
            axis="x",
            min_gap=0.18,
            name="first_slide_extends_forward_on_base",
        )

    with ctx.pose({base_to_first: 0.0, first_to_terminal: TERMINAL_STAGE_STROKE}):
        ctx.expect_contact(
            terminal_slide,
            first_slide,
            name="terminal_slide_remains_supported_at_full_extension",
        )
        ctx.expect_origin_gap(
            terminal_slide,
            first_slide,
            axis="x",
            min_gap=0.18,
            name="terminal_stage_extends_forward_from_first_slide",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
