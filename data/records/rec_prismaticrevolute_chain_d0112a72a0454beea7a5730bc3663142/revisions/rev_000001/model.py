from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import fabs

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


RAIL_LENGTH = 0.38
RAIL_WIDTH = 0.11
BASE_THICKNESS = 0.012
GUIDE_HEIGHT = 0.014
GUIDE_WIDTH = 0.018
GUIDE_LENGTH = 0.32
GUIDE_OFFSET_Y = 0.028
END_STOP_LENGTH = 0.016

CARRIAGE_LENGTH = 0.088
CARRIAGE_WIDTH = 0.096
CARRIAGE_BODY_HEIGHT = 0.022
CARRIAGE_POCKET_WIDTH = 0.050
CARRIAGE_POCKET_DEPTH = 0.012
PEDESTAL_LENGTH = 0.046
PEDESTAL_WIDTH = 0.050
PEDESTAL_HEIGHT = 0.014

BRACKET_CHEEK_LENGTH = 0.032
BRACKET_CHEEK_THICKNESS = 0.010
BRACKET_GAP = 0.016
BRACKET_CHEEK_HEIGHT = 0.044
PIN_RADIUS = 0.0045
PIVOT_Z_LOCAL = CARRIAGE_BODY_HEIGHT + 0.024

ARM_LENGTH = 0.125
ARM_DEPTH = 0.012
ARM_PLATE_THICKNESS = BRACKET_GAP
ARM_HUB_RADIUS = 0.011
ARM_ROOT_LENGTH = 0.022
ARM_WEB_WIDTH = 0.010
ARM_BAR_Z = 0.032
PAD_SIZE = 0.042
PAD_THICKNESS = 0.010
PAD_CENTER_X = ARM_LENGTH - 0.008
PAD_CENTER_Z = ARM_BAR_Z

RAIL_TOP_Z = BASE_THICKNESS + GUIDE_HEIGHT
SLIDE_TRAVEL = 0.11


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def _build_rail_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )

    guide = (
        cq.Workplane("XY")
        .box(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0015)
    )

    inner_bridge_width = 2.0 * GUIDE_OFFSET_Y - GUIDE_WIDTH
    stop = cq.Workplane("XY").box(
        END_STOP_LENGTH,
        inner_bridge_width,
        GUIDE_HEIGHT,
        centered=(True, True, False),
    )

    rail = (
        base.union(guide.translate((0.0, GUIDE_OFFSET_Y, BASE_THICKNESS)))
        .union(guide.translate((0.0, -GUIDE_OFFSET_Y, BASE_THICKNESS)))
        .union(
            stop.translate(
                (
                    0.5 * (GUIDE_LENGTH - END_STOP_LENGTH),
                    0.0,
                    BASE_THICKNESS,
                )
            )
        )
        .union(
            stop.translate(
                (
                    -0.5 * (GUIDE_LENGTH - END_STOP_LENGTH),
                    0.0,
                    BASE_THICKNESS,
                )
            )
        )
    )

    return rail


def _build_carriage_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        CARRIAGE_BODY_HEIGHT,
        centered=(True, True, False),
    )

    pocket = cq.Workplane("XY").box(
        CARRIAGE_LENGTH - 0.010,
        CARRIAGE_POCKET_WIDTH,
        CARRIAGE_POCKET_DEPTH,
        centered=(True, True, False),
    )
    carriage = carriage.cut(pocket)

    carriage = carriage.edges("|Z").fillet(0.0015)
    return carriage


def _build_bracket_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(
        PEDESTAL_LENGTH,
        BRACKET_CHEEK_THICKNESS,
        PEDESTAL_HEIGHT,
        centered=(True, True, False),
    )
    cheek = cq.Workplane("XY").box(
        BRACKET_CHEEK_LENGTH,
        BRACKET_CHEEK_THICKNESS,
        BRACKET_CHEEK_HEIGHT,
        centered=(True, True, False),
    )
    bracket = foot.union(cheek.translate((0.0, 0.0, PEDESTAL_HEIGHT)))
    bracket = bracket.edges("|Z").fillet(0.0012)
    return bracket


def _build_arm_shape() -> cq.Workplane:
    root_tongue = cq.Workplane("XY").box(
        ARM_ROOT_LENGTH,
        ARM_PLATE_THICKNESS,
        0.010,
        centered=(False, True, True),
    )
    root_tongue = root_tongue.edges("|Y").fillet(0.0015)

    riser = cq.Workplane("XY").box(
        ARM_WEB_WIDTH,
        ARM_PLATE_THICKNESS,
        ARM_BAR_Z + 0.006,
        centered=(True, True, False),
    )
    riser = riser.translate((ARM_ROOT_LENGTH - 0.004, 0.0, 0.0))

    arm_bar = cq.Workplane("XY").box(
        ARM_LENGTH - ARM_ROOT_LENGTH,
        ARM_PLATE_THICKNESS,
        ARM_DEPTH,
        centered=(False, True, True),
    )
    arm_bar = arm_bar.translate((ARM_ROOT_LENGTH, 0.0, ARM_BAR_Z))
    arm_bar = arm_bar.edges("|Y").fillet(0.002)

    hub = (
        cq.Workplane("XZ")
        .circle(ARM_HUB_RADIUS)
        .extrude(0.5 * BRACKET_GAP, both=True)
    )

    pivot_bore = (
        cq.Workplane("XZ")
        .circle(PIN_RADIUS + 0.0006)
        .extrude(0.5 * (BRACKET_GAP + 0.004), both=True)
    )

    arm = root_tongue.union(riser).union(arm_bar).union(hub).cut(pivot_bore)
    return arm


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_slide_hinge_arm")

    rail_mat = model.material("rail_steel", rgba=(0.28, 0.30, 0.34, 1.0))
    carriage_mat = model.material("carriage_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    arm_mat = model.material("arm_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    pad_mat = model.material("pad_rubber", rgba=(0.14, 0.14, 0.14, 1.0))

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(_build_rail_shape(), "rail_body"),
        material=rail_mat,
        name="rail_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "carriage_body"),
        material=carriage_mat,
        name="carriage_body",
    )

    left_bracket = model.part("left_bracket")
    left_bracket.visual(
        mesh_from_cadquery(_build_bracket_shape(), "left_bracket_body"),
        material=carriage_mat,
        name="left_bracket_body",
    )

    right_bracket = model.part("right_bracket")
    right_bracket.visual(
        mesh_from_cadquery(_build_bracket_shape(), "right_bracket_body"),
        material=carriage_mat,
        name="right_bracket_body",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_build_arm_shape(), "arm_body"),
        material=arm_mat,
        name="arm_body",
    )
    arm.visual(
        Box((PAD_SIZE, PAD_SIZE, PAD_THICKNESS)),
        origin=Origin(xyz=(PAD_CENTER_X, 0.0, PAD_CENTER_Z)),
        material=pad_mat,
        name="pad",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
        ),
    )

    cheek_center_y = 0.5 * (BRACKET_GAP + BRACKET_CHEEK_THICKNESS)
    model.articulation(
        "carriage_to_left_bracket",
        ArticulationType.FIXED,
        parent=carriage,
        child=left_bracket,
        origin=Origin(xyz=(0.0, cheek_center_y, CARRIAGE_BODY_HEIGHT)),
    )

    model.articulation(
        "carriage_to_right_bracket",
        ArticulationType.FIXED,
        parent=carriage,
        child=right_bracket,
        origin=Origin(xyz=(0.0, -cheek_center_y, CARRIAGE_BODY_HEIGHT)),
    )

    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z_LOCAL)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.35,
            upper=1.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    left_bracket = object_model.get_part("left_bracket")
    right_bracket = object_model.get_part("right_bracket")
    arm = object_model.get_part("arm")
    slide = object_model.get_articulation("rail_to_carriage")
    pivot = object_model.get_articulation("carriage_to_arm")
    pad = arm.get_visual("pad")

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
        "parts_present",
        all(part is not None for part in (rail, carriage, left_bracket, right_bracket, arm)),
        "Rail, carriage, bracket cheeks, and arm must all exist.",
    )
    ctx.check(
        "slide_axis_aligned_with_rail",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"Expected prismatic axis (1, 0, 0), got {slide.axis!r}.",
    )
    ctx.check(
        "pivot_axis_transverse",
        fabs(pivot.axis[0]) < 1e-9 and fabs(fabs(pivot.axis[1]) - 1.0) < 1e-9 and fabs(pivot.axis[2]) < 1e-9,
        f"Expected revolute axis aligned to transverse y, got {pivot.axis!r}.",
    )

    with ctx.pose({slide: 0.0, pivot: 0.0}):
        ctx.expect_contact(
            carriage,
            rail,
            name="carriage_supported_by_rail",
        )
        ctx.expect_within(
            carriage,
            rail,
            axes="xy",
            margin=0.0,
            name="carriage_envelope_stays_on_rail",
        )
        ctx.expect_contact(
            arm,
            left_bracket,
            name="arm_touches_left_hinge_cheek",
        )
        ctx.expect_contact(
            arm,
            right_bracket,
            name="arm_touches_right_hinge_cheek",
        )
        ctx.expect_gap(
            arm,
            carriage,
            axis="z",
            min_gap=0.012,
            name="arm_sits_above_carriage_body",
        )
        ctx.expect_gap(
            arm,
            rail,
            axis="z",
            min_gap=0.028,
            name="arm_clears_rail_in_rest_pose",
        )

    lower = slide.motion_limits.lower if slide.motion_limits else None
    upper = slide.motion_limits.upper if slide.motion_limits else None
    if lower is not None and upper is not None:
        with ctx.pose({slide: lower, pivot: 0.0}):
            low_pos = ctx.part_world_position(carriage)
        with ctx.pose({slide: upper, pivot: 0.0}):
            high_pos = ctx.part_world_position(carriage)

        if low_pos is not None and high_pos is not None:
            expected_dx = upper - lower
            dx = high_pos[0] - low_pos[0]
            dy = high_pos[1] - low_pos[1]
            dz = high_pos[2] - low_pos[2]
            ctx.check(
                "carriage_translates_only_along_x",
                fabs(dx - expected_dx) < 1e-6 and fabs(dy) < 1e-6 and fabs(dz) < 1e-6,
                (
                    "Expected carriage origin delta "
                    f"({expected_dx:.6f}, 0, 0), got ({dx:.6f}, {dy:.6f}, {dz:.6f})."
                ),
            )

        with ctx.pose({slide: upper, pivot: 0.0}):
            ctx.expect_within(
                carriage,
                rail,
                axes="x",
                margin=0.0,
                name="carriage_remains_captured_at_full_slide",
            )

    with ctx.pose({slide: 0.0, pivot: 0.0}):
        pad_rest_aabb = ctx.part_element_world_aabb(arm, elem=pad)
    with ctx.pose({slide: 0.0, pivot: 1.0}):
        pad_raised_aabb = ctx.part_element_world_aabb(arm, elem=pad)

    if pad_rest_aabb is not None and pad_raised_aabb is not None:
        pad_rest_center = _aabb_center(pad_rest_aabb)
        pad_raised_center = _aabb_center(pad_raised_aabb)
        ctx.check(
            "pad_rises_when_arm_rotates",
            pad_raised_center[2] > pad_rest_center[2] + 0.06
            and fabs(pad_raised_center[1] - pad_rest_center[1]) < 1e-6,
            (
                "Expected the square pad to move upward in the arm plane. "
                f"Rest center={pad_rest_center!r}, raised center={pad_raised_center!r}."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
