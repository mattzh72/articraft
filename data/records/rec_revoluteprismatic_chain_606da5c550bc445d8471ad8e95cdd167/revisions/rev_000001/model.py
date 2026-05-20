from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_LENGTH = 0.18
BASE_WIDTH = 0.11
BASE_THICKNESS = 0.014
PIVOT_Z = 0.095

CHEEK_THICKNESS = 0.012
CHEEK_CENTER_Y = 0.026
CHEEK_GAP = (2.0 * CHEEK_CENTER_Y) - CHEEK_THICKNESS
HUB_OUTER_RADIUS = 0.024
HUB_LENGTH = CHEEK_GAP - 0.006
PIVOT_PIN_RADIUS = 0.009

GUIDE_START_X = 0.205
GUIDE_LENGTH = 0.175
GUIDE_RAIL_WIDTH = 0.022
GUIDE_RAIL_HEIGHT = 0.016

SLIDER_BODY_LENGTH = 0.084
SLIDER_BODY_WIDTH = 0.036
SLIDER_BODY_HEIGHT = 0.024
SLIDER_HEAD_LENGTH = 0.022
STAGE_ORIGIN_X = 0.225
SLIDE_TRAVEL = 0.080
ARM_SWING_UPPER = 1.15


def _build_base_bracket_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.060, -0.035),
                (-0.060, 0.035),
                (0.060, -0.035),
                (0.060, 0.035),
            ]
        )
        .hole(0.010)
    )

    cheek = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.034, BASE_THICKNESS),
                (0.014, BASE_THICKNESS),
                (0.028, 0.052),
                (0.028, PIVOT_Z + 0.020),
                (0.004, PIVOT_Z + 0.042),
                (-0.022, PIVOT_Z + 0.042),
                (-0.034, PIVOT_Z + 0.018),
            ]
        )
        .close()
        .extrude(CHEEK_THICKNESS / 2.0, both=True)
    )

    left_cheek = cheek.translate((0.0, CHEEK_CENTER_Y, 0.0))
    right_cheek = cheek.translate((0.0, -CHEEK_CENTER_Y, 0.0))

    rear_web = (
        cq.Workplane("XY")
        .box(0.046, CHEEK_GAP, 0.038)
        .translate((-0.018, 0.0, BASE_THICKNESS + 0.019))
    )
    pivot_pin = (
        cq.Workplane("XZ")
        .center(0.0, PIVOT_Z)
        .circle(PIVOT_PIN_RADIUS)
        .extrude((CHEEK_GAP + 0.004) / 2.0, both=True)
    )

    return plate.union(left_cheek).union(right_cheek).union(rear_web).union(pivot_pin)


def _build_swing_arm_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XZ")
        .circle(HUB_OUTER_RADIUS)
        .extrude(HUB_LENGTH / 2.0, both=True)
        .cut(
            cq.Workplane("XZ")
            .circle(PIVOT_PIN_RADIUS)
            .extrude((HUB_LENGTH + 0.004) / 2.0, both=True)
        )
    )

    beam = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.018, -0.014),
                (0.110, -0.014),
                (0.182, -0.011),
                (0.215, -0.008),
                (0.215, 0.008),
                (0.182, 0.011),
                (0.110, 0.014),
                (0.018, 0.014),
            ]
        )
        .close()
        .extrude(0.007, both=True)
    )

    underside_rib = (
        cq.Workplane("XY")
        .box(0.165, 0.010, 0.012)
        .translate((0.120, 0.0, -0.014))
    )
    transition_block = (
        cq.Workplane("XY")
        .box(0.028, 0.016, 0.020)
        .translate((0.210, 0.0, 0.0))
    )

    return hub.union(beam).union(underside_rib).union(transition_block)


def _build_guide_rail_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(GUIDE_LENGTH, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT)
        .translate((GUIDE_START_X + (GUIDE_LENGTH / 2.0), 0.0, 0.0))
    )


def _build_slider_shape() -> cq.Workplane:
    top_block = (
        cq.Workplane("XY")
        .box(SLIDER_BODY_LENGTH, SLIDER_BODY_WIDTH, SLIDER_BODY_HEIGHT)
        .translate((SLIDER_BODY_LENGTH / 2.0, 0.0, 0.020))
    )
    left_pad = (
        cq.Workplane("XY")
        .box(SLIDER_BODY_LENGTH * 0.75, 0.008, 0.016)
        .translate((SLIDER_BODY_LENGTH * 0.40, 0.015, 0.016))
    )
    right_pad = (
        cq.Workplane("XY")
        .box(SLIDER_BODY_LENGTH * 0.75, 0.008, 0.016)
        .translate((SLIDER_BODY_LENGTH * 0.40, -0.015, 0.016))
    )
    head = (
        cq.Workplane("XY")
        .box(SLIDER_HEAD_LENGTH, 0.032, 0.024)
        .translate((SLIDER_BODY_LENGTH + (SLIDER_HEAD_LENGTH / 2.0), 0.0, 0.020))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.022, 0.024, 0.016)
        .translate((SLIDER_BODY_LENGTH + SLIDER_HEAD_LENGTH + 0.011, 0.0, 0.018))
    )
    return top_block.union(left_pad).union(right_pad).union(head).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_prismatic_bracket_arm")

    model.material("powder_coat", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("arm_alloy", rgba=(0.71, 0.74, 0.77, 1.0))
    model.material("slider_blue", rgba=(0.21, 0.35, 0.58, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        mesh_from_cadquery(_build_base_bracket_shape(), "base_bracket"),
        origin=Origin(),
        material="powder_coat",
        name="bracket_body",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.12)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    swing_arm = model.part("swing_arm")
    swing_arm.visual(
        mesh_from_cadquery(_build_swing_arm_shape(), "swing_arm"),
        origin=Origin(),
        material="arm_alloy",
        name="arm_body",
    )
    swing_arm.visual(
        mesh_from_cadquery(_build_guide_rail_shape(), "guide_rail"),
        origin=Origin(),
        material="arm_alloy",
        name="guide_rail",
    )
    swing_arm.inertial = Inertial.from_geometry(
        Box((0.40, 0.05, 0.08)),
        mass=1.8,
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
    )

    slider_stage = model.part("slider_stage")
    slider_stage.visual(
        mesh_from_cadquery(_build_slider_shape(), "slider_stage"),
        origin=Origin(),
        material="slider_blue",
        name="stage_body",
    )
    slider_stage.inertial = Inertial.from_geometry(
        Box((SLIDER_BODY_LENGTH + SLIDER_HEAD_LENGTH + 0.022, 0.036, 0.032)),
        mass=0.65,
        origin=Origin(xyz=(0.060, 0.0, 0.020)),
    )

    model.articulation(
        "arm_swing",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=swing_arm,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=ARM_SWING_UPPER,
        ),
    )
    model.articulation(
        "stage_slide",
        ArticulationType.PRISMATIC,
        parent=swing_arm,
        child=slider_stage,
        origin=Origin(xyz=(STAGE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    swing_arm = object_model.get_part("swing_arm")
    slider_stage = object_model.get_part("slider_stage")
    guide_rail = swing_arm.get_visual("guide_rail")
    stage_body = slider_stage.get_visual("stage_body")
    arm_swing = object_model.get_articulation("arm_swing")
    stage_slide = object_model.get_articulation("stage_slide")

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
    ctx.allow_overlap(
        base_bracket,
        swing_arm,
        reason="The base bracket integrates the captured hinge pin, so the arm hub intentionally nests at the pivot.",
    )
    ctx.allow_overlap(
        slider_stage,
        swing_arm,
        elem_a=stage_body,
        elem_b=guide_rail,
        reason="The slider carriage wraps around the guide rail as an intentional nested fit.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(swing_arm, base_bracket, name="pivot_is_physically_supported")

    with ctx.pose({arm_swing: 0.0, stage_slide: 0.0}):
        ctx.expect_contact(
            slider_stage,
            swing_arm,
            elem_a=stage_body,
            elem_b=guide_rail,
            name="slider_contacts_guide_when_retracted",
        )
        ctx.expect_overlap(
            slider_stage,
            swing_arm,
            axes="y",
            elem_a=stage_body,
            elem_b=guide_rail,
            min_overlap=0.014,
            name="slider_and_guide_share_lateral_overlap",
        )

    with ctx.pose({arm_swing: 0.65, stage_slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            slider_stage,
            swing_arm,
            elem_a=stage_body,
            elem_b=guide_rail,
            name="slider_remains_guided_when_extended",
        )
        ctx.expect_contact(swing_arm, base_bracket, name="pivot_remains_supported_when_arm_is_raised")

    with ctx.pose({arm_swing: 0.0, stage_slide: 0.0}):
        retracted_position = ctx.part_world_position(slider_stage)
    with ctx.pose({arm_swing: 0.0, stage_slide: SLIDE_TRAVEL}):
        extended_position = ctx.part_world_position(slider_stage)

    slider_extends_forward = (
        retracted_position is not None
        and extended_position is not None
        and extended_position[0] > retracted_position[0] + (SLIDE_TRAVEL * 0.95)
        and isclose(extended_position[2], retracted_position[2], abs_tol=1e-6)
    )
    ctx.check(
        "slider_translates_along_arm_axis",
        slider_extends_forward,
        details=(
            f"retracted={retracted_position}, extended={extended_position}, "
            f"expected +x travel about {SLIDE_TRAVEL:.3f} m with no z drift"
        ),
    )

    with ctx.pose({arm_swing: 0.0, stage_slide: 0.0}):
        low_position = ctx.part_world_position(slider_stage)
    with ctx.pose({arm_swing: ARM_SWING_UPPER, stage_slide: 0.0}):
        raised_position = ctx.part_world_position(slider_stage)

    arm_raises_slider = (
        low_position is not None
        and raised_position is not None
        and raised_position[2] > low_position[2] + 0.16
        and raised_position[0] < low_position[0]
    )
    ctx.check(
        "revolute_stage_swings_upward",
        arm_raises_slider,
        details=(
            f"low={low_position}, raised={raised_position}, "
            "expected higher z and shorter x reach at upper arm swing"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
