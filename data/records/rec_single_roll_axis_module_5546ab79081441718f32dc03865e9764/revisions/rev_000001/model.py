from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.18
FRAME_HEIGHT = 0.035
FRAME_RAIL = 0.02
FRAME_FOOT_RADIUS = 0.012
FRAME_FOOT_HEIGHT = 0.006
FRAME_FOOT_X = 0.165
FRAME_FOOT_Y = 0.065

ROLL_AXIS_Z = 0.115
SUPPORT_CENTER_X = 0.145
SUPPORT_BODY_LEN = 0.018
SUPPORT_POST_LEN = 0.036
SUPPORT_FOOT_LEN = 0.07
SUPPORT_FOOT_WIDTH = 0.026
SUPPORT_FOOT_Y = 0.037
SUPPORT_FOOT_THICKNESS = 0.012
SUPPORT_ROLLER_RADIUS = 0.006
SUPPORT_ROLLER_LENGTH = 0.018

SPINDLE_TUBE_RADIUS = 0.04
SPINDLE_WALL = 0.006
SPINDLE_TUBE_LENGTH = 0.19
SPINDLE_HUB_RADIUS = 0.028
SPINDLE_HUB_LENGTH = 0.012
SPINDLE_SHAFT_RADIUS = 0.012
SPINDLE_SHAFT_LENGTH = 0.078
COLLAR_RADIUS = 0.022
COLLAR_THICKNESS = 0.008


def cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def x_cylinder(radius: float, length: float, center: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length / 2.0, both=True).translate(center)


def make_base_frame() -> cq.Workplane:
    frame = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        FRAME_HEIGHT,
        centered=(True, True, False),
    )
    frame = frame.cut(
        cq.Workplane("XY").box(
            BASE_LENGTH - 2.0 * FRAME_RAIL,
            BASE_WIDTH - 2.0 * FRAME_RAIL,
            FRAME_HEIGHT + 0.004,
            centered=(True, True, False),
        )
    )
    for x_pos in (-SUPPORT_CENTER_X, SUPPORT_CENTER_X):
        frame = frame.union(
            cq.Workplane("XY").box(
                0.03,
                BASE_WIDTH - 2.0 * FRAME_RAIL,
                FRAME_HEIGHT,
                centered=(True, True, False),
            ).translate((x_pos, 0.0, 0.0))
        )

    for x_pos in (-FRAME_FOOT_X, FRAME_FOOT_X):
        for y_pos in (-FRAME_FOOT_Y, FRAME_FOOT_Y):
            frame = frame.union(
                cq.Workplane("XY")
                .circle(FRAME_FOOT_RADIUS)
                .extrude(FRAME_FOOT_HEIGHT)
                .translate((x_pos, y_pos, -FRAME_FOOT_HEIGHT))
            )

    return frame


def make_support() -> cq.Workplane:
    foot_bottom = FRAME_HEIGHT - ROLL_AXIS_Z

    support = cq_box(
        (SUPPORT_FOOT_LEN, SUPPORT_FOOT_WIDTH, SUPPORT_FOOT_THICKNESS),
        (0.0, -SUPPORT_FOOT_Y, foot_bottom + SUPPORT_FOOT_THICKNESS / 2.0),
    )
    support = support.union(
        cq_box(
            (SUPPORT_FOOT_LEN, SUPPORT_FOOT_WIDTH, SUPPORT_FOOT_THICKNESS),
            (0.0, SUPPORT_FOOT_Y, foot_bottom + SUPPORT_FOOT_THICKNESS / 2.0),
        )
    )
    support = support.union(
        cq_box(
            (SUPPORT_FOOT_LEN, 0.102, 0.008),
            (0.0, 0.0, foot_bottom + 0.004),
        )
    )
    support = support.union(
        cq_box(
            (SUPPORT_POST_LEN, 0.016, 0.066),
            (0.0, -0.028, -0.045),
        )
    )
    support = support.union(
        cq_box(
            (SUPPORT_POST_LEN, 0.016, 0.066),
            (0.0, 0.028, -0.045),
        )
    )
    support = support.union(cq_box((0.046, 0.012, 0.034), (0.0, -0.024, -0.061)))
    support = support.union(cq_box((0.046, 0.012, 0.034), (0.0, 0.024, -0.061)))
    support = support.union(cq_box((0.028, 0.018, 0.012), (0.0, 0.0, -0.024)))
    support = support.union(x_cylinder(SUPPORT_ROLLER_RADIUS, SUPPORT_ROLLER_LENGTH, (0.0, 0.0, -0.018)))

    for y_pos in (-0.018, 0.018):
        support = support.union(z_cylinder(0.0032, 0.016, (0.0, y_pos, -0.025)))
        support = support.union(z_cylinder(0.005, 0.004, (0.0, y_pos, -0.015)))
        support = support.cut(z_cylinder(0.0018, 0.003, (0.0, y_pos, -0.0145)))

    for y_pos in (-SUPPORT_FOOT_Y, SUPPORT_FOOT_Y):
        head_center = (0.0, y_pos, foot_bottom + SUPPORT_FOOT_THICKNESS + 0.002)
        shank_center = (0.0, y_pos, foot_bottom + SUPPORT_FOOT_THICKNESS / 2.0)
        support = support.union(z_cylinder(0.003, SUPPORT_FOOT_THICKNESS, shank_center))
        support = support.union(z_cylinder(0.005, 0.004, head_center))
        support = support.cut(z_cylinder(0.002, 0.0025, (0.0, y_pos, foot_bottom + SUPPORT_FOOT_THICKNESS + 0.0025)))
    return support


def make_spindle() -> cq.Workplane:
    tube = (
        cq.Workplane("YZ")
        .circle(SPINDLE_TUBE_RADIUS)
        .circle(SPINDLE_TUBE_RADIUS - SPINDLE_WALL)
        .extrude(SPINDLE_TUBE_LENGTH / 2.0, both=True)
    )

    spindle = tube
    shaft_center_x = SPINDLE_TUBE_LENGTH / 2.0 + SPINDLE_HUB_LENGTH + SPINDLE_SHAFT_LENGTH / 2.0
    collar_center_x = SUPPORT_CENTER_X + SUPPORT_POST_LEN / 2.0 + 0.004 + COLLAR_THICKNESS / 2.0
    end_cap_center_x = SPINDLE_TUBE_LENGTH / 2.0 + SPINDLE_HUB_LENGTH + SPINDLE_SHAFT_LENGTH - 0.004

    for sign in (-1.0, 1.0):
        hub_center = (sign * (SPINDLE_TUBE_LENGTH / 2.0 + SPINDLE_HUB_LENGTH / 2.0), 0.0, 0.0)
        shaft_center = (sign * shaft_center_x, 0.0, 0.0)
        collar_center = (sign * collar_center_x, 0.0, 0.0)
        boss_center = (sign * collar_center_x, 0.0, 0.018)
        end_cap_center = (sign * end_cap_center_x, 0.0, 0.0)

        spindle = spindle.union(x_cylinder(SPINDLE_HUB_RADIUS, SPINDLE_HUB_LENGTH, hub_center))
        spindle = spindle.union(x_cylinder(SPINDLE_SHAFT_RADIUS, SPINDLE_SHAFT_LENGTH, shaft_center))
        spindle = spindle.union(x_cylinder(0.015, 0.008, end_cap_center))

        collar = x_cylinder(COLLAR_RADIUS, COLLAR_THICKNESS, collar_center).cut(
            x_cylinder(SPINDLE_SHAFT_RADIUS, COLLAR_THICKNESS + 0.002, collar_center)
        )
        collar = collar.cut(cq_box((COLLAR_THICKNESS + 0.002, 0.012, 0.0014), (collar_center[0], 0.0, COLLAR_RADIUS - 0.0007)))
        collar = collar.union(cq_box((COLLAR_THICKNESS, 0.014, 0.008), boss_center))
        collar = collar.union(
            cq.Workplane("XZ")
            .circle(0.0025)
            .extrude(0.010, both=True)
            .translate((collar_center[0], 0.0, 0.018))
        )
        spindle = spindle.union(collar)

    return spindle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_roll_stage")

    base_mat = model.material("frame_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    support_mat = model.material("machined_support", rgba=(0.72, 0.74, 0.78, 1.0))
    spindle_mat = model.material("spindle_steel", rgba=(0.67, 0.71, 0.74, 1.0))

    base = model.part("base_frame")
    base.visual(mesh_from_cadquery(make_base_frame(), "base_frame"), material=base_mat, name="frame")
    base.inertial = None

    left_support = model.part("left_support")
    left_support.visual(mesh_from_cadquery(make_support(), "left_support"), material=support_mat, name="support")

    right_support = model.part("right_support")
    right_support.visual(mesh_from_cadquery(make_support(), "right_support"), material=support_mat, name="support")

    spindle = model.part("spindle")
    spindle.visual(mesh_from_cadquery(make_spindle(), "spindle"), material=spindle_mat, name="spindle_body")

    model.articulation(
        "base_to_left_support",
        ArticulationType.FIXED,
        parent=base,
        child=left_support,
        origin=Origin(xyz=(-SUPPORT_CENTER_X, 0.0, ROLL_AXIS_Z)),
    )
    model.articulation(
        "base_to_right_support",
        ArticulationType.FIXED,
        parent=base,
        child=right_support,
        origin=Origin(xyz=(SUPPORT_CENTER_X, 0.0, ROLL_AXIS_Z)),
    )
    model.articulation(
        "spindle_roll",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    left_support = object_model.get_part("left_support")
    right_support = object_model.get_part("right_support")
    spindle = object_model.get_part("spindle")
    roll = object_model.get_articulation("spindle_roll")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0012)
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
        "roll_axis_aligned_with_spindle",
        tuple(round(v, 6) for v in roll.axis) == (1.0, 0.0, 0.0),
        f"expected roll axis (1, 0, 0), got {roll.axis}",
    )
    ctx.expect_contact(left_support, base, name="left_support_grounded_to_frame")
    ctx.expect_contact(right_support, base, name="right_support_grounded_to_frame")
    ctx.expect_contact(spindle, left_support, contact_tol=0.0012, name="left_support_bears_spindle")
    ctx.expect_contact(spindle, right_support, contact_tol=0.0012, name="right_support_bears_spindle")
    ctx.expect_gap(
        spindle,
        base,
        axis="z",
        min_gap=0.035,
        max_gap=0.06,
        name="spindle_clears_base_frame",
    )

    with ctx.pose({roll: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_quarter_turn")
        ctx.expect_gap(
            spindle,
            base,
            axis="z",
            min_gap=0.035,
            max_gap=0.06,
            name="spindle_clear_of_frame_at_quarter_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
