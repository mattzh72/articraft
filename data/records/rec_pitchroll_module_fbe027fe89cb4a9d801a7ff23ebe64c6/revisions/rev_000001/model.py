from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_LENGTH = 0.24
BASE_WIDTH = 0.17
BASE_THICK = 0.012

OUTER_AXIS_Z = 0.052
OUTER_SUPPORT_X = 0.078
PEDESTAL_THICK_X = 0.014
PEDESTAL_DEPTH_Y = 0.032
PEDESTAL_HEIGHT = 0.050

OUTER_RING_X = 0.010
OUTER_RING_Y = 0.090
OUTER_UPRIGHT_Y = 0.050
OUTER_UPRIGHT_Z = -0.005
OUTER_UPRIGHT_H = 0.060
OUTER_TOP_BAR_Z = 0.024
OUTER_TOP_BAR_T = 0.010
OUTER_BOTTOM_BAR_Z = -0.034
OUTER_BOTTOM_BAR_T = 0.012
OUTER_PIVOT_ARM_X = 0.022
OUTER_PIVOT_ARM_T = 0.014
OUTER_SIDE_BRACE_X = 0.012
OUTER_SIDE_BRACE_Y = 0.018
OUTER_SIDE_BRACE_H = 0.022
OUTER_SIDE_BRACE_CENTER_X = 0.043
OUTER_SIDE_BRACE_CENTER_Z = -0.017
OUTER_COLLAR_R = 0.012
OUTER_COLLAR_T = 0.004
OUTER_COLLAR_X = OUTER_SUPPORT_X - PEDESTAL_THICK_X / 2.0 - OUTER_COLLAR_T / 2.0

INNER_TRAY_W = 0.074
INNER_TRAY_D = 0.024
INNER_TRAY_T = 0.008
INNER_TRAY_CENTER_Z = -0.012
INNER_HANGER_W = 0.012
INNER_HANGER_D = 0.008
INNER_HANGER_H = 0.018
INNER_HANGER_Y = 0.015
INNER_HANGER_CENTER_Z = -0.014
INNER_LIP_T = 0.008
INNER_LIP_H = 0.020
INNER_LIP_CENTER_Z = -0.002
INNER_BEAM_R = 0.0055
INNER_COLLAR_R = 0.010
INNER_COLLAR_T = 0.008
INNER_COLLAR_Y = OUTER_UPRIGHT_Y - OUTER_RING_X / 2.0 - INNER_COLLAR_T / 2.0
INNER_BEAM_LEN = 2.0 * OUTER_UPRIGHT_Y - OUTER_RING_X


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_pitch_roll_gimbal")

    model.material("base_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("outer_finish", rgba=(0.64, 0.67, 0.71, 1.0))
    model.material("inner_finish", rgba=(0.80, 0.82, 0.85, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK / 2.0)),
        material="base_finish",
        name="base_plate",
    )
    base.visual(
        Box((PEDESTAL_THICK_X, PEDESTAL_DEPTH_Y, PEDESTAL_HEIGHT)),
        origin=Origin(xyz=(-OUTER_SUPPORT_X, 0.0, BASE_THICK + PEDESTAL_HEIGHT / 2.0)),
        material="base_finish",
        name="left_pedestal",
    )
    base.visual(
        Box((PEDESTAL_THICK_X, PEDESTAL_DEPTH_Y, PEDESTAL_HEIGHT)),
        origin=Origin(xyz=(OUTER_SUPPORT_X, 0.0, BASE_THICK + PEDESTAL_HEIGHT / 2.0)),
        material="base_finish",
        name="right_pedestal",
    )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((OUTER_SIDE_BRACE_X, OUTER_SIDE_BRACE_Y, OUTER_SIDE_BRACE_H)),
        origin=Origin(xyz=(-OUTER_SIDE_BRACE_CENTER_X, 0.0, OUTER_SIDE_BRACE_CENTER_Z)),
        material="outer_finish",
        name="outer_left_brace",
    )
    outer_frame.visual(
        Box((OUTER_SIDE_BRACE_X, OUTER_SIDE_BRACE_Y, OUTER_SIDE_BRACE_H)),
        origin=Origin(xyz=(OUTER_SIDE_BRACE_CENTER_X, 0.0, OUTER_SIDE_BRACE_CENTER_Z)),
        material="outer_finish",
        name="outer_right_brace",
    )
    outer_frame.visual(
        Box((0.100, 0.090, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material="outer_finish",
        name="outer_bottom_bar",
    )
    outer_frame.visual(
        Box((0.010, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, OUTER_UPRIGHT_Y, OUTER_UPRIGHT_Z)),
        material="outer_finish",
        name="outer_front_upright",
    )
    outer_frame.visual(
        Box((0.010, 0.010, 0.060)),
        origin=Origin(xyz=(0.0, -OUTER_UPRIGHT_Y, OUTER_UPRIGHT_Z)),
        material="outer_finish",
        name="outer_rear_upright",
    )
    outer_frame.visual(
        Box((OUTER_PIVOT_ARM_X, OUTER_PIVOT_ARM_T, OUTER_PIVOT_ARM_T)),
        origin=Origin(xyz=(-0.060, 0.0, -0.004)),
        material="outer_finish",
        name="outer_left_arm",
    )
    outer_frame.visual(
        Box((OUTER_PIVOT_ARM_X, OUTER_PIVOT_ARM_T, OUTER_PIVOT_ARM_T)),
        origin=Origin(xyz=(0.060, 0.0, -0.004)),
        material="outer_finish",
        name="outer_right_arm",
    )
    outer_frame.visual(
        Cylinder(radius=OUTER_COLLAR_R, length=OUTER_COLLAR_T),
        origin=Origin(xyz=(-OUTER_COLLAR_X, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material="outer_finish",
        name="outer_left_collar",
    )
    outer_frame.visual(
        Cylinder(radius=OUTER_COLLAR_R, length=OUTER_COLLAR_T),
        origin=Origin(xyz=(OUTER_COLLAR_X, 0.0, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material="outer_finish",
        name="outer_right_collar",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=INNER_BEAM_R, length=INNER_BEAM_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="inner_finish",
        name="inner_beam",
    )
    inner_cradle.visual(
        Box((INNER_HANGER_W, INNER_HANGER_D, INNER_HANGER_H)),
        origin=Origin(xyz=(0.0, INNER_HANGER_Y, INNER_HANGER_CENTER_Z)),
        material="inner_finish",
        name="inner_front_hanger",
    )
    inner_cradle.visual(
        Box((INNER_HANGER_W, INNER_HANGER_D, INNER_HANGER_H)),
        origin=Origin(xyz=(0.0, -INNER_HANGER_Y, INNER_HANGER_CENTER_Z)),
        material="inner_finish",
        name="inner_rear_hanger",
    )
    inner_cradle.visual(
        Box((INNER_TRAY_W, INNER_TRAY_D, INNER_TRAY_T)),
        origin=Origin(xyz=(0.0, 0.0, INNER_TRAY_CENTER_Z)),
        material="inner_finish",
        name="inner_tray",
    )
    inner_cradle.visual(
        Box((INNER_LIP_T, INNER_TRAY_D, INNER_LIP_H)),
        origin=Origin(xyz=(-(INNER_TRAY_W / 2.0 - INNER_LIP_T / 2.0), 0.0, INNER_LIP_CENTER_Z)),
        material="inner_finish",
        name="inner_left_lip",
    )
    inner_cradle.visual(
        Box((INNER_LIP_T, INNER_TRAY_D, INNER_LIP_H)),
        origin=Origin(xyz=((INNER_TRAY_W / 2.0 - INNER_LIP_T / 2.0), 0.0, INNER_LIP_CENTER_Z)),
        material="inner_finish",
        name="inner_right_lip",
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_frame,
        origin=Origin(xyz=(0.0, 0.0, OUTER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.9, upper=0.9),
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=-1.1, upper=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_frame = object_model.get_part("outer_frame")
    inner_cradle = object_model.get_part("inner_cradle")
    outer_joint = object_model.get_articulation("base_to_outer")
    inner_joint = object_model.get_articulation("outer_to_inner")

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

    ctx.expect_contact(outer_frame, base, elem_a="outer_left_arm", elem_b="left_pedestal", name="left_outer_support_contact")
    ctx.expect_contact(
        outer_frame, base, elem_a="outer_right_arm", elem_b="right_pedestal", name="right_outer_support_contact"
    )
    ctx.expect_contact(
        inner_cradle,
        outer_frame,
        elem_a="inner_beam",
        elem_b="outer_front_upright",
        name="front_inner_support_contact",
    )
    ctx.expect_contact(
        inner_cradle,
        outer_frame,
        elem_a="inner_beam",
        elem_b="outer_rear_upright",
        name="rear_inner_support_contact",
    )
    ctx.expect_gap(
        outer_frame,
        base,
        axis="z",
        min_gap=0.005,
        positive_elem="outer_bottom_bar",
        negative_elem="base_plate",
        name="outer_bottom_bar_clears_base_plate",
    )
    ctx.expect_within(outer_frame, base, axes="xy", margin=0.008, name="outer_frame_kept_over_broad_base")
    ctx.expect_origin_distance(inner_cradle, outer_frame, axes="xyz", max_dist=1e-6, name="inner_axis_centered_in_outer_frame")

    outer_axis = tuple(float(v) for v in outer_joint.axis)
    inner_axis = tuple(float(v) for v in inner_joint.axis)
    axis_dot = sum(a * b for a, b in zip(outer_axis, inner_axis))
    ctx.check(
        "gimbal_axes_are_perpendicular",
        abs(axis_dot) < 1e-9,
        f"expected perpendicular axes, got dot={axis_dot}",
    )

    with ctx.pose({outer_joint: 0.6}):
        outer_center = _aabb_center(ctx.part_element_world_aabb(outer_frame, elem="outer_bottom_bar"))
        ctx.check(
            "positive_outer_rotation_moves_bottom_heavy_frame_toward_positive_y",
            outer_center is not None and outer_center[1] > 0.004,
            f"expected outer frame center y > 0.004 at +0.6 rad, got {outer_center}",
        )

    with ctx.pose({inner_joint: 0.7}):
        inner_center = _aabb_center(ctx.part_element_world_aabb(inner_cradle, elem="inner_tray"))
        ctx.check(
            "positive_inner_rotation_moves_cradle_mass_toward_negative_x",
            inner_center is not None and inner_center[0] < -0.004,
            f"expected inner cradle center x < -0.004 at +0.7 rad, got {inner_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
