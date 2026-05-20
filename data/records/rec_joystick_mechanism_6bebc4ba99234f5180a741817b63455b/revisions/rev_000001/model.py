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
    Sphere,
    TestContext,
    TestReport,
)


ROTATE_CYLINDER_TO_Y = (-pi / 2.0, 0.0, 0.0)
ROTATE_CYLINDER_TO_X = (0.0, pi / 2.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_joystick")

    model.material("bracket_paint", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("black_oxide", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    bracket = model.part("bracket")
    bracket.visual(
        Box((0.160, 0.080, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material="bracket_paint",
        name="top_plate",
    )
    bracket.visual(
        Box((0.020, 0.004, 0.048)),
        origin=Origin(xyz=(0.0, 0.034, 0.018)),
        material="bracket_paint",
        name="front_ear",
    )
    bracket.visual(
        Box((0.020, 0.004, 0.048)),
        origin=Origin(xyz=(0.0, -0.034, 0.018)),
        material="bracket_paint",
        name="rear_ear",
    )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        Box((0.104, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material="black_oxide",
        name="outer_top_bridge",
    )
    outer_yoke.visual(
        Box((0.012, 0.008, 0.028)),
        origin=Origin(xyz=(-0.046, 0.0, -0.028)),
        material="black_oxide",
        name="outer_left_upper_arm",
    )
    outer_yoke.visual(
        Box((0.012, 0.008, 0.050)),
        origin=Origin(xyz=(-0.046, 0.0, -0.077)),
        material="black_oxide",
        name="outer_left_lower_arm",
    )
    outer_yoke.visual(
        Box((0.012, 0.008, 0.010)),
        origin=Origin(xyz=(-0.046, 0.0, -0.047)),
        material="black_oxide",
        name="outer_left_mid_link",
    )
    outer_yoke.visual(
        Box((0.012, 0.008, 0.028)),
        origin=Origin(xyz=(0.046, 0.0, -0.028)),
        material="black_oxide",
        name="outer_right_upper_arm",
    )
    outer_yoke.visual(
        Box((0.012, 0.008, 0.050)),
        origin=Origin(xyz=(0.046, 0.0, -0.077)),
        material="black_oxide",
        name="outer_right_lower_arm",
    )
    outer_yoke.visual(
        Box((0.012, 0.008, 0.010)),
        origin=Origin(xyz=(0.046, 0.0, -0.047)),
        material="black_oxide",
        name="outer_right_mid_link",
    )
    outer_yoke.visual(
        Cylinder(radius=0.006, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=ROTATE_CYLINDER_TO_Y),
        material="black_oxide",
        name="outer_pivot_rod",
    )
    outer_yoke.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.0, 0.028, 0.0), rpy=ROTATE_CYLINDER_TO_Y),
        material="black_oxide",
        name="outer_front_collar",
    )
    outer_yoke.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.0, -0.028, 0.0), rpy=ROTATE_CYLINDER_TO_Y),
        material="black_oxide",
        name="outer_rear_collar",
    )

    inner_yoke = model.part("inner_yoke")
    inner_yoke.visual(
        Box((0.080, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material="grip_black",
        name="inner_top_bridge",
    )
    inner_yoke.visual(
        Box((0.008, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.024, -0.040)),
        material="grip_black",
        name="inner_front_arm",
    )
    inner_yoke.visual(
        Box((0.008, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.024, -0.040)),
        material="grip_black",
        name="inner_rear_arm",
    )
    inner_yoke.visual(
        Box((0.008, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material="grip_black",
        name="inner_bottom_bridge",
    )
    inner_yoke.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material="grip_black",
        name="stick_hub",
    )
    inner_yoke.visual(
        Cylinder(radius=0.006, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, -0.131)),
        material="grip_black",
        name="stick_stem",
    )
    inner_yoke.visual(
        Cylinder(radius=0.011, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.213)),
        material="grip_black",
        name="stick_grip",
    )
    inner_yoke.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.267)),
        material="grip_black",
        name="stick_tip",
    )

    model.articulation(
        "bracket_to_outer",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.047)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_yoke = object_model.get_part("inner_yoke")
    bracket_to_outer = object_model.get_articulation("bracket_to_outer")
    outer_to_inner = object_model.get_articulation("outer_to_inner")

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

    ctx.expect_contact(
        outer_yoke,
        bracket,
        contact_tol=0.001,
        name="outer_yoke_is_carried_by_bracket",
    )
    ctx.expect_contact(
        inner_yoke,
        outer_yoke,
        contact_tol=0.001,
        name="inner_yoke_is_carried_by_outer_yoke",
    )

    ctx.check(
        "joint_axes_are_orthogonal",
        abs(
            sum(
                a * b
                for a, b in zip(bracket_to_outer.axis, outer_to_inner.axis)
            )
        )
        < 1e-9,
        details=(
            f"expected orthogonal axes, got {bracket_to_outer.axis} "
            f"and {outer_to_inner.axis}"
        ),
    )
    ctx.check(
        "outer_joint_uses_front_back_axis",
        bracket_to_outer.axis == (0.0, 1.0, 0.0),
        details=f"unexpected outer-axis vector: {bracket_to_outer.axis}",
    )
    ctx.check(
        "inner_joint_uses_left_right_axis",
        outer_to_inner.axis == (1.0, 0.0, 0.0),
        details=f"unexpected inner-axis vector: {outer_to_inner.axis}",
    )

    outer_aabb = ctx.part_world_aabb(outer_yoke)
    inner_aabb = ctx.part_world_aabb(inner_yoke)
    stick_projects_low_enough = (
        outer_aabb is not None
        and inner_aabb is not None
        and (inner_aabb[0][2] < (outer_aabb[0][2] - 0.090))
    )
    ctx.check(
        "stick_projects_below_nested_yokes",
        stick_projects_low_enough,
        details=f"outer={outer_aabb}, inner={inner_aabb}",
    )

    rest_tip_aabb = ctx.part_element_world_aabb(inner_yoke, elem="stick_tip")

    with ctx.pose({bracket_to_outer: 0.35, outer_to_inner: -0.35}):
        tilted_tip_aabb = ctx.part_element_world_aabb(inner_yoke, elem="stick_tip")
        tilted_bracket_aabb = ctx.part_world_aabb(bracket)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    rest_tip_center = _aabb_center(rest_tip_aabb)
    tilted_tip_center = _aabb_center(tilted_tip_aabb)
    tip_shift_ok = (
        rest_tip_center is not None
        and tilted_tip_center is not None
        and (
            abs(tilted_tip_center[0] - rest_tip_center[0]) > 0.03
            or abs(tilted_tip_center[1] - rest_tip_center[1]) > 0.03
        )
    )
    ctx.check(
        "compound_tilt_moves_stick_tip_sideways",
        tip_shift_ok,
        details=f"rest_tip={rest_tip_center}, tilted_tip={tilted_tip_center}",
    )
    ctx.check(
        "compound_tilt_keeps_tip_below_bracket",
        tilted_tip_aabb is not None
        and tilted_bracket_aabb is not None
        and tilted_tip_aabb[1][2] < tilted_bracket_aabb[0][2],
        details=f"tilted_tip={tilted_tip_aabb}, bracket={tilted_bracket_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
