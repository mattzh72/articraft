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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_WIDTH = 1.14
FRAME_DEPTH = 0.36
FRAME_HEIGHT = 1.34
FOOT_HEIGHT = 0.055
PIVOT_Z = 1.248
BRACKET_XS = (-0.32, 0.0, 0.32)
ARM_SWING_LIMIT = 0.72


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _movement_ok(
    rest_center: tuple[float, float, float] | None,
    moved_center: tuple[float, float, float] | None,
    *,
    axis: int,
    min_delta: float,
) -> bool:
    if rest_center is None or moved_center is None:
        return False
    return moved_center[axis] - rest_center[axis] >= min_delta


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_three_branch_rotary_frame")

    model.material("frame_steel", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("arm_alloy", rgba=(0.73, 0.75, 0.78, 1.0))

    support = model.part("support_frame")
    support.visual(
        Box((1.02, 0.085, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT - 0.02)),
        material="frame_steel",
        name="top_bridge",
    )

    stand_x = 0.52
    leg_y = 0.15
    leg_h = FRAME_HEIGHT - FOOT_HEIGHT - 0.06
    leg_center_z = FOOT_HEIGHT + leg_h / 2.0
    for x_pos in (-stand_x, stand_x):
        support.visual(
            Box((0.11, 0.42, FOOT_HEIGHT)),
            origin=Origin(xyz=(x_pos, 0.0, FOOT_HEIGHT / 2.0)),
            material="frame_steel",
        )
        support.visual(
            Box((0.070, FRAME_DEPTH, 0.045)),
            origin=Origin(xyz=(x_pos, 0.0, 0.34)),
            material="frame_steel",
        )
        support.visual(
            Box((0.070, FRAME_DEPTH, 0.050)),
            origin=Origin(xyz=(x_pos, 0.0, FOOT_HEIGHT + leg_h)),
            material="frame_steel",
        )
        for y_pos in (-leg_y, leg_y):
            support.visual(
                Box((0.055, 0.040, leg_h)),
                origin=Origin(xyz=(x_pos, y_pos, leg_center_z)),
                material="frame_steel",
            )

    bracket_gap = 0.022
    ear_thickness = 0.012
    ear_center_delta = bracket_gap / 2.0 + ear_thickness / 2.0
    for x_pos in BRACKET_XS:
        support.visual(
            Box((0.068, 0.058, 0.024)),
            origin=Origin(xyz=(x_pos, 0.0, FRAME_HEIGHT - 0.054)),
            material="frame_steel",
        )
        for x_delta in (-ear_center_delta, ear_center_delta):
            support.visual(
                Box((ear_thickness, 0.046, 0.064)),
                origin=Origin(xyz=(x_pos + x_delta, 0.0, FRAME_HEIGHT - 0.082)),
                material="frame_steel",
            )

    support.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    arm_parts = []
    for name in ("left_branch_arm", "center_branch_arm", "right_branch_arm"):
        arm = model.part(name)
        arm.visual(
            Cylinder(radius=0.018, length=0.022),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material="arm_alloy",
            name="eye",
        )
        arm.visual(
            Box((0.016, 0.028, 0.430)),
            origin=Origin(xyz=(0.0, 0.0, -0.225)),
            material="arm_alloy",
            name="main_stem",
        )
        arm.visual(
            Box((0.016, 0.036, 0.085)),
            origin=Origin(xyz=(0.0, 0.0, -0.435)),
            material="arm_alloy",
            name="fork_block",
        )
        arm.visual(
            Box((0.016, 0.020, 0.105)),
            origin=Origin(xyz=(0.0, -0.025, -0.482), rpy=(0.42, 0.0, 0.0)),
            material="arm_alloy",
            name="left_branch_link",
        )
        arm.visual(
            Box((0.016, 0.020, 0.105)),
            origin=Origin(xyz=(0.0, 0.025, -0.482), rpy=(-0.42, 0.0, 0.0)),
            material="arm_alloy",
            name="right_branch_link",
        )
        arm.visual(
            Box((0.016, 0.024, 0.205)),
            origin=Origin(xyz=(0.0, -0.036, -0.515), rpy=(0.52, 0.0, 0.0)),
            material="arm_alloy",
            name="left_tip_branch",
        )
        arm.visual(
            Box((0.016, 0.024, 0.205)),
            origin=Origin(xyz=(0.0, 0.036, -0.515), rpy=(-0.52, 0.0, 0.0)),
            material="arm_alloy",
            name="right_tip_branch",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.040, 0.210, 0.620)),
            mass=1.8,
            origin=Origin(xyz=(0.0, 0.0, -0.310)),
        )
        arm_parts.append(arm)

    for x_pos, arm in zip(BRACKET_XS, arm_parts):
        model.articulation(
            f"support_to_{arm.name}",
            ArticulationType.REVOLUTE,
            parent=support,
            child=arm,
            origin=Origin(xyz=(x_pos, 0.0, PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-ARM_SWING_LIMIT,
                upper=ARM_SWING_LIMIT,
                effort=18.0,
                velocity=1.2,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    left_arm = object_model.get_part("left_branch_arm")
    center_arm = object_model.get_part("center_branch_arm")
    right_arm = object_model.get_part("right_branch_arm")
    left_joint = object_model.get_articulation("support_to_left_branch_arm")
    center_joint = object_model.get_articulation("support_to_center_branch_arm")
    right_joint = object_model.get_articulation("support_to_right_branch_arm")

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

    ctx.expect_contact(left_arm, support, name="left hanging arm is captured by its bracket")
    ctx.expect_contact(center_arm, support, name="center hanging arm is captured by its bracket")
    ctx.expect_contact(right_arm, support, name="right hanging arm is captured by its bracket")

    left_rest = _aabb_center(ctx.part_world_aabb(left_arm))
    center_rest = _aabb_center(ctx.part_world_aabb(center_arm))
    right_rest = _aabb_center(ctx.part_world_aabb(right_arm))

    with ctx.pose({left_joint: 0.60}):
        left_swung = _aabb_center(ctx.part_world_aabb(left_arm))
        center_steady = _aabb_center(ctx.part_world_aabb(center_arm))
        right_steady = _aabb_center(ctx.part_world_aabb(right_arm))
        ctx.expect_contact(
            left_arm,
            support,
            name="left arm stays supported when rotated forward",
        )

    ctx.check(
        "left arm swings independently on its own bracket",
        _movement_ok(left_rest, left_swung, axis=1, min_delta=0.09),
        details=f"rest={left_rest}, swung={left_swung}",
    )
    ctx.check(
        "other arms stay still when the left arm moves",
        center_rest is not None
        and center_steady is not None
        and right_rest is not None
        and right_steady is not None
        and abs(center_steady[1] - center_rest[1]) <= 0.003
        and abs(center_steady[2] - center_rest[2]) <= 0.003
        and abs(right_steady[1] - right_rest[1]) <= 0.003
        and abs(right_steady[2] - right_rest[2]) <= 0.003,
        details=(
            f"center_rest={center_rest}, center_steady={center_steady}, "
            f"right_rest={right_rest}, right_steady={right_steady}"
        ),
    )

    with ctx.pose({center_joint: -0.55}):
        center_swung = _aabb_center(ctx.part_world_aabb(center_arm))
        left_still = _aabb_center(ctx.part_world_aabb(left_arm))
        right_still = _aabb_center(ctx.part_world_aabb(right_arm))

    ctx.check(
        "center arm has its own independent revolute support",
        center_rest is not None
        and center_swung is not None
        and center_swung[1] < center_rest[1] - 0.08
        and left_rest is not None
        and left_still is not None
        and right_rest is not None
        and right_still is not None
        and abs(left_still[1] - left_rest[1]) <= 0.003
        and abs(right_still[1] - right_rest[1]) <= 0.003,
        details=(
            f"center_rest={center_rest}, center_swung={center_swung}, "
            f"left_rest={left_rest}, left_still={left_still}, "
            f"right_rest={right_rest}, right_still={right_still}"
        ),
    )

    with ctx.pose({right_joint: 0.58}):
        right_swung = _aabb_center(ctx.part_world_aabb(right_arm))

    ctx.check(
        "right arm also reads as a separate under-slung branch",
        _movement_ok(right_rest, right_swung, axis=1, min_delta=0.08),
        details=f"rest={right_rest}, swung={right_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
