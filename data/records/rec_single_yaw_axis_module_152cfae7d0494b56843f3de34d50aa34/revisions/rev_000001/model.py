from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

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


BASE_X = 0.28
BASE_Y = 0.18
BASE_Z = 0.018

SUPPORT_X = 0.024
SUPPORT_Y = 0.070
SUPPORT_HEIGHT = 0.192
SUPPORT_CENTER_X = 0.103

BRIDGE_X = 0.230
BRIDGE_Y = 0.080
BRIDGE_Z = 0.018
BRIDGE_BOTTOM_Z = BASE_Z + SUPPORT_HEIGHT - BRIDGE_Z

SHAFT_RADIUS = 0.014
SHAFT_HOLE_RADIUS = 0.0165
SHAFT_LENGTH = 0.180

LOWER_BEARING_X = 0.062
LOWER_BEARING_HALF_Y = 0.020
LOWER_BEARING_OFFSET_Y = 0.028
LOWER_BEARING_BLOCK_Z = 0.026
LOWER_BEARING_CAP_Z = 0.008
LOWER_BEARING_TOP_Z = BASE_Z + LOWER_BEARING_BLOCK_Z + LOWER_BEARING_CAP_Z

UPPER_BEARING_X = 0.058
UPPER_BEARING_HALF_Y = 0.018
UPPER_BEARING_OFFSET_Y = 0.027
UPPER_BEARING_BLOCK_Z = 0.024
UPPER_BEARING_CAP_Z = 0.008
UPPER_BEARING_BOTTOM_Z = BRIDGE_BOTTOM_Z - UPPER_BEARING_BLOCK_Z - UPPER_BEARING_CAP_Z

HEAD_SWEEP_RADIUS = 0.078
LOWER_COLLAR_LENGTH = 0.010
UPPER_COLLAR_LENGTH = 0.010
LOWER_COLLAR_CENTER_Z = 0.045
UPPER_COLLAR_CENTER_Z = 0.136
HEAD_FACE_Z = 0.105

SUPPORT_INNER_X = SUPPORT_CENTER_X - SUPPORT_X * 0.5
BEARING_INNER_X = 0.029
BEARING_ARM_LENGTH = SUPPORT_INNER_X - BEARING_INNER_X
BEARING_ARM_CENTER_X = BEARING_INNER_X + BEARING_ARM_LENGTH * 0.5
LOWER_BEARING_BOTTOM_Z = BASE_Z
UPPER_BEARING_TOP_Z = BRIDGE_BOTTOM_Z


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_axis_rotary_module")

    model.material("frame_paint", rgba=(0.18, 0.21, 0.24, 1.0))
    model.material("bearing_cap", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("tool_steel", rgba=(0.66, 0.68, 0.71, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_X, BASE_Y, BASE_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z * 0.5)),
        material="frame_paint",
        name="base_plate",
    )
    frame.visual(
        Box((SUPPORT_X, SUPPORT_Y, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(-SUPPORT_CENTER_X, 0.0, BASE_Z + SUPPORT_HEIGHT * 0.5)),
        material="frame_paint",
        name="left_support",
    )
    frame.visual(
        Box((SUPPORT_X, SUPPORT_Y, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(SUPPORT_CENTER_X, 0.0, BASE_Z + SUPPORT_HEIGHT * 0.5)),
        material="frame_paint",
        name="right_support",
    )
    frame.visual(
        Box((BRIDGE_X, 0.016, BRIDGE_Z)),
        origin=Origin(xyz=(0.0, 0.026, BRIDGE_BOTTOM_Z + BRIDGE_Z * 0.5)),
        material="frame_paint",
        name="front_bridge",
    )
    frame.visual(
        Box((BRIDGE_X, 0.016, BRIDGE_Z)),
        origin=Origin(xyz=(0.0, -0.026, BRIDGE_BOTTOM_Z + BRIDGE_Z * 0.5)),
        material="frame_paint",
        name="rear_bridge",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, LOWER_BEARING_HALF_Y * 2.0, LOWER_BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(-BEARING_ARM_CENTER_X, LOWER_BEARING_OFFSET_Y, LOWER_BEARING_BOTTOM_Z + LOWER_BEARING_BLOCK_Z * 0.5)
        ),
        material="bearing_cap",
        name="lower_arm_left_front",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, LOWER_BEARING_HALF_Y * 2.0, LOWER_BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(-BEARING_ARM_CENTER_X, -LOWER_BEARING_OFFSET_Y, LOWER_BEARING_BOTTOM_Z + LOWER_BEARING_BLOCK_Z * 0.5)
        ),
        material="bearing_cap",
        name="lower_arm_left_rear",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, LOWER_BEARING_HALF_Y * 2.0, LOWER_BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(BEARING_ARM_CENTER_X, LOWER_BEARING_OFFSET_Y, LOWER_BEARING_BOTTOM_Z + LOWER_BEARING_BLOCK_Z * 0.5)
        ),
        material="bearing_cap",
        name="lower_arm_right_front",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, LOWER_BEARING_HALF_Y * 2.0, LOWER_BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(BEARING_ARM_CENTER_X, -LOWER_BEARING_OFFSET_Y, LOWER_BEARING_BOTTOM_Z + LOWER_BEARING_BLOCK_Z * 0.5)
        ),
        material="bearing_cap",
        name="lower_arm_right_rear",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, LOWER_BEARING_HALF_Y * 2.4, LOWER_BEARING_CAP_Z)),
        origin=Origin(xyz=(-BEARING_ARM_CENTER_X, LOWER_BEARING_OFFSET_Y, LOWER_BEARING_TOP_Z - LOWER_BEARING_CAP_Z * 0.5)),
        material="bearing_cap",
        name="lower_cap_left_front",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, LOWER_BEARING_HALF_Y * 2.4, LOWER_BEARING_CAP_Z)),
        origin=Origin(xyz=(-BEARING_ARM_CENTER_X, -LOWER_BEARING_OFFSET_Y, LOWER_BEARING_TOP_Z - LOWER_BEARING_CAP_Z * 0.5)),
        material="bearing_cap",
        name="lower_cap_left_rear",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, LOWER_BEARING_HALF_Y * 2.4, LOWER_BEARING_CAP_Z)),
        origin=Origin(xyz=(BEARING_ARM_CENTER_X, LOWER_BEARING_OFFSET_Y, LOWER_BEARING_TOP_Z - LOWER_BEARING_CAP_Z * 0.5)),
        material="bearing_cap",
        name="lower_cap_right_front",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, LOWER_BEARING_HALF_Y * 2.4, LOWER_BEARING_CAP_Z)),
        origin=Origin(xyz=(BEARING_ARM_CENTER_X, -LOWER_BEARING_OFFSET_Y, LOWER_BEARING_TOP_Z - LOWER_BEARING_CAP_Z * 0.5)),
        material="bearing_cap",
        name="lower_cap_right_rear",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, UPPER_BEARING_HALF_Y * 2.0, UPPER_BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(-BEARING_ARM_CENTER_X, UPPER_BEARING_OFFSET_Y, UPPER_BEARING_BOTTOM_Z + UPPER_BEARING_BLOCK_Z * 0.5)
        ),
        material="bearing_cap",
        name="upper_arm_left_front",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, UPPER_BEARING_HALF_Y * 2.0, UPPER_BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(-BEARING_ARM_CENTER_X, -UPPER_BEARING_OFFSET_Y, UPPER_BEARING_BOTTOM_Z + UPPER_BEARING_BLOCK_Z * 0.5)
        ),
        material="bearing_cap",
        name="upper_arm_left_rear",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, UPPER_BEARING_HALF_Y * 2.0, UPPER_BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(BEARING_ARM_CENTER_X, UPPER_BEARING_OFFSET_Y, UPPER_BEARING_BOTTOM_Z + UPPER_BEARING_BLOCK_Z * 0.5)
        ),
        material="bearing_cap",
        name="upper_arm_right_front",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, UPPER_BEARING_HALF_Y * 2.0, UPPER_BEARING_BLOCK_Z)),
        origin=Origin(
            xyz=(BEARING_ARM_CENTER_X, -UPPER_BEARING_OFFSET_Y, UPPER_BEARING_BOTTOM_Z + UPPER_BEARING_BLOCK_Z * 0.5)
        ),
        material="bearing_cap",
        name="upper_arm_right_rear",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, UPPER_BEARING_HALF_Y * 2.4, UPPER_BEARING_CAP_Z)),
        origin=Origin(xyz=(-BEARING_ARM_CENTER_X, UPPER_BEARING_OFFSET_Y, UPPER_BEARING_TOP_Z - UPPER_BEARING_CAP_Z * 0.5)),
        material="bearing_cap",
        name="upper_cap_left_front",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, UPPER_BEARING_HALF_Y * 2.4, UPPER_BEARING_CAP_Z)),
        origin=Origin(xyz=(-BEARING_ARM_CENTER_X, -UPPER_BEARING_OFFSET_Y, UPPER_BEARING_TOP_Z - UPPER_BEARING_CAP_Z * 0.5)),
        material="bearing_cap",
        name="upper_cap_left_rear",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, UPPER_BEARING_HALF_Y * 2.4, UPPER_BEARING_CAP_Z)),
        origin=Origin(xyz=(BEARING_ARM_CENTER_X, UPPER_BEARING_OFFSET_Y, UPPER_BEARING_TOP_Z - UPPER_BEARING_CAP_Z * 0.5)),
        material="bearing_cap",
        name="upper_cap_right_front",
    )
    frame.visual(
        Box((BEARING_ARM_LENGTH, UPPER_BEARING_HALF_Y * 2.4, UPPER_BEARING_CAP_Z)),
        origin=Origin(xyz=(BEARING_ARM_CENTER_X, -UPPER_BEARING_OFFSET_Y, UPPER_BEARING_TOP_Z - UPPER_BEARING_CAP_Z * 0.5)),
        material="bearing_cap",
        name="upper_cap_right_rear",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_LENGTH * 0.5)),
        material="tool_steel",
        name="shaft",
    )
    head.visual(
        Cylinder(radius=0.024, length=LOWER_COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, LOWER_COLLAR_CENTER_Z)),
        material="tool_steel",
        name="lower_collar",
    )
    head.visual(
        Cylinder(radius=0.023, length=UPPER_COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, UPPER_COLLAR_CENTER_Z)),
        material="tool_steel",
        name="upper_collar",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material="tool_steel",
        name="hub",
    )
    head.visual(
        Cylinder(radius=0.048, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, HEAD_FACE_Z)),
        material="tool_steel",
        name="faceplate",
    )
    head.visual(
        Box((0.046, 0.026, 0.014)),
        origin=Origin(xyz=(0.056, 0.0, HEAD_FACE_Z)),
        material="tool_steel",
        name="clamp_pad",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.076, 0.0, HEAD_FACE_Z)),
        material="tool_steel",
        name="clamp_knob",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material="tool_steel",
        name="top_cap",
    )

    model.articulation(
        "frame_to_head",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.5, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    rotary = object_model.get_articulation("frame_to_head")
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

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_within(head, frame, axes="xy", margin=0.0, name="head_within_frame_footprint")

    lower_aabb = ctx.part_element_world_aabb(frame, elem="lower_cap_left_front")
    upper_aabb = ctx.part_element_world_aabb(frame, elem="upper_arm_left_front")
    head_aabb = ctx.part_world_aabb(head)

    if lower_aabb is not None and upper_aabb is not None and head_aabb is not None:
        visible_span = upper_aabb[0][2] - lower_aabb[1][2]
        ctx.check(
            "open_bearing_span_visible",
            visible_span >= 0.100,
            details=f"bearing gap is {visible_span:.4f} m; expected at least 0.1000 m for a readable open frame",
        )

        shaft_runs_through_supports = head_aabb[0][2] <= lower_aabb[1][2] and head_aabb[1][2] >= upper_aabb[0][2]
        ctx.check(
            "head_spans_both_bearings",
            shaft_runs_through_supports,
            details=(
                f"head z-span {head_aabb[0][2]:.4f}..{head_aabb[1][2]:.4f} m does not cover "
                f"lower/upper bearing elevations {lower_aabb[1][2]:.4f} and {upper_aabb[0][2]:.4f} m"
            ),
        )

    ctx.check(
        "rotary_axis_vertical",
        tuple(round(v, 6) for v in rotary.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical axis (0, 0, 1), got {rotary.axis}",
    )

    head_center_closed = None
    head_center_quarter_turn = None
    with ctx.pose({rotary: 0.0}):
        closed_aabb = ctx.part_world_aabb(head)
        if closed_aabb is not None:
            head_center_closed = _aabb_center(closed_aabb)
    with ctx.pose({rotary: pi / 2.0}):
        quarter_turn_aabb = ctx.part_world_aabb(head)
        if quarter_turn_aabb is not None:
            head_center_quarter_turn = _aabb_center(quarter_turn_aabb)

    if head_center_closed is not None and head_center_quarter_turn is not None:
        planar_shift = sqrt(
            (head_center_quarter_turn[0] - head_center_closed[0]) ** 2
            + (head_center_quarter_turn[1] - head_center_closed[1]) ** 2
        )
        ctx.check(
            "rotary_joint_turns_visible_head",
            planar_shift >= 0.010,
            details=f"asymmetric head center shifted only {planar_shift:.4f} m between 0 and 90 degrees",
        )

    ctx.check(
        "head_radius_clears_supports",
        HEAD_SWEEP_RADIUS <= SUPPORT_CENTER_X - SUPPORT_X * 0.5 - 0.010,
        details=(
            f"head sweep radius {HEAD_SWEEP_RADIUS:.4f} m exceeds readable side clearance "
            f"inside supports {(SUPPORT_CENTER_X - SUPPORT_X * 0.5 - 0.010):.4f} m"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
