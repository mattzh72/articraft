from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BASE_SIZE = (0.18, 0.12, 0.02)
SPINE_SIZE = (0.05, 0.08, 0.40)

HUB_RADIUS = 0.018
HUB_LENGTH = 0.024
PLATE_THICKNESS = 0.006
PLATE_OFFSET_Y = HUB_LENGTH / 2.0 + PLATE_THICKNESS / 2.0
PLATE_SIZE = (0.03, PLATE_THICKNESS, 0.052)
ROOT_BLOCK_SIZE = (0.046, 0.036, 0.042)

UPPER_HUB_X = 0.066
LOWER_HUB_X = -0.066
UPPER_HUB_Z = 0.31
LOWER_HUB_Z = 0.16

BEAM_LENGTH = 0.09
BEAM_SIZE = (BEAM_LENGTH, 0.02, 0.024)
PAD_SIZE = (0.036, 0.06, 0.012)
TRANSITION_SIZE = (0.02, 0.03, 0.024)

TRAVEL_HALF_ANGLE = math.radians(60.0)


def _add_branch_visuals(part, direction: float, prefix: str, material) -> None:
    part.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_hub",
    )
    part.visual(
        Box(BEAM_SIZE),
        origin=Origin(xyz=(direction * (HUB_RADIUS + BEAM_LENGTH / 2.0), 0.0, 0.0)),
        material=material,
        name=f"{prefix}_beam",
    )
    part.visual(
        Box(TRANSITION_SIZE),
        origin=Origin(
            xyz=(direction * (HUB_RADIUS + BEAM_LENGTH - TRANSITION_SIZE[0] / 2.0), 0.0, 0.0)
        ),
        material=material,
        name=f"{prefix}_transition",
    )
    part.visual(
        Box(PAD_SIZE),
        origin=Origin(
            xyz=(direction * (HUB_RADIUS + BEAM_LENGTH + PAD_SIZE[0] / 2.0), 0.0, 0.0)
        ),
        material=material,
        name=f"{prefix}_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_rotary_fixture", assets=ASSETS)

    dark_frame = model.material("dark_frame", rgba=(0.22, 0.24, 0.27, 1.0))
    branch_finish = model.material("branch_finish", rgba=(0.66, 0.70, 0.73, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box(BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] / 2.0)),
        material=dark_frame,
        name="base_plate",
    )
    spine.visual(
        Box(SPINE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] + SPINE_SIZE[2] / 2.0)),
        material=dark_frame,
        name="spine_column",
    )

    spine.visual(
        Box(ROOT_BLOCK_SIZE),
        origin=Origin(xyz=(0.025, 0.0, UPPER_HUB_Z)),
        material=dark_frame,
        name="upper_root",
    )
    spine.visual(
        Box(PLATE_SIZE),
        origin=Origin(xyz=(0.063, PLATE_OFFSET_Y, UPPER_HUB_Z)),
        material=dark_frame,
        name="upper_plate_pos",
    )
    spine.visual(
        Box(PLATE_SIZE),
        origin=Origin(xyz=(0.063, -PLATE_OFFSET_Y, UPPER_HUB_Z)),
        material=dark_frame,
        name="upper_plate_neg",
    )

    spine.visual(
        Box(ROOT_BLOCK_SIZE),
        origin=Origin(xyz=(-0.025, 0.0, LOWER_HUB_Z)),
        material=dark_frame,
        name="lower_root",
    )
    spine.visual(
        Box(PLATE_SIZE),
        origin=Origin(xyz=(-0.063, PLATE_OFFSET_Y, LOWER_HUB_Z)),
        material=dark_frame,
        name="lower_plate_pos",
    )
    spine.visual(
        Box(PLATE_SIZE),
        origin=Origin(xyz=(-0.063, -PLATE_OFFSET_Y, LOWER_HUB_Z)),
        material=dark_frame,
        name="lower_plate_neg",
    )

    upper_branch = model.part("upper_branch")
    _add_branch_visuals(upper_branch, direction=1.0, prefix="upper", material=branch_finish)

    lower_branch = model.part("lower_branch")
    _add_branch_visuals(lower_branch, direction=-1.0, prefix="lower", material=branch_finish)

    model.articulation(
        "spine_to_upper_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_branch,
        origin=Origin(xyz=(UPPER_HUB_X, 0.0, UPPER_HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-TRAVEL_HALF_ANGLE,
            upper=TRAVEL_HALF_ANGLE,
        ),
    )
    model.articulation(
        "spine_to_lower_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_branch,
        origin=Origin(xyz=(LOWER_HUB_X, 0.0, LOWER_HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-TRAVEL_HALF_ANGLE,
            upper=TRAVEL_HALF_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    spine = object_model.get_part("spine")
    upper_branch = object_model.get_part("upper_branch")
    lower_branch = object_model.get_part("lower_branch")

    upper_joint = object_model.get_articulation("spine_to_upper_branch")
    lower_joint = object_model.get_articulation("spine_to_lower_branch")

    upper_hub = upper_branch.get_visual("upper_hub")
    upper_pad = upper_branch.get_visual("upper_pad")
    lower_hub = lower_branch.get_visual("lower_hub")
    lower_pad = lower_branch.get_visual("lower_pad")

    upper_plate_pos = spine.get_visual("upper_plate_pos")
    upper_plate_neg = spine.get_visual("upper_plate_neg")
    lower_plate_pos = spine.get_visual("lower_plate_pos")
    lower_plate_neg = spine.get_visual("lower_plate_neg")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_gap(...)`, `expect_overlap(...)`, `expect_contact(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "fixture_parts_present",
        spine is not None and upper_branch is not None and lower_branch is not None,
        "Expected spine, upper_branch, and lower_branch parts.",
    )
    ctx.check(
        "upper_joint_definition",
        upper_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(upper_joint.axis) == (0.0, 1.0, 0.0)
        and math.isclose(upper_joint.motion_limits.lower, -TRAVEL_HALF_ANGLE, abs_tol=1e-9)
        and math.isclose(upper_joint.motion_limits.upper, TRAVEL_HALF_ANGLE, abs_tol=1e-9),
        "Upper branch should use a revolute joint about +Y with 120 degrees total travel.",
    )
    ctx.check(
        "lower_joint_definition",
        lower_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lower_joint.axis) == (0.0, 1.0, 0.0)
        and math.isclose(lower_joint.motion_limits.lower, -TRAVEL_HALF_ANGLE, abs_tol=1e-9)
        and math.isclose(lower_joint.motion_limits.upper, TRAVEL_HALF_ANGLE, abs_tol=1e-9),
        "Lower branch should use a revolute joint about +Y with 120 degrees total travel.",
    )

    ctx.expect_gap(
        spine,
        upper_branch,
        axis="y",
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=upper_plate_pos,
        negative_elem=upper_hub,
        name="upper_hub_seats_against_positive_plate",
    )
    ctx.expect_gap(
        upper_branch,
        spine,
        axis="y",
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=upper_hub,
        negative_elem=upper_plate_neg,
        name="upper_hub_seats_against_negative_plate",
    )
    ctx.expect_gap(
        spine,
        lower_branch,
        axis="y",
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=lower_plate_pos,
        negative_elem=lower_hub,
        name="lower_hub_seats_against_positive_plate",
    )
    ctx.expect_gap(
        lower_branch,
        spine,
        axis="y",
        min_gap=0.0,
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=lower_hub,
        negative_elem=lower_plate_neg,
        name="lower_hub_seats_against_negative_plate",
    )

    ctx.expect_overlap(
        upper_branch,
        spine,
        axes="xz",
        min_overlap=0.02,
        elem_a=upper_hub,
        elem_b=upper_plate_pos,
        name="upper_hub_captured_in_upper_yoke",
    )
    ctx.expect_overlap(
        lower_branch,
        spine,
        axes="xz",
        min_overlap=0.02,
        elem_a=lower_hub,
        elem_b=lower_plate_pos,
        name="lower_hub_captured_in_lower_yoke",
    )
    ctx.expect_origin_gap(
        upper_branch,
        lower_branch,
        axis="z",
        min_gap=0.12,
        name="upper_hub_is_above_lower_hub",
    )

    with ctx.pose({upper_joint: TRAVEL_HALF_ANGLE}):
        upper_pad_aabb = ctx.part_element_world_aabb(upper_branch, elem=upper_pad)
        upper_pad_center_z = (upper_pad_aabb[0][2] + upper_pad_aabb[1][2]) / 2.0
        ctx.check(
            "upper_branch_rotates_downward_at_positive_limit",
            upper_pad_center_z < UPPER_HUB_Z - 0.08,
            "Upper pad should drop noticeably when the upper joint rotates positive.",
        )

    with ctx.pose({upper_joint: -TRAVEL_HALF_ANGLE}):
        upper_pad_aabb = ctx.part_element_world_aabb(upper_branch, elem=upper_pad)
        upper_pad_center_z = (upper_pad_aabb[0][2] + upper_pad_aabb[1][2]) / 2.0
        ctx.check(
            "upper_branch_rotates_upward_at_negative_limit",
            upper_pad_center_z > UPPER_HUB_Z + 0.08,
            "Upper pad should rise noticeably when the upper joint rotates negative.",
        )

    with ctx.pose({lower_joint: TRAVEL_HALF_ANGLE}):
        lower_pad_aabb = ctx.part_element_world_aabb(lower_branch, elem=lower_pad)
        lower_pad_center_z = (lower_pad_aabb[0][2] + lower_pad_aabb[1][2]) / 2.0
        ctx.check(
            "lower_branch_rotates_upward",
            lower_pad_center_z > LOWER_HUB_Z + 0.08,
            "Lower pad should rise noticeably when the lower joint rotates positive.",
        )

    with ctx.pose({lower_joint: -TRAVEL_HALF_ANGLE}):
        lower_pad_aabb = ctx.part_element_world_aabb(lower_branch, elem=lower_pad)
        lower_pad_center_z = (lower_pad_aabb[0][2] + lower_pad_aabb[1][2]) / 2.0
        ctx.check(
            "lower_branch_rotates_downward",
            lower_pad_center_z < LOWER_HUB_Z - 0.08,
            "Lower pad should drop noticeably when the lower joint rotates negative.",
        )

    with ctx.pose({upper_joint: TRAVEL_HALF_ANGLE, lower_joint: -TRAVEL_HALF_ANGLE}):
        ctx.expect_gap(
            upper_branch,
            lower_branch,
            axis="z",
            min_gap=0.06,
            positive_elem=upper_pad,
            negative_elem=lower_pad,
            name="pads_remain_separated_at_opposing_extremes",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
