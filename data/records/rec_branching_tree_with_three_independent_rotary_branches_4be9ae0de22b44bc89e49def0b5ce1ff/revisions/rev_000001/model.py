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


BASE_SIZE = (0.34, 0.30, 0.12)
PEDESTAL_SIZE = (0.20, 0.22, 0.14)
MAST_SIZE = (0.14, 0.18, 0.72)
TOP_CAP_SIZE = (0.18, 0.22, 0.05)

MAST_FRONT_X = MAST_SIZE[0] / 2.0
JOINT_X = MAST_FRONT_X + 0.038

HUB_RADIUS = 0.022
HUB_LENGTH = 0.056
BOSS_RADIUS = 0.034
BOSS_LENGTH = 0.028
BOSS_CENTER_Y = HUB_LENGTH / 2.0 + BOSS_LENGTH / 2.0

SUPPORT_BLOCK_SIZE = (0.045, 0.05, 0.09)
SUPPORT_BLOCK_CENTER_X = MAST_FRONT_X + SUPPORT_BLOCK_SIZE[0] / 2.0
SUPPORT_BLOCK_CENTER_Y = 0.065

RIB_SIZE = (0.028, 0.04, 0.12)
RIB_CENTER_X = MAST_FRONT_X - 0.004
RIB_CENTER_Y = 0.062
RIB_Z_DROP = 0.085

BRANCH_SPECS = (
    ("lower", 0.305, 0.16),
    ("middle", 0.43, 0.145),
    ("upper", 0.59, 0.13),
)


def _box_visual(part, size, xyz, material, name) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _y_cylinder_visual(part, radius, length, xyz, material, name) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_support_set(mast, label: str, z_center: float, support_material) -> None:
    _box_visual(
        mast,
        SUPPORT_BLOCK_SIZE,
        (SUPPORT_BLOCK_CENTER_X, SUPPORT_BLOCK_CENTER_Y, z_center),
        support_material,
        f"{label}_support_left",
    )
    _box_visual(
        mast,
        SUPPORT_BLOCK_SIZE,
        (SUPPORT_BLOCK_CENTER_X, -SUPPORT_BLOCK_CENTER_Y, z_center),
        support_material,
        f"{label}_support_right",
    )
    _box_visual(
        mast,
        RIB_SIZE,
        (RIB_CENTER_X, RIB_CENTER_Y, z_center - RIB_Z_DROP),
        support_material,
        f"{label}_rib_left",
    )
    _box_visual(
        mast,
        RIB_SIZE,
        (RIB_CENTER_X, -RIB_CENTER_Y, z_center - RIB_Z_DROP),
        support_material,
        f"{label}_rib_right",
    )
    _y_cylinder_visual(
        mast,
        BOSS_RADIUS,
        BOSS_LENGTH,
        (JOINT_X, BOSS_CENTER_Y, z_center),
        support_material,
        f"{label}_boss_left",
    )
    _y_cylinder_visual(
        mast,
        BOSS_RADIUS,
        BOSS_LENGTH,
        (JOINT_X, -BOSS_CENTER_Y, z_center),
        support_material,
        f"{label}_boss_right",
    )


def _make_branch(model, name: str, arm_length: float, z_center: float, branch_material, tool_material, pad_material):
    branch = model.part(f"branch_{name}")

    _y_cylinder_visual(branch, HUB_RADIUS, HUB_LENGTH, (0.0, 0.0, 0.0), branch_material, "hub")
    _box_visual(
        branch,
        (arm_length, 0.036, 0.028),
        (HUB_RADIUS + arm_length / 2.0, 0.0, 0.0),
        branch_material,
        "arm_beam",
    )
    _box_visual(
        branch,
        (0.065, 0.026, 0.045),
        (HUB_RADIUS + 0.0325, 0.0, -0.008),
        branch_material,
        "arm_brace",
    )
    _box_visual(
        branch,
        (0.05, 0.055, 0.04),
        (HUB_RADIUS + arm_length + 0.025, 0.0, 0.0),
        tool_material,
        "tool_head",
    )
    _box_visual(
        branch,
        (0.02, 0.04, 0.026),
        (HUB_RADIUS + arm_length + 0.06, 0.0, 0.0),
        pad_material,
        "tool_pad",
    )

    model.articulation(
        f"mast_to_branch_{name}",
        ArticulationType.REVOLUTE,
        parent="mast",
        child=branch,
        origin=Origin(xyz=(JOINT_X, 0.0, z_center)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-0.5,
            upper=0.95,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_three_branch_fixture")

    body_paint = model.material("body_paint", rgba=(0.27, 0.29, 0.32, 1.0))
    support_steel = model.material("support_steel", rgba=(0.43, 0.45, 0.48, 1.0))
    branch_steel = model.material("branch_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    tool_dark = model.material("tool_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    tool_face = model.material("tool_face", rgba=(0.86, 0.47, 0.16, 1.0))

    mast = model.part("mast")
    _box_visual(
        mast,
        BASE_SIZE,
        (0.0, 0.0, BASE_SIZE[2] / 2.0),
        body_paint,
        "base_shell",
    )
    _box_visual(
        mast,
        PEDESTAL_SIZE,
        (0.0, 0.0, BASE_SIZE[2] + PEDESTAL_SIZE[2] / 2.0),
        body_paint,
        "pedestal_shroud",
    )
    _box_visual(
        mast,
        MAST_SIZE,
        (0.0, 0.0, BASE_SIZE[2] + MAST_SIZE[2] / 2.0),
        body_paint,
        "mast_column",
    )
    _box_visual(
        mast,
        (0.08, 0.05, 0.24),
        (-0.03, 0.065, BASE_SIZE[2] + 0.12),
        support_steel,
        "rear_spine_left",
    )
    _box_visual(
        mast,
        (0.08, 0.05, 0.24),
        (-0.03, -0.065, BASE_SIZE[2] + 0.12),
        support_steel,
        "rear_spine_right",
    )
    _box_visual(
        mast,
        TOP_CAP_SIZE,
        (0.0, 0.0, BASE_SIZE[2] + MAST_SIZE[2] + TOP_CAP_SIZE[2] / 2.0),
        support_steel,
        "mast_cap",
    )

    for label, z_center, _arm_length in BRANCH_SPECS:
        _add_support_set(mast, label, z_center, support_steel)

    for label, z_center, arm_length in BRANCH_SPECS:
        _make_branch(model, label, arm_length, z_center, branch_steel, tool_dark, tool_face)

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(min_corner, max_corner))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    branches = {
        "lower": object_model.get_part("branch_lower"),
        "middle": object_model.get_part("branch_middle"),
        "upper": object_model.get_part("branch_upper"),
    }
    joints = {
        "lower": object_model.get_articulation("mast_to_branch_lower"),
        "middle": object_model.get_articulation("mast_to_branch_middle"),
        "upper": object_model.get_articulation("mast_to_branch_upper"),
    }

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

    joint_ok = True
    for joint in joints.values():
        limits = joint.motion_limits
        joint_ok = joint_ok and (
            joint.articulation_type == ArticulationType.REVOLUTE
            and abs(joint.axis[0]) < 1e-9
            and abs(joint.axis[1] + 1.0) < 1e-9
            and abs(joint.axis[2]) < 1e-9
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
        )
    ctx.check(
        "three_grounded_revolute_branches",
        joint_ok,
        "Each branch should be a bounded revolute joint with a grounded ±Y support axis.",
    )

    for label, branch in branches.items():
        ctx.expect_contact(
            branch,
            mast,
            elem_a="hub",
            elem_b=f"{label}_boss_left",
            name=f"{label}_hub_contacts_left_boss",
        )
        ctx.expect_contact(
            branch,
            mast,
            elem_a="hub",
            elem_b=f"{label}_boss_right",
            name=f"{label}_hub_contacts_right_boss",
        )
        ctx.expect_gap(
            branch,
            mast,
            axis="x",
            min_gap=0.05,
            positive_elem="tool_head",
            negative_elem="mast_column",
            name=f"{label}_tool_head_projects_forward_of_mast",
        )

    for label, branch in branches.items():
        joint = joints[label]
        with ctx.pose({joint: 0.0}):
            closed_center = _aabb_center(ctx.part_element_world_aabb(branch, elem="tool_head"))
        with ctx.pose({joint: 0.75}):
            raised_center = _aabb_center(ctx.part_element_world_aabb(branch, elem="tool_head"))
            ctx.expect_gap(
                branch,
                mast,
                axis="x",
                min_gap=0.02,
                positive_elem="tool_head",
                negative_elem="mast_column",
                name=f"{label}_tool_head_clears_mast_when_raised",
            )

        rises_correctly = (
            closed_center is not None
            and raised_center is not None
            and raised_center[2] > closed_center[2] + 0.06
        )
        ctx.check(
            f"{label}_positive_rotation_lifts_tool_head",
            rises_correctly,
            "Positive branch rotation should raise the tool head above its closed position.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
