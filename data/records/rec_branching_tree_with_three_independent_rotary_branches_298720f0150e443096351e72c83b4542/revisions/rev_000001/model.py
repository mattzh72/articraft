from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, sin

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


BASE_RADIUS = 0.22
BASE_THICKNESS = 0.035
PLINTH_RADIUS = 0.11
PLINTH_HEIGHT = 0.025
COLUMN_RADIUS = 0.045
COLUMN_HEIGHT = 0.90

SUPPORT_LEN = 0.032
SUPPORT_OVERLAP = 0.006
SUPPORT_WIDTH = 0.064
SUPPORT_HEIGHT = 0.044
SUPPORT_CENTER = COLUMN_RADIUS + SUPPORT_LEN / 2.0 - SUPPORT_OVERLAP / 2.0

BLOCK_DEPTH = 0.050
BLOCK_WIDTH = 0.084
BLOCK_HEIGHT = 0.060
BLOCK_CAP_DEPTH = 0.016
JOINT_RADIUS = COLUMN_RADIUS + SUPPORT_LEN + BLOCK_DEPTH

ARM_ROOT_LENGTH = 0.034
ARM_ROOT_WIDTH = 0.040
ARM_ROOT_HEIGHT = 0.024
ARM_BEAM_LENGTH = 0.205
ARM_BEAM_WIDTH = 0.026
ARM_BEAM_HEIGHT = 0.020
ARM_END_BLOCK_LENGTH = 0.056
ARM_END_BLOCK_WIDTH = 0.052
ARM_END_BLOCK_HEIGHT = 0.052
ARM_ROOT_TO_BEAM_OVERLAP = 0.006
ARM_BEAM_TO_END_OVERLAP = 0.010
FACEPLATE_THICKNESS = 0.012
FACEPLATE_WIDTH = 0.070
FACEPLATE_HEIGHT = 0.095
FACEPLATE_OVERLAP = 0.015

BRANCH_LAYOUT = (
    ("upper_branch", "upper_block", "upper_joint", 0.78, 0.0),
    ("left_branch", "left_block", "left_joint", 0.56, 2.09439510239),
    ("right_branch", "right_block", "right_joint", 0.34, -2.09439510239),
)


def _branch_origin(radius: float, angle: float, z: float) -> Origin:
    return Origin(xyz=(radius * cos(angle), radius * sin(angle), z), rpy=(0.0, 0.0, angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tooling_tree")

    model.material("powder_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("satin_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("machined_aluminum", rgba=(0.75, 0.78, 0.81, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        material="powder_charcoal",
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        name="foot",
    )
    pedestal.visual(
        Cylinder(radius=PLINTH_RADIUS, length=PLINTH_HEIGHT),
        material="powder_charcoal",
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PLINTH_HEIGHT / 2.0)),
        name="plinth",
    )
    pedestal.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        material="powder_charcoal",
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + COLUMN_HEIGHT / 2.0)),
        name="column",
    )
    pedestal.visual(
        Cylinder(radius=COLUMN_RADIUS + 0.020, length=0.030),
        material="powder_charcoal",
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.011)),
        name="column_collar",
    )

    for index, (_, _, _, z_height, angle) in enumerate(BRANCH_LAYOUT, start=1):
        pedestal.visual(
            Box((SUPPORT_LEN + SUPPORT_OVERLAP, SUPPORT_WIDTH, SUPPORT_HEIGHT)),
            material="powder_charcoal",
            origin=Origin(
                xyz=(SUPPORT_CENTER * cos(angle), SUPPORT_CENTER * sin(angle), z_height),
                rpy=(0.0, 0.0, angle),
            ),
            name=f"support_pad_{index}",
        )

    for branch_name, block_name, joint_name, z_height, angle in BRANCH_LAYOUT:
        block = model.part(block_name)
        block.visual(
            Box((BLOCK_DEPTH, BLOCK_WIDTH, BLOCK_HEIGHT)),
            material="satin_steel",
            origin=Origin(xyz=(-BLOCK_DEPTH / 2.0, 0.0, 0.0)),
            name="block_body",
        )
        block.visual(
            Box((BLOCK_CAP_DEPTH, 0.056, 0.046)),
            material="satin_steel",
            origin=Origin(xyz=(-BLOCK_CAP_DEPTH / 2.0, 0.0, 0.0)),
            name="block_cap",
        )
        model.articulation(
            f"pedestal_to_{block_name}",
            ArticulationType.FIXED,
            parent=pedestal,
            child=block,
            origin=_branch_origin(JOINT_RADIUS, angle, z_height),
        )

        arm = model.part(branch_name)
        beam_start = ARM_ROOT_LENGTH - ARM_ROOT_TO_BEAM_OVERLAP
        beam_end = beam_start + ARM_BEAM_LENGTH
        end_start = beam_end - ARM_BEAM_TO_END_OVERLAP
        end_center = end_start + ARM_END_BLOCK_LENGTH / 2.0
        end_face_x = end_start + ARM_END_BLOCK_LENGTH
        arm.visual(
            Box((ARM_ROOT_LENGTH, ARM_ROOT_WIDTH, ARM_ROOT_HEIGHT)),
            material="satin_steel",
            origin=Origin(xyz=(ARM_ROOT_LENGTH / 2.0, 0.0, 0.0)),
            name="root_head",
        )
        arm.visual(
            Box((ARM_BEAM_LENGTH, ARM_BEAM_WIDTH, ARM_BEAM_HEIGHT)),
            material="satin_steel",
            origin=Origin(xyz=(beam_start + ARM_BEAM_LENGTH / 2.0, 0.0, 0.0)),
            name="arm_beam",
        )
        arm.visual(
            Box((ARM_END_BLOCK_LENGTH, ARM_END_BLOCK_WIDTH, ARM_END_BLOCK_HEIGHT)),
            material="satin_steel",
            origin=Origin(xyz=(end_center, 0.0, 0.0)),
            name="end_head",
        )
        arm.visual(
            Box((FACEPLATE_THICKNESS, FACEPLATE_WIDTH, FACEPLATE_HEIGHT)),
            material="machined_aluminum",
            origin=Origin(
                xyz=(
                    end_face_x - FACEPLATE_OVERLAP + FACEPLATE_THICKNESS / 2.0,
                    0.0,
                    0.0,
                )
            ),
            name="faceplate",
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=block,
            child=arm,
            origin=Origin(),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.6,
                lower=-1.0,
                upper=1.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    pedestal = object_model.get_part("pedestal")
    blocks = [object_model.get_part(block_name) for _, block_name, _, _, _ in BRANCH_LAYOUT]
    arms = [object_model.get_part(branch_name) for branch_name, _, _, _, _ in BRANCH_LAYOUT]
    joints = [object_model.get_articulation(joint_name) for _, _, joint_name, _, _ in BRANCH_LAYOUT]

    for branch_name, block_name, joint_name, _, _ in BRANCH_LAYOUT:
        block = object_model.get_part(block_name)
        arm = object_model.get_part(branch_name)
        joint = object_model.get_articulation(joint_name)

        ctx.expect_contact(block, pedestal, name=f"{block_name}_mounted_to_column")
        ctx.expect_contact(arm, block, name=f"{branch_name}_supported_at_pivot")
        ctx.check(
            f"{joint_name}_uses_vertical_revolute_axis",
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"{joint_name} should be a vertical revolute joint but is {joint.articulation_type} axis={joint.axis}",
        )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        low, high = aabb
        return (
            0.5 * (low[0] + high[0]),
            0.5 * (low[1] + high[1]),
            0.5 * (low[2] + high[2]),
        )

    def _planar_shift(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float | None:
        if a is None or b is None:
            return None
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return (dx * dx + dy * dy) ** 0.5

    with ctx.pose({joint: 0.0 for joint in joints}):
        closed_arm_centers = {arm.name: _aabb_center(ctx.part_world_aabb(arm)) for arm in arms}

    for active_arm, active_joint in zip(arms, joints):
        other_arms = [arm for arm in arms if arm.name != active_arm.name]
        with ctx.pose({active_joint: 0.8}):
            moved_center = _aabb_center(ctx.part_world_aabb(active_arm))
            stationary_centers = {other.name: _aabb_center(ctx.part_world_aabb(other)) for other in other_arms}

        moved_shift = _planar_shift(moved_center, closed_arm_centers.get(active_arm.name))
        stationary_ok = True
        for other in other_arms:
            shift = _planar_shift(stationary_centers.get(other.name), closed_arm_centers.get(other.name))
            stationary_ok = stationary_ok and shift is not None and shift <= 1e-5

        ctx.check(
            f"{active_joint.name}_moves_only_its_branch",
            moved_shift is not None and moved_shift >= 0.06 and stationary_ok,
            details=(
                f"{active_joint.name} moved its branch by {moved_shift} m in plan; "
                f"other branches must remain fixed in the same pose."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
