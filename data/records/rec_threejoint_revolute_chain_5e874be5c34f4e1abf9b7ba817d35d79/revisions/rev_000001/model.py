from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HINGE_BLOCK_LENGTH = 0.024
LUG_WIDTH = 0.012
EAR_THICKNESS = 0.006
EAR_OFFSET_Y = LUG_WIDTH / 2.0 + EAR_THICKNESS / 2.0
HINGE_BLOCK_HEIGHT = 0.040

BAR_WIDTH = 0.016
BAR_THICKNESS = 0.012
BAR_CENTER_Z = -0.034
BAR_TOP_Z = BAR_CENTER_Z + BAR_THICKNESS / 2.0

WEB_INNER_X = HINGE_BLOCK_LENGTH / 2.0
WEB_OUTER_X = 0.034
WEB_LENGTH = WEB_OUTER_X - WEB_INNER_X
WEB_CENTER_X = (WEB_INNER_X + WEB_OUTER_X) / 2.0
WEB_TOP_Z = -HINGE_BLOCK_HEIGHT / 2.0
WEB_BOTTOM_Z = BAR_TOP_Z
WEB_HEIGHT = WEB_TOP_Z - WEB_BOTTOM_Z
WEB_CENTER_Z = (WEB_TOP_Z + WEB_BOTTOM_Z) / 2.0

INTERMEDIATE_1_LENGTH = 0.170
INTERMEDIATE_2_LENGTH = 0.140
END_LINK_LENGTH = 0.082


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_clevis(part, *, axis_x: float, material, prefix: str) -> None:
    _add_box(
        part,
        name=f"{prefix}_ear_upper",
        size=(HINGE_BLOCK_LENGTH, EAR_THICKNESS, HINGE_BLOCK_HEIGHT),
        center=(axis_x, EAR_OFFSET_Y, 0.0),
        material=material,
    )
    _add_box(
        part,
        name=f"{prefix}_ear_lower",
        size=(HINGE_BLOCK_LENGTH, EAR_THICKNESS, HINGE_BLOCK_HEIGHT),
        center=(axis_x, -EAR_OFFSET_Y, 0.0),
        material=material,
    )


def _add_lug(part, *, axis_x: float, material, prefix: str) -> None:
    _add_box(
        part,
        name=f"{prefix}_lug",
        size=(HINGE_BLOCK_LENGTH, LUG_WIDTH, HINGE_BLOCK_HEIGHT),
        center=(axis_x, 0.0, 0.0),
        material=material,
    )


def _add_forward_web(part, *, axis_x: float, width_y: float, material, prefix: str) -> None:
    _add_box(
        part,
        name=f"{prefix}_forward_web",
        size=(WEB_LENGTH, width_y, WEB_HEIGHT),
        center=(axis_x + WEB_CENTER_X, 0.0, WEB_CENTER_Z),
        material=material,
    )


def _add_rear_web(part, *, axis_x: float, width_y: float, material, prefix: str) -> None:
    _add_box(
        part,
        name=f"{prefix}_rear_web",
        size=(WEB_LENGTH, width_y, WEB_HEIGHT),
        center=(axis_x - WEB_CENTER_X, 0.0, WEB_CENTER_Z),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_hinge_chain")

    base_finish = model.material("base_finish", rgba=(0.20, 0.22, 0.25, 1.0))
    link_finish = model.material("link_finish", rgba=(0.63, 0.66, 0.70, 1.0))
    end_finish = model.material("end_finish", rgba=(0.68, 0.24, 0.16, 1.0))

    base_link = model.part("base_link")
    _add_clevis(base_link, axis_x=0.0, material=base_finish, prefix="joint_1")
    _add_rear_web(base_link, axis_x=0.0, width_y=0.020, material=base_finish, prefix="joint_1")
    _add_box(
        base_link,
        name="base_arm",
        size=(0.086, 0.020, BAR_THICKNESS),
        center=(-(WEB_OUTER_X + 0.086 / 2.0), 0.0, BAR_CENTER_Z),
        material=base_finish,
    )
    _add_box(
        base_link,
        name="base_pedestal",
        size=(0.030, 0.040, 0.062),
        center=(-0.116, 0.0, -0.049),
        material=base_finish,
    )
    _add_box(
        base_link,
        name="base_foot",
        size=(0.136, 0.072, 0.016),
        center=(-0.116, 0.0, -0.088),
        material=base_finish,
    )

    link_1 = model.part("link_1")
    _add_lug(link_1, axis_x=0.0, material=link_finish, prefix="joint_1")
    _add_forward_web(link_1, axis_x=0.0, width_y=BAR_WIDTH, material=link_finish, prefix="joint_1")
    _add_box(
        link_1,
        name="link_1_bar",
        size=(INTERMEDIATE_1_LENGTH - 2.0 * WEB_OUTER_X, BAR_WIDTH, BAR_THICKNESS),
        center=(INTERMEDIATE_1_LENGTH / 2.0, 0.0, BAR_CENTER_Z),
        material=link_finish,
    )
    _add_rear_web(link_1, axis_x=INTERMEDIATE_1_LENGTH, width_y=BAR_WIDTH, material=link_finish, prefix="joint_2")
    _add_clevis(link_1, axis_x=INTERMEDIATE_1_LENGTH, material=link_finish, prefix="joint_2")

    link_2 = model.part("link_2")
    _add_lug(link_2, axis_x=0.0, material=link_finish, prefix="joint_2")
    _add_forward_web(link_2, axis_x=0.0, width_y=BAR_WIDTH, material=link_finish, prefix="joint_2")
    _add_box(
        link_2,
        name="link_2_bar",
        size=(INTERMEDIATE_2_LENGTH - 2.0 * WEB_OUTER_X, BAR_WIDTH, BAR_THICKNESS),
        center=(INTERMEDIATE_2_LENGTH / 2.0, 0.0, BAR_CENTER_Z),
        material=link_finish,
    )
    _add_rear_web(link_2, axis_x=INTERMEDIATE_2_LENGTH, width_y=BAR_WIDTH, material=link_finish, prefix="joint_3")
    _add_clevis(link_2, axis_x=INTERMEDIATE_2_LENGTH, material=link_finish, prefix="joint_3")

    end_link = model.part("end_link")
    _add_lug(end_link, axis_x=0.0, material=end_finish, prefix="joint_3")
    _add_forward_web(end_link, axis_x=0.0, width_y=BAR_WIDTH, material=end_finish, prefix="joint_3")
    _add_box(
        end_link,
        name="end_bar",
        size=(END_LINK_LENGTH - WEB_OUTER_X, BAR_WIDTH, BAR_THICKNESS),
        center=(WEB_OUTER_X + (END_LINK_LENGTH - WEB_OUTER_X) / 2.0, 0.0, BAR_CENTER_Z),
        material=end_finish,
    )
    _add_box(
        end_link,
        name="end_pad",
        size=(0.028, BAR_WIDTH, 0.018),
        center=(END_LINK_LENGTH + 0.014, 0.0, BAR_CENTER_Z),
        material=end_finish,
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base_link,
        child=link_1,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-1.0, upper=0.75),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(INTERMEDIATE_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-1.45, upper=0.75),
    )
    model.articulation(
        "link_2_to_end_link",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_link,
        origin=Origin(xyz=(INTERMEDIATE_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0, lower=-1.45, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_link = object_model.get_part("base_link")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_link = object_model.get_part("end_link")

    joint_1 = object_model.get_articulation("base_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_end_link")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    axis_checks = []
    for articulation in (joint_1, joint_2, joint_3):
        axis_checks.append(
            all(isclose(component, target, abs_tol=1e-9) for component, target in zip(articulation.axis, (0.0, 1.0, 0.0)))
        )
    ctx.check(
        "parallel_joint_axes",
        all(axis_checks),
        details="All three revolute joints should rotate about the same world-parallel Y axis.",
    )
    ctx.check(
        "joint_origins_coplanar",
        isclose(joint_1.origin.xyz[1], 0.0, abs_tol=1e-9)
        and isclose(joint_2.origin.xyz[1], 0.0, abs_tol=1e-9)
        and isclose(joint_3.origin.xyz[1], 0.0, abs_tol=1e-9)
        and joint_2.origin.xyz[0] > 0.0
        and joint_3.origin.xyz[0] > 0.0,
        details="Joint origins should march forward in the same XZ plane.",
    )

    base_aabb = ctx.part_world_aabb(base_link)
    link_1_aabb = ctx.part_world_aabb(link_1)
    link_2_aabb = ctx.part_world_aabb(link_2)
    end_aabb = ctx.part_world_aabb(end_link)
    if base_aabb and link_1_aabb and link_2_aabb and end_aabb:
        base_length = base_aabb[1][0] - base_aabb[0][0]
        link_1_length = link_1_aabb[1][0] - link_1_aabb[0][0]
        link_2_length = link_2_aabb[1][0] - link_2_aabb[0][0]
        end_length = end_aabb[1][0] - end_aabb[0][0]
        ctx.check(
            "serial_lengths_read_correctly",
            base_length > 0.10 and link_1_length > link_2_length > end_length > 0.05,
            details=(
                f"Expected descending moving-link lengths with a grounded base bracket; "
                f"got base={base_length:.3f}, link_1={link_1_length:.3f}, "
                f"link_2={link_2_length:.3f}, end={end_length:.3f}."
            ),
        )

    ctx.expect_contact(base_link, link_1, name="joint_1_contact_rest")
    ctx.expect_contact(link_1, link_2, name="joint_2_contact_rest")
    ctx.expect_contact(link_2, end_link, name="joint_3_contact_rest")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
    for joint, label, link_a, link_b in (
        (joint_1, "joint_1", base_link, link_1),
        (joint_2, "joint_2", link_1, link_2),
        (joint_3, "joint_3", link_2, end_link),
    ):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue

        lower_pose = {joint_1: 0.0, joint_2: 0.0, joint_3: 0.0, joint: limits.lower}
        upper_pose = {joint_1: 0.0, joint_2: 0.0, joint_3: 0.0, joint: limits.upper}

        with ctx.pose(lower_pose):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_lower_no_floating")
            ctx.expect_contact(link_a, link_b, name=f"{label}_lower_contact")

        with ctx.pose(upper_pose):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_upper_no_floating")
            ctx.expect_contact(link_a, link_b, name=f"{label}_upper_contact")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
