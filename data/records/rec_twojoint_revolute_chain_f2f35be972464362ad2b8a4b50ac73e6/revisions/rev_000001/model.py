from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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


PRIMARY_JOINT_OFFSET_X = 0.180

SHOULDER_RADIUS = 0.016
ELBOW_RADIUS = 0.014

BRACKET_BOSS_LENGTH = 0.018
PRIMARY_THICKNESS = 0.014
ELBOW_BOSS_LENGTH = 0.012
SECONDARY_THICKNESS = 0.010

PRIMARY_Y = BRACKET_BOSS_LENGTH + PRIMARY_THICKNESS / 2.0
ELBOW_BOSS_CENTER_Y = PRIMARY_Y + PRIMARY_THICKNESS / 2.0 + ELBOW_BOSS_LENGTH / 2.0
SECONDARY_Y = PRIMARY_Y + PRIMARY_THICKNESS / 2.0 + ELBOW_BOSS_LENGTH + SECONDARY_THICKNESS / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_link_hinged_arm")

    bracket_mat = model.material("bracket_steel", rgba=(0.27, 0.29, 0.32, 1.0))
    primary_mat = model.material("primary_link_paint", rgba=(0.77, 0.30, 0.16, 1.0))
    secondary_mat = model.material("secondary_link_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    pad_mat = model.material("end_pad_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((0.012, 0.080, 0.090)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0)),
        material=bracket_mat,
        name="back_plate",
    )
    base_bracket.visual(
        Box((0.076, 0.016, 0.020)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
        material=bracket_mat,
        name="reach_arm",
    )
    base_bracket.visual(
        Cylinder(radius=SHOULDER_RADIUS, length=BRACKET_BOSS_LENGTH),
        origin=Origin(xyz=(0.0, BRACKET_BOSS_LENGTH / 2.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bracket_mat,
        name="shoulder_boss",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.110, 0.080, 0.090)),
        mass=1.1,
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        Cylinder(radius=SHOULDER_RADIUS, length=PRIMARY_THICKNESS),
        origin=Origin(xyz=(0.0, PRIMARY_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=primary_mat,
        name="shoulder_hub",
    )
    primary_link.visual(
        Box((0.170, PRIMARY_THICKNESS, 0.018)),
        origin=Origin(xyz=(0.095, PRIMARY_Y, 0.0)),
        material=primary_mat,
        name="primary_beam",
    )
    primary_link.visual(
        Cylinder(radius=ELBOW_RADIUS, length=ELBOW_BOSS_LENGTH),
        origin=Origin(xyz=(PRIMARY_JOINT_OFFSET_X, ELBOW_BOSS_CENTER_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=primary_mat,
        name="elbow_boss",
    )
    primary_link.inertial = Inertial.from_geometry(
        Box((0.198, 0.026, 0.030)),
        mass=0.55,
        origin=Origin(xyz=(0.099, 0.031, 0.0)),
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        Cylinder(radius=ELBOW_RADIUS, length=SECONDARY_THICKNESS),
        origin=Origin(xyz=(0.0, SECONDARY_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=secondary_mat,
        name="elbow_hub",
    )
    secondary_link.visual(
        Box((0.122, SECONDARY_THICKNESS, 0.016)),
        origin=Origin(xyz=(0.071, SECONDARY_Y, 0.0)),
        material=secondary_mat,
        name="secondary_beam",
    )
    secondary_link.visual(
        Box((0.026, SECONDARY_THICKNESS, 0.018)),
        origin=Origin(xyz=(0.145, SECONDARY_Y, 0.0)),
        material=secondary_mat,
        name="tip_block",
    )
    secondary_link.visual(
        Box((0.028, 0.018, 0.008)),
        origin=Origin(xyz=(0.145, SECONDARY_Y, -0.013)),
        material=pad_mat,
        name="end_pad",
    )
    secondary_link.inertial = Inertial.from_geometry(
        Box((0.160, 0.030, 0.030)),
        mass=0.34,
        origin=Origin(xyz=(0.078, 0.042, -0.002)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=primary_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-1.0, upper=1.0),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(PRIMARY_JOINT_OFFSET_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=-0.15, upper=1.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.fail_if_isolated_parts(max_pose_samples=16, name="arm_pose_sweep_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="arm_pose_sweep_no_articulation_overlap")

    ctx.check(
        "revolute_joint_types",
        shoulder_joint.joint_type == ArticulationType.REVOLUTE and elbow_joint.joint_type == ArticulationType.REVOLUTE,
        details=f"joint types were {shoulder_joint.joint_type!r} and {elbow_joint.joint_type!r}",
    )
    ctx.check(
        "parallel_pin_axes",
        shoulder_joint.axis == elbow_joint.axis == (0.0, 1.0, 0.0),
        details=f"axes were {shoulder_joint.axis!r} and {elbow_joint.axis!r}",
    )

    ctx.expect_contact(base_bracket, primary_link, elem_a="shoulder_boss", elem_b="shoulder_hub", name="shoulder_joint_contact")
    ctx.expect_contact(primary_link, secondary_link, elem_a="elbow_boss", elem_b="elbow_hub", name="elbow_joint_contact")
    ctx.expect_gap(primary_link, base_bracket, axis="y", min_gap=0.009, max_gap=0.020, positive_elem="primary_beam", negative_elem="reach_arm", name="primary_beam_clear_of_bracket_arm")
    ctx.expect_gap(secondary_link, primary_link, axis="y", min_gap=0.009, max_gap=0.020, positive_elem="secondary_beam", negative_elem="primary_beam", name="secondary_beam_clear_of_primary_beam")
    ctx.expect_overlap(base_bracket, primary_link, axes="xz", min_overlap=0.028, elem_a="shoulder_boss", elem_b="shoulder_hub", name="shoulder_axis_alignment")
    ctx.expect_overlap(primary_link, secondary_link, axes="xz", min_overlap=0.024, elem_a="elbow_boss", elem_b="elbow_hub", name="elbow_axis_alignment")
    ctx.expect_origin_distance(
        primary_link,
        secondary_link,
        axes="x",
        min_dist=0.175,
        max_dist=0.185,
        name="elbow_origin_offset_matches_primary_length",
    )

    secondary_beam_aabb = ctx.part_element_world_aabb(secondary_link, elem="secondary_beam")
    end_pad_aabb = ctx.part_element_world_aabb(secondary_link, elem="end_pad")
    if secondary_beam_aabb is None or end_pad_aabb is None:
        ctx.fail("secondary_pad_bounds_available", "Could not resolve secondary_beam or end_pad bounds.")
    else:
        beam_min, beam_max = secondary_beam_aabb
        pad_min, pad_max = end_pad_aabb
        ctx.check(
            "end_pad_is_distal",
            pad_min[0] >= beam_max[0] - 0.010,
            details=f"pad_min_x={pad_min[0]:.4f}, beam_max_x={beam_max[0]:.4f}",
        )
        ctx.check(
            "end_pad_is_below_tip",
            pad_max[2] <= beam_min[2] + 0.001,
            details=f"pad_max_z={pad_max[2]:.4f}, beam_min_z={beam_min[2]:.4f}",
        )

    for joint, prefix, elem_a, elem_b in (
        (shoulder_joint, "shoulder_joint", "shoulder_boss", "shoulder_hub"),
        (elbow_joint, "elbow_joint", "elbow_boss", "elbow_hub"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_lower_no_floating")
                if joint is shoulder_joint:
                    ctx.expect_contact(base_bracket, primary_link, elem_a=elem_a, elem_b=elem_b, name=f"{prefix}_lower_contact")
                else:
                    ctx.expect_contact(primary_link, secondary_link, elem_a=elem_a, elem_b=elem_b, name=f"{prefix}_lower_contact")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_upper_no_floating")
                if joint is shoulder_joint:
                    ctx.expect_contact(base_bracket, primary_link, elem_a=elem_a, elem_b=elem_b, name=f"{prefix}_upper_contact")
                else:
                    ctx.expect_contact(primary_link, secondary_link, elem_a=elem_a, elem_b=elem_b, name=f"{prefix}_upper_contact")

    shoulder_limits = shoulder_joint.motion_limits
    elbow_limits = elbow_joint.motion_limits
    ctx.check(
        "joint_limit_ordering",
        (
            shoulder_limits is not None
            and elbow_limits is not None
            and shoulder_limits.lower is not None
            and shoulder_limits.upper is not None
            and elbow_limits.lower is not None
            and elbow_limits.upper is not None
            and shoulder_limits.lower < shoulder_limits.upper
            and elbow_limits.lower < elbow_limits.upper
            and isclose(shoulder_joint.origin.xyz[0], 0.0, abs_tol=1e-9)
            and isclose(elbow_joint.origin.xyz[0], PRIMARY_JOINT_OFFSET_X, abs_tol=1e-9)
        ),
        details="Joint limits were missing, unordered, or joint origins were misplaced.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
