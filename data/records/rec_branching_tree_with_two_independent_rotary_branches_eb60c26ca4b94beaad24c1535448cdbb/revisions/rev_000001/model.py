from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_two_branch_rotary_bracket")

    model.material("powder_coated_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("dark_hardware", rgba=(0.025, 0.025, 0.025, 1.0))
    model.material("brushed_edge", rgba=(0.45, 0.47, 0.48, 1.0))
    model.material("branch_metal", rgba=(0.08, 0.18, 0.30, 1.0))
    model.material("bronze_bushing", rgba=(0.66, 0.48, 0.24, 1.0))

    spine = model.part("spine")

    # Intrinsic frame: X is horizontal across the wall, Y projects out from the
    # wall, and Z is vertical.  The wall plate back face is at y=0.
    spine.visual(
        Box((0.160, 0.012, 0.360)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material="powder_coated_steel",
        name="backplate",
    )
    spine.visual(
        Box((0.052, 0.090, 0.305)),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material="powder_coated_steel",
        name="standoff_spine",
    )
    spine.visual(
        Box((0.070, 0.010, 0.325)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material="brushed_edge",
        name="raised_back_rib",
    )

    # Four shallow screw heads make the wall-backed mounting plate legible.
    for sx in (-0.054, 0.054):
        for sz in (-0.135, 0.135):
            screw_name = f"screw_{'pos' if sx > 0 else 'neg'}_{'top' if sz > 0 else 'bottom'}"
            spine.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(sx, 0.014, sz), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material="dark_hardware",
                name=screw_name,
            )

    # Separate knuckle support stations.  Each branch has a top and bottom
    # supported bearing with a clear central gap for its moving hub.
    knuckles = (
        ("upper", 0.055, 0.112, 0.082),
        ("lower", -0.055, 0.112, -0.082),
    )
    for label, px, py, pz in knuckles:
        sign = 1.0 if px > 0 else -1.0
        bridge_center_x = px / 2.0
        bridge_len_x = abs(px) + 0.052
        for side, zoff in (("top", 0.031), ("bottom", -0.031)):
            spine.visual(
                Box((bridge_len_x, 0.034, 0.010)),
                origin=Origin(xyz=(bridge_center_x, py, pz + zoff)),
                material="powder_coated_steel",
                name=f"{label}_{side}_bridge",
            )
            spine.visual(
                Cylinder(radius=0.029, length=0.010),
                origin=Origin(xyz=(px, py, pz + zoff)),
                material="bronze_bushing",
                name=f"{label}_{side}_bearing",
            )
        # A small side web ties each pair of bridges back into the standoff
        # without entering the moving hub's clearance envelope.
        spine.visual(
            Box((0.010, 0.032, 0.070)),
            origin=Origin(xyz=(sign * 0.030, py, pz)),
            material="powder_coated_steel",
            name=f"{label}_side_web",
        )

    upper_branch = model.part("upper_branch")
    upper_branch.visual(
        Cylinder(radius=0.021, length=0.044),
        origin=Origin(),
        material="branch_metal",
        name="pivot_hub",
    )
    upper_branch.visual(
        Box((0.142, 0.027, 0.024)),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material="branch_metal",
        name="branch_arm",
    )
    upper_branch.visual(
        Cylinder(radius=0.019, length=0.026),
        origin=Origin(xyz=(0.162, 0.0, 0.0)),
        material="branch_metal",
        name="end_boss",
    )
    upper_branch.visual(
        Box((0.100, 0.008, 0.030)),
        origin=Origin(xyz=(0.085, 0.0, 0.019)),
        material="brushed_edge",
        name="top_reinforcing_rib",
    )

    lower_branch = model.part("lower_branch")
    lower_branch.visual(
        Cylinder(radius=0.021, length=0.044),
        origin=Origin(),
        material="branch_metal",
        name="pivot_hub",
    )
    lower_branch.visual(
        Box((0.142, 0.027, 0.024)),
        origin=Origin(xyz=(-0.086, 0.0, 0.0)),
        material="branch_metal",
        name="branch_arm",
    )
    lower_branch.visual(
        Cylinder(radius=0.019, length=0.026),
        origin=Origin(xyz=(-0.162, 0.0, 0.0)),
        material="branch_metal",
        name="end_boss",
    )
    lower_branch.visual(
        Box((0.100, 0.008, 0.030)),
        origin=Origin(xyz=(-0.085, 0.0, 0.019)),
        material="brushed_edge",
        name="top_reinforcing_rib",
    )

    model.articulation(
        "spine_to_upper_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_branch,
        origin=Origin(xyz=(0.055, 0.112, 0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "spine_to_lower_branch",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_branch,
        origin=Origin(xyz=(-0.055, 0.112, -0.082)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.15, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_branch")
    lower = object_model.get_part("lower_branch")
    spine = object_model.get_part("spine")
    upper_joint = object_model.get_articulation("spine_to_upper_branch")
    lower_joint = object_model.get_articulation("spine_to_lower_branch")

    joints = (upper_joint, lower_joint)
    ctx.check(
        "two separate rotary branch joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and {j.child for j in joints} == {"upper_branch", "lower_branch"},
        details=f"joints={[(j.name, j.articulation_type, j.child) for j in joints]}",
    )
    ctx.check(
        "branch axes are parallel vertical",
        all(tuple(j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )
    ctx.expect_origin_distance(
        upper,
        lower,
        axes="xz",
        min_dist=0.18,
        name="knuckle stations are visibly offset",
    )

    for label, branch in (("upper", upper), ("lower", lower)):
        ctx.expect_overlap(
            spine,
            branch,
            axes="xy",
            elem_a=f"{label}_top_bearing",
            elem_b="pivot_hub",
            min_overlap=0.030,
            name=f"{label} hub sits below supported top bearing",
        )
        ctx.expect_gap(
            spine,
            branch,
            axis="z",
            positive_elem=f"{label}_top_bearing",
            negative_elem="pivot_hub",
            min_gap=0.001,
            max_gap=0.006,
            name=f"{label} hub has top running clearance",
        )
        ctx.expect_gap(
            branch,
            spine,
            axis="z",
            positive_elem="pivot_hub",
            negative_elem=f"{label}_bottom_bearing",
            min_gap=0.001,
            max_gap=0.006,
            name=f"{label} hub has bottom running clearance",
        )

    upper_end_rest = ctx.part_element_world_aabb(upper, elem="end_boss")
    lower_end_rest = ctx.part_element_world_aabb(lower, elem="end_boss")
    with ctx.pose({upper_joint: 0.80, lower_joint: -0.80}):
        upper_end_swept = ctx.part_element_world_aabb(upper, elem="end_boss")
        lower_end_swept = ctx.part_element_world_aabb(lower, elem="end_boss")

    def aabb_center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5)

    upper_rest_xy = aabb_center_xy(upper_end_rest)
    upper_swept_xy = aabb_center_xy(upper_end_swept)
    lower_rest_xy = aabb_center_xy(lower_end_rest)
    lower_swept_xy = aabb_center_xy(lower_end_swept)
    ctx.check(
        "upper branch end sweeps around its knuckle",
        upper_rest_xy is not None
        and upper_swept_xy is not None
        and abs(upper_swept_xy[1] - upper_rest_xy[1]) > 0.060,
        details=f"rest={upper_rest_xy}, swept={upper_swept_xy}",
    )
    ctx.check(
        "lower branch end sweeps around its knuckle",
        lower_rest_xy is not None
        and lower_swept_xy is not None
        and abs(lower_swept_xy[1] - lower_rest_xy[1]) > 0.060,
        details=f"rest={lower_rest_xy}, swept={lower_swept_xy}",
    )

    return ctx.report()


object_model = build_object_model()
