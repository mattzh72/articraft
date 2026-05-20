from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_style_rotary_tree")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    branch_blue = model.material("branch_blue", rgba=(0.10, 0.22, 0.48, 1.0))
    branch_green = model.material("branch_green", rgba=(0.12, 0.38, 0.22, 1.0))
    branch_orange = model.material("branch_orange", rgba=(0.72, 0.32, 0.08, 1.0))
    hard_face = model.material("hard_face", rgba=(0.86, 0.78, 0.42, 1.0))
    black_cap = model.material("black_cap", rgba=(0.02, 0.02, 0.018, 1.0))

    spine = model.part("spine")
    spine.visual(
        Box((0.46, 0.34, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="floor_plate",
    )
    spine.visual(
        Box((0.08, 0.08, 0.93)),
        origin=Origin(xyz=(0.0, 0.0, 0.485)),
        material=dark_steel,
        name="upright_post",
    )

    hub_x = 0.19
    saddle_levels = (0.29, 0.52, 0.75)
    for idx, z in enumerate(saddle_levels):
        spine.visual(
            Box((0.085, 0.20, 0.080)),
            origin=Origin(xyz=(0.0775, 0.0, z)),
            material=dark_steel,
            name=f"saddle_{idx}_backplate",
        )
        for side_idx, y in enumerate((-0.085, 0.085)):
            spine.visual(
                Box((0.17, 0.035, 0.11)),
                origin=Origin(xyz=(0.165, y, z)),
                material=galvanized,
                name=f"saddle_{idx}_ear_{side_idx}",
            )
            spine.visual(
                Cylinder(radius=0.046, length=0.020),
                origin=Origin(
                    xyz=(hub_x, -0.1115 if y < 0.0 else 0.1115, z),
                    rpy=(-pi / 2.0, 0.0, 0.0),
                ),
                material=black_cap,
                name=f"saddle_{idx}_axle_cap_{side_idx}",
            )

    def add_hub_and_arm(part, branch_material, *, terminal: str) -> None:
        part.visual(
            Cylinder(radius=0.052, length=0.135),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name="hub",
        )
        part.visual(
            Box((0.37, 0.043, 0.043)),
            origin=Origin(xyz=(0.215, 0.0, 0.0)),
            material=branch_material,
            name="arm",
        )
        if terminal == "pad":
            part.visual(
                Box((0.065, 0.160, 0.105)),
                origin=Origin(xyz=(0.425, 0.0, 0.0)),
                material=hard_face,
                name="pad_face",
            )
            part.visual(
                Box((0.030, 0.090, 0.070)),
                origin=Origin(xyz=(0.380, 0.0, 0.0)),
                material=branch_material,
                name="pad_neck",
            )
        elif terminal == "fork":
            part.visual(
                Box((0.055, 0.120, 0.055)),
                origin=Origin(xyz=(0.385, 0.0, 0.0)),
                material=branch_material,
                name="fork_bridge",
            )
            for tine_idx, y in enumerate((-0.046, 0.046)):
                part.visual(
                    Box((0.120, 0.030, 0.045)),
                    origin=Origin(xyz=(0.455, y, 0.0)),
                    material=hard_face,
                    name=f"fork_tine_{tine_idx}",
                )
        elif terminal == "clamp_tab":
            part.visual(
                Box((0.035, 0.145, 0.090)),
                origin=Origin(xyz=(0.4125, 0.0, 0.0)),
                material=hard_face,
                name="clamp_tab",
            )
            part.visual(
                Box((0.070, 0.105, 0.024)),
                origin=Origin(xyz=(0.405, 0.0, 0.057)),
                material=branch_material,
                name="tab_lip",
            )
            part.visual(
                Cylinder(radius=0.022, length=0.018),
                origin=Origin(xyz=(0.436, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
                material=black_cap,
                name="tab_boss",
            )

    pad_branch = model.part("pad_branch")
    add_hub_and_arm(pad_branch, branch_blue, terminal="pad")

    fork_branch = model.part("fork_branch")
    add_hub_and_arm(fork_branch, branch_green, terminal="fork")

    tab_branch = model.part("tab_branch")
    add_hub_and_arm(tab_branch, branch_orange, terminal="clamp_tab")

    branch_parts = (pad_branch, fork_branch, tab_branch)
    joint_names = ("spine_to_pad_branch", "spine_to_fork_branch", "spine_to_tab_branch")
    for branch, joint_name, z in zip(branch_parts, joint_names, saddle_levels):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=spine,
            child=branch,
            origin=Origin(xyz=(hub_x, 0.0, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.45, upper=0.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    branch_names = ("pad_branch", "fork_branch", "tab_branch")
    joint_names = ("spine_to_pad_branch", "spine_to_fork_branch", "spine_to_tab_branch")

    ctx.check(
        "three independent revolute branches",
        len(object_model.articulations) == 3
        and all(object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE for name in joint_names),
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )

    for idx, (branch_name, joint_name) in enumerate(zip(branch_names, joint_names)):
        branch = object_model.get_part(branch_name)
        joint = object_model.get_articulation(joint_name)
        ctx.expect_within(
            branch,
            spine,
            axes="xz",
            inner_elem="hub",
            outer_elem=f"saddle_{idx}_ear_0",
            margin=0.002,
            name=f"{branch_name} hub aligns with saddle ear",
        )
        ctx.expect_gap(
            branch,
            spine,
            axis="y",
            positive_elem="hub",
            negative_elem=f"saddle_{idx}_ear_0",
            min_gap=0.0,
            max_gap=0.001,
            name=f"{branch_name} clears negative saddle ear",
        )
        ctx.expect_gap(
            spine,
            branch,
            axis="y",
            positive_elem=f"saddle_{idx}_ear_1",
            negative_elem="hub",
            min_gap=0.0,
            max_gap=0.001,
            name=f"{branch_name} clears positive saddle ear",
        )

        rest_aabb = ctx.part_element_world_aabb(branch, elem="arm")
        with ctx.pose({joint: 0.35}):
            raised_aabb = ctx.part_element_world_aabb(branch, elem="arm")
        ctx.check(
            f"{branch_name} positive rotation raises branch",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.04,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
