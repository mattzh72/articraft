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
    model = ArticulatedObject(name="bridge_backed_tooling_tree")

    steel = model.material("powder_coated_steel", color=(0.13, 0.15, 0.16, 1.0))
    dark = model.material("dark_bearing_black", color=(0.02, 0.022, 0.024, 1.0))
    blue = model.material("anodized_branch_blue", color=(0.10, 0.30, 0.78, 1.0))
    orange = model.material("tooling_orange", color=(0.95, 0.42, 0.08, 1.0))
    silver = model.material("brushed_tooling_steel", color=(0.65, 0.67, 0.68, 1.0))

    support = model.part("support")
    support.visual(Box((1.05, 0.82, 0.04)), origin=Origin(xyz=(0.05, 0.0, 0.02)), material=steel, name="base_plate")
    support.visual(Box((0.08, 0.07, 0.62)), origin=Origin(xyz=(-0.43, -0.36, 0.33)), material=steel, name="rear_post_0")
    support.visual(Box((0.08, 0.07, 0.62)), origin=Origin(xyz=(-0.43, 0.36, 0.33)), material=steel, name="rear_post_1")
    support.visual(Box((0.12, 0.79, 0.08)), origin=Origin(xyz=(-0.43, 0.0, 0.66)), material=steel, name="rear_bridge")
    support.visual(Box((0.11, 0.18, 0.54)), origin=Origin(xyz=(-0.43, 0.0, 0.31)), material=steel, name="rear_web")
    support.visual(Box((0.82, 0.09, 0.08)), origin=Origin(xyz=(-0.02, 0.0, 0.54)), material=steel, name="forward_spine")
    support.visual(Box((0.07, 0.09, 0.50)), origin=Origin(xyz=(0.38, 0.0, 0.29)), material=steel, name="front_stanchion")

    branch_specs = [
        ("branch_0", -0.18, -0.09, 0.642, -1.05),
        ("branch_1", 0.04, 0.09, 0.735, 0.98),
        ("branch_2", 0.29, -0.09, 0.865, -0.72),
    ]

    for index, (_name, x, y, joint_z, _yaw) in enumerate(branch_specs):
        pedestal_top = joint_z - 0.038
        pedestal_height = pedestal_top - 0.565
        support.visual(
            Box((0.085, 0.12, pedestal_height)),
            origin=Origin(xyz=(x, y, 0.565 + pedestal_height / 2.0)),
            material=steel,
            name=f"riser_{index}",
        )
        support.visual(
            Cylinder(radius=0.058, length=0.018),
            origin=Origin(xyz=(x, y, joint_z - 0.029)),
            material=dark,
            name=f"bearing_{index}",
        )

    for index, (name, x, y, joint_z, yaw) in enumerate(branch_specs):
        branch = model.part(name)
        branch.visual(
            Cylinder(radius=0.047, length=0.040),
            origin=Origin(),
            material=dark,
            name="hub",
        )
        branch.visual(
            Cylinder(radius=0.062, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.026)),
            material=dark,
            name="top_washer",
        )
        branch.visual(
            Box((0.36, 0.044, 0.034)),
            origin=Origin(xyz=(0.205, 0.0, 0.0)),
            material=blue,
            name="arm_beam",
        )
        branch.visual(
            Box((0.25, 0.030, 0.028)),
            origin=Origin(xyz=(0.200, 0.0, -0.026)),
            material=blue,
            name="lower_gusset",
        )
        branch.visual(
            Box((0.105, 0.095, 0.060)),
            origin=Origin(xyz=(0.405, 0.0, 0.0)),
            material=orange,
            name="tool_plate",
        )
        branch.visual(
            Cylinder(radius=0.018, length=0.115),
            origin=Origin(xyz=(0.405, 0.0, -0.070)),
            material=silver,
            name="tool_pin",
        )
        branch.visual(
            Cylinder(radius=0.017, length=0.125),
            origin=Origin(xyz=(0.405, 0.0, 0.040), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name="cross_pin",
        )

        model.articulation(
            f"support_to_branch_{index}",
            ArticulationType.REVOLUTE,
            parent=support,
            child=branch,
            origin=Origin(xyz=(x, y, joint_z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.45, upper=0.45),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    branches = [object_model.get_part(f"branch_{i}") for i in range(3)]
    joints = [object_model.get_articulation(f"support_to_branch_{i}") for i in range(3)]

    ctx.check(
        "three independent revolute branches",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )

    for index, (branch, joint) in enumerate(zip(branches, joints)):
        limits = joint.motion_limits
        ctx.check(
            f"branch_{index} has bounded swing",
            limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < 0.0 < limits.upper,
            details=f"limits={limits}",
        )
        ctx.expect_contact(
            branch,
            support,
            elem_a="hub",
            elem_b=f"bearing_{index}",
            contact_tol=0.002,
            name=f"branch_{index} bearing clearance",
        )

    ctx.expect_gap(
        branches[1],
        branches[0],
        axis="y",
        positive_elem="tool_plate",
        negative_elem="tool_plate",
        min_gap=0.50,
        name="opposed tooling plates clear in y",
    )
    ctx.expect_gap(
        branches[1],
        branches[2],
        axis="y",
        positive_elem="tool_plate",
        negative_elem="tool_plate",
        min_gap=0.45,
        name="side tooling plates clear in y",
    )
    ctx.expect_gap(branches[2], branches[0], axis="x", min_gap=0.07, name="same-side branches are staggered in x")

    def elem_center(part, elem_name):
        bounds = ctx.part_element_world_aabb(part, elem=elem_name)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    rest_0 = elem_center(branches[0], "tool_plate")
    rest_1 = elem_center(branches[1], "tool_plate")
    with ctx.pose({joints[0]: 0.35}):
        moved_0 = elem_center(branches[0], "tool_plate")
        held_1 = elem_center(branches[1], "tool_plate")
    ctx.check(
        "branch_0 swings independently",
        rest_0 is not None
        and moved_0 is not None
        and held_1 is not None
        and rest_1 is not None
        and abs(moved_0[1] - rest_0[1]) > 0.05
        and abs(held_1[1] - rest_1[1]) < 0.002,
        details=f"rest_0={rest_0}, moved_0={moved_0}, rest_1={rest_1}, held_1={held_1}",
    )

    return ctx.report()


object_model = build_object_model()
