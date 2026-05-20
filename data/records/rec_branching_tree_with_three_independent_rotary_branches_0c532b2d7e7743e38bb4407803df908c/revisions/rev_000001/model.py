from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


BRANCH_SPECS = (
    (0.0, 0.42, "oxide_red"),
    (2.0 * math.pi / 3.0, 0.70, "safety_yellow"),
    (4.0 * math.pi / 3.0, 0.98, "tool_blue"),
)

JOINT_RADIUS = 0.145
ARM_LENGTH = 0.280
ARM_WIDTH = 0.032
HUB_HEIGHT = 0.090
HUB_WIDTH = 0.136


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_motion_rig")

    dark_steel = model.material("dark_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.55, 0.57, 0.55, 1.0))
    black_bearing = model.material("black_bearing", rgba=(0.015, 0.017, 0.018, 1.0))
    oxide_red = model.material("oxide_red", rgba=(0.72, 0.12, 0.08, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.06, 1.0))
    tool_blue = model.material("tool_blue", rgba=(0.06, 0.23, 0.70, 1.0))

    arm_materials: dict[str, Material] = {
        "oxide_red": oxide_red,
        "safety_yellow": safety_yellow,
        "tool_blue": tool_blue,
    }

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.24, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.036, length=1.120),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=brushed_steel,
        name="mast_column",
    )
    mast.visual(
        Sphere(radius=0.042),
        origin=Origin(xyz=(0.0, 0.0, 1.170)),
        material=brushed_steel,
        name="mast_top_cap",
    )

    def branch_world(theta: float, x: float, y: float, z: float) -> tuple[float, float, float]:
        radial = (math.cos(theta), math.sin(theta))
        tangent = (-math.sin(theta), math.cos(theta))
        joint_x = JOINT_RADIUS * radial[0]
        joint_y = JOINT_RADIUS * radial[1]
        return (
            joint_x + x * radial[0] + y * tangent[0],
            joint_y + x * radial[1] + y * tangent[1],
            z,
        )

    for index, (theta, height, material_name) in enumerate(BRANCH_SPECS):
        # The pad overlaps the mast and forms the rear bridge of an open yoke.
        mast.visual(
            Box((0.112, HUB_WIDTH, 0.076)),
            origin=Origin(
                xyz=branch_world(theta, -0.088, 0.0, height),
                rpy=(0.0, 0.0, theta),
            ),
            material=black_bearing,
            name=f"hub_pad_{index}",
        )
        # Two side cheeks leave a clear slot for the moving barrel while making
        # the hub block visibly bulkier than the spoke arm.
        for sign, suffix in ((1.0, "upper"), (-1.0, "lower")):
            mast.visual(
                Box((0.110, 0.030, HUB_HEIGHT)),
                origin=Origin(
                    xyz=branch_world(theta, 0.017, sign * 0.053, height),
                    rpy=(0.0, 0.0, theta),
                ),
                material=black_bearing,
                name=f"hub_cheek_{index}_{suffix}",
            )
            mast.visual(
                Cylinder(radius=0.028, length=0.018),
                origin=Origin(
                    xyz=branch_world(theta, 0.0, sign * 0.075, height),
                    rpy=(math.pi / 2.0, 0.0, theta),
                ),
                material=brushed_steel,
                name=f"hub_bearing_{index}_{suffix}",
            )
        mast.visual(
            Cylinder(radius=0.010, length=0.162),
            origin=Origin(
                xyz=branch_world(theta, 0.0, 0.0, height),
                rpy=(math.pi / 2.0, 0.0, theta),
            ),
            material=brushed_steel,
            name=f"hub_pin_{index}",
        )

        spoke = model.part(f"spoke_{index}")
        spoke.visual(
            Cylinder(radius=0.025, length=0.060),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name="pivot_barrel",
        )
        spoke.visual(
            Box((ARM_LENGTH, ARM_WIDTH, ARM_WIDTH)),
            origin=Origin(xyz=(0.153, 0.0, 0.0)),
            material=arm_materials[material_name],
            name="spoke_bar",
        )
        spoke.visual(
            Sphere(radius=0.025),
            origin=Origin(xyz=(0.305, 0.0, 0.0)),
            material=arm_materials[material_name],
            name="end_cap",
        )
        model.articulation(
            f"hub_axis_{index}",
            ArticulationType.REVOLUTE,
            parent=mast,
            child=spoke,
            origin=Origin(
                xyz=branch_world(theta, 0.0, 0.0, height),
                rpy=(0.0, 0.0, theta),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.80, upper=0.80),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    joints = [object_model.get_articulation(f"hub_axis_{i}") for i in range(3)]
    spokes = [object_model.get_part(f"spoke_{i}") for i in range(3)]

    ctx.check(
        "three independent revolute hub axes",
        len(joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and len({j.child for j in joints}) == 3,
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "hub blocks are visibly larger than spoke arms",
        HUB_HEIGHT > ARM_WIDTH * 2.0 and HUB_WIDTH > ARM_WIDTH * 3.0,
        details=f"hub_height={HUB_HEIGHT}, hub_width={HUB_WIDTH}, arm_width={ARM_WIDTH}",
    )

    for index, spoke in enumerate(spokes):
        ctx.allow_overlap(
            mast,
            spoke,
            elem_a=f"hub_pin_{index}",
            elem_b="pivot_barrel",
            reason="A fixed steel hub pin intentionally passes through the rotating spoke barrel.",
        )
        ctx.expect_overlap(
            mast,
            spoke,
            axes="xyz",
            elem_a=f"hub_pin_{index}",
            elem_b="pivot_barrel",
            min_overlap=0.010,
            name=f"spoke_{index} barrel is captured on hub pin",
        )
        ctx.expect_within(
            spoke,
            mast,
            axes="z",
            inner_elem="pivot_barrel",
            outer_elem=f"hub_cheek_{index}_upper",
            margin=0.0,
            name=f"spoke_{index} barrel fits within hub height",
        )

    for index, joint in enumerate(joints):
        spoke = spokes[index]
        rest_aabb = ctx.part_element_world_aabb(spoke, elem="spoke_bar")
        other_rest = [
            ctx.part_element_world_aabb(other, elem="spoke_bar")
            for other_index, other in enumerate(spokes)
            if other_index != index
        ]
        with ctx.pose({joint: 0.60}):
            raised_aabb = ctx.part_element_world_aabb(spoke, elem="spoke_bar")
            other_raised = [
                ctx.part_element_world_aabb(other, elem="spoke_bar")
                for other_index, other in enumerate(spokes)
                if other_index != index
            ]
        raised = (
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.08
        )
        stationary_others = all(
            before is not None
            and after is not None
            and abs(before[0][2] - after[0][2]) < 1e-6
            and abs(before[1][2] - after[1][2]) < 1e-6
            for before, after in zip(other_rest, other_raised)
        )
        ctx.check(
            f"hub_axis_{index} raises only its own spoke",
            raised and stationary_others,
            details=f"rest={rest_aabb}, raised={raised_aabb}, others={other_rest}->{other_raised}",
        )

    return ctx.report()


object_model = build_object_model()
