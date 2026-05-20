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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_fixture_head")

    dark_iron = Material("dark_blued_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    ground_steel = Material("ground_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    parkerized = Material("parkerized_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    pad_steel = Material("oiled_pad_steel", rgba=(0.35, 0.37, 0.34, 1.0))
    black = Material("black_socket_screws", rgba=(0.015, 0.015, 0.014, 1.0))

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.105, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_iron,
        name="round_base",
    )
    mast.visual(
        Cylinder(radius=0.036, length=0.58),
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
        material=ground_steel,
        name="vertical_mast",
    )
    mast.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.603)),
        material=dark_iron,
        name="top_cap",
    )

    branch_specs = (
        (0, 0.0, 0.215),
        (1, 2.0 * math.pi / 3.0, 0.330),
        (2, 4.0 * math.pi / 3.0, 0.445),
    )
    joint_radius = 0.124
    hub_height = 0.056

    for index, theta, hub_z in branch_specs:
        c = math.cos(theta)
        s = math.sin(theta)

        mast.visual(
            Cylinder(radius=0.056, length=hub_height),
            origin=Origin(xyz=(0.0, 0.0, hub_z)),
            material=parkerized,
            name=f"hub_collar_{index}",
        )
        mast.visual(
            Box((0.105, 0.076, hub_height)),
            origin=Origin(
                xyz=(c * 0.078, s * 0.078, hub_z),
                rpy=(0.0, 0.0, theta),
            ),
            material=parkerized,
            name=f"hub_block_{index}",
        )
        mast.visual(
            Cylinder(radius=0.034, length=hub_height),
            origin=Origin(xyz=(c * joint_radius, s * joint_radius, hub_z)),
            material=dark_iron,
            name=f"hub_bearing_{index}",
        )
        mast.visual(
            Box((0.050, 0.014, 0.026)),
            origin=Origin(
                xyz=(c * 0.006 - s * 0.037, s * 0.006 + c * 0.037, hub_z),
                rpy=(0.0, 0.0, theta),
            ),
            material=black,
            name=f"clamp_slot_{index}",
        )

        joint_z = hub_z + hub_height / 2.0
        arm = model.part(f"arm_{index}")
        arm.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=dark_iron,
            name="pivot_disk",
        )
        arm.visual(
            Cylinder(radius=0.014, length=0.255),
            origin=Origin(xyz=(0.142, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=ground_steel,
            name="round_arm",
        )
        arm.visual(
            Box((0.062, 0.026, 0.020)),
            origin=Origin(xyz=(0.052, 0.0, 0.022)),
            material=dark_iron,
            name="root_web",
        )
        arm.visual(
            Box((0.086, 0.070, 0.018)),
            origin=Origin(xyz=(0.292, 0.0, 0.025)),
            material=pad_steel,
            name="mounting_pad",
        )
        arm.visual(
            Box((0.046, 0.040, 0.014)),
            origin=Origin(xyz=(0.242, 0.0, 0.016)),
            material=dark_iron,
            name="pad_neck",
        )
        for bolt_i, (bx, by) in enumerate(
            ((0.270, -0.022), (0.314, -0.022), (0.270, 0.022), (0.314, 0.022))
        ):
            arm.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(bx, by, 0.0355)),
                material=black,
                name=f"pad_bolt_{bolt_i}",
            )

        model.articulation(
            f"mast_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=mast,
            child=arm,
            origin=Origin(xyz=(c * joint_radius, s * joint_radius, joint_z), rpy=(0.0, 0.0, theta)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.55, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    mast = object_model.get_part("mast")
    arms = [object_model.get_part(f"arm_{i}") for i in range(3)]
    joints = [object_model.get_articulation(f"mast_to_arm_{i}") for i in range(3)]

    for i, joint in enumerate(joints):
        ctx.check(
            f"arm {i} uses a revolute hub joint",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint.name} is {joint.articulation_type}",
        )
        ctx.expect_contact(
            arms[i],
            mast,
            elem_a="pivot_disk",
            elem_b=f"hub_bearing_{i}",
            contact_tol=0.0005,
            name=f"arm {i} pivot disk sits on its hub bearing",
        )
        ctx.expect_overlap(
            arms[i],
            mast,
            axes="xy",
            elem_a="pivot_disk",
            elem_b=f"hub_bearing_{i}",
            min_overlap=0.025,
            name=f"arm {i} pivot is centered on hub bearing",
        )

    ctx.expect_gap(arms[1], arms[0], axis="z", min_gap=0.070, name="lower and middle branch sweeps are vertically separated")
    ctx.expect_gap(arms[2], arms[1], axis="z", min_gap=0.070, name="middle and upper branch sweeps are vertically separated")

    rest_positions = [ctx.part_world_position(arm) for arm in arms]
    with ctx.pose({joints[0]: 0.55, joints[1]: -0.55, joints[2]: 0.55}):
        ctx.expect_gap(arms[1], arms[0], axis="z", min_gap=0.070, name="posed lower and middle branch sweeps stay distinct")
        ctx.expect_gap(arms[2], arms[1], axis="z", min_gap=0.070, name="posed middle and upper branch sweeps stay distinct")
        posed_positions = [ctx.part_world_position(arm) for arm in arms]

    for i, (rest, posed) in enumerate(zip(rest_positions, posed_positions)):
        ctx.check(
            f"arm {i} rotates about its hub",
            rest is not None
            and posed is not None
            and math.hypot(posed[0] - rest[0], posed[1] - rest[1]) < 0.001
            and abs(posed[2] - rest[2]) < 0.001,
            details=f"rest={rest}, posed={posed}",
        )

    return ctx.report()


object_model = build_object_model()
