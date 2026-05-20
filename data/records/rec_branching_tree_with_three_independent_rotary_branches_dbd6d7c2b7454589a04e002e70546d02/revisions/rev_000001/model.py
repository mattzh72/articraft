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
    model = ArticulatedObject(name="three_branch_radial_base")

    cast_dark = model.material("dark_cast_metal", color=(0.08, 0.085, 0.09, 1.0))
    machined = model.material("machined_steel", color=(0.56, 0.58, 0.58, 1.0))
    link_blue = model.material("blue_anodized_links", color=(0.05, 0.22, 0.62, 1.0))
    black = model.material("black_fasteners", color=(0.015, 0.015, 0.018, 1.0))
    brass = model.material("bronze_bushings", color=(0.78, 0.55, 0.22, 1.0))

    base = model.part("radial_base")
    base.visual(
        Cylinder(radius=0.23, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=cast_dark,
        name="round_foot",
    )
    base.visual(
        Cylinder(radius=0.145, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=machined,
        name="center_hub",
    )

    joint_radius = 0.48
    housing_top_z = 0.1125

    for i in range(3):
        theta = i * 2.0 * math.pi / 3.0
        c = math.cos(theta)
        s = math.sin(theta)
        x = joint_radius * c
        y = joint_radius * s

        base.visual(
            Box((joint_radius, 0.084, 0.035)),
            origin=Origin(
                xyz=(0.5 * joint_radius * c, 0.5 * joint_radius * s, 0.035),
                rpy=(0.0, 0.0, theta),
            ),
            material=cast_dark,
            name=f"radial_arm_{i}",
        )
        base.visual(
            Cylinder(radius=0.108, length=0.026),
            origin=Origin(xyz=(x, y, 0.058)),
            material=cast_dark,
            name=f"mount_pad_{i}",
        )
        base.visual(
            Cylinder(radius=0.074, length=0.071),
            origin=Origin(xyz=(x, y, 0.077)),
            material=machined,
            name=f"bearing_{i}",
        )
        base.visual(
            Cylinder(radius=0.044, length=0.010),
            origin=Origin(xyz=(x, y, 0.108)),
            material=brass,
            name=f"bushing_{i}",
        )

        for j, lateral in enumerate((-0.050, 0.050)):
            sx = x + lateral * math.cos(theta + math.pi / 2.0)
            sy = y + lateral * math.sin(theta + math.pi / 2.0)
            base.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(sx, sy, 0.067)),
                material=black,
                name=f"screw_{i}_{j}",
            )

        link = model.part(f"link_{i}")
        link.visual(
            Cylinder(radius=0.064, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, 0.0125)),
            material=machined,
            name="pivot_cap",
        )
        link.visual(
            Box((0.420, 0.046, 0.026)),
            origin=Origin(xyz=(0.230, 0.0, 0.018)),
            material=link_blue,
            name="rigid_link",
        )
        link.visual(
            Box((0.330, 0.020, 0.012)),
            origin=Origin(xyz=(0.255, 0.0, 0.036)),
            material=machined,
            name="top_rib",
        )
        link.visual(
            Cylinder(radius=0.047, length=0.030),
            origin=Origin(xyz=(0.460, 0.0, 0.018)),
            material=machined,
            name="end_boss",
        )
        link.visual(
            Cylinder(radius=0.025, length=0.034),
            origin=Origin(xyz=(0.460, 0.0, 0.018)),
            material=brass,
            name="end_bushing",
        )

        model.articulation(
            f"base_to_link_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=link,
            origin=Origin(xyz=(x, y, housing_top_z), rpy=(0.0, 0.0, theta)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.05, upper=1.05),
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

    base = object_model.get_part("radial_base")
    joints = [object_model.get_articulation(f"base_to_link_{i}") for i in range(3)]
    links = [object_model.get_part(f"link_{i}") for i in range(3)]

    ctx.check(
        "three independent revolute branches",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )

    for i, (joint, link) in enumerate(zip(joints, links)):
        ctx.expect_gap(
            link,
            base,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="pivot_cap",
            negative_elem=f"bearing_{i}",
            name=f"link_{i} pivot cap seats on bearing",
        )
        ctx.expect_overlap(
            link,
            base,
            axes="xy",
            min_overlap=0.070,
            elem_a="pivot_cap",
            elem_b=f"bearing_{i}",
            name=f"link_{i} pivot is centered over bearing",
        )
        with ctx.pose({joint: 0.70}):
            ctx.expect_gap(
                link,
                base,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem="pivot_cap",
                negative_elem=f"bearing_{i}",
                name=f"link_{i} stays seated while rotating",
            )

    return ctx.report()


object_model = build_object_model()
