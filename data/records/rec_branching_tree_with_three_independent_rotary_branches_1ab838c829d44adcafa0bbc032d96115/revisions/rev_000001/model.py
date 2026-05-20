from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


COLLAR_HEIGHT = 0.060
COLLAR_OUTER_RADIUS = 0.075
COLLAR_INNER_RADIUS = 0.046
JOURNAL_RADIUS = 0.049
SHOULDER_GAP = 0.005
COLLAR_Z = (0.220, 0.360, 0.500)


def _collar_shell() -> cq.Workplane:
    """A short bored collar centered on its joint."""
    return (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_RADIUS)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, -COLLAR_HEIGHT / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_rotary_stand")

    black = model.material("cast_black", rgba=(0.02, 0.02, 0.018, 1.0))
    steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    brass = model.material("bearing_bronze", rgba=(0.74, 0.52, 0.26, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    arm_materials = (
        model.material("oxide_red", rgba=(0.60, 0.10, 0.07, 1.0)),
        model.material("deep_green", rgba=(0.05, 0.34, 0.18, 1.0)),
        model.material("navy_blue", rgba=(0.06, 0.14, 0.45, 1.0)),
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.255, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=black,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.090, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=black,
        name="base_boss",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=steel,
        name="tower_core",
    )

    for i, z in enumerate(COLLAR_Z):
        base.visual(
            Cylinder(radius=JOURNAL_RADIUS, length=COLLAR_HEIGHT + 0.006),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brass,
            name=f"journal_{i}",
        )
        base.visual(
            Cylinder(radius=0.068, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, z - COLLAR_HEIGHT / 2.0 - SHOULDER_GAP - 0.005)),
            material=steel,
            name=f"lower_shoulder_{i}",
        )
        base.visual(
            Cylinder(radius=0.068, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, z + COLLAR_HEIGHT / 2.0 + SHOULDER_GAP + 0.005)),
            material=steel,
            name=f"upper_shoulder_{i}",
        )

    collar_mesh = mesh_from_cadquery(_collar_shell(), "collar_shell", tolerance=0.0008)
    for i, (z, yaw) in enumerate(zip(COLLAR_Z, (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0))):
        arm = model.part(f"arm_{i}")
        material = arm_materials[i]
        arm.visual(collar_mesh, material=material, name="collar_shell")
        arm.visual(
            Cylinder(radius=0.018, length=0.500),
            origin=Origin(xyz=(0.315, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name="arm_tube",
        )
        arm.visual(
            Cylinder(radius=0.017, length=0.100),
            origin=Origin(xyz=(0.565, 0.0, 0.043)),
            material=material,
            name="end_post",
        )
        arm.visual(
            Sphere(radius=0.028),
            origin=Origin(xyz=(0.565, 0.0, 0.098)),
            material=rubber,
            name="end_knob",
        )
        model.articulation(
            f"base_to_arm_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=arm,
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                lower=-math.pi,
                upper=math.pi,
                effort=12.0,
                velocity=1.6,
            ),
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

    base = object_model.get_part("base")
    arms = [object_model.get_part(f"arm_{i}") for i in range(3)]
    joints = [object_model.get_articulation(f"base_to_arm_{i}") for i in range(3)]

    ctx.check("three independent rotary branches", len(arms) == 3 and len(joints) == 3)

    for i, arm in enumerate(arms):
        ctx.allow_overlap(
            base,
            arm,
            elem_a=f"journal_{i}",
            elem_b="collar_shell",
            reason="The bronze journal is intentionally captured inside the rotating collar bore as the supported rotary axis.",
        )
        ctx.expect_overlap(
            arm,
            base,
            axes="z",
            elem_a="collar_shell",
            elem_b=f"journal_{i}",
            min_overlap=0.055,
            name=f"collar_{i}_has_journal_engagement",
        )
        ctx.expect_within(
            base,
            arm,
            axes="xy",
            inner_elem=f"journal_{i}",
            outer_elem="collar_shell",
            margin=0.0,
            name=f"journal_{i}_is_centered_in_collar",
        )
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem="collar_shell",
            negative_elem=f"lower_shoulder_{i}",
            min_gap=0.004,
            max_gap=0.006,
            name=f"collar_{i}_clears_lower_shoulder",
        )
        ctx.expect_gap(
            base,
            arm,
            axis="z",
            positive_elem=f"upper_shoulder_{i}",
            negative_elem="collar_shell",
            min_gap=0.004,
            max_gap=0.006,
            name=f"collar_{i}_clears_upper_shoulder",
        )

    ctx.expect_origin_gap(arms[1], arms[0], axis="z", min_gap=0.12, max_gap=0.16, name="middle collar is above lower collar")
    ctx.expect_origin_gap(arms[2], arms[1], axis="z", min_gap=0.12, max_gap=0.16, name="upper collar is above middle collar")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[j] + hi[j]) * 0.5 for j in range(3))

    for i, (arm, joint) in enumerate(zip(arms, joints)):
        rest = aabb_center(ctx.part_element_world_aabb(arm, elem="end_knob"))
        with ctx.pose({joint: math.pi / 2.0}):
            turned = aabb_center(ctx.part_element_world_aabb(arm, elem="end_knob"))
        moved = (
            rest is not None
            and turned is not None
            and math.hypot(turned[0] - rest[0], turned[1] - rest[1]) > 0.70
            and abs(turned[2] - rest[2]) < 0.002
        )
        ctx.check(f"arm_{i}_rotates_as_single_rigid_member", moved, details=f"rest={rest}, turned={turned}")

    return ctx.report()


object_model = build_object_model()
