from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lab_safety_eyewash_faucet")

    stainless = model.material("stainless", rgba=(0.73, 0.76, 0.79, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.32, 0.34, 0.36, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.94, 0.83, 0.13, 1.0))
    spray_green = model.material("spray_green", rgba=(0.25, 0.63, 0.30, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.075, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_metal,
        name="base_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.040, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=stainless,
        name="base_skirt",
    )
    pedestal.visual(
        Cylinder(radius=0.026, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.226)),
        material=stainless,
        name="main_column",
    )
    pedestal.visual(
        Cylinder(radius=0.018, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.411)),
        material=stainless,
        name="riser_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.017, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, 0.472), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="head_manifold",
    )
    pedestal.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(-0.047, 0.0, 0.491)),
        material=stainless,
        name="left_nozzle_neck",
    )
    pedestal.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.047, 0.0, 0.491)),
        material=stainless,
        name="right_nozzle_neck",
    )
    pedestal.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(-0.047, 0.0, 0.509)),
        material=spray_green,
        name="left_spray_head",
    )
    pedestal.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.047, 0.0, 0.509)),
        material=spray_green,
        name="right_spray_head",
    )
    pedestal.visual(
        Box((0.042, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.016, 0.404)),
        material=stainless,
        name="hinge_mount_block",
    )
    pedestal.visual(
        Box((0.008, 0.020, 0.024)),
        origin=Origin(xyz=(-0.020, 0.036, 0.404)),
        material=stainless,
        name="left_hinge_ear",
    )
    pedestal.visual(
        Box((0.008, 0.020, 0.024)),
        origin=Origin(xyz=(0.020, 0.036, 0.404)),
        material=stainless,
        name="right_hinge_ear",
    )
    pedestal.visual(
        Box((0.020, 0.044, 0.070)),
        origin=Origin(xyz=(0.0, 0.004, 0.441)),
        material=stainless,
        name="head_support_web",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.53)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
    )

    paddle_handle = model.part("paddle_handle")
    paddle_handle.visual(
        Cylinder(radius=0.0055, length=0.032),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="handle_hinge_barrel",
    )
    paddle_handle.visual(
        Box((0.014, 0.090, 0.010)),
        origin=Origin(xyz=(0.0, 0.047, -0.008)),
        material=safety_yellow,
        name="handle_arm",
    )
    paddle_handle.visual(
        Box((0.074, 0.008, 0.048)),
        origin=Origin(xyz=(0.0, 0.093, -0.019)),
        material=safety_yellow,
        name="push_paddle",
    )
    paddle_handle.inertial = Inertial.from_geometry(
        Box((0.08, 0.11, 0.05)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.055, -0.015)),
    )

    model.articulation(
        "paddle_hinge",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=paddle_handle,
        origin=Origin(xyz=(0.0, 0.036, 0.404)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=0.85,
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

    pedestal = object_model.get_part("pedestal")
    paddle_handle = object_model.get_part("paddle_handle")
    paddle_hinge = object_model.get_articulation("paddle_hinge")
    push_paddle = paddle_handle.get_visual("push_paddle")
    head_manifold = pedestal.get_visual("head_manifold")

    ctx.expect_gap(
        paddle_handle,
        pedestal,
        axis="z",
        positive_elem=push_paddle,
        negative_elem="base_skirt",
        min_gap=0.22,
        name="push paddle sits well above the pedestal base",
    )
    ctx.expect_overlap(
        pedestal,
        pedestal,
        axes="z",
        elem_a=head_manifold,
        elem_b="left_nozzle_neck",
        min_overlap=0.006,
        name="left nozzle neck is attached to the head manifold",
    )
    ctx.expect_overlap(
        pedestal,
        pedestal,
        axes="z",
        elem_a=head_manifold,
        elem_b="right_nozzle_neck",
        min_overlap=0.006,
        name="right nozzle neck is attached to the head manifold",
    )

    left_head_aabb = ctx.part_element_world_aabb(pedestal, elem="left_spray_head")
    right_head_aabb = ctx.part_element_world_aabb(pedestal, elem="right_spray_head")
    rest_aabb = ctx.part_element_world_aabb(paddle_handle, elem=push_paddle)
    with ctx.pose({paddle_hinge: 0.75}):
        actuated_aabb = ctx.part_element_world_aabb(paddle_handle, elem=push_paddle)
        ctx.expect_gap(
            paddle_handle,
            pedestal,
            axis="z",
            positive_elem=push_paddle,
            negative_elem="base_skirt",
            min_gap=0.12,
            name="actuated paddle still clears the pedestal body",
        )

    ctx.check(
        "dual spray heads are laterally paired across the manifold",
        left_head_aabb is not None
        and right_head_aabb is not None
        and left_head_aabb[1][0] < right_head_aabb[0][0]
        and abs((left_head_aabb[0][2] + left_head_aabb[1][2]) - (right_head_aabb[0][2] + right_head_aabb[1][2])) < 0.004,
        details=f"left_head_aabb={left_head_aabb}, right_head_aabb={right_head_aabb}",
    )
    ctx.check(
        "push paddle rotates downward when actuated",
        rest_aabb is not None
        and actuated_aabb is not None
        and actuated_aabb[0][2] < rest_aabb[0][2] - 0.04
        and actuated_aabb[0][1] < rest_aabb[0][1] - 0.015,
        details=f"rest_aabb={rest_aabb}, actuated_aabb={actuated_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
