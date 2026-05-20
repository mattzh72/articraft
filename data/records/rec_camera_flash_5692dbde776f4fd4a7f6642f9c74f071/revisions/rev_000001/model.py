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
    model = ArticulatedObject(name="compact_speedlight_flash")

    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.28, 0.29, 0.30, 1.0))
    metal = model.material("metal", rgba=(0.62, 0.63, 0.66, 1.0))
    diffuser = model.material("diffuser", rgba=(0.90, 0.92, 0.95, 0.92))

    body = model.part("body")
    body.visual(
        Box((0.034, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="shoe_foot",
    )
    body.visual(
        Box((0.022, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_gray,
        name="shoe_block",
    )
    body.visual(
        Box((0.014, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark_gray,
        name="shoe_stem",
    )
    body.visual(
        Box((0.054, 0.035, 0.058)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=matte_black,
        name="battery_body",
    )
    body.visual(
        Box((0.008, 0.029, 0.032)),
        origin=Origin(xyz=(0.031, 0.0, 0.037)),
        material=dark_gray,
        name="grip_pad",
    )
    body.visual(
        Box((0.044, 0.033, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=satin_black,
        name="upper_step",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=satin_black,
        name="turret_pedestal",
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=satin_black,
        name="rotating_base",
    )
    swivel.visual(
        Box((0.008, 0.054, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, 0.012)),
        material=satin_black,
        name="rear_bridge",
    )
    swivel.visual(
        Box((0.016, 0.005, 0.030)),
        origin=Origin(xyz=(0.0, 0.0245, 0.019)),
        material=satin_black,
        name="yoke_cheek_0",
    )
    swivel.visual(
        Box((0.016, 0.005, 0.030)),
        origin=Origin(xyz=(0.0, -0.0245, 0.019)),
        material=satin_black,
        name="yoke_cheek_1",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.0045, length=0.044),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="trunnion",
    )
    head.visual(
        Box((0.066, 0.040, 0.034)),
        origin=Origin(xyz=(0.033, 0.0, 0.010)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.006, 0.036, 0.028)),
        origin=Origin(xyz=(0.069, 0.0, 0.010)),
        material=satin_black,
        name="front_bezel",
    )
    head.visual(
        Box((0.002, 0.030, 0.022)),
        origin=Origin(xyz=(0.073, 0.0, 0.010)),
        material=diffuser,
        name="lamp_window",
    )

    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-0.12,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")
    swivel_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        swivel,
        body,
        axis="z",
        positive_elem="rotating_base",
        negative_elem="turret_pedestal",
        max_gap=0.001,
        max_penetration=1e-6,
        name="swivel base seats on turret pedestal",
    )
    ctx.expect_within(
        head,
        swivel,
        axes="y",
        inner_elem="head_shell",
        margin=0.007,
        name="head stays within yoke width",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        positive_elem="head_shell",
        negative_elem="upper_step",
        min_gap=0.020,
        name="lamp head clears the stepped body at rest",
    )

    rest_window = aabb_center(ctx.part_element_world_aabb(head, elem="lamp_window"))
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        ctx.expect_gap(
            head,
            body,
            axis="z",
            positive_elem="head_shell",
            negative_elem="upper_step",
            min_gap=0.030,
            name="tilted head lifts farther above the body",
        )
        tilted_window = aabb_center(ctx.part_element_world_aabb(head, elem="lamp_window"))

    ctx.check(
        "tilt joint raises the lamp window",
        rest_window is not None
        and tilted_window is not None
        and tilted_window[2] > rest_window[2] + 0.025,
        details=f"rest={rest_window}, tilted={tilted_window}",
    )

    rest_window = aabb_center(ctx.part_element_world_aabb(head, elem="lamp_window"))
    with ctx.pose({swivel_joint: swivel_joint.motion_limits.upper}):
        swiveled_window = aabb_center(ctx.part_element_world_aabb(head, elem="lamp_window"))

    ctx.check(
        "swivel joint turns the head sideways",
        rest_window is not None
        and swiveled_window is not None
        and swiveled_window[1] > rest_window[1] + 0.045,
        details=f"rest={rest_window}, swiveled={swiveled_window}",
    )

    return ctx.report()


object_model = build_object_model()
