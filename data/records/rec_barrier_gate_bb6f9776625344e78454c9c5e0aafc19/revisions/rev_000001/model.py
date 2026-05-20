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
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toll_highway_gate_arm")

    safety_white = model.material("safety_white", rgba=(0.94, 0.94, 0.88, 1.0))
    signal_red = model.material("signal_red", rgba=(0.82, 0.04, 0.03, 1.0))
    booth_gray = model.material("booth_gray", rgba=(0.55, 0.58, 0.57, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    concrete = model.material("concrete", rgba=(0.38, 0.38, 0.36, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.15, 0.32, 0.45, 0.85))
    amber = model.material("amber_lens", rgba=(1.0, 0.58, 0.05, 1.0))

    booth_post = model.part("booth_post")
    booth_post.visual(
        Box((0.66, 0.58, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="base_plinth",
    )
    booth_post.visual(
        Box((0.38, 0.44, 1.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=booth_gray,
        name="cabinet_shell",
    )
    booth_post.visual(
        Box((0.42, 0.48, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.37)),
        material=dark_metal,
        name="cap_plate",
    )
    booth_post.visual(
        Box((0.012, 0.22, 0.46)),
        origin=Origin(xyz=(0.196, 0.0, 0.70)),
        material=glass_blue,
        name="service_panel",
    )
    booth_post.visual(
        Box((0.026, 0.09, 0.09)),
        origin=Origin(xyz=(0.202, -0.105, 1.15)),
        material=amber,
        name="warning_lamp_0",
    )
    booth_post.visual(
        Box((0.026, 0.09, 0.09)),
        origin=Origin(xyz=(0.202, 0.105, 1.15)),
        material=amber,
        name="warning_lamp_1",
    )

    # Fixed yoke and axle carried by the post.  The moving boom hub sits between
    # the two cheek plates, while the axle is intentionally captured inside it.
    pivot_x = 0.36
    pivot_z = 1.08
    for y in (-0.22, 0.22):
        booth_post.visual(
            Box((0.34, 0.06, 0.34)),
            origin=Origin(xyz=(0.35, y, pivot_z)),
            material=dark_metal,
            name=f"yoke_plate_{0 if y < 0 else 1}",
        )
    booth_post.visual(
        Cylinder(radius=0.038, length=0.54),
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="axle_pin",
    )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.13, length=0.28),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_drum",
    )
    boom.visual(
        Box((0.22, 0.16, 0.18)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=dark_metal,
        name="arm_clamp",
    )
    boom.visual(
        Box((4.80, 0.10, 0.12)),
        origin=Origin(xyz=(2.55, 0.0, 0.0)),
        material=safety_white,
        name="boom_beam",
    )
    # Red reflective bands wrap the top and sides of the white boom beam.
    for idx, x in enumerate((0.75, 1.45, 2.15, 2.85, 3.55, 4.25)):
        boom.visual(
            Box((0.30, 0.112, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.064)),
            material=signal_red,
            name=f"stripe_top_{idx}",
        )
        boom.visual(
            Box((0.30, 0.008, 0.124)),
            origin=Origin(xyz=(x, -0.054, 0.0)),
            material=signal_red,
            name=f"stripe_front_{idx}",
        )
        boom.visual(
            Box((0.30, 0.008, 0.124)),
            origin=Origin(xyz=(x, 0.054, 0.0)),
            material=signal_red,
            name=f"stripe_rear_{idx}",
        )
    boom.visual(
        Box((0.22, 0.045, 0.40)),
        origin=Origin(xyz=(-0.01, 0.0, -0.26)),
        material=dark_metal,
        name="counterweight_fin",
    )

    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=booth_post,
        child=boom,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    booth_post = object_model.get_part("booth_post")
    boom = object_model.get_part("boom")
    hinge = object_model.get_articulation("post_to_boom")

    ctx.allow_overlap(
        booth_post,
        boom,
        elem_a="axle_pin",
        elem_b="pivot_drum",
        reason="The fixed axle is intentionally captured inside the rotating boom hub.",
    )
    ctx.expect_overlap(
        booth_post,
        boom,
        axes="y",
        elem_a="axle_pin",
        elem_b="pivot_drum",
        min_overlap=0.24,
        name="axle passes through the pivot drum",
    )
    ctx.expect_within(
        booth_post,
        boom,
        axes="xz",
        inner_elem="axle_pin",
        outer_elem="pivot_drum",
        margin=0.005,
        name="axle is centered in the boom hub",
    )

    limits = hinge.motion_limits
    ctx.check(
        "boom joint is a limited horizontal revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in hinge.axis) == (0.0, -1.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper > 1.2,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={limits}",
    )

    closed_aabb = ctx.part_element_world_aabb(boom, elem="boom_beam")
    with ctx.pose({hinge: limits.upper if limits is not None and limits.upper is not None else 1.35}):
        raised_aabb = ctx.part_element_world_aabb(boom, elem="boom_beam")

    if closed_aabb is not None and raised_aabb is not None:
        closed_min, closed_max = closed_aabb
        raised_min, raised_max = raised_aabb
        ctx.check(
            "long striped boom raises upward at the free end",
            raised_max[2] > closed_max[2] + 3.0,
            details=f"closed_max_z={closed_max[2]:.3f}, raised_max_z={raised_max[2]:.3f}",
        )
        ctx.check(
            "boom arm has highway barrier proportions",
            (closed_max[0] - closed_min[0]) > 4.5
            and (closed_max[1] - closed_min[1]) < 0.18
            and (closed_max[2] - closed_min[2]) < 0.18,
            details=f"closed_aabb={closed_aabb}",
        )
    else:
        ctx.fail("boom beam aabb available", "Could not measure the boom beam element.")

    return ctx.report()


object_model = build_object_model()
