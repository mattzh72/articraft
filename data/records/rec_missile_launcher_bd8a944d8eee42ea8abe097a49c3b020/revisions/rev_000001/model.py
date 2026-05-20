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
    model = ArticulatedObject(name="missile_launcher")

    # Base: Fixed pedestal
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.4, length=1.0),
        origin=Origin(xyz=(0.0, 0.0, 0.5)),
        name="pedestal",
    )

    # Turntable: Yaws on the base
    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.45, length=0.1),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="turntable_base",
    )
    # Yokes to support the cradle
    turntable.visual(
        Box((0.4, 0.1, 0.6)),
        origin=Origin(xyz=(0.0, -0.4, 0.4)),
        name="left_yoke",
    )
    turntable.visual(
        Box((0.4, 0.1, 0.6)),
        origin=Origin(xyz=(0.0, 0.4, 0.4)),
        name="right_yoke",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )

    # Cradle: Pitches on the turntable
    cradle = model.part("cradle")
    cradle.visual(
        Box((1.2, 0.6, 0.1)),
        origin=Origin(xyz=(0.2, 0.0, -0.05)),
        name="cradle_base",
    )
    # Pivot pins
    cradle.visual(
        Cylinder(radius=0.1, length=0.2),
        origin=Origin(xyz=(0.0, -0.4, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="left_pivot",
    )
    cradle.visual(
        Cylinder(radius=0.1, length=0.2),
        origin=Origin(xyz=(0.0, 0.4, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="right_pivot",
    )

    model.articulation(
        "turntable_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.6)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.2, upper=1.5),
    )

    # Canister: Rectangular canister pack mounted on top of the cradle
    canister = model.part("canister")
    canister.visual(
        Box((1.8, 0.6, 0.5)),
        origin=Origin(xyz=(0.5, 0.0, 0.25)),
        name="canister_pack",
    )

    model.articulation(
        "cradle_to_canister",
        ArticulationType.FIXED,
        parent=cradle,
        child=canister,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    canister = object_model.get_part("canister")

    # Allow overlap for the pivot pins inside the yokes
    ctx.allow_overlap(
        cradle,
        turntable,
        elem_a="left_pivot",
        elem_b="left_yoke",
        reason="Pivot pins are captured inside the yokes.",
    )
    ctx.allow_overlap(
        cradle,
        turntable,
        elem_a="right_pivot",
        elem_b="right_yoke",
        reason="Pivot pins are captured inside the yokes.",
    )

    ctx.expect_within(
        cradle,
        turntable,
        axes="y",
        inner_elem="cradle_base",
        margin=0.0,
        name="cradle fits between the yokes",
    )

    aabb_rest = ctx.part_element_world_aabb(canister, elem="canister_pack")
    with ctx.pose(turntable_to_cradle=1.5):
        aabb_up = ctx.part_element_world_aabb(canister, elem="canister_pack")
    
    if aabb_rest and aabb_up:
        ctx.check(
            "canister_pitches_up",
            aabb_up[1][2] > aabb_rest[1][2] + 0.2,
            details="The canister should pitch upward at the upper joint limit.",
        )

    return ctx.report()

object_model = build_object_model()
