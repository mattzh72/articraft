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
    model = ArticulatedObject(name="dual_finger_gripper")
    
    mat_dark = (0.2, 0.2, 0.2)
    mat_metal = (0.7, 0.7, 0.75)
    mat_accent = (0.8, 0.3, 0.1)

    palm = model.part("palm")
    # Base block
    palm.visual(Box((0.10, 0.04, 0.04)), origin=Origin(xyz=(0.0, 0.0, -0.02)), name="palm_base", color=mat_dark)
    # Wrist mount
    palm.visual(Cylinder(radius=0.015, length=0.02), origin=Origin(xyz=(0.0, 0.0, -0.05)), name="wrist_mount", color=mat_metal)
    
    # Palm brackets for Finger 0
    palm.visual(Box((0.01, 0.04, 0.04)), origin=Origin(xyz=(-0.045, 0.0, 0.02)), name="f0_bracket_outer", color=mat_dark)
    palm.visual(Box((0.01, 0.04, 0.04)), origin=Origin(xyz=(-0.015, 0.0, 0.02)), name="f0_bracket_inner", color=mat_dark)
    palm.visual(Cylinder(radius=0.005, length=0.04), origin=Origin(xyz=(-0.03, 0.0, 0.02), rpy=(0.0, math.pi/2, 0.0)), name="f0_pin", color=mat_metal)

    # Palm brackets for Finger 1
    palm.visual(Box((0.01, 0.04, 0.04)), origin=Origin(xyz=(0.015, 0.0, 0.02)), name="f1_bracket_inner", color=mat_dark)
    palm.visual(Box((0.01, 0.04, 0.04)), origin=Origin(xyz=(0.045, 0.0, 0.02)), name="f1_bracket_outer", color=mat_dark)
    palm.visual(Cylinder(radius=0.005, length=0.04), origin=Origin(xyz=(0.03, 0.0, 0.02), rpy=(0.0, math.pi/2, 0.0)), name="f1_pin", color=mat_metal)

    # Finger 0 Proximal
    f0_prox = model.part("finger_0_proximal")
    f0_prox.visual(Box((0.018, 0.02, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), name="body", color=mat_metal)
    f0_prox.visual(Box((0.004, 0.02, 0.025)), origin=Origin(xyz=(-0.007, 0.0, 0.0475)), name="ear_left", color=mat_metal)
    f0_prox.visual(Box((0.004, 0.02, 0.025)), origin=Origin(xyz=(0.007, 0.0, 0.0475)), name="ear_right", color=mat_metal)
    f0_prox.visual(Cylinder(radius=0.004, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.05), rpy=(0.0, math.pi/2, 0.0)), name="pin", color=mat_dark)

    # Finger 0 Distal
    f0_dist = model.part("finger_0_distal")
    f0_dist.visual(Box((0.009, 0.02, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), name="body", color=mat_accent)

    # Finger 1 Proximal
    f1_prox = model.part("finger_1_proximal")
    f1_prox.visual(Box((0.018, 0.02, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), name="body", color=mat_metal)
    f1_prox.visual(Box((0.004, 0.02, 0.025)), origin=Origin(xyz=(-0.007, 0.0, 0.0475)), name="ear_left", color=mat_metal)
    f1_prox.visual(Box((0.004, 0.02, 0.025)), origin=Origin(xyz=(0.007, 0.0, 0.0475)), name="ear_right", color=mat_metal)
    f1_prox.visual(Cylinder(radius=0.004, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.05), rpy=(0.0, math.pi/2, 0.0)), name="pin", color=mat_dark)

    # Finger 1 Distal
    f1_dist = model.part("finger_1_distal")
    f1_dist.visual(Box((0.009, 0.02, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), name="body", color=mat_accent)

    # Articulations
    limits_prox = MotionLimits(effort=5.0, velocity=2.0, lower=-0.2, upper=1.0)
    limits_dist = MotionLimits(effort=5.0, velocity=2.0, lower=-0.2, upper=1.2)

    model.articulation(
        "palm_to_f0_prox",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=f0_prox,
        origin=Origin(xyz=(-0.03, 0.0, 0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits_prox
    )

    model.articulation(
        "f0_prox_to_dist",
        ArticulationType.REVOLUTE,
        parent=f0_prox,
        child=f0_dist,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits_dist
    )

    model.articulation(
        "palm_to_f1_prox",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=f1_prox,
        origin=Origin(xyz=(0.03, 0.0, 0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits_prox
    )

    model.articulation(
        "f1_prox_to_dist",
        ArticulationType.REVOLUTE,
        parent=f1_prox,
        child=f1_dist,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=limits_dist
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    palm = object_model.get_part("palm")
    f0_prox = object_model.get_part("finger_0_proximal")
    f0_dist = object_model.get_part("finger_0_distal")
    f1_prox = object_model.get_part("finger_1_proximal")
    f1_dist = object_model.get_part("finger_1_distal")

    ctx.allow_overlap(palm, f0_prox, reason="Hinge pin on palm intentionally passes through proximal finger.")
    ctx.allow_overlap(palm, f1_prox, reason="Hinge pin on palm intentionally passes through proximal finger.")
    ctx.allow_overlap(f0_prox, f0_dist, reason="Hinge pin on proximal finger intentionally passes through distal finger.")
    ctx.allow_overlap(f1_prox, f1_dist, reason="Hinge pin on proximal finger intentionally passes through distal finger.")

    f0_dist_pos_rest = ctx.part_world_position(f0_dist)
    f1_dist_pos_rest = ctx.part_world_position(f1_dist)
    
    with ctx.pose({"palm_to_f0_prox": 1.0, "f0_prox_to_dist": 1.0, "palm_to_f1_prox": 1.0, "f1_prox_to_dist": 1.0}):
        f0_dist_pos_closed = ctx.part_world_position(f0_dist)
        f1_dist_pos_closed = ctx.part_world_position(f1_dist)
        
        if f0_dist_pos_rest and f0_dist_pos_closed:
            ctx.check("f0_closes_inwards", f0_dist_pos_closed[0] > f0_dist_pos_rest[0], "Finger 0 should move towards +X")
        if f1_dist_pos_rest and f1_dist_pos_closed:
            ctx.check("f1_closes_inwards", f1_dist_pos_closed[0] < f1_dist_pos_rest[0], "Finger 1 should move towards -X")

    return ctx.report()

object_model = build_object_model()