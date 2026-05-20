from __future__ import annotations

import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dropper_seatpost")

    # Outer tube
    outer_tube_cq = (
        cq.Workplane("XY")
        .circle(0.015)
        .circle(0.012)
        .extrude(0.200)
    )
    collar_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.185)
        .circle(0.020)
        .circle(0.012)
        .extrude(0.015)
    )
    outer_shell = outer_tube_cq.union(collar_cq)

    # Cutouts or mounts for the levers on the collar
    # Let's add two small bosses for the lever pivots
    left_boss = (
        cq.Workplane("XZ")
        .workplane(offset=0.020)
        .center(0, 0.1925)
        .circle(0.005)
        .extrude(0.005)
    )
    right_boss = (
        cq.Workplane("XZ")
        .workplane(offset=-0.020)
        .center(0, 0.1925)
        .circle(0.005)
        .extrude(-0.005)
    )
    outer_shell = outer_shell.union(left_boss).union(right_boss)

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        mesh_from_cadquery(outer_shell, "outer_tube_mesh"),
        name="outer_shell"
    )

    # Inner tube
    inner_tube_cq = (
        cq.Workplane("XY")
        .circle(0.0118)
        .circle(0.009)
        .extrude(0.200)
    )
    seat_clamp_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.200)
        .box(0.040, 0.030, 0.020, centered=(True, True, False))
    )
    inner_shell = inner_tube_cq.union(seat_clamp_cq)

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        mesh_from_cadquery(inner_shell, "inner_tube_mesh"),
        name="inner_shell"
    )

    # Slide joint
    model.articulation(
        "inner_tube_slide",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.140),
    )

    # Levers
    left_lever = model.part("left_lever")
    left_lever.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(0.0, 0.0025, 0.0), rpy=(math.pi/2, 0, 0)),
        name="left_lever_pivot_vis"
    )
    left_lever.visual(
        Box((0.006, 0.005, 0.030)),
        origin=Origin(xyz=(0.0, 0.0025, -0.015)),
        name="left_lever_handle"
    )

    right_lever = model.part("right_lever")
    right_lever.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(0.0, -0.0025, 0.0), rpy=(math.pi/2, 0, 0)),
        name="right_lever_pivot_vis"
    )
    right_lever.visual(
        Box((0.006, 0.005, 0.030)),
        origin=Origin(xyz=(0.0, -0.0025, -0.015)),
        name="right_lever_handle"
    )

    # Left lever joint
    model.articulation(
        "left_lever_pivot",
        ArticulationType.REVOLUTE,
        parent=outer_tube,
        child=left_lever,
        origin=Origin(xyz=(0.0, 0.025, 0.1925)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0, lower=-0.5, upper=0.0),
    )

    # Right lever joint
    model.articulation(
        "right_lever_pivot",
        ArticulationType.REVOLUTE,
        parent=outer_tube,
        child=right_lever,
        origin=Origin(xyz=(0.0, -0.025, 0.1925)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0, lower=0.0, upper=0.5),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    inner_tube = object_model.get_part("inner_tube")
    outer_tube = object_model.get_part("outer_tube")
    left_lever = object_model.get_part("left_lever")
    right_lever = object_model.get_part("right_lever")
    
    ctx.allow_isolated_part(
        inner_tube,
        reason="Telescoping inner tube slides inside outer tube sleeve with a small clearance."
    )
    
    ctx.expect_within(
        inner_tube,
        outer_tube,
        axes="xy",
        margin=0.002,
        name="inner tube stays centered in the outer tube"
    )
    
    ctx.expect_contact(
        left_lever,
        outer_tube,
        name="left lever is mounted on the outer tube collar"
    )
    
    ctx.expect_contact(
        right_lever,
        outer_tube,
        name="right lever is mounted on the outer tube collar"
    )
    
    with ctx.pose(inner_tube_slide=0.140):
        ctx.expect_overlap(
            inner_tube,
            outer_tube,
            axes="z",
            min_overlap=0.005,
            name="extended inner tube retains insertion in the outer tube"
        )
        
    with ctx.pose(left_lever_pivot=-0.5, right_lever_pivot=0.5):
        # When pressed inward, the levers should still be within the expected bounds
        ctx.expect_contact(
            left_lever,
            outer_tube,
            name="left lever remains mounted when pressed inward"
        )
        ctx.expect_contact(
            right_lever,
            outer_tube,
            name="right lever remains mounted when pressed inward"
        )
    
    return ctx.report()

object_model = build_object_model()