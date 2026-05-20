from __future__ import annotations

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
    model = ArticulatedObject(name="stand_mixer")

    red = model.material("mixer_red", color=(0.8, 0.1, 0.1))
    silver = model.material("silver", color=(0.7, 0.7, 0.7))
    steel = model.material("stainless_steel", color=(0.85, 0.85, 0.9))
    black = model.material("black_plastic", color=(0.1, 0.1, 0.1))

    # Base Frame
    base_frame = model.part("base_frame")
    base_frame.visual(Box((0.35, 0.22, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=red, name="base_plate")
    base_frame.visual(Box((0.12, 0.12, 0.35)), origin=Origin(xyz=(-0.115, 0.0, 0.225)), material=red, name="column")
    base_frame.visual(Box((0.30, 0.12, 0.10)), origin=Origin(xyz=(-0.025, 0.0, 0.45)), material=red, name="head")
    base_frame.visual(Cylinder(radius=0.03, length=0.02), origin=Origin(xyz=(0.08, 0.0, 0.39)), material=red, name="hub")

    # Lever
    lever = model.part("lever")
    lever.visual(Cylinder(radius=0.015, length=0.025), origin=Origin(xyz=(0.0, 0.0125, 0.0), rpy=(1.5708, 0.0, 0.0)), material=black, name="standoff")
    lever.visual(Cylinder(radius=0.008, length=0.10), origin=Origin(xyz=(0.05, 0.02, 0.0), rpy=(0.0, 1.5708, 0.0)), material=black, name="handle")

    lever_joint = model.articulation(
        "lever_joint",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=lever,
        origin=Origin(xyz=(-0.08, 0.059, 0.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=1.5708)
    )

    # Bowl Platform
    hoop_outer = cq.Workplane("XY").cylinder(0.02, 0.105).translate((0.135, 0, 0))
    hoop_inner = cq.Workplane("XY").cylinder(0.02, 0.095).translate((0.135, 0, 0))
    hoop = hoop_outer.cut(hoop_inner)
    slider_block = cq.Workplane("XY").rect(0.05, 0.08).extrude(0.04).translate((0.024, 0, -0.02))
    platform_geom = hoop.union(slider_block)

    bowl_platform = model.part("bowl_platform")
    bowl_platform.visual(mesh_from_cadquery(platform_geom, "platform_geom"), material=silver)

    platform_joint = model.articulation(
        "platform_joint",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=bowl_platform,
        origin=Origin(xyz=(-0.055, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.1, lower=0.0, upper=0.06)
    )

    # Bowl
    outer_bowl = cq.Workplane("XY").workplane(offset=-0.12).circle(0.06).workplane(offset=0.16).circle(0.11).loft()
    inner_bowl = cq.Workplane("XY").workplane(offset=-0.117).circle(0.057).workplane(offset=0.16).circle(0.107).loft()
    bowl_geom = outer_bowl.cut(inner_bowl)

    bowl = model.part("bowl")
    bowl.visual(mesh_from_cadquery(bowl_geom, "bowl_geom"), material=steel)

    model.articulation(
        "bowl_joint",
        ArticulationType.FIXED,
        parent=bowl_platform,
        child=bowl,
        origin=Origin(xyz=(0.135, 0.0, 0.0))
    )

    # Whisk
    shaft = cq.Workplane("XY").circle(0.005).extrude(0.15).translate((0, 0, -0.15))
    paddle = cq.Workplane("XZ").rect(0.08, 0.10).extrude(0.004, both=True).translate((0, 0, -0.10))
    cutout = cq.Workplane("XZ").rect(0.05, 0.07).extrude(0.01, both=True).translate((0, 0, -0.10))
    paddle = paddle.cut(cutout)
    whisk_geom = shaft.union(paddle)

    whisk = model.part("whisk")
    whisk.visual(mesh_from_cadquery(whisk_geom, "whisk_geom"), material=silver)

    model.articulation(
        "whisk_joint",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=whisk,
        origin=Origin(xyz=(0.08, 0.0, 0.38)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap("base_frame", "lever", reason="Lever is mounted in a recess on the base frame.")
    ctx.allow_overlap("base_frame", "bowl_platform", reason="Platform slides on the column track.")
    ctx.allow_overlap("bowl", "bowl_platform", reason="Bowl rests securely inside the platform hoop.")
    
    base_frame = object_model.get_part("base_frame")
    bowl_platform = object_model.get_part("bowl_platform")
    lever = object_model.get_part("lever")
    whisk = object_model.get_part("whisk")
    bowl = object_model.get_part("bowl")
    
    ctx.expect_gap(whisk, bowl, axis="z", min_gap=0.005, name="Whisk clears the bowl when lowered")
    
    lever_joint = object_model.get_articulation("lever_joint")
    platform_joint = object_model.get_articulation("platform_joint")
    with ctx.pose({lever_joint: 1.5708, platform_joint: 0.06}):
        ctx.expect_overlap(whisk, bowl, axes="xy", name="Whisk is inside the bowl's footprint when raised")
        
    return ctx.report()


object_model = build_object_model()