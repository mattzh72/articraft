from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

MATERIAL_METAL = Material(name="metal", rgba=(0.7, 0.7, 0.75, 1.0))

def make_tube(width: float, height: float, wall: float):
    return (
        cq.Workplane("XY")
        .box(width, width, height)
        .translate((0, 0, height / 2))
        .faces(">Z")
        .shell(-wall)
    )

def make_solid(width: float, height: float):
    return (
        cq.Workplane("XY")
        .box(width, width, height)
        .translate((0, 0, height / 2))
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_mast")

    outer = model.part("outer_stage")
    outer_geom = make_tube(0.10, 1.0, 0.005)
    outer.visual(
        mesh_from_cadquery(outer_geom, "outer_tube"),
        name="outer_tube",
        material=MATERIAL_METAL,
    )

    middle = model.part("middle_stage")
    middle_geom = make_tube(0.08, 1.0, 0.005)
    middle.visual(
        mesh_from_cadquery(middle_geom, "middle_tube"),
        name="middle_tube",
        material=MATERIAL_METAL,
    )

    inner = model.part("inner_stage")
    inner_geom = make_solid(0.06, 1.0)
    inner.visual(
        mesh_from_cadquery(inner_geom, "inner_tube"),
        name="inner_tube",
        material=MATERIAL_METAL,
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.85),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.85),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    outer = object_model.get_part("outer_stage")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    
    ctx.allow_isolated_part(middle, reason="Telescoping tubes have internal clearance and slide within each other.")
    ctx.allow_isolated_part(inner, reason="Telescoping tubes have internal clearance and slide within each other.")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    # At rest, check centering
    ctx.expect_within(
        middle, outer, axes="xy", margin=0.006, name="middle tube is centered in outer tube"
    )
    ctx.expect_within(
        inner, middle, axes="xy", margin=0.006, name="inner tube is centered in middle tube"
    )

    # Check retained insertion at rest
    ctx.expect_overlap(
        middle, outer, axes="z", min_overlap=0.9, name="middle tube is mostly inside outer at rest"
    )
    ctx.expect_overlap(
        inner, middle, axes="z", min_overlap=0.9, name="inner tube is mostly inside middle at rest"
    )

    # Check fully extended pose
    with ctx.pose({outer_to_middle: 0.85, middle_to_inner: 0.85}):
        ctx.expect_within(
            middle, outer, axes="xy", margin=0.006, name="middle tube stays centered when extended"
        )
        ctx.expect_within(
            inner, middle, axes="xy", margin=0.006, name="inner tube stays centered when extended"
        )
        
        ctx.expect_overlap(
            middle, outer, axes="z", min_overlap=0.1, name="middle tube retains insertion when extended"
        )
        ctx.expect_overlap(
            inner, middle, axes="z", min_overlap=0.1, name="inner tube retains insertion when extended"
        )
        
        # Verify it actually extends
        outer_pos = ctx.part_world_position(outer)
        middle_pos = ctx.part_world_position(middle)
        inner_pos = ctx.part_world_position(inner)
        if outer_pos and middle_pos and inner_pos:
            ctx.check(
                "middle extends upward",
                middle_pos[2] > outer_pos[2] + 0.8,
            )
            ctx.check(
                "inner extends upward",
                inner_pos[2] > middle_pos[2] + 0.8,
            )

    return ctx.report()

object_model = build_object_model()
