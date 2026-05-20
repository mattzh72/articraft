from __future__ import annotations

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_case() -> cq.Workplane:
    # Base steps
    base_step1 = cq.Workplane("XY").box(0.14, 0.14, 0.015).translate((0, 0, 0.015 / 2))
    base_step2 = cq.Workplane("XY").box(0.12, 0.12, 0.015).translate((0, 0, 0.015 + 0.015 / 2))
    
    # Pyramid body
    pyramid = (
        cq.Workplane("XY").workplane(offset=0.03)
        .rect(0.11, 0.11)
        .workplane(offset=0.20)
        .rect(0.05, 0.05)
        .loft()
    )
    
    # Hollow interior
    pyramid_inner = (
        cq.Workplane("XY").workplane(offset=0.03)
        .rect(0.09, 0.09)
        .workplane(offset=0.19)
        .rect(0.04, 0.04)
        .loft()
    )
    pyramid = pyramid.cut(pyramid_inner)
    
    # Apex slot for pendulum rod
    slot = cq.Workplane("XY").workplane(offset=0.22).box(0.06, 0.012, 0.04)
    pyramid = pyramid.cut(slot)
    
    # Apex pivot shaft
    shaft = cq.Workplane("XZ").workplane(offset=0).center(0, 0.22).circle(0.0015).extrude(0.025, both=True)
    
    # Winding key hole on the rear face (+Y)
    key_hole = cq.Workplane("XZ").workplane(offset=0).center(0, 0.08).circle(0.0035).extrude(0.1)
    pyramid = pyramid.cut(key_hole)
    
    return base_step1.union(base_step2).union(pyramid).union(shaft)


def make_rod() -> cq.Workplane:
    rod = cq.Workplane("XY").box(0.004, 0.004, 0.16)
    # Hole for the pivot shaft
    hole = cq.Workplane("XZ").workplane(offset=0).center(0, -0.04).circle(0.0015).extrude(0.01, both=True)
    rod = rod.cut(hole)
    return rod


def make_weight() -> cq.Workplane:
    weight = cq.Workplane("XZ").cylinder(0.01, 0.015)
    # Square hole for the rod
    hole = cq.Workplane("XY").box(0.004, 0.004, 0.04)
    weight = weight.cut(hole)
    return weight


def make_key() -> cq.Workplane:
    # Cylinder shaft along Y
    key = cq.Workplane("XZ").cylinder(0.015, 0.003)
    # Grip at the end of the shaft (+Y)
    grip = cq.Workplane("XZ").workplane(offset=0.0075).box(0.015, 0.01, 0.003)
    return key.union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="metronome")

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(make_case(), "case_mesh"),
        name="case_visual"
    )

    pendulum_rod = model.part("pendulum_rod")
    pendulum_rod.visual(
        mesh_from_cadquery(make_rod(), "rod_mesh"),
        origin=Origin(xyz=(0, 0, 0.04)),
        name="rod_visual"
    )

    tempo_weight = model.part("tempo_weight")
    tempo_weight.visual(
        mesh_from_cadquery(make_weight(), "weight_mesh"),
        name="weight_visual"
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        mesh_from_cadquery(make_key(), "key_mesh"),
        name="key_visual"
    )

    # Articulations
    model.articulation(
        "case_to_rod",
        ArticulationType.REVOLUTE,
        parent=case,
        child=pendulum_rod,
        origin=Origin(xyz=(0, 0, 0.22)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-0.5, upper=0.5),
    )

    model.articulation(
        "rod_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum_rod,
        child=tempo_weight,
        origin=Origin(xyz=(0, 0, 0.05)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.03, upper=0.05),
    )

    model.articulation(
        "case_to_key",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=winding_key,
        origin=Origin(xyz=(0, 0.041, 0.08)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    case = object_model.get_part("case")
    rod = object_model.get_part("pendulum_rod")
    weight = object_model.get_part("tempo_weight")
    key = object_model.get_part("winding_key")

    # Allow isolated parts
    ctx.allow_isolated_part(rod, reason="Supported by clearance fit on pivot shaft")
    ctx.allow_isolated_part(weight, reason="Supported by clearance fit on rod")

    # Allow overlap for captured features
    ctx.allow_overlap(case, rod, reason="Rod mounts onto the pivot shaft.")
    ctx.allow_overlap(rod, weight, reason="Weight slides over the rod.")
    ctx.allow_overlap(case, key, reason="Winding key inserts into the case hole.")

    # Exact checks
    ctx.expect_within(rod, case, axes="y", name="Rod stays centered in the slot")
    ctx.expect_within(weight, case, axes="y", name="Weight stays centered in Y")
    ctx.expect_within(key, case, axes="x", name="Key is centered in X")

    return ctx.report()


object_model = build_object_model()