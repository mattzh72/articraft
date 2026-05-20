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
    model = ArticulatedObject(name="record_turntable")

    # Plinth (Base)
    plinth = model.part("plinth")
    plinth.visual(
        Box((0.45, 0.35, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="plinth_body",
    )

    # Central Support for Platter
    platter_support = model.part("platter_support")
    platter_support.visual(
        Cylinder(radius=0.02, height=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        name="support_body",
    )
    model.articulation(
        "plinth_to_support",
        ArticulationType.FIXED,
        parent=plinth,
        child=platter_support,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Platter
    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.15, height=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        name="platter_body",
    )
    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=platter_support,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )

    # Tonearm Base
    tonearm_base = model.part("tonearm_base")
    tonearm_base.visual(
        Cylinder(radius=0.03, height=0.04),
        origin=Origin(xyz=(0.16, -0.12, 0.07)),
        name="base_body",
    )
    model.articulation(
        "plinth_to_tonearm_base",
        ArticulationType.FIXED,
        parent=plinth,
        child=tonearm_base,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Tonearm
    tonearm = model.part("tonearm")
    # Arm tube from y=-0.06 to y=0.22
    tonearm.visual(
        Box((0.01, 0.28, 0.01)),
        origin=Origin(xyz=(0.0, 0.08, 0.005)),
        name="arm_tube",
    )
    # Pivot cap
    tonearm.visual(
        Cylinder(radius=0.015, height=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        name="pivot_cap",
    )
    # Counterweight
    tonearm.visual(
        Cylinder(radius=0.015, height=0.02),
        origin=Origin(xyz=(0.0, -0.05, 0.005), rpy=(math.pi / 2, 0.0, 0.0)),
        name="counterweight",
    )
    # Headshell
    tonearm.visual(
        Box((0.02, 0.04, 0.01)),
        origin=Origin(xyz=(0.0, 0.22, 0.005)),
        name="headshell",
    )
    
    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent=tonearm_base,
        child=tonearm,
        origin=Origin(xyz=(0.16, -0.12, 0.09)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=1.0, lower=0.0, upper=0.8),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    plinth = object_model.get_part("plinth")
    platter_support = object_model.get_part("platter_support")
    platter = object_model.get_part("platter")
    tonearm_base = object_model.get_part("tonearm_base")
    tonearm = object_model.get_part("tonearm")

    ctx.expect_contact(platter_support, plinth, elem_a="support_body", elem_b="plinth_body")
    ctx.expect_contact(platter, platter_support, elem_a="platter_body", elem_b="support_body")
    ctx.expect_contact(tonearm_base, plinth, elem_a="base_body", elem_b="plinth_body")
    ctx.expect_contact(tonearm, tonearm_base, elem_a="pivot_cap", elem_b="base_body")

    return ctx.report()

object_model = build_object_model()