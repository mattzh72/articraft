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

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_roll_module")

    # 1. Support Frame (Root)
    # Base plate at x=-0.12, arms extending to x=0.01 at y=+/-0.12
    pin_hole = cq.Workplane("XZ").cylinder(0.30, 0.01) # along Y axis
    support_cq = (
        cq.Workplane("XY")
        .box(0.02, 0.26, 0.10).translate((-0.12, 0, 0))
        .union(cq.Workplane("XY").box(0.13, 0.02, 0.04).translate((-0.055, -0.12, 0)))
        .union(cq.Workplane("XY").box(0.13, 0.02, 0.04).translate((-0.055, 0.12, 0)))
    ) - pin_hole

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(support_cq, "support_frame_mesh"),
        origin=Origin(),
        name="support_frame_visual",
    )

    # 2. Outer Ring (Pitch - rotates around Y)
    inner_pin_hole = cq.Workplane("YZ").cylinder(0.20, 0.01) # along X axis
    outer_ring_inner_hole = cq.Workplane("XY").cylinder(0.05, 0.07)
    outer_ring_cq = (
        cq.Workplane("XY").cylinder(0.04, 0.09)
        - outer_ring_inner_hole
    )
    outer_pins = cq.Workplane("XZ").cylinder(0.24, 0.01) # along Y axis
    outer_ring_cq = outer_ring_cq.union(outer_pins) - outer_ring_inner_hole - inner_pin_hole

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(outer_ring_cq, "outer_ring_mesh"),
        origin=Origin(),
        name="outer_ring_visual",
    )

    # 3. Inner Cradle (Roll - rotates around X)
    inner_cradle_cq = (
        cq.Workplane("XY").cylinder(0.03, 0.06)
        - cq.Workplane("XY").cylinder(0.04, 0.04)
    )
    inner_pins = cq.Workplane("YZ").cylinder(0.15, 0.01) # along X axis
    center_block = cq.Workplane("XY").box(0.04, 0.04, 0.04)
    lens_hole = cq.Workplane("XY").cylinder(0.05, 0.015)
    inner_cradle_cq = inner_cradle_cq.union(inner_pins).union(center_block - lens_hole)

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(inner_cradle_cq, "inner_cradle_mesh"),
        origin=Origin(),
        name="inner_cradle_visual",
    )

    # Articulations
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=outer_ring,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-3.14, upper=3.14),
    )

    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_cradle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support_frame")
    outer = object_model.get_part("outer_ring")
    inner = object_model.get_part("inner_cradle")

    ctx.allow_overlap(outer, support, reason="Pins are modeled as exact fit in holes")
    ctx.allow_overlap(inner, outer, reason="Pins are modeled as exact fit in holes")

    ctx.expect_overlap(outer, support, axes="y", min_overlap=0.01, name="outer ring pins captured in support frame")
    ctx.expect_overlap(inner, outer, axes="x", min_overlap=0.01, name="inner cradle pins captured in outer ring")

    pitch = object_model.get_articulation("pitch_joint")
    roll = object_model.get_articulation("roll_joint")

    with ctx.pose({pitch: 1.57, roll: 1.57}):
        ctx.expect_overlap(outer, support, axes="y", min_overlap=0.01, name="outer ring pins still captured at 90 deg")
        ctx.expect_overlap(inner, outer, axes="x", min_overlap=0.01, name="inner cradle pins still captured at 90 deg")

    return ctx.report()


object_model = build_object_model()