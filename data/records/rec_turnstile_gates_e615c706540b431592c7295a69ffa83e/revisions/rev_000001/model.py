import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def make_cabinet_body():
    return cq.Workplane("XY").box(0.2, 1.2, 0.98).edges("|Z").fillet(0.05)

def make_cabinet_top():
    return cq.Workplane("XY").box(0.22, 1.22, 0.03).edges("|Z").fillet(0.06)

def make_cabinet_plinth():
    return cq.Workplane("XY").box(0.21, 1.21, 0.05).edges("|Z").fillet(0.055)

def make_glass_gate():
    return cq.Workplane("XY").box(0.21, 0.02, 0.55).edges().fillet(0.009)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turnstile_gate")

    mat_body = Material(name="body", color=(0.85, 0.85, 0.87))
    mat_top = Material(name="top", color=(0.15, 0.15, 0.15))
    mat_glass = Material(name="glass", color=(0.4, 0.6, 0.8, 0.6))
    mat_hub = Material(name="hub", color=(0.3, 0.3, 0.3))
    mat_base = Material(name="base", color=(0.2, 0.2, 0.2))
    mat_reader = Material(name="reader", color=(0.05, 0.05, 0.05))
    mat_screen = Material(name="screen", color=(0.1, 0.8, 0.4))
    mat_screen_rear = Material(name="screen_rear", color=(0.8, 0.1, 0.1))

    base = model.part("base")
    base.visual(
        Box((1.2, 1.4, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
        material=mat_base,
    )

    # Left cabinet
    left_cab = model.part("left_cabinet")
    left_cab.visual(
        mesh_from_cadquery(make_cabinet_body(), "cabinet_body"),
        origin=Origin(xyz=(-0.4, 0.0, 0.51)),
        name="left_body",
        material=mat_body,
    )
    left_cab.visual(
        mesh_from_cadquery(make_cabinet_top(), "cabinet_top"),
        origin=Origin(xyz=(-0.4, 0.0, 1.015)),
        name="left_top",
        material=mat_top,
    )
    left_cab.visual(
        mesh_from_cadquery(make_cabinet_plinth(), "cabinet_plinth"),
        origin=Origin(xyz=(-0.4, 0.0, 0.045)),
        name="left_plinth",
        material=mat_reader,
    )
    left_cab.visual(
        Box((0.1, 0.15, 0.01)),
        origin=Origin(xyz=(-0.4, -0.3, 1.035)),
        name="left_reader",
        material=mat_reader,
    )
    left_cab.visual(
        Box((0.08, 0.1, 0.005)),
        origin=Origin(xyz=(-0.4, -0.3, 1.04)),
        name="left_screen",
        material=mat_screen,
    )
    left_cab.visual(
        Box((0.1, 0.01, 0.4)),
        origin=Origin(xyz=(-0.4, -0.605, 0.6)),
        name="left_indicator_front",
        material=mat_screen,
    )
    left_cab.visual(
        Box((0.1, 0.01, 0.4)),
        origin=Origin(xyz=(-0.4, 0.605, 0.6)),
        name="left_indicator_rear",
        material=mat_screen_rear,
    )
    left_cab.visual(
        Cylinder(radius=0.02, height=0.04),
        origin=Origin(xyz=(-0.28, -0.2, 0.6), rpy=(0, 1.5708, 0)),
        name="left_mount",
        material=mat_body,
    )

    # Right cabinet
    right_cab = model.part("right_cabinet")
    right_cab.visual(
        mesh_from_cadquery(make_cabinet_body(), "cabinet_body"),
        origin=Origin(xyz=(0.4, 0.0, 0.51)),
        name="right_body",
        material=mat_body,
    )
    right_cab.visual(
        mesh_from_cadquery(make_cabinet_top(), "cabinet_top"),
        origin=Origin(xyz=(0.4, 0.0, 1.015)),
        name="right_top",
        material=mat_top,
    )
    right_cab.visual(
        mesh_from_cadquery(make_cabinet_plinth(), "cabinet_plinth"),
        origin=Origin(xyz=(0.4, 0.0, 0.045)),
        name="right_plinth",
        material=mat_reader,
    )
    right_cab.visual(
        Box((0.1, 0.15, 0.01)),
        origin=Origin(xyz=(0.4, -0.3, 1.035)),
        name="right_reader",
        material=mat_reader,
    )
    right_cab.visual(
        Box((0.08, 0.1, 0.005)),
        origin=Origin(xyz=(0.4, -0.3, 1.04)),
        name="right_screen",
        material=mat_screen,
    )
    right_cab.visual(
        Box((0.1, 0.01, 0.4)),
        origin=Origin(xyz=(0.4, -0.605, 0.6)),
        name="right_indicator_front",
        material=mat_screen,
    )
    right_cab.visual(
        Box((0.1, 0.01, 0.4)),
        origin=Origin(xyz=(0.4, 0.605, 0.6)),
        name="right_indicator_rear",
        material=mat_screen_rear,
    )
    right_cab.visual(
        Cylinder(radius=0.02, height=0.04),
        origin=Origin(xyz=(0.28, -0.2, 0.6), rpy=(0, 1.5708, 0)),
        name="right_mount",
        material=mat_body,
    )

    # Left gate
    left_gate = model.part("left_gate")
    left_gate.visual(
        Cylinder(radius=0.03, height=0.55),
        origin=Origin(xyz=(0, 0, 0)),
        name="left_hub",
        material=mat_hub,
    )
    left_gate.visual(
        mesh_from_cadquery(make_glass_gate(), "glass_gate"),
        origin=Origin(xyz=(0.135, 0, 0)),
        name="left_glass",
        material=mat_glass,
    )
    
    model.articulation(
        "left_swing",
        ArticulationType.REVOLUTE,
        parent=left_cab,
        child=left_gate,
        origin=Origin(xyz=(-0.26, -0.2, 0.6)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    # Right gate
    right_gate = model.part("right_gate")
    right_gate.visual(
        Cylinder(radius=0.03, height=0.55),
        origin=Origin(xyz=(0, 0, 0)),
        name="right_hub",
        material=mat_hub,
    )
    right_gate.visual(
        mesh_from_cadquery(make_glass_gate(), "glass_gate"),
        origin=Origin(xyz=(-0.135, 0, 0)),
        name="right_glass",
        material=mat_glass,
    )
    
    model.articulation(
        "right_swing",
        ArticulationType.REVOLUTE,
        parent=right_cab,
        child=right_gate,
        origin=Origin(xyz=(0.26, -0.2, 0.6)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    # Attach cabinets to base
    model.articulation(
        "base_to_left_cab",
        ArticulationType.FIXED,
        parent=base,
        child=left_cab,
        origin=Origin(xyz=(0, 0, 0)),
    )
    model.articulation(
        "base_to_right_cab",
        ArticulationType.FIXED,
        parent=base,
        child=right_cab,
        origin=Origin(xyz=(0, 0, 0)),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    left_cab = object_model.get_part("left_cabinet")
    right_cab = object_model.get_part("right_cabinet")
    left_gate = object_model.get_part("left_gate")
    right_gate = object_model.get_part("right_gate")
    base = object_model.get_part("base")

    ctx.allow_overlap(
        left_cab, left_gate,
        elem_a="left_mount", elem_b="left_hub",
        reason="The rotating hub is mounted on and overlaps the fixed cabinet shaft."
    )
    ctx.allow_overlap(
        right_cab, right_gate,
        elem_a="right_mount", elem_b="right_hub",
        reason="The rotating hub is mounted on and overlaps the fixed cabinet shaft."
    )

    ctx.expect_contact(left_cab, base, name="left cabinet mounted on base")
    ctx.expect_contact(right_cab, base, name="right cabinet mounted on base")

    ctx.expect_gap(
        right_gate, left_gate,
        axis="x",
        min_gap=0.03, max_gap=0.05,
        positive_elem="right_glass", negative_elem="left_glass",
        name="gap between closed gates"
    )

    with ctx.pose(left_swing=1.57, right_swing=1.57):
        ctx.expect_gap(
            right_gate, left_gate,
            axis="x",
            min_gap=0.45, max_gap=0.55,
            positive_elem="right_glass", negative_elem="left_glass",
            name="passage clears when open"
        )

    return ctx.report()

object_model = build_object_model()
