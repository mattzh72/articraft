import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")
    
    base = model.part("base")
    
    # Wall plate
    base_plate_cq = (
        cq.Workplane("XY")
        .circle(0.042)
        .extrude(0.005)
        .edges(">Z").chamfer(0.001)
    )
    base.visual(
        mesh_from_cadquery(base_plate_cq, "base_plate"), 
        name="wall_plate", 
        material=Material(name="wall_plate_mat", color=(0.9, 0.9, 0.9))
    )
    
    # Core
    core_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .circle(0.038)
        .extrude(0.0245)
    )
    base.visual(
        mesh_from_cadquery(core_cq, "base_core"), 
        name="core", 
        material=Material(name="core_mat", color=(0.05, 0.05, 0.05))
    )
    
    # Screen bezel
    screen_bezel_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.0295)
        .circle(0.036).circle(0.034)
        .extrude(0.002)
        .edges(">Z").chamfer(0.0005)
    )
    base.visual(
        mesh_from_cadquery(screen_bezel_cq, "screen_bezel"), 
        name="screen_bezel", 
        material=Material(name="bezel_mat", color=(0.02, 0.02, 0.02))
    )
    
    # Screen glass
    screen_glass_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.0295)
        .circle(0.034)
        .extrude(0.0015)
    )
    base.visual(
        mesh_from_cadquery(screen_glass_cq, "screen_glass"), 
        name="screen_glass", 
        material=Material(name="glass_mat", color=(0.0, 0.0, 0.0))
    )
    
    # Sensor
    sensor_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.0295)
        .center(0, -0.028)
        .circle(0.002)
        .extrude(0.0016)
    )
    base.visual(
        mesh_from_cadquery(sensor_cq, "sensor"), 
        name="sensor", 
        material=Material(name="sensor_mat", color=(0.05, 0.0, 0.0))
    )
    
    # UI Element (Temperature arc/ring)
    ui_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.031)
        .circle(0.025).circle(0.023)
        .extrude(0.0002)
    )
    base.visual(
        mesh_from_cadquery(ui_cq, "ui_ring"),
        name="ui_ring",
        material=Material(name="ui_mat", color=(0.4, 0.8, 1.0))
    )
    
    # UI Element (Center readout block)
    ui_center_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.031)
        .rect(0.015, 0.01)
        .extrude(0.0002)
    )
    base.visual(
        mesh_from_cadquery(ui_center_cq, "ui_center"),
        name="ui_center",
        material=Material(name="ui_center_mat", color=(0.9, 0.9, 0.9))
    )
    
    dial = model.part("dial")
    
    # Dial ring
    dial_cq = (
        cq.Workplane("XY")
        .circle(0.042).circle(0.0385)
        .extrude(0.0265)
        .edges(">Z").chamfer(0.001)
        .edges("<Z").chamfer(0.0005)
    )
    
    indicator_cut = (
        cq.Workplane("XY")
        .workplane(offset=0.0265)
        .center(0, 0.040)
        .circle(0.001)
        .extrude(-0.002)
    )
    dial_cq = dial_cq.cut(indicator_cut)
    
    dial.visual(
        mesh_from_cadquery(dial_cq, "dial_ring"), 
        name="ring", 
        material=Material(name="ring_mat", color=(0.7, 0.7, 0.75))
    )
    
    # Dial indicator fill
    indicator_cq = (
        cq.Workplane("XY")
        .workplane(offset=0.0245)
        .center(0, 0.040)
        .circle(0.0009)
        .extrude(0.002)
    )
    dial.visual(
        mesh_from_cadquery(indicator_cq, "dial_indicator"),
        name="indicator",
        material=Material(name="indicator_mat", color=(0.1, 0.1, 0.1))
    )
    
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0, 0, 0.005)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    dial = object_model.get_part("dial")
    
    # Check that the core is within the dial ring footprint
    ctx.expect_within(
        base, dial,
        axes="xy",
        inner_elem="core",
        outer_elem="ring",
        margin=0.0,
        name="core_within_dial"
    )
    
    # Check the contact between the wall plate and the dial ring
    ctx.expect_contact(
        dial, base,
        elem_a="ring",
        elem_b="wall_plate",
        name="dial_touches_wall_plate"
    )
    
    with ctx.pose({"dial_turn": 1.0}):
        ctx.expect_within(
            base, dial,
            axes="xy",
            inner_elem="core",
            outer_elem="ring",
            margin=0.0,
            name="core_within_dial_rotated"
        )
    
    return ctx.report()

object_model = build_object_model()