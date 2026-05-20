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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_fan")

    base_material = Material(name="metal", color=(0.2, 0.18, 0.15, 1.0))
    glass_material = Material(name="glass", color=(0.9, 0.9, 0.9, 0.8))
    wood_material = Material(name="wood", color=(0.25, 0.15, 0.08, 1.0))

    # --- Base (Static Mount, Downrod, Motor Top, Switch Housing, Light Dome) ---
    base = model.part("base")
    
    # Canopy at the ceiling
    canopy_cq = (
        cq.Workplane("XY")
        .circle(0.075)
        .extrude(0.06)
        .faces(">Z")
        .chamfer(0.01)
        .translate((0, 0, 0.37))
    )
    # Downrod connecting canopy to motor housing
    downrod_cq = (
        cq.Workplane("XY")
        .circle(0.0125)
        .extrude(0.28)
        .translate((0, 0, 0.10))
    )
    # Upper static half of the motor housing
    motor_top_cq = (
        cq.Workplane("XY")
        .circle(0.125)
        .extrude(0.12)
        .faces(">Z")
        .chamfer(0.02)
    )
    
    # Shaft extending through the rotor hub
    shaft_cq = (
        cq.Workplane("XY")
        .circle(0.02)
        .extrude(0.06)
        .translate((0, 0, -0.05))
    )
    
    # Switch housing below the rotor
    switch_housing_cq = (
        cq.Workplane("XY")
        .circle(0.11)
        .extrude(0.038)
        .faces("<Z")
        .chamfer(0.01)
        .translate((0, 0, -0.08))
    )
    
    # Light dome (glass) at the very bottom
    light_dome_cq = (
        cq.Workplane("XY")
        .circle(0.09)
        .extrude(0.055)
        .faces("<Z")
        .fillet(0.04)
        .translate((0, 0, -0.13))
    )
    
    base.visual(
        mesh_from_cadquery(canopy_cq.union(downrod_cq).union(motor_top_cq), "motor_top_mesh"),
        name="motor_top_vis",
        material=base_material,
    )
    base.visual(
        mesh_from_cadquery(shaft_cq, "shaft_mesh"),
        name="shaft_vis",
        material=base_material,
    )
    base.visual(
        mesh_from_cadquery(switch_housing_cq, "switch_housing_mesh"),
        name="switch_housing_vis",
        material=base_material,
    )
    base.visual(
        mesh_from_cadquery(light_dome_cq, "light_dome_mesh"),
        name="light_dome_vis",
        material=glass_material,
    )

    # --- Rotor (Spinning Hub, Blade Irons, Blades) ---
    rotor = model.part("rotor")
    
    # Spinning hub ring (solid, shaft will overlap inside)
    hub_cq = (
        cq.Workplane("XY")
        .circle(0.125)
        .extrude(0.038)
        .translate((0, 0, -0.04))
    )
    
    # Base iron and blade shapes
    iron_base = (
        cq.Workplane("XY")
        .box(0.12, 0.04, 0.008)
        .edges("|Z")
        .fillet(0.01)
        .translate((0.15, 0, -0.021))
        # Pitch the blade iron 15 degrees around its radial axis
        .rotate((0, 0, -0.021), (1, 0, -0.021), 15)
    )
    
    blade_base = (
        cq.Workplane("XY")
        .box(0.50, 0.15, 0.006)
        .edges("|Z")
        .fillet(0.04)
        .translate((0.44, 0, -0.021))
        # Pitch the blade 15 degrees around its radial axis
        .rotate((0, 0, -0.021), (1, 0, -0.021), 15)
    )
    
    # Polar array for 5 blades
    irons = iron_base
    blades = blade_base
    for i in range(1, 5):
        angle = i * 72.0
        irons = irons.union(iron_base.rotate((0, 0, 0), (0, 0, 1), angle))
        blades = blades.union(blade_base.rotate((0, 0, 0), (0, 0, 1), angle))
        
    rotor.visual(
        mesh_from_cadquery(hub_cq, "hub_mesh"),
        name="hub_vis",
        material=base_material,
    )
    rotor.visual(
        mesh_from_cadquery(irons, "irons_mesh"),
        name="irons_vis",
        material=base_material,
    )
    rotor.visual(
        mesh_from_cadquery(blades, "blades_mesh"),
        name="blades_vis",
        material=wood_material,
    )

    # --- Articulation ---
    # The rotor spins continuously around the Z axis
    model.articulation(
        name="fan_spin",
        articulation_type=ArticulationType.CONTINUOUS,
        parent=base,
        child=rotor,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    rotor = object_model.get_part("rotor")
    
    ctx.allow_overlap(base, rotor, elem_a="shaft_vis", elem_b="hub_vis", reason="The central shaft acts as the axle captured inside the spinning rotor hub.")
    
    # Verify the gap between the static base and the spinning rotor at the top
    ctx.expect_gap(base, rotor, axis="z", min_gap=0.001, max_gap=0.005, positive_elem="motor_top_vis", negative_elem="hub_vis", name="rotor_top_clearance")
    
    # Verify the gap between the spinning rotor and the static switch housing at the bottom
    ctx.expect_gap(rotor, base, axis="z", min_gap=0.001, max_gap=0.005, positive_elem="hub_vis", negative_elem="switch_housing_vis", name="rotor_bottom_clearance")
    
    # Verify the shaft passes through the hub
    ctx.expect_within(base, rotor, axes="xy", inner_elem="shaft_vis", outer_elem="hub_vis", margin=0.0, name="shaft_inside_hub")

    return ctx.report()


object_model = build_object_model()