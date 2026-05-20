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

    mat_metal = Material(name="metal", rgba=(0.3, 0.3, 0.35, 1.0))
    mat_blades = Material(name="blades", rgba=(0.3, 0.2, 0.1, 1.0))
    mat_housing = Material(name="housing", rgba=(0.2, 0.2, 0.2, 1.0))

    # 1. Canopy (Root)
    # Revolved dome-like shape from Z=0 down to Z=-0.06, hollowed out
    canopy_cq = (
        cq.Workplane("XY")
        .cylinder(0.06, 0.08)
        .translate((0, 0, -0.03))
    )
    canopy_inside = (
        cq.Workplane("XY")
        .cylinder(0.055, 0.075)
        .translate((0, 0, -0.0325))
    )
    canopy_cq = canopy_cq.cut(canopy_inside)
    
    canopy = model.part("canopy")
    canopy.visual(
        mesh_from_cadquery(canopy_cq, "canopy_geom"),
        material=mat_metal,
        name="canopy_visual",
    )

    # 2. Downrod
    # Thin cylinder from Z=0 down to Z=-0.4
    downrod_length = 0.4
    downrod_cq = (
        cq.Workplane("XY")
        .cylinder(downrod_length, 0.012)
        .translate((0, 0, -downrod_length / 2))
    )
    downrod = model.part("downrod")
    downrod.visual(
        mesh_from_cadquery(downrod_cq, "downrod_geom"),
        material=mat_metal,
        name="downrod_visual",
    )

    # Fixed mount from canopy to downrod (swivel joint, but fixed)
    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0, 0, 0)),
    )

    # 3. Motor Housing
    # Sealed hollow housing from Z=0 down to Z=-0.12
    housing_height = 0.12
    housing_radius = 0.12
    housing_cq = (
        cq.Workplane("XY")
        .cylinder(housing_height, housing_radius)
        .translate((0, 0, -housing_height / 2))
        .faces(">Z or <Z").fillet(0.01)
    )
    inside = (
        cq.Workplane("XY")
        .cylinder(housing_height - 0.01, housing_radius - 0.01)
        .translate((0, 0, -housing_height / 2))
    )
    shaft_hole = (
        cq.Workplane("XY")
        .cylinder(0.02, 0.01)
        .translate((0, 0, -housing_height))
    )
    housing_cq = housing_cq.cut(inside).cut(shaft_hole)
    
    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        mesh_from_cadquery(housing_cq, "housing_geom"),
        material=mat_housing,
        name="housing_visual",
    )

    # Fixed mount from downrod to motor housing
    model.articulation(
        "downrod_to_motor",
        ArticulationType.FIXED,
        parent=downrod,
        child=motor_housing,
        origin=Origin(xyz=(0, 0, -downrod_length)),
    )

    # 4. Blade Assembly
    # Hub from Z=0 down to Z=-0.04, with 2 wide blade arms and blades.
    hub_radius = 0.08
    hub_height = 0.04
    arm_length = 0.12
    arm_width = 0.03
    arm_thickness = 0.006
    blade_length = 0.5
    blade_width = 0.15
    blade_thickness = 0.005

    blade_assembly_cq = (
        cq.Workplane("XY")
        .cylinder(hub_height, hub_radius)
        .translate((0, 0, -hub_height / 2))
    )
    
    shaft = (
        cq.Workplane("XY")
        .cylinder(0.02, 0.01)
        .translate((0, 0, 0.01))
    )
    blade_assembly_cq = blade_assembly_cq.union(shaft)

    for i in range(2):
        angle = i * 180
        # Arm
        arm = (
            cq.Workplane("XY")
            .box(arm_length, arm_width, arm_thickness)
            .translate((hub_radius + arm_length / 2, 0, -hub_height / 2))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        blade_assembly_cq = blade_assembly_cq.union(arm)

        # Blade
        blade = (
            cq.Workplane("XY")
            .box(blade_length, blade_width, blade_thickness)
            .rotate((0, 0, 0), (1, 0, 0), 12) # Pitch by 12 degrees
            .translate((hub_radius + arm_length + blade_length / 2, 0, -hub_height / 2))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        blade_assembly_cq = blade_assembly_cq.union(blade)

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        mesh_from_cadquery(blade_assembly_cq, "blade_assembly_geom"),
        material=mat_blades,
        name="blade_assembly_visual",
    )

    # Continuous joint for rotation
    model.articulation(
        "motor_to_blades",
        ArticulationType.CONTINUOUS,
        parent=motor_housing,
        child=blade_assembly,
        origin=Origin(xyz=(0, 0, -housing_height - 0.005)), # 5mm gap
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    motor_housing = object_model.get_part("motor_housing")
    blade_assembly = object_model.get_part("blade_assembly")

    ctx.allow_overlap(
        canopy,
        downrod,
        reason="downrod is mounted into the canopy ceiling mount"
    )
    ctx.allow_overlap(
        blade_assembly,
        motor_housing,
        reason="shaft is captured inside the motor housing"
    )

    ctx.expect_overlap(canopy, downrod, axes="z", name="canopy connects to downrod")
    ctx.expect_contact(downrod, motor_housing, name="downrod connects to motor housing")
    ctx.expect_gap(motor_housing, blade_assembly, axis="z", max_penetration=0.02)

    return ctx.report()


object_model = build_object_model()