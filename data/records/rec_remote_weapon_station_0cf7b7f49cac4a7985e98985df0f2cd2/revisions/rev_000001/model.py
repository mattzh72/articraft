from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def make_base_cyl():
    return cq.Workplane("XY").transformed(offset=(0, 0, 0.025)).cylinder(0.05, 0.4)

def make_base_pedestal():
    return cq.Workplane("XY").transformed(offset=(0, 0, 0.12)).cylinder(0.16, 0.3)

def make_housing_base():
    return cq.Workplane("XY").transformed(offset=(0, 0, 0.025)).cylinder(0.05, 0.3)

def make_housing_arms():
    left_arm = (
        cq.Workplane("XY")
        .transformed(offset=(-0.25, 0, 0.27))
        .box(0.1, 0.3, 0.46)
        .edges("|Z").chamfer(0.02)
        .edges(">Z").chamfer(0.02)
    )
    right_arm = (
        cq.Workplane("XY")
        .transformed(offset=(0.25, 0, 0.27))
        .box(0.1, 0.3, 0.46)
        .edges("|Z").chamfer(0.02)
        .edges(">Z").chamfer(0.02)
    )
    return left_arm.union(right_arm)

def make_housing_center():
    return (
        cq.Workplane("XY")
        .transformed(offset=(0, -0.18, 0.12))
        .box(0.4, 0.15, 0.16)
        .edges("|Z").chamfer(0.02)
    )

def make_housing_sensor():
    return (
        cq.Workplane("XY")
        .transformed(offset=(-0.34, 0.1, 0.3))
        .box(0.15, 0.2, 0.2)
        .edges().chamfer(0.02)
    )

def make_cradle_block():
    return (
        cq.Workplane("XY")
        .box(0.38, 0.3, 0.15)
        .edges("|Y").chamfer(0.02)
    )

def make_cradle_shafts():
    left_shaft = cq.Workplane("YZ", origin=(-0.24, 0, 0)).cylinder(0.12, 0.05)
    right_shaft = cq.Workplane("YZ", origin=(0.24, 0, 0)).cylinder(0.12, 0.05)
    return left_shaft.union(right_shaft)

def make_cradle_ammo():
    return (
        cq.Workplane("XY")
        .transformed(offset=(0.37, 0, 0))
        .box(0.16, 0.3, 0.3)
        .edges().chamfer(0.02)
    )

def make_weapon_receiver():
    return (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, 0.075))
        .box(0.15, 0.5, 0.15)
        .edges("|Y").chamfer(0.02)
    )

def make_weapon_barrel():
    barrel = cq.Workplane("XZ", origin=(0, 0.6, 0.075)).cylinder(0.8, 0.025)
    muzzle = cq.Workplane("XZ", origin=(0, 1.0, 0.075)).cylinder(0.1, 0.035)
    return barrel.union(muzzle)

def make_weapon_rail():
    return cq.Workplane("XY").transformed(offset=(0, -0.1, 0.155)).box(0.05, 0.3, 0.03)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station")
    
    base = model.part("base")
    base.visual(mesh_from_cadquery(make_base_cyl(), "base_cyl"), name="base_cyl", color=(0.2, 0.2, 0.2))
    base.visual(mesh_from_cadquery(make_base_pedestal(), "base_pedestal"), name="pedestal", color=(0.2, 0.2, 0.2))
    
    housing = model.part("housing")
    housing.visual(mesh_from_cadquery(make_housing_base(), "housing_base"), name="base_plate", color=(0.3, 0.35, 0.25))
    housing.visual(mesh_from_cadquery(make_housing_arms(), "housing_arms"), name="arms", color=(0.3, 0.35, 0.25))
    housing.visual(mesh_from_cadquery(make_housing_center(), "housing_center"), name="center", color=(0.3, 0.35, 0.25))
    housing.visual(mesh_from_cadquery(make_housing_sensor(), "housing_sensor"), name="sensor", color=(0.15, 0.15, 0.15))
    
    cradle = model.part("cradle")
    cradle.visual(mesh_from_cadquery(make_cradle_block(), "cradle_block"), name="block", color=(0.3, 0.35, 0.25))
    cradle.visual(mesh_from_cadquery(make_cradle_shafts(), "cradle_shafts"), name="shafts", color=(0.2, 0.2, 0.2))
    cradle.visual(mesh_from_cadquery(make_cradle_ammo(), "cradle_ammo"), name="ammo", color=(0.3, 0.35, 0.25))
    
    weapon = model.part("weapon")
    weapon.visual(mesh_from_cadquery(make_weapon_receiver(), "weapon_receiver"), name="receiver", color=(0.15, 0.15, 0.15))
    weapon.visual(mesh_from_cadquery(make_weapon_barrel(), "weapon_barrel"), name="barrel", color=(0.2, 0.2, 0.2))
    weapon.visual(mesh_from_cadquery(make_weapon_rail(), "weapon_rail"), name="rail", color=(0.1, 0.1, 0.1))
    
    model.articulation(
        "base_to_housing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0, 0, 0.2)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-3.14, upper=3.14)
    )
    
    model.articulation(
        "housing_to_cradle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cradle,
        origin=Origin(xyz=(0, 0, 0.4)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=-0.2, upper=1.0)
    )
    
    model.articulation(
        "cradle_to_weapon",
        ArticulationType.FIXED,
        parent=cradle,
        child=weapon,
        origin=Origin(xyz=(0, 0, 0.075))
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    cradle = object_model.get_part("cradle")
    weapon = object_model.get_part("weapon")
    
    ctx.allow_overlap(housing, cradle, reason="Cradle trunnion shafts pass through housing arms.")
    
    # Allow exact touching overlap between weapon receiver and cradle block
    ctx.allow_overlap(weapon, cradle, elem_a="receiver", elem_b="block", reason="Weapon receiver sits flush on cradle block.")
    
    ctx.expect_gap(housing, base, axis="z", max_gap=0.001, max_penetration=0.001, positive_elem="base_plate", negative_elem="pedestal")
    ctx.expect_overlap(housing, base, axes="xy", min_overlap=0.05, elem_a="base_plate", elem_b="pedestal")
    
    ctx.expect_gap(weapon, cradle, axis="z", max_gap=0.001, max_penetration=0.001, positive_elem="receiver", negative_elem="block")
    ctx.expect_overlap(weapon, cradle, axes="xy", min_overlap=0.05, elem_a="receiver", elem_b="block")
    
    pan = object_model.get_articulation("base_to_housing")
    pitch = object_model.get_articulation("housing_to_cradle")
    
    with ctx.pose({pan: 1.5, pitch: 1.0}):
        ctx.expect_overlap(housing, base, axes="xy", min_overlap=0.05, elem_a="base_plate", elem_b="pedestal")
        
    return ctx.report()

object_model = build_object_model()