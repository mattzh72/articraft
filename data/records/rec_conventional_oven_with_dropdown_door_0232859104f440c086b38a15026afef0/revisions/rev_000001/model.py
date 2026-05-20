import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def make_housing():
    # Main body
    res = cq.Workplane("XY").box(1.0, 1.0, 1.4).translate((0, 0, 0.9))
    
    # Chamber 1 (Bottom)
    c1 = cq.Workplane("XY").box(0.8, 0.9, 0.35).translate((0, 0.05, 0.575))
    # Chamber 2 (Top)
    c2 = cq.Workplane("XY").box(0.8, 0.9, 0.35).translate((0, 0.05, 1.125))
    res = res.cut(c1).cut(c2)
    
    # Rails for Chamber 1
    r1_l = cq.Workplane("XY").box(0.04, 0.8, 0.02).translate((-0.38, 0.05, 0.45))
    r1_r = cq.Workplane("XY").box(0.04, 0.8, 0.02).translate((0.38, 0.05, 0.45))
    
    # Rails for Chamber 2
    r2_l = cq.Workplane("XY").box(0.04, 0.8, 0.02).translate((-0.38, 0.05, 1.0))
    r2_r = cq.Workplane("XY").box(0.04, 0.8, 0.02).translate((0.38, 0.05, 1.0))
    
    res = res.union(r1_l).union(r1_r).union(r2_l).union(r2_r)
    
    # Legs
    for x in [-0.45, 0.45]:
        for y in [-0.45, 0.45]:
            leg = cq.Workplane("XY").box(0.05, 0.05, 0.2).translate((x, y, 0.1))
            res = res.union(leg)
            
    return res

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_deck_oven")

    # Housing
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(make_housing(), "housing_mesh"),
        name="housing_shell",
    )

    # --- Bottom Deck (Chamber 1) ---
    door_1 = model.part("door_1")
    door_1.visual(
        Box((0.84, 0.04, 0.39)),
        origin=Origin(xyz=(0.0, 0.02, 0.195)),
        name="door_1_panel",
    )
    door_1.visual(
        Box((0.60, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.06, 0.34)),
        name="door_1_handle",
    )
    model.articulation(
        "door_1_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door_1,
        origin=Origin(xyz=(0.0, 0.5, 0.4)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=math.pi / 2),
    )

    rack_1 = model.part("rack_1")
    rack_1.visual(
        Box((0.74, 0.8, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="rack_1_plate",
    )
    rack_1.visual(
        Box((0.74, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.39, 0.01)),
        name="rack_1_lip",
    )
    model.articulation(
        "rack_1_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=rack_1,
        origin=Origin(xyz=(0.0, 0.05, 0.47)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=0.6),
    )

    # --- Top Deck (Chamber 2) ---
    door_2 = model.part("door_2")
    door_2.visual(
        Box((0.84, 0.04, 0.39)),
        origin=Origin(xyz=(0.0, 0.02, 0.195)),
        name="door_2_panel",
    )
    door_2.visual(
        Box((0.60, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.06, 0.34)),
        name="door_2_handle",
    )
    model.articulation(
        "door_2_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door_2,
        origin=Origin(xyz=(0.0, 0.5, 0.95)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=math.pi / 2),
    )

    rack_2 = model.part("rack_2")
    rack_2.visual(
        Box((0.74, 0.8, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="rack_2_plate",
    )
    rack_2.visual(
        Box((0.74, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.39, 0.01)),
        name="rack_2_lip",
    )
    model.articulation(
        "rack_2_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=rack_2,
        origin=Origin(xyz=(0.0, 0.05, 1.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=0.6),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    housing = object_model.get_part("housing")
    door_1 = object_model.get_part("door_1")
    door_2 = object_model.get_part("door_2")
    rack_1 = object_model.get_part("rack_1")
    rack_2 = object_model.get_part("rack_2")

    # The racks rest on the rails which are part of the housing mesh.
    # We allow overlap because the rack resting exactly on the rail might trigger minor floating point overlap.
    ctx.allow_overlap(rack_1, housing, reason="Rack 1 rests on the housing rails.")
    ctx.allow_overlap(rack_2, housing, reason="Rack 2 rests on the housing rails.")

    # Doors are flush with the housing front.
    ctx.allow_overlap(door_1, housing, reason="Door 1 hinge is flush with the housing front face.")
    ctx.allow_overlap(door_2, housing, reason="Door 2 hinge is flush with the housing front face.")

    # Assertions for closed state
    ctx.expect_within(rack_1, housing, axes="x", name="Rack 1 centered in chamber")
    ctx.expect_within(rack_2, housing, axes="x", name="Rack 2 centered in chamber")

    with ctx.pose(door_1_hinge=math.pi/2, rack_1_slider=0.6):
        ctx.expect_gap(rack_1, door_1, axis="z", min_gap=0.01, name="Rack 1 clears open Door 1")
        
    with ctx.pose(door_2_hinge=math.pi/2, rack_2_slider=0.6):
        ctx.expect_gap(rack_2, door_2, axis="z", min_gap=0.01, name="Rack 2 clears open Door 2")

    return ctx.report()

object_model = build_object_model()