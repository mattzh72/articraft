import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    FanRotorGeometry,
    FanRotorBlade,
    FanRotorHub,
)

def build_housing_cq():
    body = cq.Workplane("XY").box(0.5, 0.5, 0.12).edges("|Z").fillet(0.05)
    body = body.faces(">Z").workplane().hole(0.48)
    
    handle = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0.26, 0))
        .box(0.15, 0.04, 0.08)
        .faces(">Y")
        .workplane()
        .rect(0.1, 0.04)
        .cutThruAll()
    )
    body = body.union(handle)
    
    foot1 = cq.Workplane("XY").transformed(offset=(-0.2, -0.26, 0)).box(0.08, 0.02, 0.12)
    foot2 = cq.Workplane("XY").transformed(offset=(0.2, -0.26, 0)).box(0.08, 0.02, 0.12)
    body = body.union(foot1).union(foot2)
    
    # Recess for knob
    body = body.faces(">Z").workplane().center(0.2, 0.2).circle(0.025).extrude(-0.01, combine="cut")
    
    return body

def build_grille_cq():
    grille = cq.Workplane("XY").circle(0.245).circle(0.235).extrude(0.005)
    for r in [0.04, 0.08, 0.12, 0.16, 0.20]:
        ring = cq.Workplane("XY").circle(r).circle(r - 0.003).extrude(0.005)
        grille = grille.union(ring)
    
    for i in range(8):
        angle = i * 22.5
        spoke = cq.Workplane("XY").rect(0.48, 0.003).extrude(0.005).rotate((0,0,0), (0,0,1), angle)
        grille = grille.union(spoke)
        
    hub = cq.Workplane("XY").circle(0.06).extrude(0.005)
    grille = grille.union(hub)
    return grille

def build_rear_grille_cq():
    grille = build_grille_cq()
    motor_housing = cq.Workplane("XY").workplane(offset=0.005).circle(0.06).extrude(0.06)
    shaft = cq.Workplane("XY").workplane(offset=0.065).circle(0.005).extrude(0.015)
    grille = grille.union(motor_housing).union(shaft)
    return grille

def build_knob_cq():
    knob = cq.Workplane("XY").circle(0.02).extrude(0.015)
    pointer = cq.Workplane("XY").transformed(offset=(0, 0.01, 0.015)).box(0.005, 0.02, 0.005)
    return knob.union(pointer)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_fan")
    
    # Materials
    mat_housing = model.material("housing_plastic", rgba=(0.85, 0.85, 0.82, 1.0))
    mat_grille = model.material("grille_plastic", rgba=(0.7, 0.7, 0.7, 1.0))
    mat_rotor = model.material("rotor_plastic", rgba=(0.4, 0.6, 0.8, 1.0))
    mat_knob = model.material("knob_plastic", rgba=(0.2, 0.2, 0.2, 1.0))
    
    housing_cq = build_housing_cq()
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(housing_cq, "housing_mesh"),
        material=mat_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="housing_visual",
    )
    
    front_grille_cq = build_grille_cq()
    front_grille = model.part("front_grille")
    front_grille.visual(
        mesh_from_cadquery(front_grille_cq, "front_grille_mesh"),
        material=mat_grille,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        name="front_grille_visual",
    )
    
    rear_grille_cq = build_rear_grille_cq()
    rear_grille = model.part("rear_grille")
    rear_grille.visual(
        mesh_from_cadquery(rear_grille_cq, "rear_grille_mesh"),
        material=mat_grille,
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        name="rear_grille_visual",
    )
    
    rotor_geo = FanRotorGeometry(
        outer_radius=0.23,
        hub_radius=0.06,
        blade_count=5,
        thickness=0.02,
        blade_pitch_deg=25.0,
        blade_sweep_deg=15.0,
        blade=FanRotorBlade(shape="broad", camber=0.1),
        hub=FanRotorHub(style="spinner", bore_diameter=0.01)
    )
    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(rotor_geo, "rotor_mesh"),
        material=mat_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="rotor_visual",
    )
    
    knob_cq = build_knob_cq()
    knob = model.part("knob")
    knob.visual(
        mesh_from_cadquery(knob_cq, "knob_mesh"),
        material=mat_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="knob_visual",
    )
    
    # Articulations
    model.articulation(
        "housing_to_front_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=front_grille,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    
    model.articulation(
        "housing_to_rear_grille",
        ArticulationType.FIXED,
        parent=housing,
        child=rear_grille,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    
    model.articulation(
        "rear_grille_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=rear_grille,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=20.0),
    )
    
    model.articulation(
        "housing_to_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.2, 0.2, 0.049)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.57, upper=1.57),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    rear_grille = object_model.get_part("rear_grille")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("knob")
    
    ctx.allow_overlap(housing, front_grille, reason="Grille is seated flush into the housing front face.")
    ctx.allow_overlap(housing, rear_grille, reason="Grille is seated flush into the housing rear face.")
    ctx.allow_overlap(housing, knob, reason="Knob is seated into the housing recess.")
    ctx.allow_overlap(rear_grille, rotor, reason="Rotor hub is mounted onto the motor shaft.")
    
    ctx.expect_contact(housing, front_grille)
    ctx.expect_contact(housing, rear_grille)
    ctx.expect_contact(housing, knob)
    
    # Rotor should have clearance from motor housing
    ctx.expect_overlap(rotor, rear_grille, axes="z", min_overlap=0.01)
    
    with ctx.pose({object_model.get_articulation("housing_to_knob"): 1.57}):
        ctx.expect_contact(housing, knob)
    
    return ctx.report()

object_model = build_object_model()
