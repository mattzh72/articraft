import math
import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    BlowerWheelGeometry,
    VentGrilleGeometry,
    VentGrilleSleeve,
    SlotPatternPanelGeometry,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    # Dimensions
    pedestal_radius = 0.14
    pedestal_height = 0.04
    
    column_width = 0.16
    column_depth = 0.16
    column_height = 0.8
    wall = 0.004

    # 1. Pedestal (root)
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=pedestal_radius, height=pedestal_height),
        origin=Origin(xyz=(0, 0, pedestal_height / 2)),
        name="base"
    )

    # 2. Column
    column_cq = (
        cq.Workplane("XY")
        .box(column_width, column_depth, column_height)
        .translate((0, 0, column_height / 2))
        .edges("|Z").fillet(0.04)
        .edges(">Z").chamfer(0.02)
        .faces("<Z").shell(-wall)
    )
    # Cut front and rear grille openings
    column_cq = column_cq.faces(">Y").workplane().rect(0.10, 0.65).cutThruAll()
    # Cut top recesses and shaft holes
    top_wp = column_cq.faces(">Z").workplane()
    column_cq = top_wp.pushPoints([(-0.04, 0), (0.04, 0)]).circle(0.025).cutBlind(-0.005)
    column_cq = column_cq.faces(">Z").workplane().pushPoints([(-0.04, 0), (0.04, 0)]).circle(0.004).cutBlind(-0.02)

    column = model.part("column")
    column.visual(
        mesh_from_cadquery(column_cq, "column_housing"),
        origin=Origin(xyz=(0, 0, 0)),
        name="housing"
    )

    model.articulation(
        "oscillation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=column,
        origin=Origin(xyz=(0, 0, pedestal_height)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-math.pi/3, upper=math.pi/3)
    )

    # 3. Front Grille
    front_grille_geom = VentGrilleGeometry(
        (0.10, 0.65),
        frame=0.006,
        face_thickness=0.004,
        slat_pitch=0.012,
        slat_width=0.01,
        slat_angle_deg=0.0,
        sleeve=VentGrilleSleeve("none"),
        center=True
    )
    front_grille = model.part("front_grille")
    front_grille.visual(mesh_from_geometry(front_grille_geom, "front_grille_mesh"), name="grille")
    
    model.articulation(
        "column_to_front_grille",
        ArticulationType.FIXED,
        parent=column,
        child=front_grille,
        origin=Origin(xyz=(0, column_depth / 2 - wall / 2, column_height / 2), rpy=(-math.pi/2, 0, 0))
    )

    # 4. Rear Grille
    rear_grille_geom = SlotPatternPanelGeometry(
        (0.10, 0.65),
        thickness=0.004,
        slot_size=(0.08, 0.006),
        pitch=(0.085, 0.012),
        frame=0.006,
        center=True
    )
    rear_grille = model.part("rear_grille")
    rear_grille.visual(mesh_from_geometry(rear_grille_geom, "rear_grille_mesh"), name="grille")
    
    model.articulation(
        "column_to_rear_grille",
        ArticulationType.FIXED,
        parent=column,
        child=rear_grille,
        origin=Origin(xyz=(0, -column_depth / 2 + wall / 2, column_height / 2), rpy=(math.pi/2, 0, 0))
    )

    # 5. Blower Rotor
    blower_geom = BlowerWheelGeometry(
        outer_radius=0.045,
        inner_radius=0.035,
        width=0.6,
        blade_count=36,
        blade_thickness=0.002,
        blade_sweep_deg=25.0,
        center=True
    )
    blower = model.part("blower")
    blower.visual(mesh_from_geometry(blower_geom, "blower_mesh"), name="rotor")
    
    model.articulation(
        "blower_spin",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=blower,
        origin=Origin(xyz=(0, 0, column_height / 2)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=0.5, velocity=10.0)
    )

    # 6. Controls
    control_cq = (
        cq.Workplane("XY")
        .cylinder(0.01, 0.02)
        .faces("<Z").workplane().circle(0.0035).extrude(0.014)
        .faces(">Z").workplane().rect(0.004, 0.04).cutBlind(-0.002)
    )
    
    for i, x_pos in enumerate([-0.04, 0.04]):
        ctrl = model.part(f"control_{i}")
        ctrl.visual(mesh_from_cadquery(control_cq, f"control_mesh_{i}"), name="knob")
        
        model.articulation(
            f"control_{i}_turn",
            ArticulationType.REVOLUTE,
            parent=column,
            child=ctrl,
            origin=Origin(xyz=(x_pos, 0, column_height - 0.005 + 0.005)), # 0.8 - 0.005 (recess) + 0.005 (half height) = 0.800
            axis=(0, 0, 1),
            motion_limits=MotionLimits(effort=0.1, velocity=5.0, lower=-math.pi/2, upper=math.pi/2)
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Assertions
    ctx.expect_contact(object_model.get_part("column"), object_model.get_part("pedestal"), name="column sits on pedestal")
    ctx.expect_within(object_model.get_part("blower"), object_model.get_part("column"), name="blower is inside column")
    ctx.expect_contact(object_model.get_part("front_grille"), object_model.get_part("column"), name="front grille is mounted on column")
    ctx.expect_contact(object_model.get_part("rear_grille"), object_model.get_part("column"), name="rear grille is mounted on column")
    
    for i in range(2):
        ctrl = object_model.get_part(f"control_{i}")
        ctx.expect_contact(ctrl, object_model.get_part("column"), name=f"control_{i} sits in top cap recess")
    
    # Overlap allowances
    ctx.allow_isolated_part("blower", reason="The blower rotor spins freely inside the column housing without a modeled shaft.")
    ctx.allow_overlap("column", "control_0", reason="The control knob is captured inside the top cap recess.")
    ctx.allow_overlap("column", "control_1", reason="The control knob is captured inside the top cap recess.")
    
    # The grilles are placed exactly at the hole boundary, small overlap is expected due to floating point or intentional flush mounting.
    ctx.allow_overlap("front_grille", "column", reason="Front grille sits flush in the column opening.")
    ctx.allow_overlap("rear_grille", "column", reason="Rear grille sits flush in the column opening.")
    
    return ctx.report()


object_model = build_object_model()
