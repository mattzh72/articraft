from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    FanRotorGeometry,
    FanRotorBlade,
    FanRotorHub,
    mesh_from_geometry,
    mesh_from_cadquery,
)
import cadquery as cq

def build_fan_head():
    # Motor housing: Z from -0.05 to 0.05
    motor = cq.Workplane("XY").cylinder(0.10, 0.04)
    
    # Back rim: Z from 0.04 to 0.05
    rim_back = cq.Workplane("XY").circle(0.11).circle(0.105).extrude(0.01).translate((0, 0, 0.04))
    
    # Front rim: Z from 0.07 to 0.08
    rim_front = cq.Workplane("XY").circle(0.11).circle(0.105).extrude(0.01).translate((0, 0, 0.07))
    
    # Front cap: Z from 0.073 to 0.077
    front_cap = cq.Workplane("XY").cylinder(0.004, 0.02).translate((0, 0, 0.075))
    
    # Spokes
    spokes = cq.Workplane("XY").box(0.22, 0.002, 0.002)
    for i in range(1, 4):
        spokes = spokes.union(cq.Workplane("XY").box(0.22, 0.002, 0.002).rotate((0,0,0), (0,0,1), i*45))
        
    # Back spokes: Z from 0.044 to 0.046
    spokes_back = spokes.translate((0, 0, 0.045))
    
    # Front spokes: Z from 0.074 to 0.076
    spokes_front = spokes.translate((0, 0, 0.075))
    
    # Axial struts: Z from 0.04 to 0.08
    struts = cq.Workplane("XY").workplane(offset=0.04).polarArray(0.1075, 0, 360, 8).circle(0.002).extrude(0.04)
    
    # Trunnion: X from -0.14 to 0.14, at Z=0
    trunnion = cq.Workplane("YZ").cylinder(0.28, 0.008)
    
    # Shaft: Z from 0.05 to 0.06
    shaft = cq.Workplane("XY").cylinder(0.01, 0.004).translate((0, 0, 0.055))
    
    return motor.union(rim_back).union(rim_front).union(front_cap).union(spokes_back).union(spokes_front).union(struts).union(trunnion).union(shaft)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_fan")

    base = model.part("base")
    base.visual(Cylinder(0.12, 0.02), origin=Origin(xyz=(0, 0, 0.01)), name="base_pad")
    base.visual(Cylinder(0.02, 0.08), origin=Origin(xyz=(0, 0, 0.06)), name="column")
    
    yoke_geom = TrunnionYokeGeometry(
        overall_size=(0.28, 0.06, 0.18),
        span_width=0.24,
        trunnion_diameter=0.016,
        trunnion_center_z=0.15,
        base_thickness=0.02,
        center=False
    )
    base.visual(mesh_from_geometry(yoke_geom, "yoke"), origin=Origin(xyz=(0, 0, 0.10)), name="yoke")

    head = model.part("head")
    head_mesh = mesh_from_cadquery(build_fan_head(), "head_mesh")
    head.visual(head_mesh, origin=Origin(xyz=(0, 0, 0)), name="head_mesh")

    rotor = model.part("rotor")
    rotor_geom = FanRotorGeometry(
        outer_radius=0.10,
        hub_radius=0.015,
        blade_count=5,
        thickness=0.01,
        blade_pitch_deg=30.0,
        blade=FanRotorBlade(shape="broad"),
        hub=FanRotorHub(style="spinner")
    )
    rotor.visual(mesh_from_geometry(rotor_geom, "rotor_geom"), origin=Origin(xyz=(0, 0, 0)), name="rotor_mesh")

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0, 0, 0.25)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-0.5, upper=0.5)
    )

    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0, 0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.allow_overlap(
        "base", "head",
        elem_a="yoke", elem_b="head_mesh",
        reason="Trunnions are captured in the yoke bores."
    )
    ctx.allow_overlap(
        "head", "rotor",
        elem_a="head_mesh", elem_b="rotor_mesh",
        reason="Motor shaft connects to the rotor hub."
    )
    
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    
    ctx.expect_contact(base, head, elem_a="yoke", elem_b="head_mesh")
    ctx.expect_within(rotor, head, axes="xy")
    
    return ctx.report()

object_model = build_object_model()
