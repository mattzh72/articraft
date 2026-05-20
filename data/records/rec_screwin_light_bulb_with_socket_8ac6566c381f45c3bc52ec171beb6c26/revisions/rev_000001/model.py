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
    model = ArticulatedObject(name="screw_in_light_bulb")

    # Materials
    model.material("frosted_glass", rgba=(0.95, 0.95, 0.95, 0.8))
    model.material("aluminum_base", rgba=(0.8, 0.8, 0.85, 1.0))
    model.material("black_insulator", rgba=(0.1, 0.1, 0.1, 1.0))
    model.material("brass_contact", rgba=(0.85, 0.75, 0.4, 1.0))
    model.material("black_bakelite", rgba=(0.12, 0.12, 0.12, 1.0))

    # Socket Housing Part
    socket_part = model.part("socket_housing")
    
    socket_geom = (
        cq.Workplane("XY")
        .cylinder(0.040, 0.020, centered=(True, True, False))
        .union(cq.Workplane("XY").cylinder(0.010, 0.025, centered=(True, True, False)))
        .faces(">Z").workplane().hole(0.027, 0.030)
    )
    socket_part.visual(mesh_from_cadquery(socket_geom, "socket_geom_mesh"), name="socket_geom", material="black_bakelite")
    
    socket_contact_geom = cq.Workplane("XY").workplane(offset=0.009).cylinder(0.001, 0.006, centered=(True, True, False))
    socket_part.visual(mesh_from_cadquery(socket_contact_geom, "socket_contact_geom_mesh"), name="socket_contact_geom", material="brass_contact")

    # Bulb Part
    bulb_part = model.part("bulb")
    
    glass_geom = (
        cq.Workplane("XZ")
        .moveTo(0, 0.027)
        .lineTo(0.013, 0.027)
        .lineTo(0.013, 0.040)
        .radiusArc((0.030, 0.070), -0.040)
        .radiusArc((0, 0.100), 0.030)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    bulb_part.visual(mesh_from_cadquery(glass_geom, "glass_geom_mesh"), name="glass_geom", material="frosted_glass")
    
    base_geom = cq.Workplane("XY").cylinder(0.027, 0.0125, centered=(True, True, False))
    for z_mm in range(3, 25, 3):
        z = z_mm / 1000.0
        ring = cq.Workplane("XY").workplane(offset=z).cylinder(0.0015, 0.013, centered=(True, True, False))
        base_geom = base_geom.union(ring)
    bulb_part.visual(mesh_from_cadquery(base_geom, "base_geom_mesh"), name="base_geom", material="aluminum_base")
    
    insulator_geom = cq.Workplane("XY").workplane(offset=-0.002).cylinder(0.002, 0.008, centered=(True, True, False))
    bulb_part.visual(mesh_from_cadquery(insulator_geom, "insulator_geom_mesh"), name="insulator_geom", material="black_insulator")
    
    contact_geom = cq.Workplane("XY").workplane(offset=-0.004).cylinder(0.002, 0.004, centered=(True, True, False))
    bulb_part.visual(mesh_from_cadquery(contact_geom, "contact_geom_mesh"), name="contact_geom", material="brass_contact")

    # Articulation
    model.articulation(
        "bulb_twist",
        ArticulationType.REVOLUTE,
        parent="socket_housing",
        child="bulb",
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-12.56, upper=0.0)
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    ctx.expect_contact("socket_housing", "bulb", elem_a="socket_contact_geom", elem_b="contact_geom")
    ctx.expect_within("bulb", "socket_housing", axes="xy", inner_elem="base_geom", outer_elem="socket_geom", margin=0.001)
    
    return ctx.report()

object_model = build_object_model()
