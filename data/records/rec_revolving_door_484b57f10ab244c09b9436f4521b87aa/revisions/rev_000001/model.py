from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq
import math

def build_housing_walls():
    outer_r = 1.25
    inner_r = 1.20
    h = 2.5
    cut_w = 2 * outer_r * math.sin(math.radians(40))
    
    walls = (
        cq.Workplane("XY")
        .cylinder(h, outer_r)
        .cut(cq.Workplane("XY").cylinder(h, inner_r))
        .cut(cq.Workplane("XY").box(cut_w, outer_r * 4, h + 0.1))
    )
    return walls

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolving_door")

    model.material("dark_metal", color=(0.2, 0.2, 0.2))
    model.material("glass", rgba=(0.7, 0.8, 0.9, 0.5))
    model.material("shaft_metal", color=(0.3, 0.3, 0.3))

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=1.25, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base",
        material="dark_metal",
    )
    housing.visual(
        Cylinder(radius=1.25, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 2.65)),
        name="canopy",
        material="dark_metal",
    )
    housing.visual(
        mesh_from_cadquery(build_housing_walls(), "housing_walls"),
        origin=Origin(xyz=(0.0, 0.0, 1.3)),
        name="walls",
        material="glass",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.05, length=2.50),
        origin=Origin(xyz=(0.0, 0.0, 1.3)),
        name="shaft",
        material="shaft_metal",
    )
    
    rotor.visual(
        Box((1.14, 0.04, 2.46)),
        origin=Origin(xyz=(0.62, 0.0, 1.3)),
        name="wing_1",
        material="glass",
    )
    rotor.visual(
        Box((1.14, 0.04, 2.46)),
        origin=Origin(xyz=(-0.62, 0.0, 1.3)),
        name="wing_2",
        material="glass",
    )
    rotor.visual(
        Box((0.04, 1.14, 2.46)),
        origin=Origin(xyz=(0.0, 0.62, 1.3)),
        name="wing_3",
        material="glass",
    )
    rotor.visual(
        Box((0.04, 1.14, 2.46)),
        origin=Origin(xyz=(0.0, -0.62, 1.3)),
        name="wing_4",
        material="glass",
    )

    model.articulation(
        "rotor_joint",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    
    ctx.expect_within(rotor, housing, axes="xy", margin=0.0)
    
    ctx.expect_contact(rotor, housing, elem_a="shaft", elem_b="base")
    ctx.expect_contact(housing, rotor, elem_a="canopy", elem_b="shaft")

    with ctx.pose({"rotor_joint": math.pi / 4}):
        ctx.expect_within(rotor, housing, axes="xy", margin=0.0)

    return ctx.report()

object_model = build_object_model()
