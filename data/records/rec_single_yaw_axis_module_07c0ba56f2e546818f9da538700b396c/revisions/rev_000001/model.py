import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turntable_mechanism")

    # Base: low profile, slightly chamfered housing, hollowed out
    base_cq = (
        cq.Workplane("XY")
        .cylinder(0.01, 0.18, centered=(True, True, False)) # Mounting flange
        .faces(">Z")
        .workplane()
        .cylinder(0.04, 0.15, centered=(True, True, False)) # Main housing
        .edges(">Z")
        .chamfer(0.005)
        .faces(">Z")
        .workplane()
        .cylinder(0.002, 0.06, centered=(True, True, False)) # Bearing boss
        .faces("<Z")
        .shell(-0.005)
    )
    
    # Top plate: circular plate
    top_plate_cq = (
        cq.Workplane("XY")
        .cylinder(0.015, 0.15, centered=(True, True, False))
        .edges(">Z or <Z")
        .chamfer(0.002)
    )
    
    # Pedestal: rigid mounting pedestal with a flange
    pedestal_cq = (
        cq.Workplane("XY")
        .cylinder(0.01, 0.06, centered=(True, True, False)) # Flange
        .faces(">Z")
        .workplane()
        .cylinder(0.12, 0.03, centered=(True, True, False)) # Shaft
        .faces(">Z")
        .workplane()
        .box(0.10, 0.10, 0.01, centered=(True, True, False)) # Sensor mounting plate
        .edges(">Z").chamfer(0.002)
    )

    # Base part
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_cq, "base_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="base_geom",
        material=Material(name="base_mat", color=(0.2, 0.2, 0.22)),
    )
    
    # Top plate part
    top_plate = model.part("top_plate")
    top_plate.visual(
        mesh_from_cadquery(top_plate_cq, "top_plate_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="top_plate_geom",
        material=Material(name="top_plate_mat", color=(0.7, 0.7, 0.75)),
    )
    
    # Pedestal part
    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(pedestal_cq, "pedestal_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="pedestal_geom",
        material=Material(name="pedestal_mat", color=(0.15, 0.15, 0.15)),
    )

    # Articulations
    # Pan joint (base to top_plate)
    # Base height: 0.01 (flange) + 0.04 (housing) + 0.002 (boss) = 0.052
    model.articulation(
        "pan_joint",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0),
    )
    
    # Fixed joint (top_plate to pedestal)
    # Top plate height: 0.015
    model.articulation(
        "top_plate_to_pedestal",
        ArticulationType.FIXED,
        parent=top_plate,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    top_plate = object_model.get_part("top_plate")
    pedestal = object_model.get_part("pedestal")
    
    ctx.expect_gap(top_plate, base, axis="z", min_gap=0.0, max_gap=0.002)
    ctx.expect_overlap(top_plate, base, axes="xy", min_overlap=0.04)
    
    ctx.expect_gap(pedestal, top_plate, axis="z", min_gap=0.0, max_gap=0.002)
    ctx.expect_overlap(pedestal, top_plate, axes="xy", min_overlap=0.05)

    return ctx.report()

object_model = build_object_model()