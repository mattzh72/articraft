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

def make_base_plate():
    plate = cq.Workplane("XY").box(0.4, 0.4, 0.02)
    pts = [(x*0.175, y*0.175) for x in [-1, 1] for y in [-1, 1]]
    plate = plate.pushPoints(pts).cboreHole(0.008, 0.014, 0.01)
    return plate

def make_top_plate():
    plate = cq.Workplane("XY").box(0.15, 0.15, 0.02)
    pts = [(x*0.025, y*0.025) for x in range(-2, 3) for y in range(-2, 3)]
    plate = plate.pushPoints(pts).hole(0.006)
    return plate

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xy_stage")
    
    color_plate = (0.15, 0.15, 0.15)
    color_rail = (0.8, 0.8, 0.8)
    color_block = (0.1, 0.1, 0.1)
    color_top = (0.25, 0.25, 0.25)

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_plate(), "base_plate"), 
        origin=Origin(xyz=(0, 0, 0.01)), 
        name="base_plate", 
        color=color_plate
    )
    base.visual(Box((0.35, 0.02, 0.015)), origin=Origin(xyz=(0, -0.1, 0.0275)), name="x_rail_1", color=color_rail)
    base.visual(Box((0.35, 0.02, 0.015)), origin=Origin(xyz=(0, 0.1, 0.0275)), name="x_rail_2", color=color_rail)
    
    x_carriage = model.part("x_carriage")
    x_carriage.visual(Box((0.15, 0.04, 0.02)), origin=Origin(xyz=(0, -0.1, 0.045)), name="x_block_1", color=color_block)
    x_carriage.visual(Box((0.15, 0.04, 0.02)), origin=Origin(xyz=(0, 0.1, 0.045)), name="x_block_2", color=color_block)
    x_carriage.visual(Box((0.15, 0.25, 0.02)), origin=Origin(xyz=(0, 0, 0.065)), name="x_plate", color=color_plate)
    x_carriage.visual(Box((0.02, 0.25, 0.015)), origin=Origin(xyz=(-0.05, 0, 0.0825)), name="y_rail_1", color=color_rail)
    x_carriage.visual(Box((0.02, 0.25, 0.015)), origin=Origin(xyz=(0.05, 0, 0.0825)), name="y_rail_2", color=color_rail)
    
    y_carriage = model.part("y_carriage")
    y_carriage.visual(Box((0.04, 0.10, 0.02)), origin=Origin(xyz=(-0.05, 0, 0.10)), name="y_block_1", color=color_block)
    y_carriage.visual(Box((0.04, 0.10, 0.02)), origin=Origin(xyz=(0.05, 0, 0.10)), name="y_block_2", color=color_block)
    y_carriage.visual(
        mesh_from_cadquery(make_top_plate(), "top_plate"), 
        origin=Origin(xyz=(0, 0, 0.12)), 
        name="top_plate", 
        color=color_top
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=-0.1, upper=0.1, effort=100, velocity=1)
    )

    model.articulation(
        "y_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(lower=-0.075, upper=0.075, effort=100, velocity=1)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    
    # Check X axis motion
    ctx.expect_contact(base, x_carriage, name="x_carriage rests on base rails")
    
    with ctx.pose(x_axis=0.1):
        ctx.expect_contact(base, x_carriage, name="x_carriage rests on base rails at max extension")
        
    # Check Y axis motion
    ctx.expect_contact(x_carriage, y_carriage, name="y_carriage rests on x_carriage rails")
    
    with ctx.pose(y_axis=0.075):
        ctx.expect_contact(x_carriage, y_carriage, name="y_carriage rests on x_carriage rails at max extension")
        
    return ctx.report()

object_model = build_object_model()
