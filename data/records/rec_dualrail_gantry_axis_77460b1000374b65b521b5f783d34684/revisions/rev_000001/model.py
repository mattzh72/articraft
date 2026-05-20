from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_rail_gantry")

    # Materials
    mat_base = Material("base_mat", rgba=(0.2, 0.2, 0.22, 1.0))
    mat_rail = Material("rail_mat", rgba=(0.85, 0.85, 0.9, 1.0))
    mat_block = Material("block_mat", rgba=(0.15, 0.15, 0.15, 1.0))
    mat_motor = Material("motor_mat", rgba=(0.1, 0.1, 0.1, 1.0))
    mat_brass = Material("brass_mat", rgba=(0.8, 0.7, 0.3, 1.0))
    mat_plate = Material("plate_mat", rgba=(0.7, 0.7, 0.75, 1.0))

    base = model.part("base")
    
    # Base plate
    base.visual(
        Box((0.700, 0.200, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        name="base_plate",
        material=mat_base,
    )
    
    # Rails
    base.visual(
        Box((0.600, 0.015, 0.020)),
        origin=Origin(xyz=(0.0, -0.060, 0.025)),
        name="rail_1",
        material=mat_rail,
    )
    base.visual(
        Box((0.600, 0.015, 0.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.025)),
        name="rail_2",
        material=mat_rail,
    )

    # Ballscrew
    base.visual(
        Cylinder(radius=0.006, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.035), rpy=(0.0, math.pi / 2, 0.0)),
        name="ballscrew",
        material=mat_rail,
    )

    # Motor
    base.visual(
        Box((0.040, 0.040, 0.040)),
        origin=Origin(xyz=(-0.310, 0.0, 0.035)),
        name="motor",
        material=mat_motor,
    )

    # End bearing
    base.visual(
        Box((0.020, 0.030, 0.025)),
        origin=Origin(xyz=(0.300, 0.0, 0.0275)),
        name="end_bearing",
        material=mat_block,
    )

    carriage = model.part("carriage")
    
    # Guide blocks
    carriage.visual(
        Box((0.080, 0.025, 0.020)),
        origin=Origin(xyz=(0.0, -0.060, 0.040)),
        name="guide_block_1",
        material=mat_block,
    )
    carriage.visual(
        Box((0.080, 0.025, 0.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.040)),
        name="guide_block_2",
        material=mat_block,
    )
    
    # Carriage nut
    carriage.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.035), rpy=(0.0, math.pi / 2, 0.0)),
        name="carriage_nut",
        material=mat_brass,
    )

    # Carriage plate
    carriage.visual(
        Box((0.100, 0.160, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        name="carriage_plate",
        material=mat_plate,
    )

    # Articulation
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=-0.235, upper=0.235),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    
    # Allow overlaps because the guide blocks wrap the rails
    ctx.allow_overlap(
        carriage, base,
        elem_a="guide_block_1", elem_b="rail_1",
        reason="Guide block 1 wraps around rail 1"
    )
    ctx.allow_overlap(
        carriage, base,
        elem_a="guide_block_2", elem_b="rail_2",
        reason="Guide block 2 wraps around rail 2"
    )
    # The carriage nut wraps the ballscrew
    ctx.allow_overlap(
        carriage, base,
        elem_a="carriage_nut", elem_b="ballscrew",
        reason="Carriage nut wraps around the ballscrew"
    )
    
    # Tests
    # Centered in Y
    ctx.expect_within(
        carriage, base,
        axes="y",
        inner_elem="guide_block_1", outer_elem="base_plate",
        margin=0.0
    )
    
    # Overlap on X to ensure it stays on rails
    ctx.expect_overlap(
        carriage, base,
        axes="x",
        elem_a="guide_block_1", elem_b="rail_1",
        min_overlap=0.075
    )
    
    # Test at limits
    slide = object_model.get_articulation("carriage_slide")
    with ctx.pose({slide: 0.235}):
        ctx.expect_overlap(
            carriage, base,
            axes="x",
            elem_a="guide_block_1", elem_b="rail_1",
            min_overlap=0.075
        )
        ctx.expect_overlap(
            carriage, base,
            axes="x",
            elem_a="carriage_nut", elem_b="ballscrew",
            min_overlap=0.030
        )
    with ctx.pose({slide: -0.235}):
        ctx.expect_overlap(
            carriage, base,
            axes="x",
            elem_a="guide_block_1", elem_b="rail_1",
            min_overlap=0.075
        )
        ctx.expect_overlap(
            carriage, base,
            axes="x",
            elem_a="carriage_nut", elem_b="ballscrew",
            min_overlap=0.030
        )
        
    return ctx.report()

object_model = build_object_model()
