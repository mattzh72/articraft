from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_fork_carriage")

    mast = model.part("mast")
    # Mast rails
    mast.visual(Box((0.1, 0.1, 2.5)), origin=Origin(xyz=(0.0, -0.3, 1.25)), name="left_rail")
    mast.visual(Box((0.1, 0.1, 2.5)), origin=Origin(xyz=(0.0, 0.3, 1.25)), name="right_rail")
    # Mast crossbars
    mast.visual(Box((0.1, 0.7, 0.1)), origin=Origin(xyz=(-0.1, 0.0, 0.05)), name="bottom_crossbar")
    mast.visual(Box((0.1, 0.7, 0.1)), origin=Origin(xyz=(-0.1, 0.0, 2.45)), name="top_crossbar")

    carriage = model.part("carriage")
    
    # Carriage left wraparound guide (front, outer, inner)
    carriage.visual(Box((0.05, 0.12, 1.0)), origin=Origin(xyz=(0.075, -0.3, 0.5)), name="left_upright_front")
    carriage.visual(Box((0.15, 0.02, 1.0)), origin=Origin(xyz=(0.025, -0.37, 0.5)), name="left_upright_outer")
    carriage.visual(Box((0.15, 0.02, 1.0)), origin=Origin(xyz=(0.025, -0.23, 0.5)), name="left_upright_inner")
    
    # Carriage right wraparound guide (front, outer, inner)
    carriage.visual(Box((0.05, 0.12, 1.0)), origin=Origin(xyz=(0.075, 0.3, 0.5)), name="right_upright_front")
    carriage.visual(Box((0.15, 0.02, 1.0)), origin=Origin(xyz=(0.025, 0.37, 0.5)), name="right_upright_outer")
    carriage.visual(Box((0.15, 0.02, 1.0)), origin=Origin(xyz=(0.025, 0.23, 0.5)), name="right_upright_inner")
    
    # Carriage horizontal bars
    carriage.visual(Box((0.05, 0.72, 0.1)), origin=Origin(xyz=(0.075, 0.0, 0.1)), name="bottom_carriage_bar")
    carriage.visual(Box((0.05, 0.72, 0.1)), origin=Origin(xyz=(0.075, 0.0, 0.85)), name="top_carriage_bar")
    
    # Forks
    # Left fork
    carriage.visual(Box((0.05, 0.1, 0.85)), origin=Origin(xyz=(0.125, -0.2, 0.475)), name="left_fork_shank")
    carriage.visual(Box((1.0, 0.1, 0.05)), origin=Origin(xyz=(0.65, -0.2, 0.075)), name="left_fork_blade")
    
    # Right fork
    carriage.visual(Box((0.05, 0.1, 0.85)), origin=Origin(xyz=(0.125, 0.2, 0.475)), name="right_fork_shank")
    carriage.visual(Box((1.0, 0.1, 0.05)), origin=Origin(xyz=(0.65, 0.2, 0.075)), name="right_fork_blade")

    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1000.0, velocity=1.0, lower=0.0, upper=1.4),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("carriage_lift")
    
    # Carriage should stay within mast Z bounds
    ctx.expect_within(carriage, mast, axes="z", margin=0.0, name="carriage rests within mast bounds")
    
    # Test lifted pose
    with ctx.pose({lift: 1.4}):
        ctx.expect_within(carriage, mast, axes="z", margin=0.0)
        
    return ctx.report()

object_model = build_object_model()