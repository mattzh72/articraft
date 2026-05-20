from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xz_stage")

    # Materials
    mat_base = Material(name="mat_base", color=(0.33, 0.33, 0.33))
    mat_rail = Material(name="mat_rail", color=(0.75, 0.75, 0.75))
    mat_carriage = Material(name="mat_carriage", color=(0.2, 0.2, 0.2))
    mat_platform = Material(name="mat_platform", color=(0.27, 0.27, 0.27))

    # 1. Base
    base = model.part("base")
    base.visual(Box((0.6, 0.15, 0.02)), origin=Origin((0.0, 0.0, 0.01)), name="base_plate", material=mat_base)
    base.visual(Box((0.5, 0.04, 0.02)), origin=Origin((0.0, 0.0, 0.03)), name="x_rail", material=mat_rail)

    # 2. X-Carriage
    x_carriage = model.part("x_carriage")
    # Origin at the top center of the X-rail
    
    # Carriage block wrapping X-rail (exact fit)
    x_carriage.visual(Box((0.08, 0.06, 0.01)), origin=Origin((0.0, 0.0, 0.005)), name="x_carriage_top", material=mat_carriage)
    x_carriage.visual(Box((0.08, 0.01, 0.03)), origin=Origin((0.0, 0.025, -0.005)), name="x_carriage_front", material=mat_carriage)
    x_carriage.visual(Box((0.08, 0.01, 0.03)), origin=Origin((0.0, -0.025, -0.005)), name="x_carriage_back", material=mat_carriage)

    # L-bracket mounted on top of x_carriage
    x_carriage.visual(Box((0.08, 0.08, 0.01)), origin=Origin((0.0, -0.01, 0.015)), name="bracket_base", material=mat_carriage)
    x_carriage.visual(Box((0.08, 0.02, 0.39)), origin=Origin((0.0, -0.04, 0.215)), name="bracket_upright", material=mat_carriage)

    # Z-rail mounted on the front of bracket_upright
    x_carriage.visual(Box((0.04, 0.02, 0.39)), origin=Origin((0.0, -0.02, 0.215)), name="z_rail", material=mat_rail)

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin((0.0, 0.0, 0.04)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.2, upper=0.2),
    )

    # 3. Z-Carriage
    z_carriage = model.part("z_carriage")
    
    # Carriage block wrapping Z-rail (exact fit)
    z_carriage.visual(Box((0.06, 0.01, 0.08)), origin=Origin((0.0, 0.005, 0.04)), name="z_carriage_front", material=mat_carriage)
    z_carriage.visual(Box((0.01, 0.03, 0.08)), origin=Origin((-0.025, -0.005, 0.04)), name="z_carriage_left", material=mat_carriage)
    z_carriage.visual(Box((0.01, 0.03, 0.08)), origin=Origin((0.025, -0.005, 0.04)), name="z_carriage_right", material=mat_carriage)

    # Lift platform
    z_carriage.visual(Box((0.1, 0.1, 0.01)), origin=Origin((0.0, 0.06, 0.075)), name="lift_platform", material=mat_platform)
    z_carriage.visual(Box((0.04, 0.07, 0.03)), origin=Origin((0.0, 0.045, 0.055)), name="platform_support", material=mat_platform)

    model.articulation(
        "x_to_z",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_carriage,
        origin=Origin((0.0, -0.01, 0.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.31),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    z_carriage = object_model.get_part("z_carriage")
    
    # Check X carriage is within X rail limits
    ctx.expect_within(x_carriage, base, axes="x", inner_elem="x_carriage_top", outer_elem="x_rail", margin=0.0)
    
    # Check Z carriage is within Z rail limits
    ctx.expect_within(z_carriage, x_carriage, axes="z", inner_elem="z_carriage_front", outer_elem="z_rail", margin=0.0)
    
    # Check exact contact
    ctx.expect_contact(x_carriage, base, elem_a="x_carriage_top", elem_b="x_rail")
    ctx.expect_contact(z_carriage, x_carriage, elem_a="z_carriage_front", elem_b="z_rail")

    with ctx.pose({"base_to_x": 0.2}):
        ctx.expect_within(x_carriage, base, axes="x", inner_elem="x_carriage_top", outer_elem="x_rail", margin=0.0)
        
    with ctx.pose({"x_to_z": 0.31}):
        ctx.expect_within(z_carriage, x_carriage, axes="z", inner_elem="z_carriage_front", outer_elem="z_rail", margin=0.0)
        
    return ctx.report()

object_model = build_object_model()
