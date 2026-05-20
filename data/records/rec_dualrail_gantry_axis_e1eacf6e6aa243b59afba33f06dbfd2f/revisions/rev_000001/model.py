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
    model = ArticulatedObject(name="gantry")

    chassis = model.part("chassis")
    # Rails along X from 0.0 to 1.0
    chassis.visual(Box((1.0, 0.05, 0.05)), origin=Origin(xyz=(0.5, -0.4, 0.025)), name="rail_1")
    chassis.visual(Box((1.0, 0.05, 0.05)), origin=Origin(xyz=(0.5, 0.4, 0.025)), name="rail_2")
    # Crossbars connecting rails
    chassis.visual(Box((0.05, 0.75, 0.05)), origin=Origin(xyz=(0.025, 0.0, 0.025)), name="crossbar_1")
    chassis.visual(Box((0.05, 0.75, 0.05)), origin=Origin(xyz=(0.975, 0.0, 0.025)), name="crossbar_2")

    bridge = model.part("bridge")
    # End truck 1 (Y = -0.4)
    bridge.visual(Box((0.15, 0.07, 0.02)), origin=Origin(xyz=(0.0, -0.4, 0.06)), name="truck_1_top")
    bridge.visual(Box((0.15, 0.01, 0.04)), origin=Origin(xyz=(0.0, -0.37, 0.03)), name="truck_1_side_in")
    bridge.visual(Box((0.15, 0.01, 0.04)), origin=Origin(xyz=(0.0, -0.43, 0.03)), name="truck_1_side_out")
    
    # End truck 2 (Y = 0.4)
    bridge.visual(Box((0.15, 0.07, 0.02)), origin=Origin(xyz=(0.0, 0.4, 0.06)), name="truck_2_top")
    bridge.visual(Box((0.15, 0.01, 0.04)), origin=Origin(xyz=(0.0, 0.37, 0.03)), name="truck_2_side_in")
    bridge.visual(Box((0.15, 0.01, 0.04)), origin=Origin(xyz=(0.0, 0.43, 0.03)), name="truck_2_side_out")

    # Transverse beam
    bridge.visual(Box((0.05, 0.86, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.095)), name="beam")

    runner = model.part("runner")
    # Runner main body riding on beam
    runner.visual(Box((0.09, 0.08, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.13)), name="runner_top")
    runner.visual(Box((0.02, 0.08, 0.04)), origin=Origin(xyz=(0.035, 0.0, 0.10)), name="runner_side_pos_x")
    runner.visual(Box((0.02, 0.08, 0.04)), origin=Origin(xyz=(-0.035, 0.0, 0.10)), name="runner_side_neg_x")
    # Tool hanging from runner
    runner.visual(Box((0.04, 0.04, 0.1)), origin=Origin(xyz=(0.065, 0.0, 0.08)), name="runner_tool")

    model.articulation(
        "bridge_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=bridge,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.70),
    )

    model.articulation(
        "runner_slide",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=runner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.34, upper=0.34),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    chassis = object_model.get_part("chassis")
    bridge = object_model.get_part("bridge")
    runner = object_model.get_part("runner")
    
    # We will rely on contact checks instead.
    ctx.expect_contact(bridge, chassis, elem_a="truck_1_top", elem_b="rail_1")
    ctx.expect_contact(bridge, chassis, elem_a="truck_2_top", elem_b="rail_2")
    
    ctx.expect_contact(runner, bridge, elem_a="runner_top", elem_b="beam")

    bridge_slide = object_model.get_articulation("bridge_slide")
    runner_slide = object_model.get_articulation("runner_slide")

    with ctx.pose({bridge_slide: 0.0}):
        ctx.expect_gap(bridge, chassis, axis="x", min_gap=0.02, positive_elem="truck_1_top", negative_elem="crossbar_1")
    
    with ctx.pose({bridge_slide: 0.70}):
        ctx.expect_gap(chassis, bridge, axis="x", min_gap=0.02, positive_elem="crossbar_2", negative_elem="truck_1_top")

    with ctx.pose({runner_slide: 0.34}):
        ctx.expect_gap(bridge, runner, axis="y", min_gap=0.004, positive_elem="truck_2_top", negative_elem="runner_tool")

    with ctx.pose({runner_slide: -0.34}):
        ctx.expect_gap(runner, bridge, axis="y", min_gap=0.004, positive_elem="runner_tool", negative_elem="truck_1_top")

    return ctx.report()

object_model = build_object_model()