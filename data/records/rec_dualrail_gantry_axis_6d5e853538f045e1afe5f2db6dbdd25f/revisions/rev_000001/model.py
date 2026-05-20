from __future__ import annotations

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
    model = ArticulatedObject(name="dual_rail_gantry_axis")

    dark = model.material("black_anodized", rgba=(0.02, 0.022, 0.025, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    orange = model.material("painted_bridge_orange", rgba=(0.95, 0.36, 0.08, 1.0))
    blue = model.material("painted_truck_blue", rgba=(0.05, 0.22, 0.85, 1.0))
    rubber = model.material("dark_bearing_blocks", rgba=(0.015, 0.015, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.60, 0.055, 0.050)),
        origin=Origin(xyz=(0.0, -0.35, 0.075)),
        material=dark,
        name="base_rail_0",
    )
    base.visual(
        Box((1.60, 0.055, 0.050)),
        origin=Origin(xyz=(0.0, 0.35, 0.075)),
        material=dark,
        name="base_rail_1",
    )
    base.visual(
        Box((1.58, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, -0.35, 0.100)),
        material=steel,
        name="rail_cap_0",
    )
    base.visual(
        Box((1.58, 0.035, 0.012)),
        origin=Origin(xyz=(0.0, 0.35, 0.100)),
        material=steel,
        name="rail_cap_1",
    )
    for x, name in ((-0.75, "end_tie_0"), (0.75, "end_tie_1")):
        base.visual(
            Box((0.11, 0.86, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.035)),
            material=dark,
            name=name,
        )
    base.visual(
        Box((1.50, 0.060, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark,
        name="center_spine",
    )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.18, 0.13, 0.060)),
        origin=Origin(xyz=(0.0, -0.35, 0.0)),
        material=rubber,
        name="bearing_block_0",
    )
    bridge.visual(
        Box((0.18, 0.13, 0.060)),
        origin=Origin(xyz=(0.0, 0.35, 0.0)),
        material=rubber,
        name="bearing_block_1",
    )
    bridge.visual(
        Box((0.10, 0.09, 0.220)),
        origin=Origin(xyz=(0.0, -0.35, 0.105)),
        material=orange,
        name="upright_0",
    )
    bridge.visual(
        Box((0.10, 0.09, 0.220)),
        origin=Origin(xyz=(0.0, 0.35, 0.105)),
        material=orange,
        name="upright_1",
    )
    bridge.visual(
        Box((0.12, 0.86, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=orange,
        name="crossbeam",
    )
    bridge.visual(
        Box((0.018, 0.74, 0.018)),
        origin=Origin(xyz=(0.067, 0.0, 0.240)),
        material=steel,
        name="lower_cross_guide",
    )
    bridge.visual(
        Box((0.018, 0.74, 0.018)),
        origin=Origin(xyz=(0.067, 0.0, 0.280)),
        material=steel,
        name="upper_cross_guide",
    )

    truck = model.part("truck")
    truck.visual(
        Box((0.055, 0.125, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=blue,
        name="truck_body",
    )
    truck.visual(
        Box((0.012, 0.110, 0.026)),
        origin=Origin(xyz=(-0.022, 0.0, -0.020)),
        material=rubber,
        name="lower_bearing_pad",
    )
    truck.visual(
        Box((0.012, 0.110, 0.026)),
        origin=Origin(xyz=(-0.022, 0.0, 0.020)),
        material=rubber,
        name="upper_bearing_pad",
    )
    truck.visual(
        Box((0.038, 0.160, 0.220)),
        origin=Origin(xyz=(0.044, 0.0, -0.100)),
        material=blue,
        name="front_tool_plate",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(-0.45, 0.0, 0.136)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.90, effort=300.0, velocity=0.65),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=truck,
        origin=Origin(xyz=(0.104, -0.28, 0.260)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.56, effort=120.0, velocity=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    truck = object_model.get_part("truck")
    base_slide = object_model.get_articulation("base_to_bridge")
    cross_slide = object_model.get_articulation("bridge_to_truck")

    ctx.check(
        "two prismatic gantry axes",
        base_slide.articulation_type == ArticulationType.PRISMATIC
        and cross_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"base={base_slide.articulation_type}, cross={cross_slide.articulation_type}",
    )

    ctx.expect_contact(
        bridge,
        base,
        elem_a="bearing_block_0",
        elem_b="rail_cap_0",
        contact_tol=1e-4,
        name="bridge bearing sits on rail 0",
    )
    ctx.expect_contact(
        bridge,
        base,
        elem_a="bearing_block_1",
        elem_b="rail_cap_1",
        contact_tol=1e-4,
        name="bridge bearing sits on rail 1",
    )
    ctx.expect_within(
        bridge,
        base,
        axes="x",
        inner_elem="bearing_block_0",
        outer_elem="rail_cap_0",
        margin=0.0,
        name="bridge remains over rail 0 at home",
    )
    ctx.expect_within(
        truck,
        bridge,
        axes="y",
        inner_elem="lower_bearing_pad",
        outer_elem="lower_cross_guide",
        margin=0.0,
        name="truck bearing starts on cross guide",
    )
    ctx.expect_contact(
        truck,
        bridge,
        elem_a="lower_bearing_pad",
        elem_b="lower_cross_guide",
        contact_tol=1e-4,
        name="truck lower pad contacts guide",
    )
    ctx.expect_contact(
        truck,
        bridge,
        elem_a="upper_bearing_pad",
        elem_b="upper_cross_guide",
        contact_tol=1e-4,
        name="truck upper pad contacts guide",
    )

    bridge_home = ctx.part_world_position(bridge)
    with ctx.pose({base_slide: 0.90}):
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="bearing_block_1",
            outer_elem="rail_cap_1",
            margin=0.0,
            name="bridge remains over rail 1 at far travel",
        )
        bridge_far = ctx.part_world_position(bridge)
    ctx.check(
        "bridge travels along base rails",
        bridge_home is not None and bridge_far is not None and bridge_far[0] > bridge_home[0] + 0.80,
        details=f"home={bridge_home}, far={bridge_far}",
    )

    truck_home = ctx.part_world_position(truck)
    with ctx.pose({cross_slide: 0.56}):
        ctx.expect_within(
            truck,
            bridge,
            axes="y",
            inner_elem="upper_bearing_pad",
            outer_elem="upper_cross_guide",
            margin=0.0,
            name="truck bearing stays on cross guide at far travel",
        )
        truck_far = ctx.part_world_position(truck)
    ctx.check(
        "truck travels across bridge",
        truck_home is not None and truck_far is not None and truck_far[1] > truck_home[1] + 0.50,
        details=f"home={truck_home}, far={truck_far}",
    )

    return ctx.report()


object_model = build_object_model()
