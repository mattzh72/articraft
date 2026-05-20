from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_gantry_positioner")

    painted = model.material("painted_frame", rgba=(0.10, 0.12, 0.14, 1.0))
    steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.48, 0.52, 0.55, 1.0))
    blue = model.material("blue_bridge", rgba=(0.08, 0.24, 0.62, 1.0))
    black = model.material("black_bearing_blocks", rgba=(0.02, 0.025, 0.03, 1.0))
    orange = model.material("safety_orange", rgba=(0.95, 0.38, 0.06, 1.0))

    base = model.part("base_rails")
    base.visual(
        Box((1.62, 0.90, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=painted,
        name="base_plate",
    )
    for i, y in enumerate((-0.32, 0.32)):
        base.visual(
            Box((1.48, 0.075, 0.080)),
            origin=Origin(xyz=(0.0, y, 0.090)),
            material=aluminum,
            name=f"rail_extrusion_{i}",
        )
        base.visual(
            Box((1.42, 0.035, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.1425)),
            material=steel,
            name=("guide_rail_0", "guide_rail_1")[i],
        )
        for j, x in enumerate((-0.73, 0.73)):
            base.visual(
                Box((0.045, 0.095, 0.045)),
                origin=Origin(xyz=(x, y, 0.1775)),
                material=orange,
                name=f"hard_stop_{i}_{j}",
            )

    bridge = model.part("bridge_carriage")
    for i, y in enumerate((-0.32, 0.32)):
        bridge.visual(
            Box((0.220, 0.120, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.025)),
            material=black,
            name=("bearing_block_0", "bearing_block_1")[i],
        )
        bridge.visual(
            Box((0.120, 0.080, 0.350)),
            origin=Origin(xyz=(0.0, y, 0.225)),
            material=blue,
            name=f"upright_{i}",
        )
    bridge.visual(
        Box((0.110, 0.840, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=blue,
        name="bridge_beam",
    )
    bridge.visual(
        Box((0.015, 0.660, 0.018)),
        origin=Origin(xyz=(-0.0625, 0.0, 0.435)),
        material=steel,
        name="front_guide_upper",
    )
    bridge.visual(
        Box((0.015, 0.660, 0.018)),
        origin=Origin(xyz=(-0.0625, 0.0, 0.395)),
        material=steel,
        name="front_guide_lower",
    )

    tool = model.part("tool_truck")
    tool.visual(
        Box((0.024, 0.180, 0.180)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        material=black,
        name="rear_wear_pad",
    )
    tool.visual(
        Box((0.070, 0.220, 0.220)),
        origin=Origin(xyz=(-0.055, 0.0, -0.030)),
        material=painted,
        name="truck_body",
    )
    tool.visual(
        Box((0.055, 0.130, 0.160)),
        origin=Origin(xyz=(-0.065, 0.0, -0.185)),
        material=painted,
        name="tool_mount",
    )
    tool.visual(
        Cylinder(radius=0.026, length=0.210),
        origin=Origin(xyz=(-0.065, 0.0, -0.330)),
        material=steel,
        name="tool_spindle",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.60, lower=-0.50, upper=0.50),
    )
    model.articulation(
        "bridge_to_truck",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=tool,
        origin=Origin(xyz=(-0.070, 0.0, 0.415)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.45, lower=-0.22, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_rails")
    bridge = object_model.get_part("bridge_carriage")
    tool = object_model.get_part("tool_truck")
    bridge_slide = object_model.get_articulation("base_to_bridge")
    truck_slide = object_model.get_articulation("bridge_to_truck")

    ctx.check(
        "orthogonal gantry axes",
        bridge_slide.axis == (1.0, 0.0, 0.0) and truck_slide.axis == (0.0, 1.0, 0.0),
        details=f"bridge_axis={bridge_slide.axis}, truck_axis={truck_slide.axis}",
    )
    ctx.expect_contact(
        bridge,
        base,
        elem_a="bearing_block_0",
        elem_b="guide_rail_0",
        name="rear bridge block sits on rear rail",
    )
    ctx.expect_contact(
        bridge,
        base,
        elem_a="bearing_block_1",
        elem_b="guide_rail_1",
        name="front bridge block sits on front rail",
    )
    ctx.expect_contact(
        tool,
        bridge,
        elem_a="rear_wear_pad",
        elem_b="front_guide_upper",
        name="tool truck bears on bridge slide",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="xy",
        elem_a="bearing_block_0",
        elem_b="guide_rail_0",
        min_overlap=0.030,
        name="rear bearing block captures rail footprint",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="xy",
        elem_a="bearing_block_1",
        elem_b="guide_rail_1",
        min_overlap=0.030,
        name="front bearing block captures rail footprint",
    )
    ctx.expect_overlap(
        tool,
        bridge,
        axes="y",
        elem_a="rear_wear_pad",
        elem_b="front_guide_upper",
        min_overlap=0.160,
        name="centered tool truck remains on beam slide",
    )

    rest_bridge = ctx.part_world_position(bridge)
    rest_tool = ctx.part_world_position(tool)
    with ctx.pose({bridge_slide: 0.42}):
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            elem_a="bearing_block_0",
            elem_b="guide_rail_0",
            min_overlap=0.180,
            name="bridge carriage remains engaged at rail travel",
        )
        moved_bridge = ctx.part_world_position(bridge)
    with ctx.pose({truck_slide: 0.18}):
        ctx.expect_overlap(
            tool,
            bridge,
            axes="y",
            elem_a="rear_wear_pad",
            elem_b="front_guide_upper",
            min_overlap=0.090,
            name="tool truck remains engaged at beam travel",
        )
        moved_tool = ctx.part_world_position(tool)
    ctx.check(
        "bridge translates along rail axis",
        rest_bridge is not None and moved_bridge is not None and moved_bridge[0] > rest_bridge[0] + 0.35,
        details=f"rest={rest_bridge}, moved={moved_bridge}",
    )
    ctx.check(
        "tool truck translates across beam",
        rest_tool is not None and moved_tool is not None and moved_tool[1] > rest_tool[1] + 0.12,
        details=f"rest={rest_tool}, moved={moved_tool}",
    )

    return ctx.report()


object_model = build_object_model()
