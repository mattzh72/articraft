from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_base_dual_rail_gantry")

    painted_cast = model.material("painted_cast", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_steel = model.material("brushed_rail_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.08, 0.22, 0.55, 1.0))
    tool_black = model.material("tool_black", rgba=(0.025, 0.025, 0.025, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.70, 0.05, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.80, 0.86, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=painted_cast,
        name="base_plinth",
    )
    base.visual(
        Box((1.70, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, 0.30, 0.0925)),
        material=dark_cast,
        name="rail_bed_0",
    )
    base.visual(
        Box((1.70, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, -0.30, 0.0925)),
        material=dark_cast,
        name="rail_bed_1",
    )
    base.visual(
        Box((1.60, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.30, 0.1325)),
        material=rail_steel,
        name="rail_0",
    )
    base.visual(
        Box((1.60, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, -0.30, 0.1325)),
        material=rail_steel,
        name="rail_1",
    )
    for i, x in enumerate((-0.86, 0.86)):
        base.visual(
            Box((0.050, 0.78, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0975)),
            material=safety_yellow,
            name=f"end_stop_{i}",
        )
    for i, (x, y) in enumerate(
        ((-0.73, -0.35), (-0.73, 0.35), (0.73, -0.35), (0.73, 0.35))
    ):
        base.visual(
            Cylinder(radius=0.045, length=0.035),
            origin=Origin(xyz=(x, y, -0.0175)),
            material=dark_cast,
            name=f"leveling_foot_{i}",
        )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.28, 0.12, 0.045)),
        origin=Origin(xyz=(0.0, 0.30, 0.1725)),
        material=carriage_blue,
        name="bearing_0",
    )
    bridge.visual(
        Box((0.28, 0.12, 0.045)),
        origin=Origin(xyz=(0.0, -0.30, 0.1725)),
        material=carriage_blue,
        name="bearing_1",
    )
    for i, y in enumerate((0.36, -0.36)):
        bridge.visual(
            Box((0.12, 0.10, 0.420)),
            origin=Origin(xyz=(0.0, y, 0.405)),
            material=carriage_blue,
            name=f"upright_{i}",
        )
    bridge.visual(
        Box((0.16, 0.90, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        material=carriage_blue,
        name="crossbeam",
    )
    bridge.visual(
        Box((0.025, 0.74, 0.020)),
        origin=Origin(xyz=(-0.0925, 0.0, 0.675)),
        material=rail_steel,
        name="tool_guide_0",
    )
    bridge.visual(
        Box((0.025, 0.74, 0.020)),
        origin=Origin(xyz=(-0.0925, 0.0, 0.605)),
        material=rail_steel,
        name="tool_guide_1",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=-0.55, upper=0.55),
    )

    tool_slide = model.part("tool_slide")
    tool_slide.visual(
        Box((0.060, 0.160, 0.180)),
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material=dark_cast,
        name="carriage_plate",
    )
    tool_slide.visual(
        Box((0.100, 0.120, 0.120)),
        origin=Origin(xyz=(-0.105, 0.0, -0.050)),
        material=tool_black,
        name="motor_housing",
    )
    tool_slide.visual(
        Cylinder(radius=0.026, length=0.130),
        origin=Origin(xyz=(-0.105, 0.0, -0.175)),
        material=rail_steel,
        name="spindle_body",
    )
    tool_slide.visual(
        Cylinder(radius=0.012, length=0.065),
        origin=Origin(xyz=(-0.105, 0.0, -0.2725)),
        material=tool_black,
        name="tool_bit",
    )

    model.articulation(
        "bridge_to_tool_slide",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=tool_slide,
        origin=Origin(xyz=(-0.110, 0.0, 0.620)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=-0.30, upper=0.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    tool_slide = object_model.get_part("tool_slide")
    base_axis = object_model.get_articulation("base_to_bridge")
    tool_axis = object_model.get_articulation("bridge_to_tool_slide")

    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="bearing_0",
        negative_elem="rail_0",
        name="bridge bearing sits on first base rail",
    )
    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="bearing_1",
        negative_elem="rail_1",
        name="bridge bearing sits on second base rail",
    )
    ctx.expect_gap(
        bridge,
        tool_slide,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="tool_guide_0",
        negative_elem="carriage_plate",
        name="tool plate rides on bridge guide face",
    )
    ctx.expect_within(
        tool_slide,
        bridge,
        axes="y",
        inner_elem="carriage_plate",
        outer_elem="crossbeam",
        margin=0.0,
        name="tool slide starts near bridge midpoint",
    )

    with ctx.pose({base_axis: 0.55}):
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            min_overlap=0.10,
            elem_a="bearing_0",
            elem_b="rail_0",
            name="first bearing remains captured at forward travel",
        )
        forward_pos = ctx.part_world_position(bridge)
    with ctx.pose({base_axis: -0.55}):
        ctx.expect_overlap(
            bridge,
            base,
            axes="x",
            min_overlap=0.10,
            elem_a="bearing_1",
            elem_b="rail_1",
            name="second bearing remains captured at rear travel",
        )
        rear_pos = ctx.part_world_position(bridge)
    ctx.check(
        "bridge travels along base rail axis",
        forward_pos is not None and rear_pos is not None and forward_pos[0] > rear_pos[0] + 1.0,
        details=f"rear={rear_pos}, forward={forward_pos}",
    )

    with ctx.pose({tool_axis: 0.30}):
        ctx.expect_within(
            tool_slide,
            bridge,
            axes="y",
            inner_elem="carriage_plate",
            outer_elem="crossbeam",
            margin=0.0,
            name="tool slide remains on bridge at positive cross travel",
        )
        positive_tool_pos = ctx.part_world_position(tool_slide)
    with ctx.pose({tool_axis: -0.30}):
        ctx.expect_within(
            tool_slide,
            bridge,
            axes="y",
            inner_elem="carriage_plate",
            outer_elem="crossbeam",
            margin=0.0,
            name="tool slide remains on bridge at negative cross travel",
        )
        negative_tool_pos = ctx.part_world_position(tool_slide)
    ctx.check(
        "tool slide travels across the bridge",
        positive_tool_pos is not None
        and negative_tool_pos is not None
        and positive_tool_pos[1] > negative_tool_pos[1] + 0.55,
        details=f"negative={negative_tool_pos}, positive={positive_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()
