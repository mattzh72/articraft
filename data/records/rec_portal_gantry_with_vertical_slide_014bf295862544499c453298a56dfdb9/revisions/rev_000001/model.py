from __future__ import annotations

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
    model = ArticulatedObject(name="bridge_and_lift_portal_module")

    painted_base = Material("painted_base", rgba=(0.18, 0.19, 0.20, 1.0))
    blued_steel = Material("blued_steel", rgba=(0.03, 0.20, 0.55, 1.0))
    black_carriage = Material("black_carriage", rgba=(0.035, 0.037, 0.040, 1.0))
    polished_rail = Material("polished_rail", rgba=(0.72, 0.75, 0.76, 1.0))
    safety_orange = Material("safety_orange", rgba=(0.95, 0.34, 0.05, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.45, 0.72, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_base,
        name="ground_plate",
    )
    for y, name in ((0.25, "rail_0"), (-0.25, "rail_1")):
        base.visual(
            Box((1.25, 0.045, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.1075)),
            material=polished_rail,
            name=name,
        )
        for x, stop_name in ((-0.665, f"{name}_stop_0"), (0.665, f"{name}_stop_1")):
            base.visual(
                Box((0.045, 0.085, 0.075)),
                origin=Origin(xyz=(x, y, 0.1175)),
                material=black_carriage,
                name=stop_name,
            )
    for x, name in ((-0.62, "foot_0"), (0.62, "foot_1")):
        base.visual(
            Box((0.16, 0.78, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.0125)),
            material=rubber,
            name=name,
        )

    portal_bridge = model.part("portal_bridge")
    for y, suffix in ((0.25, "0"), (-0.25, "1")):
        portal_bridge.visual(
            Box((0.18, 0.105, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.1575)),
            material=black_carriage,
            name=f"side_carriage_{suffix}",
        )
        portal_bridge.visual(
            Box((0.18, 0.020, 0.056)),
            origin=Origin(xyz=(0.0, y + 0.0445, 0.111)),
            material=black_carriage,
            name=f"outer_guide_shoe_{suffix}",
        )
        portal_bridge.visual(
            Box((0.18, 0.020, 0.056)),
            origin=Origin(xyz=(0.0, y - 0.0445, 0.111)),
            material=black_carriage,
            name=f"inner_guide_shoe_{suffix}",
        )
        portal_bridge.visual(
            Box((0.09, 0.08, 0.62)),
            origin=Origin(xyz=(0.0, y, 0.485)),
            material=blued_steel,
            name=f"upright_{suffix}",
        )
    portal_bridge.visual(
        Box((0.12, 0.68, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=blued_steel,
        name="bridge_beam",
    )
    portal_bridge.visual(
        Box((0.045, 0.58, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        material=polished_rail,
        name="bridge_y_rail",
    )
    portal_bridge.visual(
        Box((0.03, 0.62, 0.035)),
        origin=Origin(xyz=(0.075, 0.0, 0.765)),
        material=black_carriage,
        name="rail_backing_bar",
    )
    for y, name in ((0.295, "backing_tab_0"), (-0.295, "backing_tab_1")):
        portal_bridge.visual(
            Box((0.03, 0.03, 0.055)),
            origin=Origin(xyz=(0.06, y, 0.8075)),
            material=black_carriage,
            name=name,
        )

    model.articulation(
        "base_to_portal_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=portal_bridge,
        origin=Origin(xyz=(-0.38, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.55, lower=0.0, upper=0.75),
    )

    center_runner = model.part("center_runner")
    center_runner.visual(
        Box((0.12, 0.18, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.0525)),
        material=black_carriage,
        name="saddle_lower",
    )
    for x, name in ((0.0475, "saddle_cheek_0"), (-0.0475, "saddle_cheek_1")):
        center_runner.visual(
            Box((0.025, 0.18, 0.05)),
            origin=Origin(xyz=(x, 0.0, -0.015)),
            material=black_carriage,
            name=name,
        )
    center_runner.visual(
        Box((0.09, 0.14, 0.08)),
        origin=Origin(xyz=(-0.08, 0.0, -0.075)),
        material=black_carriage,
        name="nose_block",
    )
    center_runner.visual(
        Box((0.04, 0.22, 0.46)),
        origin=Origin(xyz=(-0.145, 0.0, -0.28)),
        material=blued_steel,
        name="vertical_carrier",
    )
    for y, name in ((0.07, "lift_guide_0"), (-0.07, "lift_guide_1")):
        center_runner.visual(
            Box((0.03, 0.035, 0.42)),
            origin=Origin(xyz=(-0.18, y, -0.29)),
            material=polished_rail,
            name=name,
        )

    model.articulation(
        "portal_bridge_to_center_runner",
        ArticulationType.PRISMATIC,
        parent=portal_bridge,
        child=center_runner,
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.45, lower=-0.26, upper=0.26),
    )

    lift_slide = model.part("lift_slide")
    lift_slide.visual(
        Box((0.04, 0.20, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=safety_orange,
        name="slide_plate",
    )
    lift_slide.visual(
        Box((0.06, 0.16, 0.08)),
        origin=Origin(xyz=(-0.05, 0.0, -0.165)),
        material=safety_orange,
        name="tool_mount",
    )
    lift_slide.visual(
        Box((0.035, 0.19, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=black_carriage,
        name="upper_stop_block",
    )
    lift_slide.visual(
        Box((0.05, 0.12, 0.035)),
        origin=Origin(xyz=(-0.05, 0.0, -0.2225)),
        material=black_carriage,
        name="tool_pad",
    )

    model.articulation(
        "center_runner_to_lift_slide",
        ArticulationType.PRISMATIC,
        parent=center_runner,
        child=lift_slide,
        origin=Origin(xyz=(-0.215, 0.0, -0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.30, lower=0.0, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    portal_bridge = object_model.get_part("portal_bridge")
    center_runner = object_model.get_part("center_runner")
    lift_slide = object_model.get_part("lift_slide")
    bridge_axis = object_model.get_articulation("base_to_portal_bridge")
    runner_axis = object_model.get_articulation("portal_bridge_to_center_runner")
    lift_axis = object_model.get_articulation("center_runner_to_lift_slide")

    ctx.check(
        "three supported prismatic joints",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (bridge_axis, runner_axis, lift_axis)
        ),
        details="portal bridge, center runner, and lift slide must all use prismatic motion",
    )

    ctx.expect_gap(
        portal_bridge,
        base,
        axis="z",
        positive_elem="side_carriage_0",
        negative_elem="rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="portal carriage sits on rail 0",
    )
    ctx.expect_gap(
        portal_bridge,
        base,
        axis="z",
        positive_elem="side_carriage_1",
        negative_elem="rail_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="portal carriage sits on rail 1",
    )
    ctx.expect_overlap(
        portal_bridge,
        base,
        axes="xy",
        elem_a="side_carriage_0",
        elem_b="rail_0",
        min_overlap=0.04,
        name="portal carriage 0 is captured over its rail",
    )
    ctx.expect_gap(
        portal_bridge,
        center_runner,
        axis="z",
        positive_elem="bridge_y_rail",
        negative_elem="saddle_lower",
        max_gap=0.001,
        max_penetration=0.0,
        name="runner saddle hangs from bridge rail",
    )
    ctx.expect_overlap(
        center_runner,
        portal_bridge,
        axes="xy",
        elem_a="saddle_lower",
        elem_b="bridge_y_rail",
        min_overlap=0.04,
        name="runner remains laterally captured on bridge rail",
    )
    ctx.expect_gap(
        center_runner,
        lift_slide,
        axis="x",
        positive_elem="lift_guide_0",
        negative_elem="slide_plate",
        max_gap=0.001,
        max_penetration=0.0005,
        name="lift slide bears on vertical guide",
    )
    ctx.expect_overlap(
        lift_slide,
        center_runner,
        axes="yz",
        elem_a="slide_plate",
        elem_b="lift_guide_0",
        min_overlap=0.03,
        name="lift slide is retained on vertical guide",
    )

    bridge_rest = ctx.part_world_position(portal_bridge)
    runner_rest = ctx.part_world_position(center_runner)
    lift_rest = ctx.part_world_position(lift_slide)

    with ctx.pose({bridge_axis: 0.75, runner_axis: 0.26, lift_axis: 0.30}):
        bridge_extended = ctx.part_world_position(portal_bridge)
        runner_extended = ctx.part_world_position(center_runner)
        lift_extended = ctx.part_world_position(lift_slide)
        ctx.expect_within(
            portal_bridge,
            base,
            axes="x",
            inner_elem="side_carriage_0",
            outer_elem="rail_0",
            margin=0.0,
            name="portal carriage remains on rail at full travel",
        )
        ctx.expect_overlap(
            center_runner,
            portal_bridge,
            axes="y",
            elem_a="saddle_lower",
            elem_b="bridge_y_rail",
            min_overlap=0.08,
            name="center runner remains engaged at full travel",
        )
        ctx.expect_overlap(
            lift_slide,
            center_runner,
            axes="z",
            elem_a="slide_plate",
            elem_b="lift_guide_0",
            min_overlap=0.08,
            name="lift slide remains engaged at full lift",
        )

    ctx.check(
        "portal bridge translates along base rails",
        bridge_rest is not None
        and bridge_extended is not None
        and bridge_extended[0] > bridge_rest[0] + 0.70,
        details=f"rest={bridge_rest}, extended={bridge_extended}",
    )
    ctx.check(
        "center runner translates across bridge",
        runner_rest is not None
        and runner_extended is not None
        and runner_extended[1] > runner_rest[1] + 0.24,
        details=f"rest={runner_rest}, extended={runner_extended}",
    )
    ctx.check(
        "lift slide translates vertically",
        lift_rest is not None
        and lift_extended is not None
        and lift_extended[2] > lift_rest[2] + 0.28,
        details=f"rest={lift_rest}, extended={lift_extended}",
    )

    return ctx.report()


object_model = build_object_model()
