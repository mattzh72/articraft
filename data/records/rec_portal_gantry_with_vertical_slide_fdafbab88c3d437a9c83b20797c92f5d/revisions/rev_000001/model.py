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
    model = ArticulatedObject(name="routing_portal_gantry")

    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    painted_blue = Material("painted_blue", rgba=(0.05, 0.20, 0.48, 1.0))
    black = Material("black_anodized", rgba=(0.01, 0.012, 0.015, 1.0))
    silver = Material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    orange = Material("safety_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    rubber = Material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))
    wood = Material("spoilboard", rgba=(0.55, 0.38, 0.20, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((3.60, 1.55, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="floor_tie",
    )
    base.visual(
        Box((3.25, 1.05, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=wood,
        name="slatted_work_bed",
    )
    for y in (-0.76, 0.76):
        base.visual(
            Box((3.55, 0.18, 0.60)),
            origin=Origin(xyz=(0.0, y, 0.34)),
            material=dark_steel,
            name=f"side_frame_{0 if y < 0 else 1}",
        )
        base.visual(
            Box((3.50, 0.14, 0.12)),
            origin=Origin(xyz=(0.0, y, 0.65)),
            material=dark_steel,
            name=f"rail_bed_{0 if y < 0 else 1}",
        )
        if y < 0:
            base.visual(
                Box((3.30, 0.055, 0.06)),
                origin=Origin(xyz=(0.0, y, 0.72)),
                material=silver,
                name="base_linear_rail_0",
            )
        else:
            base.visual(
                Box((3.30, 0.055, 0.06)),
                origin=Origin(xyz=(0.0, y, 0.72)),
                material=silver,
                name="base_linear_rail_1",
            )
        base.visual(
            Box((3.20, 0.030, 0.035)),
            origin=Origin(xyz=(0.0, -0.845 if y < 0 else 0.845, 0.690)),
            material=silver,
            name=f"rack_strip_{0 if y < 0 else 1}",
        )
        for x in (-1.55, 0.0, 1.55):
            base.visual(
                Box((0.28, 0.30, 0.06)),
                origin=Origin(xyz=(x, y, 0.03)),
                material=dark_steel,
                name=f"foot_{0 if y < 0 else 1}_{x:+.1f}",
            )
    for x in (-1.68, 1.68):
        base.visual(
            Box((0.12, 1.68, 0.14)),
            origin=Origin(xyz=(x, 0.0, 0.34)),
            material=dark_steel,
            name=f"end_tie_{0 if x < 0 else 1}",
        )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.24, 1.72, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_blue,
        name="box_beam",
    )
    for y in (-0.76, 0.76):
        bridge.visual(
            Box((0.20, 0.16, 0.43)),
            origin=Origin(xyz=(0.0, y, -0.335)),
            material=painted_blue,
            name=f"upright_{0 if y < 0 else 1}",
        )
        if y < 0:
            bridge.visual(
                Box((0.42, 0.18, 0.08)),
                origin=Origin(xyz=(0.0, y, -0.59)),
                material=black,
                name="base_bearing_0",
            )
        else:
            bridge.visual(
                Box((0.42, 0.18, 0.08)),
                origin=Origin(xyz=(0.0, y, -0.59)),
                material=black,
                name="base_bearing_1",
            )
        bridge.visual(
            Box((0.32, 0.06, 0.045)),
            origin=Origin(xyz=(0.0, y, -0.515)),
            material=silver,
            name=f"truck_cap_{0 if y < 0 else 1}",
        )
    bridge.visual(
        Box((0.030, 1.46, 0.035)),
        origin=Origin(xyz=(-0.135, 0.0, 0.045)),
        material=silver,
        name="beam_rail_upper",
    )
    bridge.visual(
        Box((0.030, 1.46, 0.035)),
        origin=Origin(xyz=(-0.135, 0.0, -0.105)),
        material=silver,
        name="beam_rail_lower",
    )
    bridge.visual(
        Cylinder(radius=0.025, length=1.52),
        origin=Origin(xyz=(-0.145, 0.0, -0.040), rpy=(-1.5708, 0.0, 0.0)),
        material=silver,
        name="beam_ball_screw",
    )
    bridge.visual(
        Box((0.18, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, -0.88, 0.0)),
        material=black,
        name="beam_servo_cover",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.12, 0.30, 0.50)),
        origin=Origin(xyz=(-0.235, 0.0, -0.08)),
        material=black,
        name="saddle_plate",
    )
    for z in (0.045, -0.105):
        if z > 0:
            carriage.visual(
                Box((0.050, 0.24, 0.070)),
                origin=Origin(xyz=(-0.175, 0.0, z)),
                material=black,
                name="beam_bearing_0",
            )
        else:
            carriage.visual(
                Box((0.050, 0.24, 0.070)),
                origin=Origin(xyz=(-0.175, 0.0, z)),
                material=black,
                name="beam_bearing_1",
            )
    carriage.visual(
        Box((0.08, 0.26, 0.08)),
        origin=Origin(xyz=(-0.225, 0.0, 0.21)),
        material=black,
        name="saddle_top_bridge",
    )
    for y in (-0.075, 0.075):
        if y < 0:
            carriage.visual(
                Box((0.030, 0.035, 0.72)),
                origin=Origin(xyz=(-0.310, y, -0.275)),
                material=silver,
                name="vertical_rail_0",
            )
        else:
            carriage.visual(
                Box((0.030, 0.035, 0.72)),
                origin=Origin(xyz=(-0.310, y, -0.275)),
                material=silver,
                name="vertical_rail_1",
            )
    carriage.visual(
        Box((0.040, 0.18, 0.74)),
        origin=Origin(xyz=(-0.280, 0.0, -0.275)),
        material=black,
        name="z_axis_backbone",
    )

    tool_head = model.part("tool_head")
    tool_head.visual(
        Box((0.080, 0.245, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=orange,
        name="moving_z_plate",
    )
    tool_head.visual(
        Box((0.060, 0.060, 0.100)),
        origin=Origin(xyz=(0.030, -0.075, 0.075)),
        material=black,
        name="z_bearing_0_0",
    )
    tool_head.visual(
        Box((0.060, 0.060, 0.100)),
        origin=Origin(xyz=(0.030, -0.075, -0.295)),
        material=black,
        name="z_bearing_0_1",
    )
    tool_head.visual(
        Box((0.060, 0.060, 0.100)),
        origin=Origin(xyz=(0.030, 0.075, 0.075)),
        material=black,
        name="z_bearing_1_0",
    )
    tool_head.visual(
        Box((0.060, 0.060, 0.100)),
        origin=Origin(xyz=(0.030, 0.075, -0.295)),
        material=black,
        name="z_bearing_1_1",
    )
    tool_head.visual(
        Box((0.100, 0.180, 0.120)),
        origin=Origin(xyz=(-0.065, 0.0, -0.225)),
        material=black,
        name="spindle_clamp",
    )
    tool_head.visual(
        Cylinder(radius=0.060, length=0.38),
        origin=Origin(xyz=(-0.120, 0.0, -0.260)),
        material=silver,
        name="router_spindle",
    )
    tool_head.visual(
        Cylinder(radius=0.030, length=0.06),
        origin=Origin(xyz=(-0.120, 0.0, -0.480)),
        material=black,
        name="collet_nut",
    )
    tool_head.visual(
        Cylinder(radius=0.012, length=0.22),
        origin=Origin(xyz=(-0.120, 0.0, -0.620)),
        material=silver,
        name="cutting_bit",
    )
    tool_head.visual(
        Box((0.22, 0.040, 0.035)),
        origin=Origin(xyz=(-0.060, 0.140, -0.055)),
        material=rubber,
        name="dust_hose_stub",
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(-1.05, 0.0, 1.38)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=0.80, lower=0.0, upper=2.10),
    )
    model.articulation(
        "bridge_to_carriage",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.45, -0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.60, lower=0.0, upper=0.90),
    )
    model.articulation(
        "carriage_to_tool_head",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=tool_head,
        origin=Origin(xyz=(-0.385, 0.0, -0.05)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.35, lower=0.0, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("carriage")
    tool_head = object_model.get_part("tool_head")
    x_slide = object_model.get_articulation("base_to_bridge")
    y_slide = object_model.get_articulation("bridge_to_carriage")
    z_slide = object_model.get_articulation("carriage_to_tool_head")

    ctx.check(
        "three orthogonal prismatic axes",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(y_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(z_slide.axis) == (0.0, 0.0, -1.0),
        details="Bridge should move on X, carriage on Y, and tool head vertically downward.",
    )

    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        positive_elem="base_bearing_0",
        negative_elem="base_linear_rail_0",
        max_gap=0.002,
        max_penetration=0.001,
        name="bridge truck rests on first base rail",
    )
    ctx.expect_gap(
        bridge,
        base,
        axis="z",
        positive_elem="base_bearing_1",
        negative_elem="base_linear_rail_1",
        max_gap=0.002,
        max_penetration=0.001,
        name="bridge truck rests on second base rail",
    )
    ctx.expect_within(
        carriage,
        bridge,
        axes="y",
        inner_elem="saddle_plate",
        outer_elem="box_beam",
        margin=0.0,
        name="carriage is captured within bridge span",
    )
    ctx.expect_gap(
        bridge,
        carriage,
        axis="x",
        positive_elem="beam_rail_upper",
        negative_elem="beam_bearing_0",
        max_gap=0.002,
        max_penetration=0.001,
        name="carriage bearing rides bridge rail",
    )
    ctx.expect_gap(
        carriage,
        tool_head,
        axis="x",
        positive_elem="vertical_rail_0",
        negative_elem="z_bearing_0_0",
        max_gap=0.002,
        max_penetration=0.001,
        name="tool head bearing rides vertical rail",
    )

    bridge_rest = ctx.part_world_position(bridge)
    carriage_rest = ctx.part_world_position(carriage)
    tool_rest = ctx.part_world_position(tool_head)
    with ctx.pose({x_slide: 2.10, y_slide: 0.90, z_slide: 0.45}):
        bridge_far = ctx.part_world_position(bridge)
        carriage_far = ctx.part_world_position(carriage)
        tool_low = ctx.part_world_position(tool_head)
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="base_bearing_0",
            outer_elem="base_linear_rail_0",
            margin=0.0,
            name="extended bridge truck stays on first rail",
        )
        ctx.expect_within(
            carriage,
            bridge,
            axes="y",
            inner_elem="saddle_plate",
            outer_elem="box_beam",
            margin=0.0,
            name="extended carriage stays on bridge",
        )
        ctx.expect_gap(
            tool_head,
            base,
            axis="z",
            positive_elem="cutting_bit",
            negative_elem="slatted_work_bed",
            min_gap=0.001,
            name="lowered cutter remains above spoilboard",
        )

    ctx.check(
        "slides move in commanded directions",
        bridge_rest is not None
        and bridge_far is not None
        and carriage_rest is not None
        and carriage_far is not None
        and tool_rest is not None
        and tool_low is not None
        and bridge_far[0] > bridge_rest[0] + 2.0
        and carriage_far[1] > carriage_rest[1] + 0.8
        and tool_low[2] < tool_rest[2] - 0.4,
        details=f"rest={bridge_rest, carriage_rest, tool_rest}, far={bridge_far, carriage_far, tool_low}",
    )

    return ctx.report()


object_model = build_object_model()
