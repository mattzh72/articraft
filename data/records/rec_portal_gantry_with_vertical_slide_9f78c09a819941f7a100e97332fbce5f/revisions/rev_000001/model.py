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
    model = ArticulatedObject(name="portal_gantry_module")

    painted_blue = model.material("painted_blue", rgba=(0.08, 0.18, 0.32, 1.0))
    bridge_orange = model.material("bridge_orange", rgba=(0.95, 0.38, 0.08, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    linear_rail = model.material("linear_rail", rgba=(0.72, 0.74, 0.74, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.02, 0.025, 0.03, 1.0))
    tool_aluminum = model.material("tool_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    caution_yellow = model.material("caution_yellow", rgba=(1.0, 0.78, 0.05, 1.0))

    # Fixed machine base: two long rails tied together by bolted crossmembers.
    base = model.part("base")
    for i, x in enumerate((-0.78, 0.0, 0.78)):
        base.visual(
            Box((0.10, 1.34, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.0225)),
            material=painted_blue,
            name=f"crossmember_{i}",
        )
    for i, (y, rail_beam_name, guide_name) in enumerate(
        (
            (-0.50, "rail_beam_0", "guide_0"),
            (0.50, "rail_beam_1", "guide_1"),
        )
    ):
        base.visual(
            Box((2.15, 0.105, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.077)),
            material=dark_steel,
            name=rail_beam_name,
        )
        base.visual(
            Box((2.02, 0.035, 0.027)),
            origin=Origin(xyz=(0.0, y, 0.1235)),
            material=linear_rail,
            name=guide_name,
        )
        for j, x in enumerate((-1.03, 1.03)):
            base.visual(
                Box((0.060, 0.180, 0.115)),
                origin=Origin(xyz=(x, y, 0.145)),
                material=caution_yellow,
                name=f"end_stop_{i}_{j}",
            )

    # Moving portal bridge: side frames sit on the rail trucks and carry the beam.
    bridge = model.part("bridge")
    for i, (y, rail_shoe_name) in enumerate(
        ((-0.50, "rail_shoe_0"), (0.50, "rail_shoe_1"))
    ):
        bridge.visual(
            Box((0.280, 0.190, 0.072)),
            origin=Origin(xyz=(0.0, y, 0.173)),
            material=bridge_orange,
            name=rail_shoe_name,
        )
        for j, x in enumerate((-0.090, 0.090)):
            bridge.visual(
                Box((0.055, 0.058, 0.700)),
                origin=Origin(xyz=(x, y, 0.525)),
                material=bridge_orange,
                name=f"side_post_{i}_{j}",
            )
        bridge.visual(
            Box((0.038, 0.046, 0.720)),
            origin=Origin(xyz=(0.0, y, 0.525), rpy=(0.0, 0.26, 0.0)),
            material=bridge_orange,
            name=f"diagonal_brace_{i}_0",
        )
        bridge.visual(
            Box((0.038, 0.046, 0.720)),
            origin=Origin(xyz=(0.0, y, 0.525), rpy=(0.0, -0.26, 0.0)),
            material=bridge_orange,
            name=f"diagonal_brace_{i}_1",
        )
    bridge.visual(
        Box((0.180, 1.250, 0.145)),
        origin=Origin(xyz=(0.0, 0.0, 0.895)),
        material=bridge_orange,
        name="bridge_beam",
    )
    bridge.visual(
        Box((0.027, 1.050, 0.027)),
        origin=Origin(xyz=(-0.1035, 0.0, 0.855)),
        material=linear_rail,
        name="beam_rail_lower",
    )
    bridge.visual(
        Box((0.027, 1.050, 0.027)),
        origin=Origin(xyz=(-0.1035, 0.0, 0.935)),
        material=linear_rail,
        name="beam_rail_upper",
    )
    bridge.visual(
        Box((0.030, 0.110, 0.115)),
        origin=Origin(xyz=(-0.118, -0.575, 0.895)),
        material=caution_yellow,
        name="carriage_stop_0",
    )
    bridge.visual(
        Box((0.030, 0.110, 0.115)),
        origin=Origin(xyz=(-0.118, 0.575, 0.895)),
        material=caution_yellow,
        name="carriage_stop_1",
    )

    # Beam carriage: bearing trucks ride on the two linear guides and support
    # the vertical slide plate.
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.056, 0.320, 0.330)),
        origin=Origin(xyz=(-0.023, 0.0, 0.0)),
        material=dark_steel,
        name="carriage_plate",
    )
    for bearing_name, y, z in (
        ("bearing_0_0", -0.100, -0.040),
        ("bearing_0_1", 0.100, -0.040),
        ("bearing_1_0", -0.100, 0.040),
        ("bearing_1_1", 0.100, 0.040),
    ):
        carriage.visual(
            Box((0.022, 0.090, 0.052)),
            origin=Origin(xyz=(0.016, y, z)),
            material=bearing_black,
            name=bearing_name,
        )
    for cheek_name, y, z in (
        ("slide_cheek_0", -0.112, -0.125),
        ("slide_cheek_1", 0.112, -0.125),
        ("slide_cheek_2", -0.112, 0.125),
        ("slide_cheek_3", 0.112, 0.125),
    ):
        carriage.visual(
            Box((0.030, 0.055, 0.070)),
            origin=Origin(xyz=(-0.065, y, z)),
            material=painted_blue,
            name=cheek_name,
        )

    # Moving vertical ram and tool: the ram remains captured by the carriage
    # while the probe/spindle nose travels downward for routing or inspection.
    tool_head = model.part("tool_head")
    tool_head.visual(
        Box((0.082, 0.145, 0.560)),
        origin=Origin(xyz=(-0.020, 0.0, -0.120)),
        material=tool_aluminum,
        name="vertical_ram",
    )
    tool_head.visual(
        Box((0.095, 0.170, 0.070)),
        origin=Origin(xyz=(-0.020, 0.0, -0.425)),
        material=dark_steel,
        name="spindle_clamp",
    )
    tool_head.visual(
        Cylinder(radius=0.036, length=0.220),
        origin=Origin(xyz=(-0.020, 0.0, -0.555)),
        material=bearing_black,
        name="spindle_body",
    )
    tool_head.visual(
        Cylinder(radius=0.010, length=0.150),
        origin=Origin(xyz=(-0.020, 0.0, -0.740)),
        material=linear_rail,
        name="tool_probe",
    )

    model.articulation(
        "bridge_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.65, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "carriage_axis",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(-0.144, 0.0, 0.895)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.55, lower=-0.38, upper=0.38),
    )
    model.articulation(
        "tool_axis",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=tool_head,
        origin=Origin(xyz=(-0.072, 0.0, -0.030)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.30, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    carriage = object_model.get_part("carriage")
    tool_head = object_model.get_part("tool_head")
    bridge_axis = object_model.get_articulation("bridge_axis")
    carriage_axis = object_model.get_articulation("carriage_axis")
    tool_axis = object_model.get_articulation("tool_axis")

    # Rail-mounted bridge shoes sit on both fixed guide rails.
    for i, rail_shoe_name, guide_name in (
        (0, "rail_shoe_0", "guide_0"),
        (1, "rail_shoe_1", "guide_1"),
    ):
        ctx.expect_gap(
            bridge,
            base,
            axis="z",
            positive_elem=rail_shoe_name,
            negative_elem=guide_name,
            max_gap=0.001,
            max_penetration=1e-6,
            name=f"bridge shoe {i} sits on base guide",
        )
        ctx.expect_overlap(
            bridge,
            base,
            axes="xy",
            elem_a=rail_shoe_name,
            elem_b=guide_name,
            min_overlap=0.02,
            name=f"bridge shoe {i} overlaps guide footprint",
        )

    # The carriage trucks bear against the front linear rails of the beam.
    ctx.expect_gap(
        bridge,
        carriage,
        axis="x",
        positive_elem="beam_rail_lower",
        negative_elem="bearing_0_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower beam bearing contacts rail",
    )
    ctx.expect_gap(
        bridge,
        carriage,
        axis="x",
        positive_elem="beam_rail_upper",
        negative_elem="bearing_1_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper beam bearing contacts rail",
    )

    # The vertical ram is carried on the front face of the carriage slide mount.
    ctx.expect_gap(
        carriage,
        tool_head,
        axis="x",
        positive_elem="carriage_plate",
        negative_elem="vertical_ram",
        max_gap=0.001,
        max_penetration=1e-6,
        name="tool ram is seated on carriage",
    )

    rest_bridge_pos = ctx.part_world_position(bridge)
    with ctx.pose({bridge_axis: 0.40}):
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="rail_shoe_0",
            outer_elem="guide_0",
            margin=0.0,
            name="bridge remains on rail at travel limit",
        )
        moved_bridge_pos = ctx.part_world_position(bridge)
    ctx.check(
        "bridge moves along base rails",
        rest_bridge_pos is not None
        and moved_bridge_pos is not None
        and moved_bridge_pos[0] > rest_bridge_pos[0] + 0.35,
        details=f"rest={rest_bridge_pos}, moved={moved_bridge_pos}",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({carriage_axis: 0.32}):
        ctx.expect_within(
            carriage,
            bridge,
            axes="y",
            inner_elem="bearing_0_0",
            outer_elem="beam_rail_lower",
            margin=0.0,
            name="carriage bearing stays on beam rail",
        )
        moved_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "carriage moves along bridge beam",
        rest_carriage_pos is not None
        and moved_carriage_pos is not None
        and moved_carriage_pos[1] > rest_carriage_pos[1] + 0.25,
        details=f"rest={rest_carriage_pos}, moved={moved_carriage_pos}",
    )

    rest_tool_pos = ctx.part_world_position(tool_head)
    with ctx.pose({tool_axis: 0.14}):
        ctx.expect_overlap(
            tool_head,
            carriage,
            axes="z",
            elem_a="vertical_ram",
            elem_b="carriage_plate",
            min_overlap=0.015,
            name="extended vertical ram remains captured",
        )
        moved_tool_pos = ctx.part_world_position(tool_head)
    ctx.check(
        "tool head moves downward on vertical axis",
        rest_tool_pos is not None
        and moved_tool_pos is not None
        and moved_tool_pos[2] < rest_tool_pos[2] - 0.10,
        details=f"rest={rest_tool_pos}, moved={moved_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()
