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


def _mat(name: str, rgba: tuple[float, float, float, float]) -> Material:
    return Material(name=name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_slide_wrist_tool_bracket")

    frame_dark = _mat("dark powder coated steel", (0.08, 0.085, 0.09, 1.0))
    frame_edge = _mat("blackened frame edges", (0.025, 0.025, 0.03, 1.0))
    rail_steel = _mat("ground linear rail steel", (0.72, 0.74, 0.73, 1.0))
    screw_steel = _mat("polished lead screw steel", (0.78, 0.78, 0.76, 1.0))
    bearing_black = _mat("black linear bearing blocks", (0.015, 0.017, 0.02, 1.0))
    carriage_orange = _mat("orange anodized carriage", (0.95, 0.38, 0.05, 1.0))
    bracket_blue = _mat("blue anodized rotary bracket", (0.05, 0.17, 0.45, 1.0))
    tool_dark = _mat("matte black rotary tool body", (0.02, 0.022, 0.025, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.38, 0.28, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=frame_dark,
        name="base_plate",
    )
    frame.visual(
        Box((0.25, 0.024, 1.02)),
        origin=Origin(xyz=(0.0, 0.012, 0.535)),
        material=frame_dark,
        name="backplate",
    )
    frame.visual(
        Box((0.30, 0.05, 0.045)),
        origin=Origin(xyz=(0.0, 0.005, 1.065)),
        material=frame_edge,
        name="top_cap",
    )
    for x, name in ((-0.145, "side_post_0"), (0.145, "side_post_1")):
        frame.visual(
            Box((0.035, 0.04, 1.03)),
            origin=Origin(xyz=(x, 0.0, 0.535)),
            material=frame_edge,
            name=name,
        )
    frame.visual(
        Box((0.018, 0.014, 0.86)),
        origin=Origin(xyz=(-0.085, -0.003, 0.55)),
        material=rail_steel,
        name="rail_0",
    )
    frame.visual(
        Box((0.018, 0.014, 0.86)),
        origin=Origin(xyz=(0.085, -0.003, 0.55)),
        material=rail_steel,
        name="rail_1",
    )
    frame.visual(
        Cylinder(radius=0.008, length=0.90),
        origin=Origin(xyz=(0.0, -0.018, 0.55)),
        material=screw_steel,
        name="lead_screw",
    )
    for z, name in ((0.095, "lower_screw_bearing"), (1.005, "upper_screw_bearing")):
        frame.visual(
            Box((0.060, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, -0.008, z)),
            material=frame_edge,
            name=name,
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.020, 0.18)),
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=carriage_orange,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.045, 0.025, 0.052)),
        origin=Origin(xyz=(-0.085, -0.0225, 0.055)),
        material=bearing_black,
        name="upper_bearing_0",
    )
    carriage.visual(
        Box((0.045, 0.025, 0.052)),
        origin=Origin(xyz=(0.085, -0.0225, 0.055)),
        material=bearing_black,
        name="upper_bearing_1",
    )
    carriage.visual(
        Box((0.045, 0.025, 0.052)),
        origin=Origin(xyz=(-0.085, -0.0225, -0.055)),
        material=bearing_black,
        name="lower_bearing_0",
    )
    carriage.visual(
        Box((0.045, 0.025, 0.052)),
        origin=Origin(xyz=(0.085, -0.0225, -0.055)),
        material=bearing_black,
        name="lower_bearing_1",
    )
    carriage.visual(
        Box((0.15, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.052, 0.095)),
        material=carriage_orange,
        name="wrist_pedestal",
    )
    carriage.visual(
        Box((0.018, 0.055, 0.100)),
        origin=Origin(xyz=(-0.060, -0.062, 0.145)),
        material=carriage_orange,
        name="wrist_cheek_0",
    )
    carriage.visual(
        Box((0.018, 0.055, 0.100)),
        origin=Origin(xyz=(0.060, -0.062, 0.145)),
        material=carriage_orange,
        name="wrist_cheek_1",
    )

    tool_bracket = model.part("tool_bracket")
    tool_bracket.visual(
        Cylinder(radius=0.022, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=screw_steel,
        name="wrist_barrel",
    )
    tool_bracket.visual(
        Box((0.070, 0.026, 0.095)),
        origin=Origin(xyz=(0.0, -0.022, -0.065)),
        material=bracket_blue,
        name="drop_link",
    )
    tool_bracket.visual(
        Box((0.082, 0.050, 0.055)),
        origin=Origin(xyz=(0.0, -0.058, -0.105)),
        material=bracket_blue,
        name="split_clamp",
    )
    tool_bracket.visual(
        Cylinder(radius=0.024, length=0.160),
        origin=Origin(xyz=(0.0, -0.083, -0.130)),
        material=tool_dark,
        name="tool_body",
    )
    tool_bracket.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(0.0, -0.083, -0.237)),
        material=screw_steel,
        name="tool_collet",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.28, lower=-0.30, upper=0.18),
    )
    model.articulation(
        "carriage_to_tool_bracket",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tool_bracket,
        origin=Origin(xyz=(0.0, -0.062, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.6, lower=-1.10, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    tool_bracket = object_model.get_part("tool_bracket")
    slide = object_model.get_articulation("frame_to_carriage")
    wrist = object_model.get_articulation("carriage_to_tool_bracket")

    ctx.expect_gap(
        carriage,
        frame,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="base_plate",
        min_gap=0.55,
        name="carriage rests visibly lifted above the base",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a="rail_0",
        elem_b="upper_bearing_0",
        name="left upper bearing runs just in front of its guide rail",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a="rail_1",
        elem_b="upper_bearing_1",
        name="right upper bearing runs just in front of its guide rail",
    )
    ctx.expect_contact(
        tool_bracket,
        carriage,
        elem_a="wrist_barrel",
        elem_b="wrist_cheek_0",
        name="wrist barrel contacts the first hinge cheek",
    )
    ctx.expect_contact(
        tool_bracket,
        carriage,
        elem_a="wrist_barrel",
        elem_b="wrist_cheek_1",
        name="wrist barrel contacts the second hinge cheek",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.18}):
        raised_position = ctx.part_world_position(carriage)
        ctx.expect_contact(
            frame,
            carriage,
            elem_a="rail_1",
            elem_b="upper_bearing_1",
            name="raised carriage keeps guide clearance",
        )
    ctx.check(
        "vertical slide raises the carriage",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.15,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    def _center_z(elem_aabb):
        if elem_aabb is None:
            return None
        lower, upper = elem_aabb
        return 0.5 * (lower[2] + upper[2])

    rest_tool_z = _center_z(ctx.part_element_world_aabb(tool_bracket, elem="tool_body"))
    with ctx.pose({wrist: -0.8}):
        tilted_tool_z = _center_z(ctx.part_element_world_aabb(tool_bracket, elem="tool_body"))
    ctx.check(
        "single top wrist hinge pitches the rotary tool bracket",
        rest_tool_z is not None
        and tilted_tool_z is not None
        and tilted_tool_z > rest_tool_z + 0.05,
        details=f"rest_tool_z={rest_tool_z}, tilted_tool_z={tilted_tool_z}",
    )

    return ctx.report()


object_model = build_object_model()
