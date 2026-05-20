from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="side_mounted_yz_positioner")

    tower_paint = model.material("graphite_painted_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    rail_metal = model.material("polished_linear_rail", rgba=(0.78, 0.80, 0.78, 1.0))
    carriage_blue = model.material("blue_anodized_carriage", rgba=(0.05, 0.20, 0.48, 1.0))
    tool_aluminum = model.material("brushed_tool_plate", rgba=(0.66, 0.67, 0.64, 1.0))
    dark_hardware = model.material("black_socket_hardware", rgba=(0.02, 0.02, 0.018, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.42, 0.36, 0.06)),
        origin=Origin(xyz=(0.0, 0.10, 0.03)),
        material=tower_paint,
        name="floor_base",
    )
    tower.visual(
        Box((0.16, 0.14, 1.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        material=tower_paint,
        name="back_tower",
    )
    tower.visual(
        Box((0.12, 0.08, 0.07)),
        origin=Origin(xyz=(0.04, 0.08, 1.05)),
        material=tower_paint,
        name="upper_rail_socket",
    )
    tower.visual(
        Box((0.12, 0.08, 0.07)),
        origin=Origin(xyz=(0.04, 0.08, 0.83)),
        material=tower_paint,
        name="lower_rail_socket",
    )
    tower.visual(
        Box((0.06, 0.86, 0.028)),
        origin=Origin(xyz=(0.04, 0.45, 0.94)),
        material=tower_paint,
        name="rail_spine",
    )
    tower.visual(
        Box((0.09, 0.05, 0.32)),
        origin=Origin(xyz=(0.04, 0.88, 0.94)),
        material=tower_paint,
        name="rail_end_bridge",
    )
    tower.visual(
        Cylinder(radius=0.018, length=0.86),
        origin=Origin(xyz=(0.04, 0.45, 1.05), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rail_metal,
        name="upper_y_rail",
    )
    tower.visual(
        Cylinder(radius=0.018, length=0.86),
        origin=Origin(xyz=(0.04, 0.45, 0.83), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rail_metal,
        name="lower_y_rail",
    )
    for x in (-0.045, 0.045):
        for z in (0.30, 1.15):
            tower.visual(
                Cylinder(radius=0.013, length=0.012),
                origin=Origin(xyz=(x, 0.076, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=dark_hardware,
                name=f"tower_bolt_{x}_{z}",
            )

    beam_carriage = model.part("beam_carriage")
    beam_carriage.visual(
        Box((0.04, 0.16, 0.34)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material=carriage_blue,
        name="bearing_backplate",
    )
    beam_carriage.visual(
        Box((0.035, 0.14, 0.055)),
        origin=Origin(xyz=(0.0355, 0.0, 0.11)),
        material=carriage_blue,
        name="upper_rail_shoe",
    )
    beam_carriage.visual(
        Box((0.035, 0.14, 0.055)),
        origin=Origin(xyz=(0.0355, 0.0, -0.11)),
        material=carriage_blue,
        name="lower_rail_shoe",
    )
    beam_carriage.visual(
        Box((0.46, 0.12, 0.08)),
        origin=Origin(xyz=(0.27, 0.0, 0.10)),
        material=carriage_blue,
        name="lateral_beam",
    )
    beam_carriage.visual(
        Box((0.40, 0.045, 0.10)),
        origin=Origin(xyz=(0.30, 0.0, 0.025)),
        material=carriage_blue,
        name="beam_rib",
    )
    beam_carriage.visual(
        Box((0.12, 0.34, 0.05)),
        origin=Origin(xyz=(0.50, 0.0, 0.045)),
        material=carriage_blue,
        name="z_guide_top_bridge",
    )
    beam_carriage.visual(
        Cylinder(radius=0.012, length=0.76),
        origin=Origin(xyz=(0.50, 0.13, -0.34)),
        material=rail_metal,
        name="z_guide_rail_0",
    )
    beam_carriage.visual(
        Cylinder(radius=0.012, length=0.76),
        origin=Origin(xyz=(0.50, -0.13, -0.34)),
        material=rail_metal,
        name="z_guide_rail_1",
    )
    beam_carriage.visual(
        Box((0.08, 0.32, 0.04)),
        origin=Origin(xyz=(0.50, 0.0, -0.72)),
        material=carriage_blue,
        name="z_guide_bottom_bridge",
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        Box((0.04, 0.21, 0.34)),
        origin=Origin(xyz=(0.07, 0.0, -0.22)),
        material=tool_aluminum,
        name="tool_mount_plate",
    )
    tool_plate.visual(
        Box((0.07, 0.25, 0.06)),
        origin=Origin(xyz=(0.055, 0.0, -0.055)),
        material=tool_aluminum,
        name="upper_slide_block",
    )
    tool_plate.visual(
        Box((0.04, 0.022, 0.09)),
        origin=Origin(xyz=(0.027, 0.112, -0.12)),
        material=tool_aluminum,
        name="upper_guide_pad_0",
    )
    tool_plate.visual(
        Box((0.04, 0.022, 0.09)),
        origin=Origin(xyz=(0.027, 0.112, -0.29)),
        material=tool_aluminum,
        name="lower_guide_pad_0",
    )
    tool_plate.visual(
        Box((0.02, 0.026, 0.26)),
        origin=Origin(xyz=(0.055, 0.112, -0.205)),
        material=tool_aluminum,
        name="guide_pad_spine_0",
    )
    tool_plate.visual(
        Box((0.04, 0.022, 0.09)),
        origin=Origin(xyz=(0.027, -0.112, -0.12)),
        material=tool_aluminum,
        name="upper_guide_pad_1",
    )
    tool_plate.visual(
        Box((0.04, 0.022, 0.09)),
        origin=Origin(xyz=(0.027, -0.112, -0.29)),
        material=tool_aluminum,
        name="lower_guide_pad_1",
    )
    tool_plate.visual(
        Box((0.02, 0.026, 0.26)),
        origin=Origin(xyz=(0.055, -0.112, -0.205)),
        material=tool_aluminum,
        name="guide_pad_spine_1",
    )
    tool_plate.visual(
        Box((0.06, 0.16, 0.07)),
        origin=Origin(xyz=(0.09, 0.0, -0.41)),
        material=tool_aluminum,
        name="tool_adapter_block",
    )
    tool_plate.visual(
        Cylinder(radius=0.035, length=0.035),
        origin=Origin(xyz=(0.13, 0.0, -0.41), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_hardware,
        name="round_tool_clamp",
    )
    for y in (-0.06, 0.06):
        for z in (-0.15, -0.29):
            tool_plate.visual(
                Cylinder(radius=0.010, length=0.010),
                origin=Origin(xyz=(0.095, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_hardware,
                name=f"plate_bolt_{y}_{z}",
            )

    model.articulation(
        "tower_to_beam",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=beam_carriage,
        origin=Origin(xyz=(0.04, 0.25, 0.94)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.42),
    )
    model.articulation(
        "beam_to_tool",
        ArticulationType.PRISMATIC,
        parent=beam_carriage,
        child=tool_plate,
        origin=Origin(xyz=(0.50, 0.0, 0.01)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beam = object_model.get_part("beam_carriage")
    tool = object_model.get_part("tool_plate")
    y_slide = object_model.get_articulation("tower_to_beam")
    z_slide = object_model.get_articulation("beam_to_tool")

    for pad_name, rail_name in (
        ("upper_guide_pad_0", "z_guide_rail_0"),
        ("lower_guide_pad_0", "z_guide_rail_0"),
        ("upper_guide_pad_1", "z_guide_rail_1"),
        ("lower_guide_pad_1", "z_guide_rail_1"),
    ):
        ctx.allow_overlap(
            tool,
            beam,
            elem_a=pad_name,
            elem_b=rail_name,
            reason="The guide pad is intentionally shown as a captured low-clearance slider around the vertical rail.",
        )

    ctx.expect_overlap(
        beam,
        tower,
        axes="y",
        elem_a="upper_rail_shoe",
        elem_b="upper_y_rail",
        min_overlap=0.10,
        name="beam carriage remains engaged on upper Y rail",
    )
    ctx.expect_overlap(
        beam,
        tower,
        axes="y",
        elem_a="lower_rail_shoe",
        elem_b="lower_y_rail",
        min_overlap=0.10,
        name="beam carriage remains engaged on lower Y rail",
    )
    ctx.expect_overlap(
        tool,
        beam,
        axes="z",
        elem_a="upper_guide_pad_0",
        elem_b="z_guide_rail_0",
        min_overlap=0.05,
        name="tool slide is captured by vertical guide rail",
    )
    ctx.expect_overlap(
        tool,
        beam,
        axes="z",
        elem_a="lower_guide_pad_0",
        elem_b="z_guide_rail_0",
        min_overlap=0.05,
        name="lower guide pad 0 remains on vertical rail",
    )
    ctx.expect_overlap(
        tool,
        beam,
        axes="z",
        elem_a="upper_guide_pad_1",
        elem_b="z_guide_rail_1",
        min_overlap=0.05,
        name="upper guide pad 1 remains on vertical rail",
    )
    ctx.expect_overlap(
        tool,
        beam,
        axes="z",
        elem_a="lower_guide_pad_1",
        elem_b="z_guide_rail_1",
        min_overlap=0.05,
        name="lower guide pad 1 remains on vertical rail",
    )

    rest_beam_pos = ctx.part_world_position(beam)
    rest_tool_pos = ctx.part_world_position(tool)
    with ctx.pose({y_slide: 0.42, z_slide: 0.30}):
        ctx.expect_overlap(
            beam,
            tower,
            axes="y",
            elem_a="upper_rail_shoe",
            elem_b="upper_y_rail",
            min_overlap=0.10,
            name="extended beam still rides on Y rail",
        )
        ctx.expect_overlap(
            tool,
            beam,
            axes="z",
            elem_a="lower_guide_pad_0",
            elem_b="z_guide_rail_0",
            min_overlap=0.05,
            name="lowered tool remains on vertical guide",
        )
        extended_beam_pos = ctx.part_world_position(beam)
        lowered_tool_pos = ctx.part_world_position(tool)

    ctx.check(
        "beam carriage travels horizontally along Y",
        rest_beam_pos is not None
        and extended_beam_pos is not None
        and extended_beam_pos[1] > rest_beam_pos[1] + 0.35,
        details=f"rest={rest_beam_pos}, extended={extended_beam_pos}",
    )
    ctx.check(
        "tool plate travels downward along Z",
        rest_tool_pos is not None
        and lowered_tool_pos is not None
        and lowered_tool_pos[2] < rest_tool_pos[2] - 0.25,
        details=f"rest={rest_tool_pos}, lowered={lowered_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()
