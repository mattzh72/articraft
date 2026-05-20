from __future__ import annotations

import math

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
    model = ArticulatedObject(name="side_transfer_stage")

    painted_steel = Material("painted_steel", rgba=(0.18, 0.22, 0.25, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.04, 0.05, 0.055, 1.0))
    machined = Material("machined_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    carriage_blue = Material("carriage_blue", rgba=(0.06, 0.22, 0.55, 1.0))
    tool_orange = Material("tool_orange", rgba=(0.92, 0.38, 0.08, 1.0))
    black = Material("black_hardware", rgba=(0.01, 0.01, 0.012, 1.0))

    frame = model.part("frame")

    # A side-mounted machine frame: a rigid back tower/wall with welded feet,
    # gussets, and a pair of long Y guide rails carried on proud mounting pads.
    frame.visual(
        Box((0.70, 1.44, 0.06)),
        origin=Origin(xyz=(0.00, 0.0, 0.03)),
        material=dark_steel,
        name="floor_foot",
    )
    frame.visual(
        Box((0.16, 1.34, 1.18)),
        origin=Origin(xyz=(-0.30, 0.0, 0.65)),
        material=painted_steel,
        name="back_tower",
    )
    frame.visual(
        Box((0.18, 1.42, 0.08)),
        origin=Origin(xyz=(-0.30, 0.0, 1.28)),
        material=dark_steel,
        name="top_cap",
    )
    frame.visual(
        Box((0.22, 0.08, 1.15)),
        origin=Origin(xyz=(-0.22, -0.67, 0.65)),
        material=dark_steel,
        name="end_upright_0",
    )
    frame.visual(
        Box((0.22, 0.08, 1.15)),
        origin=Origin(xyz=(-0.22, 0.67, 0.65)),
        material=dark_steel,
        name="end_upright_1",
    )
    frame.visual(
        Box((0.18, 0.28, 0.08)),
        origin=Origin(xyz=(-0.20, -0.67, 0.12)),
        material=dark_steel,
        name="foot_pad_0",
    )
    frame.visual(
        Box((0.18, 0.28, 0.08)),
        origin=Origin(xyz=(-0.20, 0.67, 0.12)),
        material=dark_steel,
        name="foot_pad_1",
    )
    frame.visual(
        Box((0.045, 1.24, 0.09)),
        origin=Origin(xyz=(-0.205, 0.0, 1.15)),
        material=dark_steel,
        name="upper_rail_pad",
    )
    frame.visual(
        Box((0.045, 1.24, 0.09)),
        origin=Origin(xyz=(-0.205, 0.0, 0.85)),
        material=dark_steel,
        name="lower_rail_pad",
    )
    frame.visual(
        Box((0.055, 1.22, 0.045)),
        origin=Origin(xyz=(-0.180, 0.0, 1.15)),
        material=machined,
        name="upper_y_rail",
    )
    frame.visual(
        Box((0.055, 1.22, 0.045)),
        origin=Origin(xyz=(-0.180, 0.0, 0.85)),
        material=machined,
        name="lower_y_rail",
    )
    frame.visual(
        Box((0.08, 0.05, 0.42)),
        origin=Origin(xyz=(-0.155, -0.635, 1.00)),
        material=black,
        name="y_end_stop_0",
    )
    frame.visual(
        Box((0.08, 0.05, 0.42)),
        origin=Origin(xyz=(-0.155, 0.635, 1.00)),
        material=black,
        name="y_end_stop_1",
    )

    crosshead = model.part("crosshead")
    crosshead.visual(
        Box((0.10, 0.32, 0.44)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_blue,
        name="carriage_backplate",
    )
    crosshead.visual(
        Box((0.040, 0.26, 0.075)),
        origin=Origin(xyz=(-0.0525, 0.0, 0.150)),
        material=machined,
        name="upper_bearing_shoe",
    )
    crosshead.visual(
        Box((0.040, 0.26, 0.075)),
        origin=Origin(xyz=(-0.0525, 0.0, -0.150)),
        material=machined,
        name="lower_bearing_shoe",
    )
    crosshead.visual(
        Box((0.34, 0.18, 0.08)),
        origin=Origin(xyz=(0.12, 0.0, 0.10)),
        material=carriage_blue,
        name="cantilever_arm",
    )
    crosshead.visual(
        Box((0.10, 0.32, 0.075)),
        origin=Origin(xyz=(0.23, 0.0, 0.035)),
        material=carriage_blue,
        name="guide_top_clamp",
    )
    crosshead.visual(
        Cylinder(radius=0.012, length=0.72),
        origin=Origin(xyz=(0.23, -0.11, -0.31)),
        material=machined,
        name="vertical_guide_0",
    )
    crosshead.visual(
        Cylinder(radius=0.012, length=0.72),
        origin=Origin(xyz=(0.23, 0.11, -0.31)),
        material=machined,
        name="vertical_guide_1",
    )
    crosshead.visual(
        Box((0.05, 0.035, 0.035)),
        origin=Origin(xyz=(0.23, -0.11, -0.655)),
        material=black,
        name="z_stop_0",
    )
    crosshead.visual(
        Box((0.05, 0.035, 0.035)),
        origin=Origin(xyz=(0.23, 0.11, -0.655)),
        material=black,
        name="z_stop_1",
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        Box((0.060, 0.30, 0.25)),
        origin=Origin(xyz=(0.085, 0.0, -0.23)),
        material=tool_orange,
        name="front_plate",
    )
    tool_plate.visual(
        Box((0.050, 0.060, 0.12)),
        origin=Origin(xyz=(0.037, -0.11, -0.18)),
        material=machined,
        name="guide_shoe_0",
    )
    tool_plate.visual(
        Box((0.050, 0.060, 0.12)),
        origin=Origin(xyz=(0.037, 0.11, -0.18)),
        material=machined,
        name="guide_shoe_1",
    )
    tool_plate.visual(
        Box((0.10, 0.18, 0.08)),
        origin=Origin(xyz=(0.10, 0.0, -0.385)),
        material=dark_steel,
        name="tool_mount",
    )
    tool_plate.visual(
        Cylinder(radius=0.030, length=0.10),
        origin=Origin(xyz=(0.10, 0.0, -0.472)),
        material=black,
        name="tool_socket",
    )
    for bolt_index, (bolt_y, bolt_z) in enumerate(
        ((-0.10, -0.15), (0.10, -0.15), (-0.10, -0.30), (0.10, -0.30))
    ):
        tool_plate.visual(
            Cylinder(radius=0.012, length=0.012),
            origin=Origin(
                xyz=(0.121, bolt_y, bolt_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black,
            name=f"plate_bolt_{bolt_index}",
        )

    model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(-0.080, -0.40, 1.00)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.55, lower=0.0, upper=0.80),
    )
    model.articulation(
        "crosshead_to_tool_plate",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=tool_plate,
        origin=Origin(xyz=(0.23, 0.0, -0.05)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.35, lower=0.0, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crosshead = object_model.get_part("crosshead")
    tool_plate = object_model.get_part("tool_plate")
    y_slide = object_model.get_articulation("frame_to_crosshead")
    z_slide = object_model.get_articulation("crosshead_to_tool_plate")

    ctx.check(
        "crosshead uses a Y prismatic slide",
        y_slide.articulation_type == ArticulationType.PRISMATIC and tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"type={y_slide.articulation_type}, axis={y_slide.axis}",
    )
    ctx.check(
        "tool plate uses a descending Z prismatic slide",
        z_slide.articulation_type == ArticulationType.PRISMATIC and tuple(z_slide.axis) == (0.0, 0.0, -1.0),
        details=f"type={z_slide.articulation_type}, axis={z_slide.axis}",
    )

    ctx.expect_within(
        crosshead,
        frame,
        axes="y",
        inner_elem="upper_bearing_shoe",
        outer_elem="upper_y_rail",
        margin=0.0,
        name="carriage starts captured on the upper Y guide",
    )
    ctx.expect_within(
        crosshead,
        frame,
        axes="y",
        inner_elem="lower_bearing_shoe",
        outer_elem="lower_y_rail",
        margin=0.0,
        name="carriage starts captured on the lower Y guide",
    )
    ctx.expect_overlap(
        tool_plate,
        crosshead,
        axes="z",
        elem_a="guide_shoe_0",
        elem_b="vertical_guide_0",
        min_overlap=0.08,
        name="tool plate starts engaged on one vertical guide",
    )
    ctx.expect_overlap(
        tool_plate,
        crosshead,
        axes="z",
        elem_a="guide_shoe_1",
        elem_b="vertical_guide_1",
        min_overlap=0.08,
        name="tool plate starts engaged on the second vertical guide",
    )

    rest_crosshead = ctx.part_world_position(crosshead)
    with ctx.pose({y_slide: 0.80}):
        extended_crosshead = ctx.part_world_position(crosshead)
        ctx.expect_within(
            crosshead,
            frame,
            axes="y",
            inner_elem="upper_bearing_shoe",
            outer_elem="upper_y_rail",
            margin=0.0,
            name="carriage remains captured at full Y travel",
        )

    ctx.check(
        "crosshead moves laterally in +Y",
        rest_crosshead is not None
        and extended_crosshead is not None
        and extended_crosshead[1] > rest_crosshead[1] + 0.70,
        details=f"rest={rest_crosshead}, extended={extended_crosshead}",
    )

    rest_tool = ctx.part_world_position(tool_plate)
    with ctx.pose({z_slide: 0.28}):
        lowered_tool = ctx.part_world_position(tool_plate)
        ctx.expect_overlap(
            tool_plate,
            crosshead,
            axes="z",
            elem_a="guide_shoe_0",
            elem_b="vertical_guide_0",
            min_overlap=0.08,
            name="tool plate remains guided at full down stroke",
        )

    ctx.check(
        "tool plate descends in -Z",
        rest_tool is not None
        and lowered_tool is not None
        and lowered_tool[2] < rest_tool[2] - 0.24,
        details=f"rest={rest_tool}, lowered={lowered_tool}",
    )

    return ctx.report()


object_model = build_object_model()
