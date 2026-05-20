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
    model = ArticulatedObject(name="drill_press")

    # Frame (Base, Column, Head)
    frame = model.part("frame")
    frame.visual(
        Box((0.4, 0.6, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="base_plate",
    )
    frame.visual(
        Cylinder(radius=0.04, height=1.6),
        origin=Origin(xyz=(0.0, -0.2, 0.8)),
        name="column_tube",
    )
    frame.visual(
        Box((0.2, 0.6, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 1.5)),
        name="head_casing",
    )
    frame.visual(
        Cylinder(radius=0.02, height=0.15),
        origin=Origin(xyz=(0.0, 0.2, 1.35)),
        name="spindle",
    )

    # Table Bracket (slides vertically on column)
    table_bracket = model.part("table_bracket")
    table_bracket.visual(
        Cylinder(radius=0.05, height=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="bracket_sleeve",
    )
    table_bracket.visual(
        Box((0.08, 0.109, 0.08)),
        origin=Origin(xyz=(0.0, 0.0955, -0.04)),
        name="bracket_arm",
    )

    model.articulation(
        "table_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_bracket,
        origin=Origin(xyz=(0.0, -0.2, 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.5, lower=-0.3, upper=0.5),
    )

    # Table (tilts on bracket)
    table = model.part("table")
    table.visual(
        Box((0.4, 0.4, 0.03)),
        origin=Origin(xyz=(0.0, 0.2, 0.015)),
        name="table_top",
    )
    # Fence (fixed to table)
    table.visual(
        Box((0.4, 0.04, 0.06)),
        origin=Origin(xyz=(0.0, 0.05, 0.055)),
        name="fence_body",
    )
    # Fence rail
    table.visual(
        Box((0.4, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.05, 0.09)),
        name="fence_rail",
    )

    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=table,
        origin=Origin(xyz=(0.0, 0.15, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=-0.785, upper=0.785),
    )

    # Stop Block (slides on fence rail)
    stop_block = model.part("stop_block")
    stop_block.visual(
        Box((0.04, 0.04, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="block_top",
    )
    stop_block.visual(
        Box((0.04, 0.01, 0.06)),
        origin=Origin(xyz=(0.0, 0.025, -0.02)),
        name="block_face",
    )

    model.articulation(
        "stop_slide",
        ArticulationType.PRISMATIC,
        parent=table,
        child=stop_block,
        origin=Origin(xyz=(0.0, 0.05, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=-0.18, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "frame",
        "table_bracket",
        elem_a="column_tube",
        elem_b="bracket_sleeve",
        reason="The bracket sleeve is a proxy that slides over the column tube.",
    )

    ctx.expect_within(
        "frame",
        "table_bracket",
        axes="xy",
        inner_elem="column_tube",
        outer_elem="bracket_sleeve",
        margin=0.001,
        name="Bracket sleeve wraps column tube",
    )

    ctx.expect_overlap(
        "frame",
        "table_bracket",
        axes="z",
        elem_a="column_tube",
        elem_b="bracket_sleeve",
        min_overlap=0.1,
        name="Bracket stays on column",
    )

    ctx.expect_overlap(
        "table",
        "stop_block",
        axes="x",
        elem_a="fence_rail",
        elem_b="block_top",
        min_overlap=0.03,
        name="Stop block remains on rail",
    )

    with ctx.pose(stop_slide=0.15):
        ctx.expect_overlap(
            "table",
            "stop_block",
            axes="x",
            elem_a="fence_rail",
            elem_b="block_top",
            min_overlap=0.03,
            name="Stop block remains on rail when slid",
        )

    with ctx.pose(table_tilt=0.5):
        ctx.expect_overlap(
            "frame",
            "table_bracket",
            axes="z",
            elem_a="column_tube",
            elem_b="bracket_sleeve",
            min_overlap=0.1,
            name="Bracket stays on column when tilted",
        )

    return ctx.report()


object_model = build_object_model()
