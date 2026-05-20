from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


_CIRCLE_SEGMENTS = 72


def _circle_profile(radius: float, *, segments: int = _CIRCLE_SEGMENTS) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(2.0 * pi * index / segments),
            radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _ring_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    axis: str,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        thickness,
        center=True,
    )
    if axis == "y":
        geom.rotate_x(-pi / 2.0)
    elif axis == "x":
        geom.rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _plate_mesh(width: float, depth: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius),
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="miniature_precision_drill_press")

    machine_paint = model.material("machine_paint", rgba=(0.26, 0.35, 0.32, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.10, 0.11, 0.12, 1.0))

    base_plate = _plate_mesh(0.18, 0.14, 0.018, 0.016, "base_plate")
    table_plate = _plate_mesh(0.085, 0.060, 0.008, 0.006, "table_plate")
    chuck_mesh = mesh_from_geometry(ConeGeometry(radius=0.0070, height=0.014), "quill_chuck")

    base = model.part("base")
    base.visual(
        base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=machine_paint,
        name="base_plinth",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.011, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=machined_steel,
        name="column_shaft",
    )
    column.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="column_collar",
    )

    head = model.part("head")
    head.visual(
        Box((0.006, 0.038, 0.052)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0)),
        material=machine_paint,
        name="head_clamp_left",
    )
    head.visual(
        Box((0.006, 0.038, 0.052)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=machine_paint,
        name="head_clamp_right",
    )
    head.visual(
        Box((0.030, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=machine_paint,
        name="head_column_clamp",
    )
    head.visual(
        Box((0.030, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=machine_paint,
        name="head_clamp_back",
    )
    head.visual(
        Box((0.050, 0.040, 0.036)),
        origin=Origin(xyz=(0.0, 0.033, 0.018)),
        material=machine_paint,
        name="head_bridge",
    )
    head.visual(
        Box((0.090, 0.055, 0.046)),
        origin=Origin(xyz=(0.0, 0.073, 0.054)),
        material=machine_paint,
        name="head_body",
    )
    head.visual(
        Box((0.012, 0.024, 0.060)),
        origin=Origin(xyz=(-0.018, 0.079, 0.004)),
        material=machine_paint,
        name="head_left_web",
    )
    head.visual(
        Box((0.012, 0.024, 0.060)),
        origin=Origin(xyz=(0.018, 0.079, 0.004)),
        material=machine_paint,
        name="head_right_web",
    )
    head.visual(
        Box((0.028, 0.004, 0.046)),
        origin=Origin(xyz=(0.0, 0.0732, -0.026)),
        material=dark_steel,
        name="head_quill_nose",
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Box((0.006, 0.028, 0.032)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0)),
        material=machine_paint,
        name="carriage_sleeve_left",
    )
    table_carriage.visual(
        Box((0.006, 0.028, 0.032)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=machine_paint,
        name="carriage_sleeve_right",
    )
    table_carriage.visual(
        Box((0.028, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=machine_paint,
        name="carriage_sleeve",
    )
    table_carriage.visual(
        Box((0.028, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=machine_paint,
        name="carriage_sleeve_back",
    )
    table_carriage.visual(
        Box((0.022, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, 0.037, 0.000)),
        material=machine_paint,
        name="carriage_arm",
    )
    table_carriage.visual(
        Box((0.060, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.068, -0.008)),
        material=dark_steel,
        name="trunnion_block",
    )
    table_carriage.visual(
        Box((0.008, 0.018, 0.028)),
        origin=Origin(xyz=(-0.026, 0.068, 0.000)),
        material=dark_steel,
        name="left_pivot_cheek",
    )
    table_carriage.visual(
        Box((0.008, 0.018, 0.028)),
        origin=Origin(xyz=(0.026, 0.068, 0.000)),
        material=dark_steel,
        name="right_pivot_cheek",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.0046, length=0.044),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="table_pivot_shaft",
    )
    table.visual(
        Box((0.022, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, 0.004)),
        material=dark_steel,
        name="table_neck",
    )
    table.visual(
        Box((0.032, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.018, 0.002)),
        material=dark_steel,
        name="table_rib",
    )
    table.visual(
        table_plate,
        origin=Origin(xyz=(0.0, 0.036, 0.012)),
        material=machined_steel,
        name="table_top",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.0078, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=machined_steel,
        name="quill_body",
    )
    quill.visual(
        chuck_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=dark_steel,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.0015, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=blackened_steel,
        name="drill_bit",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, -0.032, 0.018)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
    )
    model.articulation(
        "column_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.15, lower=0.0, upper=0.070),
    )
    model.articulation(
        "table_carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.068, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, 0.083, -0.026)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.08, lower=0.0, upper=0.032),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    column = object_model.get_part("column")
    head = object_model.get_part("head")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    quill = object_model.get_part("quill")

    table_lift = object_model.get_articulation("column_to_table_carriage")
    table_tilt = object_model.get_articulation("table_carriage_to_table")
    quill_drop = object_model.get_articulation("head_to_quill")

    lift_upper = table_lift.motion_limits.upper if table_lift.motion_limits is not None else 0.0
    tilt_upper = table_tilt.motion_limits.upper if table_tilt.motion_limits is not None else 0.0
    quill_upper = quill_drop.motion_limits.upper if quill_drop.motion_limits is not None else 0.0

    ctx.expect_contact(
        column,
        head,
        elem_a="column_shaft",
        elem_b="head_clamp_left",
        name="left head clamp pad bears on the round column",
    )
    ctx.expect_contact(
        column,
        head,
        elem_a="column_shaft",
        elem_b="head_clamp_right",
        name="right head clamp pad bears on the round column",
    )
    ctx.expect_overlap(
        column,
        head,
        axes="z",
        elem_a="column_shaft",
        elem_b="head_clamp_left",
        min_overlap=0.045,
        name="head clamp wraps a meaningful length of the column",
    )

    with ctx.pose({table_lift: 0.0}):
        ctx.expect_contact(
            column,
            table_carriage,
            elem_a="column_shaft",
            elem_b="carriage_sleeve_left",
            name="lowered carriage left pad bears on the column",
        )
        ctx.expect_contact(
            column,
            table_carriage,
            elem_a="column_shaft",
            elem_b="carriage_sleeve_right",
            name="lowered carriage right pad bears on the column",
        )
        ctx.expect_overlap(
            column,
            table_carriage,
            axes="z",
            elem_a="column_shaft",
            elem_b="carriage_sleeve_left",
            min_overlap=0.025,
            name="lowered carriage sleeve spans a useful grip length on the column",
        )

    rest_table_pos = ctx.part_world_position(table)
    with ctx.pose({table_lift: lift_upper}):
        ctx.expect_contact(
            column,
            table_carriage,
            elem_a="column_shaft",
            elem_b="carriage_sleeve_left",
            name="raised carriage left pad still bears on the column",
        )
        ctx.expect_contact(
            column,
            table_carriage,
            elem_a="column_shaft",
            elem_b="carriage_sleeve_right",
            name="raised carriage right pad still bears on the column",
        )
        ctx.expect_overlap(
            column,
            table_carriage,
            axes="z",
            elem_a="column_shaft",
            elem_b="carriage_sleeve_left",
            min_overlap=0.025,
            name="raised carriage sleeve still spans a useful grip length on the column",
        )
        ctx.expect_gap(
            head,
            table,
            axis="z",
            min_gap=0.030,
            name="raised table still clears the spindle head",
        )
        raised_table_pos = ctx.part_world_position(table)
    ctx.check(
        "table lifts upward along the column",
        rest_table_pos is not None
        and raised_table_pos is not None
        and raised_table_pos[2] > rest_table_pos[2] + 0.05,
        details=f"rest={rest_table_pos}, raised={raised_table_pos}",
    )

    rest_top_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    with ctx.pose({table_tilt: tilt_upper}):
        tilted_top_aabb = ctx.part_element_world_aabb(table, elem="table_top")
    rest_top_height = None if rest_top_aabb is None else rest_top_aabb[1][2] - rest_top_aabb[0][2]
    tilted_top_height = None if tilted_top_aabb is None else tilted_top_aabb[1][2] - tilted_top_aabb[0][2]
    ctx.check(
        "table tilt articulation changes the table attitude",
        rest_top_height is not None
        and tilted_top_height is not None
        and tilted_top_height > rest_top_height + 0.02,
        details=f"rest_height={rest_top_height}, tilted_height={tilted_top_height}",
    )

    ctx.expect_contact(
        quill,
        head,
        elem_a="quill_body",
        elem_b="head_quill_nose",
        name="quill stays guided by the spindle nose pad",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="xz",
        elem_a="quill_body",
        elem_b="head_quill_nose",
        min_overlap=0.014,
        name="spindle nose pad overlaps the quill over a real guide patch",
    )
    rest_quill_pos = ctx.part_world_position(quill)
    with ctx.pose({quill_drop: quill_upper}):
        ctx.expect_contact(
            quill,
            head,
            elem_a="quill_body",
            elem_b="head_quill_nose",
            name="dropped quill remains guided by the spindle nose pad",
        )
        ctx.expect_overlap(
            quill,
            head,
            axes="xz",
            elem_a="quill_body",
            elem_b="head_quill_nose",
            min_overlap=0.014,
            name="dropped quill keeps a real guide patch at the spindle nose",
        )
        dropped_quill_pos = ctx.part_world_position(quill)
    ctx.check(
        "quill drops downward",
        rest_quill_pos is not None
        and dropped_quill_pos is not None
        and dropped_quill_pos[2] < rest_quill_pos[2] - 0.02,
        details=f"rest={rest_quill_pos}, dropped={dropped_quill_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
