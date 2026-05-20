from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _slotted_table_mesh():
    """Machined upper cross-slide table with three closed-bottom T slots."""
    table = _cq_box((0.50, 0.32, 0.060), (0.0, 0.0, 0.030))
    for y in (-0.085, 0.0, 0.085):
        # Narrow throat plus wider counterbore makes a recognizable T-slot while
        # leaving a continuous bottom web so the table remains one casting.
        throat = _cq_box((0.54, 0.014, 0.040), (0.0, y, 0.050))
        shoulder = _cq_box((0.54, 0.036, 0.018), (0.0, y, 0.059))
        table = table.cut(throat).cut(shoulder)
    return table


def _tube_mesh(outer_radius: float, inner_radius: float, length: float):
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_cross_slide_drill_table")

    cast = model.material("dark_cast_iron", rgba=(0.08, 0.10, 0.11, 1.0))
    blue = model.material("blue_machine_casting", rgba=(0.05, 0.18, 0.30, 1.0))
    steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.68, 1.0))
    black = model.material("blackened_ways", rgba=(0.01, 0.012, 0.014, 1.0))
    bit_mat = model.material("drill_bit_steel", rgba=(0.43, 0.45, 0.46, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.74, 0.62, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=cast,
        name="base_slab",
    )
    for x, way_name in ((-0.22, "y_way_0"), (0.22, "y_way_1")):
        base.visual(
            Box((0.075, 0.48, 0.020)),
            origin=Origin(xyz=(x, -0.075, 0.090)),
            material=black,
            name=way_name,
        )
    base.visual(
        Box((0.19, 0.12, 0.020)),
        origin=Origin(xyz=(0.0, 0.235, 0.090)),
        material=cast,
        name="column_seat",
    )
    base.visual(
        Box((0.090, 0.040, 0.100)),
        origin=Origin(xyz=(0.0, -0.290, 0.130)),
        material=cast,
        name="y_wheel_bracket",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.040, length=0.950),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=steel,
        name="round_column",
    )
    column.visual(
        Cylinder(radius=0.066, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=cast,
        name="column_foot",
    )

    lower_table = model.part("lower_table")
    lower_table.visual(
        Box((0.57, 0.30, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=blue,
        name="lower_saddle",
    )
    for y, way_name in ((-0.112, "x_way_0"), (0.112, "x_way_1")):
        lower_table.visual(
            Box((0.45, 0.040, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.060)),
            material=black,
            name=way_name,
        )
    lower_table.visual(
        Box((0.060, 0.25, 0.040)),
        origin=Origin(xyz=(-0.315, 0.0, 0.030)),
        material=blue,
        name="y_end_boss",
    )
    lower_table.visual(
        Box((0.070, 0.110, 0.080)),
        origin=Origin(xyz=(-0.320, 0.0, 0.075)),
        material=blue,
        name="x_wheel_boss",
    )

    upper_table = model.part("upper_table")
    upper_table.visual(
        mesh_from_cadquery(_slotted_table_mesh(), "upper_slotted_table"),
        material=blue,
        name="slotted_table",
    )

    drill_head = model.part("drill_head")
    drill_head.visual(
        Box((0.18, 0.060, 0.220)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=cast,
        name="column_clamp",
    )
    drill_head.visual(
        Box((0.30, 0.280, 0.180)),
        origin=Origin(xyz=(0.0, -0.180, 0.0)),
        material=blue,
        name="head_casting",
    )
    drill_head.visual(
        Box((0.20, 0.16, 0.080)),
        origin=Origin(xyz=(0.0, -0.170, 0.130)),
        material=blue,
        name="belt_cover",
    )
    drill_head.visual(
        mesh_from_cadquery(_tube_mesh(0.052, 0.030, 0.200), "quill_nose_sleeve"),
        origin=Origin(xyz=(0.0, -0.220, -0.290)),
        material=steel,
        name="nose_sleeve",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.020, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=steel,
        name="quill_bar",
    )
    quill.visual(
        Cylinder(radius=0.034, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.275)),
        material=cast,
        name="chuck",
    )
    quill.visual(
        Cylinder(radius=0.006, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.370)),
        material=bit_mat,
        name="drill_bit",
    )

    handwheel_mesh = KnobGeometry(
        0.078,
        0.020,
        body_style="faceted",
        top_diameter=0.066,
        base_diameter=0.078,
        grip=KnobGrip(style="fluted", count=24, depth=0.002),
    )

    y_handwheel = model.part("y_handwheel")
    y_handwheel.visual(
        mesh_from_geometry(handwheel_mesh, "y_handwheel_knob"),
        origin=Origin(rpy=(-1.57079632679, 0.0, 0.0)),
        material=black,
        name="knurled_wheel",
    )
    y_handwheel.visual(
        Cylinder(radius=0.007, length=0.055),
        origin=Origin(xyz=(0.0, 0.0275, 0.0), rpy=(-1.57079632679, 0.0, 0.0)),
        material=steel,
        name="shaft",
    )

    x_handwheel = model.part("x_handwheel")
    x_handwheel.visual(
        mesh_from_geometry(handwheel_mesh, "x_handwheel_knob"),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=black,
        name="knurled_wheel",
    )
    x_handwheel.visual(
        Cylinder(radius=0.007, length=0.035),
        origin=Origin(xyz=(0.0175, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=steel,
        name="shaft",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.235, 0.100)),
    )
    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_table,
        origin=Origin(xyz=(0.0, -0.075, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.08, lower=-0.080, upper=0.080),
    )
    model.articulation(
        "base_to_y_wheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=y_handwheel,
        origin=Origin(xyz=(0.0, -0.365, 0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower_table,
        child=upper_table,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.08, lower=-0.105, upper=0.105),
    )
    model.articulation(
        "lower_to_x_wheel",
        ArticulationType.CONTINUOUS,
        parent=lower_table,
        child=x_handwheel,
        origin=Origin(xyz=(-0.390, 0.0, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=column,
        child=drill_head,
        origin=Origin(xyz=(0.0, -0.040, 0.840)),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=drill_head,
        child=quill,
        origin=Origin(xyz=(0.0, -0.220, -0.090)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.12, lower=0.0, upper=0.110),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower = object_model.get_part("lower_table")
    upper = object_model.get_part("upper_table")
    head = object_model.get_part("drill_head")
    quill = object_model.get_part("quill")

    y_slide = object_model.get_articulation("base_to_lower")
    x_slide = object_model.get_articulation("lower_to_upper")
    q_slide = object_model.get_articulation("head_to_quill")

    ctx.expect_gap(
        lower,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_saddle",
        negative_elem="y_way_0",
        name="lower saddle sits on the Y ways",
    )
    ctx.expect_overlap(
        lower,
        base,
        axes="xy",
        min_overlap=0.20,
        name="lower saddle is retained over the base ways",
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="slotted_table",
        negative_elem="x_way_0",
        name="upper table sits on the X ways",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="xy",
        min_overlap=0.20,
        name="upper table footprint overlaps the saddle",
    )
    ctx.expect_within(
        quill,
        head,
        axes="xy",
        inner_elem="quill_bar",
        outer_elem="nose_sleeve",
        margin=0.001,
        name="quill is centered inside the nose sleeve",
    )
    ctx.expect_overlap(
        quill,
        head,
        axes="z",
        elem_a="quill_bar",
        elem_b="nose_sleeve",
        min_overlap=0.080,
        name="quill remains inserted in the head sleeve",
    )

    rest_lower = ctx.part_world_position(lower)
    rest_upper = ctx.part_world_position(upper)
    rest_quill = ctx.part_world_position(quill)
    with ctx.pose({y_slide: 0.060, x_slide: 0.075, q_slide: 0.080}):
        moved_lower = ctx.part_world_position(lower)
        moved_upper = ctx.part_world_position(upper)
        moved_quill = ctx.part_world_position(quill)
        ctx.expect_overlap(
            quill,
            head,
            axes="z",
            elem_a="quill_bar",
            elem_b="nose_sleeve",
            min_overlap=0.055,
            name="dropped quill is still captured by the sleeve",
        )

    ctx.check(
        "lower table travels along positive Y",
        rest_lower is not None and moved_lower is not None and moved_lower[1] > rest_lower[1] + 0.050,
        details=f"rest={rest_lower}, moved={moved_lower}",
    )
    ctx.check(
        "upper table travels along positive X",
        rest_upper is not None and moved_upper is not None and moved_upper[0] > rest_upper[0] + 0.065,
        details=f"rest={rest_upper}, moved={moved_upper}",
    )
    ctx.check(
        "quill drops downward from the head",
        rest_quill is not None and moved_quill is not None and moved_quill[2] < rest_quill[2] - 0.070,
        details=f"rest={rest_quill}, moved={moved_quill}",
    )

    return ctx.report()


object_model = build_object_model()
