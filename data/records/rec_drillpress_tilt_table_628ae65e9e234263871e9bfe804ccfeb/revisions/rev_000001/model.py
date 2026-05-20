from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeWithHolesGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cnc_table_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.33, 0.36, 0.39, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_paint = model.material("dark_paint", rgba=(0.13, 0.14, 0.16, 1.0))
    accent = model.material("machine_accent", rgba=(0.58, 0.10, 0.08, 1.0))

    def rect_loop(width: float, height: float) -> list[tuple[float, float]]:
        hw = width * 0.5
        hh = height * 0.5
        return [
            (-hw, -hh),
            (hw, -hh),
            (hw, hh),
            (-hw, hh),
        ]

    model.part("base")
    base = model.get_part("base")
    base.visual(
        Box((0.56, 0.38, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="base_casting",
    )
    base.visual(
        Box((0.18, 0.11, 0.06)),
        origin=Origin(xyz=(0.0, 0.13, 0.08)),
        material=cast_iron,
        name="column_pedestal",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.03),
        origin=Origin(xyz=(0.0, 0.13, 0.125)),
        material=cast_iron,
        name="column_flange",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.58),
        origin=Origin(xyz=(0.0, 0.13, 0.40)),
        material=machined_steel,
        name="column",
    )
    base.visual(
        Box((0.38, 0.03, 0.025)),
        origin=Origin(xyz=(0.0, -0.08, 0.0625)),
        material=machined_steel,
        name="x_rail_front",
    )
    base.visual(
        Box((0.38, 0.03, 0.025)),
        origin=Origin(xyz=(0.0, 0.00, 0.0625)),
        material=machined_steel,
        name="x_rail_rear",
    )
    base.visual(
        Box((0.38, 0.02, 0.012)),
        origin=Origin(xyz=(0.0, -0.04, 0.056)),
        material=dark_paint,
        name="x_way_cover",
    )
    base.visual(
        Box((0.08, 0.08, 0.08)),
        origin=Origin(xyz=(0.23, -0.15, 0.09)),
        material=dark_paint,
        name="x_motor_housing",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.11),
        origin=Origin(xyz=(0.325, -0.15, 0.09), rpy=(0.0, 1.57079632679, 0.0)),
        material=accent,
        name="x_motor_cap",
    )

    x_saddle = model.part("x_saddle")
    x_saddle.visual(
        Box((0.32, 0.15, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="x_carriage_body",
    )
    x_saddle.visual(
        Box((0.03, 0.20, 0.02)),
        origin=Origin(xyz=(-0.08, 0.0, 0.06)),
        material=machined_steel,
        name="y_rail_left",
    )
    x_saddle.visual(
        Box((0.03, 0.20, 0.02)),
        origin=Origin(xyz=(0.08, 0.0, 0.06)),
        material=machined_steel,
        name="y_rail_right",
    )
    x_saddle.visual(
        Box((0.11, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, -0.11, 0.025)),
        material=dark_paint,
        name="y_motor_housing",
    )
    x_saddle.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.0, -0.1725, 0.025), rpy=(1.57079632679, 0.0, 0.0)),
        material=accent,
        name="y_motor_cap",
    )

    table_outer = rounded_rect_profile(0.34, 0.22, 0.012, corner_segments=6)
    table_slots = [
        rect_loop(0.018, 0.17),
        [(x - 0.09, y) for x, y in rect_loop(0.018, 0.17)],
        [(x + 0.09, y) for x, y in rect_loop(0.018, 0.17)],
    ]
    table_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            table_outer,
            table_slots,
            0.04,
            cap=True,
            center=True,
            closed=True,
        ),
        "drill_press_table",
    )

    y_table = model.part("y_table")
    y_table.visual(
        table_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=machined_steel,
        name="table_top",
    )
    y_table.visual(
        Box((0.18, 0.07, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast_iron,
        name="table_center_rib",
    )

    head = model.part("head")
    head.visual(
        Box((0.07, 0.16, 0.24)),
        origin=Origin(xyz=(-0.09, -0.125, 0.0)),
        material=cast_iron,
        name="head_left_cheek",
    )
    head.visual(
        Box((0.07, 0.16, 0.24)),
        origin=Origin(xyz=(0.09, -0.125, 0.0)),
        material=cast_iron,
        name="head_right_cheek",
    )
    head.visual(
        Box((0.25, 0.14, 0.06)),
        origin=Origin(xyz=(0.0, -0.125, 0.09)),
        material=cast_iron,
        name="head_top_bridge",
    )
    head.visual(
        Box((0.16, 0.045, 0.06)),
        origin=Origin(xyz=(0.0, -0.1025, -0.08)),
        material=cast_iron,
        name="head_lower_bridge",
    )
    head.visual(
        Box((0.19, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.095, 0.16)),
        material=dark_paint,
        name="motor_cover",
    )
    head.visual(
        Box((0.12, 0.05, 0.28)),
        origin=Origin(xyz=(0.0, -0.07, 0.0)),
        material=cast_iron,
        name="column_clamp_block",
    )
    head.visual(
        Box((0.014, 0.08, 0.18)),
        origin=Origin(xyz=(-0.039, -0.165, -0.03)),
        material=cast_iron,
        name="quill_guide_left",
    )
    head.visual(
        Box((0.014, 0.08, 0.18)),
        origin=Origin(xyz=(0.039, -0.165, -0.03)),
        material=cast_iron,
        name="quill_guide_right",
    )
    head.visual(
        Box((0.078, 0.014, 0.18)),
        origin=Origin(xyz=(0.0, -0.204, -0.03)),
        material=cast_iron,
        name="quill_guide_front",
    )
    head.visual(
        Box((0.078, 0.014, 0.18)),
        origin=Origin(xyz=(0.0, -0.126, -0.03)),
        material=cast_iron,
        name="quill_guide_rear",
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.032, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=machined_steel,
        name="quill_body",
    )
    quill.visual(
        Cylinder(radius=0.024, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.1675)),
        material=dark_paint,
        name="chuck_body",
    )
    quill.visual(
        Cylinder(radius=0.007, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=machined_steel,
        name="tool_stub",
    )

    model.articulation(
        "base_to_x_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_saddle,
        origin=Origin(xyz=(0.0, -0.04, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.08,
            lower=-0.10,
            upper=0.10,
        ),
    )
    model.articulation(
        "x_saddle_to_y_table",
        ArticulationType.PRISMATIC,
        parent=x_saddle,
        child=y_table,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.08,
            lower=-0.07,
            upper=0.07,
        ),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.FIXED,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.13, 0.57)),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, -0.165, 0.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.05,
            lower=0.0,
            upper=0.12,
        ),
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

    base = object_model.get_part("base")
    x_saddle = object_model.get_part("x_saddle")
    y_table = object_model.get_part("y_table")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")

    x_slide = object_model.get_articulation("base_to_x_saddle")
    y_slide = object_model.get_articulation("x_saddle_to_y_table")
    quill_slide = object_model.get_articulation("head_to_quill")

    with ctx.pose({x_slide: 0.0, y_slide: 0.0, quill_slide: 0.0}):
        ctx.expect_gap(
            x_saddle,
            base,
            axis="z",
            positive_elem="x_carriage_body",
            negative_elem="x_rail_front",
            max_gap=0.001,
            max_penetration=0.0,
            name="x saddle sits on the X rails",
        )
        ctx.expect_overlap(
            x_saddle,
            base,
            axes="x",
            elem_a="x_carriage_body",
            elem_b="x_rail_front",
            min_overlap=0.25,
            name="x saddle spans the X rails",
        )
        ctx.expect_gap(
            y_table,
            x_saddle,
            axis="z",
            positive_elem="table_center_rib",
            negative_elem="y_rail_left",
            max_gap=0.001,
            max_penetration=0.0,
            name="Y table rides on the Y rails",
        )
        ctx.expect_overlap(
            y_table,
            x_saddle,
            axes="y",
            elem_a="table_top",
            elem_b="y_rail_left",
            min_overlap=0.16,
            name="Y table retains travel on its left rail",
        )
        ctx.expect_gap(
            quill,
            head,
            axis="x",
            positive_elem="quill_body",
            negative_elem="quill_guide_left",
            max_gap=0.001,
            max_penetration=1e-6,
            name="quill bears against the left guide",
        )
        ctx.expect_gap(
            head,
            quill,
            axis="x",
            positive_elem="quill_guide_right",
            negative_elem="quill_body",
            max_gap=0.001,
            max_penetration=1e-6,
            name="quill bears against the right guide",
        )
        ctx.expect_gap(
            quill,
            head,
            axis="y",
            positive_elem="quill_body",
            negative_elem="quill_guide_front",
            max_gap=0.001,
            max_penetration=1e-6,
            name="quill bears against the front guide",
        )
        ctx.expect_gap(
            head,
            quill,
            axis="y",
            positive_elem="quill_guide_rear",
            negative_elem="quill_body",
            max_gap=0.001,
            max_penetration=1e-6,
            name="quill bears against the rear guide",
        )
        ctx.expect_overlap(
            quill,
            head,
            axes="z",
            elem_a="quill_body",
            elem_b="quill_guide_left",
            min_overlap=0.10,
            name="quill remains guided at rest",
        )

    x_rest = ctx.part_world_position(x_saddle)
    y_rest = ctx.part_world_position(y_table)
    quill_rest = ctx.part_world_position(quill)

    with ctx.pose({x_slide: x_slide.motion_limits.upper}):
        x_extended = ctx.part_world_position(x_saddle)
        ctx.check(
            "X axis moves in +X",
            x_rest is not None
            and x_extended is not None
            and x_extended[0] > x_rest[0] + 0.09,
            details=f"rest={x_rest}, extended={x_extended}",
        )

    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        y_extended = ctx.part_world_position(y_table)
        ctx.check(
            "Y axis moves in +Y",
            y_rest is not None
            and y_extended is not None
            and y_extended[1] > y_rest[1] + 0.06,
            details=f"rest={y_rest}, extended={y_extended}",
        )

    with ctx.pose({quill_slide: quill_slide.motion_limits.upper}):
        quill_extended = ctx.part_world_position(quill)
        ctx.check(
            "quill drops downward",
            quill_rest is not None
            and quill_extended is not None
            and quill_extended[2] < quill_rest[2] - 0.10,
            details=f"rest={quill_rest}, extended={quill_extended}",
        )
        ctx.expect_overlap(
            quill,
            head,
            axes="z",
            elem_a="quill_body",
            elem_b="quill_guide_left",
            min_overlap=0.02,
            name="quill retains insertion at full stroke",
        )

    ctx.expect_gap(
        base,
        head,
        axis="y",
        positive_elem="column",
        negative_elem="column_clamp_block",
        max_gap=0.001,
        max_penetration=0.0,
        name="head clamp seats against the round column",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
