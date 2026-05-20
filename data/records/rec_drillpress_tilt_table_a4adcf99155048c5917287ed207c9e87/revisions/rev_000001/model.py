from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mill_drill_combo_press")

    cast_iron = model.material("cast_iron", rgba=(0.34, 0.37, 0.40, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    black_paint = model.material("black_paint", rgba=(0.14, 0.15, 0.16, 1.0))
    tool_steel = model.material("tool_steel", rgba=(0.58, 0.60, 0.62, 1.0))

    def translated_profile(
        profile: list[tuple[float, float]],
        dx: float = 0.0,
        dy: float = 0.0,
    ) -> list[tuple[float, float]]:
        return [(x + dx, y + dy) for x, y in profile]

    base = model.part("base")
    base.visual(
        Box((0.56, 0.42, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=cast_iron,
        name="base_foot",
    )
    base.visual(
        Box((0.36, 0.30, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=cast_iron,
        name="base_plinth",
    )
    base.visual(
        Box((0.32, 0.26, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=machined_steel,
        name="slide_deck",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.06),
        origin=Origin(xyz=(0.0, -0.155, 0.28)),
        material=cast_iron,
        name="column_flange",
    )
    base.visual(
        Cylinder(radius=0.065, length=0.68),
        origin=Origin(xyz=(0.0, -0.155, 0.59)),
        material=cast_iron,
        name="round_column",
    )
    base.visual(
        Box((0.30, 0.12, 0.14)),
        origin=Origin(xyz=(0.0, -0.08, 0.80)),
        material=cast_iron,
        name="head_mount_block",
    )
    base.visual(
        Box((0.03, 0.05, 0.08)),
        origin=Origin(xyz=(-0.155, -0.015, 0.80)),
        material=machined_steel,
        name="left_trunnion_ear",
    )
    base.visual(
        Box((0.03, 0.05, 0.08)),
        origin=Origin(xyz=(0.155, -0.015, 0.80)),
        material=machined_steel,
        name="right_trunnion_ear",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.56, 0.42, 0.93)),
        mass=85.0,
        origin=Origin(xyz=(0.0, -0.04, 0.465)),
    )

    y_saddle = model.part("y_saddle")
    y_saddle.visual(
        Box((0.30, 0.24, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="y_saddle_body",
    )
    y_saddle.visual(
        Box((0.24, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=machined_steel,
        name="y_way_block",
    )
    y_saddle.visual(
        Box((0.18, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.105, 0.03)),
        material=cast_iron,
        name="front_apron",
    )
    y_saddle.inertial = Inertial.from_geometry(
        Box((0.30, 0.24, 0.085)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
    )

    table_outer = rounded_rect_profile(0.50, 0.20, 0.012, corner_segments=8)
    slot_profile = rounded_rect_profile(0.34, 0.016, 0.006, corner_segments=8)
    table_plate_geom = ExtrudeWithHolesGeometry(
        table_outer,
        (
            translated_profile(slot_profile, dy=-0.055),
            translated_profile(slot_profile, dy=0.0),
            translated_profile(slot_profile, dy=0.055),
        ),
        0.024,
        center=True,
        closed=True,
    )
    table_plate_mesh = mesh_from_geometry(table_plate_geom, "cross_slide_table_top")

    x_table = model.part("x_table")
    x_table.visual(
        Box((0.38, 0.12, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="table_rib",
    )
    x_table.visual(
        Box((0.05, 0.14, 0.028)),
        origin=Origin(xyz=(-0.215, 0.0, 0.016)),
        material=cast_iron,
        name="left_end_cap",
    )
    x_table.visual(
        Box((0.05, 0.14, 0.028)),
        origin=Origin(xyz=(0.215, 0.0, 0.016)),
        material=cast_iron,
        name="right_end_cap",
    )
    x_table.visual(
        table_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=machined_steel,
        name="table_top",
    )
    x_table.inertial = Inertial.from_geometry(
        Box((0.50, 0.20, 0.054)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.035, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="trunnion",
    )
    head.visual(
        Box((0.19, 0.08, 0.17)),
        origin=Origin(xyz=(0.0, 0.02, -0.12)),
        material=cast_iron,
        name="head_body",
    )
    head.visual(
        Box((0.10, 0.03, 0.08)),
        origin=Origin(xyz=(0.0, 0.065, -0.12)),
        material=cast_iron,
        name="quill_bridge",
    )
    head.visual(
        Cylinder(radius=0.055, length=0.16),
        origin=Origin(xyz=(0.0, 0.04, 0.03)),
        material=black_paint,
        name="motor_housing",
    )
    quill_sleeve_geom = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.055, -0.10), (0.055, 0.10)],
        inner_profile=[(0.034, -0.10), (0.034, 0.10)],
        segments=48,
    )
    quill_sleeve_mesh = mesh_from_geometry(quill_sleeve_geom, "quill_sleeve")
    head.visual(
        quill_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.12, -0.15)),
        material=machined_steel,
        name="quill_sleeve",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.28, 0.22, 0.32)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.05, -0.08)),
    )

    quill = model.part("quill")
    quill.visual(
        Cylinder(radius=0.028, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=machined_steel,
        name="quill_shaft",
    )
    quill.visual(
        Cylinder(radius=0.018, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, -0.17)),
        material=machined_steel,
        name="spindle_nose",
    )
    quill.visual(
        Cylinder(radius=0.006, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.22)),
        material=tool_steel,
        name="tool_stub",
    )
    quill.inertial = Inertial.from_geometry(
        Cylinder(radius=0.03, length=0.24),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    model.articulation(
        "base_to_y_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=y_saddle,
        origin=Origin(xyz=(0.0, 0.05, 0.27)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.10,
            lower=-0.08,
            upper=0.08,
        ),
    )
    model.articulation(
        "y_saddle_to_x_table",
        ArticulationType.PRISMATIC,
        parent=y_saddle,
        child=x_table,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.12,
            lower=-0.14,
            upper=0.14,
        ),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.04, 0.80)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.60,
            lower=math.radians(-25.0),
            upper=math.radians(30.0),
        ),
    )
    model.articulation(
        "head_to_quill",
        ArticulationType.PRISMATIC,
        parent=head,
        child=quill,
        origin=Origin(xyz=(0.0, 0.12, -0.05)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.10,
            lower=0.0,
            upper=0.06,
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
    y_saddle = object_model.get_part("y_saddle")
    x_table = object_model.get_part("x_table")
    head = object_model.get_part("head")
    quill = object_model.get_part("quill")

    y_slide = object_model.get_articulation("base_to_y_saddle")
    x_slide = object_model.get_articulation("y_saddle_to_x_table")
    head_tilt = object_model.get_articulation("base_to_head")
    quill_slide = object_model.get_articulation("head_to_quill")

    ctx.expect_gap(
        y_saddle,
        base,
        axis="z",
        positive_elem="y_saddle_body",
        negative_elem="slide_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="y saddle sits on the machined deck",
    )
    ctx.expect_gap(
        x_table,
        y_saddle,
        axis="z",
        positive_elem="table_rib",
        negative_elem="y_way_block",
        max_gap=0.001,
        max_penetration=0.0,
        name="x table sits on the saddle ways",
    )

    table_rest = ctx.part_world_position(x_table)
    with ctx.pose({x_slide: x_slide.motion_limits.upper}):
        table_x = ctx.part_world_position(x_table)
    ctx.check(
        "table translates along +x",
        table_rest is not None
        and table_x is not None
        and table_x[0] > table_rest[0] + 0.10,
        details=f"rest={table_rest}, x_travel={table_x}",
    )

    saddle_rest = ctx.part_world_position(y_saddle)
    with ctx.pose({y_slide: y_slide.motion_limits.upper}):
        saddle_y = ctx.part_world_position(y_saddle)
    ctx.check(
        "saddle translates along +y",
        saddle_rest is not None
        and saddle_y is not None
        and saddle_y[1] > saddle_rest[1] + 0.05,
        details=f"rest={saddle_rest}, y_travel={saddle_y}",
    )

    quill_rest = ctx.part_world_position(quill)
    with ctx.pose({head_tilt: math.radians(20.0)}):
        quill_tilted = ctx.part_world_position(quill)
    ctx.check(
        "head tilt swings the spindle nose downward",
        quill_rest is not None
        and quill_tilted is not None
        and quill_tilted[2] < quill_rest[2] - 0.02
        and quill_tilted[1] < quill_rest[1] - 0.01,
        details=f"rest={quill_rest}, tilted={quill_tilted}",
    )

    with ctx.pose({quill_slide: quill_slide.motion_limits.upper}):
        quill_dropped = ctx.part_world_position(quill)
    ctx.check(
        "quill drops along the spindle axis",
        quill_rest is not None
        and quill_dropped is not None
        and quill_dropped[2] < quill_rest[2] - 0.05,
        details=f"rest={quill_rest}, dropped={quill_dropped}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
