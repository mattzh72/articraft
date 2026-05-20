from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _make_base(
    base_len: float,
    base_w: float,
    base_h: float,
    rail_len: float,
    rail_w: float,
    rail_h: float,
    rail_y: float,
) -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(base_len, base_w, base_h, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(base_len - 0.08, 0.09)
        .cutBlind(-0.008)
    )

    rail_left = (
        cq.Workplane("XY")
        .box(rail_len, rail_w, rail_h, centered=(True, True, False))
        .translate((0.0, rail_y, base_h))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(rail_len, rail_w, rail_h, centered=(True, True, False))
        .translate((0.0, -rail_y, base_h))
    )

    return body.union(rail_left).union(rail_right)


def _make_x_saddle(
    body_len: float,
    body_w: float,
    lower_h: float,
    deck_h: float,
    upper_rail_len: float,
    upper_rail_w: float,
    upper_rail_h: float,
    upper_rail_x: float,
) -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(body_len, body_w, lower_h, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    deck = (
        cq.Workplane("XY")
        .box(body_len - 0.03, body_w - 0.03, deck_h, centered=(True, True, False))
        .translate((0.0, 0.0, lower_h))
    )
    rail_left = (
        cq.Workplane("XY")
        .box(upper_rail_w, upper_rail_len, upper_rail_h, centered=(True, True, False))
        .translate((upper_rail_x, 0.0, lower_h + deck_h))
    )
    rail_right = (
        cq.Workplane("XY")
        .box(upper_rail_w, upper_rail_len, upper_rail_h, centered=(True, True, False))
        .translate((-upper_rail_x, 0.0, lower_h + deck_h))
    )
    saddle = lower.union(deck).union(rail_left).union(rail_right)
    saddle = (
        saddle.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.05, upper_rail_len - 0.03)
        .cutBlind(-0.006)
    )
    return saddle


def _make_y_table(
    guide_x: float,
    guide_y: float,
    guide_h: float,
    table_x: float,
    table_y: float,
    table_h: float,
) -> cq.Workplane:
    guide = (
        cq.Workplane("XY")
        .box(guide_x, guide_y, guide_h, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    table = (
        cq.Workplane("XY")
        .box(table_x, table_y, table_h, centered=(True, True, False))
        .translate((0.0, 0.0, guide_h))
    )
    top = guide.union(table)
    top = (
        top.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.05, 0.0), (0.0, 0.0), (0.05, 0.0)])
        .rect(0.012, 0.16)
        .cutBlind(-0.006)
    )
    return top


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xy_stage")

    base_len = 0.36
    base_w = 0.24
    base_h = 0.04
    base_rail_len = 0.32
    base_rail_w = 0.03
    base_rail_h = 0.018
    base_rail_y = 0.075

    x_body_len = 0.19
    x_body_w = 0.17
    x_lower_h = 0.026
    x_deck_h = 0.010
    x_upper_rail_len = 0.18
    x_upper_rail_w = 0.026
    x_upper_rail_h = 0.016
    x_upper_rail_x = 0.048

    y_guide_x = 0.13
    y_guide_y = 0.16
    y_guide_h = 0.010
    y_table_x = 0.19
    y_table_y = 0.22
    y_table_h = 0.020

    x_travel = 0.08
    y_travel = 0.06

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.25, 0.28, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.60, 0.62, 0.65, 1.0))
    table_finish = model.material("table_finish", rgba=(0.74, 0.76, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(
            _make_base(
                base_len=base_len,
                base_w=base_w,
                base_h=base_h,
                rail_len=base_rail_len,
                rail_w=base_rail_w,
                rail_h=base_rail_h,
                rail_y=base_rail_y,
            ),
            "xy_stage_base",
        ),
        material=cast_iron,
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_len, base_w, base_h + base_rail_h)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, (base_h + base_rail_h) * 0.5)),
    )

    x_saddle = model.part("x_saddle")
    x_saddle.visual(
        mesh_from_cadquery(
            _make_x_saddle(
                body_len=x_body_len,
                body_w=x_body_w,
                lower_h=x_lower_h,
                deck_h=x_deck_h,
                upper_rail_len=x_upper_rail_len,
                upper_rail_w=x_upper_rail_w,
                upper_rail_h=x_upper_rail_h,
                upper_rail_x=x_upper_rail_x,
            ),
            "xy_stage_x_saddle",
        ),
        material=machined_steel,
        name="carriage_body",
    )
    x_saddle.inertial = Inertial.from_geometry(
        Box((x_body_len, x_body_w, x_lower_h + x_deck_h + x_upper_rail_h)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, (x_lower_h + x_deck_h + x_upper_rail_h) * 0.5)),
    )

    y_table = model.part("y_table")
    y_table.visual(
        mesh_from_cadquery(
            _make_y_table(
                guide_x=y_guide_x,
                guide_y=y_guide_y,
                guide_h=y_guide_h,
                table_x=y_table_x,
                table_y=y_table_y,
                table_h=y_table_h,
            ),
            "xy_stage_y_table",
        ),
        material=table_finish,
        name="top_table",
    )
    y_table.inertial = Inertial.from_geometry(
        Box((y_table_x, y_table_y, y_guide_h + y_table_h)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, (y_guide_h + y_table_h) * 0.5)),
    )

    model.articulation(
        "base_to_x_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_saddle,
        origin=Origin(xyz=(0.0, 0.0, base_h + base_rail_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=-x_travel,
            upper=x_travel,
        ),
    )
    model.articulation(
        "x_saddle_to_y_table",
        ArticulationType.PRISMATIC,
        parent=x_saddle,
        child=y_table,
        origin=Origin(xyz=(0.0, 0.0, x_lower_h + x_deck_h + x_upper_rail_h)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.25,
            lower=-y_travel,
            upper=y_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_saddle = object_model.get_part("x_saddle")
    y_table = object_model.get_part("y_table")
    x_slide = object_model.get_articulation("base_to_x_saddle")
    y_slide = object_model.get_articulation("x_saddle_to_y_table")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "x_slide_axis_is_world_x",
        tuple(x_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected X-axis prismatic slide, got {x_slide.axis}",
    )
    ctx.check(
        "y_slide_axis_is_world_y",
        tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected Y-axis prismatic slide, got {y_slide.axis}",
    )

    ctx.expect_contact(
        x_saddle,
        base,
        contact_tol=2e-4,
        name="x_saddle_supported_on_base_rails",
    )
    ctx.expect_contact(
        y_table,
        x_saddle,
        contact_tol=2e-4,
        name="y_table_supported_on_cross_slide",
    )
    ctx.expect_overlap(
        x_saddle,
        base,
        axes="xy",
        min_overlap=0.10,
        name="x_saddle_overlaps_base_footprint",
    )
    ctx.expect_overlap(
        y_table,
        x_saddle,
        axes="xy",
        min_overlap=0.09,
        name="y_table_overlaps_cross_slide_footprint",
    )
    ctx.expect_gap(
        y_table,
        base,
        axis="z",
        min_gap=0.04,
        name="top_table_stands_above_grounded_base",
    )

    home_x = ctx.part_world_position(x_saddle)
    home_y = ctx.part_world_position(y_table)
    x_upper = x_slide.motion_limits.upper
    y_upper = y_slide.motion_limits.upper
    if home_x is not None and home_y is not None and x_upper is not None and y_upper is not None:
        with ctx.pose({x_slide: x_upper}):
            moved_x = ctx.part_world_position(x_saddle)
            ctx.check(
                "x_prismatic_motion_translates_only_along_x",
                moved_x is not None
                and moved_x[0] > home_x[0] + 0.075
                and abs(moved_x[1] - home_x[1]) < 1e-6
                and abs(moved_x[2] - home_x[2]) < 1e-6,
                details=f"home={home_x}, moved={moved_x}",
            )

        with ctx.pose({y_slide: y_upper}):
            moved_y = ctx.part_world_position(y_table)
            ctx.check(
                "y_prismatic_motion_translates_only_along_y",
                moved_y is not None
                and moved_y[1] > home_y[1] + 0.055
                and abs(moved_y[0] - home_y[0]) < 1e-6
                and abs(moved_y[2] - home_y[2]) < 1e-6,
                details=f"home={home_y}, moved={moved_y}",
            )

        with ctx.pose({x_slide: x_upper, y_slide: y_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_at_positive_travel_limits")
            ctx.expect_contact(
                x_saddle,
                base,
                contact_tol=2e-4,
                name="x_saddle_remains_supported_at_limit",
            )
            ctx.expect_contact(
                y_table,
                x_saddle,
                contact_tol=2e-4,
                name="y_table_remains_supported_at_limit",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
