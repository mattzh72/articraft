from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


FRAME_W = 0.92
FRAME_H = 1.08
FRAME_T = 0.05

FRAME_RAIL_LEN = 0.80
FRAME_RAIL_D = 0.014
FRAME_RAIL_H = 0.018
FRAME_RAIL_Y = (FRAME_T / 2.0) + (FRAME_RAIL_D / 2.0)
UPPER_RAIL_Z = 0.06
LOWER_RAIL_Z = -0.06
FRAME_TO_CROSSHEAD_Y = FRAME_RAIL_Y + (FRAME_RAIL_D / 2.0)
FRAME_TO_CROSSHEAD_Z = 0.16

CROSSHEAD_W = 0.46
CROSSHEAD_D = 0.072
CROSSHEAD_H = 0.14
SHOE_W = 0.11
SHOE_D = 0.024
SHOE_H = 0.042
SHOE_X = 0.145
GUIDE_RAIL_X = 0.051
GUIDE_RAIL_W = 0.012
GUIDE_RAIL_D = 0.014
GUIDE_RAIL_H = 0.40
GUIDE_RAIL_Y = 0.050
GUIDE_RAIL_Z = -0.17

SLIDE_BLOCK_W = 0.090
SLIDE_BLOCK_D = 0.030
SLIDE_BLOCK_H = 0.100
SLIDE_COLUMN_W = 0.078
SLIDE_COLUMN_D = 0.050
SLIDE_COLUMN_H = 0.340
TOOL_PLATE_W = 0.160
TOOL_PLATE_D = 0.046
TOOL_PLATE_H = 0.018

X_TRAVEL = 0.24
Z_TRAVEL = 0.22


def _frame_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(FRAME_W, FRAME_T, FRAME_H)

    front_bezel = (
        cq.Workplane("XY")
        .box(FRAME_W - 0.08, 0.026, FRAME_H - 0.08)
        .translate((0.0, (FRAME_T / 2.0) + 0.013, 0.0))
    )
    left_service_pod = (
        cq.Workplane("XY")
        .box(0.12, 0.020, FRAME_H - 0.18)
        .translate((-FRAME_W / 2.0 + 0.11, (FRAME_T / 2.0) + 0.010, -0.02))
    )
    right_service_pod = (
        cq.Workplane("XY")
        .box(0.12, 0.020, FRAME_H - 0.18)
        .translate((FRAME_W / 2.0 - 0.11, (FRAME_T / 2.0) + 0.010, -0.02))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(FRAME_W - 0.16, 0.018, 0.11)
        .translate((0.0, (FRAME_T / 2.0) + 0.013, FRAME_H / 2.0 - 0.11))
    )
    lower_service_boss = (
        cq.Workplane("XY")
        .box(FRAME_W - 0.26, 0.016, 0.10)
        .translate((0.0, (FRAME_T / 2.0) + 0.011, -FRAME_H / 2.0 + 0.10))
    )

    panel = body.union(front_bezel).union(left_service_pod).union(right_service_pod).union(top_cap).union(lower_service_boss)

    front_recess = (
        cq.Workplane("XY")
        .box(FRAME_W - 0.24, 0.040, FRAME_H - 0.30)
        .translate((0.0, (FRAME_T / 2.0) + 0.008, -0.03))
    )
    cable_recess = (
        cq.Workplane("XY")
        .box(0.08, 0.044, FRAME_H - 0.36)
        .translate((-FRAME_W / 2.0 + 0.13, (FRAME_T / 2.0) + 0.010, -0.05))
    )
    service_window = (
        cq.Workplane("XY")
        .box(FRAME_W - 0.36, 0.034, 0.14)
        .translate((0.0, (FRAME_T / 2.0) + 0.010, -FRAME_H / 2.0 + 0.19))
    )
    return panel.cut(front_recess).cut(cable_recess).cut(service_window)


def _crosshead_body_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(CROSSHEAD_W, CROSSHEAD_D, CROSSHEAD_H, centered=(True, False, True))
    beam = (
        beam.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rect(CROSSHEAD_W - 0.12, CROSSHEAD_H - 0.05)
        .cutBlind(-0.014)
    )

    front_cap = (
        cq.Workplane("XY")
        .box(CROSSHEAD_W - 0.08, 0.018, 0.080, centered=(True, False, True))
        .translate((0.0, CROSSHEAD_D + 0.009, 0.0))
    )
    lower_mount = (
        cq.Workplane("XY")
        .box(0.18, 0.040, 0.060, centered=(True, False, True))
        .translate((0.0, 0.040, -0.080))
    )
    top_service_cover = (
        cq.Workplane("XY")
        .box(0.16, 0.026, 0.090, centered=(True, False, True))
        .translate((0.0, CROSSHEAD_D - 0.010, 0.020))
    )
    return beam.union(front_cap).union(lower_mount).union(top_service_cover)


def _slide_column_shape() -> cq.Workplane:
    column = (
        cq.Workplane("XY")
        .box(SLIDE_COLUMN_W, SLIDE_COLUMN_D, SLIDE_COLUMN_H, centered=(True, False, False))
        .translate((0.0, 0.000, -SLIDE_BLOCK_H / 2.0 - SLIDE_COLUMN_H))
    )
    return (
        column.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rect(SLIDE_COLUMN_W - 0.05, SLIDE_COLUMN_H - 0.11)
        .cutBlind(-0.010)
    )


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_panel_axis")

    model.material("panel_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("rail_steel", rgba=(0.44, 0.47, 0.51, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("slide_silver", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("accent_blue", rgba=(0.18, 0.39, 0.69, 1.0))
    model.material("tool_dark", rgba=(0.24, 0.26, 0.29, 1.0))

    frame = model.part("back_frame")
    frame.visual(
        mesh_from_cadquery(_frame_body_shape(), "service_panel_frame"),
        material="panel_gray",
        name="body_shell",
    )
    _add_box_visual(
        frame,
        (FRAME_RAIL_LEN, FRAME_RAIL_D, FRAME_RAIL_H),
        (0.0, FRAME_RAIL_Y, UPPER_RAIL_Z + FRAME_TO_CROSSHEAD_Z),
        material="rail_steel",
        name="upper_rail",
    )
    _add_box_visual(
        frame,
        (FRAME_RAIL_LEN, FRAME_RAIL_D, FRAME_RAIL_H),
        (0.0, FRAME_RAIL_Y, LOWER_RAIL_Z + FRAME_TO_CROSSHEAD_Z),
        material="rail_steel",
        name="lower_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, FRAME_T + 0.03, FRAME_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(_crosshead_body_shape(), "service_panel_crosshead"),
        material="carriage_dark",
        name="body_shell",
    )
    for sign_x, x_label in ((-1.0, "left"), (1.0, "right")):
        _add_box_visual(
            crosshead,
            (SHOE_W, SHOE_D, SHOE_H),
            (sign_x * SHOE_X, SHOE_D / 2.0, UPPER_RAIL_Z),
            material="rail_steel",
            name=f"upper_{x_label}_shoe",
        )
        _add_box_visual(
            crosshead,
            (SHOE_W, SHOE_D, SHOE_H),
            (sign_x * SHOE_X, SHOE_D / 2.0, LOWER_RAIL_Z),
            material="rail_steel",
            name=f"lower_{x_label}_shoe",
        )
    _add_box_visual(
        crosshead,
        (GUIDE_RAIL_W, GUIDE_RAIL_D, GUIDE_RAIL_H),
        (-GUIDE_RAIL_X, GUIDE_RAIL_Y, GUIDE_RAIL_Z),
        material="rail_steel",
        name="left_guide_rail",
    )
    _add_box_visual(
        crosshead,
        (GUIDE_RAIL_W, GUIDE_RAIL_D, GUIDE_RAIL_H),
        (GUIDE_RAIL_X, GUIDE_RAIL_Y, GUIDE_RAIL_Z),
        material="rail_steel",
        name="right_guide_rail",
    )
    crosshead.inertial = Inertial.from_geometry(
        Box((CROSSHEAD_W, 0.10, 0.26)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.045, -0.06)),
    )

    slide = model.part("z_slide")
    _add_box_visual(
        slide,
        (SLIDE_BLOCK_W, SLIDE_BLOCK_D, SLIDE_BLOCK_H),
        (0.0, 0.0, 0.0),
        material="tool_dark",
        name="carriage_block",
    )
    slide.visual(
        mesh_from_cadquery(_slide_column_shape(), "service_panel_z_slide"),
        material="slide_silver",
        name="slide_column",
    )
    _add_box_visual(
        slide,
        (TOOL_PLATE_W, TOOL_PLATE_D, TOOL_PLATE_H),
        (0.0, 0.010, -SLIDE_BLOCK_H / 2.0 - SLIDE_COLUMN_H - TOOL_PLATE_H / 2.0),
        material="accent_blue",
        name="tool_plate",
    )
    slide.inertial = Inertial.from_geometry(
        Box((TOOL_PLATE_W, 0.06, 0.48)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.020, -0.24)),
    )

    model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(0.0, FRAME_TO_CROSSHEAD_Y, FRAME_TO_CROSSHEAD_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=850.0,
            velocity=0.8,
        ),
    )
    model.articulation(
        "crosshead_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=slide,
        origin=Origin(xyz=(0.0, GUIDE_RAIL_Y, GUIDE_RAIL_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=500.0,
            velocity=0.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("back_frame")
    crosshead = object_model.get_part("crosshead")
    slide = object_model.get_part("z_slide")
    x_axis = object_model.get_articulation("frame_to_crosshead")
    z_axis = object_model.get_articulation("crosshead_to_z_slide")

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
        "crosshead_axis_is_horizontal_x",
        all(isclose(a, b, abs_tol=1e-9) for a, b in zip(x_axis.axis, (1.0, 0.0, 0.0))),
        details=f"expected (1, 0, 0), got {x_axis.axis}",
    )
    ctx.check(
        "slide_axis_is_vertical_z",
        all(isclose(a, b, abs_tol=1e-9) for a, b in zip(z_axis.axis, (0.0, 0.0, -1.0))),
        details=f"expected (0, 0, -1), got {z_axis.axis}",
    )
    ctx.check(
        "prismatic_axes_are_orthogonal",
        abs(sum(a * b for a, b in zip(x_axis.axis, z_axis.axis))) < 1e-9,
        details=f"{x_axis.axis} dot {z_axis.axis}",
    )

    ctx.expect_contact(
        crosshead,
        frame,
        elem_a="upper_left_shoe",
        elem_b="upper_rail",
        name="crosshead_upper_shoe_contacts_upper_rail",
    )
    ctx.expect_contact(
        crosshead,
        frame,
        elem_a="lower_left_shoe",
        elem_b="lower_rail",
        name="crosshead_lower_shoe_contacts_lower_rail",
    )
    ctx.expect_contact(
        slide,
        crosshead,
        elem_a="carriage_block",
        elem_b="left_guide_rail",
        name="slide_carriage_contacts_left_guide",
    )

    with ctx.pose({x_axis: X_TRAVEL * 0.85, z_axis: Z_TRAVEL * 0.80}):
        ctx.expect_contact(
            crosshead,
            frame,
            elem_a="upper_right_shoe",
            elem_b="upper_rail",
            name="crosshead_stays_supported_near_end_of_travel",
        )
        ctx.expect_contact(
            slide,
            crosshead,
            elem_a="carriage_block",
            elem_b="right_guide_rail",
            name="z_slide_stays_on_guides_near_extension",
        )
        ctx.expect_gap(
            crosshead,
            slide,
            axis="z",
            positive_elem="body_shell",
            negative_elem="tool_plate",
            min_gap=0.12,
            name="tool_plate_hangs_below_crosshead_body",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
