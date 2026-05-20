from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BODY_L = 0.24
BODY_W = 0.14
BASE_T = 0.012
RAIL_L = 0.180
RAIL_W = 0.018
RAIL_H = 0.018
RAIL_Y = 0.030

RUNNER_L = 0.064
RUNNER_W = 0.016
RUNNER_H = 0.008
CARRIAGE_L = 0.078
CARRIAGE_W = 0.084
CARRIAGE_H = 0.018

HINGE_X = 0.075
HINGE_Z = 0.030
HINGE_R = 0.005
LUG_L = 0.012
LUG_Y_OFFSET = 0.026

TAB_L = 0.034
TAB_W = 0.032
TAB_T = 0.004
TAB_BARREL_L = 0.040
TAB_LIP_L = 0.008
TAB_LIP_H = 0.006

SLIDE_ORIGIN_X = -0.072
SLIDE_Z = BASE_T + RAIL_H
SLIDE_TRAVEL = 0.085


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        BODY_L,
        BODY_W,
        BASE_T,
        centered=(True, True, False),
    )
    for y_pos in (-RAIL_Y, RAIL_Y):
        body = body.union(
            cq.Workplane("XY")
            .transformed(offset=(0.0, y_pos, BASE_T))
            .box(RAIL_L, RAIL_W, RAIL_H, centered=(True, True, False))
        )

    body = body.union(
        cq.Workplane("XY")
        .transformed(offset=(-0.095, 0.0, BASE_T))
        .box(0.030, 0.058, 0.022, centered=(True, True, False))
    )
    body = body.union(
        cq.Workplane("XY")
        .transformed(offset=(0.098, 0.0, BASE_T))
        .box(0.026, 0.044, 0.018, centered=(True, True, False))
    )

    return body


def _carriage_shape() -> cq.Workplane:
    carriage = None
    for y_pos in (-RAIL_Y, RAIL_Y):
        shoe = (
            cq.Workplane("XY")
            .transformed(offset=(0.022, y_pos, 0.0))
            .box(RUNNER_L, RUNNER_W, RUNNER_H, centered=(True, True, False))
        )
        carriage = shoe if carriage is None else carriage.union(shoe)

    carriage = carriage.union(
        cq.Workplane("XY")
        .transformed(offset=(0.028, 0.0, RUNNER_H))
        .box(CARRIAGE_L, CARRIAGE_W, CARRIAGE_H, centered=(True, True, False))
    )

    for y_pos in (-LUG_Y_OFFSET, LUG_Y_OFFSET):
        carriage = carriage.union(
            cq.Workplane("XY")
            .transformed(offset=(HINGE_X - 0.005, y_pos, RUNNER_H + CARRIAGE_H - 0.002))
            .box(0.016, 0.010, 0.008, centered=(True, True, False))
        )
        carriage = carriage.union(
            cq.Workplane("XZ")
            .center(HINGE_X, HINGE_Z)
            .circle(HINGE_R)
            .extrude(LUG_L / 2.0, both=True)
            .translate((0.0, y_pos, 0.0))
        )

    return carriage


def _tab_shape() -> cq.Workplane:
    tab = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(HINGE_R)
        .extrude(TAB_BARREL_L / 2.0, both=True)
    )
    tab = tab.union(
        cq.Workplane("XY").box(0.018, 0.012, 0.008).translate((0.010, 0.0, 0.0)),
    )
    tab = tab.union(
        cq.Workplane("XY").box(TAB_L, TAB_W, TAB_T).translate((0.026, 0.0, 0.0)),
    )
    tab = tab.union(
        cq.Workplane("XY")
        .box(TAB_LIP_L, TAB_W * 0.70, TAB_LIP_H)
        .translate((0.026 + (TAB_L / 2.0) - (TAB_LIP_L / 2.0), 0.0, -0.005)),
    )
    return tab


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_slide_hinge_module")

    model.material("body_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    model.material("carriage_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("tab_orange", rgba=(0.86, 0.47, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "service_module_body"),
        material="body_gray",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_L, BODY_W, BASE_T + 0.022)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + 0.022) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "service_module_carriage"),
        material="carriage_black",
        name="carriage_shell",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_L, CARRIAGE_W, RUNNER_H + CARRIAGE_H + 0.010)),
        mass=0.45,
        origin=Origin(
            xyz=(0.030, 0.0, (RUNNER_H + CARRIAGE_H + 0.010) / 2.0),
        ),
    )

    tab = model.part("tab")
    tab.visual(
        mesh_from_cadquery(_tab_shape(), "service_module_tab"),
        material="tab_orange",
        name="tab_panel",
    )
    tab.inertial = Inertial.from_geometry(
        Box((TAB_L, TAB_W, TAB_T + TAB_LIP_H)),
        mass=0.08,
        origin=Origin(xyz=(TAB_L / 2.0, 0.0, -0.004)),
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, SLIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=140.0,
            velocity=0.20,
        ),
    )
    model.articulation(
        "carriage_to_tab",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tab,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.20,
            effort=8.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    tab = object_model.get_part("tab")
    slide = object_model.get_articulation("body_to_carriage")
    hinge = object_model.get_articulation("carriage_to_tab")

    ctx.allow_overlap(
        carriage,
        tab,
        elem_a="carriage_shell",
        elem_b="tab_panel",
        reason=(
            "The compact hinge is represented as interleaved knuckle geometry without "
            "a separately modeled through-pin bore, so the hinge axis uses a small "
            "intentional shared-volume proxy."
        ),
    )

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

    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0
    hinge_upper = hinge.motion_limits.upper if hinge.motion_limits is not None else 0.0

    ctx.expect_contact(
        carriage,
        body,
        contact_tol=0.0008,
        name="carriage is seated on the grounded body at rest",
    )
    ctx.expect_within(
        carriage,
        body,
        axes="y",
        margin=0.0015,
        name="carriage stays centered over the body rails at rest",
    )
    ctx.expect_contact(
        tab,
        carriage,
        contact_tol=0.0008,
        name="tab remains physically pinned to the carriage hinge at rest",
    )
    rest_carriage_pos = ctx.part_world_position(carriage)
    closed_panel_aabb = ctx.part_element_world_aabb(tab, elem="tab_panel")

    with ctx.pose({slide: slide_upper}):
        ctx.expect_contact(
            carriage,
            body,
            contact_tol=0.0008,
            name="carriage remains supported at full slide extension",
        )
        ctx.expect_within(
            carriage,
            body,
            axes="y",
            margin=0.0015,
            name="carriage remains centered over the rails at full slide extension",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage extends forward along +X",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.060,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    with ctx.pose({hinge: hinge_upper}):
        open_panel_aabb = ctx.part_element_world_aabb(tab, elem="tab_panel")

    ctx.check(
        "tab lifts upward when the hinge opens",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.020,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    with ctx.pose({slide: slide_upper, hinge: hinge_upper}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="service pose clears the body, carriage, and tab",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
