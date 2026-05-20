from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


OPENING_WIDTH = 0.62
OPENING_HEIGHT = 0.50
OUTER_WIDTH = 0.84
OUTER_HEIGHT = 0.70
FRAME_DEPTH = 0.040
PANEL_WIDTH = 0.57
PANEL_HEIGHT = 0.46
PANEL_THICKNESS = 0.024
HINGE_Y = 0.048
HINGE_Z = -OPENING_HEIGHT / 2.0
STAY_PIVOT_X = 0.36
STAY_PIVOT_Y = 0.055
STAY_PIVOT_Z = 0.12
STAY_LENGTH = 0.30


def _stay_bar_mesh(name: str):
    """Flat rigid stay bar with real clearance holes, extruded along local X."""
    outer_radius = 0.022
    hole_radius = 0.006
    thickness = 0.010
    web_width = 0.016
    web_length = STAY_LENGTH - 2.0 * outer_radius

    upper_eye = (
        cq.Workplane("YZ")
        .center(0.0, 0.0)
        .circle(outer_radius)
        .extrude(thickness, both=True)
    )
    lower_eye = (
        cq.Workplane("YZ")
        .center(0.0, -STAY_LENGTH)
        .circle(outer_radius)
        .extrude(thickness, both=True)
    )
    web = (
        cq.Workplane("YZ")
        .center(0.0, -STAY_LENGTH / 2.0)
        .rect(web_width, web_length + 0.012)
        .extrude(thickness, both=True)
    )
    blank = upper_eye.union(lower_eye).union(web)
    upper_hole = cq.Workplane("YZ").circle(hole_radius).extrude(thickness * 3.0, both=True)
    lower_hole = (
        cq.Workplane("YZ")
        .center(0.0, -STAY_LENGTH)
        .circle(hole_radius)
        .extrude(thickness * 3.0, both=True)
    )
    return mesh_from_cadquery(blank.cut(upper_hole).cut(lower_hole), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_cabinet_hatch")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.57, 0.61, 0.62, 1.0))
    darker_recess = model.material("darker_recess", rgba=(0.08, 0.09, 0.09, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.64, 0.64, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    warning_label = model.material("warning_label", rgba=(0.92, 0.74, 0.18, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_geometry(
            BezelGeometry(
                (OPENING_WIDTH, OPENING_HEIGHT),
                (OUTER_WIDTH, OUTER_HEIGHT),
                FRAME_DEPTH,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.018,
                outer_corner_radius=0.035,
                face=BezelFace(style="radiused_step", front_lip=0.006, fillet=0.003),
            ),
            "cabinet_frame",
        ),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=cabinet_paint,
        name="frame_shell",
    )
    # A shallow black sleeve just inside the opening makes the hatch read as a
    # real cabinet aperture rather than a painted solid panel.
    cabinet.visual(
        Box((OPENING_WIDTH - 0.060, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, OPENING_HEIGHT / 2.0 - 0.015)),
        material=darker_recess,
        name="top_inner_lip",
    )
    cabinet.visual(
        Box((OPENING_WIDTH - 0.060, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, -OPENING_HEIGHT / 2.0 + 0.015)),
        material=darker_recess,
        name="bottom_inner_lip",
    )
    cabinet.visual(
        Box((0.030, 0.018, OPENING_HEIGHT - 0.060)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 + 0.015, -0.020, 0.0)),
        material=darker_recess,
        name="side_inner_lip_0",
    )
    cabinet.visual(
        Box((0.030, 0.018, OPENING_HEIGHT - 0.060)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 - 0.015, -0.020, 0.0)),
        material=darker_recess,
        name="side_inner_lip_1",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=0.160),
        origin=Origin(xyz=(-0.22, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="fixed_knuckle_0",
    )
    cabinet.visual(
        Box((0.160, 0.030, 0.012)),
        origin=Origin(xyz=(-0.22, 0.034, HINGE_Z)),
        material=hinge_steel,
        name="fixed_hinge_web_0",
    )
    cabinet.visual(
        Box((0.160, 0.008, 0.052)),
        origin=Origin(xyz=(-0.22, 0.024, HINGE_Z - 0.028)),
        material=hinge_steel,
        name="fixed_hinge_leaf_0",
    )
    cabinet.visual(
        Cylinder(radius=0.014, length=0.160),
        origin=Origin(xyz=(0.22, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="fixed_knuckle_1",
    )
    cabinet.visual(
        Box((0.160, 0.030, 0.012)),
        origin=Origin(xyz=(0.22, 0.034, HINGE_Z)),
        material=hinge_steel,
        name="fixed_hinge_web_1",
    )
    cabinet.visual(
        Box((0.160, 0.008, 0.052)),
        origin=Origin(xyz=(0.22, 0.024, HINGE_Z - 0.028)),
        material=hinge_steel,
        name="fixed_hinge_leaf_1",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.650),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="hinge_pin",
    )
    cabinet.visual(
        Box((0.082, 0.012, 0.070)),
        origin=Origin(xyz=(-STAY_PIVOT_X, 0.022, STAY_PIVOT_Z)),
        material=hinge_steel,
        name="stay_bracket_0",
    )
    cabinet.visual(
        Box((0.016, 0.038, 0.045)),
        origin=Origin(xyz=(-STAY_PIVOT_X - 0.023, 0.040, STAY_PIVOT_Z)),
        material=hinge_steel,
        name="stay_fork_0_0",
    )
    cabinet.visual(
        Box((0.016, 0.038, 0.045)),
        origin=Origin(xyz=(-STAY_PIVOT_X + 0.023, 0.040, STAY_PIVOT_Z)),
        material=hinge_steel,
        name="stay_fork_0_1",
    )
    cabinet.visual(
        Cylinder(radius=0.0068, length=0.060),
        origin=Origin(xyz=(-STAY_PIVOT_X, STAY_PIVOT_Y, STAY_PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="stay_pivot_pin_0",
    )
    cabinet.visual(
        Box((0.082, 0.012, 0.070)),
        origin=Origin(xyz=(STAY_PIVOT_X, 0.022, STAY_PIVOT_Z)),
        material=hinge_steel,
        name="stay_bracket_1",
    )
    cabinet.visual(
        Box((0.016, 0.038, 0.045)),
        origin=Origin(xyz=(STAY_PIVOT_X - 0.023, 0.040, STAY_PIVOT_Z)),
        material=hinge_steel,
        name="stay_fork_1_0",
    )
    cabinet.visual(
        Box((0.016, 0.038, 0.045)),
        origin=Origin(xyz=(STAY_PIVOT_X + 0.023, 0.040, STAY_PIVOT_Z)),
        material=hinge_steel,
        name="stay_fork_1_1",
    )
    cabinet.visual(
        Cylinder(radius=0.0068, length=0.060),
        origin=Origin(xyz=(STAY_PIVOT_X, STAY_PIVOT_Y, STAY_PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="stay_pivot_pin_1",
    )

    panel = model.part("panel")
    panel.visual(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS / 2.0, PANEL_HEIGHT / 2.0 + 0.015)),
        material=galvanized,
        name="panel_slab",
    )
    panel.visual(
        Box((PANEL_WIDTH - 0.045, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS + 0.004, PANEL_HEIGHT - 0.020)),
        material=hinge_steel,
        name="top_stiffener",
    )
    panel.visual(
        Box((PANEL_WIDTH - 0.045, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS + 0.004, 0.075)),
        material=hinge_steel,
        name="bottom_stiffener",
    )
    panel.visual(
        Box((0.030, 0.008, PANEL_HEIGHT - 0.080)),
        origin=Origin(xyz=(-PANEL_WIDTH / 2.0 + 0.032, PANEL_THICKNESS + 0.004, PANEL_HEIGHT / 2.0 + 0.015)),
        material=hinge_steel,
        name="side_stiffener_0",
    )
    panel.visual(
        Box((0.030, 0.008, PANEL_HEIGHT - 0.080)),
        origin=Origin(xyz=(PANEL_WIDTH / 2.0 - 0.032, PANEL_THICKNESS + 0.004, PANEL_HEIGHT / 2.0 + 0.015)),
        material=hinge_steel,
        name="side_stiffener_1",
    )
    panel.visual(
        Box((0.115, 0.010, 0.045)),
        origin=Origin(xyz=(0.0, PANEL_THICKNESS + 0.003, PANEL_HEIGHT - 0.095)),
        material=warning_label,
        name="pull_label",
    )
    panel.visual(
        Cylinder(radius=0.014, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_steel,
        name="moving_knuckle",
    )
    panel.visual(
        Box((0.200, 0.026, 0.044)),
        origin=Origin(xyz=(0.0, 0.010, 0.032)),
        material=hinge_steel,
        name="moving_hinge_leaf",
    )
    panel.visual(
        Box((0.060, 0.018, 0.038)),
        origin=Origin(xyz=(-0.315, PANEL_THICKNESS / 2.0, 0.070)),
        material=hinge_steel,
        name="stay_lug_0",
    )
    panel.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(-STAY_PIVOT_X, 0.007, 0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="stay_pin_0",
    )
    panel.visual(
        Box((0.060, 0.018, 0.038)),
        origin=Origin(xyz=(0.315, PANEL_THICKNESS / 2.0, 0.070)),
        material=hinge_steel,
        name="stay_lug_1",
    )
    panel.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(STAY_PIVOT_X, 0.007, 0.070), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="stay_pin_1",
    )

    stay_bar_0 = model.part("stay_bar_0")
    stay_bar_0.visual(
        _stay_bar_mesh("stay_bar_0_plate"),
        material=hinge_steel,
        name="bar_plate",
    )

    stay_bar_1 = model.part("stay_bar_1")
    stay_bar_1.visual(
        _stay_bar_mesh("stay_bar_1_plate"),
        material=hinge_steel,
        name="bar_plate",
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=panel,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=1.35),
    )
    model.articulation(
        "stay_pivot_0",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=stay_bar_0,
        origin=Origin(xyz=(-STAY_PIVOT_X, STAY_PIVOT_Y, STAY_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "stay_pivot_1",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=stay_bar_1,
        origin=Origin(xyz=(STAY_PIVOT_X, STAY_PIVOT_Y, STAY_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel = object_model.get_part("panel")
    cabinet = object_model.get_part("cabinet")
    stay_bar_0 = object_model.get_part("stay_bar_0")
    stay_bar_1 = object_model.get_part("stay_bar_1")
    panel_hinge = object_model.get_articulation("panel_hinge")
    stay_pivot_0 = object_model.get_articulation("stay_pivot_0")
    stay_pivot_1 = object_model.get_articulation("stay_pivot_1")

    ctx.allow_overlap(
        cabinet,
        panel,
        elem_a="hinge_pin",
        elem_b="moving_knuckle",
        reason="The cabinet hinge pin intentionally passes through the panel knuckle.",
    )
    ctx.allow_overlap(
        cabinet,
        stay_bar_0,
        elem_a="stay_pivot_pin_0",
        elem_b="bar_plate",
        reason="The stay bar eye is captured on a side pivot pin with a tiny modeled interference fit.",
    )
    ctx.allow_overlap(
        cabinet,
        stay_bar_1,
        elem_a="stay_pivot_pin_1",
        elem_b="bar_plate",
        reason="The stay bar eye is captured on a side pivot pin with a tiny modeled interference fit.",
    )
    ctx.allow_overlap(
        panel,
        stay_bar_0,
        elem_a="stay_pin_0",
        elem_b="bar_plate",
        reason="The lower stay eye is intentionally retained by the access-panel pin.",
    )
    ctx.allow_overlap(
        panel,
        stay_bar_1,
        elem_a="stay_pin_1",
        elem_b="bar_plate",
        reason="The lower stay eye is intentionally retained by the access-panel pin.",
    )

    ctx.expect_within(
        panel,
        cabinet,
        axes="xz",
        inner_elem="panel_slab",
        outer_elem="frame_shell",
        margin=0.025,
        name="closed panel sits within the fixed frame outline",
    )
    ctx.expect_overlap(
        panel,
        cabinet,
        axes="x",
        elem_a="moving_knuckle",
        elem_b="hinge_pin",
        min_overlap=0.17,
        name="moving hinge knuckle is captured on the lower hinge pin",
    )
    ctx.expect_contact(
        stay_bar_0,
        cabinet,
        elem_a="bar_plate",
        elem_b="stay_pivot_pin_0",
        contact_tol=0.006,
        name="stay bar 0 is carried by its side pivot pin",
    )
    ctx.expect_contact(
        stay_bar_1,
        cabinet,
        elem_a="bar_plate",
        elem_b="stay_pivot_pin_1",
        contact_tol=0.006,
        name="stay bar 1 is carried by its side pivot pin",
    )
    ctx.expect_overlap(
        stay_bar_0,
        panel,
        axes="yz",
        elem_a="bar_plate",
        elem_b="stay_pin_0",
        min_overlap=0.008,
        name="stay bar 0 lower eye stays on the panel pin",
    )
    ctx.expect_overlap(
        stay_bar_1,
        panel,
        axes="yz",
        elem_a="bar_plate",
        elem_b="stay_pin_1",
        min_overlap=0.008,
        name="stay bar 1 lower eye stays on the panel pin",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_slab")
    closed_bar_aabb = ctx.part_element_world_aabb(stay_bar_0, elem="bar_plate")
    with ctx.pose({panel_hinge: 1.15, stay_pivot_0: 0.80, stay_pivot_1: 0.80}):
        open_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_slab")
        open_bar_aabb = ctx.part_element_world_aabb(stay_bar_0, elem="bar_plate")

    ctx.check(
        "panel opens outward and downward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.20
        and open_panel_aabb[1][2] < closed_panel_aabb[1][2] - 0.05,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )
    ctx.check(
        "stay bars swing out from side pivots",
        closed_bar_aabb is not None
        and open_bar_aabb is not None
        and open_bar_aabb[1][1] > closed_bar_aabb[1][1] + 0.15,
        details=f"closed={closed_bar_aabb}, open={open_bar_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
