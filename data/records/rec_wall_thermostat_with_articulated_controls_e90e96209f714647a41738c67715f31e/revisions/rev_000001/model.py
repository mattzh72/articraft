from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.118
BODY_H = 0.118
BODY_D = 0.028
BODY_CORNER_R = 0.008

DIAL_CENTER_Z = 0.022
DIAL_OUTER_R = 0.034
DIAL_INNER_R = 0.021
DIAL_DEPTH = 0.0065

DISPLAY_W = 0.033
DISPLAY_H = 0.013
DISPLAY_T = 0.0016
DISPLAY_CENTER_Y = BODY_D - 0.0037

PANEL_CENTER_Z = -0.033
PANEL_W = 0.080
PANEL_H = 0.031
PANEL_T = 0.004
PANEL_HINGE_Z = PANEL_CENTER_Z - PANEL_H / 2.0
PANEL_OPEN_LIMIT = 1.30

CAVITY_W = 0.084
CAVITY_H = 0.033
CAVITY_D = 0.012
CAVITY_REAR_Y = BODY_D - CAVITY_D


def make_housing_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, False, True))
        .edges("|Y")
        .fillet(BODY_CORNER_R)
    )

    display_recess = (
        cq.Workplane("XY")
        .box(0.044, 0.0045, 0.018, centered=(True, True, True))
        .translate((0.0, BODY_D - 0.00225, DIAL_CENTER_Z))
    )
    controls_cavity = (
        cq.Workplane("XY")
        .box(CAVITY_W, CAVITY_D, CAVITY_H, centered=(True, True, True))
        .translate((0.0, BODY_D - CAVITY_D / 2.0, PANEL_CENTER_Z))
    )

    return shell.cut(display_recess).cut(controls_cavity)


def make_dial_ring() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(DIAL_OUTER_R)
        .circle(DIAL_INNER_R)
        .extrude(DIAL_DEPTH)
        .faces(">Y")
        .edges()
        .chamfer(0.0008)
    )


def make_panel_door() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_T, PANEL_H, centered=(True, False, False))
        .edges("|Y")
        .fillet(0.0014)
    )
    hinge_rail = (
        cq.Workplane("XY")
        .box(0.050, 0.0012, 0.0015, centered=(True, False, True))
        .translate((0.0, 0.0, -0.00075))
    )
    finger_lip = (
        cq.Workplane("XY")
        .box(0.036, 0.0018, 0.004, centered=(True, False, False))
        .translate((0.0, PANEL_T, PANEL_H - 0.005))
    )
    return panel.union(hinge_rail).union(finger_lip)


def make_controls_tray() -> cq.Workplane:
    tray = cq.Workplane("XY").box(0.070, 0.0025, 0.020, centered=(True, False, True))
    for x_pos in (-0.021, 0.0, 0.021):
        tray = tray.union(
            cq.Workplane("XY")
            .box(0.014, 0.0032, 0.008, centered=(True, False, True))
            .translate((x_pos, 0.0025, 0.0))
        )
    for x_pos in (-0.023, 0.023):
        tray = tray.union(
            cq.Workplane("XY")
            .box(0.006, 0.001, 0.006, centered=(True, False, True))
            .translate((x_pos, -0.001, 0.0))
        )
    return tray


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_wall_thermostat")

    housing_white = model.material("housing_white", rgba=(0.95, 0.95, 0.93, 1.0))
    dial_satin = model.material("dial_satin", rgba=(0.74, 0.75, 0.77, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.11, 0.12, 0.14, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.87, 0.88, 0.89, 1.0))
    button_dark = model.material("button_dark", rgba=(0.22, 0.23, 0.25, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(make_housing_shell(), "housing_shell"),
        material=housing_white,
        name="shell",
    )

    housing.visual(
        Box((DISPLAY_W, DISPLAY_T, DISPLAY_H)),
        origin=Origin(xyz=(0.0, DISPLAY_CENTER_Y, DIAL_CENTER_Z)),
        material=glass_dark,
        name="display_lens",
    )

    housing.visual(
        mesh_from_cadquery(make_controls_tray(), "controls_tray"),
        origin=Origin(xyz=(0.0, CAVITY_REAR_Y + 0.001, PANEL_CENTER_Z)),
        material=button_dark,
        name="controls",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(make_dial_ring(), "dial_ring"),
        origin=Origin(xyz=(0.0, DIAL_DEPTH, 0.0)),
        material=dial_satin,
        name="dial_ring",
    )

    panel = model.part("panel")
    panel.visual(
        mesh_from_cadquery(make_panel_door(), "panel_door"),
        material=panel_gray,
        name="panel_face",
    )

    model.articulation(
        "housing_to_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.0, BODY_D, DIAL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=10.0),
    )
    model.articulation(
        "housing_to_panel",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=panel,
        origin=Origin(xyz=(0.0, BODY_D, PANEL_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=PANEL_OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    dial = object_model.get_part("dial")
    panel = object_model.get_part("panel")

    dial_joint = object_model.get_articulation("housing_to_dial")
    panel_joint = object_model.get_articulation("housing_to_panel")

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial uses continuous center-axis rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None
        and tuple(round(v, 3) for v in dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}, limits={dial_limits}",
    )

    ctx.check(
        "panel hinge opens downward from the lower edge",
        panel_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in panel_joint.axis) == (-1.0, 0.0, 0.0)
        and panel_joint.motion_limits is not None
        and abs(panel_joint.motion_limits.lower - 0.0) < 1e-6
        and abs(panel_joint.motion_limits.upper - PANEL_OPEN_LIMIT) < 1e-6,
        details=f"axis={panel_joint.axis}, limits={panel_joint.motion_limits}",
    )

    ctx.expect_overlap(
        dial,
        housing,
        axes="xz",
        elem_a="dial_ring",
        elem_b="display_lens",
        min_overlap=0.012,
        name="dial remains centered around the display",
    )
    ctx.expect_gap(
        panel,
        housing,
        axis="y",
        max_gap=0.0006,
        max_penetration=0.0,
        elem_a="panel_face",
        elem_b="shell",
        name="panel closes nearly flush with the housing front",
    )
    ctx.expect_gap(
        panel,
        housing,
        axis="y",
        min_gap=0.003,
        max_gap=0.008,
        elem_a="panel_face",
        elem_b="controls",
        name="closed panel leaves a shallow concealment gap over the controls",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_face")
    with ctx.pose({panel_joint: PANEL_OPEN_LIMIT}):
        open_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_face")

    swings_outward = (
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.020
    )
    drops_down = (
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] < closed_panel_aabb[1][2] - 0.010
    )
    ctx.check(
        "open panel swings outward and down to reveal the controls",
        swings_outward and drops_down,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
