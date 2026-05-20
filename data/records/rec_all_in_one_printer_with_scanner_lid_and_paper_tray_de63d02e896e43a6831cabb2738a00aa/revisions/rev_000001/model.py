from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.39
BODY_D = 0.34
BODY_H = 0.215

SCANNER_W = 0.312
SCANNER_D = 0.236
SCANNER_RECESS = 0.014

LID_W = 0.346
LID_D = 0.286
LID_T = 0.026
LID_GAP = 0.0

TRAY_W = 0.302
TRAY_D = 0.218
TRAY_H = 0.024
TRAY_FASCIA_T = 0.004
TRAY_Z0 = 0.018
TRAY_TRAVEL = 0.120

FEEDER_W = 0.278
FEEDER_D = 0.110
FEEDER_T = 0.006
FEEDER_GAP = 0.0

KNOB_D = 0.028
KNOB_H = 0.016
BUTTON_R = 0.009
BUTTON_L = 0.006
BUTTON_TRAVEL = 0.003

KNOB_X = 0.098
KNOB_Z = 0.136
BUTTON_X = 0.052
BUTTON_TOP_Z = 0.145
BUTTON_BOTTOM_Z = 0.119


def make_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    body = body.edges("|Z").fillet(0.012)
    body = body.faces(">Z").edges().fillet(0.008)

    scanner_cutter = (
        cq.Workplane("XY")
        .box(SCANNER_W, SCANNER_D, SCANNER_RECESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, BODY_H - SCANNER_RECESS))
    )

    paper_cavity = cq.Workplane("XY").box(
        0.324, 0.256, 0.110, centered=(True, True, False)
    ).translate((0.0, 0.0, 0.016))

    tray_opening = cq.Workplane("XY").box(
        0.314, 0.055, 0.032, centered=(True, True, False)
    ).translate((0.0, -BODY_D / 2.0 + 0.0275, 0.016))

    rear_slot = cq.Workplane("XY").box(
        FEEDER_W + 0.008, 0.044, 0.042, centered=(True, True, False)
    ).translate((0.0, BODY_D / 2.0 - 0.022, 0.122))

    knob_hole = cq.Workplane("XY").box(
        0.020, 0.012, 0.020, centered=(True, True, False)
    ).translate((KNOB_X, -BODY_D / 2.0 + 0.006, KNOB_Z - 0.010))

    button_pocket_top = cq.Workplane("XY").box(
        0.018, 0.008, 0.018, centered=(True, True, False)
    ).translate((BUTTON_X, -BODY_D / 2.0 + 0.004, BUTTON_TOP_Z - 0.009))

    button_pocket_bottom = cq.Workplane("XY").box(
        0.018, 0.008, 0.018, centered=(True, True, False)
    ).translate((BUTTON_X, -BODY_D / 2.0 + 0.004, BUTTON_BOTTOM_Z - 0.009))

    return (
        body.cut(scanner_cutter)
        .cut(paper_cavity)
        .cut(tray_opening)
        .cut(rear_slot)
        .cut(knob_hole)
        .cut(button_pocket_top)
        .cut(button_pocket_bottom)
    )


def make_lid_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T)
        .edges("|Z")
        .fillet(0.010)
        .faces(">Z")
        .edges()
        .fillet(0.012)
        .translate((0.0, -LID_D / 2.0, LID_T / 2.0))
    )


def make_tray_shape() -> cq.Workplane:
    front_panel = cq.Workplane("XY").box(0.326, TRAY_FASCIA_T, 0.036).translate(
        (0.0, 0.0, 0.018)
    )
    shell = cq.Workplane("XY").box(TRAY_W, TRAY_D, TRAY_H).translate(
        (0.0, TRAY_D / 2.0 + TRAY_FASCIA_T / 2.0, TRAY_H / 2.0)
    )
    cavity = cq.Workplane("XY").box(
        TRAY_W - 0.012, TRAY_D - 0.012, TRAY_H, centered=(True, True, False)
    ).translate((0.0, 0.008 + (TRAY_D - 0.012) / 2.0, 0.004))
    finger_scoop = cq.Workplane("XY").box(
        0.100, TRAY_FASCIA_T + 0.002, 0.010, centered=(True, True, False)
    ).translate((0.0, 0.0, TRAY_H - 0.010))

    return front_panel.union(shell).cut(cavity).cut(finger_scoop)


def make_feeder_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FEEDER_W, FEEDER_D, FEEDER_T)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, FEEDER_D / 2.0, FEEDER_T / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="minimalist_all_in_one_printer")

    body_finish = model.material("body_finish", rgba=(0.94, 0.95, 0.96, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.90, 0.91, 0.93, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.26, 0.28, 0.30, 1.0))
    control_finish = model.material("control_finish", rgba=(0.13, 0.14, 0.15, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shape(), "printer_body"),
        material=body_finish,
        name="body_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(make_lid_shape(), "scanner_lid"),
        material=lid_finish,
        name="lid_shell",
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(make_tray_shape(), "paper_tray"),
        material=tray_finish,
        name="tray_shell",
    )

    feeder = model.part("feeder")
    feeder.visual(
        mesh_from_cadquery(make_feeder_shape(), "rear_feeder"),
        material=lid_finish,
        name="feeder_panel",
    )

    knob = model.part("selector_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_D,
                KNOB_H,
                body_style="skirted",
                top_diameter=0.022,
                center=False,
            ),
            "selector_knob",
        ),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="knob_cap",
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Cylinder(radius=BUTTON_R, length=BUTTON_L),
        origin=Origin(xyz=(0.0, -BUTTON_L / 2.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="button_cap",
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Cylinder(radius=BUTTON_R, length=BUTTON_L),
        origin=Origin(xyz=(0.0, -BUTTON_L / 2.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="button_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.146, BODY_H + LID_GAP)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=8.0, velocity=1.8),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 - TRAY_FASCIA_T / 2.0, TRAY_Z0)
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=25.0, velocity=0.18),
    )
    model.articulation(
        "body_to_feeder",
        ArticulationType.REVOLUTE,
        parent=body,
        child=feeder,
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.010, BODY_H + FEEDER_GAP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=5.0, velocity=1.8),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(KNOB_X, -BODY_D / 2.0, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0),
    )
    model.articulation(
        "body_to_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_0,
        origin=Origin(xyz=(BUTTON_X, -BODY_D / 2.0, BUTTON_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=2.0, velocity=0.08),
    )
    model.articulation(
        "body_to_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_1,
        origin=Origin(xyz=(BUTTON_X, -BODY_D / 2.0, BUTTON_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=2.0, velocity=0.08),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    feeder = object_model.get_part("feeder")
    knob = object_model.get_part("selector_knob")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    lid_joint = object_model.get_articulation("body_to_lid")
    tray_joint = object_model.get_articulation("body_to_tray")
    feeder_joint = object_model.get_articulation("body_to_feeder")
    knob_joint = object_model.get_articulation("body_to_selector_knob")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected a body AABB.")
    ctx.allow_overlap(
        body,
        tray,
        elem_a="body_shell",
        elem_b="tray_shell",
        reason="The paper tray is intentionally represented as a nested member retained inside the printer's front paper-path housing.",
    )
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "desktop_printer_scale",
            0.36 <= size[0] <= 0.42 and 0.31 <= size[1] <= 0.37 and 0.20 <= size[2] <= 0.24,
            details=f"size={size!r}",
        )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.004,
            max_penetration=0.0,
            name="lid rests just above the top deck when closed",
        )
    lid_closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.25}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and float(lid_open_aabb[1][2]) > float(lid_closed_aabb[1][2]) + 0.08,
        details=f"closed={lid_closed_aabb!r}, open={lid_open_aabb!r}",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: TRAY_TRAVEL}):
        tray_extended = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.090,
            name="tray remains inserted when fully extended",
        )
    ctx.check(
        "tray slides forward",
        tray_rest is not None
        and tray_extended is not None
        and float(tray_extended[1]) < float(tray_rest[1]) - 0.10,
        details=f"rest={tray_rest!r}, extended={tray_extended!r}",
    )

    with ctx.pose({feeder_joint: 0.0}):
        ctx.expect_gap(
            feeder,
            body,
            axis="z",
            max_gap=0.004,
            max_penetration=0.0,
            name="rear feeder folds nearly flush to the body",
        )
    feeder_closed_aabb = ctx.part_world_aabb(feeder)
    with ctx.pose({feeder_joint: 1.30}):
        feeder_open_aabb = ctx.part_world_aabb(feeder)
    ctx.check(
        "rear feeder pops upward",
        feeder_closed_aabb is not None
        and feeder_open_aabb is not None
        and float(feeder_open_aabb[1][2]) > float(feeder_closed_aabb[1][2]) + 0.06,
        details=f"closed={feeder_closed_aabb!r}, open={feeder_open_aabb!r}",
    )

    button_0_aabb = ctx.part_world_aabb(button_0)
    ctx.check(
        "top button protrudes from the front panel",
        body_aabb is not None
        and button_0_aabb is not None
        and float(button_0_aabb[0][1]) < float(body_aabb[0][1]) - 0.002,
        details=f"body={body_aabb!r}, button={button_0_aabb!r}",
    )
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: BUTTON_TRAVEL}):
        button_1_pressed = ctx.part_world_position(button_1)
    ctx.check(
        "lower button presses inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and float(button_1_pressed[1]) > float(button_1_rest[1]) + 0.0025,
        details=f"rest={button_1_rest!r}, pressed={button_1_pressed!r}",
    )

    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: 1.75}):
        knob_rotated = ctx.part_world_position(knob)
    ctx.check(
        "selector knob spins in place",
        knob_rest is not None and knob_rotated is not None and tuple(knob_rest) == tuple(knob_rotated),
        details=f"rest={knob_rest!r}, rotated={knob_rotated!r}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_0_joint: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "top button presses inward",
        button_0_rest is not None
        and button_0_pressed is not None
        and float(button_0_pressed[1]) > float(button_0_rest[1]) + 0.0025,
        details=f"rest={button_0_rest!r}, pressed={button_0_pressed!r}",
    )

    ctx.expect_origin_distance(
        button_0,
        button_1,
        axes="z",
        min_dist=0.020,
        max_dist=0.032,
        name="front buttons sit as a nearby pair",
    )

    return ctx.report()


object_model = build_object_model()
