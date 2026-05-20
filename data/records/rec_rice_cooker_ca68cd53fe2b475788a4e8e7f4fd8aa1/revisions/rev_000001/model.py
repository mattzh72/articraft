from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.215
BODY_WIDTH = 0.185
BODY_HEIGHT = 0.125
BODY_WALL = 0.0055

PANEL_THICK = 0.004
PANEL_WIDTH = 0.104
PANEL_HEIGHT = 0.088
PANEL_CENTER_Z = 0.061

HINGE_X = -BODY_DEPTH / 2.0 + 0.014
HINGE_Z = BODY_HEIGHT + 0.007

BUTTON_Y_OFFSET = 0.020
BUTTON_LOCAL_Z = -0.031
DIAL_LOCAL_Z = -0.002
LATCH_LOCAL_Z = 0.031


def _body_shell_shape():
    shell = (
        cq.Workplane("XY")
        .rect(BODY_DEPTH, BODY_WIDTH)
        .extrude(BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.028)
        .faces(">Z")
        .shell(-BODY_WALL)
    )

    control_opening = (
        cq.Workplane("XY")
        .box(0.030, PANEL_WIDTH, PANEL_HEIGHT)
        .translate((BODY_DEPTH / 2.0 - 0.012, 0.0, PANEL_CENTER_Z))
    )
    return shell.cut(control_opening)


def _panel_shape():
    panel = (
        cq.Workplane("XY")
        .box(PANEL_THICK, PANEL_WIDTH, PANEL_HEIGHT)
        .edges("|X")
        .fillet(0.006)
    )

    latch_opening = (
        cq.Workplane("XY")
        .box(PANEL_THICK * 2.5, 0.042, 0.019)
        .translate((0.0, 0.0, LATCH_LOCAL_Z))
    )
    dial_opening = (
        cq.Workplane("XY")
        .box(PANEL_THICK * 2.5, 0.012, 0.012)
        .translate((0.0, 0.0, DIAL_LOCAL_Z))
    )
    button_0_opening = (
        cq.Workplane("XY")
        .box(PANEL_THICK * 2.5, 0.022, 0.014)
        .translate((0.0, -BUTTON_Y_OFFSET, BUTTON_LOCAL_Z))
    )
    button_1_opening = (
        cq.Workplane("XY")
        .box(PANEL_THICK * 2.5, 0.022, 0.014)
        .translate((0.0, BUTTON_Y_OFFSET, BUTTON_LOCAL_Z))
    )
    return panel.cut(latch_opening).cut(dial_opening).cut(button_0_opening).cut(button_1_opening)


def _lid_shell_shape():
    return (
        cq.Workplane("XY")
        .rect(0.200, 0.176)
        .extrude(0.028)
        .edges("|Z")
        .fillet(0.018)
        .faces("<Z")
        .shell(-0.0035)
        .translate((0.110, 0.0, -0.002))
    )


def _selector_dial_shape():
    skirt = cq.Workplane("YZ").circle(0.019).extrude(0.003)
    body = (
        cq.Workplane("YZ")
        .workplane(offset=0.0025)
        .circle(0.0165)
        .workplane(offset=0.0075)
        .circle(0.0135)
        .loft(combine=True)
    )
    cap = cq.Workplane("YZ").workplane(offset=0.010).circle(0.0130).extrude(0.003)
    return skirt.union(body).union(cap).edges(">X").fillet(0.002)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_rice_cooker")

    body_white = model.material("body_white", rgba=(0.93, 0.92, 0.88, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.68, 0.69, 0.71, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.80, 0.81, 0.82, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.20, 0.21, 0.22, 1.0))
    button_grey = model.material("button_grey", rgba=(0.90, 0.90, 0.91, 1.0))
    inner_metal = model.material("inner_metal", rgba=(0.82, 0.83, 0.84, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "rice_cooker_body_shell"),
        material=body_white,
        name="housing_shell",
    )
    for index, y_pos in enumerate((-0.031, 0.031)):
        body.visual(
            Box((0.020, 0.030, 0.016)),
            origin=Origin(xyz=(HINGE_X - 0.001, y_pos, BODY_HEIGHT - 0.008)),
            material=trim_grey,
            name=f"hinge_mount_{index}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(xyz=(HINGE_X, y_pos, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim_grey,
            name=f"hinge_barrel_{index}",
        )
    body.visual(
        Box((0.030, 0.060, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=trim_grey,
        name="base_trim",
    )

    panel = model.part("panel")
    panel.visual(
        mesh_from_cadquery(_panel_shape(), "rice_cooker_front_panel"),
        material=panel_grey,
        name="faceplate",
    )
    panel.visual(
        Cylinder(radius=0.019, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, DIAL_LOCAL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=panel_grey,
        name="dial_bezel",
    )
    model.articulation(
        "body_to_panel",
        ArticulationType.FIXED,
        parent=body,
        child=panel,
        origin=Origin(xyz=(BODY_DEPTH / 2.0 - PANEL_THICK / 2.0, 0.0, PANEL_CENTER_Z)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "rice_cooker_lid_shell"),
        material=body_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.194, 0.170, 0.003)),
        origin=Origin(xyz=(0.110, 0.0, -0.0005)),
        material=inner_metal,
        name="inner_plate",
    )
    lid.visual(
        Box((0.022, 0.040, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.006)),
        material=body_white,
        name="rear_bridge",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_grey,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.020, 0.058, 0.010)),
        origin=Origin(xyz=(0.200, 0.0, 0.006)),
        material=trim_grey,
        name="front_handle",
    )
    lid.visual(
        Box((0.010, 0.030, 0.010)),
        origin=Origin(xyz=(0.202, 0.0, -0.001)),
        material=trim_grey,
        name="latch_tab",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.005),
        origin=Origin(xyz=(0.154, -0.044, 0.022)),
        material=trim_grey,
        name="steam_vent",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        mesh_from_cadquery(_selector_dial_shape(), "rice_cooker_selector_dial"),
        origin=Origin(xyz=(0.0005, 0.0, 0.0)),
        material=dark_grey,
        name="knob_shell",
    )
    selector_dial.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_grey,
        name="shaft",
    )
    selector_dial.visual(
        Box((0.002, 0.004, 0.012)),
        origin=Origin(xyz=(0.0105, 0.0, 0.008)),
        material=button_grey,
        name="indicator",
    )
    model.articulation(
        "panel_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=panel,
        child=selector_dial,
        origin=Origin(xyz=(PANEL_THICK / 2.0, 0.0, DIAL_LOCAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    latch = model.part("latch")
    latch.visual(
        Box((0.010, 0.036, 0.014)),
        origin=Origin(xyz=(-0.0045, 0.0, 0.0)),
        material=button_grey,
        name="latch_cap",
    )
    latch.visual(
        Box((0.012, 0.042, 0.019)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material=trim_grey,
        name="latch_stem",
    )
    model.articulation(
        "panel_to_latch",
        ArticulationType.PRISMATIC,
        parent=panel,
        child=latch,
        origin=Origin(xyz=(PANEL_THICK / 2.0, 0.0, LATCH_LOCAL_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )

    for index, y_pos in enumerate((-BUTTON_Y_OFFSET, BUTTON_Y_OFFSET)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.009, 0.018, 0.010)),
            origin=Origin(xyz=(-0.004, 0.0, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.022, 0.014)),
            origin=Origin(xyz=(-0.007, 0.0, 0.0)),
            material=trim_grey,
            name="button_stem",
        )
        model.articulation(
            f"panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=Origin(xyz=(PANEL_THICK / 2.0, y_pos, BUTTON_LOCAL_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    panel = object_model.get_part("panel")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    dial = object_model.get_part("selector_dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_slide = object_model.get_articulation("panel_to_latch")
    button_0_slide = object_model.get_articulation("panel_to_button_0")
    button_1_slide = object_model.get_articulation("panel_to_button_1")

    ctx.expect_contact(panel, body, name="front panel mounts into body opening")
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="housing_shell",
        max_gap=0.010,
        max_penetration=0.001,
        name="closed lid sits just above housing rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="housing_shell",
        min_overlap=0.120,
        name="lid covers the cooker body",
    )

    latch_pos = ctx.part_world_position(latch)
    dial_pos = ctx.part_world_position(dial)
    button_0_pos = ctx.part_world_position(button_0)
    button_1_pos = ctx.part_world_position(button_1)
    ctx.check(
        "dial sits below latch",
        latch_pos is not None
        and dial_pos is not None
        and abs(latch_pos[1] - dial_pos[1]) < 0.005
        and dial_pos[2] < latch_pos[2] - 0.020,
        details=f"latch={latch_pos}, dial={dial_pos}",
    )
    ctx.check(
        "button cluster sits near front base below dial",
        button_0_pos is not None
        and button_1_pos is not None
        and dial_pos is not None
        and button_0_pos[2] < dial_pos[2] - 0.015
        and button_1_pos[2] < dial_pos[2] - 0.015
        and abs(button_0_pos[1] - button_1_pos[1]) > 0.025,
        details=f"dial={dial_pos}, button_0={button_0_pos}, button_1={button_1_pos}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
    lid_limits = lid_hinge.motion_limits
    opened_handle_aabb = None
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            opened_handle_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
    ctx.check(
        "lid front rises when opened",
        closed_handle_aabb is not None
        and opened_handle_aabb is not None
        and opened_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.060,
        details=f"closed={closed_handle_aabb}, opened={opened_handle_aabb}",
    )

    rest_latch_pos = ctx.part_world_position(latch)
    with ctx.pose({latch_slide: 0.004}):
        pressed_latch_pos = ctx.part_world_position(latch)
    ctx.check(
        "latch button presses inward",
        rest_latch_pos is not None
        and pressed_latch_pos is not None
        and pressed_latch_pos[0] < rest_latch_pos[0] - 0.003,
        details=f"rest={rest_latch_pos}, pressed={pressed_latch_pos}",
    )

    rest_button_0_pos = ctx.part_world_position(button_0)
    rest_button_1_pos = ctx.part_world_position(button_1)
    with ctx.pose({button_0_slide: 0.003, button_1_slide: 0.003}):
        pressed_button_0_pos = ctx.part_world_position(button_0)
        pressed_button_1_pos = ctx.part_world_position(button_1)
    ctx.check(
        "button cluster presses inward",
        rest_button_0_pos is not None
        and rest_button_1_pos is not None
        and pressed_button_0_pos is not None
        and pressed_button_1_pos is not None
        and pressed_button_0_pos[0] < rest_button_0_pos[0] - 0.002
        and pressed_button_1_pos[0] < rest_button_1_pos[0] - 0.002,
        details=(
            f"button_0_rest={rest_button_0_pos}, button_0_pressed={pressed_button_0_pos}, "
            f"button_1_rest={rest_button_1_pos}, button_1_pressed={pressed_button_1_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
