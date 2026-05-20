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


BODY_WIDTH = 0.300
BODY_DEPTH = 0.255
BODY_HEIGHT = 0.205
HINGE_Y = -0.108
HINGE_Z = 0.2050
DIAL_Z = 0.079
DIAL_AXIS_Y = 0.130


def make_body_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .ellipse(0.126, 0.108)
        .workplane(offset=0.118)
        .ellipse(0.150, 0.128)
        .workplane(offset=0.087)
        .ellipse(0.144, 0.122)
        .loft(combine=True)
    )

    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.022)
        .ellipse(0.108, 0.088)
        .workplane(offset=0.210)
        .ellipse(0.114, 0.092)
        .loft(combine=True)
    )

    return outer.cut(inner)


def make_front_panel() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .workplane(offset=0.118)
        .center(0.0, 0.090)
        .rect(0.186, 0.110)
        .extrude(0.011)
    )


def make_lid_crown() -> cq.Workplane:
    lid_center_y = 0.118

    outer = (
        cq.Workplane("XY")
        .ellipse(0.134, 0.118)
        .workplane(offset=0.017)
        .ellipse(0.130, 0.114)
        .workplane(offset=0.019)
        .ellipse(0.122, 0.106)
        .loft(combine=True)
    )

    inner = (
        cq.Workplane("XY")
        .ellipse(0.118, 0.102)
        .workplane(offset=0.017)
        .ellipse(0.114, 0.098)
        .workplane(offset=0.013)
        .ellipse(0.108, 0.092)
        .loft(combine=True)
    )

    handle_recess = (
        cq.Workplane("XY")
        .center(0.0, 0.043)
        .rect(0.094, 0.036)
        .extrude(0.014)
        .translate((0.0, 0.0, 0.024))
    )

    vent_cut = (
        cq.Workplane("XY")
        .center(0.0, -0.020)
        .ellipse(0.020, 0.010)
        .extrude(0.016)
        .translate((0.0, 0.0, 0.021))
    )

    return outer.cut(inner).cut(handle_recess).cut(vent_cut).translate((0.0, lid_center_y, 0.0))


def make_vent_cap() -> cq.Workplane:
    lid_center_y = 0.118
    return (
        cq.Workplane("XY")
        .center(0.0, -0.020)
        .ellipse(0.028, 0.015)
        .extrude(0.009)
        .translate((0.0, lid_center_y, 0.022))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.94, 0.94, 0.92, 1.0))
    panel_black = model.material("panel_black", rgba=(0.14, 0.15, 0.16, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.75, 0.76, 0.78, 1.0))
    knob_graphite = model.material("knob_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    vent_black = model.material("vent_black", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shell(), "body_shell"),
        material=shell_white,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(make_front_panel(), "front_panel"),
        material=panel_black,
        name="control_panel",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(xyz=(0.0, 0.125, DIAL_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="dial_seat",
    )
    body.visual(
        Box((0.052, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.131, 0.140)),
        material=trim_silver,
        name="latch_button",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(make_lid_crown(), "lid_crown"),
        material=shell_white,
        name="crown",
    )
    lid.visual(
        mesh_from_cadquery(make_vent_cap(), "steam_vent"),
        material=vent_black,
        name="vent_cap",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_graphite,
        name="knob_skirt",
    )
    dial.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_graphite,
        name="knob_body",
    )
    dial.visual(
        Box((0.004, 0.002, 0.016)),
        origin=Origin(xyz=(0.0, 0.0185, 0.0)),
        material=trim_silver,
        name="indicator",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.85),
    )
    model.articulation(
        "selector_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, DIAL_AXIS_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("lid_hinge")
    selector_dial = object_model.get_articulation("selector_dial")

    ctx.expect_contact(
        dial,
        body,
        elem_a="knob_skirt",
        elem_b="dial_seat",
        name="dial seats on the front boss",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="z",
        positive_elem="latch_button",
        negative_elem="knob_skirt",
        min_gap=0.018,
        name="dial sits below the latch",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="crown",
            negative_elem="shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="closed lid rests just above the housing",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="crown",
            elem_b="shell",
            min_overlap=0.180,
            name="lid covers the cooking chamber",
        )

    closed_crown = ctx.part_element_world_aabb(lid, elem="crown")
    with ctx.pose({lid_hinge: 1.65}):
        open_crown = ctx.part_element_world_aabb(lid, elem="crown")

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_crown is not None
        and open_crown is not None
        and open_crown[1][2] > closed_crown[1][2] + 0.090
        and open_crown[0][1] < closed_crown[0][1] - 0.020,
        details=f"closed={closed_crown}, open={open_crown}",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({selector_dial: 1.4}):
        dial_turned = ctx.part_world_position(dial)

    ctx.check(
        "dial rotates about a fixed front axle",
        dial_rest is not None
        and dial_turned is not None
        and max(abs(a - b) for a, b in zip(dial_rest, dial_turned)) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turned}",
    )

    return ctx.report()


object_model = build_object_model()
