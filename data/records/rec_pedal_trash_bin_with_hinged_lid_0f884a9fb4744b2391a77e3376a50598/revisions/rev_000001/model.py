from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_RADIUS = 0.145
BODY_HEIGHT = 0.420
BODY_WALL = 0.003
BASE_HEIGHT = 0.026
LID_RADIUS = 0.149
LID_WALL = 0.0022
LID_DOME_HEIGHT = 0.048
PEDAL_WIDTH = 0.205


def make_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .circle(OUTER_RADIUS)
        .extrude(BODY_HEIGHT)
        .faces(">Z")
        .shell(-BODY_WALL)
    )

    rear_support = (
        cq.Workplane("XY")
        .circle(OUTER_RADIUS + 0.012)
        .circle(OUTER_RADIUS - 0.003)
        .extrude(0.028)
        .translate((0.0, 0.0, BODY_HEIGHT - 0.028))
        .intersect(
            cq.Workplane("XY").box(
                0.090,
                0.180,
                0.030,
                centered=(True, True, False),
            ).translate((-0.108, 0.0, BODY_HEIGHT - 0.028))
        )
    )

    bracket_y = 0.084
    bracket_a = (
        cq.Workplane("XY")
        .box(0.046, 0.016, 0.024, centered=(True, True, False))
        .translate((0.131, bracket_y, 0.036))
    )
    bracket_b = (
        cq.Workplane("XY")
        .box(0.046, 0.016, 0.024, centered=(True, True, False))
        .translate((0.131, -bracket_y, 0.036))
    )

    return shell.union(rear_support).union(bracket_a).union(bracket_b).clean()


def make_base_ring() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(OUTER_RADIUS + 0.008)
        .circle(OUTER_RADIUS - 0.018)
        .extrude(BASE_HEIGHT)
        .clean()
    )


def make_lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(LID_RADIUS)
        .workplane(offset=0.010)
        .circle(LID_RADIUS * 0.99)
        .workplane(offset=0.020)
        .circle(LID_RADIUS * 0.78)
        .workplane(offset=0.018)
        .circle(LID_RADIUS * 0.16)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .circle(LID_RADIUS - 0.013)
        .workplane(offset=0.008)
        .circle(LID_RADIUS * 0.90)
        .workplane(offset=0.018)
        .circle(LID_RADIUS * 0.68)
        .workplane(offset=0.017)
        .circle(LID_RADIUS * 0.06)
        .loft(combine=True)
    )
    dome = outer.cut(inner)

    dome = dome.translate((LID_RADIUS, 0.0, 0.0))

    rear_barrel = (
        cq.Workplane("XZ")
        .center(0.004, 0.010)
        .circle(0.0055)
        .extrude(0.122, both=True)
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.020, 0.122, 0.014, centered=(False, True, False))
        .translate((-0.004, 0.0, 0.0))
    )

    return dome.union(rear_barrel).union(rear_bridge).clean()


def make_pedal_bar() -> cq.Workplane:
    pivot_rod = cq.Workplane("XZ").circle(0.004).extrude(0.188, both=True)
    foot_bar = (
        cq.Workplane("XY")
        .box(0.022, PEDAL_WIDTH, 0.008, centered=(False, True, True))
        .translate((0.024, 0.0, -0.010))
    )
    side_arm_a = (
        cq.Workplane("XY")
        .box(0.034, 0.014, 0.012, centered=(False, True, True))
        .translate((0.0, 0.086, -0.005))
    )
    side_arm_b = (
        cq.Workplane("XY")
        .box(0.034, 0.014, 0.012, centered=(False, True, True))
        .translate((0.0, -0.086, -0.005))
    )

    return pivot_rod.union(foot_bar).union(side_arm_a).union(side_arm_b).clean()


def make_damper_cover() -> cq.Workplane:
    hood = (
        cq.Workplane("XY")
        .box(0.046, 0.108, 0.028, centered=(False, True, False))
        .translate((0.006, 0.0, -0.010))
        .edges("|Z")
        .fillet(0.006)
        .edges(">Z")
        .fillet(0.005)
    )
    hood = hood.faces("<Z").shell(-0.0025)

    hinge_barrel = (
        cq.Workplane("XZ")
        .center(0.0025, 0.0)
        .circle(0.0042)
        .extrude(0.078, both=True)
    )
    hinge_bridge = (
        cq.Workplane("XY")
        .box(0.012, 0.078, 0.009, centered=(False, True, False))
        .translate((0.0, 0.0, -0.004))
    )
    front_lip = (
        cq.Workplane("XY")
        .box(0.010, 0.060, 0.008, centered=(False, True, False))
        .translate((0.042, 0.0, -0.014))
    )

    return hood.union(hinge_barrel).union(hinge_bridge).union(front_lip).clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="step_bin")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.82, 1.0))
    trim = model.material("trim", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.11, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_body_shell(), "body_shell"),
        material=stainless,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(make_base_ring(), "base_ring"),
        material=trim,
        name="base_ring",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(make_lid_shell(), "lid_shell"),
        material=stainless,
        name="lid_shell",
    )

    pedal = model.part("pedal")
    pedal.visual(
        mesh_from_cadquery(make_pedal_bar(), "pedal_bar"),
        material=trim,
        name="pedal_bar",
    )

    damper_cover = model.part("damper_cover")
    damper_cover.visual(
        mesh_from_cadquery(make_damper_cover(), "damper_cover"),
        material=dark_trim,
        name="damper_cover",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-OUTER_RADIUS, 0.0, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )

    model.articulation(
        "pedal_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.158, 0.0, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=0.55,
        ),
    )

    model.articulation(
        "damper_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=damper_cover,
        origin=Origin(xyz=(-OUTER_RADIUS - 0.053, 0.0, BODY_HEIGHT + 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    damper_cover = object_model.get_part("damper_cover")

    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_hinge = object_model.get_articulation("pedal_hinge")
    damper_hinge = object_model.get_articulation("damper_hinge")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.004,
            max_penetration=0.0,
            name="lid seats on the rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.26,
            name="lid covers the bin opening",
        )

    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_shell")
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
            lid_open = ctx.part_element_world_aabb(lid, elem="lid_shell")
        ctx.check(
            "lid opens upward",
            lid_closed is not None
            and lid_open is not None
            and lid_open[1][2] > lid_closed[1][2] + 0.09,
            details=f"closed={lid_closed}, open={lid_open}",
        )

    pedal_rest = ctx.part_element_world_aabb(pedal, elem="pedal_bar")
    if pedal_hinge.motion_limits is not None and pedal_hinge.motion_limits.upper is not None:
        with ctx.pose({pedal_hinge: pedal_hinge.motion_limits.upper}):
            pedal_pressed = ctx.part_element_world_aabb(pedal, elem="pedal_bar")
        ctx.check(
            "pedal bar rotates downward",
            pedal_rest is not None
            and pedal_pressed is not None
            and pedal_pressed[0][2] < pedal_rest[0][2] - 0.012,
            details=f"rest={pedal_rest}, pressed={pedal_pressed}",
        )

    with ctx.pose({damper_hinge: 0.0}):
        ctx.expect_overlap(
            damper_cover,
            body,
            axes="y",
            elem_a="damper_cover",
            elem_b="body_shell",
            min_overlap=0.07,
            name="damper cover spans the rear hinge band",
        )

    cover_closed = ctx.part_element_world_aabb(damper_cover, elem="damper_cover")
    if damper_hinge.motion_limits is not None and damper_hinge.motion_limits.upper is not None:
        with ctx.pose({damper_hinge: damper_hinge.motion_limits.upper}):
            cover_open = ctx.part_element_world_aabb(damper_cover, elem="damper_cover")
        ctx.check(
            "damper cover lifts upward",
            cover_closed is not None
            and cover_open is not None
            and cover_open[1][2] > cover_closed[1][2] + 0.025,
            details=f"closed={cover_closed}, open={cover_open}",
        )

    return ctx.report()


object_model = build_object_model()
