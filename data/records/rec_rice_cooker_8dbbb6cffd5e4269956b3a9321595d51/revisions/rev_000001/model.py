from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_HEIGHT = 0.190
HINGE_X = -0.145
HINGE_Z = 0.198
FRONT_PANEL_X = 0.151
KNOB_FACE_X = 0.1545


def _rounded_body_shell() -> cq.Workplane:
    """Swollen oval cooker body with a real top recess for the inner pot."""
    shell = (
        cq.Workplane("XY")
        .workplane(offset=0.014)
        .ellipse(0.125, 0.140)
        .workplane(offset=0.080)
        .ellipse(0.156, 0.168)
        .workplane(offset=0.096)
        .ellipse(0.148, 0.160)
        .loft(combine=True)
    )

    top_recess = (
        cq.Workplane("XY")
        .workplane(offset=0.132)
        .ellipse(0.109, 0.129)
        .extrude(0.080)
    )
    return shell.cut(top_recess)


def _inner_pot() -> cq.Workplane:
    """A shallow metal pot nested into the open top of the cooker body."""
    outer = (
        cq.Workplane("XY")
        .workplane(offset=0.130)
        .ellipse(0.111, 0.131)
        .extrude(0.040)
    )
    hollow = (
        cq.Workplane("XY")
        .workplane(offset=0.140)
        .ellipse(0.095, 0.115)
        .extrude(0.042)
    )
    return outer.cut(hollow)


def _lid_shell() -> cq.Workplane:
    """Domed lid authored in the lid hinge frame; local +X points forward."""
    lid = (
        cq.Workplane("XY")
        .center(0.145, 0.0)
        .ellipse(0.138, 0.158)
        .workplane(offset=0.026)
        .ellipse(0.132, 0.152)
        .workplane(offset=0.028)
        .ellipse(0.108, 0.128)
        .loft(combine=True)
    )

    hinge_barrel = (
        cq.Workplane("XY")
        .circle(0.0080)
        .circle(0.0032)
        .extrude(0.082)
        .translate((0.0, 0.0, -0.041))
        .rotate((0, 0, 0), (1, 0, 0), -90)
    )
    hinge_leaf = cq.Workplane("XY").box(0.055, 0.078, 0.006).translate(
        (0.030, 0.0, 0.009)
    )
    return lid.union(hinge_barrel).union(hinge_leaf)


def _lid_liner() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(0.145, 0.0)
        .ellipse(0.101, 0.121)
        .extrude(0.004)
        .translate((0.0, 0.0, -0.004))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    warm_white = model.material("warm_white_plastic", rgba=(0.92, 0.88, 0.78, 1.0))
    cream = model.material("cream_panel", rgba=(0.82, 0.78, 0.67, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.08, 0.075, 0.07, 1.0))
    graphite = model.material("graphite_hinge", rgba=(0.18, 0.18, 0.17, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.70, 0.70, 0.66, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shell(), "rounded_body_shell", tolerance=0.0008),
        material=warm_white,
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_inner_pot(), "inner_pot", tolerance=0.0007),
        material=brushed,
        name="inner_pot",
    )
    body.visual(
        Box((0.007, 0.116, 0.074)),
        origin=Origin(xyz=(FRONT_PANEL_X, 0.0, 0.086)),
        material=cream,
        name="control_panel",
    )
    body.visual(
        Box((0.030, 0.070, 0.018)),
        origin=Origin(xyz=(-0.139, -0.096, 0.181)),
        material=graphite,
        name="hinge_mount_0",
    )
    body.visual(
        Box((0.030, 0.070, 0.018)),
        origin=Origin(xyz=(-0.139, 0.096, 0.181)),
        material=graphite,
        name="hinge_mount_1",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.064),
        origin=Origin(xyz=(HINGE_X, -0.096, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.064),
        origin=Origin(xyz=(HINGE_X, 0.096, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_knuckle_1",
    )
    body.visual(
        Cylinder(radius=0.0034, length=0.256),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_pin",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "domed_lid_shell", tolerance=0.0008),
        material=warm_white,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_lid_liner(), "lid_inner_liner", tolerance=0.0007),
        material=brushed,
        name="inner_liner",
    )
    lid.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(0.202, 0.0, 0.055)),
        material=charcoal,
        name="steam_vent",
    )

    knob = model.part("selector_knob")
    selector = KnobGeometry(
        0.052,
        0.026,
        body_style="skirted",
        top_diameter=0.043,
        base_diameter=0.058,
        edge_radius=0.0014,
        skirt=KnobSkirt(0.060, 0.004, flare=0.04, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0011),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(selector, "front_selector_knob"),
        material=charcoal,
        name="knob_cap",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "selector_axis",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(KNOB_FACE_X, 0.0, 0.086), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("selector_knob")
    lid_hinge = object_model.get_articulation("lid_hinge")
    selector_axis = object_model.get_articulation("selector_axis")

    ctx.check(
        "lid hinge is rear revolute joint",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.2,
        details=f"type={lid_hinge.articulation_type}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "selector knob rotates continuously",
        selector_axis.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_axis.articulation_type}",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="lid_shell",
        reason=(
            "The rear hinge pin is intentionally captured by the lid barrel; "
            "a tiny interference fit keeps the articulated lid grounded."
        ),
    )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.23,
        elem_a="lid_shell",
        elem_b="shell",
        name="closed lid covers the cooker body footprint",
    )
    ctx.expect_gap(
        knob,
        body,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0005,
        positive_elem="knob_cap",
        negative_elem="control_panel",
        name="selector knob is seated on the front panel",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="y",
        min_overlap=0.075,
        elem_a="hinge_pin",
        elem_b="lid_shell",
        name="hinge pin spans through the lid barrel",
    )
    ctx.expect_within(
        body,
        lid,
        axes="xz",
        margin=0.003,
        inner_elem="hinge_pin",
        outer_elem="lid_shell",
        name="hinge pin is centered in the rear barrel",
    )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward from the rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.09,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
