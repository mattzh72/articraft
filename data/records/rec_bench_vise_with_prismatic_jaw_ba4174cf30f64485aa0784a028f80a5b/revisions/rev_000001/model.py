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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BALL_CENTER_Z = 0.046
BALL_RADIUS = 0.0335
GUIDE_START_X = 0.006
GUIDE_LENGTH = 0.030
GUIDE_CENTER_Z = 0.058
JAW_TRAVEL = 0.012


def _build_support_shape() -> object:
    foot = cq.Workplane("XY").circle(0.047).extrude(0.012)

    support_block = cq.Workplane("XY").box(0.062, 0.076, 0.044).translate((-0.012, 0.0, 0.033))
    ball_cavity = cq.Workplane("XY").sphere(BALL_RADIUS + 0.0002).translate((0.0, 0.0, BALL_CENTER_Z))
    top_opening = cq.Workplane("XY").box(0.100, 0.100, 0.052).translate((0.0, 0.0, 0.078))
    front_opening = cq.Workplane("XY").box(0.066, 0.090, 0.054).translate((0.034, 0.0, 0.046))

    return foot.union(support_block.cut(ball_cavity).cut(top_opening).cut(front_opening))


def _build_front_jaw_shape() -> object:
    jaw_block = cq.Workplane("XY").box(0.022, 0.026, 0.046).translate((0.011, 0.0, 0.011))
    guide_tunnel = cq.Workplane("XY").box(0.024, 0.020, 0.012).translate((0.011, 0.0, -0.001))
    screw_bore = cq.Workplane("YZ").center(0.0, 0.014).circle(0.0038).extrude(0.024)
    return jaw_block.cut(guide_tunnel).cut(screw_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jewelers_vise")

    satin_black = model.material("satin_black", rgba=(0.17, 0.18, 0.20, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.26, 0.27, 0.29, 1.0))
    brass = model.material("brass", rgba=(0.71, 0.58, 0.25, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_support_shape(), "jewelers_vise_support"),
        material=satin_black,
        name="support",
    )

    body = model.part("body")
    body.visual(
        Sphere(radius=BALL_RADIUS),
        material=dark_steel,
        name="ball",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0435)),
        material=dark_steel,
        name="neck",
    )
    body.visual(
        Box((0.030, 0.032, 0.012)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0595)),
        material=gunmetal,
        name="carriage",
    )
    body.visual(
        Box((0.014, 0.022, 0.032)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0785)),
        material=gunmetal,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.002, 0.018, 0.018)),
        origin=Origin(xyz=(0.004, 0.0, 0.081)),
        material=brass,
        name="fixed_face",
    )
    body.visual(
        Box((0.024, 0.024, 0.006)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0955)),
        material=dark_steel,
        name="top_cap",
    )
    body.visual(
        Box((GUIDE_LENGTH, 0.018, 0.008)),
        origin=Origin(xyz=(GUIDE_START_X + GUIDE_LENGTH * 0.5, 0.0, GUIDE_CENTER_Z)),
        material=dark_steel,
        name="guide",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_build_front_jaw_shape(), "jewelers_vise_front_jaw"),
        material=gunmetal,
        name="jaw",
    )
    jaw.visual(
        Box((0.002, 0.018, 0.018)),
        origin=Origin(xyz=(0.001, 0.0, 0.022)),
        material=brass,
        name="jaw_face",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0026, length=0.020),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.0085, length=0.010),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="grip",
    )
    knob.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.0015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="collar",
    )
    knob.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="cap",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BALL_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5),
    )
    model.articulation(
        "body_to_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(GUIDE_START_X, 0.0, GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.05, lower=0.0, upper=JAW_TRAVEL),
    )
    model.articulation(
        "jaw_to_knob",
        ArticulationType.CONTINUOUS,
        parent=jaw,
        child=knob,
        origin=Origin(xyz=(0.022, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    knob = object_model.get_part("knob")

    body_rotate = object_model.get_articulation("base_to_body")
    jaw_slide = object_model.get_articulation("body_to_jaw")
    knob_spin = object_model.get_articulation("jaw_to_knob")

    ctx.expect_gap(
        jaw,
        body,
        axis="x",
        positive_elem="jaw_face",
        negative_elem="fixed_face",
        min_gap=0.0005,
        max_gap=0.0020,
        name="closed jaws keep a hairline opening",
    )
    ctx.expect_overlap(
        jaw,
        body,
        axes="x",
        elem_a="jaw",
        elem_b="guide",
        min_overlap=0.020,
        name="closed jaw remains deeply engaged on the guide",
    )
    ctx.expect_overlap(
        knob,
        jaw,
        axes="x",
        elem_a="shaft",
        elem_b="jaw",
        min_overlap=0.016,
        name="lead screw shaft stays inserted through the jaw",
    )

    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        ctx.expect_gap(
            jaw,
            body,
            axis="x",
            positive_elem="jaw_face",
            negative_elem="fixed_face",
            min_gap=0.012,
            max_gap=0.0145,
            name="open pose widens the clamping gap",
        )
        ctx.expect_overlap(
            jaw,
            body,
            axes="x",
            elem_a="jaw",
            elem_b="guide",
            min_overlap=0.014,
            name="open jaw still retains guide overlap",
        )

    with ctx.pose({knob_spin: math.pi / 2.0}):
        ctx.expect_overlap(
            knob,
            jaw,
            axes="x",
            elem_a="shaft",
            elem_b="jaw",
            min_overlap=0.016,
            name="knob can rotate while staying engaged on axis",
        )

    knob_rest = ctx.part_world_position(knob)
    knob_rotated = None
    with ctx.pose({body_rotate: math.pi / 2.0}):
        knob_rotated = ctx.part_world_position(knob)

    ctx.check(
        "vise body yaws in the ball support",
        knob_rest is not None
        and knob_rotated is not None
        and knob_rest[0] > 0.02
        and abs(knob_rotated[0]) < 0.012
        and abs(knob_rotated[1] - knob_rest[0]) < 0.012
        and abs(knob_rotated[2] - knob_rest[2]) < 1e-6,
        details=f"rest={knob_rest}, rotated={knob_rotated}",
    )

    ctx.check(
        "ball support sits below the vise body",
        ctx.part_world_position(base) is not None
        and ctx.part_world_position(body) is not None
        and ctx.part_world_position(body)[2] > ctx.part_world_position(base)[2] + 0.03,
        details=f"base={ctx.part_world_position(base)}, body={ctx.part_world_position(body)}",
    )

    return ctx.report()


object_model = build_object_model()
