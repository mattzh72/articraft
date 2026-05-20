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


def _base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.165, 0.094, 0.012, centered=(True, True, False))
    cheek = cq.Workplane("XY").box(0.104, 0.012, 0.040, centered=(True, True, False))
    cheek_left = cheek.translate((0.0, 0.040, 0.012))
    cheek_right = cheek.translate((0.0, -0.040, 0.012))

    trunnion_hole = cq.Workplane("XZ").center(0.0, 0.034).circle(0.0085).extrude(0.020)
    hole_left = trunnion_hole.translate((0.0, 0.050, 0.0))
    hole_right = trunnion_hole.translate((0.0, -0.030, 0.0))

    clamp_slot = cq.Workplane("XY").box(0.040, 0.010, 0.020, centered=(True, True, False))
    slot_left = clamp_slot.translate((0.0, 0.028, -0.004))
    slot_right = clamp_slot.translate((0.0, -0.028, -0.004))

    return plate.union(cheek_left).union(cheek_right).cut(hole_left).cut(hole_right).cut(slot_left).cut(slot_right)


def _body_casting_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(0.126, 0.066, 0.020, centered=(True, True, False))
    fixed_jaw = (
        cq.Workplane("XY")
        .box(0.020, 0.066, 0.024, centered=(True, True, False))
        .translate((-0.018, 0.0, 0.036))
    )
    buttress = cq.Workplane("XY").box(0.016, 0.010, 0.018, centered=(True, True, False))
    buttress_left = buttress.translate((-0.028, 0.027, 0.020))
    buttress_right = buttress.translate((-0.028, -0.027, 0.020))
    front_pad = (
        cq.Workplane("XY")
        .box(0.014, 0.038, 0.014, centered=(True, True, False))
        .translate((0.054, 0.0, 0.008))
    )
    trunnion = cq.Workplane("XZ").circle(0.0085).extrude(0.016)
    trunnion_neck = cq.Workplane("XY").box(0.018, 0.014, 0.018, centered=(True, True, True))
    neck_left = trunnion_neck.translate((0.0, 0.040, 0.0))
    neck_right = trunnion_neck.translate((0.0, -0.040, 0.0))
    trunnion_left = trunnion.translate((0.0, 0.048, 0.0))
    trunnion_right = trunnion.translate((0.0, -0.032, 0.0))

    return (
        carriage
        .union(fixed_jaw)
        .union(buttress_left)
        .union(buttress_right)
        .union(front_pad)
        .union(neck_left)
        .union(neck_right)
        .union(trunnion_left)
        .union(trunnion_right)
    )


def _front_jaw_shape() -> cq.Workplane:
    skirt = cq.Workplane("XY").box(0.040, 0.010, 0.015, centered=(True, True, False))
    skirt_left = skirt.translate((0.017, 0.020, -0.0075))
    skirt_right = skirt.translate((0.017, -0.020, -0.0075))
    riser = cq.Workplane("XY").box(0.014, 0.012, 0.020, centered=(True, True, False))
    riser_left = riser.translate((0.010, 0.024, -0.0005))
    riser_right = riser.translate((0.010, -0.024, -0.0005))
    jaw_block = (
        cq.Workplane("XY")
        .box(0.024, 0.070, 0.032, centered=(True, True, False))
        .translate((0.015, 0.0, 0.012))
    )

    return skirt_left.union(skirt_right).union(riser_left).union(riser_right).union(jaw_block)


def _handwheel_shape() -> cq.Workplane:
    rim = cq.Workplane("YZ").circle(0.024).circle(0.019).extrude(0.004, both=True)
    hub = cq.Workplane("YZ").circle(0.007).extrude(0.006, both=True)

    wheel = rim.union(hub)
    spoke = cq.Workplane("XY").box(0.004, 0.005, 0.024).translate((0.0, 0.0, 0.012))
    for angle_deg in (0.0, 120.0, 240.0):
        wheel = wheel.union(spoke.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg))

    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_drill_vise")

    cast_iron = model.material("cast_iron", rgba=(0.27, 0.29, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "vise_base"), material=cast_iron, name="base_shell")

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_casting_shape(), "vise_body"), material=cast_iron, name="casting")
    body.visual(
        Box((0.116, 0.028, 0.016)),
        origin=Origin(xyz=(0.006, 0.0, 0.0275)),
        material=steel,
        name="guide",
    )
    body.visual(
        Box((0.004, 0.060, 0.028)),
        origin=Origin(xyz=(-0.006, 0.0, 0.036)),
        material=steel,
        name="fixed_jaw_face",
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(mesh_from_cadquery(_front_jaw_shape(), "front_jaw"), material=cast_iron, name="jaw_body")
    front_jaw.visual(
        Box((0.004, 0.060, 0.028)),
        origin=Origin(xyz=(0.004, 0.0, 0.022)),
        material=steel,
        name="jaw_face",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(mesh_from_cadquery(_handwheel_shape(), "handwheel"), material=steel, name="wheel")
    handwheel.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="grip",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.0,
            lower=-math.radians(35.0),
            upper=math.radians(35.0),
        ),
    )
    model.articulation(
        "body_to_front_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.08,
            lower=0.0,
            upper=0.040,
        ),
    )
    model.articulation(
        "front_jaw_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=handwheel,
        origin=Origin(xyz=(0.041, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )

    return model


def _aabb_center(bounds):
    if bounds is None:
        return None
    lower, upper = bounds
    return tuple((a + b) * 0.5 for a, b in zip(lower, upper))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    front_jaw = object_model.get_part("front_jaw")
    handwheel = object_model.get_part("handwheel")

    tilt = object_model.get_articulation("base_to_body")
    slide = object_model.get_articulation("body_to_front_jaw")
    wheel_spin = object_model.get_articulation("front_jaw_to_handwheel")

    ctx.allow_overlap(
        base,
        body,
        elem_a="base_shell",
        elem_b="casting",
        reason="The body is intentionally captured in the base's trunnion cradle around the side pivot.",
    )
    ctx.allow_overlap(
        body,
        front_jaw,
        elem_a="casting",
        elem_b="jaw_body",
        reason="The moving jaw is represented as a close-fitting guided slide on the vise body.",
    )

    ctx.expect_gap(
        front_jaw,
        body,
        axis="x",
        positive_elem="jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.004,
        max_gap=0.010,
        name="jaw closes to a narrow clamping gap",
    )
    ctx.expect_overlap(
        front_jaw,
        body,
        axes="yz",
        min_overlap=0.035,
        name="front jaw stays centered on the top guide",
    )

    closed_face = _aabb_center(ctx.part_element_world_aabb(front_jaw, elem="jaw_face"))
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_overlap(
            front_jaw,
            body,
            axes="x",
            min_overlap=0.026,
            name="opened jaw retains guide engagement",
        )
        open_face = _aabb_center(ctx.part_element_world_aabb(front_jaw, elem="jaw_face"))

    ctx.check(
        "jaw opens forward along the screw axis",
        closed_face is not None
        and open_face is not None
        and open_face[0] > closed_face[0] + 0.030,
        details=f"closed_face={closed_face}, open_face={open_face}",
    )

    handwheel_rest = ctx.part_world_position(handwheel)
    with ctx.pose({tilt: math.radians(25.0)}):
        handwheel_tilted = ctx.part_world_position(handwheel)
    ctx.check(
        "positive tilt raises the vise nose",
        handwheel_rest is not None
        and handwheel_tilted is not None
        and handwheel_tilted[2] > handwheel_rest[2] + 0.010,
        details=f"rest={handwheel_rest}, tilted={handwheel_tilted}",
    )

    grip_rest = _aabb_center(ctx.part_element_world_aabb(handwheel, elem="grip"))
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        grip_quarter_turn = _aabb_center(ctx.part_element_world_aabb(handwheel, elem="grip"))
    ctx.check(
        "handwheel grip sweeps around the screw axis",
        grip_rest is not None
        and grip_quarter_turn is not None
        and grip_quarter_turn[2] < grip_rest[2] - 0.015
        and abs(grip_quarter_turn[1] - grip_rest[1]) > 0.015,
        details=f"grip_rest={grip_rest}, grip_quarter_turn={grip_quarter_turn}",
    )

    return ctx.report()


object_model = build_object_model()
