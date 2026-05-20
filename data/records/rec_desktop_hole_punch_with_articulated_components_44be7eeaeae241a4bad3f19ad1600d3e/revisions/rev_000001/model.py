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


BASE_LENGTH = 0.118
BASE_WIDTH = 0.052
BASE_HEIGHT = 0.034
HINGE_X = -0.040
HINGE_Z = 0.047
PUNCH_X = 0.028


def _make_base_shell() -> cq.Workplane:
    rear_body = (
        cq.Workplane("XY")
        .box(0.078, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .translate((-0.015, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(BASE_HEIGHT)
        .translate((0.026, 0.0, 0.0))
    )
    shell = rear_body.union(nose)

    hinge_cheek_y = 0.016
    cheek = (
        cq.Workplane("XY")
        .box(0.016, 0.010, 0.018, centered=(True, True, False))
        .translate((HINGE_X, hinge_cheek_y, BASE_HEIGHT))
    )
    shell = shell.union(cheek).union(cheek.mirror("XZ"))

    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.054, 0.038, 0.016, centered=(True, True, False))
        .translate((0.022, 0.007, 0.005))
    )
    punch_hole = (
        cq.Workplane("XY")
        .circle(0.0045)
        .extrude(0.028)
        .translate((PUNCH_X, 0.0, 0.008))
    )

    return shell.cut(drawer_cavity).cut(punch_hole)


def _make_arm_body() -> cq.Workplane:
    lower_beam = (
        cq.Workplane("XY")
        .box(0.068, 0.024, 0.011, centered=(True, True, True))
        .translate((0.044, 0.0, -0.004))
    )
    finger_pad = (
        cq.Workplane("XY")
        .box(0.050, 0.040, 0.009, centered=(True, True, True))
        .translate((0.048, 0.0, 0.005))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.020, 0.020, 0.008, centered=(True, True, True))
        .translate((0.087, 0.0, -0.003))
    )
    return lower_beam.union(finger_pad).union(nose)


def _make_drawer_tray() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(0.050, 0.034, 0.014, centered=(True, True, True))
        .translate((0.0, -0.017, 0.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(0.046, 0.030, 0.012, centered=(True, True, True))
        .translate((0.0, -0.017, 0.002))
    )
    grip = (
        cq.Workplane("XY")
        .box(0.018, 0.004, 0.010, centered=(True, True, True))
        .translate((0.0, 0.001, 0.0))
    )
    return outer.cut(inner).union(grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_one_hole_punch")

    body_black = model.material("body_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    smoke = model.material("smoke", rgba=(0.32, 0.33, 0.35, 0.78))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shell(), "punch_base"),
        material=body_black,
        name="shell",
    )
    base.visual(
        Cylinder(radius=0.0085, length=0.0015),
        origin=Origin(xyz=(PUNCH_X, 0.0, BASE_HEIGHT + 0.00075)),
        material=steel,
        name="die_plate",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_make_arm_body(), "punch_arm"),
        material=graphite,
        name="body",
    )
    arm.visual(
        Box((0.008, 0.018, 0.010)),
        origin=Origin(xyz=(0.007, 0.0, -0.001)),
        material=graphite,
        name="neck",
    )
    arm.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="barrel",
    )
    arm.visual(
        Cylinder(radius=0.0036, length=0.017),
        origin=Origin(xyz=(0.068, 0.0, -0.0135)),
        material=steel,
        name="pin",
    )
    arm.visual(
        Box((0.005, 0.004, 0.006)),
        origin=Origin(xyz=(0.086, 0.010, 0.004)),
        material=graphite,
        name="lug_0",
    )
    arm.visual(
        Box((0.005, 0.004, 0.006)),
        origin=Origin(xyz=(0.086, -0.010, 0.004)),
        material=graphite,
        name="lug_1",
    )

    clip = model.part("clip")
    clip.visual(
        Cylinder(radius=0.0027, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="barrel",
    )
    clip.visual(
        Box((0.028, 0.012, 0.0018)),
        origin=Origin(xyz=(-0.014, 0.0, -0.0012)),
        material=steel,
        name="tab",
    )
    clip.visual(
        Box((0.004, 0.010, 0.005)),
        origin=Origin(xyz=(-0.028, 0.0, 0.001)),
        material=steel,
        name="hook",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_make_drawer_tray(), "punch_drawer"),
        material=smoke,
        name="tray",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "arm_to_clip",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=clip,
        origin=Origin(xyz=(0.090, 0.0, 0.0116)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "base_to_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.022, BASE_WIDTH / 2.0, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.20,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    clip = object_model.get_part("clip")
    drawer = object_model.get_part("drawer")
    arm_hinge = object_model.get_articulation("base_to_arm")
    clip_hinge = object_model.get_articulation("arm_to_clip")
    drawer_slide = object_model.get_articulation("base_to_drawer")

    ctx.allow_overlap(
        base,
        drawer,
        elem_a="shell",
        elem_b="tray",
        reason="The pedestal base is represented as a simplified closed shell proxy around the captured confetti drawer cavity.",
    )

    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="body",
        negative_elem="die_plate",
        max_gap=0.004,
        max_penetration=0.0,
        name="arm rests just above the punch die",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        elem_a="body",
        elem_b="shell",
        min_overlap=0.028,
        name="arm covers the punch body in plan",
    )
    ctx.expect_gap(
        clip,
        arm,
        axis="z",
        positive_elem="tab",
        negative_elem="body",
        max_gap=0.003,
        max_penetration=0.0,
        name="storage clip folds flat on the arm nose",
    )
    ctx.expect_overlap(
        clip,
        arm,
        axes="xy",
        elem_a="tab",
        elem_b="body",
        min_overlap=0.010,
        name="storage clip sits over the arm nose footprint",
    )
    ctx.expect_overlap(
        drawer,
        base,
        axes="xz",
        elem_a="tray",
        elem_b="shell",
        min_overlap=0.012,
        name="drawer stays aligned with the base slot",
    )

    rest_arm_aabb = ctx.part_element_world_aabb(arm, elem="body")
    rest_clip_aabb = ctx.part_element_world_aabb(clip, elem="tab")
    drawer_rest_pos = ctx.part_world_position(drawer)

    with ctx.pose({arm_hinge: 1.0}):
        open_arm_aabb = ctx.part_element_world_aabb(arm, elem="body")

    with ctx.pose({clip_hinge: 1.0}):
        raised_clip_aabb = ctx.part_element_world_aabb(clip, elem="tab")

    with ctx.pose({drawer_slide: 0.018}):
        ctx.expect_overlap(
            drawer,
            base,
            axes="xz",
            elem_a="tray",
            elem_b="shell",
            min_overlap=0.012,
            name="drawer remains captured when extended",
        )
        drawer_extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "arm opens upward",
        rest_arm_aabb is not None
        and open_arm_aabb is not None
        and open_arm_aabb[1][2] > rest_arm_aabb[1][2] + 0.018,
        details=f"rest={rest_arm_aabb}, open={open_arm_aabb}",
    )
    ctx.check(
        "clip lifts away from the arm",
        rest_clip_aabb is not None
        and raised_clip_aabb is not None
        and raised_clip_aabb[1][2] > rest_clip_aabb[1][2] + 0.010,
        details=f"rest={rest_clip_aabb}, raised={raised_clip_aabb}",
    )
    ctx.check(
        "drawer slides out from the side",
        drawer_rest_pos is not None
        and drawer_extended_pos is not None
        and drawer_extended_pos[1] > drawer_rest_pos[1] + 0.015,
        details=f"rest={drawer_rest_pos}, extended={drawer_extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
