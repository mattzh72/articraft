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


BASE_RADIUS = 0.070
BASE_THICKNESS = 0.012
STEM_RADIUS = 0.012
STEM_HEIGHT = 0.090
COLLAR_RADIUS = 0.018
COLLAR_HEIGHT = 0.012
PAN_ORIGIN_Z = BASE_THICKNESS + STEM_HEIGHT + COLLAR_HEIGHT / 2.0 + 0.010

YOKE_ARM_OFFSET_Y = 0.064
YOKE_ARM_THICKNESS = 0.008
YOKE_TILT_Z = 0.055

CAN_RADIUS = 0.044
CAN_LENGTH = 0.120
CAN_INNER_RADIUS = 0.038
FRONT_RING_RADIUS = 0.050
FRONT_RING_LENGTH = 0.018
TRUNNION_RADIUS = 0.007
TRUNNION_LENGTH = 0.014

SWITCH_WIDTH = 0.016
SWITCH_HEIGHT = 0.022
SWITCH_THICKNESS = 0.008
SWITCH_PIVOT_X = -0.062
SWITCH_PIVOT_Z = 0.010


def _build_can_shell_shape() -> cq.Workplane:
    shell = cq.Workplane("YZ").circle(CAN_RADIUS).extrude(CAN_LENGTH / 2.0, both=True)
    cavity = (
        cq.Workplane("YZ")
        .circle(CAN_INNER_RADIUS)
        .extrude((CAN_LENGTH - 0.008) / 2.0, both=True)
        .translate((0.004, 0.0, 0.0))
    )
    switch_pad = cq.Workplane("XY").box(0.004, 0.024, 0.032).translate((-0.060, 0.0, 0.000))

    return shell.cut(cavity).union(switch_pad)


def _build_front_ring_shape() -> cq.Workplane:
    ring = cq.Workplane("YZ").circle(FRONT_RING_RADIUS).extrude(FRONT_RING_LENGTH / 2.0, both=True)
    bore = cq.Workplane("YZ").circle(CAN_INNER_RADIUS).extrude((FRONT_RING_LENGTH + 0.002) / 2.0, both=True)
    return ring.cut(bore).translate((0.065, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_spotlight")

    base_metal = model.material("base_metal", rgba=(0.16, 0.16, 0.18, 1.0))
    yoke_metal = model.material("yoke_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    can_finish = model.material("can_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    ring_finish = model.material("ring_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    switch_finish = model.material("switch_finish", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_metal,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=STEM_RADIUS, length=STEM_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + STEM_HEIGHT / 2.0)),
        material=base_metal,
        name="stem",
    )
    base.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + STEM_HEIGHT + COLLAR_HEIGHT / 2.0)),
        material=base_metal,
        name="pan_collar",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=COLLAR_RADIUS, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=yoke_metal,
        name="pan_hub",
    )
    yoke.visual(
        Box((0.018, YOKE_ARM_THICKNESS, 0.081)),
        origin=Origin(xyz=(0.0, YOKE_ARM_OFFSET_Y, 0.0445)),
        material=yoke_metal,
        name="arm_pos",
    )
    yoke.visual(
        Box((0.018, YOKE_ARM_THICKNESS, 0.081)),
        origin=Origin(xyz=(0.0, -YOKE_ARM_OFFSET_Y, 0.0445)),
        material=yoke_metal,
        name="arm_neg",
    )
    yoke.visual(
        Box((0.040, 0.128, 0.008)),
        origin=Origin(xyz=(-0.018, 0.0, 0.000)),
        material=yoke_metal,
        name="rear_band",
    )
    yoke.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.060, YOKE_TILT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yoke_metal,
        name="collar_pos",
    )
    yoke.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, -0.060, YOKE_TILT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yoke_metal,
        name="collar_neg",
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_build_can_shell_shape(), "can_shell"),
        material=can_finish,
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(0.0, 0.050, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ring_finish,
        name="trunnion_pos",
    )
    can.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ring_finish,
        name="trunnion_neg",
    )
    can.visual(
        mesh_from_cadquery(_build_front_ring_shape(), "front_ring"),
        material=ring_finish,
        name="front_ring",
    )

    switch = model.part("switch")
    switch.visual(
        Box((SWITCH_THICKNESS, SWITCH_WIDTH, SWITCH_HEIGHT)),
        origin=Origin(xyz=(-SWITCH_THICKNESS / 2.0, 0.0, -SWITCH_HEIGHT / 2.0)),
        material=switch_finish,
        name="paddle",
    )
    switch.visual(
        Box((0.010, SWITCH_WIDTH, 0.008)),
        origin=Origin(xyz=(-0.005, 0.0, -0.018)),
        material=switch_finish,
        name="toe",
    )
    switch.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(xyz=(-0.001, 0.0, -0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=switch_finish,
        name="pivot",
    )

    model.articulation(
        "pan_joint",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, YOKE_TILT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.60,
            upper=0.90,
        ),
    )
    model.articulation(
        "switch_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=switch,
        origin=Origin(xyz=(SWITCH_PIVOT_X, 0.0, SWITCH_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=-0.22,
            upper=0.22,
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    switch = object_model.get_part("switch")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")
    switch_hinge = object_model.get_articulation("switch_hinge")

    ctx.expect_gap(
        yoke,
        can,
        axis="y",
        positive_elem="arm_pos",
        negative_elem="can_shell",
        min_gap=0.012,
        max_gap=0.020,
        name="positive arm clears the can body",
    )
    ctx.expect_gap(
        can,
        yoke,
        axis="y",
        positive_elem="can_shell",
        negative_elem="arm_neg",
        min_gap=0.012,
        max_gap=0.020,
        name="negative arm clears the can body",
    )
    ctx.expect_gap(
        yoke,
        can,
        axis="y",
        positive_elem="collar_pos",
        negative_elem="trunnion_pos",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="positive trunnion seats in the yoke collar",
    )
    ctx.expect_gap(
        can,
        yoke,
        axis="y",
        positive_elem="trunnion_neg",
        negative_elem="collar_neg",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="negative trunnion seats in the yoke collar",
    )
    ctx.expect_gap(
        can,
        yoke,
        axis="z",
        positive_elem="can_shell",
        negative_elem="rear_band",
        min_gap=0.004,
        max_gap=0.012,
        name="can shell stays above the yoke band",
    )
    ctx.expect_gap(
        can,
        switch,
        axis="x",
        positive_elem="can_shell",
        negative_elem="paddle",
        max_gap=0.0005,
        max_penetration=0.0,
        name="switch seats flush to the rear switch pad",
    )

    rest_ring_center = _aabb_center(ctx.part_element_world_aabb(can, elem="front_ring"))
    with ctx.pose({tilt_joint: 0.65}):
        raised_ring_center = _aabb_center(ctx.part_element_world_aabb(can, elem="front_ring"))
    ctx.check(
        "tilt joint raises the front ring",
        rest_ring_center is not None
        and raised_ring_center is not None
        and raised_ring_center[2] > rest_ring_center[2] + 0.02,
        details=f"rest={rest_ring_center}, raised={raised_ring_center}",
    )

    with ctx.pose({pan_joint: math.pi / 4.0}):
        panned_ring_center = _aabb_center(ctx.part_element_world_aabb(can, elem="front_ring"))
    ctx.check(
        "pan joint swings the head around the vertical axis",
        rest_ring_center is not None
        and panned_ring_center is not None
        and abs(panned_ring_center[1] - rest_ring_center[1]) > 0.02,
        details=f"rest={rest_ring_center}, panned={panned_ring_center}",
    )

    rest_switch_center = _aabb_center(ctx.part_element_world_aabb(switch, elem="paddle"))
    with ctx.pose({switch_hinge: 0.20}):
        rocked_switch_center = _aabb_center(ctx.part_element_world_aabb(switch, elem="paddle"))
    ctx.check(
        "rocker switch pivots on its local hinge",
        rest_switch_center is not None
        and rocked_switch_center is not None
        and abs(rocked_switch_center[0] - rest_switch_center[0]) > 0.0015,
        details=f"rest={rest_switch_center}, rocked={rocked_switch_center}",
    )

    return ctx.report()


object_model = build_object_model()
