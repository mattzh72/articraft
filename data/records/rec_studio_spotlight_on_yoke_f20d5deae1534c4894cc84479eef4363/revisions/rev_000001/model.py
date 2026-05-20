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


BASE_RADIUS = 0.082
BASE_THICKNESS = 0.016
STEM_RADIUS = 0.009
STEM_HEIGHT = 0.118
HEAD_RADIUS = 0.019
HEAD_HEIGHT = 0.014
PAN_AXIS_Z = BASE_THICKNESS + STEM_HEIGHT + HEAD_HEIGHT

TILT_AXIS_X = 0.018
TILT_AXIS_Z = 0.087

CAN_RADIUS = 0.044
CAN_INNER_RADIUS = 0.038
CAN_LENGTH = 0.068
CAN_CENTER_X = 0.004
FRONT_RING_RADIUS = 0.048
FRONT_RING_LENGTH = 0.006
FRONT_RING_CENTER_X = 0.039

FILTER_HINGE_X = 0.047
FILTER_HINGE_Z = 0.041


def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def _stand_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    stem = (
        cq.Workplane("XY")
        .circle(STEM_RADIUS)
        .extrude(STEM_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    shoulder = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.016)
        .translate((0.0, 0.0, BASE_THICKNESS + STEM_HEIGHT - 0.004))
    )
    head = (
        cq.Workplane("XY")
        .circle(HEAD_RADIUS)
        .extrude(HEAD_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + STEM_HEIGHT))
    )
    return base.union(stem).union(shoulder).union(head)
def _can_body_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("YZ")
        .circle(CAN_RADIUS)
        .extrude(CAN_LENGTH)
        .translate((CAN_CENTER_X - (CAN_LENGTH * 0.5), 0.0, 0.0))
    )
    inner = (
        cq.Workplane("YZ")
        .circle(CAN_INNER_RADIUS)
        .extrude(CAN_LENGTH - 0.006)
        .translate((CAN_CENTER_X - ((CAN_LENGTH - 0.006) * 0.5) + 0.003, 0.0, 0.0))
    )
    rear_cap = (
        cq.Workplane("YZ")
        .circle(0.021)
        .extrude(0.010)
        .translate((-0.040, 0.0, 0.0))
    )
    return outer.cut(inner).union(rear_cap)


def _front_ring_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("YZ")
        .circle(FRONT_RING_RADIUS)
        .extrude(FRONT_RING_LENGTH)
        .translate((FRONT_RING_CENTER_X - (FRONT_RING_LENGTH * 0.5), 0.0, 0.0))
    )
    opening = (
        cq.Workplane("YZ")
        .circle(0.039)
        .extrude(FRONT_RING_LENGTH)
        .translate((FRONT_RING_CENTER_X - (FRONT_RING_LENGTH * 0.5), 0.0, 0.0))
    )
    return ring.cut(opening)
def _filter_frame_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(0.004, 0.116, 0.108)
        .translate((-0.001, 0.0, -0.054))
    )
    inner = (
        cq.Workplane("XY")
        .box(0.006, 0.092, 0.084)
        .translate((-0.001, 0.0, -0.054))
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight")

    stand_black = model.material("stand_black", rgba=(0.11, 0.11, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.22, 0.24, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.29, 0.30, 0.31, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_cadquery(_stand_shape(), "spotlight_stand"),
        material=stand_black,
        name="stand_body",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=graphite,
        name="hub",
    )
    yoke.visual(
        Box((0.020, 0.032, 0.024)),
        origin=Origin(xyz=(0.010, 0.0, 0.024)),
        material=graphite,
        name="spine",
    )
    yoke.visual(
        Box((0.020, 0.136, 0.020)),
        origin=Origin(xyz=(0.016, 0.0, 0.028)),
        material=graphite,
        name="bridge",
    )
    yoke.visual(
        Box((0.020, 0.018, 0.058)),
        origin=Origin(xyz=(0.016, 0.059, 0.067)),
        material=graphite,
        name="arm_0",
    )
    yoke.visual(
        Box((0.020, 0.018, 0.058)),
        origin=Origin(xyz=(0.016, -0.059, 0.067)),
        material=graphite,
        name="arm_1",
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_can_body_shape(), "spotlight_can_body"),
        material=satin_black,
        name="can_body",
    )
    can.visual(
        mesh_from_cadquery(_front_ring_shape(), "spotlight_front_ring"),
        material=dark_metal,
        name="front_ring",
    )
    can.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_0",
    )
    can.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_1",
    )
    can.visual(
        Box((0.008, 0.012, 0.006)),
        origin=Origin(xyz=(0.043, 0.016, 0.041)),
        material=dark_metal,
        name="filter_tab_0",
    )
    can.visual(
        Box((0.008, 0.012, 0.006)),
        origin=Origin(xyz=(0.043, -0.016, 0.041)),
        material=dark_metal,
        name="filter_tab_1",
    )
    can.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(FILTER_HINGE_X, 0.016, FILTER_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="filter_barrel_0",
    )
    can.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(FILTER_HINGE_X, -0.016, FILTER_HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="filter_barrel_1",
    )

    filter_holder = model.part("filter_holder")
    filter_holder.visual(
        mesh_from_cadquery(_filter_frame_shape(), "spotlight_filter_frame"),
        material=satin_black,
        name="holder_frame",
    )
    filter_holder.visual(
        Cylinder(radius=0.0032, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="holder_knuckle",
    )
    filter_holder.visual(
        Box((0.004, 0.018, 0.004)),
        origin=Origin(xyz=(0.001, 0.0, -0.002)),
        material=dark_metal,
        name="holder_strap",
    )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-2.2, upper=2.2),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(TILT_AXIS_X, 0.0, TILT_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.8, upper=0.95),
    )
    model.articulation(
        "filter_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=filter_holder,
        origin=Origin(xyz=(FILTER_HINGE_X, 0.0, FILTER_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=0.0, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    filter_holder = object_model.get_part("filter_holder")

    with ctx.pose(pan=0.0, tilt=0.0, filter_hinge=0.0):
        ctx.expect_gap(
            yoke,
            stand,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            name="yoke seats on the stand head",
        )
        ctx.expect_gap(
            filter_holder,
            can,
            axis="x",
            positive_elem="holder_frame",
            negative_elem="front_ring",
            min_gap=0.0015,
            max_gap=0.005,
            name="filter holder sits just ahead of the front ring",
        )
        ctx.expect_overlap(
            filter_holder,
            can,
            axes="yz",
            elem_a="holder_frame",
            elem_b="front_ring",
            min_overlap=0.088,
            name="filter holder covers the front opening",
        )

        rest_can = ctx.part_world_position(can)
        rest_ring = _center_from_aabb(ctx.part_element_world_aabb(can, elem="front_ring"))
        rest_holder = _center_from_aabb(ctx.part_element_world_aabb(filter_holder, elem="holder_frame"))

    with ctx.pose(pan=1.0, tilt=0.0, filter_hinge=0.0):
        panned_can = ctx.part_world_position(can)

    ctx.check(
        "pan swings the can around the stand axis",
        rest_can is not None and panned_can is not None and panned_can[1] > rest_can[1] + 0.01,
        details=f"rest={rest_can}, panned={panned_can}",
    )

    with ctx.pose(pan=0.0, tilt=0.7, filter_hinge=0.0):
        tilted_ring = _center_from_aabb(ctx.part_element_world_aabb(can, elem="front_ring"))

    ctx.check(
        "tilt raises the front ring",
        rest_ring is not None and tilted_ring is not None and tilted_ring[2] > rest_ring[2] + 0.015,
        details=f"rest={rest_ring}, tilted={tilted_ring}",
    )

    with ctx.pose(pan=0.0, tilt=0.0, filter_hinge=1.35):
        dropped_holder = _center_from_aabb(ctx.part_element_world_aabb(filter_holder, elem="holder_frame"))

    ctx.check(
        "filter holder swings away from the front ring",
        rest_holder is not None
        and dropped_holder is not None
        and dropped_holder[0] > rest_holder[0] + 0.045
        and dropped_holder[2] > rest_holder[2] + 0.020,
        details=f"rest={rest_holder}, dropped={dropped_holder}",
    )

    return ctx.report()


object_model = build_object_model()
