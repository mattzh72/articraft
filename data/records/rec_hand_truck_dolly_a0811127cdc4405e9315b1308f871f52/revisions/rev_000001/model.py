from __future__ import annotations

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_truck")

    frame = model.part("frame")
    
    # CadQuery frame
    toe_plate = cq.Workplane("XY").box(0.4, 0.25, 0.01).translate((0, 0.125, 0.005))
    rails = cq.Workplane("XY").pushPoints([(-0.15, 0), (0.15, 0)]).circle(0.015).extrude(1.2)
    crossbars = cq.Workplane("YZ").pushPoints([(0, 0.3), (0, 0.6), (0, 0.9)]).circle(0.0125).extrude(0.15, both=True)
    
    path = cq.Workplane("XZ").center(0, 1.2).moveTo(-0.15, 0).threePointArc((0, 0.15), (0.15, 0))
    handle = cq.Workplane("XY").center(-0.15, 0).workplane(offset=1.2).circle(0.015).sweep(path)
    
    supports = cq.Workplane("XZ").workplane(offset=-0.01).pushPoints([(-0.15, 0.125), (0.15, 0.125)]).circle(0.015).extrude(0.12)
    axle = cq.Workplane("YZ").center(-0.1, 0.125).circle(0.01).extrude(0.25, both=True)
    
    frame_cq = toe_plate.union(rails).union(crossbars).union(handle).union(supports).union(axle)
    
    frame.visual(
        mesh_from_cadquery(frame_cq, "frame_mesh"),
        origin=Origin(),
        name="frame_visual"
    )
    
    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=0.125, length=0.06),
        origin=Origin(rpy=(0, math.pi / 2, 0)),
        name="left_wheel_visual"
    )
    
    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=0.125, length=0.06),
        origin=Origin(rpy=(0, math.pi / 2, 0)),
        name="right_wheel_visual"
    )
    
    model.articulation(
        "left_wheel_joint",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.22, -0.1, 0.125)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )
    
    model.articulation(
        "right_wheel_joint",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(0.22, -0.1, 0.125)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    ctx.allow_overlap(
        left_wheel,
        frame,
        reason="The wheel is mounted on the axle and overlaps with it.",
    )
    ctx.allow_overlap(
        right_wheel,
        frame,
        reason="The wheel is mounted on the axle and overlaps with it.",
    )

    ctx.expect_contact(left_wheel, frame, name="left wheel is mounted on axle")
    ctx.expect_contact(right_wheel, frame, name="right wheel is mounted on axle")

    return ctx.report()


object_model = build_object_model()
