from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    # 1. Tray (Root)
    # A lofted tub with sloped front and back.
    tray_cq = (
        cq.Workplane("XY")
        .workplane(offset=0).rect(0.4, 0.5)
        .workplane(offset=0.3).rect(0.8, 0.65)
        .loft()
        .faces(">Z")
        .shell(-0.02)
    )

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(tray_cq, "tray_mesh"))

    # 2. Frame (Handles, cross braces, and legs)
    # Handles converge towards the front.
    left_handle = (
        cq.Workplane("XY")
        .box(1.6, 0.04, 0.04)
        .rotate((0, 0, 0), (0, 0, 1), -6)
        .translate((-0.05, 0.22, 0))
    )
    right_handle = (
        cq.Workplane("XY")
        .box(1.6, 0.04, 0.04)
        .rotate((0, 0, 0), (0, 0, 1), 6)
        .translate((-0.05, -0.22, 0))
    )
    
    # Cross braces to connect the handles
    cross_brace_1 = cq.Workplane("XY").center(0.3, 0).box(0.04, 0.4, 0.04)
    cross_brace_2 = cq.Workplane("XY").center(-0.2, 0).box(0.04, 0.5, 0.04)

    # Legs at the rear
    legs = (
        cq.Workplane("XY")
        .center(-0.4, 0.256)
        .box(0.04, 0.04, 0.2)
        .translate((0, 0, -0.1))
        .union(
            cq.Workplane("XY")
            .center(-0.4, -0.256)
            .box(0.04, 0.04, 0.2)
            .translate((0, 0, -0.1))
        )
    )

    frame_cq = left_handle.union(right_handle).union(cross_brace_1).union(cross_brace_2).union(legs)

    frame = model.part("frame")
    # Shift frame visual so its top (Z=0.02) sits exactly at Z=0 to touch the tray bottom
    frame.visual(mesh_from_cadquery(frame_cq, "frame_mesh"), origin=Origin(xyz=(0, 0, -0.02)))

    # Mount frame to tray
    model.articulation(
        "tray_to_frame",
        ArticulationType.FIXED,
        parent=tray,
        child=frame,
        origin=Origin(xyz=(0, 0, 0)),
    )

    # 3. Front Wheel Assembly
    # Dual wheels pushed outboard for a wider stance, sharing one axle.
    left_wheel = cq.Workplane("XZ").cylinder(0.06, 0.2).translate((0, 0.1, 0))
    right_wheel = cq.Workplane("XZ").cylinder(0.06, 0.2).translate((0, -0.1, 0))
    axle = cq.Workplane("XZ").cylinder(0.30, 0.02)
    wheel_cq = left_wheel.union(right_wheel).union(axle)

    front_wheel = model.part("front_wheel")
    front_wheel.visual(mesh_from_cadquery(wheel_cq, "wheel_mesh"))

    # Mount front wheel to frame
    # Wheel center is at X=0.7. The frame visual is shifted by Z=-0.02.
    model.articulation(
        "wheel_joint",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel,
        origin=Origin(xyz=(0.7, 0, -0.02)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray = object_model.get_part("tray")
    frame = object_model.get_part("frame")
    front_wheel = object_model.get_part("front_wheel")

    # The frame is bolted flush to the flat bottom of the tray.
    ctx.allow_overlap(tray, frame, reason="Frame is bolted to the bottom of the tray")
    
    # The wheel axle is intentionally mounted into the frame handles.
    ctx.allow_overlap(front_wheel, frame, reason="Axle penetrates the frame handles for mounting")

    # Check basic relationships
    ctx.expect_gap(front_wheel, tray, axis="x", min_gap=0.05, name="Wheel is mounted in front of the tray")

    # Rotate the wheel to ensure it stays clear
    wheel_joint = object_model.get_articulation("wheel_joint")
    with ctx.pose({wheel_joint: 1.57}):
        ctx.expect_gap(front_wheel, tray, axis="x", min_gap=0.05, name="Wheel remains clear of tray when rotated")

    return ctx.report()


object_model = build_object_model()
