from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roll_axis_rotary_module")

    frame = model.part("frame")
    
    # Material colors
    frame_color = (0.2, 0.2, 0.2, 1.0)
    bearing_color = (0.7, 0.7, 0.7, 1.0)
    shaft_color = (0.8, 0.8, 0.8, 1.0)
    member_color = (0.1, 0.4, 0.8, 1.0)
    
    # Rectangular outer frame
    # Base plate
    frame.visual(
        Box((0.24, 0.12, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        name="base_plate",
        color=frame_color,
    )
    
    # End plate 1 (+X)
    frame.visual(
        Box((0.02, 0.12, 0.12)),
        origin=Origin(xyz=(0.11, 0.0, -0.01)),
        name="end_plate_1",
        color=frame_color,
    )
    
    # End plate 2 (-X)
    frame.visual(
        Box((0.02, 0.12, 0.12)),
        origin=Origin(xyz=(-0.11, 0.0, -0.01)),
        name="end_plate_2",
        color=frame_color,
    )
    
    # Side rails to make it a rectangular outer frame
    frame.visual(
        Box((0.20, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.05, 0.04)),
        name="side_rail_1",
        color=frame_color,
    )
    frame.visual(
        Box((0.20, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, -0.05, 0.04)),
        name="side_rail_2",
        color=frame_color,
    )

    # Bearing cartridges
    # Compact bearing cartridge 1
    frame.visual(
        Cylinder(radius=0.025, length=0.03),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="bearing_cartridge_1",
        color=bearing_color,
    )
    
    # Compact bearing cartridge 2
    frame.visual(
        Cylinder(radius=0.025, length=0.03),
        origin=Origin(xyz=(-0.085, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="bearing_cartridge_2",
        color=bearing_color,
    )

    shaft = model.part("shaft")
    
    # Central cylindrical shaft
    shaft.visual(
        Cylinder(radius=0.015, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="central_shaft",
        color=shaft_color,
    )
    
    # Rotating member in the center
    shaft.visual(
        Cylinder(radius=0.04, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        name="rotating_member",
        color=member_color,
    )

    # Roll-axis joint
    model.articulation(
        "frame_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    # Allow overlap because the shaft is intentionally nested inside the bearing cartridges and end plates
    ctx.allow_overlap(
        "shaft",
        "frame",
        elem_a="central_shaft",
        elem_b="bearing_cartridge_1",
        reason="Shaft passes through bearing cartridge 1",
    )
    ctx.allow_overlap(
        "shaft",
        "frame",
        elem_a="central_shaft",
        elem_b="bearing_cartridge_2",
        reason="Shaft passes through bearing cartridge 2",
    )
    ctx.allow_overlap(
        "shaft",
        "frame",
        elem_a="central_shaft",
        elem_b="end_plate_1",
        reason="Shaft passes through end plate 1",
    )
    ctx.allow_overlap(
        "shaft",
        "frame",
        elem_a="central_shaft",
        elem_b="end_plate_2",
        reason="Shaft passes through end plate 2",
    )

    ctx.expect_within(
        "shaft",
        "frame",
        axes="yz",
        inner_elem="central_shaft",
        outer_elem="bearing_cartridge_1",
        margin=0.0,
        name="shaft is centered in bearing 1",
    )
    ctx.expect_within(
        "shaft",
        "frame",
        axes="yz",
        inner_elem="central_shaft",
        outer_elem="bearing_cartridge_2",
        margin=0.0,
        name="shaft is centered in bearing 2",
    )

    return ctx.report()

object_model = build_object_model()