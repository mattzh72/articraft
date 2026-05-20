import math
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
    model = ArticulatedObject(name="swing_set")

    frame = model.part("frame")
    
    # Crossbeam
    frame.visual(
        Cylinder(radius=0.05, length=2.4),
        origin=Origin(xyz=(0.0, 0.0, 2.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="crossbeam",
    )
    
    # A-frame legs
    leg_length = math.hypot(0.75, 2.0)
    theta = math.atan2(0.75, 2.0)
    
    # Left A-frame
    frame.visual(
        Cylinder(radius=0.04, length=leg_length),
        origin=Origin(xyz=(0.375, -1.0, 1.0), rpy=(0.0, -theta, 0.0)),
        name="left_front_leg",
    )
    frame.visual(
        Cylinder(radius=0.04, length=leg_length),
        origin=Origin(xyz=(-0.375, -1.0, 1.0), rpy=(0.0, theta, 0.0)),
        name="left_back_leg",
    )
    frame.visual(
        Cylinder(radius=0.03, length=0.75),
        origin=Origin(xyz=(0.0, -1.0, 1.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="left_brace",
    )
    
    # Right A-frame
    frame.visual(
        Cylinder(radius=0.04, length=leg_length),
        origin=Origin(xyz=(0.375, 1.0, 1.0), rpy=(0.0, -theta, 0.0)),
        name="right_front_leg",
    )
    frame.visual(
        Cylinder(radius=0.04, length=leg_length),
        origin=Origin(xyz=(-0.375, 1.0, 1.0), rpy=(0.0, theta, 0.0)),
        name="right_back_leg",
    )
    frame.visual(
        Cylinder(radius=0.03, length=0.75),
        origin=Origin(xyz=(0.0, 1.0, 1.0), rpy=(0.0, math.pi / 2, 0.0)),
        name="right_brace",
    )

    # Seat assembly
    # Modeled as a single part to represent the rigid pendulum motion of the seat and both hangers.
    seat_assembly = model.part("seat_assembly")
    
    # Hangers
    seat_assembly.visual(
        Cylinder(radius=0.01, length=1.44),
        origin=Origin(xyz=(0.0, -0.25, -0.78)),
        name="left_hanger",
    )
    seat_assembly.visual(
        Cylinder(radius=0.01, length=1.44),
        origin=Origin(xyz=(0.0, 0.25, -0.78)),
        name="right_hanger",
    )
    
    # Pivot rings
    seat_assembly.visual(
        Cylinder(radius=0.06, length=0.02),
        origin=Origin(xyz=(0.0, -0.25, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="left_pivot_ring",
    )
    seat_assembly.visual(
        Cylinder(radius=0.06, length=0.02),
        origin=Origin(xyz=(0.0, 0.25, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        name="right_pivot_ring",
    )
    
    # Seat
    seat_assembly.visual(
        Box((0.2, 0.6, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -1.5)),
        name="seat",
    )

    # Articulation
    model.articulation(
        "swing_joint",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_assembly,
        origin=Origin(xyz=(0.0, 0.0, 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=5.0, lower=-1.0, upper=1.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    seat_assembly = object_model.get_part("seat_assembly")
    swing_joint = object_model.get_articulation("swing_joint")
    
    # The pivot rings intentionally wrap around the crossbeam
    ctx.allow_overlap(
        seat_assembly,
        frame,
        elem_a="left_pivot_ring",
        elem_b="crossbeam",
        reason="The pivot ring is captured around the crossbeam.",
    )
    ctx.allow_overlap(
        seat_assembly,
        frame,
        elem_a="right_pivot_ring",
        elem_b="crossbeam",
        reason="The pivot ring is captured around the crossbeam.",
    )
    
    # Rest pose checks
    ctx.expect_overlap(seat_assembly, frame, axes="y", elem_a="left_pivot_ring", elem_b="crossbeam")
    ctx.expect_overlap(seat_assembly, frame, axes="y", elem_a="right_pivot_ring", elem_b="crossbeam")
    
    # The seat should be centered between the legs
    ctx.expect_within(seat_assembly, frame, axes="y", inner_elem="seat")
    
    # Check pendulum motion using the seat element AABB since the part origin is at the pivot
    rest_aabb = ctx.part_element_world_aabb(seat_assembly, elem="seat")
    with ctx.pose({swing_joint: 1.0}):
        swung_aabb = ctx.part_element_world_aabb(seat_assembly, elem="seat")
        
        if rest_aabb is not None and swung_aabb is not None:
            rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) / 2
            rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2
            swung_center_x = (swung_aabb[0][0] + swung_aabb[1][0]) / 2
            swung_center_z = (swung_aabb[0][2] + swung_aabb[1][2]) / 2
            
            ctx.check(
                "seat swings backward and upward",
                swung_center_x < rest_center_x - 0.5 and swung_center_z > rest_center_z + 0.2,
                details=f"rest=({rest_center_x}, {rest_center_z}), swung=({swung_center_x}, {swung_center_z})",
            )
        else:
            ctx.fail("seat swings upward and forward", "Could not get seat AABB")
        
    return ctx.report()

object_model = build_object_model()
