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
import math

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_shutter")
    
    frame = model.part("frame")
    
    # Stiles
    frame.visual(Box((0.04, 0.04, 0.8)), origin=Origin(xyz=(-0.18, 0.0, 0.4)), name="left_stile")
    frame.visual(Box((0.04, 0.04, 0.8)), origin=Origin(xyz=(0.18, 0.0, 0.4)), name="right_stile")
    
    # Rails
    frame.visual(Box((0.32, 0.04, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.78)), name="top_rail")
    frame.visual(Box((0.32, 0.04, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.02)), name="bottom_rail")
    
    num_slats = 12
    z_start = 0.07
    z_step = 0.06
    
    for i in range(num_slats):
        slat = model.part(f"slat_{i}")
        z_pos = z_start + i * z_step
        
        slat.visual(Box((0.316, 0.055, 0.005)), name="blade")
        
        # Pins
        slat.visual(
            Cylinder(radius=0.003, length=0.01),
            origin=Origin(xyz=(-0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            name="left_pin"
        )
        slat.visual(
            Cylinder(radius=0.003, length=0.01),
            origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
            name="right_pin"
        )
        
        model.articulation(
            f"slat_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.0, upper=1.0),
        )
        
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    
    for i in range(12):
        slat = object_model.get_part(f"slat_{i}")
        
        ctx.allow_overlap(
            frame, slat,
            elem_a="left_stile", elem_b="left_pin",
            reason="Pin is captured in the stile."
        )
        ctx.allow_overlap(
            frame, slat,
            elem_a="right_stile", elem_b="right_pin",
            reason="Pin is captured in the stile."
        )
        
        ctx.expect_within(
            slat, frame,
            axes="z",
            inner_elem="blade",
            name=f"slat_{i} blade stays vertically within frame"
        )
        
        ctx.expect_overlap(
            slat, frame,
            axes="x",
            elem_a="left_pin", elem_b="left_stile",
            min_overlap=0.002,
            name=f"slat_{i} left pin is inserted into stile"
        )
        ctx.expect_overlap(
            slat, frame,
            axes="x",
            elem_a="right_pin", elem_b="right_stile",
            min_overlap=0.002,
            name=f"slat_{i} right pin is inserted into stile"
        )
        
    return ctx.report()

object_model = build_object_model()
