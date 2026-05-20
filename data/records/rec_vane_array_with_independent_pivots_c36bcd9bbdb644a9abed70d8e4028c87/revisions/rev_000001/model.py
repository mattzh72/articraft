from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vane_array")
    
    frame = model.part("frame")
    
    # Left rail
    frame.visual(
        Box((0.05, 0.8, 0.1)),
        origin=Origin(xyz=(-0.275, 0.0, 0.0)),
        name="left_rail",
    )
    # Right rail
    frame.visual(
        Box((0.05, 0.8, 0.1)),
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        name="right_rail",
    )
    # Top rail
    frame.visual(
        Box((0.6, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.375, -0.025)),
        name="top_rail",
    )
    # Bottom rail
    frame.visual(
        Box((0.6, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, -0.375, -0.025)),
        name="bottom_rail",
    )
    
    num_blades = 5
    blade_pitch = 0.14
    blade_y_start = 0.28
    
    for i in range(num_blades):
        blade = model.part(f"blade_{i}")
        blade_y = blade_y_start - i * blade_pitch
        
        blade.visual(
            Box((0.49, 0.135, 0.01)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            name=f"blade_shell_{i}",
        )
        
        blade.visual(
            Box((0.52, 0.02, 0.02)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            name=f"blade_pins_{i}",
        )
        
        model.articulation(
            f"blade_joint_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(0.0, blade_y, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.57, upper=1.57),
        )
        
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    frame = object_model.get_part("frame")
    
    for i in range(5):
        blade = object_model.get_part(f"blade_{i}")
        
        ctx.allow_overlap(
            frame,
            blade,
            elem_a="left_rail",
            elem_b=f"blade_pins_{i}",
            reason="Pins are captured in the left rail."
        )
        ctx.allow_overlap(
            frame,
            blade,
            elem_a="right_rail",
            elem_b=f"blade_pins_{i}",
            reason="Pins are captured in the right rail."
        )
        
        ctx.expect_within(
            blade,
            frame,
            axes="x",
            inner_elem=f"blade_shell_{i}",
            name=f"blade {i} fits horizontally within frame"
        )
        
        ctx.expect_overlap(
            blade,
            frame,
            axes="x",
            elem_a=f"blade_pins_{i}",
            elem_b="left_rail",
            min_overlap=0.005,
            name=f"blade {i} pins captured in left rail"
        )
        ctx.expect_overlap(
            blade,
            frame,
            axes="x",
            elem_a=f"blade_pins_{i}",
            elem_b="right_rail",
            min_overlap=0.005,
            name=f"blade {i} pins captured in right rail"
        )

    return ctx.report()

object_model = build_object_model()