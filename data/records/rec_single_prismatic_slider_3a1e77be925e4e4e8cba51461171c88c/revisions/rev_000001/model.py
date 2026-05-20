from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_guided_carriage")

    bridge = model.part("bridge_frame")
    
    # Beam composed of 3 boxes to form a C-channel open at the bottom
    bridge.visual(
        Box((1.2, 0.2, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        name="beam_top",
    )
    bridge.visual(
        Box((1.2, 0.05, 0.15)),
        origin=Origin(xyz=(0.0, -0.075, 0.475)),
        name="beam_left",
    )
    bridge.visual(
        Box((1.2, 0.05, 0.15)),
        origin=Origin(xyz=(0.0, 0.075, 0.475)),
        name="beam_right",
    )
    
    # Legs supporting the beam
    bridge.visual(
        Box((0.1, 0.2, 0.4)),
        origin=Origin(xyz=(-0.55, 0.0, 0.2)),
        name="left_leg",
    )
    bridge.visual(
        Box((0.1, 0.2, 0.4)),
        origin=Origin(xyz=(0.55, 0.0, 0.2)),
        name="right_leg",
    )
    
    carriage = model.part("carriage")
    # Slider block fits inside the channel
    carriage.visual(
        Box((0.2, 0.1, 0.15)),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        name="slider_block",
    )
    # Neck passes through the bottom opening
    carriage.visual(
        Box((0.2, 0.04, 0.1)),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        name="neck",
    )
    # Payload attachment plate
    carriage.visual(
        Box((0.2, 0.15, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        name="plate",
    )
    
    # Prismatic joint for the carriage sliding along the beam
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-0.4, upper=0.4),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    bridge = object_model.get_part("bridge_frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("carriage_slide")
    
    # Check that the slider block is contained within the bridge bounds
    ctx.expect_within(
        carriage,
        bridge,
        axes="yz",
        inner_elem="slider_block",
        margin=0.01,
        name="slider fits inside beam channel"
    )
    
    # Verify behavior at limits
    with ctx.pose({slide: 0.4}):
        ctx.expect_within(
            carriage,
            bridge,
            axes="yz",
            inner_elem="slider_block",
            margin=0.01,
            name="slider fits inside beam channel at max extension"
        )
        # Carriage should approach but not penetrate the leg
        ctx.expect_gap(
            bridge,
            carriage,
            axis="x",
            positive_elem="right_leg",
            negative_elem="slider_block",
            min_gap=0.0,
            name="carriage stops before right leg"
        )
        
    with ctx.pose({slide: -0.4}):
        ctx.expect_gap(
            carriage,
            bridge,
            axis="x",
            positive_elem="slider_block",
            negative_elem="left_leg",
            min_gap=0.0,
            name="carriage stops before left leg"
        )
    
    return ctx.report()

object_model = build_object_model()
