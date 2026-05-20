import cadquery as cq
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

def build_wheel():
    # Axle
    wheel = cq.Workplane("YZ").cylinder(1.02, 0.04)
    # Hub
    hub = cq.Workplane("YZ").cylinder(0.2, 0.12)
    wheel = wheel.union(hub)
    
    for i in range(8):
        angle = i * 45.0
        wp = cq.Workplane("YZ").transformed(rotate=cq.Vector(0, 0, angle))
        
        # Spoke
        spoke = wp.center(0, 0.26).box(0.04, 0.28, 0.04)
        wheel = wheel.union(spoke)
        
        # Paddle
        paddle = wp.center(0, 0.45).box(0.02, 0.2, 0.5)
        wheel = wheel.union(paddle)
        
    return wheel

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undershot_waterwheel")
    
    base = model.part("base")
    
    # Central support under the rotating stage
    base.visual(
        Box((0.4, 0.6, 0.175)),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        name="central_support",
        color=(0.5, 0.5, 0.5, 1.0)
    )
    
    # Trough bottom
    base.visual(
        Box((0.8, 1.6, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        name="trough_bottom",
        color=(0.4, 0.3, 0.2, 1.0)
    )
    
    # Trough side walls
    base.visual(
        Box((0.05, 1.6, 0.3)),
        origin=Origin(xyz=(-0.375, 0.0, 0.375)),
        name="trough_left_wall",
        color=(0.4, 0.3, 0.2, 1.0)
    )
    base.visual(
        Box((0.05, 1.6, 0.3)),
        origin=Origin(xyz=(0.375, 0.0, 0.375)),
        name="trough_right_wall",
        color=(0.4, 0.3, 0.2, 1.0)
    )
    
    # Side frames supporting the axle
    base.visual(
        Box((0.1, 0.2, 0.9)),
        origin=Origin(xyz=(-0.45, 0.0, 0.675)),
        name="frame_left",
        color=(0.3, 0.2, 0.1, 1.0)
    )
    base.visual(
        Box((0.1, 0.2, 0.9)),
        origin=Origin(xyz=(0.45, 0.0, 0.675)),
        name="frame_right",
        color=(0.3, 0.2, 0.1, 1.0)
    )
    
    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(build_wheel(), "waterwheel"),
        origin=Origin(xyz=(0, 0, 0)),
        name="wheel_geometry",
        color=(0.35, 0.25, 0.15, 1.0)
    )
    
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.8)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    wheel = object_model.get_part("wheel")
    
    # The axle of the wheel passes through the side frames
    ctx.allow_overlap(
        wheel, base,
        elem_a="wheel_geometry",
        elem_b="frame_left",
        reason="Axle passes through the left frame."
    )
    ctx.allow_overlap(
        wheel, base,
        elem_a="wheel_geometry",
        elem_b="frame_right",
        reason="Axle passes through the right frame."
    )
    
    ctx.expect_overlap(wheel, base, axes="x", elem_a="wheel_geometry", elem_b="frame_left", min_overlap=0.05)
    ctx.expect_overlap(wheel, base, axes="x", elem_a="wheel_geometry", elem_b="frame_right", min_overlap=0.05)
    
    # Wheel paddles should clear the trough bottom
    ctx.expect_gap(wheel, base, axis="z", positive_elem="wheel_geometry", negative_elem="trough_bottom", min_gap=0.01)
    
    # Wheel should be contained within the trough lengthwise
    ctx.expect_within(
        wheel, base,
        axes="y",
        inner_elem="wheel_geometry",
        outer_elem="trough_bottom",
        margin=0.0
    )
    
    return ctx.report()

object_model = build_object_model()