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
    model = ArticulatedObject(name="wall_cabinet")
    
    cabinet = model.part("cabinet")
    
    # Top panel
    cabinet.visual(
        Box((0.60, 0.30, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.79)),
        name="top_panel",
    )
    
    # Bottom panel
    cabinet.visual(
        Box((0.60, 0.30, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="bottom_panel",
    )
    
    # Left panel
    cabinet.visual(
        Box((0.02, 0.30, 0.76)),
        origin=Origin(xyz=(-0.29, 0.0, 0.40)),
        name="left_panel",
    )
    
    # Right panel
    cabinet.visual(
        Box((0.02, 0.30, 0.76)),
        origin=Origin(xyz=(0.29, 0.0, 0.40)),
        name="right_panel",
    )
    
    # Back panel
    cabinet.visual(
        Box((0.56, 0.02, 0.76)),
        origin=Origin(xyz=(0.0, 0.14, 0.40)),
        name="back_panel",
    )
    
    # Central shelf
    cabinet.visual(
        Box((0.56, 0.28, 0.02)),
        origin=Origin(xyz=(0.0, -0.01, 0.40)),
        name="shelf",
    )
    
    # Left door
    left_door = model.part("left_door")
    left_door.visual(
        Box((0.298, 0.02, 0.80)),
        # The hinge is at x=-0.30, y=-0.15. The door extends +0.149 in X, -0.01 in Y
        origin=Origin(xyz=(0.149, -0.01, 0.0)),
        name="left_door_panel",
    )
    
    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(-0.30, -0.15, 0.40)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.92),
    )
    
    # Right door
    right_door = model.part("right_door")
    right_door.visual(
        Box((0.298, 0.02, 0.80)),
        # The hinge is at x=0.30, y=-0.15. The door extends -0.149 in X, -0.01 in Y
        origin=Origin(xyz=(-0.149, -0.01, 0.0)),
        name="right_door_panel",
    )
    
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(0.30, -0.15, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.92),
    )
    
    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    
    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    
    # Closed state checks
    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_gap(cabinet, left_door, axis="y", max_penetration=0.001)
        ctx.expect_gap(cabinet, right_door, axis="y", max_penetration=0.001)
        # Check gap between doors
        ctx.expect_gap(right_door, left_door, axis="x", max_penetration=0.0)
        
    # Open state checks
    with ctx.pose({left_hinge: 1.92, right_hinge: 1.92}):
        ctx.expect_gap(cabinet, left_door, axis="x", max_penetration=0.0)
        ctx.expect_gap(right_door, cabinet, axis="x", max_penetration=0.0)
        
    return ctx.report()

object_model = build_object_model()
