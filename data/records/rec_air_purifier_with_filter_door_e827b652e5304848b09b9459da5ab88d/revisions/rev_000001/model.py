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
    model = ArticulatedObject(name="air_purifier")

    chassis = model.part("chassis")
    
    # Back panel
    chassis.visual(
        Box((0.40, 0.02, 0.60)),
        origin=Origin(xyz=(0.0, -0.04, 0.0)),
        name="back_panel",
    )
    # Left wall
    chassis.visual(
        Box((0.02, 0.08, 0.60)),
        origin=Origin(xyz=(-0.19, 0.01, 0.0)),
        name="left_wall",
    )
    # Right wall
    chassis.visual(
        Box((0.02, 0.08, 0.60)),
        origin=Origin(xyz=(0.19, 0.01, 0.0)),
        name="right_wall",
    )
    # Top wall
    chassis.visual(
        Box((0.36, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.01, 0.29)),
        name="top_wall",
    )
    # Bottom wall
    chassis.visual(
        Box((0.36, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.01, -0.29)),
        name="bottom_wall",
    )
    # Left rail
    chassis.visual(
        Box((0.02, 0.08, 0.01)),
        origin=Origin(xyz=(-0.10, 0.01, -0.275)),
        name="left_rail",
    )
    # Right rail
    chassis.visual(
        Box((0.02, 0.08, 0.01)),
        origin=Origin(xyz=(0.10, 0.01, -0.275)),
        name="right_rail",
    )

    door = model.part("door")
    door.visual(
        Box((0.40, 0.02, 0.60)),
        origin=Origin(xyz=(0.20, 0.01, 0.0)),
        name="door_panel",
    )

    model.articulation(
        "chassis_to_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(-0.20, 0.051, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=2.0),
    )

    filter_cartridge = model.part("filter_cartridge")
    filter_cartridge.visual(
        Box((0.34, 0.058, 0.538)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="filter_body",
    )

    model.articulation(
        "chassis_to_filter",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=filter_cartridge,
        origin=Origin(xyz=(0.0, 0.02, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.04),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    door = object_model.get_part("door")
    filter_cartridge = object_model.get_part("filter_cartridge")
    
    door_joint = object_model.get_articulation("chassis_to_door")
    filter_joint = object_model.get_articulation("chassis_to_filter")

    ctx.allow_isolated_part(door, reason="Door is supported by a vertical hinge on the left edge.")
    ctx.allow_isolated_part(filter_cartridge, reason="Filter cartridge slides on the bottom rails with a realistic clearance gap.")

    # Closed pose
    ctx.expect_gap(filter_cartridge, chassis, axis="z", min_gap=0.001, max_gap=0.002, positive_elem="filter_body", negative_elem="left_rail")
    ctx.expect_gap(filter_cartridge, chassis, axis="z", min_gap=0.001, max_gap=0.002, positive_elem="filter_body", negative_elem="right_rail")
    ctx.expect_gap(door, filter_cartridge, axis="y", min_gap=0.001)

    # Open pose
    with ctx.pose({door_joint: 1.5}):
        ctx.expect_gap(door, filter_cartridge, axis="y", min_gap=0.001)
        
        # Extended pose
        with ctx.pose({door_joint: 1.5, filter_joint: 0.04}):
            ctx.expect_overlap(filter_cartridge, chassis, axes="y", min_overlap=0.01, elem_a="filter_body", elem_b="left_rail")
            ctx.expect_overlap(filter_cartridge, chassis, axes="y", min_overlap=0.01, elem_a="filter_body", elem_b="right_rail")

    return ctx.report()

object_model = build_object_model()