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
    model = ArticulatedObject(name="rack_chassis")

    # --- Chassis Base ---
    chassis = model.part("chassis")
    
    # Main body: 17" wide (0.432m), 0.498m deep, 4U tall (~0.178m total)
    chassis.visual(
        Box((0.432, 0.498, 0.005)),
        origin=Origin(xyz=(0.0, 0.001, 0.0025)),
        name="bottom_panel",
    )
    chassis.visual(
        Box((0.005, 0.498, 0.168)),
        origin=Origin(xyz=(-0.2135, 0.001, 0.089)),
        name="left_wall",
    )
    chassis.visual(
        Box((0.005, 0.498, 0.168)),
        origin=Origin(xyz=(0.2135, 0.001, 0.089)),
        name="right_wall",
    )
    chassis.visual(
        Box((0.422, 0.005, 0.168)),
        origin=Origin(xyz=(0.0, 0.2475, 0.089)),
        name="rear_wall",
    )
    
    # Rack ears: extending to 19" total width (0.483m)
    chassis.visual(
        Box((0.0255, 0.022, 0.178)),
        origin=Origin(xyz=(-0.22875, -0.259, 0.089)),
        name="left_ear",
    )
    chassis.visual(
        Box((0.0255, 0.022, 0.178)),
        origin=Origin(xyz=(0.22875, -0.259, 0.089)),
        name="right_ear",
    )

    # Top panel hinge mounts on chassis
    chassis.visual(
        Cylinder(radius=0.006, length=0.02),
        origin=Origin(xyz=(-0.180, 0.250, 0.178), rpy=(0.0, 1.5708, 0.0)),
        name="left_hinge_mount",
    )
    chassis.visual(
        Cylinder(radius=0.006, length=0.02),
        origin=Origin(xyz=(0.180, 0.250, 0.178), rpy=(0.0, 1.5708, 0.0)),
        name="right_hinge_mount",
    )

    # Front bezel hinge mount
    chassis.visual(
        Cylinder(radius=0.005, length=0.02),
        origin=Origin(xyz=(-0.216, -0.250, 0.089)),
        name="front_hinge_mount",
    )

    # --- Front Bezel ---
    front_bezel = model.part("front_bezel")
    # Bezel sits exactly between the ears, 0.432m wide
    front_bezel.visual(
        Box((0.432, 0.020, 0.178)),
        origin=Origin(xyz=(0.216, -0.010, 0.0)),
        name="bezel_panel",
    )
    front_bezel.visual(
        Cylinder(radius=0.004, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="front_hinge_pin",
    )

    model.articulation(
        "front_bezel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_bezel,
        origin=Origin(xyz=(-0.216, -0.250, 0.089)),
        axis=(0.0, 0.0, -1.0), # Swings outward to the left
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=2.0),
    )

    # --- Top Panel ---
    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((0.432, 0.498, 0.005)),
        origin=Origin(xyz=(0.0, -0.249, -0.0025)),
        name="panel_sheet",
    )
    top_panel.visual(
        Cylinder(radius=0.004, length=0.04),
        origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
        name="left_hinge_pin",
    )
    top_panel.visual(
        Cylinder(radius=0.004, length=0.04),
        origin=Origin(xyz=(0.180, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)),
        name="right_hinge_pin",
    )

    model.articulation(
        "top_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(0.0, 0.250, 0.178)),
        axis=(-1.0, 0.0, 0.0), # Swings up and back
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    front_bezel = object_model.get_part("front_bezel")
    top_panel = object_model.get_part("top_panel")

    # Allow overlap for the captured hinge pins
    ctx.allow_overlap(
        top_panel, chassis,
        elem_a="left_hinge_pin", elem_b="left_hinge_mount",
        reason="Hinge pin is captured inside the mount barrel."
    )
    ctx.allow_overlap(
        top_panel, chassis,
        elem_a="right_hinge_pin", elem_b="right_hinge_mount",
        reason="Hinge pin is captured inside the mount barrel."
    )
    ctx.allow_overlap(
        top_panel, chassis,
        elem_a="panel_sheet", elem_b="left_hinge_mount",
        reason="Hinge mount barrel slightly intersects the panel sheet."
    )
    ctx.allow_overlap(
        top_panel, chassis,
        elem_a="panel_sheet", elem_b="right_hinge_mount",
        reason="Hinge mount barrel slightly intersects the panel sheet."
    )
    ctx.allow_overlap(
        front_bezel, chassis,
        elem_a="front_hinge_pin", elem_b="front_hinge_mount",
        reason="Front bezel hinge pin is captured inside the mount barrel."
    )
    ctx.allow_overlap(
        front_bezel, chassis,
        elem_a="bezel_panel", elem_b="front_hinge_mount",
        reason="Hinge mount barrel slightly intersects the bezel panel."
    )

    # Exact gap checks at rest
    # Top panel sits perfectly on top of the chassis walls
    ctx.expect_gap(
        top_panel, chassis,
        axis="z",
        min_gap=0.0, max_gap=0.001,
        positive_elem="panel_sheet", negative_elem="left_wall",
        name="top panel rests on chassis walls"
    )

    # Front bezel sits just in front of the chassis walls
    ctx.expect_gap(
        chassis, front_bezel,
        axis="y",
        min_gap=0.001, max_gap=0.003,
        positive_elem="left_wall", negative_elem="bezel_panel",
        name="front bezel sits in front of chassis"
    )

    # Pose checks
    with ctx.pose(top_panel_hinge=1.0):
        # Top panel should lift up
        panel_aabb = ctx.part_world_aabb(top_panel)
        ctx.check(
            "top panel opens upward",
            panel_aabb is not None and panel_aabb[1][2] > 0.3,
            details=f"Top panel AABB: {panel_aabb}"
        )

    with ctx.pose(front_bezel_hinge=1.5):
        # Front bezel should swing forward and left
        bezel_aabb = ctx.part_world_aabb(front_bezel)
        ctx.check(
            "front bezel swings outward",
            bezel_aabb is not None and bezel_aabb[0][1] < -0.4,
            details=f"Front bezel AABB: {bezel_aabb}"
        )

    return ctx.report()


object_model = build_object_model()
