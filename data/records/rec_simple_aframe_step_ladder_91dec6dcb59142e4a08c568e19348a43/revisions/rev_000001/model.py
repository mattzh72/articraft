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
    model = ArticulatedObject(name="step_ladder")

    # Front frame (root)
    front = model.part("front_frame")
    
    # Front rails
    front.visual(
        Box((0.025, 0.08, 1.18)),
        origin=Origin(xyz=(-0.1875, 0.0, 0.59)),
        name="left_rail",
    )
    front.visual(
        Box((0.025, 0.08, 1.18)),
        origin=Origin(xyz=(0.1875, 0.0, 0.59)),
        name="right_rail",
    )
    
    # Treads
    for i, z_height in enumerate([0.3, 0.6, 0.9]):
        front.visual(
            Box((0.35, 0.08, 0.025)),
            origin=Origin(xyz=(0.0, 0.0, z_height)),
            name=f"tread_{i+1}",
        )
        
    # Top cap
    front.visual(
        Box((0.45, 0.16, 0.04)),
        origin=Origin(xyz=(0.0, 0.03, 1.20)),
        name="top_cap_plate",
    )
    front.visual(
        Box((0.025, 0.16, 0.10)),
        origin=Origin(xyz=(-0.2125, 0.03, 1.13)),
        name="top_cap_left_skirt",
    )
    front.visual(
        Box((0.025, 0.16, 0.10)),
        origin=Origin(xyz=(0.2125, 0.03, 1.13)),
        name="top_cap_right_skirt",
    )
    
    # Hinge pin
    front.visual(
        Cylinder(radius=0.006, length=0.45),
        origin=Origin(xyz=(0.0, 0.08, 1.16), rpy=(0.0, 1.5708, 0.0)),
        name="hinge_pin",
    )

    # Rear frame
    rear = model.part("rear_frame")
    
    # Rear supports
    rear.visual(
        Box((0.025, 0.03, 1.16)),
        origin=Origin(xyz=(-0.1875, -0.015, -0.58)),
        name="left_support",
    )
    rear.visual(
        Box((0.025, 0.03, 1.16)),
        origin=Origin(xyz=(0.1875, -0.015, -0.58)),
        name="right_support",
    )
    
    # Rear cross braces
    rear.visual(
        Box((0.35, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, -0.015, -0.40)),
        name="cross_brace_1",
    )
    rear.visual(
        Box((0.35, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, -0.015, -0.80)),
        name="cross_brace_2",
    )

    # Articulation
    model.articulation(
        "front_to_rear",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.08, 1.16)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=0.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hinge = object_model.get_articulation("front_to_rear")
    
    ctx.allow_overlap(
        front,
        rear,
        elem_a="hinge_pin",
        elem_b="left_support",
        reason="The hinge pin passes through the left support of the rear frame.",
    )
    ctx.allow_overlap(
        front,
        rear,
        elem_a="hinge_pin",
        elem_b="right_support",
        reason="The hinge pin passes through the right support of the rear frame.",
    )

    # In folded state, rear frame is behind front frame rails
    ctx.expect_gap(rear, front, axis="y", min_gap=0.005, negative_elem="left_rail", name="rear frame is behind front frame when folded")
    
    # In deployed state, rear frame swings out further back
    with ctx.pose({hinge: 0.5}):
        ctx.expect_gap(rear, front, axis="y", min_gap=0.3, positive_elem="cross_brace_2", negative_elem="left_rail", name="rear frame swings out to deploy")

    return ctx.report()


object_model = build_object_model()