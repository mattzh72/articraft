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
    model = ArticulatedObject(name="knee_scooter")

    # --- Frame ---
    frame = model.part("frame")
    # Main longitudinal tube (stops exactly at the head tube to avoid overlapping the steering stem)
    frame.visual(Box((0.57, 0.05, 0.05)), origin=Origin(xyz=(0.285, 0.0, 0.1)), name="main_tube")
    # Rear axle support
    frame.visual(Box((0.05, 0.38, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.1)), name="rear_axle_tube")
    # Head tube for steering
    frame.visual(Cylinder(radius=0.03, length=0.21), origin=Origin(xyz=(0.6, 0.0, 0.23)), name="head_tube")
    # Post supporting the knee pad
    frame.visual(Cylinder(radius=0.025, length=0.4), origin=Origin(xyz=(0.2, 0.0, 0.3)), name="pad_post")
    # Knee pad base plate
    frame.visual(Box((0.3, 0.15, 0.02)), origin=Origin(xyz=(0.2, 0.0, 0.51)), name="pad_base")
    # Knee pad cushion
    frame.visual(Box((0.35, 0.18, 0.06)), origin=Origin(xyz=(0.2, 0.0, 0.55)), name="pad_cushion")

    # --- Steering Column ---
    steering = model.part("steering")
    steering_origin = Origin(xyz=(0.6, 0.0, 0.2))

    # Stem passes through the head tube
    steering.visual(Cylinder(radius=0.02, length=0.9), origin=Origin(xyz=(0.0, 0.0, 0.35)), name="stem")
    # Front fork crossbar
    steering.visual(Box((0.04, 0.38, 0.04)), origin=Origin(xyz=(0.0, 0.0, -0.1)), name="fork_crossbar")
    # Handlebar
    steering.visual(
        Cylinder(radius=0.015, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 0.78), rpy=(math.pi / 2, 0.0, 0.0)),
        name="handlebar"
    )
    # Grips
    steering.visual(
        Cylinder(radius=0.02, length=0.1),
        origin=Origin(xyz=(0.0, 0.225, 0.78), rpy=(math.pi / 2, 0.0, 0.0)),
        name="left_grip"
    )
    steering.visual(
        Cylinder(radius=0.02, length=0.1),
        origin=Origin(xyz=(0.0, -0.225, 0.78), rpy=(math.pi / 2, 0.0, 0.0)),
        name="right_grip"
    )

    model.articulation(
        "steering_joint",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering,
        origin=steering_origin,
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.0, upper=1.0)
    )

    # --- Wheels ---
    def add_wheel(name, parent, origin_xyz):
        wheel = model.part(name)
        wheel.visual(
            Cylinder(radius=0.1, length=0.04),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            name="tire"
        )
        wheel.visual(
            Cylinder(radius=0.04, length=0.04),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            name="hub"
        )
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel,
            origin=Origin(xyz=origin_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=10.0),
        )
        return wheel

    # Rear wheels attached to frame
    add_wheel("rear_left_wheel", frame, (0.0, 0.21, 0.1))
    add_wheel("rear_right_wheel", frame, (0.0, -0.21, 0.1))

    # Front wheels attached to steering
    add_wheel("front_left_wheel", steering, (0.0, 0.21, -0.1))
    add_wheel("front_right_wheel", steering, (0.0, -0.21, -0.1))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    steering = object_model.get_part("steering")

    # Allow overlap for the steering stem inside the head tube
    ctx.allow_overlap(
        frame,
        steering,
        elem_a="head_tube",
        elem_b="stem",
        reason="The steering stem intentionally rotates inside the frame's head tube."
    )

    # Verify the stem is actually inside the head tube
    ctx.expect_within(
        steering,
        frame,
        axes="xy",
        inner_elem="stem",
        outer_elem="head_tube",
        name="Stem is centered inside the head tube"
    )

    # Verify wheels are mounted correctly (in contact with their respective axle tubes)
    ctx.expect_contact(object_model.get_part("rear_left_wheel"), frame, elem_b="rear_axle_tube")
    ctx.expect_contact(object_model.get_part("rear_right_wheel"), frame, elem_b="rear_axle_tube")
    ctx.expect_contact(object_model.get_part("front_left_wheel"), steering, elem_b="fork_crossbar")
    ctx.expect_contact(object_model.get_part("front_right_wheel"), steering, elem_b="fork_crossbar")

    # Verify steering pose works
    fl_wheel = object_model.get_part("front_left_wheel")
    rest_pos = ctx.part_world_position(fl_wheel)

    with ctx.pose({"steering_joint": 1.0}):
        turned_pos = ctx.part_world_position(fl_wheel)

    if rest_pos and turned_pos:
        # At rest, front left wheel is at y=0.21, x=0.6
        # Turned by 1.0 rad around (0.6, 0.0, 0.2)
        # Its x should decrease (move backwards), y should decrease (move inwards)
        ctx.check(
            "Steering moves front wheel",
            turned_pos[0] < rest_pos[0] and turned_pos[1] < rest_pos[1],
            details=f"rest={rest_pos}, turned={turned_pos}"
        )

    return ctx.report()


object_model = build_object_model()
