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
    model = ArticulatedObject(name="hospital_stretcher_elevator")

    shaft = model.part("shaft")
    shaft.visual(Box((3.4, 2.0, 0.1)), origin=Origin(xyz=(0.0, 0.0, -0.05)), name="base")
    shaft.visual(Box((3.4, 0.2, 0.1)), origin=Origin(xyz=(0.0, 0.0, 8.05)), name="top_beam")
    shaft.visual(Box((0.05, 0.1, 8.0)), origin=Origin(xyz=(-1.575, 0.0, 4.0)), name="left_rail")
    shaft.visual(Box((0.05, 0.1, 8.0)), origin=Origin(xyz=(1.575, 0.0, 4.0)), name="right_rail")

    car = model.part("car")
    car.visual(Box((3.0, 1.6, 0.1)), origin=Origin(xyz=(0.0, 0.0, -1.15)), name="floor")
    car.visual(Box((3.0, 1.6, 0.1)), origin=Origin(xyz=(0.0, 0.0, 1.15)), name="ceiling")
    car.visual(Box((3.0, 0.1, 2.2)), origin=Origin(xyz=(0.0, 0.75, 0.0)), name="back_wall")
    car.visual(Box((0.1, 1.4, 2.2)), origin=Origin(xyz=(-1.45, 0.0, 0.0)), name="left_wall")
    car.visual(Box((0.1, 1.4, 2.2)), origin=Origin(xyz=(1.45, 0.0, 0.0)), name="right_wall")
    car.visual(Box((0.85, 0.1, 2.2)), origin=Origin(xyz=(-1.075, -0.75, 0.0)), name="front_left_wall")
    car.visual(Box((0.85, 0.1, 2.2)), origin=Origin(xyz=(1.075, -0.75, 0.0)), name="front_right_wall")
    car.visual(Box((2.8, 0.1, 0.05)), origin=Origin(xyz=(0.0, -0.65, 1.075)), name="door_track")
    car.visual(Box((0.05, 0.1, 0.4)), origin=Origin(xyz=(-1.525, 0.0, 0.0)), name="left_mount")
    car.visual(Box((0.05, 0.1, 0.4)), origin=Origin(xyz=(1.525, 0.0, 0.0)), name="right_mount")
    
    # Handrails
    car.visual(Box((2.8, 0.06, 0.06)), origin=Origin(xyz=(0.0, 0.67, 0.0)), name="back_handrail")
    car.visual(Box((0.06, 1.2, 0.06)), origin=Origin(xyz=(-1.37, 0.0, 0.0)), name="left_handrail")
    car.visual(Box((0.06, 1.2, 0.06)), origin=Origin(xyz=(1.37, 0.0, 0.0)), name="right_handrail")
    
    # Control panel
    car.visual(Box((0.05, 0.3, 0.6)), origin=Origin(xyz=(1.375, 0.0, 0.4)), name="control_panel")

    model.articulation(
        "car_lift",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 1.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=5.5),
    )

    door_left = model.part("door_left")
    door_left.visual(Box((0.7, 0.05, 2.15)), origin=Origin(xyz=(0.0, 0.0, 0.0)), name="panel")
    
    model.articulation(
        "door_left_slide",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_left,
        origin=Origin(xyz=(-0.35, -0.65, -0.025)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.7),
    )

    door_right = model.part("door_right")
    door_right.visual(Box((0.7, 0.05, 2.15)), origin=Origin(xyz=(0.0, 0.0, 0.0)), name="panel")

    model.articulation(
        "door_right_slide",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_right,
        origin=Origin(xyz=(0.35, -0.65, -0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.7),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    door_left = object_model.get_part("door_left")
    door_right = object_model.get_part("door_right")

    ctx.expect_contact(car, shaft, elem_a="left_mount", elem_b="left_rail")
    ctx.expect_contact(car, shaft, elem_a="right_mount", elem_b="right_rail")
    
    ctx.expect_contact(door_left, car, elem_a="panel", elem_b="door_track")
    ctx.expect_contact(door_right, car, elem_a="panel", elem_b="door_track")
    
    ctx.expect_contact(door_left, car, elem_a="panel", elem_b="floor")
    ctx.expect_contact(door_right, car, elem_a="panel", elem_b="floor")
    
    ctx.expect_contact(door_left, door_right, elem_a="panel", elem_b="panel")
    
    door_l_joint = object_model.get_articulation("door_left_slide")
    door_r_joint = object_model.get_articulation("door_right_slide")
    
    with ctx.pose({door_l_joint: 0.7, door_r_joint: 0.7}):
        ctx.expect_contact(door_left, car, elem_a="panel", elem_b="left_wall")
        ctx.expect_contact(door_right, car, elem_a="panel", elem_b="right_wall")
        ctx.expect_gap(door_right, door_left, axis="x", min_gap=1.39, max_gap=1.41)
        
    car_lift = object_model.get_articulation("car_lift")
    with ctx.pose({car_lift: 5.5}):
        ctx.expect_gap(shaft, car, axis="z", positive_elem="top_beam", negative_elem="ceiling", min_gap=0.04, max_gap=0.06)

    return ctx.report()

object_model = build_object_model()