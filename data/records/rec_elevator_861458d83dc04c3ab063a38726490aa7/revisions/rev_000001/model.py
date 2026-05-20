from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traction_passenger_elevator")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.62, 0.64, 1.0))
    brushed = model.material("brushed_stainless", rgba=(0.78, 0.80, 0.80, 1.0))
    dark = model.material("dark_rubber", rgba=(0.04, 0.045, 0.05, 1.0))
    glass = model.material("smoked_glass", rgba=(0.08, 0.13, 0.16, 0.72))
    shaft_wall = model.material("pale_shaft_wall", rgba=(0.72, 0.76, 0.78, 0.42))
    rail = model.material("machined_rail", rgba=(0.42, 0.44, 0.45, 1.0))
    cable = model.material("black_hoist_cable", rgba=(0.01, 0.01, 0.012, 1.0))
    yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))

    # Root link: an open-front two-stop shaft with visible rails, machine beam,
    # traction sheave, and static hoist-rope runs.  The walls are thin and
    # translucent enough that the articulated car remains readable inside.
    shaft = model.part("shaft")
    shaft.visual(Box((3.35, 3.15, 0.16)), origin=Origin(xyz=(0.0, 0.0, 0.08)), material=shaft_wall, name="pit_floor")
    shaft.visual(Box((3.35, 0.12, 8.0)), origin=Origin(xyz=(0.0, 1.52, 4.0)), material=shaft_wall, name="rear_wall")
    shaft.visual(Box((0.12, 3.15, 8.0)), origin=Origin(xyz=(-1.675, 0.0, 4.0)), material=shaft_wall, name="side_wall_0")
    shaft.visual(Box((0.12, 3.15, 8.0)), origin=Origin(xyz=(1.675, 0.0, 4.0)), material=shaft_wall, name="side_wall_1")
    shaft.visual(Box((3.35, 3.15, 0.18)), origin=Origin(xyz=(0.0, 0.0, 8.09)), material=shaft_wall, name="overhead_slab")

    # Front landing portal and sill make the open shaft entrance explicit.
    shaft.visual(Box((0.14, 0.14, 2.35)), origin=Origin(xyz=(-1.12, -1.52, 1.25)), material=galvanized, name="landing_jamb_0")
    shaft.visual(Box((0.14, 0.14, 2.35)), origin=Origin(xyz=(1.12, -1.52, 1.25)), material=galvanized, name="landing_jamb_1")
    shaft.visual(Box((2.38, 0.14, 0.18)), origin=Origin(xyz=(0.0, -1.52, 2.38)), material=galvanized, name="landing_header")
    shaft.visual(Box((2.38, 0.18, 0.08)), origin=Origin(xyz=(0.0, -1.50, 0.20)), material=dark, name="landing_sill")

    # Elevator guide rails, placed clear of the car envelope and tied into the pit.
    shaft.visual(Box((0.07, 0.09, 7.7)), origin=Origin(xyz=(-1.22, 0.35, 3.93)), material=rail, name="guide_rail_0")
    shaft.visual(Box((0.07, 0.09, 7.7)), origin=Origin(xyz=(1.22, 0.35, 3.93)), material=rail, name="guide_rail_1")
    shaft.visual(Box((0.12, 0.10, 7.4)), origin=Origin(xyz=(1.48, 0.45, 3.86)), material=rail, name="counter_rail_0")
    shaft.visual(Box((0.12, 0.10, 7.4)), origin=Origin(xyz=(1.48, 1.45, 3.86)), material=rail, name="counter_rail_1")

    # Traction-machine mount, sheave, axle, and rope runs.
    shaft.visual(Box((3.00, 0.26, 0.22)), origin=Origin(xyz=(0.28, 1.45, 7.58)), material=galvanized, name="machine_beam")
    shaft.visual(
        Cylinder(radius=0.32, length=0.10),
        origin=Origin(xyz=(-0.38, 1.30, 7.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail,
        name="traction_sheave",
    )
    shaft.visual(
        Cylinder(radius=0.055, length=0.62),
        origin=Origin(xyz=(-0.38, 1.38, 7.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="sheave_axle",
    )
    shaft.visual(
        Cylinder(radius=0.22, length=0.08),
        origin=Origin(xyz=(1.34, 1.39, 7.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rail,
        name="deflector_sheave",
    )
    shaft.visual(Cylinder(radius=0.014, length=5.35), origin=Origin(xyz=(-0.64, 1.30, 4.90)), material=cable, name="car_rope_0")
    shaft.visual(Cylinder(radius=0.014, length=5.35), origin=Origin(xyz=(-0.50, 1.30, 4.90)), material=cable, name="car_rope_1")
    shaft.visual(Cylinder(radius=0.014, length=4.85), origin=Origin(xyz=(1.29, 1.44, 5.10)), material=cable, name="counter_rope_0")
    shaft.visual(Cylinder(radius=0.014, length=4.85), origin=Origin(xyz=(1.39, 1.44, 5.10)), material=cable, name="counter_rope_1")

    # Moving passenger car.  The car frame origin is at the center of the car
    # floor, so vertical prismatic travel reads as actual floor-to-floor motion.
    car = model.part("car")
    car.visual(Box((2.10, 1.65, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=dark, name="car_floor")
    car.visual(Box((2.10, 1.65, 0.12)), origin=Origin(xyz=(0.0, 0.0, 2.42)), material=galvanized, name="car_roof")
    car.visual(Box((2.10, 0.08, 2.36)), origin=Origin(xyz=(0.0, 0.825, 1.24)), material=brushed, name="rear_panel")
    car.visual(Box((0.08, 1.65, 2.36)), origin=Origin(xyz=(-1.05, 0.0, 1.24)), material=brushed, name="side_panel_0")
    car.visual(Box((0.08, 1.65, 2.36)), origin=Origin(xyz=(1.05, 0.0, 1.24)), material=brushed, name="side_panel_1")
    car.visual(Box((0.34, 0.08, 2.22)), origin=Origin(xyz=(-0.88, -0.825, 1.17)), material=brushed, name="front_jamb_0")
    car.visual(Box((0.34, 0.08, 2.22)), origin=Origin(xyz=(0.88, -0.825, 1.17)), material=brushed, name="front_jamb_1")
    car.visual(Box((1.42, 0.08, 0.24)), origin=Origin(xyz=(0.0, -0.825, 2.22)), material=brushed, name="front_header")
    car.visual(Box((1.42, 0.08, 0.08)), origin=Origin(xyz=(0.0, -0.825, 0.14)), material=rail, name="door_sill")
    car.visual(Box((1.44, 0.06, 0.08)), origin=Origin(xyz=(0.0, -0.8625, 2.09)), material=rail, name="door_track")
    car.visual(Box((1.45, 0.05, 0.08)), origin=Origin(xyz=(0.0, 0.80, 2.52)), material=galvanized, name="rope_hitch")
    car.visual(Cylinder(radius=0.035, length=1.25), origin=Origin(xyz=(0.0, 0.775, 1.05), rpy=(0.0, math.pi / 2.0, 0.0)), material=rail, name="handrail")
    car.visual(Box((0.095, 0.20, 0.30)), origin=Origin(xyz=(-1.1375, 0.35, 1.35)), material=yellow, name="guide_shoe_0")
    car.visual(Box((0.095, 0.20, 0.30)), origin=Origin(xyz=(1.1375, 0.35, 1.35)), material=yellow, name="guide_shoe_1")

    # Counterweight included because a traction elevator is visibly balanced by
    # one.  It is mimic-linked to the car lift so it descends as the car rises.
    counterweight = model.part("counterweight")
    counterweight.visual(Box((0.34, 0.82, 1.58)), origin=Origin(xyz=(0.0, 0.0, 0.79)), material=dark, name="weight_stack")
    counterweight.visual(Box((0.42, 0.90, 0.08)), origin=Origin(xyz=(0.0, 0.0, 1.62)), material=galvanized, name="top_frame")
    counterweight.visual(Box((0.42, 0.90, 0.08)), origin=Origin(xyz=(0.0, 0.0, -0.04)), material=galvanized, name="bottom_frame")
    counterweight.visual(Box((0.06, 0.96, 1.70)), origin=Origin(xyz=(-0.23, 0.0, 0.80)), material=galvanized, name="side_frame_0")
    counterweight.visual(Box((0.06, 0.96, 1.70)), origin=Origin(xyz=(0.23, 0.0, 0.80)), material=galvanized, name="side_frame_1")
    counterweight.visual(Box((0.18, 0.18, 0.10)), origin=Origin(xyz=(0.0, 0.0, 1.70)), material=yellow, name="rope_socket")

    door_0 = model.part("door_0")
    door_0.visual(Box((0.55, 0.045, 1.92)), origin=Origin(xyz=(0.0, -0.005, 1.10)), material=brushed, name="door_leaf")
    door_0.visual(Box((0.18, 0.008, 0.34)), origin=Origin(xyz=(-0.08, -0.025, 1.53)), material=glass, name="vision_panel")
    door_0.visual(Box((0.52, 0.035, 0.08)), origin=Origin(xyz=(0.0, 0.005, 2.10)), material=rail, name="hanger")

    door_1 = model.part("door_1")
    door_1.visual(Box((0.55, 0.045, 1.92)), origin=Origin(xyz=(0.0, -0.005, 1.10)), material=brushed, name="door_leaf")
    door_1.visual(Box((0.18, 0.008, 0.34)), origin=Origin(xyz=(0.08, -0.025, 1.53)), material=glass, name="vision_panel")
    door_1.visual(Box((0.52, 0.035, 0.08)), origin=Origin(xyz=(0.0, 0.005, 2.10)), material=rail, name="hanger")

    car_lift = model.articulation(
        "car_lift",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22000.0, velocity=1.6, lower=0.0, upper=4.50),
    )
    model.articulation(
        "counterweight_lift",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=counterweight,
        origin=Origin(xyz=(1.34, 0.95, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=1.6, lower=0.0, upper=4.50),
        mimic=Mimic(joint=car_lift.name, multiplier=-1.0, offset=4.50),
    )
    model.articulation(
        "door_0_slide",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_0,
        origin=Origin(xyz=(-0.276, -0.915, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.65, lower=0.0, upper=0.45),
    )
    model.articulation(
        "door_1_slide",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_1,
        origin=Origin(xyz=(0.276, -0.915, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.65, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    counterweight = object_model.get_part("counterweight")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    car_lift = object_model.get_articulation("car_lift")
    door_0_slide = object_model.get_articulation("door_0_slide")
    door_1_slide = object_model.get_articulation("door_1_slide")

    ctx.expect_within(car, shaft, axes="xy", margin=0.02, name="car remains inside the shaft footprint")
    ctx.expect_gap(door_1, door_0, axis="x", max_gap=0.004, max_penetration=0.0, name="closed center doors meet at the centerline")
    ctx.expect_overlap(door_0, car, axes="z", min_overlap=1.80, name="door_0 covers the passenger opening height")
    ctx.expect_overlap(door_1, car, axes="z", min_overlap=1.80, name="door_1 covers the passenger opening height")

    rest_car = ctx.part_world_position(car)
    rest_counter = ctx.part_world_position(counterweight)
    with ctx.pose({car_lift: 4.50}):
        ctx.expect_within(car, shaft, axes="xy", margin=0.02, name="raised car stays centered in the shaft")
        raised_car = ctx.part_world_position(car)
        raised_counter = ctx.part_world_position(counterweight)

    ctx.check(
        "car lift travels vertically",
        rest_car is not None and raised_car is not None and raised_car[2] > rest_car[2] + 4.0,
        details=f"rest={rest_car}, raised={raised_car}",
    )
    ctx.check(
        "counterweight moves opposite the car",
        rest_counter is not None and raised_counter is not None and raised_counter[2] < rest_counter[2] - 4.0,
        details=f"rest={rest_counter}, raised={raised_counter}",
    )

    with ctx.pose({door_0_slide: 0.45, door_1_slide: 0.45}):
        ctx.expect_gap(door_1, door_0, axis="x", min_gap=0.88, name="door panels open away from each other")

    return ctx.report()


object_model = build_object_model()
