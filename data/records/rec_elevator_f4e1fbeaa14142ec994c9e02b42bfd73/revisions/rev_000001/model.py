from __future__ import annotations

from math import pi, sin, cos

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_pinion_residential_elevator")

    warm_galv = model.material("warm_galvanized_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    dark_steel = model.material("dark_rack_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    cabin_paint = model.material("painted_elevator_car", rgba=(0.82, 0.84, 0.80, 1.0))
    floor_mat = model.material("dark_rubber_floor", rgba=(0.05, 0.05, 0.05, 1.0))
    door_mat = model.material("brushed_door_panel", rgba=(0.66, 0.68, 0.66, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.42, 0.72, 0.95, 0.38))
    warning_red = model.material("safety_red", rgba=(0.84, 0.06, 0.03, 1.0))
    rubber_black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    shaft = model.part("shaft")
    # A compact residential lift tower: floor plate, top crosshead, rear guide
    # post, and the visible rack strip are one welded static structure.
    shaft.visual(
        Box((1.95, 1.35, 0.08)),
        origin=Origin(xyz=(-0.10, 0.02, 0.04)),
        material=warm_galv,
        name="floor_plate",
    )
    shaft.visual(
        Box((0.10, 0.12, 3.58)),
        origin=Origin(xyz=(-0.84, 0.00, 1.87)),
        material=warm_galv,
        name="rack_column",
    )
    shaft.visual(
        Box((0.055, 0.060, 3.36)),
        origin=Origin(xyz=(-0.755, 0.00, 1.84)),
        material=dark_steel,
        name="rack_backer",
    )
    rack_tooth_names = (
        "rack_tooth_0",
        "rack_tooth_1",
        "rack_tooth_2",
        "rack_tooth_3",
        "rack_tooth_4",
        "rack_tooth_5",
        "rack_tooth_6",
        "rack_tooth_7",
        "rack_tooth_8",
        "rack_tooth_9",
        "rack_tooth_10",
        "rack_tooth_11",
        "rack_tooth_12",
        "rack_tooth_13",
        "rack_tooth_14",
        "rack_tooth_15",
        "rack_tooth_16",
        "rack_tooth_17",
        "rack_tooth_18",
        "rack_tooth_19",
        "rack_tooth_20",
        "rack_tooth_21",
        "rack_tooth_22",
        "rack_tooth_23",
    )
    for i, tooth_name in enumerate(rack_tooth_names):
        z = 0.27 + i * 0.135
        shaft.visual(
            Box((0.095, 0.145, 0.026)),
            origin=Origin(xyz=(-0.765, 0.00, z)),
            material=dark_steel,
            name=tooth_name,
        )
    shaft.visual(
        Box((0.040, 0.080, 3.53)),
        origin=Origin(xyz=(-0.660, 0.32, 1.845)),
        material=warm_galv,
        name="car_guide_rail",
    )
    shaft.visual(
        Box((0.08, 0.08, 3.42)),
        origin=Origin(xyz=(0.68, 0.52, 1.79)),
        material=warm_galv,
        name="rear_guide_post",
    )
    shaft.visual(
        Box((1.55, 0.12, 0.10)),
        origin=Origin(xyz=(-0.08, 0.00, 3.66)),
        material=warm_galv,
        name="top_crosshead",
    )
    shaft.visual(
        Box((0.055, 0.055, 3.24)),
        origin=Origin(xyz=(-0.78, -0.58, 1.70)),
        material=warm_galv,
        name="front_post_0",
    )
    shaft.visual(
        Box((0.055, 0.055, 3.24)),
        origin=Origin(xyz=(0.62, -0.58, 1.70)),
        material=warm_galv,
        name="front_post_1",
    )
    shaft.visual(
        Box((1.42, 0.055, 0.055)),
        origin=Origin(xyz=(-0.08, -0.58, 3.34)),
        material=warm_galv,
        name="front_header_tie",
    )
    shaft.visual(
        Box((0.30, 0.12, 0.08)),
        origin=Origin(xyz=(-0.76, -0.08, 3.26)),
        material=warning_red,
        name="overtravel_stop",
    )

    car = model.part("car")
    # The car frame is drawn as a small residential cab: a rectangular cabin
    # with open front, side glazing, roof, floor, guide shoes, and a side drive
    # pinion close to the rack.
    car.visual(
        Box((1.12, 1.02, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=floor_mat,
        name="floor_pan",
    )
    car.visual(
        Box((1.12, 0.060, 2.06)),
        origin=Origin(xyz=(0.0, 0.51, 1.09)),
        material=cabin_paint,
        name="rear_wall",
    )
    car.visual(
        Box((0.060, 1.02, 2.06)),
        origin=Origin(xyz=(-0.56, 0.0, 1.09)),
        material=cabin_paint,
        name="rack_side_wall",
    )
    car.visual(
        Box((0.060, 1.02, 2.06)),
        origin=Origin(xyz=(0.56, 0.0, 1.09)),
        material=cabin_paint,
        name="plain_side_wall",
    )
    car.visual(
        Box((1.12, 1.02, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 2.1375)),
        material=cabin_paint,
        name="roof_panel",
    )
    car.visual(
        Box((0.14, 0.055, 2.00)),
        origin=Origin(xyz=(-0.49, -0.5375, 1.08)),
        material=cabin_paint,
        name="front_jamb_0",
    )
    car.visual(
        Box((0.14, 0.055, 2.00)),
        origin=Origin(xyz=(0.49, -0.5375, 1.08)),
        material=cabin_paint,
        name="front_jamb_1",
    )
    car.visual(
        Box((1.12, 0.055, 0.12)),
        origin=Origin(xyz=(0.0, -0.5375, 2.08)),
        material=cabin_paint,
        name="front_header",
    )
    car.visual(
        Box((1.12, 0.055, 0.10)),
        origin=Origin(xyz=(0.0, -0.5375, 0.12)),
        material=cabin_paint,
        name="front_sill",
    )
    car.visual(
        Box((0.030, 0.035, 1.76)),
        origin=Origin(xyz=(-0.02, -0.5275, 1.03)),
        material=dark_steel,
        name="door_center_post",
    )
    car.visual(
        Box((0.92, 0.045, 0.065)),
        origin=Origin(xyz=(0.0, -0.5325, 1.955)),
        material=dark_steel,
        name="door_top_track",
    )
    car.visual(
        Box((0.92, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, -0.5275, 0.185)),
        material=dark_steel,
        name="door_bottom_track",
    )
    car.visual(
        Box((0.018, 0.50, 0.70)),
        origin=Origin(xyz=(-0.594, 0.02, 1.18)),
        material=glass,
        name="side_window_0",
    )
    car.visual(
        Box((0.018, 0.50, 0.70)),
        origin=Origin(xyz=(0.594, 0.02, 1.18)),
        material=glass,
        name="side_window_1",
    )
    car.visual(
        Box((0.16, 0.22, 0.20)),
        origin=Origin(xyz=(-0.615, 0.14, 1.00)),
        material=dark_steel,
        name="drive_motor_box",
    )
    car.visual(
        Cylinder(radius=0.088, length=0.065),
        origin=Origin(xyz=(-0.595, 0.02, 1.00), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_galv,
        name="drive_pinion",
    )
    for i in range(12):
        theta = i * 2.0 * pi / 12.0
        car.visual(
            Box((0.030, 0.050, 0.020)),
            origin=Origin(
                xyz=(-0.595 + cos(theta) * 0.097, 0.02, 1.00 + sin(theta) * 0.097),
                rpy=(0.0, -theta, 0.0),
            ),
            material=warm_galv,
            name=f"pinion_tooth_{i}",
        )
    car.visual(
        Box((0.13, 0.05, 0.10)),
        origin=Origin(xyz=(-0.575, 0.32, 0.48)),
        material=rubber_black,
        name="lower_guide_shoe",
    )
    car.visual(
        Box((0.13, 0.05, 0.10)),
        origin=Origin(xyz=(-0.575, 0.32, 1.80)),
        material=rubber_black,
        name="upper_guide_shoe",
    )
    car.visual(
        Box((0.060, 0.030, 0.070)),
        origin=Origin(xyz=(-0.62, -0.1275, 2.060)),
        material=dark_steel,
        name="catch_hinge_lug_0",
    )
    car.visual(
        Box((0.060, 0.030, 0.070)),
        origin=Origin(xyz=(-0.62, -0.2525, 2.060)),
        material=dark_steel,
        name="catch_hinge_lug_1",
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.45, lower=0.0, upper=1.20),
    )

    door_panel = model.part("door_panel")
    door_panel.visual(
        Box((0.57, 0.040, 1.66)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=door_mat,
        name="sliding_panel",
    )
    door_panel.visual(
        Box((0.52, 0.012, 0.90)),
        origin=Origin(xyz=(0.0, -0.024, 0.05)),
        material=glass,
        name="door_vision_lite",
    )
    door_panel.visual(
        Box((0.12, 0.030, 0.16)),
        origin=Origin(xyz=(-0.19, 0.0, 0.91)),
        material=dark_steel,
        name="hanger_0",
    )
    door_panel.visual(
        Box((0.12, 0.030, 0.16)),
        origin=Origin(xyz=(0.19, 0.0, 0.91)),
        material=dark_steel,
        name="hanger_1",
    )
    door_panel.visual(
        Cylinder(radius=0.032, length=0.060),
        origin=Origin(xyz=(-0.19, 0.0, 0.910), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_black,
        name="roller_0",
    )
    door_panel.visual(
        Cylinder(radius=0.032, length=0.060),
        origin=Origin(xyz=(0.19, 0.0, 0.910), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_black,
        name="roller_1",
    )
    door_panel.visual(
        Box((0.035, 0.020, 0.62)),
        origin=Origin(xyz=(0.235, -0.033, -0.03)),
        material=dark_steel,
        name="recessed_pull",
    )

    model.articulation(
        "car_to_door_panel",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_panel,
        origin=Origin(xyz=(-0.04, -0.587, 1.03)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.48),
    )

    catch_arm = model.part("catch_arm")
    catch_arm.visual(
        Cylinder(radius=0.026, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    catch_arm.visual(
        Box((0.38, 0.044, 0.045)),
        origin=Origin(xyz=(-0.19, 0.0, -0.044)),
        material=warning_red,
        name="pawl_body",
    )
    catch_arm.visual(
        Box((0.090, 0.055, 0.085)),
        origin=Origin(xyz=(-0.405, 0.0, -0.060), rpy=(0.0, 0.35, 0.0)),
        material=warning_red,
        name="hook_tip",
    )
    catch_arm.visual(
        Cylinder(radius=0.012, length=0.23),
        origin=Origin(xyz=(-0.19, 0.0, -0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spring_pin",
    )

    model.articulation(
        "car_to_catch_arm",
        ArticulationType.REVOLUTE,
        parent=car,
        child=catch_arm,
        origin=Origin(xyz=(-0.62, -0.19, 2.059)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    door_panel = object_model.get_part("door_panel")
    catch_arm = object_model.get_part("catch_arm")
    lift = object_model.get_articulation("shaft_to_car")
    door_slide = object_model.get_articulation("car_to_door_panel")
    catch_hinge = object_model.get_articulation("car_to_catch_arm")

    ctx.expect_gap(
        car,
        shaft,
        axis="x",
        positive_elem="drive_pinion",
        negative_elem="rack_tooth_6",
        min_gap=0.010,
        max_gap=0.080,
        name="pinion sits just clear of the rack teeth",
    )
    ctx.expect_within(
        door_panel,
        car,
        axes="z",
        inner_elem="sliding_panel",
        outer_elem="front_jamb_0",
        margin=0.10,
        name="door panel height is captured by the car front frame",
    )
    ctx.expect_gap(
        car,
        catch_arm,
        axis="z",
        positive_elem="roof_panel",
        negative_elem="hinge_barrel",
        min_gap=0.004,
        max_gap=0.040,
        name="catch hinge is mounted just below the roof underside",
    )

    car_rest = ctx.part_world_position(car)
    with ctx.pose({lift: 1.20}):
        car_raised = ctx.part_world_position(car)
        ctx.expect_gap(
            shaft,
            car,
            axis="z",
            positive_elem="top_crosshead",
            negative_elem="roof_panel",
            min_gap=0.015,
            max_gap=0.12,
            name="raised car stops below top crosshead",
        )
    ctx.check(
        "car rides upward on the vertical prismatic joint",
        car_rest is not None and car_raised is not None and car_raised[2] > car_rest[2] + 1.10,
        details=f"rest={car_rest}, raised={car_raised}",
    )

    door_rest = ctx.part_world_position(door_panel)
    with ctx.pose({door_slide: 0.48}):
        door_open = ctx.part_world_position(door_panel)
    ctx.check(
        "front access door slides horizontally",
        door_rest is not None and door_open is not None and door_open[0] > door_rest[0] + 0.40,
        details=f"rest={door_rest}, open={door_open}",
    )

    catch_rest = ctx.part_world_aabb(catch_arm)
    with ctx.pose({catch_hinge: 0.95}):
        catch_deployed = ctx.part_world_aabb(catch_arm)
    ctx.check(
        "safety catch arm swings downward from its roof hinge",
        catch_rest is not None
        and catch_deployed is not None
        and catch_deployed[0][2] < catch_rest[0][2] - 0.10,
        details=f"rest={catch_rest}, deployed={catch_deployed}",
    )

    return ctx.report()


object_model = build_object_model()
