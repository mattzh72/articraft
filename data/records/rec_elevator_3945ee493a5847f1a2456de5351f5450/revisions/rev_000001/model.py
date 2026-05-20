from __future__ import annotations

from math import pi

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


CAR_WIDTH = 2.65
CAR_DEPTH = 3.50
CAR_HEIGHT = 2.45
DOOR_WIDTH = 1.16
DOOR_HEIGHT = 2.10
DOOR_THICKNESS = 0.060
DOOR_TRAVEL = 0.76
CAR_LIFT = 1.05
GUIDE_SHOE_X = 1.4297


def _box(part, size, center, material, name):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _cylinder(part, radius, length, center, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_stretcher_elevator")

    model.material("painted_column", rgba=(0.52, 0.55, 0.58, 1.0))
    model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("brushed_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    model.material("car_wall", rgba=(0.78, 0.80, 0.78, 1.0))
    model.material("floor_plate", rgba=(0.35, 0.37, 0.38, 1.0))
    model.material("door_satin", rgba=(0.82, 0.84, 0.84, 1.0))
    model.material("window_glass", rgba=(0.36, 0.58, 0.70, 0.45))
    model.material("rubber_black", rgba=(0.03, 0.03, 0.03, 1.0))
    model.material("warning_yellow", rgba=(0.92, 0.72, 0.10, 1.0))

    shaft = model.part("shaft_frame")
    _box(shaft, (3.70, 4.25, 0.12), (0.0, 0.0, 0.06), "dark_steel", "pit_floor")
    _box(shaft, (3.20, 0.16, 0.16), (0.0, -1.55, 3.64), "painted_column", "front_top_beam")
    _box(shaft, (3.20, 0.16, 0.16), (0.0, 1.55, 3.64), "painted_column", "rear_top_beam")
    _box(shaft, (0.16, 3.26, 0.16), (-1.52, 0.0, 3.64), "painted_column", "side_top_beam_0")
    _box(shaft, (0.16, 3.26, 0.16), (1.52, 0.0, 3.64), "painted_column", "side_top_beam_1")
    for x, side in [(-1.52, "0"), (1.52, "1")]:
        for y, end in [(-1.55, "front"), (1.55, "rear")]:
            _box(
                shaft,
                (0.08, 0.12, 3.52),
                (x * 1.01, y, 1.86),
                "painted_column",
                f"guide_column_{side}_{end}",
            )
            _box(
                shaft,
                (0.055, 0.090, 3.36),
                (x * 0.985, y, 1.88),
                "brushed_steel",
                f"guide_rail_{side}_{end}",
            )
    _box(shaft, (0.26, 3.40, 0.08), (-1.52, 0.0, 0.34), "dark_steel", "lower_rail_tie_0")
    _box(shaft, (0.26, 3.40, 0.08), (1.52, 0.0, 0.34), "dark_steel", "lower_rail_tie_1")
    _box(shaft, (1.30, 0.15, 0.18), (0.0, -1.55, 3.81), "dark_steel", "machine_header")
    _box(shaft, (0.92, 0.05, 0.34), (0.0, -2.13, 1.65), "warning_yellow", "landing_clearance_bar")
    _box(shaft, (0.06, 0.06, 1.65), (-0.49, -2.13, 0.865), "dark_steel", "landing_bar_post_0")
    _box(shaft, (0.06, 0.06, 1.65), (0.49, -2.13, 0.865), "dark_steel", "landing_bar_post_1")

    car = model.part("car")
    _box(car, (CAR_WIDTH, CAR_DEPTH, 0.14), (0.0, 0.0, 0.07), "floor_plate", "car_floor")
    _box(car, (CAR_WIDTH, CAR_DEPTH, 0.12), (0.0, 0.0, CAR_HEIGHT), "car_wall", "car_roof")
    _box(car, (0.10, CAR_DEPTH, CAR_HEIGHT), (-CAR_WIDTH / 2 + 0.05, 0.0, CAR_HEIGHT / 2), "car_wall", "side_wall_0")
    _box(car, (0.10, CAR_DEPTH, CAR_HEIGHT), (CAR_WIDTH / 2 - 0.05, 0.0, CAR_HEIGHT / 2), "car_wall", "side_wall_1")
    _box(car, (CAR_WIDTH, 0.10, CAR_HEIGHT), (0.0, CAR_DEPTH / 2 - 0.05, CAR_HEIGHT / 2), "car_wall", "rear_wall")
    _box(car, (CAR_WIDTH, 0.12, 0.22), (0.0, -CAR_DEPTH / 2 - 0.01, 2.43), "car_wall", "front_header")
    _box(car, (0.18, 0.12, 2.28), (-1.26, -CAR_DEPTH / 2 - 0.01, 1.22), "car_wall", "front_jamb_0")
    _box(car, (0.18, 0.12, 2.28), (1.26, -CAR_DEPTH / 2 - 0.01, 1.22), "car_wall", "front_jamb_1")
    _box(car, (2.35, 0.10, 0.09), (0.0, -CAR_DEPTH / 2 - 0.03, 0.17), "brushed_steel", "door_sill_track")
    _box(car, (2.55, 0.12, 0.10), (0.0, -CAR_DEPTH / 2 - 0.037, 2.41), "brushed_steel", "upper_door_track")
    _box(car, (2.25, 0.035, 0.055), (0.0, -CAR_DEPTH / 2 - 0.1145, 2.355), "dark_steel", "hanger_rail")
    _box(car, (0.14, 3.05, 0.16), (-1.38, 0.0, 1.23), "dark_steel", "car_side_belt_0")
    _box(car, (0.14, 3.05, 0.16), (1.38, 0.0, 1.23), "dark_steel", "car_side_belt_1")
    for x, side in [(-GUIDE_SHOE_X, "0"), (GUIDE_SHOE_X, "1")]:
        for y, end in [(-1.55, "front"), (1.55, "rear")]:
            _box(car, (0.080, 0.22, 0.18), (x, y, 0.82), "brushed_steel", f"lower_guide_shoe_{side}_{end}")
            _box(car, (0.080, 0.22, 0.18), (x, y, 1.92), "brushed_steel", f"upper_guide_shoe_{side}_{end}")
            _box(car, (0.15, 0.16, 0.12), (x * 0.95, y, 0.82), "dark_steel", f"lower_shoe_bracket_{side}_{end}")
            _box(car, (0.15, 0.16, 0.12), (x * 0.95, y, 1.92), "dark_steel", f"upper_shoe_bracket_{side}_{end}")

    door_0 = model.part("door_0")
    _box(door_0, (DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT), (0.0, 0.0, 0.0), "door_satin", "panel")
    _box(door_0, (0.34, 0.018, 0.62), (0.34, -0.034, 0.30), "window_glass", "vision_window")
    _box(door_0, (0.08, 0.025, 0.70), (0.48, -0.046, 0.08), "dark_steel", "vertical_pull")
    _box(door_0, (0.88, 0.030, 0.06), (0.0, 0.0, 1.08), "dark_steel", "top_hanger_bar")
    for x, idx in [(-0.38, "0"), (0.38, "1")]:
        _box(door_0, (0.045, 0.030, 0.080), (x, 0.0, 1.125), "dark_steel", f"roller_stem_{idx}")
        _cylinder(door_0, 0.052, 0.036, (x, 0.0, 1.185), "rubber_black", f"hanger_wheel_{idx}", rpy=(pi / 2, 0.0, 0.0))
    _box(door_0, (0.62, 0.026, 0.035), (0.0, 0.0, -1.065), "dark_steel", "bottom_guide")

    door_1 = model.part("door_1")
    _box(door_1, (DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT), (0.0, 0.0, 0.0), "door_satin", "panel")
    _box(door_1, (0.34, 0.018, 0.62), (-0.34, -0.034, 0.30), "window_glass", "vision_window")
    _box(door_1, (0.08, 0.025, 0.70), (-0.48, -0.046, 0.08), "dark_steel", "vertical_pull")
    _box(door_1, (0.88, 0.030, 0.06), (0.0, 0.0, 1.08), "dark_steel", "top_hanger_bar")
    for x, idx in [(-0.38, "0"), (0.38, "1")]:
        _box(door_1, (0.045, 0.030, 0.080), (x, 0.0, 1.125), "dark_steel", f"roller_stem_{idx}")
        _cylinder(door_1, 0.052, 0.036, (x, 0.0, 1.185), "rubber_black", f"hanger_wheel_{idx}", rpy=(pi / 2, 0.0, 0.0))
    _box(door_1, (0.62, 0.026, 0.035), (0.0, 0.0, -1.065), "dark_steel", "bottom_guide")

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.65, lower=0.0, upper=CAR_LIFT),
    )
    model.articulation(
        "car_to_door_0",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_0,
        origin=Origin(xyz=(-0.59, -CAR_DEPTH / 2 - 0.15, 1.24)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.50, lower=0.0, upper=DOOR_TRAVEL),
    )
    model.articulation(
        "car_to_door_1",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_1,
        origin=Origin(xyz=(0.59, -CAR_DEPTH / 2 - 0.15, 1.24)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=550.0, velocity=0.50, lower=0.0, upper=DOOR_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft = object_model.get_part("shaft_frame")
    car = object_model.get_part("car")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    lift = object_model.get_articulation("shaft_to_car")
    slide_0 = object_model.get_articulation("car_to_door_0")
    slide_1 = object_model.get_articulation("car_to_door_1")

    ctx.expect_gap(
        door_1,
        door_0,
        axis="x",
        min_gap=0.0,
        max_gap=0.05,
        positive_elem="panel",
        negative_elem="panel",
        name="closed door panels meet at a narrow center seam",
    )
    ctx.expect_overlap(
        door_0,
        door_1,
        axes="z",
        min_overlap=1.8,
        elem_a="panel",
        elem_b="panel",
        name="two wide door panels share the full doorway height",
    )
    ctx.expect_gap(
        shaft,
        car,
        axis="x",
        max_gap=0.012,
        positive_elem="guide_rail_1_front",
        negative_elem="lower_guide_shoe_1_front",
        name="right front guide shoe runs beside the rail",
    )
    ctx.expect_gap(
        car,
        shaft,
        axis="x",
        max_gap=0.012,
        positive_elem="lower_guide_shoe_0_front",
        negative_elem="guide_rail_0_front",
        name="left front guide shoe runs beside the rail",
    )

    rest_car_pos = ctx.part_world_position(car)
    rest_door_0_pos = ctx.part_world_position(door_0)
    rest_door_1_pos = ctx.part_world_position(door_1)
    with ctx.pose({lift: CAR_LIFT, slide_0: DOOR_TRAVEL, slide_1: DOOR_TRAVEL}):
        raised_car_pos = ctx.part_world_position(car)
        open_door_0_pos = ctx.part_world_position(door_0)
        open_door_1_pos = ctx.part_world_position(door_1)
        ctx.expect_gap(
            door_1,
            door_0,
            axis="x",
            min_gap=1.25,
            positive_elem="panel",
            negative_elem="panel",
            name="opened panels slide sideways clear of the stretcher doorway",
        )

    ctx.check(
        "car translates upward on the vertical prismatic rails",
        rest_car_pos is not None
        and raised_car_pos is not None
        and raised_car_pos[2] > rest_car_pos[2] + 0.95,
        details=f"rest={rest_car_pos}, raised={raised_car_pos}",
    )
    ctx.check(
        "door panels move in opposite sideways directions",
        rest_door_0_pos is not None
        and rest_door_1_pos is not None
        and open_door_0_pos is not None
        and open_door_1_pos is not None
        and open_door_0_pos[0] < rest_door_0_pos[0] - 0.70
        and open_door_1_pos[0] > rest_door_1_pos[0] + 0.70,
        details=f"rest0={rest_door_0_pos}, open0={open_door_0_pos}, rest1={rest_door_1_pos}, open1={open_door_1_pos}",
    )

    return ctx.report()


object_model = build_object_model()
