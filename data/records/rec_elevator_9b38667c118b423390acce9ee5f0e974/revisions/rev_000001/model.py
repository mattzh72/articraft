from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_home_elevator")

    brushed_steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_rail = Material("dark_guide_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    warm_wall = Material("warm_white_car", rgba=(0.86, 0.84, 0.78, 1.0))
    floor_mat = Material("stone_floor", rgba=(0.36, 0.35, 0.33, 1.0))
    door_mat = Material("satin_door_panel", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_glass = Material("smoked_glass", rgba=(0.05, 0.07, 0.08, 0.72))
    rubber = Material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    frame = model.part("guide_frame")
    frame.visual(
        Box((1.40, 0.16, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=brushed_steel,
        name="base_beam",
    )
    frame.visual(
        Box((1.40, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 3.46)),
        material=brushed_steel,
        name="top_beam",
    )
    for rail_name, foot_name, x in (
        ("left_rail", "left_base_foot", -0.60),
        ("right_rail", "right_base_foot", 0.60),
    ):
        frame.visual(
            Box((0.06, 0.08, 3.40)),
            origin=Origin(xyz=(x, 0.0, 1.75)),
            material=dark_rail,
            name=rail_name,
        )
        frame.visual(
            Box((0.14, 0.10, 0.12)),
            origin=Origin(xyz=(x, 0.0, 0.12)),
            material=brushed_steel,
            name=foot_name,
        )

    car = model.part("car")
    car.visual(
        Box((1.00, 0.88, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=floor_mat,
        name="floor_slab",
    )
    car.visual(
        Box((1.00, 0.88, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.12)),
        material=warm_wall,
        name="ceiling_slab",
    )
    car.visual(
        Box((0.04, 0.88, 2.08)),
        origin=Origin(xyz=(-0.48, 0.0, 1.08)),
        material=warm_wall,
        name="side_wall_0",
    )
    car.visual(
        Box((0.04, 0.88, 2.08)),
        origin=Origin(xyz=(0.48, 0.0, 1.08)),
        material=warm_wall,
        name="side_wall_1",
    )
    car.visual(
        Box((1.00, 0.04, 2.08)),
        origin=Origin(xyz=(0.0, 0.42, 1.08)),
        material=warm_wall,
        name="back_wall",
    )
    car.visual(
        Box((1.60, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, -0.465, 2.085)),
        material=brushed_steel,
        name="top_track",
    )
    car.visual(
        Box((1.60, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, -0.465, 0.095)),
        material=brushed_steel,
        name="bottom_sill",
    )
    for shoe_name, x, z in (
        ("left_lower_shoe", -0.535, 0.36),
        ("left_upper_shoe", -0.535, 1.72),
        ("right_lower_shoe", 0.535, 0.36),
        ("right_upper_shoe", 0.535, 1.72),
    ):
        car.visual(
            Box((0.07, 0.12, 0.16)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=brushed_steel,
            name=shoe_name,
        )

    left_door = model.part("left_door")
    left_door.visual(
        Box((0.415, 0.035, 1.89)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=door_mat,
        name="door_leaf",
    )
    left_door.visual(
        Box((0.007, 0.038, 1.89)),
        origin=Origin(xyz=(0.211, 0.0, 0.0)),
        material=rubber,
        name="meeting_seal",
    )
    left_door.visual(
        Box((0.32, 0.035, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.975)),
        material=brushed_steel,
        name="top_hanger",
    )
    left_door.visual(
        Box((0.12, 0.006, 0.85)),
        origin=Origin(xyz=(-0.06, -0.0205, 0.04)),
        material=dark_glass,
        name="glass_lite",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((0.415, 0.035, 1.89)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=door_mat,
        name="door_leaf",
    )
    right_door.visual(
        Box((0.007, 0.038, 1.89)),
        origin=Origin(xyz=(-0.211, 0.0, 0.0)),
        material=rubber,
        name="meeting_seal",
    )
    right_door.visual(
        Box((0.32, 0.035, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.975)),
        material=brushed_steel,
        name="top_hanger",
    )
    right_door.visual(
        Box((0.12, 0.006, 0.85)),
        origin=Origin(xyz=(0.06, -0.0205, 0.04)),
        material=dark_glass,
        name="glass_lite",
    )

    model.articulation(
        "frame_to_car",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.45, lower=0.0, upper=0.85),
    )
    model.articulation(
        "car_to_left_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=left_door,
        origin=Origin(xyz=(-0.218, -0.47, 1.055)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.34),
    )
    model.articulation(
        "car_to_right_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=right_door,
        origin=Origin(xyz=(0.218, -0.47, 1.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.34),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("guide_frame")
    car = object_model.get_part("car")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    car_slide = object_model.get_articulation("frame_to_car")
    left_slide = object_model.get_articulation("car_to_left_door")
    right_slide = object_model.get_articulation("car_to_right_door")

    ctx.expect_contact(
        car,
        frame,
        elem_a="right_lower_shoe",
        elem_b="right_rail",
        contact_tol=1e-5,
        name="car guide shoe rides the right rail",
    )
    ctx.expect_contact(
        car,
        frame,
        elem_a="left_lower_shoe",
        elem_b="left_rail",
        contact_tol=1e-5,
        name="car guide shoe rides the left rail",
    )
    ctx.expect_contact(
        left_door,
        car,
        elem_a="top_hanger",
        elem_b="top_track",
        contact_tol=1e-5,
        name="left door hanger is captured by top track",
    )
    ctx.expect_contact(
        right_door,
        car,
        elem_a="top_hanger",
        elem_b="top_track",
        contact_tol=1e-5,
        name="right door hanger is captured by top track",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.004,
        max_gap=0.012,
        name="closed bi-parting doors leave a narrow center reveal",
    )

    rest_car_pos = ctx.part_world_position(car)
    with ctx.pose({car_slide: 0.85}):
        raised_car_pos = ctx.part_world_position(car)
        ctx.expect_contact(
            car,
            frame,
            elem_a="right_upper_shoe",
            elem_b="right_rail",
            contact_tol=1e-5,
            name="raised car remains guided on the right rail",
        )
        ctx.expect_contact(
            car,
            frame,
            elem_a="left_upper_shoe",
            elem_b="left_rail",
            contact_tol=1e-5,
            name="raised car remains guided on the left rail",
        )

    ctx.check(
        "car prismatic joint lifts vertically",
        rest_car_pos is not None
        and raised_car_pos is not None
        and raised_car_pos[2] > rest_car_pos[2] + 0.80,
        details=f"rest={rest_car_pos}, raised={raised_car_pos}",
    )

    rest_left_pos = ctx.part_world_position(left_door)
    rest_right_pos = ctx.part_world_position(right_door)
    with ctx.pose({left_slide: 0.34, right_slide: 0.34}):
        open_left_pos = ctx.part_world_position(left_door)
        open_right_pos = ctx.part_world_position(right_door)
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.65,
            name="opened bi-parting doors separate to opposite sides",
        )
        ctx.expect_within(
            left_door,
            car,
            axes="x",
            inner_elem="top_hanger",
            outer_elem="top_track",
            margin=0.002,
            name="left door remains under the header track when open",
        )
        ctx.expect_within(
            right_door,
            car,
            axes="x",
            inner_elem="top_hanger",
            outer_elem="top_track",
            margin=0.002,
            name="right door remains under the header track when open",
        )

    ctx.check(
        "left door slides away from the center",
        rest_left_pos is not None
        and open_left_pos is not None
        and open_left_pos[0] < rest_left_pos[0] - 0.30,
        details=f"rest={rest_left_pos}, open={open_left_pos}",
    )
    ctx.check(
        "right door slides away from the center",
        rest_right_pos is not None
        and open_right_pos is not None
        and open_right_pos[0] > rest_right_pos[0] + 0.30,
        details=f"rest={rest_right_pos}, open={open_right_pos}",
    )

    return ctx.report()


object_model = build_object_model()
