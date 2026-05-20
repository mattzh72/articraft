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


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder_y(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_scenic_elevator")

    glass = model.material("slightly_blue_glass", rgba=(0.55, 0.82, 1.0, 0.28))
    dark_glass = model.material("smoked_car_glass", rgba=(0.36, 0.70, 0.95, 0.34))
    brushed = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_metal = model.material("dark_guide_steel", rgba=(0.10, 0.12, 0.13, 1.0))
    floor_mat = model.material("warm_gray_floor", rgba=(0.30, 0.31, 0.30, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    column = model.part("guide_column")
    _box(column, "foundation_slab", (2.35, 3.10, 0.18), (0.0, 0.0, 0.09), dark_metal)
    _box(column, "top_header", (2.35, 3.10, 0.16), (0.0, 0.0, 5.48), dark_metal)
    for x in (-1.10, 1.10):
        for y in (-1.45, 1.45):
            _box(column, f"corner_post_{x}_{y}", (0.085, 0.085, 5.30), (x, y, 2.80), brushed)
    _box(column, "rear_glass_wall", (0.018, 2.82, 4.85), (-1.10, 0.0, 2.78), glass)
    _box(column, "front_glass_wall", (0.018, 2.82, 4.85), (1.10, 0.0, 2.78), glass)
    _box(column, "side_glass_0", (2.12, 0.018, 4.85), (0.0, -1.45, 2.78), glass)
    _box(column, "side_glass_1", (2.12, 0.018, 4.85), (0.0, 1.45, 2.78), glass)
    _box(column, "rear_guide_rail_0", (0.040, 0.040, 5.30), (-1.06, -0.86, 2.80), dark_metal)
    _box(column, "rear_guide_rail_1", (0.040, 0.040, 5.30), (-1.06, 0.86, 2.80), dark_metal)
    _box(column, "bottom_landing_sill", (0.16, 1.55, 0.06), (1.00, 0.0, 0.21), brushed)

    car = model.part("glass_car")
    _box(car, "floor_plate", (1.36, 1.48, 0.12), (0.0, 0.0, 0.06), floor_mat)
    _box(car, "front_sill", (0.10, 1.48, 0.075), (0.66, 0.0, 0.155), brushed)
    _box(car, "rear_sill", (0.09, 1.48, 0.08), (-0.66, 0.0, 0.16), brushed)
    _box(car, "front_header", (0.10, 1.48, 0.09), (0.66, 0.0, 2.28), brushed)
    _box(car, "rear_header", (0.09, 1.48, 0.09), (-0.66, 0.0, 2.28), brushed)
    _box(car, "side_header_0", (1.34, 0.08, 0.09), (0.0, -0.74, 2.28), brushed)
    _box(car, "side_header_1", (1.34, 0.08, 0.09), (0.0, 0.74, 2.28), brushed)
    for x in (-0.66, 0.66):
        for y in (-0.74, 0.74):
            _box(car, f"car_corner_post_{x}_{y}", (0.065, 0.065, 2.26), (x, y, 1.18), brushed)
    _box(car, "rear_glass_panel", (0.022, 1.42, 1.86), (-0.70, 0.0, 1.20), dark_glass)
    _box(car, "side_glass_panel_0", (1.28, 0.022, 1.86), (0.0, -0.77, 1.20), dark_glass)
    _box(car, "side_glass_panel_1", (1.28, 0.022, 1.86), (0.0, 0.77, 1.20), dark_glass)
    _box(car, "lower_door_track", (0.14, 1.38, 0.060), (0.78, 0.0, 0.2225), dark_metal)
    _box(car, "upper_door_track", (0.14, 1.38, 0.110), (0.78, 0.0, 2.1825), dark_metal)
    _box(car, "guide_shoe_arm_0", (0.36, 0.12, 0.08), (-0.80, -0.80, 1.20), brushed)
    _box(car, "guide_shoe_arm_1", (0.36, 0.12, 0.08), (-0.80, 0.80, 1.20), brushed)
    _box(car, "guide_shoe_0", (0.08, 0.11, 0.18), (-1.00, -0.86, 1.20), black)
    _box(car, "guide_shoe_1", (0.08, 0.11, 0.18), (-1.00, 0.86, 1.20), black)

    door = model.part("sliding_door")
    _box(door, "door_glass", (0.030, 1.20, 1.76), (0.0, 0.0, 0.0), glass)
    _box(door, "door_top_rail", (0.044, 1.24, 0.055), (0.0, 0.0, 0.91), brushed)
    _box(door, "door_bottom_rail", (0.044, 1.24, 0.055), (0.0, 0.0, -0.91), brushed)
    _box(door, "door_side_rail_0", (0.044, 0.050, 1.86), (0.0, -0.62, 0.0), brushed)
    _box(door, "door_side_rail_1", (0.044, 0.050, 1.86), (0.0, 0.62, 0.0), brushed)
    _box(door, "vertical_pull", (0.030, 0.040, 0.58), (0.030, -0.44, 0.05), dark_metal)
    _box(door, "lower_track_shoe", (0.050, 0.20, 0.035), (-0.045, 0.0, -0.91), dark_metal)
    _box(door, "upper_track_shoe", (0.050, 0.20, 0.035), (-0.045, 0.0, 0.91), dark_metal)

    lip = model.part("safety_lip")
    _box(lip, "lip_plate", (0.32, 1.34, 0.035), (0.16, 0.0, -0.018), safety_yellow)
    _box(lip, "black_grip_strip", (0.030, 1.34, 0.010), (0.30, 0.0, 0.0045), black)
    _box(lip, "hinge_leaf", (0.020, 1.32, 0.050), (-0.015, 0.0, 0.014), brushed)
    _cylinder_y(lip, "hinge_barrel", 0.025, 1.36, (0.0, 0.0, -0.018), brushed)

    model.articulation(
        "column_to_car",
        ArticulationType.PRISMATIC,
        parent=column,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=1.0, lower=0.0, upper=2.35),
    )
    model.articulation(
        "car_to_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door,
        origin=Origin(xyz=(0.872, 0.0, 1.19)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.55, lower=0.0, upper=0.62),
    )
    model.articulation(
        "car_to_lip",
        ArticulationType.REVOLUTE,
        parent=car,
        child=lip,
        origin=Origin(xyz=(0.735, 0.0, 0.12)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=0.0, upper=0.72),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("guide_column")
    car = object_model.get_part("glass_car")
    door = object_model.get_part("sliding_door")
    lip = object_model.get_part("safety_lip")
    lift = object_model.get_articulation("column_to_car")
    door_slide = object_model.get_articulation("car_to_door")
    lip_hinge = object_model.get_articulation("car_to_lip")

    ctx.check(
        "primary mechanisms are articulated",
        lift.articulation_type == ArticulationType.PRISMATIC
        and door_slide.articulation_type == ArticulationType.PRISMATIC
        and lip_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"{lift.articulation_type}, {door_slide.articulation_type}, {lip_hinge.articulation_type}",
    )
    ctx.expect_within(
        car,
        column,
        axes="xy",
        margin=0.02,
        name="car rides inside glass guide column footprint",
    )
    ctx.expect_gap(
        lip,
        car,
        axis="x",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem="hinge_leaf",
        negative_elem="front_sill",
        name="safety lip hinge sits at car sill",
    )
    ctx.expect_overlap(
        lip,
        car,
        axes="y",
        min_overlap=1.20,
        elem_a="lip_plate",
        elem_b="front_sill",
        name="safety lip spans the threshold width",
    )

    rest_car = ctx.part_world_position(car)
    rest_door = ctx.part_world_position(door)
    rest_lip_aabb = ctx.part_world_aabb(lip)
    with ctx.pose({lift: 2.35, door_slide: 0.62}):
        raised_car = ctx.part_world_position(car)
        open_door = ctx.part_world_position(door)
        ctx.expect_within(
            car,
            column,
            axes="xy",
            margin=0.02,
            name="raised car remains inside guide column footprint",
        )
    with ctx.pose({lip_hinge: 0.72}):
        deployed_lip_aabb = ctx.part_world_aabb(lip)

    ctx.check(
        "car translates upward on guide column",
        rest_car is not None and raised_car is not None and raised_car[2] > rest_car[2] + 2.0,
        details=f"rest={rest_car}, raised={raised_car}",
    )
    ctx.check(
        "single door panel slides sideways",
        rest_door is not None and open_door is not None and open_door[1] > rest_door[1] + 0.5,
        details=f"closed={rest_door}, open={open_door}",
    )
    ctx.check(
        "threshold lip hinges downward from sill",
        rest_lip_aabb is not None
        and deployed_lip_aabb is not None
        and deployed_lip_aabb[0][2] < rest_lip_aabb[0][2] - 0.12,
        details=f"rest={rest_lip_aabb}, deployed={deployed_lip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
