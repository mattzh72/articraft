from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panoramic_scenic_elevator")

    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    glass = model.material("glass", rgba=(0.69, 0.84, 0.92, 0.28))
    smoked_glass = model.material("smoked_glass", rgba=(0.50, 0.70, 0.78, 0.32))
    floor_finish = model.material("floor_finish", rgba=(0.19, 0.20, 0.22, 1.0))
    threshold = model.material("threshold", rgba=(0.82, 0.80, 0.73, 1.0))

    guide_column = model.part("guide_column")
    guide_column.visual(
        Box((2.30, 2.05, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="base_plinth",
    )

    post_height = 4.82
    post_center_z = 0.18 + post_height / 2.0
    for name, x, y in (
        ("rear_left_post", -1.08, -0.83),
        ("rear_right_post", 1.08, -0.83),
        ("front_left_post", -1.08, 0.83),
        ("front_right_post", 1.08, 0.83),
    ):
        guide_column.visual(
            Box((0.09, 0.09, post_height)),
            origin=Origin(xyz=(x, y, post_center_z)),
            material=steel,
            name=name,
        )

    for name, x in (("left_side_beam", -1.08), ("right_side_beam", 1.08)):
        guide_column.visual(
            Box((0.09, 1.75, 0.12)),
            origin=Origin(xyz=(x, 0.0, 5.06)),
            material=steel,
            name=name,
        )
    guide_column.visual(
        Box((2.25, 0.09, 0.12)),
        origin=Origin(xyz=(0.0, -0.83, 5.06)),
        material=steel,
        name="rear_top_beam",
    )
    guide_column.visual(
        Box((2.25, 0.09, 0.12)),
        origin=Origin(xyz=(0.0, 0.83, 5.06)),
        material=steel,
        name="front_top_beam",
    )
    guide_column.visual(
        Box((2.28, 1.88, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 5.17)),
        material=dark_steel,
        name="roof_cap",
    )

    guide_column.visual(
        Box((0.016, 1.57, 4.64)),
        origin=Origin(xyz=(-1.027, 0.0, 2.50)),
        material=glass,
        name="left_glass_wall",
    )
    guide_column.visual(
        Box((0.016, 1.57, 4.64)),
        origin=Origin(xyz=(1.027, 0.0, 2.50)),
        material=glass,
        name="right_glass_wall",
    )
    guide_column.visual(
        Box((1.84, 0.016, 4.64)),
        origin=Origin(xyz=(0.0, -0.787, 2.50)),
        material=glass,
        name="rear_glass_wall",
    )

    guide_column.visual(
        Box((0.09, 0.16, 0.18)),
        origin=Origin(xyz=(-1.045, -0.68, 0.43)),
        material=steel,
        name="left_rail_base_bracket",
    )
    guide_column.visual(
        Box((0.09, 0.16, 0.18)),
        origin=Origin(xyz=(1.045, -0.68, 0.43)),
        material=steel,
        name="right_rail_base_bracket",
    )
    guide_column.visual(
        Box((0.09, 0.16, 0.18)),
        origin=Origin(xyz=(-1.045, -0.68, 4.73)),
        material=steel,
        name="left_rail_top_bracket",
    )
    guide_column.visual(
        Box((0.09, 0.16, 0.18)),
        origin=Origin(xyz=(1.045, -0.68, 4.73)),
        material=steel,
        name="right_rail_top_bracket",
    )
    guide_column.visual(
        Box((0.02, 0.08, 4.28)),
        origin=Origin(xyz=(-1.00, -0.68, 2.50)),
        material=dark_steel,
        name="guide_rail_left",
    )
    guide_column.visual(
        Box((0.02, 0.08, 4.28)),
        origin=Origin(xyz=(1.00, -0.68, 2.50)),
        material=dark_steel,
        name="guide_rail_right",
    )
    guide_column.inertial = Inertial.from_geometry(
        Box((2.30, 2.05, 5.22)),
        mass=1200.0,
        origin=Origin(xyz=(0.0, 0.0, 2.61)),
    )

    elevator_car = model.part("elevator_car")
    elevator_car.visual(
        Box((1.82, 1.42, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=floor_finish,
        name="car_floor",
    )
    elevator_car.visual(
        Box((1.76, 1.34, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=threshold,
        name="floor_trim",
    )
    elevator_car.visual(
        Box((1.82, 1.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.26)),
        material=steel,
        name="car_roof",
    )
    for name, x, y in (
        ("rear_left_stile", -0.88, -0.68),
        ("rear_right_stile", 0.88, -0.68),
        ("front_left_stile", -0.88, 0.68),
        ("front_right_stile", 0.88, 0.68),
    ):
        elevator_car.visual(
            Box((0.06, 0.06, 2.17)),
            origin=Origin(xyz=(x, y, 1.175)),
            material=steel,
            name=name,
        )
    elevator_car.visual(
        Box((1.70, 0.014, 2.13)),
        origin=Origin(xyz=(0.0, -0.68, 1.155)),
        material=smoked_glass,
        name="rear_glass",
    )
    elevator_car.visual(
        Box((0.014, 1.30, 2.13)),
        origin=Origin(xyz=(-0.88, 0.0, 1.155)),
        material=smoked_glass,
        name="left_glass",
    )
    elevator_car.visual(
        Box((0.014, 1.30, 2.13)),
        origin=Origin(xyz=(0.88, 0.0, 1.155)),
        material=smoked_glass,
        name="right_glass",
    )
    elevator_car.visual(
        Box((0.80, 0.014, 2.13)),
        origin=Origin(xyz=(0.44, 0.68, 1.155)),
        material=smoked_glass,
        name="front_sidelite",
    )
    elevator_car.visual(
        Box((1.76, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.67, 2.19)),
        material=steel,
        name="front_header",
    )
    elevator_car.visual(
        Box((1.76, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.67, 0.11)),
        material=steel,
        name="front_sill",
    )
    elevator_car.visual(
        Box((0.24, 0.08, 2.10)),
        origin=Origin(xyz=(0.0, -0.73, 1.14)),
        material=dark_steel,
        name="rear_sling_mast",
    )
    elevator_car.visual(
        Box((0.01, 0.05, 2.10)),
        origin=Origin(xyz=(-0.845, 0.655, 1.16)),
        material=steel,
        name="door_stop_left",
    )
    elevator_car.visual(
        Box((0.04, 0.05, 2.10)),
        origin=Origin(xyz=(0.01, 0.68, 1.16)),
        material=steel,
        name="door_meeting_stile",
    )
    elevator_car.visual(
        Box((0.84, 0.02, 0.02)),
        origin=Origin(xyz=(-0.425, 0.70, 0.10)),
        material=threshold,
        name="sill_cap",
    )
    for name, x, z in (
        ("guide_shoe_lower_left", -0.95, 0.55),
        ("guide_shoe_upper_left", -0.95, 1.79),
        ("guide_shoe_lower_right", 0.95, 0.55),
        ("guide_shoe_upper_right", 0.95, 1.79),
    ):
        elevator_car.visual(
            Box((0.08, 0.06, 0.14)),
            origin=Origin(xyz=(x, -0.68, z)),
            material=dark_steel,
            name=name,
        )
    elevator_car.inertial = Inertial.from_geometry(
        Box((1.82, 1.42, 2.34)),
        mass=680.0,
        origin=Origin(xyz=(0.0, 0.0, 1.17)),
    )

    model.articulation(
        "car_travel",
        ArticulationType.PRISMATIC,
        parent=guide_column,
        child=elevator_car,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18000.0,
            velocity=1.6,
            lower=0.0,
            upper=2.22,
        ),
    )

    car_door = model.part("car_door")
    car_door.visual(
        Box((0.75, 0.014, 1.86)),
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        material=glass,
        name="door_glass",
    )
    car_door.visual(
        Box((0.04, 0.022, 1.98)),
        origin=Origin(xyz=(-0.395, 0.0, 0.99)),
        material=steel,
        name="left_stile",
    )
    car_door.visual(
        Box((0.04, 0.022, 1.98)),
        origin=Origin(xyz=(0.395, 0.0, 0.99)),
        material=steel,
        name="right_stile",
    )
    car_door.visual(
        Box((0.83, 0.022, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=steel,
        name="bottom_rail",
    )
    car_door.visual(
        Box((0.83, 0.022, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 1.95)),
        material=steel,
        name="top_rail",
    )
    car_door.inertial = Inertial.from_geometry(
        Box((0.83, 0.022, 1.98)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
    )

    model.articulation(
        "door_slide",
        ArticulationType.PRISMATIC,
        parent=elevator_car,
        child=car_door,
        origin=Origin(xyz=(-0.425, 0.655, 0.131)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.8,
            lower=0.0,
            upper=0.84,
        ),
    )

    safety_lip = model.part("safety_lip")
    safety_lip.visual(
        Box((0.84, 0.12, 0.02)),
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
        material=threshold,
        name="lip_plate",
    )
    safety_lip.visual(
        Box((0.84, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.105, -0.005)),
        material=steel,
        name="lip_nosing",
    )
    safety_lip.inertial = Inertial.from_geometry(
        Box((0.84, 0.12, 0.03)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
    )

    model.articulation(
        "lip_hinge",
        ArticulationType.REVOLUTE,
        parent=elevator_car,
        child=safety_lip,
        origin=Origin(xyz=(-0.425, 0.71, 0.10)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=1.5,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_column = object_model.get_part("guide_column")
    elevator_car = object_model.get_part("elevator_car")
    car_door = object_model.get_part("car_door")
    safety_lip = object_model.get_part("safety_lip")
    car_travel = object_model.get_articulation("car_travel")
    door_slide = object_model.get_articulation("door_slide")
    lip_hinge = object_model.get_articulation("lip_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        elevator_car,
        guide_column,
        elem_a="guide_shoe_lower_left",
        elem_b="guide_rail_left",
    )
    ctx.expect_contact(
        elevator_car,
        guide_column,
        elem_a="guide_shoe_lower_right",
        elem_b="guide_rail_right",
    )
    ctx.expect_contact(
        car_door,
        elevator_car,
        elem_a="left_stile",
        elem_b="door_stop_left",
    )
    ctx.expect_contact(
        safety_lip,
        elevator_car,
        elem_a="lip_plate",
        elem_b="sill_cap",
    )
    ctx.expect_within(elevator_car, guide_column, axes="xy", margin=0.03)
    ctx.expect_within(car_door, elevator_car, axes="yz", margin=0.03)

    car_rest = ctx.part_world_position(elevator_car)
    door_rest = ctx.part_world_position(car_door)
    lip_rest = ctx.part_element_world_aabb(safety_lip, elem="lip_plate")
    assert car_rest is not None
    assert door_rest is not None
    assert lip_rest is not None

    with ctx.pose({car_travel: 2.10}):
        car_high = ctx.part_world_position(elevator_car)
        assert car_high is not None
        assert car_high[2] > car_rest[2] + 2.0
        ctx.expect_within(elevator_car, guide_column, axes="xy", margin=0.03)

    with ctx.pose({door_slide: 0.84}):
        door_open = ctx.part_world_position(car_door)
        assert door_open is not None
        assert door_open[0] > door_rest[0] + 0.80
        ctx.expect_within(car_door, elevator_car, axes="yz", margin=0.03)

    with ctx.pose({lip_hinge: 1.0}):
        lip_deployed = ctx.part_element_world_aabb(safety_lip, elem="lip_plate")
        assert lip_deployed is not None
        assert lip_deployed[0][2] < lip_rest[0][2] - 0.08

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
