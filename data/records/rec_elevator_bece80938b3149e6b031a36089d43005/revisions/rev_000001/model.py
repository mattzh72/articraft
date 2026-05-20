from __future__ import annotations

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
    model = ArticulatedObject(name="glass_panoramic_elevator")

    steel = model.material("brushed_steel", rgba=(0.62, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_rail_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    floor_mat = model.material("dark_rubber_floor", rgba=(0.045, 0.048, 0.052, 1.0))
    concrete = model.material("matte_concrete", rgba=(0.55, 0.55, 0.52, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.58, 0.82, 0.95, 0.34))
    door_glass = model.material("door_glass", rgba=(0.62, 0.88, 1.0, 0.42))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    rail_system = model.part("rail_system")
    rail_system.visual(
        Box((2.45, 1.65, 0.12)),
        origin=Origin(xyz=(0.0, 0.05, 0.06)),
        material=concrete,
        name="base_slab",
    )
    for x, suffix in ((-0.965, "0"), (0.965, "1")):
        rail_system.visual(
            Cylinder(radius=0.050, length=5.25),
            origin=Origin(xyz=(x, 0.72, 2.675)),
            material=dark_steel,
            name=f"guide_column_{suffix}",
        )
    for z, name, height in ((0.18, "lower_crossbeam", 0.16), (2.72, "middle_crossbeam", 0.11), (5.16, "upper_crossbeam", 0.18)):
        rail_system.visual(
            Box((2.08, 0.16, height)),
            origin=Origin(xyz=(0.0, 0.72, z)),
            material=steel,
            name=name,
        )
    rail_system.visual(
        Box((0.18, 0.32, 5.12)),
        origin=Origin(xyz=(-0.965, 0.82, 2.68)),
        material=steel,
        name="column_backing_0",
    )
    rail_system.visual(
        Box((0.18, 0.32, 5.12)),
        origin=Origin(xyz=(0.965, 0.82, 2.68)),
        material=steel,
        name="column_backing_1",
    )

    car = model.part("car")
    car.visual(
        Box((1.55, 1.25, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=floor_mat,
        name="floor_pan",
    )
    car.visual(
        Box((1.55, 1.25, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.35)),
        material=steel,
        name="ceiling_pan",
    )
    for x, sx in ((-0.745, "0"), (0.745, "1")):
        for y, sy in ((-0.595, "front"), (0.595, "rear")):
            car.visual(
                Box((0.060, 0.060, 2.22)),
                origin=Origin(xyz=(x, y, 1.21)),
                material=steel,
                name=f"{sy}_post_{sx}",
            )
    for y, label in ((-0.595, "front"), (0.595, "rear")):
        car.visual(
            Box((1.55, 0.055, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.25)),
            material=steel,
            name=f"{label}_lower_rail",
        )
        car.visual(
            Box((1.55, 0.055, 0.070)),
            origin=Origin(xyz=(0.0, y, 2.22)),
            material=steel,
            name=f"{label}_header_rail",
        )
    for x, label in ((-0.745, "side_0"), (0.745, "side_1")):
        car.visual(
            Box((0.055, 1.25, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.25)),
            material=steel,
            name=f"{label}_lower_rail",
        )
        car.visual(
            Box((0.055, 1.25, 0.070)),
            origin=Origin(xyz=(x, 0.0, 2.22)),
            material=steel,
            name=f"{label}_header_rail",
        )
    car.visual(
        Box((1.36, 0.018, 1.96)),
        origin=Origin(xyz=(0.0, 0.626, 1.23)),
        material=glass,
        name="rear_glass",
    )
    car.visual(
        Box((0.018, 1.16, 1.86)),
        origin=Origin(xyz=(-0.775, 0.0, 1.23)),
        material=glass,
        name="side_glass_0",
    )
    car.visual(
        Box((0.018, 1.16, 1.86)),
        origin=Origin(xyz=(0.775, 0.0, 1.23)),
        material=glass,
        name="side_glass_1",
    )
    car.visual(
        Box((1.48, 0.080, 0.090)),
        origin=Origin(xyz=(0.0, -0.655, 2.18)),
        material=dark_steel,
        name="upper_door_guide",
    )
    car.visual(
        Box((1.48, 0.070, 0.060)),
        origin=Origin(xyz=(0.0, -0.655, 0.17)),
        material=dark_steel,
        name="lower_door_track",
    )
    for x, shoe_x, side in ((0.810, 0.860, "1"), (-0.810, -0.860, "0")):
        for z, level in ((0.78, "lower"), (1.85, "upper")):
            car.visual(
                Box((0.08, 0.080, 0.070)),
                origin=Origin(xyz=(x, 0.655, z)),
                material=steel,
                name=f"{level}_guide_arm_{side}",
            )
            car.visual(
                Box((0.030, 0.090, 0.180)),
                origin=Origin(xyz=(shoe_x, 0.665, z)),
                material=dark_steel,
                name=f"{level}_guide_shoe_{side}",
            )

    door_panel = model.part("door_panel")
    door_panel.visual(
        Box((0.82, 0.026, 1.82)),
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        material=door_glass,
        name="glass_panel",
    )
    door_panel.visual(
        Box((0.88, 0.040, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=steel,
        name="bottom_stile",
    )
    door_panel.visual(
        Box((0.88, 0.040, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 1.895)),
        material=steel,
        name="top_stile",
    )
    door_panel.visual(
        Box((0.045, 0.040, 1.88)),
        origin=Origin(xyz=(-0.425, 0.0, 0.98)),
        material=steel,
        name="side_stile_0",
    )
    door_panel.visual(
        Cylinder(radius=0.045, length=0.025),
        origin=Origin(xyz=(-0.234, 0.030, 1.985), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_0",
    )
    door_panel.visual(
        Box((0.050, 0.035, 0.055)),
        origin=Origin(xyz=(-0.234, 0.016, 1.925)),
        material=steel,
        name="roller_hanger_0",
    )
    door_panel.visual(
        Box((0.045, 0.040, 1.88)),
        origin=Origin(xyz=(0.425, 0.0, 0.98)),
        material=steel,
        name="side_stile_1",
    )
    door_panel.visual(
        Cylinder(radius=0.045, length=0.025),
        origin=Origin(xyz=(0.234, 0.030, 1.985), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_1",
    )
    door_panel.visual(
        Box((0.050, 0.035, 0.055)),
        origin=Origin(xyz=(0.234, 0.016, 1.925)),
        material=steel,
        name="roller_hanger_1",
    )

    model.articulation(
        "rail_to_car",
        ArticulationType.PRISMATIC,
        parent=rail_system,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=0.7, lower=0.0, upper=2.35),
    )
    model.articulation(
        "car_to_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door_panel,
        origin=Origin(xyz=(0.0, -0.7375, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.55, lower=0.0, upper=0.62),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_system = object_model.get_part("rail_system")
    car = object_model.get_part("car")
    door_panel = object_model.get_part("door_panel")
    rail_to_car = object_model.get_articulation("rail_to_car")
    car_to_door = object_model.get_articulation("car_to_door")

    ctx.check(
        "car uses a vertical prismatic guide",
        rail_to_car.articulation_type == ArticulationType.PRISMATIC
        and tuple(rail_to_car.axis) == (0.0, 0.0, 1.0),
        details=f"type={rail_to_car.articulation_type}, axis={rail_to_car.axis}",
    )
    ctx.check(
        "door uses a horizontal prismatic guide",
        car_to_door.articulation_type == ArticulationType.PRISMATIC
        and tuple(car_to_door.axis) == (1.0, 0.0, 0.0),
        details=f"type={car_to_door.articulation_type}, axis={car_to_door.axis}",
    )
    ctx.expect_within(
        car,
        rail_system,
        axes="x",
        margin=0.02,
        name="car remains between the two vertical rail columns",
    )
    ctx.expect_overlap(
        car,
        rail_system,
        axes="y",
        min_overlap=0.04,
        name="car guide shoes align with the rear rail system",
    )
    ctx.expect_overlap(
        door_panel,
        car,
        axes="z",
        elem_a="top_roller_0",
        elem_b="upper_door_guide",
        min_overlap=0.02,
        name="door rollers sit under the horizontal entrance guide",
    )
    ctx.expect_contact(
        door_panel,
        car,
        elem_a="top_roller_0",
        elem_b="upper_door_guide",
        contact_tol=0.001,
        name="door roller bears on the entrance guide",
    )

    car_rest = ctx.part_world_position(car)
    with ctx.pose({rail_to_car: 2.0}):
        car_raised = ctx.part_world_position(car)
    ctx.check(
        "car raises along the columns",
        car_rest is not None and car_raised is not None and car_raised[2] > car_rest[2] + 1.9,
        details=f"rest={car_rest}, raised={car_raised}",
    )

    door_rest = ctx.part_world_position(door_panel)
    with ctx.pose({car_to_door: 0.60}):
        door_shifted = ctx.part_world_position(door_panel)
    ctx.check(
        "door slides sideways on the entrance rail",
        door_rest is not None and door_shifted is not None and door_shifted[0] > door_rest[0] + 0.55,
        details=f"rest={door_rest}, shifted={door_shifted}",
    )

    return ctx.report()


object_model = build_object_model()
