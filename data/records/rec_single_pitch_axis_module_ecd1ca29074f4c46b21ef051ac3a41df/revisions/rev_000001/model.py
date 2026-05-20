from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motorized_tilt_cradle")

    axis_z = 0.52

    graphite = Material("powder_coated_graphite", color=(0.08, 0.085, 0.09, 1.0))
    dark_plate = Material("anodized_dark_plate", color=(0.12, 0.15, 0.17, 1.0))
    brushed = Material("brushed_aluminum", color=(0.72, 0.74, 0.72, 1.0))
    motor_blue = Material("servo_blue", color=(0.08, 0.20, 0.45, 1.0))
    rubber = Material("matte_rubber", color=(0.015, 0.016, 0.017, 1.0))
    bronze = Material("oilite_bronze", color=(0.55, 0.38, 0.16, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.72, 0.38, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=graphite,
        name="base_foot",
    )
    support.visual(
        Box((0.17, 0.13, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=graphite,
        name="tower_body",
    )
    support.visual(
        Box((0.62, 0.12, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=graphite,
        name="top_crosshead",
    )

    # Four-bar bearing frames leave a true clearance hole around the rotating
    # trunnion instead of hiding a shaft inside a solid block.
    bearing_specs = (
        ("left_upper_bearing", -0.29, Box((0.06, 0.16, 0.045)), (0.0, axis_z + 0.0675), graphite),
        ("left_lower_bearing", -0.29, Box((0.06, 0.16, 0.045)), (0.0, axis_z - 0.0675), graphite),
        ("left_front_bearing", -0.29, Box((0.06, 0.035, 0.09)), (0.0625, axis_z), graphite),
        ("left_rear_bearing", -0.29, Box((0.06, 0.035, 0.09)), (-0.0625, axis_z), graphite),
        ("left_top_bushing", -0.29, Box((0.062, 0.092, 0.012)), (0.0, axis_z + 0.032), bronze),
        ("left_bottom_bushing", -0.29, Box((0.062, 0.092, 0.012)), (0.0, axis_z - 0.032), bronze),
        ("left_front_bushing", -0.29, Box((0.062, 0.012, 0.092)), (0.032, axis_z), bronze),
        ("left_rear_bushing", -0.29, Box((0.062, 0.012, 0.092)), (-0.032, axis_z), bronze),
        ("right_upper_bearing", 0.29, Box((0.06, 0.16, 0.045)), (0.0, axis_z + 0.0675), graphite),
        ("right_lower_bearing", 0.29, Box((0.06, 0.16, 0.045)), (0.0, axis_z - 0.0675), graphite),
        ("right_front_bearing", 0.29, Box((0.06, 0.035, 0.09)), (0.0625, axis_z), graphite),
        ("right_rear_bearing", 0.29, Box((0.06, 0.035, 0.09)), (-0.0625, axis_z), graphite),
        ("right_top_bushing", 0.29, Box((0.062, 0.092, 0.012)), (0.0, axis_z + 0.032), bronze),
        ("right_bottom_bushing", 0.29, Box((0.062, 0.092, 0.012)), (0.0, axis_z - 0.032), bronze),
        ("right_front_bushing", 0.29, Box((0.062, 0.012, 0.092)), (0.032, axis_z), bronze),
        ("right_rear_bushing", 0.29, Box((0.062, 0.012, 0.092)), (-0.032, axis_z), bronze),
    )
    for name, x, geometry, (y, z), material in bearing_specs:
        support.visual(
            geometry,
            origin=Origin(xyz=(x, y, z)),
            material=material,
            name=name,
        )

    support.visual(
        Box((0.055, 0.11, 0.16)),
        origin=Origin(xyz=(0.3475, 0.0, 0.50)),
        material=graphite,
        name="motor_stanchion",
    )
    support.visual(
        Cylinder(radius=0.072, length=0.17),
        origin=Origin(xyz=(0.455, 0.0, axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_blue,
        name="servo_can",
    )
    support.visual(
        Cylinder(radius=0.086, length=0.024),
        origin=Origin(xyz=(0.367, 0.0, axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="motor_flange",
    )
    support.visual(
        Box((0.16, 0.035, 0.03)),
        origin=Origin(xyz=(0.44, -0.086, 0.455)),
        material=rubber,
        name="cable_gland",
    )

    plate = model.part("equipment_plate")
    plate.visual(
        Box((0.46, 0.34, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_plate,
        name="plate_panel",
    )
    plate.visual(
        Box((0.34, 0.22, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=rubber,
        name="equipment_deck",
    )
    for idx, x in enumerate((-0.18, 0.18)):
        plate.visual(
            Box((0.035, 0.305, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=brushed,
            name=f"mounting_rail_{idx}",
        )
    plate.visual(
        Cylinder(radius=0.055, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="central_hub",
    )
    plate.visual(
        Cylinder(radius=0.026, length=0.64),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="trunnion_shaft",
    )
    plate.visual(
        Box((0.28, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, 0.155, -0.019)),
        material=brushed,
        name="front_balance_rib",
    )
    plate.visual(
        Box((0.28, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, -0.155, -0.019)),
        material=brushed,
        name="rear_balance_rib",
    )

    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=plate,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=-0.70, upper=0.70),
        motion_properties=MotionProperties(damping=1.8, friction=0.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    plate = object_model.get_part("equipment_plate")
    tilt = object_model.get_articulation("tilt_axis")

    ctx.check(
        "one clean revolute tilt axis",
        len(object_model.articulations) == 1
        and tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations!r}",
    )

    ctx.expect_gap(
        plate,
        support,
        axis="x",
        positive_elem="plate_panel",
        negative_elem="left_lower_bearing",
        min_gap=0.02,
        name="plate clears left bearing frame",
    )
    ctx.expect_gap(
        support,
        plate,
        axis="x",
        positive_elem="right_lower_bearing",
        negative_elem="plate_panel",
        min_gap=0.02,
        name="plate clears right bearing frame",
    )
    ctx.expect_overlap(
        plate,
        support,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="left_upper_bearing",
        min_overlap=0.05,
        name="shaft spans left bearing",
    )
    ctx.expect_overlap(
        plate,
        support,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="right_upper_bearing",
        min_overlap=0.05,
        name="shaft spans right bearing",
    )
    ctx.expect_contact(
        support,
        plate,
        elem_a="right_top_bushing",
        elem_b="trunnion_shaft",
        contact_tol=0.001,
        name="upper bushing has running clearance",
    )
    ctx.expect_contact(
        plate,
        support,
        elem_a="trunnion_shaft",
        elem_b="right_bottom_bushing",
        contact_tol=0.001,
        name="lower bushing has running clearance",
    )

    rest_deck = ctx.part_element_world_aabb(plate, elem="equipment_deck")
    with ctx.pose({tilt: 0.65}):
        tilted_deck = ctx.part_element_world_aabb(plate, elem="equipment_deck")
    ctx.check(
        "plate visibly tilts about shaft",
        rest_deck is not None
        and tilted_deck is not None
        and (tilted_deck[1][2] - tilted_deck[0][2]) > (rest_deck[1][2] - rest_deck[0][2]) + 0.10,
        details=f"rest_deck={rest_deck}, tilted_deck={tilted_deck}",
    )

    return ctx.report()


object_model = build_object_model()
