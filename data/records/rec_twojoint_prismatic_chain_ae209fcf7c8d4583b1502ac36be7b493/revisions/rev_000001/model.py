from __future__ import annotations

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
    model = ArticulatedObject(name="side_wall_double_carriage")

    anodized_black = model.material("anodized_black", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_plate = model.material("dark_plate", rgba=(0.16, 0.17, 0.18, 1.0))
    linear_steel = model.material("linear_steel", rgba=(0.70, 0.72, 0.73, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.08, 0.24, 0.48, 1.0))
    slider_orange = model.material("slider_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.03, 0.03, 0.035, 1.0))
    screw_dark = model.material("screw_dark", rgba=(0.04, 0.045, 0.05, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.90, 0.018, 0.36)),
        origin=Origin(),
        material=dark_plate,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.88, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, -0.012, -0.155)),
        material=anodized_black,
        name="lower_mount_lip",
    )
    side_plate.visual(
        Box((0.88, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, -0.012, 0.155)),
        material=anodized_black,
        name="upper_mount_lip",
    )
    side_plate.visual(
        Cylinder(radius=0.011, length=0.82),
        origin=Origin(xyz=(0.0, 0.020, -0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=linear_steel,
        name="lower_rail",
    )
    side_plate.visual(
        Cylinder(radius=0.011, length=0.82),
        origin=Origin(xyz=(0.0, 0.020, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=linear_steel,
        name="upper_rail",
    )
    for z, rail_name in ((-0.080, "lower_rail"), (0.080, "upper_rail")):
        for x in (-0.30, 0.0, 0.30):
            side_plate.visual(
                Box((0.040, 0.018, 0.035)),
                origin=Origin(xyz=(x, 0.012, z)),
                material=anodized_black,
                name=f"{rail_name}_stand_{int((x + 0.30) * 100):02d}",
            )

    for x, name in ((-0.425, "start_stop"), (0.425, "end_stop")):
        side_plate.visual(
            Box((0.028, 0.052, 0.245)),
            origin=Origin(xyz=(x, 0.022, 0.0)),
            material=anodized_black,
            name=name,
        )
        side_plate.visual(
            Box((0.016, 0.020, 0.150)),
            origin=Origin(xyz=(x, 0.054, 0.0)),
            material=rubber_black,
            name=f"{name}_bumper",
        )
    for x in (-0.36, -0.12, 0.12, 0.36):
        for z in (-0.135, 0.135):
            side_plate.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(x, 0.012, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=screw_dark,
                name=f"plate_screw_{x:+.2f}_{z:+.2f}",
            )
    side_plate.inertial = Inertial.from_geometry(
        Box((0.92, 0.08, 0.38)),
        mass=8.0,
    )

    first_carriage = model.part("first_carriage")
    first_carriage.visual(
        Box((0.205, 0.022, 0.205)),
        origin=Origin(),
        material=carriage_blue,
        name="carriage_body",
    )
    first_carriage.visual(
        Box((0.175, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, -0.012, -0.080)),
        material=anodized_black,
        name="lower_bearing_shoe",
    )
    first_carriage.visual(
        Box((0.155, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -0.025, -0.080)),
        material=rubber_black,
        name="lower_bearing_shoe_liner",
    )
    first_carriage.visual(
        Box((0.175, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, -0.012, 0.080)),
        material=anodized_black,
        name="upper_bearing_shoe",
    )
    first_carriage.visual(
        Box((0.155, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -0.025, 0.080)),
        material=rubber_black,
        name="upper_bearing_shoe_liner",
    )
    first_carriage.visual(
        Box((0.315, 0.016, 0.040)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=carriage_blue,
        name="front_crossbar",
    )
    for x in (-0.155, 0.155):
        first_carriage.visual(
            Box((0.020, 0.034, 0.135)),
            origin=Origin(xyz=(x, 0.014, 0.0)),
            material=carriage_blue,
            name=f"rail_bridge_{x:+.2f}",
        )
    first_carriage.visual(
        Cylinder(radius=0.006, length=0.36),
        origin=Origin(xyz=(0.0, 0.027, -0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=linear_steel,
        name="front_lower_rail",
    )
    first_carriage.visual(
        Cylinder(radius=0.006, length=0.36),
        origin=Origin(xyz=(0.0, 0.027, 0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=linear_steel,
        name="front_upper_rail",
    )
    for x in (-0.062, 0.062):
        for z in (-0.062, 0.062):
            first_carriage.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, 0.014, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=screw_dark,
                name=f"body_screw_{x:+.2f}_{z:+.2f}",
            )
    first_carriage.inertial = Inertial.from_geometry(
        Box((0.34, 0.075, 0.23)),
        mass=2.3,
    )

    second_slider = model.part("second_slider")
    second_slider.visual(
        Box((0.115, 0.026, 0.135)),
        origin=Origin(),
        material=slider_orange,
        name="slider_body",
    )
    second_slider.visual(
        Box((0.105, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, -0.024, -0.046)),
        material=anodized_black,
        name="rear_lower_bearing",
    )
    second_slider.visual(
        Box((0.090, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.034, -0.046)),
        material=rubber_black,
        name="rear_lower_bearing_liner",
    )
    second_slider.visual(
        Box((0.105, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, -0.024, 0.046)),
        material=anodized_black,
        name="rear_upper_bearing",
    )
    second_slider.visual(
        Box((0.090, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.034, 0.046)),
        material=rubber_black,
        name="rear_upper_bearing_liner",
    )
    second_slider.visual(
        Box((0.100, 0.012, 0.105)),
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        material=slider_orange,
        name="tool_face",
    )
    second_slider.visual(
        Box((0.080, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material=anodized_black,
        name="payload_boss",
    )
    for x in (-0.035, 0.035):
        for z in (-0.038, 0.038):
            second_slider.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(x, 0.028, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=screw_dark,
                name=f"face_screw_{x:+.2f}_{z:+.2f}",
            )
    second_slider.inertial = Inertial.from_geometry(
        Box((0.13, 0.07, 0.15)),
        mass=1.1,
    )

    model.articulation(
        "plate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=side_plate,
        child=first_carriage,
        origin=Origin(xyz=(-0.220, 0.056, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.55, lower=0.0, upper=0.36),
    )
    model.articulation(
        "carriage_to_slider",
        ArticulationType.PRISMATIC,
        parent=first_carriage,
        child=second_slider,
        origin=Origin(xyz=(-0.070, 0.069, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.45, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    first_carriage = object_model.get_part("first_carriage")
    second_slider = object_model.get_part("second_slider")
    stage_one = object_model.get_articulation("plate_to_carriage")
    stage_two = object_model.get_articulation("carriage_to_slider")

    ctx.expect_gap(
        first_carriage,
        side_plate,
        axis="y",
        positive_elem="lower_bearing_shoe",
        negative_elem="lower_rail",
        min_gap=0.001,
        max_gap=0.006,
        name="first carriage clears lower wall rail",
    )
    ctx.expect_gap(
        first_carriage,
        side_plate,
        axis="y",
        positive_elem="upper_bearing_shoe",
        negative_elem="upper_rail",
        min_gap=0.001,
        max_gap=0.006,
        name="first carriage clears upper wall rail",
    )
    ctx.expect_overlap(
        first_carriage,
        side_plate,
        axes="xz",
        elem_a="lower_bearing_shoe",
        elem_b="lower_rail",
        min_overlap=0.020,
        name="first carriage lower shoe tracks rail",
    )
    ctx.expect_overlap(
        first_carriage,
        side_plate,
        axes="xz",
        elem_a="upper_bearing_shoe",
        elem_b="upper_rail",
        min_overlap=0.020,
        name="first carriage upper shoe tracks rail",
    )
    ctx.expect_contact(
        second_slider,
        first_carriage,
        elem_a="rear_lower_bearing_liner",
        elem_b="front_lower_rail",
        contact_tol=0.001,
        name="second slider bearing contacts carried lower rail",
    )
    ctx.expect_overlap(
        second_slider,
        first_carriage,
        axes="xz",
        elem_a="rear_lower_bearing",
        elem_b="front_lower_rail",
        min_overlap=0.010,
        name="second slider remains on carried lower rail",
    )

    rest_first = ctx.part_world_position(first_carriage)
    rest_second = ctx.part_world_position(second_slider)
    with ctx.pose({stage_one: 0.36, stage_two: 0.18}):
        ctx.expect_overlap(
            first_carriage,
            side_plate,
            axes="x",
            elem_a="lower_bearing_shoe",
            elem_b="lower_rail",
            min_overlap=0.120,
            name="extended first carriage retains rail engagement",
        )
        ctx.expect_overlap(
            second_slider,
            first_carriage,
            axes="x",
            elem_a="rear_lower_bearing",
            elem_b="front_lower_rail",
            min_overlap=0.080,
            name="extended second slider retains rail engagement",
        )
        extended_first = ctx.part_world_position(first_carriage)
        extended_second = ctx.part_world_position(second_slider)

    ctx.check(
        "first stage translates along wall",
        rest_first is not None
        and extended_first is not None
        and extended_first[0] > rest_first[0] + 0.30
        and abs(extended_first[1] - rest_first[1]) < 1e-6
        and abs(extended_first[2] - rest_first[2]) < 1e-6,
        details=f"rest={rest_first}, extended={extended_first}",
    )
    ctx.check(
        "second stage compounds first travel",
        rest_second is not None
        and extended_second is not None
        and extended_first is not None
        and rest_first is not None
        and extended_second[0] - rest_second[0] > 0.50
        and (extended_second[0] - rest_second[0]) > (extended_first[0] - rest_first[0]),
        details=f"rest={rest_second}, extended={extended_second}",
    )

    return ctx.report()


object_model = build_object_model()
