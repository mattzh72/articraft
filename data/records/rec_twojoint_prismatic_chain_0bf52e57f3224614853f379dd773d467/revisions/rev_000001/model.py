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
    model = ArticulatedObject(name="compact_linear_extension_unit")

    body_mat = model.material("dark_hard_anodized", rgba=(0.13, 0.14, 0.15, 1.0))
    rail_mat = model.material("ground_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    carriage_mat = model.material("clear_anodized_aluminum", rgba=(0.55, 0.57, 0.59, 1.0))
    nose_mat = model.material("satin_titanium", rgba=(0.72, 0.70, 0.66, 1.0))
    screw_mat = model.material("black_oxide_screws", rgba=(0.015, 0.014, 0.012, 1.0))
    pad_mat = model.material("matte_black_end_pads", rgba=(0.045, 0.048, 0.050, 1.0))

    outer = model.part("outer_body")
    outer.visual(
        Box((0.82, 0.24, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=body_mat,
        name="base_plate",
    )
    outer.visual(
        Box((0.78, 0.028, 0.13)),
        origin=Origin(xyz=(0.0, 0.106, 0.0925)),
        material=body_mat,
        name="side_wall_0",
    )
    outer.visual(
        Box((0.78, 0.028, 0.13)),
        origin=Origin(xyz=(0.0, -0.106, 0.0925)),
        material=body_mat,
        name="side_wall_1",
    )
    outer.visual(
        Box((0.72, 0.034, 0.080)),
        origin=Origin(xyz=(0.0, 0.075, 0.105)),
        material=rail_mat,
        name="inner_way_0",
    )
    outer.visual(
        Box((0.72, 0.034, 0.080)),
        origin=Origin(xyz=(0.0, -0.075, 0.105)),
        material=rail_mat,
        name="inner_way_1",
    )
    outer.visual(
        Box((0.72, 0.045, 0.018)),
        origin=Origin(xyz=(-0.03, 0.086, 0.160)),
        material=body_mat,
        name="upper_cap_0",
    )
    outer.visual(
        Box((0.72, 0.045, 0.018)),
        origin=Origin(xyz=(-0.03, -0.086, 0.160)),
        material=body_mat,
        name="upper_cap_1",
    )
    outer.visual(
        Box((0.035, 0.20, 0.10)),
        origin=Origin(xyz=(-0.3925, 0.0, 0.095)),
        material=pad_mat,
        name="rear_stop",
    )
    for index, y in enumerate((0.106, -0.106)):
        outer.visual(
            Box((0.040, 0.050, 0.125)),
            origin=Origin(xyz=(0.392, y, 0.095)),
            material=pad_mat,
            name=f"front_cheek_{index}",
        )
    for index, (x, y) in enumerate(
        ((-0.300, 0.088), (-0.045, 0.088), (0.210, 0.088), (-0.300, -0.088), (-0.045, -0.088), (0.210, -0.088))
    ):
        outer.visual(
            Cylinder(radius=0.0085, length=0.005),
            origin=Origin(xyz=(x, y, 0.1705)),
            material=screw_mat,
            name=f"top_screw_{index}",
        )
    for index, x in enumerate((-0.300, -0.080, 0.140, 0.315)):
        outer.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, 0.122, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_mat,
            name=f"side_screw_0_{index}",
        )
        outer.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, -0.122, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_mat,
            name=f"side_screw_1_{index}",
        )
    for index, (x, y) in enumerate(((-0.305, 0.088), (0.305, 0.088), (-0.305, -0.088), (0.305, -0.088))):
        outer.visual(
            Box((0.105, 0.050, 0.014)),
            origin=Origin(xyz=(x, y, -0.004)),
            material=pad_mat,
            name=f"ground_foot_{index}",
        )
    outer.inertial = Inertial.from_geometry(
        Box((0.82, 0.24, 0.17)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    carriage = model.part("carriage_sleeve")
    carriage.visual(
        Box((0.56, 0.116, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=carriage_mat,
        name="bottom_plate",
    )
    carriage.visual(
        Box((0.56, 0.116, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
        material=carriage_mat,
        name="top_plate",
    )
    carriage.visual(
        Box((0.56, 0.018, 0.074)),
        origin=Origin(xyz=(0.0, 0.049, 0.105)),
        material=carriage_mat,
        name="side_plate_0",
    )
    carriage.visual(
        Box((0.56, 0.018, 0.074)),
        origin=Origin(xyz=(0.0, -0.049, 0.105)),
        material=carriage_mat,
        name="side_plate_1",
    )
    carriage.visual(
        Box((0.54, 0.010, 0.070)),
        origin=Origin(xyz=(-0.010, 0.053, 0.105)),
        material=rail_mat,
        name="wear_face_0",
    )
    carriage.visual(
        Box((0.54, 0.010, 0.070)),
        origin=Origin(xyz=(-0.010, -0.053, 0.105)),
        material=rail_mat,
        name="wear_face_1",
    )
    carriage.visual(
        Box((0.035, 0.014, 0.060)),
        origin=Origin(xyz=(0.2975, 0.051, 0.112)),
        material=carriage_mat,
        name="front_lip_0",
    )
    carriage.visual(
        Box((0.035, 0.014, 0.060)),
        origin=Origin(xyz=(0.2975, -0.051, 0.112)),
        material=carriage_mat,
        name="front_lip_1",
    )
    carriage.visual(
        Box((0.035, 0.100, 0.010)),
        origin=Origin(xyz=(0.2975, 0.0, 0.136)),
        material=carriage_mat,
        name="front_lip_top",
    )
    carriage.visual(
        Box((0.030, 0.116, 0.070)),
        origin=Origin(xyz=(-0.275, 0.0, 0.105)),
        material=pad_mat,
        name="rear_end_pad",
    )
    for index, (x, y) in enumerate(((-0.185, 0.034), (0.055, 0.034), (0.205, 0.034), (-0.185, -0.034), (0.055, -0.034), (0.205, -0.034))):
        carriage.visual(
            Cylinder(radius=0.0062, length=0.004),
            origin=Origin(xyz=(x, y, 0.1485)),
            material=screw_mat,
            name=f"carriage_screw_{index}",
        )
    carriage.inertial = Inertial.from_geometry(
        Box((0.56, 0.116, 0.085)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
    )

    nose = model.part("nose_stage")
    nose.visual(
        Box((0.56, 0.052, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.097)),
        material=nose_mat,
        name="nose_bar",
    )
    nose.visual(
        Box((0.045, 0.064, 0.038)),
        origin=Origin(xyz=(0.322, 0.0, 0.101)),
        material=nose_mat,
        name="terminal_pad",
    )
    nose.visual(
        Box((0.300, 0.010, 0.004)),
        origin=Origin(xyz=(0.040, 0.0, 0.114)),
        material=rail_mat,
        name="top_land",
    )
    nose.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.323, 0.0, 0.122)),
        material=screw_mat,
        name="terminal_screw",
    )
    nose.inertial = Inertial.from_geometry(
        Box((0.60, 0.065, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.040, 0.0, 0.101)),
    )

    model.articulation(
        "outer_to_carriage",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=240.0, velocity=0.18, lower=0.0, upper=0.22),
    )
    model.articulation(
        "carriage_to_nose",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=nose,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.16, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_body")
    carriage = object_model.get_part("carriage_sleeve")
    nose = object_model.get_part("nose_stage")
    outer_slide = object_model.get_articulation("outer_to_carriage")
    nose_slide = object_model.get_articulation("carriage_to_nose")

    ctx.expect_contact(
        carriage,
        outer,
        elem_a="wear_face_0",
        elem_b="inner_way_0",
        name="carriage rides on the upper body way",
    )
    ctx.expect_contact(
        carriage,
        outer,
        elem_a="wear_face_1",
        elem_b="inner_way_1",
        name="carriage is laterally captured by the body way",
    )
    ctx.expect_overlap(
        carriage,
        outer,
        axes="x",
        elem_a="wear_face_0",
        elem_b="inner_way_0",
        min_overlap=0.50,
        name="carriage has retained length inside the outer body",
    )
    ctx.expect_gap(
        carriage,
        nose,
        axis="z",
        positive_elem="top_plate",
        negative_elem="nose_bar",
        min_gap=0.015,
        name="nose clears the carriage top plate",
    )
    ctx.expect_gap(
        carriage,
        nose,
        axis="y",
        positive_elem="side_plate_0",
        negative_elem="nose_bar",
        min_gap=0.010,
        name="nose clears the upper side wall",
    )
    ctx.expect_gap(
        nose,
        carriage,
        axis="y",
        positive_elem="nose_bar",
        negative_elem="side_plate_1",
        min_gap=0.010,
        name="nose clears the lower side wall",
    )
    ctx.expect_gap(
        nose,
        carriage,
        axis="z",
        positive_elem="nose_bar",
        negative_elem="bottom_plate",
        max_gap=0.0005,
        max_penetration=0.0,
        name="nose is supported on the carriage floor",
    )
    ctx.expect_overlap(
        nose,
        carriage,
        axes="x",
        elem_a="nose_bar",
        elem_b="bottom_plate",
        min_overlap=0.50,
        name="collapsed nose stage remains deeply nested",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_nose_pos = ctx.part_world_position(nose)
    with ctx.pose({outer_slide: 0.22, nose_slide: 0.18}):
        ctx.expect_contact(
            carriage,
            outer,
            elem_a="wear_face_0",
            elem_b="inner_way_0",
            name="extended carriage remains guided by the body way",
        )
        ctx.expect_overlap(
            carriage,
            outer,
            axes="x",
            elem_a="wear_face_0",
            elem_b="inner_way_0",
            min_overlap=0.25,
            name="extended carriage retains insertion in the body",
        )
        ctx.expect_gap(
            carriage,
            nose,
            axis="z",
            positive_elem="top_plate",
            negative_elem="nose_bar",
            min_gap=0.015,
            name="extended nose clears the carriage cover",
        )
        ctx.expect_gap(
            nose,
            carriage,
            axis="z",
            positive_elem="nose_bar",
            negative_elem="bottom_plate",
            max_gap=0.0005,
            max_penetration=0.0,
            name="extended nose remains supported on the carriage floor",
        )
        ctx.expect_overlap(
            nose,
            carriage,
            axes="x",
            elem_a="nose_bar",
            elem_b="bottom_plate",
            min_overlap=0.30,
            name="extended nose retains insertion in the carriage sleeve",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)
        extended_nose_pos = ctx.part_world_position(nose)

    ctx.check(
        "serial slides extend along one aligned axis",
        rest_carriage_pos is not None
        and rest_nose_pos is not None
        and extended_carriage_pos is not None
        and extended_nose_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.20
        and extended_nose_pos[0] > rest_nose_pos[0] + 0.38
        and abs(extended_nose_pos[1] - rest_nose_pos[1]) < 1e-6
        and abs(extended_nose_pos[2] - rest_nose_pos[2]) < 1e-6,
        details=(
            f"rest_carriage={rest_carriage_pos}, extended_carriage={extended_carriage_pos}, "
            f"rest_nose={rest_nose_pos}, extended_nose={extended_nose_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
