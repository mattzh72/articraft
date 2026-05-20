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
    model = ArticulatedObject(name="mast_lift_wrist")

    powder_black = model.material("powder_black", rgba=(0.04, 0.045, 0.05, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    polished_rail = model.material("polished_rail", rgba=(0.72, 0.76, 0.78, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.05, 0.22, 0.42, 1.0))
    bearing_gray = model.material("bearing_gray", rgba=(0.42, 0.45, 0.46, 1.0))
    hinge_pin = model.material("hinge_pin", rgba=(0.82, 0.78, 0.62, 1.0))
    nose_orange = model.material("nose_orange", rgba=(0.95, 0.42, 0.08, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.013, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.40, 0.32, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=powder_black,
        name="base_plate",
    )
    mast.visual(
        Box((0.08, 0.12, 1.05)),
        origin=Origin(xyz=(-0.055, 0.0, 0.575)),
        material=dark_steel,
        name="spine",
    )
    mast.visual(
        Box((0.16, 0.18, 0.04)),
        origin=Origin(xyz=(-0.015, 0.0, 0.09)),
        material=dark_steel,
        name="lower_rail_block",
    )
    mast.visual(
        Box((0.16, 0.18, 0.04)),
        origin=Origin(xyz=(-0.015, 0.0, 1.12)),
        material=dark_steel,
        name="upper_rail_block",
    )
    mast.visual(
        Cylinder(radius=0.011, length=1.02),
        origin=Origin(xyz=(0.025, -0.055, 0.595)),
        material=polished_rail,
        name="guide_rail_0",
    )
    mast.visual(
        Cylinder(radius=0.011, length=1.02),
        origin=Origin(xyz=(0.025, 0.055, 0.595)),
        material=polished_rail,
        name="guide_rail_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.055, 0.18, 0.18)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=carriage_blue,
        name="rear_plate",
    )
    carriage.visual(
        Box((0.105, 0.13, 0.13)),
        origin=Origin(xyz=(0.083, 0.0, 0.0)),
        material=carriage_blue,
        name="carriage_block",
    )
    for side, y in enumerate((-0.055, 0.055)):
        for level, z in enumerate((-0.058, 0.058)):
            carriage.visual(
                Box((0.020, 0.025, 0.045)),
                origin=Origin(xyz=(-0.006, y, z)),
                material=bearing_gray,
                name=f"slide_shoe_{side}_{level}",
            )
    carriage.visual(
        Box((0.060, 0.026, 0.075)),
        origin=Origin(xyz=(0.150, -0.065, 0.0)),
        material=carriage_blue,
        name="wrist_cheek_0",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.009),
        origin=Origin(xyz=(0.150, -0.0806, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_pin,
        name="pin_cap_0",
    )
    carriage.visual(
        Box((0.060, 0.026, 0.075)),
        origin=Origin(xyz=(0.150, 0.065, 0.0)),
        material=carriage_blue,
        name="wrist_cheek_1",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.009),
        origin=Origin(xyz=(0.150, 0.0806, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_pin,
        name="pin_cap_1",
    )

    nose = model.part("nose_bracket")
    nose.visual(
        Cylinder(radius=0.017, length=0.104),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_pin,
        name="hinge_barrel",
    )
    nose.visual(
        Box((0.160, 0.055, 0.035)),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=nose_orange,
        name="nose_arm",
    )
    nose.visual(
        Box((0.025, 0.080, 0.055)),
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        material=nose_orange,
        name="nose_face",
    )
    nose.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.1835, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="front_pad",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.052, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.65),
    )
    model.articulation(
        "carriage_to_nose",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=nose,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.6, lower=-0.55, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    nose = object_model.get_part("nose_bracket")
    lift = object_model.get_articulation("mast_to_carriage")
    wrist = object_model.get_articulation("carriage_to_nose")

    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="rear_plate",
        negative_elem="guide_rail_0",
        min_gap=0.006,
        max_gap=0.020,
        name="carriage rides just in front of the guide rail",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a="rear_plate",
        elem_b="guide_rail_0",
        min_overlap=0.15,
        name="carriage remains engaged on the vertical mast at the low stop",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.65}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="rear_plate",
            elem_b="guide_rail_0",
            min_overlap=0.15,
            name="carriage remains engaged on the vertical mast at full lift",
        )
    ctx.check(
        "lift joint translates carriage upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.60,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_face = ctx.part_element_world_aabb(nose, elem="nose_face")
    with ctx.pose({wrist: 0.85}):
        raised_face = ctx.part_element_world_aabb(nose, elem="nose_face")
    rest_face_z = None if rest_face is None else (rest_face[0][2] + rest_face[1][2]) / 2.0
    raised_face_z = None if raised_face is None else (raised_face[0][2] + raised_face[1][2]) / 2.0
    ctx.check(
        "wrist hinge pitches the nose bracket upward",
        rest_face_z is not None and raised_face_z is not None and raised_face_z > rest_face_z + 0.08,
        details=f"rest_z={rest_face_z}, raised_z={raised_face_z}",
    )
    ctx.expect_gap(
        nose,
        carriage,
        axis="y",
        positive_elem="hinge_barrel",
        negative_elem="wrist_cheek_0",
        min_gap=0.0,
        max_gap=0.002,
        name="barrel fits against lower side cheek",
    )
    ctx.expect_gap(
        carriage,
        nose,
        axis="y",
        positive_elem="wrist_cheek_1",
        negative_elem="hinge_barrel",
        min_gap=0.0,
        max_gap=0.002,
        name="barrel fits against upper side cheek",
    )

    return ctx.report()


object_model = build_object_model()
