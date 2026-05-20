from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="mast_and_wrist_slide")

    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.73, 0.74, 1.0))
    rail_chrome = model.material("rail_chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    carriage_blue = model.material("carriage_blue", rgba=(0.05, 0.25, 0.58, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.015, 0.015, 0.018, 1.0))
    output_orange = model.material("output_orange", rgba=(0.95, 0.45, 0.10, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.55, 0.36, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="base_plate",
    )
    mast.visual(
        Box((0.08, 0.26, 1.00)),
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
        material=dark_steel,
        name="backbone",
    )
    mast.visual(
        Box((0.12, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=brushed_steel,
        name="lower_rail_clamp",
    )
    mast.visual(
        Box((0.12, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        material=brushed_steel,
        name="upper_rail_clamp",
    )
    mast.visual(
        Cylinder(radius=0.012, length=0.92),
        origin=Origin(xyz=(0.052, -0.085, 0.56)),
        material=rail_chrome,
        name="rail_0",
    )
    mast.visual(
        Cylinder(radius=0.012, length=0.92),
        origin=Origin(xyz=(0.052, 0.085, 0.56)),
        material=rail_chrome,
        name="rail_1",
    )
    mast.visual(
        Cylinder(radius=0.007, length=0.88),
        origin=Origin(xyz=(0.050, 0.0, 0.56)),
        material=brushed_steel,
        name="lead_screw",
    )
    mast.visual(
        Box((0.16, 0.04, 0.30)),
        origin=Origin(xyz=(-0.08, -0.10, 0.18), rpy=(0.0, 0.55, 0.0)),
        material=dark_steel,
        name="brace_0",
    )
    mast.visual(
        Box((0.16, 0.04, 0.30)),
        origin=Origin(xyz=(-0.08, 0.10, 0.18), rpy=(0.0, 0.55, 0.0)),
        material=dark_steel,
        name="brace_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.076, 0.245, 0.180)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=carriage_blue,
        name="body",
    )
    carriage.visual(
        Box((0.010, 0.052, 0.160)),
        origin=Origin(xyz=(0.005, -0.085, 0.0)),
        material=rubber_black,
        name="bearing_pad_0",
    )
    carriage.visual(
        Box((0.010, 0.052, 0.160)),
        origin=Origin(xyz=(0.005, 0.085, 0.0)),
        material=rubber_black,
        name="bearing_pad_1",
    )
    carriage.visual(
        Box((0.054, 0.030, 0.095)),
        origin=Origin(xyz=(0.103, -0.120, 0.0)),
        material=carriage_blue,
        name="clevis_ear_0",
    )
    carriage.visual(
        Box((0.054, 0.030, 0.095)),
        origin=Origin(xyz=(0.103, 0.120, 0.0)),
        material=carriage_blue,
        name="clevis_ear_1",
    )

    output_face = model.part("output_face")
    output_face.visual(
        Cylinder(radius=0.016, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_pin",
    )
    output_face.visual(
        Box((0.040, 0.100, 0.050)),
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
        material=output_orange,
        name="hub",
    )
    output_face.visual(
        Box((0.024, 0.180, 0.140)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=output_orange,
        name="face_plate",
    )
    output_face.visual(
        Box((0.008, 0.150, 0.010)),
        origin=Origin(xyz=(0.071, 0.0, 0.047)),
        material=rubber_black,
        name="top_grip",
    )
    output_face.visual(
        Box((0.008, 0.150, 0.010)),
        origin=Origin(xyz=(0.071, 0.0, -0.047)),
        material=rubber_black,
        name="bottom_grip",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.065, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.55),
    )
    model.articulation(
        "carriage_to_face",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=output_face,
        origin=Origin(xyz=(0.103, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.80, upper=0.80),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    output_face = object_model.get_part("output_face")
    slide = object_model.get_articulation("mast_to_carriage")
    wrist = object_model.get_articulation("carriage_to_face")

    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="body",
        negative_elem="rail_0",
        min_gap=0.0,
        max_gap=0.003,
        name="carriage rides just proud of rail 0",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="body",
        negative_elem="rail_1",
        min_gap=0.0,
        max_gap=0.003,
        name="carriage rides just proud of rail 1",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="yz",
        elem_a="body",
        elem_b="rail_0",
        min_overlap=0.020,
        name="carriage wraps rail 0 in projection",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="yz",
        elem_a="body",
        elem_b="rail_1",
        min_overlap=0.020,
        name="carriage wraps rail 1 in projection",
    )

    ctx.expect_gap(
        carriage,
        output_face,
        axis="y",
        positive_elem="clevis_ear_1",
        negative_elem="hinge_pin",
        min_gap=0.0,
        max_gap=0.001,
        name="hinge pin clears positive clevis ear",
    )
    ctx.expect_gap(
        output_face,
        carriage,
        axis="y",
        positive_elem="hinge_pin",
        negative_elem="clevis_ear_0",
        min_gap=0.0,
        max_gap=0.001,
        name="hinge pin clears negative clevis ear",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_face_pos = ctx.part_world_position(output_face)
    with ctx.pose({slide: 0.55}):
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="body",
            elem_b="rail_0",
            min_overlap=0.150,
            name="raised carriage remains engaged on rail",
        )
        raised_carriage_pos = ctx.part_world_position(carriage)
        raised_face_pos = ctx.part_world_position(output_face)

    ctx.check(
        "vertical slide lifts carriage and carried face",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and rest_face_pos is not None
        and raised_face_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.50
        and raised_face_pos[2] > rest_face_pos[2] + 0.50,
        details=f"carriage {rest_carriage_pos}->{raised_carriage_pos}, face {rest_face_pos}->{raised_face_pos}",
    )

    rest_face_aabb = ctx.part_element_world_aabb(output_face, elem="face_plate")
    with ctx.pose({wrist: 0.80}):
        tilted_face_aabb = ctx.part_element_world_aabb(output_face, elem="face_plate")

    if rest_face_aabb is not None and tilted_face_aabb is not None:
        rest_dx = rest_face_aabb[1][0] - rest_face_aabb[0][0]
        tilted_dx = tilted_face_aabb[1][0] - tilted_face_aabb[0][0]
        rest_center_z = (rest_face_aabb[0][2] + rest_face_aabb[1][2]) * 0.5
        tilted_center_z = (tilted_face_aabb[0][2] + tilted_face_aabb[1][2]) * 0.5
        wrist_ok = tilted_dx > rest_dx + 0.050 and tilted_center_z > rest_center_z + 0.025
    else:
        wrist_ok = False
        rest_dx = tilted_dx = rest_center_z = tilted_center_z = None
    ctx.check(
        "wrist hinge pitches the output face upward",
        wrist_ok,
        details=f"dx {rest_dx}->{tilted_dx}, center_z {rest_center_z}->{tilted_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
