from __future__ import annotations

import math

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
    model = ArticulatedObject(name="cantilever_xz_positioning_head")

    anodized = Material("clear_anodized_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark = Material("matte_black_hardcoat", rgba=(0.03, 0.035, 0.04, 1.0))
    steel = Material("ground_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    yellow = Material("safety_yellow_stops", rgba=(0.95, 0.70, 0.08, 1.0))

    rail = model.part("fixed_rail")
    rail.visual(
        Box((0.90, 0.060, 0.080)),
        origin=Origin(),
        material=anodized,
        name="rail_beam",
    )
    rail.visual(
        Box((0.82, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.015, 0.042)),
        material=steel,
        name="top_track",
    )
    rail.visual(
        Box((0.82, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.015, -0.042)),
        material=steel,
        name="bottom_track",
    )
    rail.visual(
        Box((0.035, 0.095, 0.125)),
        origin=Origin(xyz=(-0.462, 0.000, 0.000)),
        material=yellow,
        name="end_stop_0",
    )
    rail.visual(
        Box((0.035, 0.095, 0.125)),
        origin=Origin(xyz=(0.462, 0.000, 0.000)),
        material=yellow,
        name="end_stop_1",
    )
    rail.visual(
        Box((0.085, 0.190, 0.330)),
        origin=Origin(xyz=(-0.495, -0.010, -0.130)),
        material=dark,
        name="support_web",
    )
    rail.visual(
        Box((0.190, 0.250, 0.045)),
        origin=Origin(xyz=(-0.495, -0.010, -0.285)),
        material=dark,
        name="mounting_foot",
    )
    rail.visual(
        Box((0.190, 0.030, 0.185)),
        origin=Origin(xyz=(-0.455, -0.083, -0.065), rpy=(0.0, -0.58, 0.0)),
        material=dark,
        name="rear_gusset",
    )
    rail.visual(
        Cylinder(radius=0.009, length=0.007),
        origin=Origin(xyz=(-0.540, -0.083, -0.292), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="foot_bolt_0",
    )
    rail.visual(
        Cylinder(radius=0.009, length=0.007),
        origin=Origin(xyz=(-0.450, -0.083, -0.292), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="foot_bolt_1",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.160, 0.040, 0.190)),
        origin=Origin(xyz=(0.0, 0.084, 0.000)),
        material=dark,
        name="front_saddle",
    )
    carriage.visual(
        Box((0.160, 0.060, 0.025)),
        origin=Origin(xyz=(0.0, 0.040, 0.0575)),
        material=anodized,
        name="top_shoe",
    )
    carriage.visual(
        Box((0.160, 0.060, 0.025)),
        origin=Origin(xyz=(0.0, 0.040, -0.0575)),
        material=anodized,
        name="bottom_shoe",
    )
    carriage.visual(
        Box((0.124, 0.064, 0.420)),
        origin=Origin(xyz=(0.0, 0.132, -0.040)),
        material=anodized,
        name="vertical_column",
    )
    carriage.visual(
        Box((0.142, 0.074, 0.026)),
        origin=Origin(xyz=(0.0, 0.132, 0.176)),
        material=yellow,
        name="top_stop",
    )
    carriage.visual(
        Box((0.142, 0.074, 0.026)),
        origin=Origin(xyz=(0.0, 0.132, -0.256)),
        material=yellow,
        name="bottom_stop",
    )
    carriage.visual(
        Box((0.020, 0.007, 0.370)),
        origin=Origin(xyz=(-0.040, 0.1635, -0.040)),
        material=steel,
        name="vertical_guide_0",
    )
    carriage.visual(
        Box((0.020, 0.007, 0.370)),
        origin=Origin(xyz=(0.040, 0.1635, -0.040)),
        material=steel,
        name="vertical_guide_1",
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        Box((0.180, 0.035, 0.160)),
        origin=Origin(),
        material=anodized,
        name="tool_face",
    )
    for x, z, pad_name in (
        (-0.040, -0.052, "rear_pad_0_0"),
        (-0.040, 0.052, "rear_pad_0_1"),
        (0.040, -0.052, "rear_pad_1_0"),
        (0.040, 0.052, "rear_pad_1_1"),
    ):
        tool_plate.visual(
            Box((0.038, 0.006, 0.030)),
            origin=Origin(xyz=(x, -0.0200, z)),
            material=steel,
            name=pad_name,
        )
    tool_plate.visual(
        Box((0.070, 0.010, 0.082)),
        origin=Origin(xyz=(0.0, 0.0215, 0.0)),
        material=dark,
        name="tool_mount",
    )
    for x, z, bolt_name in (
        (-0.068, -0.058, "face_bolt_0_0"),
        (-0.068, 0.058, "face_bolt_0_1"),
        (0.068, -0.058, "face_bolt_1_0"),
        (0.068, 0.058, "face_bolt_1_1"),
    ):
        tool_plate.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, 0.0205, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=bolt_name,
        )

    model.articulation(
        "x_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(-0.240, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.480),
    )
    model.articulation(
        "z_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=tool_plate,
        origin=Origin(xyz=(0.0, 0.190, -0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.200),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("fixed_rail")
    carriage = object_model.get_part("carriage")
    tool_plate = object_model.get_part("tool_plate")
    x_slide = object_model.get_articulation("x_slide")
    z_slide = object_model.get_articulation("z_slide")

    ctx.expect_contact(
        carriage,
        rail,
        elem_a="top_shoe",
        elem_b="top_track",
        contact_tol=1e-4,
        name="carriage upper shoe bears on rail track",
    )
    ctx.expect_contact(
        carriage,
        rail,
        elem_a="bottom_shoe",
        elem_b="bottom_track",
        contact_tol=1e-4,
        name="carriage lower shoe bears on rail track",
    )
    ctx.expect_contact(
        tool_plate,
        carriage,
        elem_a="rear_pad_0_0",
        elem_b="vertical_guide_0",
        contact_tol=1e-4,
        name="tool plate pad bears on vertical guide",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_tool_pos = ctx.part_world_position(tool_plate)

    with ctx.pose({x_slide: 0.480}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            rail,
            axes="x",
            elem_a="top_shoe",
            elem_b="top_track",
            min_overlap=0.080,
            name="carriage remains horizontally engaged at x travel",
        )

    with ctx.pose({z_slide: 0.200}):
        raised_tool_pos = ctx.part_world_position(tool_plate)
        ctx.expect_overlap(
            tool_plate,
            carriage,
            axes="z",
            elem_a="tool_face",
            elem_b="vertical_column",
            min_overlap=0.080,
            name="raised tool plate remains on column",
        )

    ctx.check(
        "x slide moves carriage horizontally",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.45,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )
    ctx.check(
        "z slide raises tool plate vertically",
        rest_tool_pos is not None
        and raised_tool_pos is not None
        and raised_tool_pos[2] > rest_tool_pos[2] + 0.18,
        details=f"rest={rest_tool_pos}, raised={raised_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()
