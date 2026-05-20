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
    model = ArticulatedObject(name="under_beam_portal_axis")

    beam_paint = Material("satin_blue_beam", rgba=(0.08, 0.18, 0.32, 1.0))
    rail_steel = Material("brushed_steel_rails", rgba=(0.62, 0.66, 0.68, 1.0))
    rider_paint = Material("warm_gray_rider", rgba=(0.42, 0.44, 0.45, 1.0))
    bearing_liner = Material("dark_bearing_liner", rgba=(0.04, 0.045, 0.05, 1.0))
    carriage_paint = Material("orange_carriage", rgba=(0.95, 0.42, 0.10, 1.0))
    face_plate = Material("black_mounting_face", rgba=(0.02, 0.02, 0.018, 1.0))
    stop_yellow = Material("safety_yellow_stops", rgba=(1.0, 0.78, 0.05, 1.0))

    top_beam = model.part("top_beam")
    top_beam.visual(
        Box((1.80, 0.18, 0.14)),
        origin=Origin(),
        material=beam_paint,
        name="main_beam",
    )
    top_beam.visual(
        Box((1.72, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, 0.055, -0.090)),
        material=rail_steel,
        name="rail_pos",
    )
    top_beam.visual(
        Box((1.72, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, -0.055, -0.090)),
        material=rail_steel,
        name="rail_neg",
    )
    for x, name in ((-0.92, "end_stop_0"), (0.92, "end_stop_1")):
        top_beam.visual(
            Box((0.040, 0.220, 0.180)),
            origin=Origin(xyz=(x, 0.0, -0.020)),
            material=stop_yellow,
            name=name,
        )

    beam_rider = model.part("beam_rider")
    beam_rider.visual(
        Box((0.300, 0.018, 0.170)),
        origin=Origin(xyz=(0.0, 0.105, -0.105)),
        material=rider_paint,
        name="side_plate_pos",
    )
    beam_rider.visual(
        Box((0.300, 0.018, 0.170)),
        origin=Origin(xyz=(0.0, -0.105, -0.105)),
        material=rider_paint,
        name="side_plate_neg",
    )
    beam_rider.visual(
        Box((0.240, 0.0235, 0.030)),
        origin=Origin(xyz=(0.0, 0.08425, -0.090)),
        material=bearing_liner,
        name="pad_pos",
    )
    beam_rider.visual(
        Box((0.240, 0.0235, 0.030)),
        origin=Origin(xyz=(0.0, -0.08425, -0.090)),
        material=bearing_liner,
        name="pad_neg",
    )

    # A four-sided collar leaves an open throat for the vertical ram while tying
    # the hanging yoke into one rigid rider.
    for x, name in ((0.095, "collar_x_pos"), (-0.095, "collar_x_neg")):
        beam_rider.visual(
            Box((0.090, 0.250, 0.030)),
            origin=Origin(xyz=(x, 0.0, -0.190)),
            material=rider_paint,
            name=name,
        )
    for y, name in ((0.100, "collar_y_pos"), (-0.100, "collar_y_neg")):
        beam_rider.visual(
            Box((0.320, 0.050, 0.030)),
            origin=Origin(xyz=(0.0, y, -0.190)),
            material=rider_paint,
            name=name,
        )

    # Compact vertical linear bearing below the beam-rider center.
    beam_rider.visual(
        Box((0.020, 0.160, 0.220)),
        origin=Origin(xyz=(0.070, 0.0, -0.315)),
        material=rider_paint,
        name="guide_x_pos",
    )
    beam_rider.visual(
        Box((0.020, 0.160, 0.220)),
        origin=Origin(xyz=(-0.070, 0.0, -0.315)),
        material=rider_paint,
        name="guide_x_neg",
    )
    for y, name in ((0.070, "guide_y_pos"), (-0.070, "guide_y_neg")):
        beam_rider.visual(
            Box((0.160, 0.020, 0.220)),
            origin=Origin(xyz=(0.0, y, -0.315)),
            material=rider_paint,
            name=name,
        )

    vertical_carriage = model.part("vertical_carriage")
    vertical_carriage.visual(
        Box((0.080, 0.080, 0.580)),
        origin=Origin(xyz=(0.0, 0.0, -0.240)),
        material=carriage_paint,
        name="ram",
    )
    vertical_carriage.visual(
        Box((0.020, 0.120, 0.220)),
        origin=Origin(xyz=(0.050, 0.0, -0.125)),
        material=bearing_liner,
        name="gib_x_pos",
    )
    vertical_carriage.visual(
        Box((0.020, 0.120, 0.220)),
        origin=Origin(xyz=(-0.050, 0.0, -0.125)),
        material=bearing_liner,
        name="gib_x_neg",
    )
    for y, name in ((0.050, "gib_y_pos"), (-0.050, "gib_y_neg")):
        vertical_carriage.visual(
            Box((0.120, 0.020, 0.220)),
            origin=Origin(xyz=(0.0, y, -0.125)),
            material=bearing_liner,
            name=name,
        )
    vertical_carriage.visual(
        Box((0.180, 0.140, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, -0.550)),
        material=carriage_paint,
        name="head_block",
    )
    vertical_carriage.visual(
        Box((0.200, 0.160, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.6025)),
        material=face_plate,
        name="tool_face",
    )

    model.articulation(
        "beam_slide",
        ArticulationType.PRISMATIC,
        parent=top_beam,
        child=beam_rider,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.80, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "vertical_slide",
        ArticulationType.PRISMATIC,
        parent=beam_rider,
        child=vertical_carriage,
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_beam = object_model.get_part("top_beam")
    beam_rider = object_model.get_part("beam_rider")
    vertical_carriage = object_model.get_part("vertical_carriage")
    beam_slide = object_model.get_articulation("beam_slide")
    vertical_slide = object_model.get_articulation("vertical_slide")

    ctx.check(
        "two prismatic axes",
        beam_slide.articulation_type == ArticulationType.PRISMATIC
        and vertical_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"beam_slide={beam_slide.articulation_type}, vertical_slide={vertical_slide.articulation_type}",
    )
    ctx.expect_overlap(
        beam_rider,
        top_beam,
        axes="x",
        elem_a="side_plate_pos",
        elem_b="main_beam",
        min_overlap=0.25,
        name="rider stays under the beam span",
    )
    ctx.expect_gap(
        beam_rider,
        top_beam,
        axis="y",
        positive_elem="pad_pos",
        negative_elem="rail_pos",
        min_gap=0.0,
        max_gap=0.001,
        name="positive rail shoe contact",
    )
    ctx.expect_gap(
        top_beam,
        beam_rider,
        axis="y",
        positive_elem="rail_neg",
        negative_elem="pad_neg",
        min_gap=0.0,
        max_gap=0.001,
        name="negative rail shoe contact",
    )
    ctx.expect_gap(
        beam_rider,
        vertical_carriage,
        axis="x",
        positive_elem="guide_x_pos",
        negative_elem="ram",
        min_gap=0.015,
        max_gap=0.025,
        name="vertical ram positive x guide clearance",
    )
    ctx.expect_gap(
        vertical_carriage,
        beam_rider,
        axis="x",
        positive_elem="ram",
        negative_elem="guide_x_neg",
        min_gap=0.015,
        max_gap=0.025,
        name="vertical ram negative x guide clearance",
    )
    ctx.expect_overlap(
        vertical_carriage,
        beam_rider,
        axes="z",
        elem_a="ram",
        elem_b="guide_x_pos",
        min_overlap=0.20,
        name="retracted ram remains in guide",
    )
    ctx.expect_contact(
        vertical_carriage,
        beam_rider,
        elem_a="gib_x_pos",
        elem_b="guide_x_pos",
        contact_tol=0.001,
        name="positive vertical gib bears on guide",
    )
    ctx.expect_contact(
        vertical_carriage,
        beam_rider,
        elem_a="gib_x_neg",
        elem_b="guide_x_neg",
        contact_tol=0.001,
        name="negative vertical gib bears on guide",
    )

    rider_rest = ctx.part_world_position(beam_rider)
    with ctx.pose({beam_slide: 0.55}):
        rider_extended = ctx.part_world_position(beam_rider)
        ctx.expect_overlap(
            beam_rider,
            top_beam,
            axes="x",
            elem_a="side_plate_pos",
            elem_b="main_beam",
            min_overlap=0.25,
            name="rider remains engaged at travel end",
        )

    ctx.check(
        "beam rider translates along x",
        rider_rest is not None
        and rider_extended is not None
        and rider_extended[0] > rider_rest[0] + 0.50
        and abs(rider_extended[1] - rider_rest[1]) < 1e-6
        and abs(rider_extended[2] - rider_rest[2]) < 1e-6,
        details=f"rest={rider_rest}, extended={rider_extended}",
    )

    carriage_rest = ctx.part_world_position(vertical_carriage)
    with ctx.pose({vertical_slide: 0.22}):
        carriage_lowered = ctx.part_world_position(vertical_carriage)
        ctx.expect_overlap(
            vertical_carriage,
            beam_rider,
            axes="z",
            elem_a="ram",
            elem_b="guide_x_pos",
            min_overlap=0.060,
            name="lowered ram retains guide insertion",
        )

    ctx.check(
        "carriage translates downward",
        carriage_rest is not None
        and carriage_lowered is not None
        and carriage_lowered[2] < carriage_rest[2] - 0.20
        and abs(carriage_lowered[0] - carriage_rest[0]) < 1e-6
        and abs(carriage_lowered[1] - carriage_rest[1]) < 1e-6,
        details=f"rest={carriage_rest}, lowered={carriage_lowered}",
    )

    return ctx.report()


object_model = build_object_model()
