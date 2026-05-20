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
    model = ArticulatedObject(name="hanging_beam_slide")

    painted = Material("safety_yellow_paint", rgba=(0.92, 0.66, 0.08, 1.0))
    dark_paint = Material("charcoal_machine_paint", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    black = Material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    bronze = Material("oiled_bronze", rgba=(0.56, 0.38, 0.18, 1.0))
    urethane = Material("blue_polyurethane", rgba=(0.05, 0.20, 0.55, 1.0))
    stripe = Material("white_index_mark", rgba=(0.95, 0.94, 0.86, 1.0))

    beam = model.part("beam")
    beam.visual(
        Box((1.22, 0.10, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=painted,
        name="beam_body",
    )
    beam.visual(
        Box((1.16, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, 0.055, 0.60)),
        material=steel,
        name="side_rail_pos",
    )
    beam.visual(
        Box((1.16, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, -0.055, 0.60)),
        material=steel,
        name="side_rail_neg",
    )
    beam.visual(
        Box((1.16, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.552)),
        material=steel,
        name="lower_rail",
    )
    beam.visual(
        Box((0.035, 0.160, 0.050)),
        origin=Origin(xyz=(-0.54, 0.0, 0.535)),
        material=dark_paint,
        name="travel_stop_0",
    )
    beam.visual(
        Box((0.020, 0.122, 0.018)),
        origin=Origin(xyz=(-0.5373, 0.0, 0.501)),
        material=black,
        name="stop_bumper_0",
    )
    beam.visual(
        Box((0.035, 0.160, 0.050)),
        origin=Origin(xyz=(0.54, 0.0, 0.535)),
        material=dark_paint,
        name="travel_stop_1",
    )
    beam.visual(
        Box((0.020, 0.122, 0.018)),
        origin=Origin(xyz=(0.5373, 0.0, 0.501)),
        material=black,
        name="stop_bumper_1",
    )
    beam.visual(
        Box((1.20, 0.40, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.830)),
        material=dark_paint,
        name="ceiling_plate",
    )
    for x, visual_name in ((-0.565, "hanger_0"), (0.565, "hanger_1")):
        beam.visual(
            Box((0.040, 0.040, 0.180)),
            origin=Origin(xyz=(x, 0.160, 0.730)),
            material=dark_paint,
            name=visual_name,
        )
        beam.visual(
            Box((0.080, 0.220, 0.012)),
            origin=Origin(xyz=(x, 0.105, 0.646)),
            material=steel,
            name=f"hanger_pad_{visual_name[-1]}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.200, 0.016, 0.170)),
        origin=Origin(xyz=(0.0, 0.088, 0.0)),
        material=dark_paint,
        name="side_plate_pos",
    )
    carriage.visual(
        Box((0.200, 0.016, 0.170)),
        origin=Origin(xyz=(0.0, -0.088, 0.0)),
        material=dark_paint,
        name="side_plate_neg",
    )
    carriage.visual(
        Box((0.200, 0.192, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=dark_paint,
        name="top_bridge",
    )
    carriage.visual(
        Box((0.200, 0.192, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=dark_paint,
        name="bottom_bridge",
    )
    carriage.visual(
        Box((0.190, 0.014, 0.160)),
        origin=Origin(xyz=(0.0, -0.102, 0.0)),
        material=painted,
        name="front_face",
    )
    carriage.visual(
        Box((0.140, 0.010, 0.112)),
        origin=Origin(xyz=(0.0, 0.101, 0.0)),
        material=steel,
        name="rear_service_cover",
    )
    carriage.visual(
        Box((0.126, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, 0.070, 0.0)),
        material=bronze,
        name="wear_pad_pos",
    )
    carriage.visual(
        Box((0.126, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, -0.070, 0.0)),
        material=bronze,
        name="wear_pad_neg",
    )
    for x in (-0.106, 0.106):
        suffix = "0" if x < 0.0 else "1"
        carriage.visual(
            Box((0.012, 0.012, 0.090)),
            origin=Origin(xyz=(x, 0.074, 0.0)),
            material=black,
            name=f"wiper_pos_{suffix}",
        )
        carriage.visual(
            Box((0.012, 0.012, 0.090)),
            origin=Origin(xyz=(x, -0.074, 0.0)),
            material=black,
            name=f"wiper_neg_{suffix}",
        )
    carriage.visual(
        Cylinder(radius=0.040, length=0.025),
        origin=Origin(xyz=(0.0, -0.1215, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bearing_flange",
    )
    for x in (-0.060, 0.060):
        for z in (-0.047, 0.047):
            carriage.visual(
                Cylinder(radius=0.008, length=0.007),
                origin=Origin(xyz=(x, -0.1125, z), rpy=(-pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"shoulder_bolt_{'pos' if x > 0 else 'neg'}_{'top' if z > 0 else 'bot'}",
            )
    carriage.visual(
        Box((0.145, 0.190, 0.012)),
        origin=Origin(xyz=(0.0, -0.198, 0.070)),
        material=painted,
        name="top_guard",
    )
    carriage.visual(
        Box((0.145, 0.190, 0.012)),
        origin=Origin(xyz=(0.0, -0.198, -0.070)),
        material=painted,
        name="bottom_guard",
    )
    carriage.visual(
        Box((0.010, 0.190, 0.152)),
        origin=Origin(xyz=(0.068, -0.198, 0.0)),
        material=painted,
        name="guard_side_pos",
    )
    carriage.visual(
        Box((0.010, 0.190, 0.152)),
        origin=Origin(xyz=(-0.068, -0.198, 0.0)),
        material=painted,
        name="guard_side_neg",
    )

    roller = model.part("roller")
    roller.visual(
        Cylinder(radius=0.045, length=0.110),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=urethane,
        name="roller_body",
    )
    roller.visual(
        Cylinder(radius=0.013, length=0.142),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_pin",
    )
    roller.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.064, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="inner_end_cap",
    )
    roller.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, -0.061, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="outer_end_cap",
    )
    roller.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, -0.0745, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="machined_end_cap",
    )
    roller.visual(
        Box((0.010, 0.080, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0465)),
        material=stripe,
        name="index_stripe",
    )

    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(-0.35, 0.0, 0.60)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=0.0, upper=0.70),
    )
    model.articulation(
        "carriage_to_roller",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=roller,
        origin=Origin(xyz=(0.0, -0.205, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    roller = object_model.get_part("roller")
    slide = object_model.get_articulation("beam_to_carriage")
    spin = object_model.get_articulation("carriage_to_roller")

    ctx.check(
        "carriage uses guided prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 0.70,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )
    ctx.check(
        "roller uses separate axle revolute joint",
        spin.articulation_type == ArticulationType.REVOLUTE
        and tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    with ctx.pose({slide: 0.0, spin: 0.0}):
        rest_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            beam,
            axis="x",
            negative_elem="travel_stop_0",
            min_gap=0.045,
            name="shuttle clears lower travel stop",
        )
        ctx.expect_contact(
            carriage,
            beam,
            elem_a="wear_pad_pos",
            elem_b="side_rail_pos",
            contact_tol=0.001,
            name="positive wear pad rides rail",
        )
        ctx.expect_contact(
            carriage,
            beam,
            elem_a="wear_pad_neg",
            elem_b="side_rail_neg",
            contact_tol=0.001,
            name="negative wear pad rides rail",
        )
        ctx.expect_gap(
            carriage,
            beam,
            axis="z",
            positive_elem="top_bridge",
            negative_elem="beam_body",
            min_gap=0.030,
            name="top bridge clears beam",
        )
        ctx.expect_gap(
            beam,
            carriage,
            axis="z",
            positive_elem="beam_body",
            negative_elem="bottom_bridge",
            min_gap=0.030,
            name="bottom bridge clears beam",
        )
        ctx.expect_contact(
            roller,
            carriage,
            elem_a="axle_pin",
            elem_b="bearing_flange",
            contact_tol=0.001,
            name="roller axle is supported in bearing",
        )

    with ctx.pose({slide: 0.70, spin: 0.0}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            beam,
            carriage,
            axis="x",
            positive_elem="travel_stop_1",
            min_gap=0.045,
            name="shuttle clears upper travel stop",
        )
        ctx.expect_contact(
            carriage,
            beam,
            elem_a="wear_pad_pos",
            elem_b="side_rail_pos",
            contact_tol=0.001,
            name="positive wear pad rides rail at extension",
        )
        ctx.expect_contact(
            carriage,
            beam,
            elem_a="wear_pad_neg",
            elem_b="side_rail_neg",
            contact_tol=0.001,
            name="negative wear pad rides rail at extension",
        )

    ctx.check(
        "shuttle moves along beam axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.65
        and abs(extended_pos[1] - rest_pos[1]) < 0.001
        and abs(extended_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    for angle, label in ((0.0, "indexed"), (pi / 2.0, "quarter_turn"), (pi, "half_turn")):
        with ctx.pose({spin: angle}):
            ctx.expect_gap(
                carriage,
                roller,
                axis="z",
                positive_elem="top_guard",
                min_gap=0.008,
                name=f"roller clears top guard at {label}",
            )
            ctx.expect_gap(
                roller,
                carriage,
                axis="z",
                negative_elem="bottom_guard",
                min_gap=0.008,
                name=f"roller clears bottom guard at {label}",
            )
            ctx.expect_gap(
                carriage,
                roller,
                axis="x",
                positive_elem="guard_side_pos",
                min_gap=0.007,
                name=f"roller clears positive side guard at {label}",
            )
            ctx.expect_gap(
                roller,
                carriage,
                axis="x",
                negative_elem="guard_side_neg",
                min_gap=0.007,
                name=f"roller clears negative side guard at {label}",
            )

    return ctx.report()


object_model = build_object_model()
