from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="underslung_radial_arm_carriage")

    model.material("painted_steel", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("dark_bearing", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("safety_yellow", rgba=(0.95, 0.67, 0.12, 1.0))
    model.material("hardened_rail", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("carriage_blue", rgba=(0.10, 0.20, 0.32, 1.0))
    model.material("rubber_black", rgba=(0.02, 0.02, 0.02, 1.0))

    support = model.part("top_support")
    support.visual(
        Box((0.62, 0.38, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material="painted_steel",
        name="ceiling_plate",
    )
    support.visual(
        Cylinder(radius=0.055, length=0.270),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material="painted_steel",
        name="drop_tube",
    )
    support.visual(
        Box((0.20, 0.018, 0.245)),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material="painted_steel",
        name="cross_gusset_x",
    )
    support.visual(
        Box((0.018, 0.20, 0.245)),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material="painted_steel",
        name="cross_gusset_y",
    )
    support.visual(
        Cylinder(radius=0.140, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="painted_steel",
        name="bearing_cap",
    )
    support.visual(
        Cylinder(radius=0.108, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material="dark_bearing",
        name="lower_race",
    )

    beam = model.part("rotating_beam")
    beam.visual(
        Cylinder(radius=0.105, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0645)),
        material="safety_yellow",
        name="rotary_hub",
    )
    beam.visual(
        Box((0.920, 0.110, 0.075)),
        origin=Origin(xyz=(0.550, 0.0, -0.130)),
        material="safety_yellow",
        name="box_beam",
    )
    beam.visual(
        Box((0.940, 0.145, 0.018)),
        origin=Origin(xyz=(0.560, 0.0, -0.086)),
        material="safety_yellow",
        name="top_flange",
    )
    beam.visual(
        Box((0.840, 0.014, 0.030)),
        origin=Origin(xyz=(0.550, -0.047, -0.174)),
        material="safety_yellow",
        name="rail_web_0",
    )
    beam.visual(
        Box((0.840, 0.020, 0.024)),
        origin=Origin(xyz=(0.550, -0.047, -0.197)),
        material="hardened_rail",
        name="rail_0",
    )
    beam.visual(
        Box((0.840, 0.014, 0.030)),
        origin=Origin(xyz=(0.550, 0.047, -0.174)),
        material="safety_yellow",
        name="rail_web_1",
    )
    beam.visual(
        Box((0.840, 0.020, 0.024)),
        origin=Origin(xyz=(0.550, 0.047, -0.197)),
        material="hardened_rail",
        name="rail_1",
    )
    beam.visual(
        Box((0.026, 0.165, 0.120)),
        origin=Origin(xyz=(0.145, 0.0, -0.151)),
        material="safety_yellow",
        name="inner_stop",
    )
    beam.visual(
        Box((0.036, 0.165, 0.130)),
        origin=Origin(xyz=(1.025, 0.0, -0.151)),
        material="safety_yellow",
        name="outer_stop",
    )

    carriage = model.part("hanging_carriage")
    carriage.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="roller_0",
    )
    carriage.visual(
        Cylinder(radius=0.006, length=0.190),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="hardened_rail",
        name="axle_0",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="roller_1",
    )
    carriage.visual(
        Cylinder(radius=0.006, length=0.190),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="hardened_rail",
        name="axle_1",
    )
    carriage.visual(
        Box((0.190, 0.012, 0.105)),
        origin=Origin(xyz=(0.0, -0.089, -0.052)),
        material="carriage_blue",
        name="side_plate_0",
    )
    carriage.visual(
        Box((0.190, 0.012, 0.105)),
        origin=Origin(xyz=(0.0, 0.089, -0.052)),
        material="carriage_blue",
        name="side_plate_1",
    )
    carriage.visual(
        Box((0.205, 0.190, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material="carriage_blue",
        name="lower_saddle",
    )
    carriage.visual(
        Box((0.055, 0.065, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material="carriage_blue",
        name="hanger_stem",
    )
    carriage.visual(
        Cylinder(radius=0.042, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.282)),
        material="hardened_rail",
        name="swivel_boss",
    )
    carriage.visual(
        Box((0.220, 0.150, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.315)),
        material="carriage_blue",
        name="load_plate",
    )

    model.articulation(
        "support_to_beam",
        ArticulationType.REVOLUTE,
        parent=support,
        child=beam,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=-2.2, upper=2.2),
    )
    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.350, 0.0, -0.227)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.35, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    beam = object_model.get_part("rotating_beam")
    carriage = object_model.get_part("hanging_carriage")
    swing = object_model.get_articulation("support_to_beam")
    slide = object_model.get_articulation("beam_to_carriage")

    ctx.check(
        "beam uses vertical revolute joint",
        swing.articulation_type == ArticulationType.REVOLUTE and tuple(swing.axis) == (0.0, 0.0, 1.0),
        details=f"type={swing.articulation_type}, axis={swing.axis}",
    )
    ctx.check(
        "carriage uses beam-axis prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )

    ctx.expect_gap(
        support,
        beam,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_race",
        negative_elem="rotary_hub",
        name="bearing race seats on rotary hub",
    )
    ctx.expect_overlap(
        support,
        beam,
        axes="xy",
        min_overlap=0.08,
        elem_a="lower_race",
        elem_b="rotary_hub",
        name="hub remains centered below top support",
    )
    for rail_name, roller_name in (("rail_0", "roller_0"), ("rail_1", "roller_1")):
        ctx.expect_gap(
            beam,
            carriage,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=rail_name,
            negative_elem=roller_name,
            name=f"{roller_name} bears on {rail_name}",
        )
        ctx.expect_overlap(
            beam,
            carriage,
            axes="xy",
            min_overlap=0.015,
            elem_a=rail_name,
            elem_b=roller_name,
            name=f"{roller_name} overlaps {rail_name} footprint",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.45}):
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            beam,
            carriage,
            axes="xy",
            min_overlap=0.015,
            elem_a="rail_0",
            elem_b="roller_0",
            name="outer slide remains on rail",
        )
    ctx.check(
        "carriage slides outward along beam",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.40,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({swing: 1.0}):
        swung_pos = ctx.part_world_position(carriage)
    ctx.check(
        "rotating beam carries carriage around pivot",
        rest_pos is not None and swung_pos is not None and abs(swung_pos[1]) > 0.25,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
