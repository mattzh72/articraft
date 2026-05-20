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
    model = ArticulatedObject(name="lift_slide_telescoping_arm")

    dark = Material("anodized_dark", color=(0.08, 0.09, 0.10, 1.0))
    rail = Material("brushed_steel", color=(0.70, 0.73, 0.74, 1.0))
    carriage_blue = Material("painted_carriage_blue", color=(0.05, 0.22, 0.44, 1.0))
    plate_orange = Material("safety_orange_plate", color=(0.95, 0.38, 0.08, 1.0))
    probe_steel = Material("polished_probe", color=(0.86, 0.84, 0.75, 1.0))
    rubber = Material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))

    guide_stage = model.part("guide_stage")
    guide_stage.visual(
        Box((0.46, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark,
        name="floor_base",
    )
    guide_stage.visual(
        Box((0.16, 0.08, 0.86)),
        origin=Origin(xyz=(0.0, 0.04, 0.45)),
        material=dark,
        name="upright_mast",
    )
    guide_stage.visual(
        Box((0.025, 0.025, 0.72)),
        origin=Origin(xyz=(-0.055, -0.008, 0.44)),
        material=rail,
        name="rail_0",
    )
    guide_stage.visual(
        Box((0.025, 0.025, 0.72)),
        origin=Origin(xyz=(0.055, -0.008, 0.44)),
        material=rail,
        name="rail_1",
    )
    guide_stage.visual(
        Box((0.19, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.006, 0.055)),
        material=rail,
        name="bottom_crosshead",
    )
    guide_stage.visual(
        Box((0.19, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.006, 0.83)),
        material=rail,
        name="top_crosshead",
    )
    guide_stage.visual(
        Cylinder(radius=0.009, length=0.77),
        origin=Origin(xyz=(0.0, 0.018, 0.442)),
        material=rail,
        name="lead_screw",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.22, 0.045, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_blue,
        name="slider_body",
    )
    carriage.visual(
        Box((0.032, 0.070, 0.12)),
        origin=Origin(xyz=(-0.080, -0.052, 0.020)),
        material=carriage_blue,
        name="clevis_0",
    )
    carriage.visual(
        Box((0.032, 0.070, 0.12)),
        origin=Origin(xyz=(0.080, -0.052, 0.020)),
        material=carriage_blue,
        name="clevis_1",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.096, -0.085, 0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="pin_cap_0",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.096, -0.085, 0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="pin_cap_1",
    )
    for i, x in enumerate((-0.055, 0.055)):
        carriage.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, 0.0175, 0.060), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"guide_wheel_{i}",
        )

    guide_slide = model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=guide_stage,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.043, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.35, lower=0.0, upper=0.32),
    )

    elbow_plate = model.part("elbow_plate")
    elbow_plate.visual(
        Box((0.110, 0.350, 0.035)),
        origin=Origin(xyz=(0.0, -0.175, 0.0)),
        material=plate_orange,
        name="link_plate",
    )
    elbow_plate.visual(
        Cylinder(radius=0.028, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail,
        name="pivot_bushing",
    )
    elbow_plate.visual(
        Cylinder(radius=0.018, length=0.125),
        origin=Origin(xyz=(0.0, -0.350, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail,
        name="tip_spacer",
    )
    elbow_plate.visual(
        Box((0.090, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, -0.430, -0.036)),
        material=plate_orange,
        name="sleeve_top",
    )
    elbow_plate.visual(
        Box((0.090, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, -0.430, -0.084)),
        material=plate_orange,
        name="sleeve_bottom",
    )
    elbow_plate.visual(
        Box((0.012, 0.180, 0.060)),
        origin=Origin(xyz=(-0.039, -0.430, -0.060)),
        material=plate_orange,
        name="sleeve_wall_0",
    )
    elbow_plate.visual(
        Box((0.012, 0.180, 0.060)),
        origin=Origin(xyz=(0.039, -0.430, -0.060)),
        material=plate_orange,
        name="sleeve_wall_1",
    )
    elbow_plate.visual(
        Box((0.090, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, -0.350, -0.024)),
        material=plate_orange,
        name="sleeve_web",
    )

    elbow_hinge = model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow_plate,
        origin=Origin(xyz=(0.0, -0.085, 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.35, upper=1.20),
    )

    probe = model.part("probe")
    probe.visual(
        Box((0.045, 0.360, 0.036)),
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
        material=probe_steel,
        name="probe_bar",
    )
    probe.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.0, -0.236, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="contact_tip",
    )
    probe.visual(
        Box((0.047, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.118, 0.0)),
        material=rubber,
        name="stop_collar",
    )

    probe_slide = model.articulation(
        "probe_slide",
        ArticulationType.PRISMATIC,
        parent=elbow_plate,
        child=probe,
        origin=Origin(xyz=(0.0, -0.430, -0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.25, lower=0.0, upper=0.16),
    )

    # Keep lints quiet while preserving readable handles for tests and probes.
    _ = (guide_slide, elbow_hinge, probe_slide)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    guide_stage = object_model.get_part("guide_stage")
    carriage = object_model.get_part("carriage")
    elbow_plate = object_model.get_part("elbow_plate")
    probe = object_model.get_part("probe")
    guide_slide = object_model.get_articulation("guide_slide")
    elbow_hinge = object_model.get_articulation("elbow_hinge")
    probe_slide = object_model.get_articulation("probe_slide")

    ctx.check(
        "three primary joints are present",
        len(object_model.articulations) == 3,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.expect_gap(
        guide_stage,
        carriage,
        axis="y",
        positive_elem="rail_0",
        negative_elem="slider_body",
        max_gap=0.001,
        max_penetration=0.00001,
        name="carriage face bears on guide rail",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({guide_slide: 0.32}):
        lifted_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            guide_stage,
            axes="z",
            elem_a="slider_body",
            elem_b="rail_0",
            min_overlap=0.10,
            name="lifted carriage remains on the rails",
        )

    ctx.check(
        "root slide lifts the carriage",
        rest_carriage is not None
        and lifted_carriage is not None
        and lifted_carriage[2] > rest_carriage[2] + 0.30,
        details=f"rest={rest_carriage}, lifted={lifted_carriage}",
    )

    rest_probe_origin = ctx.part_world_position(probe)
    with ctx.pose({elbow_hinge: 0.85}):
        raised_probe_origin = ctx.part_world_position(probe)

    ctx.check(
        "elbow hinge raises the distal sleeve",
        rest_probe_origin is not None
        and raised_probe_origin is not None
        and raised_probe_origin[2] > rest_probe_origin[2] + 0.25,
        details=f"rest={rest_probe_origin}, raised={raised_probe_origin}",
    )

    ctx.expect_within(
        probe,
        elbow_plate,
        axes="xz",
        inner_elem="probe_bar",
        outer_elem="sleeve_top",
        margin=0.050,
        name="probe bar is centered under the sleeve width",
    )
    ctx.expect_overlap(
        probe,
        elbow_plate,
        axes="y",
        elem_a="probe_bar",
        elem_b="sleeve_top",
        min_overlap=0.15,
        name="retracted probe is captured in the distal sleeve",
    )

    rest_probe_slide = ctx.part_world_position(probe)
    with ctx.pose({probe_slide: 0.16}):
        extended_probe_slide = ctx.part_world_position(probe)
        ctx.expect_overlap(
            probe,
            elbow_plate,
            axes="y",
            elem_a="probe_bar",
            elem_b="sleeve_top",
            min_overlap=0.055,
            name="extended probe keeps retained insertion",
        )

    ctx.check(
        "distal slide extends the probe outward",
        rest_probe_slide is not None
        and extended_probe_slide is not None
        and extended_probe_slide[1] < rest_probe_slide[1] - 0.15,
        details=f"rest={rest_probe_slide}, extended={extended_probe_slide}",
    )

    return ctx.report()


object_model = build_object_model()
