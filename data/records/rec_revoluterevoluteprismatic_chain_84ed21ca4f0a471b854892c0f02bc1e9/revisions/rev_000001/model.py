from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_telescoping_manipulator")

    cast = model.material("dark_cast_metal", color=(0.08, 0.085, 0.09, 1.0))
    blue = model.material("blue_anodized_link", color=(0.05, 0.22, 0.55, 1.0))
    graphite = model.material("graphite_forearm", color=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("brushed_steel", color=(0.72, 0.72, 0.68, 1.0))
    black = model.material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))
    brass = model.material("brass_bushings", color=(0.78, 0.57, 0.25, 1.0))

    shoulder_block = model.part("shoulder_block")
    shoulder_block.visual(
        Box((0.46, 0.34, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=cast,
        name="bench_plate",
    )
    shoulder_block.visual(
        Box((0.17, 0.17, 0.19)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=cast,
        name="pedestal",
    )
    shoulder_block.visual(
        Box((0.10, 0.030, 0.18)),
        origin=Origin(xyz=(0.0, 0.086, 0.300)),
        material=cast,
        name="shoulder_cheek_0",
    )
    shoulder_block.visual(
        Box((0.10, 0.030, 0.18)),
        origin=Origin(xyz=(0.0, -0.086, 0.300)),
        material=cast,
        name="shoulder_cheek_1",
    )
    shoulder_block.visual(
        Cylinder(radius=0.033, length=0.19),
        origin=Origin(xyz=(0.0, 0.0, 0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="shoulder_bushing",
    )
    shoulder_block.visual(
        Box((0.13, 0.20, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.222)),
        material=cast,
        name="yoke_bridge",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.045, length=0.090),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="shoulder_hub",
    )
    upper_link.visual(
        Box((0.445, 0.060, 0.060)),
        origin=Origin(xyz=(0.2675, 0.0, 0.0)),
        material=blue,
        name="main_beam",
    )
    upper_link.visual(
        Box((0.050, 0.160, 0.060)),
        origin=Origin(xyz=(0.470, 0.0, 0.0)),
        material=blue,
        name="elbow_yoke_bridge",
    )
    upper_link.visual(
        Box((0.130, 0.026, 0.088)),
        origin=Origin(xyz=(0.555, 0.069, 0.0)),
        material=blue,
        name="elbow_cheek_0",
    )
    upper_link.visual(
        Box((0.130, 0.026, 0.088)),
        origin=Origin(xyz=(0.555, -0.069, 0.0)),
        material=blue,
        name="elbow_cheek_1",
    )
    upper_link.visual(
        Cylinder(radius=0.026, length=0.16),
        origin=Origin(xyz=(0.555, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="elbow_cross_pin",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.040, length=0.088),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.385, 0.070, 0.070)),
        origin=Origin(xyz=(0.2275, 0.0, 0.0)),
        material=graphite,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.024, 0.108, 0.015)),
        origin=Origin(xyz=(0.412, 0.0, 0.0405)),
        material=graphite,
        name="rear_collar_top",
    )
    forearm.visual(
        Box((0.024, 0.108, 0.015)),
        origin=Origin(xyz=(0.412, 0.0, -0.0405)),
        material=graphite,
        name="rear_collar_bottom",
    )
    forearm.visual(
        Box((0.024, 0.015, 0.066)),
        origin=Origin(xyz=(0.412, 0.0405, 0.0)),
        material=graphite,
        name="rear_collar_side_0",
    )
    forearm.visual(
        Box((0.024, 0.015, 0.066)),
        origin=Origin(xyz=(0.412, -0.0405, 0.0)),
        material=graphite,
        name="rear_collar_side_1",
    )
    forearm.visual(
        Box((0.240, 0.092, 0.014)),
        origin=Origin(xyz=(0.540, 0.0, 0.0385)),
        material=graphite,
        name="nose_top_rail",
    )
    forearm.visual(
        Box((0.240, 0.092, 0.014)),
        origin=Origin(xyz=(0.540, 0.0, -0.0385)),
        material=graphite,
        name="nose_bottom_rail",
    )
    forearm.visual(
        Box((0.240, 0.014, 0.063)),
        origin=Origin(xyz=(0.540, 0.0385, 0.0)),
        material=graphite,
        name="nose_side_rail_0",
    )
    forearm.visual(
        Box((0.240, 0.014, 0.063)),
        origin=Origin(xyz=(0.540, -0.0385, 0.0)),
        material=graphite,
        name="nose_side_rail_1",
    )
    forearm.visual(
        Box((0.018, 0.102, 0.014)),
        origin=Origin(xyz=(0.660, 0.0, 0.0385)),
        material=brass,
        name="lip_bushing_top",
    )
    forearm.visual(
        Box((0.018, 0.102, 0.014)),
        origin=Origin(xyz=(0.660, 0.0, -0.0385)),
        material=brass,
        name="lip_bushing_bottom",
    )
    forearm.visual(
        Box((0.018, 0.014, 0.063)),
        origin=Origin(xyz=(0.660, 0.0385, 0.0)),
        material=brass,
        name="lip_bushing_side_0",
    )
    forearm.visual(
        Box((0.018, 0.014, 0.063)),
        origin=Origin(xyz=(0.660, -0.0385, 0.0)),
        material=brass,
        name="lip_bushing_side_1",
    )

    probe = model.part("probe")
    probe.visual(
        Cylinder(radius=0.021, length=0.300),
        origin=Origin(xyz=(-0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="sliding_barrel",
    )
    probe.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="probe_rod",
    )
    probe.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        material=black,
        name="contact_ball",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=shoulder_block,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.45, upper=1.25),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(0.555, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "probe_slide",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=probe,
        origin=Origin(xyz=(0.660, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.22, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    probe_slide = object_model.get_articulation("probe_slide")
    shoulder_block = object_model.get_part("shoulder_block")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    probe = object_model.get_part("probe")

    ctx.allow_overlap(
        shoulder_block,
        upper_link,
        elem_a="shoulder_bushing",
        elem_b="shoulder_hub",
        reason="The fixed shoulder bushing is intentionally shown captured through the rotating shoulder hub.",
    )
    ctx.allow_overlap(
        upper_link,
        forearm,
        elem_a="elbow_cross_pin",
        elem_b="elbow_hub",
        reason="The elbow cross pin is intentionally represented as captured inside the forearm hub.",
    )

    ctx.check(
        "serial shoulder elbow and probe slide",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and probe_slide.articulation_type == ArticulationType.PRISMATIC,
        details="Bench manipulator must have two serial revolute joints and a final prismatic probe.",
    )

    ctx.expect_within(
        probe,
        forearm,
        axes="yz",
        inner_elem="sliding_barrel",
        margin=0.0,
        name="probe barrel is centered inside the forearm nose bore",
    )
    ctx.expect_overlap(
        probe,
        forearm,
        axes="x",
        elem_a="sliding_barrel",
        elem_b="nose_top_rail",
        min_overlap=0.20,
        name="collapsed probe remains housed in the nose sleeve",
    )
    ctx.expect_within(
        shoulder_block,
        upper_link,
        axes="xz",
        inner_elem="shoulder_bushing",
        outer_elem="shoulder_hub",
        margin=0.0,
        name="shoulder bushing is concentric inside the hub",
    )
    ctx.expect_within(
        upper_link,
        forearm,
        axes="xz",
        inner_elem="elbow_cross_pin",
        outer_elem="elbow_hub",
        margin=0.0,
        name="elbow pin is concentric inside the forearm hub",
    )

    rest_probe_position = ctx.part_world_position(probe)
    with ctx.pose({probe_slide: 0.18}):
        ctx.expect_overlap(
            probe,
            forearm,
            axes="x",
            elem_a="sliding_barrel",
            elem_b="nose_top_rail",
            min_overlap=0.055,
            name="extended probe retains insertion in the nose sleeve",
        )
        extended_probe_position = ctx.part_world_position(probe)

    ctx.check(
        "probe slide extends along forearm axis",
        rest_probe_position is not None
        and extended_probe_position is not None
        and extended_probe_position[0] > rest_probe_position[0] + 0.17,
        details=f"rest={rest_probe_position}, extended={extended_probe_position}",
    )

    return ctx.report()


object_model = build_object_model()
