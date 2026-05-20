from __future__ import annotations

from math import cos, pi, sin

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
    model = ArticulatedObject(name="bench_robot_arm")

    model.material("cast_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("warm_gray_casting", rgba=(0.47, 0.50, 0.51, 1.0))
    model.material("painted_blue", rgba=(0.08, 0.20, 0.42, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("black_rubber", rgba=(0.015, 0.016, 0.017, 1.0))
    model.material("service_black", rgba=(0.04, 0.045, 0.05, 1.0))

    base = model.part("bench_base")
    base.visual(
        Box((0.46, 0.36, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="cast_graphite",
        name="bench_plate",
    )
    base.visual(
        Cylinder(radius=0.165, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material="machined_steel",
        name="bearing_ring",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material="service_black",
        name="bearing_shadow",
    )
    for i, (x_pos, y_pos) in enumerate(
        [(-0.17, -0.12), (-0.17, 0.12), (0.17, -0.12), (0.17, 0.12)]
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.007),
            origin=Origin(xyz=(x_pos, y_pos, 0.0375)),
            material="machined_steel",
            name=f"anchor_bolt_{i}",
        )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="painted_blue",
        name="turntable_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.092, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material="painted_blue",
        name="yaw_column",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.276)),
        material="machined_steel",
        name="top_service_cap",
    )
    pedestal.visual(
        Box((0.045, 0.170, 0.130)),
        origin=Origin(xyz=(0.0875, 0.0, 0.310)),
        material="painted_blue",
        name="mount_pad",
    )
    pedestal.visual(
        Cylinder(radius=0.057, length=0.012),
        origin=Origin(xyz=(0.104, 0.052, 0.338), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="pad_boss_0",
    )
    pedestal.visual(
        Cylinder(radius=0.057, length=0.012),
        origin=Origin(xyz=(0.104, -0.052, 0.338), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="pad_boss_1",
    )
    pedestal.visual(
        Box((0.050, 0.130, 0.028)),
        origin=Origin(xyz=(0.055, 0.0, 0.248)),
        material="service_black",
        name="cable_cover",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.075, length=0.150),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_gray_casting",
        name="prox_housing",
    )
    upper_arm.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.075, 0.081, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="prox_cap_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.075, -0.081, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="prox_cap_1",
    )
    upper_arm.visual(
        Box((0.455, 0.032, 0.074)),
        origin=Origin(xyz=(0.314, 0.056, 0.0)),
        material="warm_gray_casting",
        name="side_rail_0",
    )
    upper_arm.visual(
        Box((0.455, 0.032, 0.074)),
        origin=Origin(xyz=(0.314, -0.056, 0.0)),
        material="warm_gray_casting",
        name="side_rail_1",
    )
    upper_arm.visual(
        Box((0.210, 0.090, 0.088)),
        origin=Origin(xyz=(0.310, 0.0, 0.0)),
        material="service_black",
        name="recessed_web",
    )
    upper_arm.visual(
        Cylinder(radius=0.080, length=0.035),
        origin=Origin(xyz=(0.560, 0.062, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_gray_casting",
        name="distal_ear_pos",
    )
    upper_arm.visual(
        Cylinder(radius=0.080, length=0.035),
        origin=Origin(xyz=(0.560, -0.062, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="warm_gray_casting",
        name="distal_ear_neg",
    )
    upper_arm.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.560, 0.084, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="elbow_cap_0",
    )
    upper_arm.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.560, -0.084, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="elbow_cap_1",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.065, length=0.089),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="painted_blue",
        name="elbow_lug",
    )
    forearm.visual(
        Box((0.340, 0.058, 0.064)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material="painted_blue",
        name="forearm_spine",
    )
    forearm.visual(
        Box((0.250, 0.026, 0.078)),
        origin=Origin(xyz=(0.215, 0.0, 0.043)),
        material="service_black",
        name="top_service_cover",
    )
    forearm.visual(
        Box((0.210, 0.022, 0.020)),
        origin=Origin(xyz=(0.215, 0.038, -0.024)),
        material="machined_steel",
        name="side_rail_pos",
    )
    forearm.visual(
        Box((0.210, 0.022, 0.020)),
        origin=Origin(xyz=(0.215, -0.038, -0.024)),
        material="machined_steel",
        name="side_rail_neg",
    )
    forearm.visual(
        Cylinder(radius=0.055, length=0.070),
        origin=Origin(xyz=(0.385, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_gray_casting",
        name="wrist_bearing",
    )
    forearm.visual(
        Cylinder(radius=0.038, length=0.008),
        origin=Origin(xyz=(0.418, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="bearing_face",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.045, length=0.120),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="warm_gray_casting",
        name="roll_cartridge",
    )
    wrist.visual(
        Box((0.078, 0.050, 0.020)),
        origin=Origin(xyz=(0.060, 0.0, 0.043)),
        material="service_black",
        name="wrist_service_cap",
    )
    wrist.visual(
        Cylinder(radius=0.065, length=0.024),
        origin=Origin(xyz=(0.130, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="tool_flange",
    )
    wrist.visual(
        Cylinder(radius=0.043, length=0.007),
        origin=Origin(xyz=(0.1445, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="service_black",
        name="flange_center_bore",
    )
    bolt_radius = 0.045
    for i in range(6):
        angle = i * pi / 3.0
        wrist.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(
                xyz=(0.145, bolt_radius * cos(angle), bolt_radius * sin(angle)),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="cast_graphite",
            name=f"flange_bolt_{i}",
        )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.70, upper=2.70, effort=180.0, velocity=1.20),
    )
    model.articulation(
        "upper_mount",
        ArticulationType.FIXED,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.110, 0.0, 0.310)),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.560, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=1.25, effort=95.0, velocity=1.40),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.422, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-2.60, upper=2.60, effort=32.0, velocity=2.50),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("bench_base")
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    shoulder = object_model.get_articulation("shoulder_yaw")
    elbow = object_model.get_articulation("elbow")
    wrist_roll = object_model.get_articulation("wrist_roll")

    ctx.check(
        "industrial serial chain has requested parts",
        all(p is not None for p in (base, pedestal, upper_arm, forearm, wrist)),
    )
    ctx.check(
        "joint axes match requested mechanisms",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and wrist_roll.axis == (1.0, 0.0, 0.0),
        details=f"axes: shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist_roll.axis}",
    )

    ctx.expect_gap(
        pedestal,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="yaw turntable seats on bearing ring",
    )
    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="x",
        positive_elem="prox_housing",
        negative_elem="mount_pad",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper arm housing bolts to pedestal pad",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem="distal_ear_pos",
        negative_elem="elbow_lug",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="positive elbow fork captures central lug without overlap",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_lug",
        negative_elem="distal_ear_neg",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="negative elbow fork captures central lug without overlap",
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="x",
        positive_elem="roll_cartridge",
        negative_elem="bearing_face",
        max_gap=0.001,
        max_penetration=0.0,
        name="wrist cartridge seats against forearm bearing",
    )

    wrist_rest = ctx.part_world_position(wrist)
    with ctx.pose({shoulder: 1.1, elbow: 0.95, wrist_roll: 1.4}):
        wrist_raised = ctx.part_world_position(wrist)
        ctx.expect_contact(
            wrist,
            forearm,
            elem_a="roll_cartridge",
            elem_b="bearing_face",
            contact_tol=0.001,
            name="rolled wrist remains seated in raised pose",
        )
        ctx.expect_gap(
            wrist,
            base,
            axis="z",
            min_gap=0.18,
            name="raised terminal module clears bench base",
        )

    ctx.check(
        "elbow upper pose raises terminal module",
        wrist_rest is not None
        and wrist_raised is not None
        and wrist_raised[2] > wrist_rest[2] + 0.20,
        details=f"rest={wrist_rest}, raised={wrist_raised}",
    )

    with ctx.pose({elbow: -0.65, wrist_roll: -1.8}):
        ctx.expect_contact(
            wrist,
            forearm,
            elem_a="roll_cartridge",
            elem_b="bearing_face",
            contact_tol=0.001,
            name="rolled wrist remains seated in lowered pose",
        )
        ctx.expect_gap(
            wrist,
            base,
            axis="x",
            min_gap=0.45,
            name="lowered wrist stays forward of bench base",
        )

    return ctx.report()


object_model = build_object_model()
