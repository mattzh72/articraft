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
    model = ArticulatedObject(name="dual_arm_ceiling_exam_lamp")

    white = model.material("satin_white", rgba=(0.93, 0.94, 0.92, 1.0))
    metal = model.material("brushed_metal", rgba=(0.62, 0.65, 0.66, 1.0))
    dark = model.material("dark_joint_rubber", rgba=(0.08, 0.09, 0.10, 1.0))
    diffuser = model.material("warm_translucent_diffuser", rgba=(1.0, 0.88, 0.55, 0.58))

    ceiling_plate = model.part("ceiling_plate")
    ceiling_plate.visual(
        Cylinder(radius=0.30, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=white,
        name="round_mount_plate",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(-0.15, 0.0, -0.009)),
        material=metal,
        name="shoulder_pad_0",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.15, 0.0, -0.009)),
        material=metal,
        name="shoulder_pad_1",
    )

    def make_branch(index: int, shoulder_x: float, shoulder_yaw: float) -> None:
        upper_arm = model.part(f"upper_arm_{index}")
        upper_arm.visual(
            Cylinder(radius=0.046, length=0.095),
            origin=Origin(xyz=(0.0, 0.0, -0.0475)),
            material=metal,
            name="shoulder_hub",
        )
        upper_arm.visual(
            Cylinder(radius=0.024, length=0.530),
            origin=Origin(xyz=(0.265, 0.0, -0.110), rpy=(0.0, pi / 2.0, 0.0)),
            material=white,
            name="upper_tube",
        )
        upper_arm.visual(
            Cylinder(radius=0.055, length=0.080),
            origin=Origin(xyz=(0.580, 0.0, -0.110)),
            material=dark,
            name="elbow_socket",
        )

        forearm = model.part(f"forearm_{index}")
        forearm.visual(
            Cylinder(radius=0.048, length=0.105),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark,
            name="elbow_hub",
        )
        forearm.visual(
            Cylinder(radius=0.023, length=0.560),
            origin=Origin(xyz=(0.280, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=white,
            name="forearm_tube",
        )
        forearm.visual(
            Cylinder(radius=0.043, length=0.070),
            origin=Origin(xyz=(0.585, 0.0, 0.0)),
            material=metal,
            name="distal_collar",
        )
        forearm.visual(
            Box((0.060, 0.050, 0.075)),
            origin=Origin(xyz=(0.585, 0.0, -0.0375)),
            material=metal,
            name="yoke_neck",
        )
        forearm.visual(
            Cylinder(radius=0.018, length=0.540),
            origin=Origin(xyz=(0.585, 0.0, -0.075), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="yoke_bridge",
        )
        forearm.visual(
            Box((0.055, 0.035, 0.260)),
            origin=Origin(xyz=(0.585, 0.245, -0.200)),
            material=metal,
            name="yoke_ear_pos",
        )
        forearm.visual(
            Box((0.055, 0.035, 0.260)),
            origin=Origin(xyz=(0.585, -0.245, -0.200)),
            material=metal,
            name="yoke_ear_neg",
        )

        lamp_head = model.part(f"lamp_head_{index}")
        lamp_head.visual(
            Cylinder(radius=0.205, length=0.090),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=white,
            name="round_head_shell",
        )
        lamp_head.visual(
            Cylinder(radius=0.026, length=0.560),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="tilt_trunnion",
        )
        lamp_head.visual(
            Cylinder(radius=0.172, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.052)),
            material=diffuser,
            name="diffuser_lens",
        )
        lamp_head.visual(
            Cylinder(radius=0.030, length=0.110),
            origin=Origin(xyz=(0.0, 0.0, -0.107)),
            material=dark,
            name="sterile_handle",
        )
        lamp_head.visual(
            Cylinder(radius=0.080, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.053)),
            material=metal,
            name="rear_service_cap",
        )

        model.articulation(
            f"shoulder_{index}",
            ArticulationType.REVOLUTE,
            parent=ceiling_plate,
            child=upper_arm,
            origin=Origin(xyz=(shoulder_x, 0.0, -0.018), rpy=(0.0, 0.0, shoulder_yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=25.0, velocity=1.3, lower=-2.6, upper=2.6),
        )
        model.articulation(
            f"elbow_{index}",
            ArticulationType.REVOLUTE,
            parent=upper_arm,
            child=forearm,
            origin=Origin(xyz=(0.580, 0.0, -0.110)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-2.4, upper=2.4),
        )
        model.articulation(
            f"tilt_{index}",
            ArticulationType.REVOLUTE,
            parent=forearm,
            child=lamp_head,
            origin=Origin(xyz=(0.585, 0.0, -0.300)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.65, upper=0.65),
        )

    make_branch(0, shoulder_x=-0.15, shoulder_yaw=pi)
    make_branch(1, shoulder_x=0.15, shoulder_yaw=0.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plate = object_model.get_part("ceiling_plate")
    for index in (0, 1):
        upper_arm = object_model.get_part(f"upper_arm_{index}")
        forearm = object_model.get_part(f"forearm_{index}")
        lamp_head = object_model.get_part(f"lamp_head_{index}")
        shoulder = object_model.get_articulation(f"shoulder_{index}")
        elbow = object_model.get_articulation(f"elbow_{index}")
        tilt = object_model.get_articulation(f"tilt_{index}")

        ctx.allow_overlap(
            upper_arm,
            forearm,
            elem_a="elbow_socket",
            elem_b="elbow_hub",
            reason="The coaxial elbow hub is intentionally nested in the upper-arm elbow socket.",
        )
        ctx.allow_overlap(
            upper_arm,
            forearm,
            elem_a="elbow_socket",
            elem_b="forearm_tube",
            reason="The forearm tube deliberately enters the elbow socket as a short captured bearing insert.",
        )
        ctx.allow_overlap(
            forearm,
            lamp_head,
            elem_a="yoke_ear_pos",
            elem_b="tilt_trunnion",
            reason="The lamp-head trunnion is intentionally captured in the positive-side yoke ear.",
        )
        ctx.allow_overlap(
            forearm,
            lamp_head,
            elem_a="yoke_ear_neg",
            elem_b="tilt_trunnion",
            reason="The lamp-head trunnion is intentionally captured in the negative-side yoke ear.",
        )

        ctx.expect_gap(
            plate,
            upper_arm,
            axis="z",
            max_gap=0.001,
            max_penetration=0.020,
            positive_elem=f"shoulder_pad_{index}",
            negative_elem="shoulder_hub",
            name=f"shoulder_{index} hub is seated under the ceiling plate",
        )
        ctx.expect_overlap(
            upper_arm,
            forearm,
            axes="xyz",
            min_overlap=0.015,
            elem_a="elbow_socket",
            elem_b="elbow_hub",
            name=f"elbow_{index} bearing remains coaxially engaged",
        )
        ctx.expect_overlap(
            upper_arm,
            forearm,
            axes="xyz",
            min_overlap=0.010,
            elem_a="elbow_socket",
            elem_b="forearm_tube",
            name=f"forearm_{index} tube is captured by the elbow socket",
        )
        ctx.expect_overlap(
            forearm,
            lamp_head,
            axes="xyz",
            min_overlap=0.010,
            elem_a="yoke_ear_pos",
            elem_b="tilt_trunnion",
            name=f"tilt_{index} trunnion engages positive yoke ear",
        )
        ctx.expect_overlap(
            forearm,
            lamp_head,
            axes="xyz",
            min_overlap=0.010,
            elem_a="yoke_ear_neg",
            elem_b="tilt_trunnion",
            name=f"tilt_{index} trunnion engages negative yoke ear",
        )

        with ctx.pose({shoulder: 0.45, elbow: -0.80, tilt: 0.55}):
            ctx.expect_gap(
                forearm,
                lamp_head,
                axis="z",
                min_gap=0.020,
                positive_elem="yoke_bridge",
                negative_elem="round_head_shell",
                name=f"lamp_head_{index} tilts without striking the yoke bridge",
            )

    ctx.check(
        "two independent articulated branches",
        all(object_model.get_articulation(name) is not None for name in (
            "shoulder_0",
            "elbow_0",
            "tilt_0",
            "shoulder_1",
            "elbow_1",
            "tilt_1",
        )),
        details="Expected shoulder, elbow, and head-tilt revolute joints on both ceiling-plate branches.",
    )

    return ctx.report()


object_model = build_object_model()
