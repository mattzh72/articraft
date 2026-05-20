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
    model = ArticulatedObject(name="four_link_deployable_arm")

    aluminum = model.material("clear_anodized_aluminum", rgba=(0.72, 0.76, 0.78, 1.0))
    graphite = model.material("dark_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    joint_black = model.material("black_bearing", rgba=(0.01, 0.012, 0.014, 1.0))
    amber = model.material("amber_joint_bushing", rgba=(0.92, 0.54, 0.12, 1.0))
    glass = model.material("blue_black_glass", rgba=(0.02, 0.07, 0.12, 0.85))

    def add_pitch_link(part, length: float, *, final: bool = False) -> None:
        """A light rectangular arm section with a central proximal lug."""
        beam_start = 0.030
        beam_end = length if final else length - 0.065
        beam_length = beam_end - beam_start
        part.visual(
            Box((beam_length, 0.038, 0.028)),
            origin=Origin(xyz=(beam_start + beam_length * 0.5, 0.0, 0.0)),
            material=aluminum,
            name="beam",
        )
        recess_length = max(beam_length - 0.080, 0.10)
        part.visual(
            Box((recess_length, 0.026, 0.006)),
            origin=Origin(xyz=(beam_start + 0.055 + recess_length * 0.5, 0.0, 0.017)),
            material=graphite,
            name="top_recess",
        )
        part.visual(
            Cylinder(radius=0.045, length=0.050),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=joint_black,
            name="proximal_lug",
        )
        part.visual(
            Cylinder(radius=0.024, length=0.058),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=amber,
            name="proximal_bushing",
        )

        if final:
            part.visual(
                Box((0.055, 0.070, 0.045)),
                origin=Origin(xyz=(length - 0.0275, 0.0, 0.0)),
                material=joint_black,
                name="tip_clamp",
            )
            return

        # Distal fork: the following link's proximal lug sits between these ears.
        part.visual(
            Box((0.130, 0.024, 0.074)),
            origin=Origin(xyz=(length - 0.005, 0.052, 0.0)),
            material=aluminum,
            name="distal_ear_pos",
        )
        part.visual(
            Box((0.130, 0.024, 0.074)),
            origin=Origin(xyz=(length - 0.005, -0.052, 0.0)),
            material=aluminum,
            name="distal_ear_neg",
        )
        part.visual(
            Box((0.050, 0.128, 0.040)),
            origin=Origin(xyz=(length - 0.075, 0.0, 0.0)),
            material=aluminum,
            name="distal_bridge",
        )
        part.visual(
            Cylinder(radius=0.034, length=0.008),
            origin=Origin(xyz=(length, 0.068, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=joint_black,
            name="distal_cap_pos",
        )
        part.visual(
            Cylinder(radius=0.034, length=0.008),
            origin=Origin(xyz=(length, -0.068, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=joint_black,
            name="distal_cap_neg",
        )
        part.visual(
            Cylinder(radius=0.013, length=0.132),
            origin=Origin(xyz=(length, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=amber,
            name="distal_pin",
        )

    base = model.part("base")
    base.visual(
        Box((0.24, 0.18, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.134),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=aluminum,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.086, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=joint_black,
        name="turntable",
    )

    link_0 = model.part("link_0")
    link_0.visual(
        Box((0.415, 0.038, 0.028)),
        origin=Origin(xyz=(0.2075, 0.0, 0.0)),
        material=aluminum,
        name="beam",
    )
    link_0.visual(
        Box((0.335, 0.026, 0.006)),
        origin=Origin(xyz=(0.2225, 0.0, 0.017)),
        material=graphite,
        name="top_recess",
    )
    link_0.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(),
        material=joint_black,
        name="yaw_hub",
    )
    link_0.visual(
        Cylinder(radius=0.030, length=0.052),
        origin=Origin(),
        material=amber,
        name="yaw_bushing",
    )
    link_0.visual(
        Box((0.130, 0.024, 0.074)),
        origin=Origin(xyz=(0.480 - 0.005, 0.052, 0.0)),
        material=aluminum,
        name="distal_ear_pos",
    )
    link_0.visual(
        Box((0.130, 0.024, 0.074)),
        origin=Origin(xyz=(0.480 - 0.005, -0.052, 0.0)),
        material=aluminum,
        name="distal_ear_neg",
    )
    link_0.visual(
        Box((0.050, 0.128, 0.040)),
        origin=Origin(xyz=(0.480 - 0.075, 0.0, 0.0)),
        material=aluminum,
        name="distal_bridge",
    )
    link_0.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.480, 0.068, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_black,
        name="distal_cap_pos",
    )
    link_0.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.480, -0.068, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=joint_black,
        name="distal_cap_neg",
    )
    link_0.visual(
        Cylinder(radius=0.013, length=0.132),
        origin=Origin(xyz=(0.480, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="distal_pin",
    )

    link_1 = model.part("link_1")
    add_pitch_link(link_1, 0.420)

    link_2 = model.part("link_2")
    add_pitch_link(link_2, 0.340)

    link_3 = model.part("link_3")
    add_pitch_link(link_3, 0.260, final=True)

    sensor_pod = model.part("sensor_pod")
    sensor_pod.visual(
        Box((0.040, 0.058, 0.038)),
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        material=joint_black,
        name="mount_foot",
    )
    sensor_pod.visual(
        Box((0.090, 0.066, 0.054)),
        origin=Origin(xyz=(0.083, 0.0, 0.0)),
        material=graphite,
        name="pod_body",
    )
    sensor_pod.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.135, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-1.05, upper=1.45),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "tip_sensor_mount",
        ArticulationType.FIXED,
        parent=link_3,
        child=sensor_pod,
        origin=Origin(xyz=(0.260, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for parent_name, child_name, joint_label in (
        ("link_0", "link_1", "shoulder"),
        ("link_1", "link_2", "elbow"),
        ("link_2", "link_3", "wrist"),
    ):
        ctx.allow_overlap(
            parent_name,
            child_name,
            elem_a="distal_pin",
            elem_b="proximal_lug",
            reason=f"The {joint_label} hinge pin is intentionally captured through the lug bore.",
        )
        ctx.allow_overlap(
            parent_name,
            child_name,
            elem_a="distal_pin",
            elem_b="proximal_bushing",
            reason=f"The {joint_label} hinge pin is intentionally nested inside the visible bushing sleeve.",
        )

    revolute_names = [
        joint.name
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "four serial revolute joints",
        revolute_names == ["base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch"],
        details=f"revolute joints were {revolute_names}",
    )

    sensor_joint = object_model.get_articulation("tip_sensor_mount")
    ctx.check(
        "sensor pod is fixed at the tip",
        sensor_joint.articulation_type == ArticulationType.FIXED
        and sensor_joint.parent == "link_3"
        and sensor_joint.child == "sensor_pod",
        details=f"sensor joint={sensor_joint}",
    )

    ctx.expect_contact(
        "sensor_pod",
        "link_3",
        elem_a="mount_foot",
        elem_b="tip_clamp",
        contact_tol=1e-5,
        name="sensor foot seats on terminal clamp",
    )
    ctx.expect_gap(
        "link_0",
        "link_1",
        axis="y",
        positive_elem="distal_ear_pos",
        negative_elem="proximal_lug",
        min_gap=0.010,
        name="shoulder lug clears positive fork ear",
    )
    ctx.expect_gap(
        "link_1",
        "link_2",
        axis="y",
        positive_elem="distal_ear_pos",
        negative_elem="proximal_lug",
        min_gap=0.010,
        name="elbow lug clears positive fork ear",
    )
    ctx.expect_gap(
        "link_2",
        "link_3",
        axis="y",
        positive_elem="distal_ear_pos",
        negative_elem="proximal_lug",
        min_gap=0.010,
        name="wrist lug clears positive fork ear",
    )
    ctx.expect_overlap(
        "link_0",
        "link_1",
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_lug",
        min_overlap=0.045,
        name="shoulder pin spans the lug",
    )
    ctx.expect_overlap(
        "link_1",
        "link_2",
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_lug",
        min_overlap=0.045,
        name="elbow pin spans the lug",
    )
    ctx.expect_overlap(
        "link_2",
        "link_3",
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_lug",
        min_overlap=0.045,
        name="wrist pin spans the lug",
    )
    ctx.expect_overlap(
        "link_0",
        "link_1",
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_bushing",
        min_overlap=0.050,
        name="shoulder pin passes through bushing",
    )
    ctx.expect_overlap(
        "link_1",
        "link_2",
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_bushing",
        min_overlap=0.050,
        name="elbow pin passes through bushing",
    )
    ctx.expect_overlap(
        "link_2",
        "link_3",
        axes="y",
        elem_a="distal_pin",
        elem_b="proximal_bushing",
        min_overlap=0.050,
        name="wrist pin passes through bushing",
    )

    sensor = object_model.get_part("sensor_pod")
    rest_tip = ctx.part_world_position(sensor)
    with ctx.pose(
        {
            "base_yaw": 0.55,
            "shoulder_pitch": 0.70,
            "elbow_pitch": -0.35,
            "wrist_pitch": 0.45,
        }
    ):
        deployed_tip = ctx.part_world_position(sensor)
    ctx.check(
        "deploy pose moves the sensor outward and upward",
        rest_tip is not None
        and deployed_tip is not None
        and deployed_tip[1] > rest_tip[1] + 0.30
        and deployed_tip[2] > rest_tip[2] + 0.20,
        details=f"rest={rest_tip}, deployed={deployed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
