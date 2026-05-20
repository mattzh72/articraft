from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ClevisBracketGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_arm")

    base = model.part("base")
    fork_geom = ClevisBracketGeometry(
        (0.12, 0.12, 0.15),
        gap_width=0.06,
        bore_diameter=0.02,
        bore_center_z=0.10,
        base_thickness=0.02,
        center=False,
    )
    base.visual(mesh_from_geometry(fork_geom, "base_fork"), name="fork")

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Box((0.06, 0.05, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        name="upper_arm_shell",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-1.57, upper=1.57),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Box((0.04, 0.04, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        name="forearm_shell",
    )

    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-2.0, upper=2.0),
    )

    wrist_flange = model.part("wrist_flange")
    wrist_flange.visual(
        Cylinder(radius=0.03, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="flange_body",
    )

    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_flange,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # We allow overlap between base and upper_arm since upper_arm is captured in the fork
    ctx.allow_overlap(
        "base", "upper_arm",
        reason="Upper arm is captured in the base clevis fork."
    )

    ctx.expect_contact("upper_arm", "base", name="upper arm is mounted in the base fork")
    ctx.expect_contact("forearm", "upper_arm", name="forearm is mounted to upper arm")
    ctx.expect_contact("wrist_flange", "forearm", name="wrist flange is mounted to forearm")

    return ctx.report()


object_model = build_object_model()
