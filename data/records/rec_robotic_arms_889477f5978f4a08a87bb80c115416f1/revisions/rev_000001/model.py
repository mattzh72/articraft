import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_arm")

    # Base
    base = model.part("base")
    base_geom = cq.Workplane("XY").cylinder(0.1, 0.15).translate((0, 0, 0.05))
    base.visual(mesh_from_cadquery(base_geom, "base_mesh"), color=(0.2, 0.2, 0.2, 1.0))

    # Shoulder
    shoulder = model.part("shoulder")
    shoulder_geom = (
        cq.Workplane("XY")
        .cylinder(0.05, 0.10)
        .translate((0, 0, 0.025))
        .union(
            cq.Workplane("XY")
            .box(0.10, 0.12, 0.11)
            .translate((0, 0, 0.095))
        )
        .faces(">Z")
        .edges("|Y")
        .fillet(0.049)
    ).cut(
        cq.Workplane("XY")
        .box(0.11, 0.08, 0.15)
        .translate((0, 0, 0.125))
    )
    shoulder.visual(mesh_from_cadquery(shoulder_geom, "shoulder_mesh"), color=(0.9, 0.4, 0.1, 1.0))

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder,
        origin=Origin(xyz=(0, 0, 0.1)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-3.14, upper=3.14)
    )

    # Upper Arm
    upper_arm = model.part("upper_arm")
    upper_arm_pivot = cq.Workplane("XZ").workplane(offset=-0.04).circle(0.04).extrude(0.08)
    upper_arm_body = cq.Workplane("XY").box(0.05, 0.07, 0.36).translate((0, 0, 0.17)).edges("|Z").fillet(0.01)
    upper_arm_neck = cq.Workplane("XY").box(0.04, 0.072, 0.09).translate((0, 0, 0.365))
    upper_arm_top_pivot = cq.Workplane("XZ", origin=(0, 0, 0.4)).workplane(offset=-0.036).circle(0.04).extrude(0.072)
    upper_arm_geom = upper_arm_pivot.union(upper_arm_body).union(upper_arm_neck).union(upper_arm_top_pivot)

    upper_arm.visual(mesh_from_cadquery(upper_arm_geom, "upper_arm_mesh"), color=(0.8, 0.8, 0.8, 1.0))

    model.articulation(
        "shoulder_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0, 0, 0.1)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(lower=-1.57, upper=1.57)
    )

    # Forearm
    forearm = model.part("forearm")
    forearm_base = (
        cq.Workplane("XZ")
        .cylinder(0.11, 0.055)
        .union(
            cq.Workplane("XY").box(0.06, 0.11, 0.05).translate((0, 0, 0.025))
        )
    ).cut(
        cq.Workplane("XZ").cylinder(0.072, 0.045)
    ).cut(
        cq.Workplane("XY").box(0.1, 0.072, 0.1).translate((0, 0, -0.05))
    )
    forearm_body = cq.Workplane("XY").box(0.04, 0.05, 0.24).translate((0, 0, 0.16)).edges("|Z").fillet(0.01)
    forearm_top_fork = (
        cq.Workplane("XZ")
        .center(0, 0.3)
        .cylinder(0.05, 0.025)
        .union(
            cq.Workplane("XY").box(0.04, 0.05, 0.06).translate((0, 0, 0.27))
        )
    ).cut(
        cq.Workplane("XZ").center(0, 0.3).cylinder(0.032, 0.03)
    ).cut(
        cq.Workplane("XY").box(0.1, 0.032, 0.1).translate((0, 0, 0.35))
    )
    forearm_geom = forearm_base.union(forearm_body).union(forearm_top_fork)

    forearm.visual(mesh_from_cadquery(forearm_geom, "forearm_mesh"), color=(0.9, 0.4, 0.1, 1.0))

    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0, 0, 0.4)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(lower=-2.5, upper=2.5)
    )

    # Wrist Pitch
    wrist_pitch = model.part("wrist_pitch")
    wrist_pitch_geom = (
        cq.Workplane("XZ")
        .workplane(offset=-0.016)
        .circle(0.02)
        .extrude(0.032)
        .union(
            cq.Workplane("XY").box(0.03, 0.032, 0.05).translate((0, 0, 0.015))
        )
    )
    wrist_pitch.visual(mesh_from_cadquery(wrist_pitch_geom, "wrist_pitch_mesh"), color=(0.2, 0.2, 0.2, 1.0))

    model.articulation(
        "forearm_to_wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_pitch,
        origin=Origin(xyz=(0, 0, 0.3)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(lower=-1.57, upper=1.57)
    )

    # Gripper Base (Wrist Roll)
    gripper_base = model.part("gripper_base")
    gripper_base_geom = (
        cq.Workplane("XY")
        .cylinder(0.02, 0.025)
        .translate((0, 0, 0.01))
        .union(
            cq.Workplane("XY").box(0.12, 0.03, 0.03).translate((0, 0, 0.025))
        )
    )
    gripper_base.visual(mesh_from_cadquery(gripper_base_geom, "gripper_base_mesh"), color=(0.8, 0.8, 0.8, 1.0))

    model.articulation(
        "wrist_pitch_to_gripper_base",
        ArticulationType.REVOLUTE,
        parent=wrist_pitch,
        child=gripper_base,
        origin=Origin(xyz=(0, 0, 0.04)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=-3.14, upper=3.14)
    )

    # Left Jaw
    left_jaw = model.part("left_jaw")
    jaw_base = cq.Workplane("XY").box(0.01, 0.03, 0.06).translate((0, 0, 0.03))
    left_pad = cq.Workplane("XY").box(0.015, 0.02, 0.02).translate((0.0125, 0, 0.05))
    left_jaw_geom = jaw_base.union(left_pad)
    left_jaw.visual(mesh_from_cadquery(left_jaw_geom, "left_jaw_mesh"), color=(0.1, 0.1, 0.1, 1.0))

    model.articulation(
        "gripper_base_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=gripper_base,
        child=left_jaw,
        origin=Origin(xyz=(-0.04, 0, 0.04)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=0.0, upper=0.02)
    )

    # Right Jaw
    right_jaw = model.part("right_jaw")
    right_pad = cq.Workplane("XY").box(0.015, 0.02, 0.02).translate((-0.0125, 0, 0.05))
    right_jaw_geom = jaw_base.union(right_pad)
    right_jaw.visual(mesh_from_cadquery(right_jaw_geom, "right_jaw_mesh"), color=(0.1, 0.1, 0.1, 1.0))

    model.articulation(
        "gripper_base_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=gripper_base,
        child=right_jaw,
        origin=Origin(xyz=(0.04, 0, 0.04)),
        axis=(-1, 0, 0),
        motion_limits=MotionLimits(lower=0.0, upper=0.02)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shoulder = object_model.get_part("shoulder")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_pitch = object_model.get_part("wrist_pitch")

    # Exact checks for assembly seating
    ctx.expect_gap(shoulder, base, axis="z", max_gap=0.001, max_penetration=0.001)

    # The pivot cylinders are authored to exactly match the fork gap widths to avoid floating parts.
    # This exact contact causes tiny meshing overlaps in the compiler QC.
    ctx.allow_overlap(shoulder, upper_arm, reason="Upper arm pivot exactly fills the shoulder fork gap.")
    ctx.allow_overlap(upper_arm, forearm, reason="Upper arm top pivot exactly fills the forearm base fork gap.")
    ctx.allow_overlap(forearm, wrist_pitch, reason="Wrist pitch pivot exactly fills the forearm top fork gap.")

    return ctx.report()

object_model = build_object_model()
