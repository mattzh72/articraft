import math
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
    model = ArticulatedObject(name="task_arm")

    base_mat = Material("base_mat", color=(0.3, 0.3, 0.3, 1.0))
    arm_mat = Material("arm_mat", color=(0.9, 0.5, 0.1, 1.0))
    pad_mat = Material("pad_mat", color=(0.1, 0.1, 0.1, 1.0))
    hub_mat = Material("hub_mat", color=(0.2, 0.2, 0.2, 1.0))

    # Base
    base = model.part("base")
    base.visual(Cylinder(0.12, 0.04), origin=Origin(xyz=(0.0, 0.0, 0.02)), material=base_mat, name="base_plate")
    base.visual(Cylinder(0.08, 0.062), origin=Origin(xyz=(0.0, 0.0, 0.069)), material=base_mat, name="base_pedestal")
    base.visual(Box((0.08, 0.08, 0.06)), origin=Origin(xyz=(0.0, 0.0, 0.12)), material=base_mat, name="base_column")
    base.visual(Cylinder(0.05, 0.12), origin=Origin(xyz=(0.0, 0.0, 0.15), rpy=(math.pi/2, 0.0, 0.0)), material=hub_mat, name="base_shoulder_hub")

    # Link 1 (Upper Arm)
    link1 = model.part("link1")
    link1.visual(Cylinder(0.05, 0.042), origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi/2, 0.0, 0.0)), material=hub_mat, name="link1_shoulder_hub")
    link1.visual(Box((0.06, 0.04, 0.402)), origin=Origin(xyz=(0.0, 0.021, 0.2)), material=arm_mat, name="link1_body")
    link1.visual(Cylinder(0.04, 0.042), origin=Origin(xyz=(0.0, 0.021, 0.4), rpy=(math.pi/2, 0.0, 0.0)), material=hub_mat, name="link1_elbow_hub")

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0, 0.06, 0.15)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-math.pi/2, upper=math.pi/2)
    )

    # Link 2 (Forearm)
    link2 = model.part("link2")
    link2.visual(Cylinder(0.04, 0.042), origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi/2, 0.0, 0.0)), material=hub_mat, name="link2_elbow_hub")
    link2.visual(Box((0.05, 0.04, 0.302)), origin=Origin(xyz=(0.0, 0.021, 0.15)), material=arm_mat, name="link2_body")
    link2.visual(Cylinder(0.03, 0.084), origin=Origin(xyz=(0.0, 0.0, 0.3), rpy=(math.pi/2, 0.0, 0.0)), material=hub_mat, name="link2_wrist_shaft")

    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(0.0, 0.042, 0.4)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-math.pi*0.8, upper=math.pi*0.8)
    )

    # Link 3 (Wrist/End Effector Mount)
    link3 = model.part("link3")
    link3.visual(Cylinder(0.03, 0.042), origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(math.pi/2, 0.0, 0.0)), material=hub_mat, name="link3_wrist_hub")
    link3.visual(Box((0.04, 0.04, 0.052)), origin=Origin(xyz=(0.0, -0.021, 0.025)), material=base_mat, name="link3_body")
    link3.visual(Box((0.10, 0.08, 0.02)), origin=Origin(xyz=(0.0, -0.04, 0.06)), material=pad_mat, name="pad")

    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(0.0, -0.042, 0.3)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-math.pi/2, upper=math.pi/2)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")

    # Verify the exact contact at the joints
    ctx.expect_contact(link1, base, elem_a="link1_shoulder_hub", elem_b="base_shoulder_hub", contact_tol=1e-4)
    ctx.expect_contact(link2, link1, elem_a="link2_elbow_hub", elem_b="link1_elbow_hub", contact_tol=1e-4)
    ctx.expect_contact(link2, link3, elem_a="link2_wrist_shaft", elem_b="link3_wrist_hub", contact_tol=1e-4)

    # Verify folding clearance: Link 3 passes Link 1 without collision
    ctx.expect_gap(link1, link3, axis="y", min_gap=0.001, max_gap=0.003, positive_elem="link1_body", negative_elem="link3_body")

    return ctx.report()


object_model = build_object_model()