import math
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
    model = ArticulatedObject(name="robotic_leg")

    # 1. Hip Support (Root)
    hip_support = model.part("hip_support")
    hip_support.visual(
        Box((0.12, 0.08, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.6)),
        name="support_body"
    )
    hip_support.visual(
        Cylinder(radius=0.04, length=0.02),
        origin=Origin(xyz=(0.0, 0.05, 0.6), rpy=(math.pi/2, 0.0, 0.0)),
        name="hip_hub"
    )

    # 2. Thigh
    thigh = model.part("thigh")
    thigh.visual(
        Cylinder(radius=0.04, length=0.04),
        origin=Origin(xyz=(0.0, 0.02, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="thigh_top_hub"
    )
    thigh.visual(
        Cylinder(radius=0.035, length=0.04),
        origin=Origin(xyz=(0.0, 0.02, -0.28), rpy=(math.pi/2, 0.0, 0.0)),
        name="thigh_bottom_hub"
    )
    thigh.visual(
        Box((0.06, 0.04, 0.28)),
        origin=Origin(xyz=(0.0, 0.02, -0.14)),
        name="thigh_body"
    )

    model.articulation(
        "hip_joint",
        ArticulationType.REVOLUTE,
        parent=hip_support,
        child=thigh,
        origin=Origin(xyz=(0.0, 0.06, 0.6)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.57, upper=1.57)
    )

    # 3. Shank
    shank = model.part("shank")
    shank.visual(
        Cylinder(radius=0.035, length=0.04),
        origin=Origin(xyz=(0.0, 0.02, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="shank_top_hub"
    )
    shank.visual(
        Cylinder(radius=0.03, length=0.04),
        origin=Origin(xyz=(0.0, 0.02, -0.28), rpy=(math.pi/2, 0.0, 0.0)),
        name="shank_bottom_hub"
    )
    shank.visual(
        Box((0.05, 0.04, 0.28)),
        origin=Origin(xyz=(0.0, 0.02, -0.14)),
        name="shank_body"
    )

    model.articulation(
        "knee_joint",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.04, -0.28)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-2.5, upper=0.0)
    )

    # 4. Foot
    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.03, length=0.04),
        origin=Origin(xyz=(0.0, -0.02, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        name="foot_ankle_hub"
    )
    foot.visual(
        Box((0.14, 0.06, 0.04)),
        origin=Origin(xyz=(0.04, -0.03, -0.02)),
        name="foot_body"
    )

    model.articulation(
        "ankle_joint",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.0, upper=1.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hip_support = object_model.get_part("hip_support")
    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")

    ctx.allow_coplanar_surfaces(hip_support, thigh, elem_a="hip_hub", elem_b="thigh_top_hub", reason="Joint interface")
    ctx.allow_coplanar_surfaces(thigh, shank, elem_a="thigh_bottom_hub", elem_b="shank_top_hub", reason="Joint interface")
    ctx.allow_coplanar_surfaces(shank, foot, elem_a="shank_bottom_hub", elem_b="foot_ankle_hub", reason="Joint interface")

    ctx.expect_contact(hip_support, thigh, elem_a="hip_hub", elem_b="thigh_top_hub", name="hip touches thigh")
    ctx.expect_contact(thigh, shank, elem_a="thigh_bottom_hub", elem_b="shank_top_hub", name="thigh touches shank")
    ctx.expect_contact(shank, foot, elem_a="shank_bottom_hub", elem_b="foot_ankle_hub", name="shank touches foot")

    hip_joint = object_model.get_articulation("hip_joint")
    knee_joint = object_model.get_articulation("knee_joint")
    ankle_joint = object_model.get_articulation("ankle_joint")

    with ctx.pose({hip_joint: 1.0, knee_joint: -1.0, ankle_joint: 0.5}):
        ctx.expect_contact(hip_support, thigh, elem_a="hip_hub", elem_b="thigh_top_hub", name="hip touches thigh in pose")
        ctx.expect_contact(thigh, shank, elem_a="thigh_bottom_hub", elem_b="shank_top_hub", name="thigh touches shank in pose")
        ctx.expect_contact(shank, foot, elem_a="shank_bottom_hub", elem_b="foot_ankle_hub", name="shank touches foot in pose")

    return ctx.report()

object_model = build_object_model()