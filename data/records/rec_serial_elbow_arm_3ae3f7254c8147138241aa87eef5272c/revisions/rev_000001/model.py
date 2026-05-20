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
    Material,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_shoulder_elbow_arm")

    mat_base = Material("base_mat", color=(0.2, 0.2, 0.2))
    mat_upper = Material("upper_mat", color=(0.8, 0.3, 0.1))
    mat_fore = Material("fore_mat", color=(0.1, 0.5, 0.8))
    mat_joint = Material("joint_mat", color=(0.4, 0.4, 0.4))
    mat_face = Material("face_mat", color=(0.8, 0.8, 0.8))

    # 1. Base
    base = model.part("base")
    base.visual(
        Box((0.2, 0.2, 0.2)),
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        material=mat_base,
        name="base_block"
    )
    # Shaft extending out to capture the shoulder hub
    base.visual(
        Cylinder(radius=0.04, length=0.018),
        origin=Origin(xyz=(0.0, 0.109, 0.15), rpy=(math.pi/2, 0.0, 0.0)),
        material=mat_joint,
        name="shoulder_shaft"
    )

    # 2. Upper Link
    upper_link = model.part("upper_link")
    # Hub at shoulder
    upper_link.visual(
        Cylinder(radius=0.06, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        material=mat_upper,
        name="upper_shoulder_hub"
    )
    # Main body
    upper_link.visual(
        Box((0.12, 0.04, 0.3)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=mat_upper,
        name="upper_body"
    )
    # Hub at elbow
    upper_link.visual(
        Cylinder(radius=0.06, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.3), rpy=(math.pi/2, 0.0, 0.0)),
        material=mat_upper,
        name="upper_elbow_hub"
    )
    # Shaft extending out to capture the forelink hub
    upper_link.visual(
        Cylinder(radius=0.03, length=0.018),
        origin=Origin(xyz=(0.0, 0.029, 0.3), rpy=(math.pi/2, 0.0, 0.0)),
        material=mat_joint,
        name="elbow_shaft"
    )

    # 3. Shoulder Articulation
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.14, 0.15)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.5, upper=1.5, effort=10.0, velocity=2.0),
    )

    # 4. Forelink
    forelink = model.part("forelink")
    # Hub at elbow
    forelink.visual(
        Cylinder(radius=0.04, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0.0, 0.0)),
        material=mat_fore,
        name="fore_elbow_hub"
    )
    # Main body
    forelink.visual(
        Box((0.06, 0.03, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=mat_fore,
        name="fore_body"
    )
    # Faceplate
    forelink.visual(
        Box((0.08, 0.08, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=mat_face,
        name="faceplate"
    )

    # 5. Elbow Articulation
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forelink,
        origin=Origin(xyz=(0.0, 0.055, 0.3)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.5, upper=2.5, effort=5.0, velocity=3.0),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allow overlaps for captured shafts
    ctx.allow_overlap(
        "base", "upper_link",
        elem_a="shoulder_shaft", elem_b="upper_shoulder_hub",
        reason="Shoulder shaft is captured inside the upper link hub."
    )
    ctx.allow_overlap(
        "upper_link", "forelink",
        elem_a="elbow_shaft", elem_b="fore_elbow_hub",
        reason="Elbow shaft is captured inside the forelink hub."
    )

    # Verify gaps to ensure parts don't collide outside the shaft
    ctx.expect_gap(
        "upper_link", "base",
        axis="y",
        positive_elem="upper_shoulder_hub",
        negative_elem="base_block",
        min_gap=0.005,
        max_gap=0.015,
        name="gap between upper link and base"
    )
    ctx.expect_gap(
        "forelink", "upper_link",
        axis="y",
        positive_elem="fore_elbow_hub",
        negative_elem="upper_elbow_hub",
        min_gap=0.002,
        max_gap=0.010,
        name="gap between forelink and upper link"
    )

    # Check pose behavior
    with ctx.pose(shoulder_pitch=1.0, elbow_pitch=1.0):
        # Just ensure it poses without errors
        pass

    return ctx.report()

object_model = build_object_model()
