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
    model = ArticulatedObject(name="two_joint_revolute_chain")

    brushed = Material("brushed_aluminum", color=(0.72, 0.74, 0.74, 1.0))
    dark_steel = Material("dark_steel", color=(0.18, 0.19, 0.20, 1.0))
    rubber = Material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.22, 0.18, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_steel,
        name="mounting_plate",
    )
    for y, name in ((-0.055, "bracket_cheek_0"), (0.055, "bracket_cheek_1")):
        base.visual(
            Box((0.060, 0.018, 0.200)),
            origin=Origin(xyz=(0.0, y, 0.110)),
            material=brushed,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.012, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.160), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_pin",
    )

    first_link = model.part("first_link")
    first_link.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="shoulder_hub",
    )
    first_link.visual(
        Box((0.340, 0.035, 0.030)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=brushed,
        name="main_beam",
    )
    first_link.visual(
        Box((0.020, 0.090, 0.035)),
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        material=brushed,
        name="elbow_bridge",
    )
    for y, name in ((-0.038, "elbow_fork_0"), (0.038, "elbow_fork_1")):
        first_link.visual(
            Box((0.080, 0.018, 0.055)),
            origin=Origin(xyz=(0.400, y, 0.0)),
            material=brushed,
            name=name,
        )
    first_link.visual(
        Cylinder(radius=0.011, length=0.105),
        origin=Origin(xyz=(0.400, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="elbow_pin",
    )

    second_link = model.part("second_link")
    second_link.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="elbow_hub",
    )
    second_link.visual(
        Box((0.300, 0.030, 0.026)),
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        material=brushed,
        name="outer_beam",
    )
    second_link.visual(
        Cylinder(radius=0.045, length=0.025),
        origin=Origin(xyz=(0.3425, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="end_pad",
    )

    travel = MotionLimits(effort=12.0, velocity=2.0, lower=-pi / 2.0, upper=pi / 2.0)
    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=travel,
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(0.400, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=travel,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder_joint")
    elbow = object_model.get_articulation("elbow_joint")

    ctx.allow_overlap(
        "base_bracket",
        "first_link",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The shoulder pin is intentionally captured through the rotating hub.",
    )
    ctx.allow_overlap(
        "first_link",
        "second_link",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The elbow pin is intentionally captured through the second-link hub.",
    )

    ctx.check(
        "two serial revolutes",
        len(object_model.articulations) == 2
        and shoulder.parent == "base_bracket"
        and shoulder.child == "first_link"
        and elbow.parent == "first_link"
        and elbow.child == "second_link",
        details="Expected base -> first_link -> second_link kinematic chain.",
    )
    ctx.check(
        "parallel in-plane joint axes",
        shoulder.axis == (0.0, 1.0, 0.0) and elbow.axis == (0.0, 1.0, 0.0),
        details=f"axes were shoulder={shoulder.axis}, elbow={elbow.axis}",
    )
    for joint in (shoulder, elbow):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has ninety degree travel each way",
            limits is not None
            and abs(limits.lower + pi / 2.0) < 1.0e-6
            and abs(limits.upper - pi / 2.0) < 1.0e-6,
            details=f"limits were {limits}",
        )

    ctx.expect_overlap(
        "base_bracket",
        "first_link",
        axes="xyz",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.020,
        name="shoulder pin is retained in hub",
    )
    ctx.expect_overlap(
        "first_link",
        "second_link",
        axes="xyz",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.020,
        name="elbow pin is retained in hub",
    )
    with ctx.pose({shoulder: pi / 2.0, elbow: -pi / 2.0}):
        ctx.expect_overlap(
            "first_link",
            "second_link",
            axes="xz",
            elem_a="elbow_fork_0",
            elem_b="elbow_hub",
            min_overlap=0.020,
            name="elbow hub stays between fork cheeks during bend",
        )

    return ctx.report()


object_model = build_object_model()
