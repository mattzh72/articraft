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
    model = ArticulatedObject(name="robotic_arm")

    # Base bracket
    base = model.part("base")
    base.visual(
        Box((0.15, 0.15, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        name="base_plate",
    )
    # Prongs supporting the shoulder
    base.visual(
        Box((0.08, 0.02, 0.10)),
        origin=Origin(xyz=(0.0, 0.05, 0.06)),
        name="base_prong_left",
    )
    base.visual(
        Box((0.08, 0.02, 0.10)),
        origin=Origin(xyz=(0.0, -0.05, 0.06)),
        name="base_prong_right",
    )

    # Proximal link
    upper_arm = model.part("upper_arm")
    # Main body of upper arm
    upper_arm.visual(
        Box((0.35, 0.06, 0.04)),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        name="upper_arm_body",
    )
    # Shoulder hub
    upper_arm.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0, 0)),
        name="shoulder_hub",
    )
    # Elbow prongs
    upper_arm.visual(
        Box((0.08, 0.02, 0.04)),
        origin=Origin(xyz=(0.39, 0.04, 0.0)),
        name="elbow_prong_left",
    )
    upper_arm.visual(
        Box((0.08, 0.02, 0.04)),
        origin=Origin(xyz=(0.39, -0.04, 0.0)),
        name="elbow_prong_right",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi/4, upper=math.pi/2),
    )

    # Distal link
    forearm = model.part("forearm")
    # Main body
    forearm.visual(
        Box((0.30, 0.05, 0.04)),
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        name="forearm_body",
    )
    # Elbow hub
    forearm.visual(
        Cylinder(radius=0.03, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi/2, 0, 0)),
        name="elbow_hub",
    )
    # Rail for the slider
    forearm.visual(
        Box((0.20, 0.02, 0.02)),
        origin=Origin(xyz=(0.20, 0.0, 0.03)),
        name="forearm_rail",
    )

    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.4, 0.0, 0.0)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-math.pi/2, upper=math.pi/2),
    )

    # Tip slider
    tip_slider = model.part("tip_slider")
    # Slider body wrapping around the rail
    tip_slider.visual(
        Box((0.06, 0.04, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="slider_top",
    )
    # Side guides to grip the rail
    tip_slider.visual(
        Box((0.06, 0.01, 0.02)),
        origin=Origin(xyz=(0.0, 0.015, -0.01)),
        name="slider_guide_left",
    )
    tip_slider.visual(
        Box((0.06, 0.01, 0.02)),
        origin=Origin(xyz=(0.0, -0.015, -0.01)),
        name="slider_guide_right",
    )

    model.articulation(
        "tip_extension",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=tip_slider,
        # Start the slider on the rail at X=0.12 in forearm space
        origin=Origin(xyz=(0.12, 0.0, 0.045)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.5, lower=0.0, upper=0.15),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "base", "upper_arm",
        elem_a="base_prong_left", elem_b="shoulder_hub",
        reason="Shoulder hub is captured within the base left prong."
    )
    ctx.allow_overlap(
        "base", "upper_arm",
        elem_a="base_prong_right", elem_b="shoulder_hub",
        reason="Shoulder hub is captured within the base right prong."
    )

    ctx.allow_overlap(
        "upper_arm", "forearm",
        elem_a="elbow_prong_left", elem_b="elbow_hub",
        reason="Elbow hub is captured within the upper arm left prong."
    )
    ctx.allow_overlap(
        "upper_arm", "forearm",
        elem_a="elbow_prong_right", elem_b="elbow_hub",
        reason="Elbow hub is captured within the upper arm right prong."
    )

    ctx.allow_overlap(
        "forearm", "tip_slider",
        elem_a="forearm_rail", elem_b="slider_guide_left",
        reason="Slider guides overlap rail for retention."
    )
    ctx.allow_overlap(
        "forearm", "tip_slider",
        elem_a="forearm_rail", elem_b="slider_guide_right",
        reason="Slider guides overlap rail for retention."
    )
    ctx.allow_overlap(
        "forearm", "tip_slider",
        elem_a="forearm_rail", elem_b="slider_top",
        reason="Slider top rides on the rail."
    )

    # Prove the slider stays on the rail
    slider = object_model.get_part("tip_slider")
    forearm = object_model.get_part("forearm")
    tip_ext = object_model.get_articulation("tip_extension")

    ctx.expect_within(
        slider, forearm,
        axes="y",
        margin=0.02,
        name="slider stays centered on rail"
    )
    ctx.expect_overlap(
        slider, forearm,
        axes="x",
        elem_a="slider_top",
        elem_b="forearm_rail",
        min_overlap=0.02,
        name="slider overlaps rail at rest"
    )

    with ctx.pose({tip_ext: 0.15}):
        ctx.expect_within(
            slider, forearm,
            axes="y",
            margin=0.02,
            name="extended slider stays centered on rail"
        )
        ctx.expect_overlap(
            slider, forearm,
            axes="x",
            elem_a="slider_top",
            elem_b="forearm_rail",
            min_overlap=0.01,
            name="extended slider retains insertion"
        )

    return ctx.report()

object_model = build_object_model()
