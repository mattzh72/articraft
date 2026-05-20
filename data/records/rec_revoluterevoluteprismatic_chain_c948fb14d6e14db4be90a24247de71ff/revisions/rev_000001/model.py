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
    model = ArticulatedObject(name="shoulder_elbow_extension_chain")

    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    blue_anodized = model.material("blue_anodized", rgba=(0.05, 0.24, 0.72, 1.0))
    orange_anodized = model.material("orange_anodized", rgba=(0.92, 0.42, 0.08, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.03, 0.03, 0.035, 1.0))

    # Root part: a grounded foot, short pedestal, and an open shoulder yoke.
    base = model.part("base")
    base.visual(
        Box((0.34, 0.30, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_steel,
        name="ground_foot",
    )
    base.visual(
        Box((0.12, 0.17, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=dark_steel,
        name="pedestal",
    )
    for side, y in enumerate((-0.085, 0.085)):
        base.visual(
            Box((0.14, 0.035, 0.16)),
            origin=Origin(xyz=(0.0, y, 0.280)),
            material=dark_steel,
            name=f"shoulder_cheek_{side}",
        )
        base.visual(
            Cylinder(radius=0.043, length=0.014),
            origin=Origin(xyz=(0.0, y * 1.285, 0.280), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"shoulder_bearing_{side}",
        )

    # First hinged member.  Its part frame is exactly on the shoulder pin axis.
    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.045, length=0.135),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.51, 0.052, 0.045)),
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        material=blue_anodized,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.040, 0.135, 0.052)),
        origin=Origin(xyz=(0.555, 0.0, 0.0)),
        material=blue_anodized,
        name="elbow_bridge",
    )
    for side, y in enumerate((-0.060, 0.060)):
        upper_arm.visual(
            Box((0.120, 0.025, 0.105)),
            origin=Origin(xyz=(0.620, y, 0.0)),
            material=blue_anodized,
            name=f"elbow_cheek_{side}",
        )
        upper_arm.visual(
            Cylinder(radius=0.034, length=0.012),
            origin=Origin(xyz=(0.620, y * 1.145, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"elbow_bearing_{side}",
        )

    # Second hinged member.  Its frame is on the elbow pin axis.
    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.041, length=0.095),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.360, 0.045, 0.040)),
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        material=orange_anodized,
        name="forearm_beam",
    )
    # A real-looking rectangular guide sleeve at the tip.  It is made from
    # four connected rails so the moving slider has actual clearance through it.
    forearm.visual(
        Box((0.230, 0.086, 0.012)),
        origin=Origin(xyz=(0.500, 0.0, 0.036)),
        material=dark_steel,
        name="sleeve_top",
    )
    forearm.visual(
        Box((0.230, 0.086, 0.012)),
        origin=Origin(xyz=(0.500, 0.0, -0.036)),
        material=dark_steel,
        name="sleeve_bottom",
    )
    for side, y in enumerate((-0.028, 0.028)):
        forearm.visual(
            Box((0.230, 0.012, 0.072)),
            origin=Origin(xyz=(0.500, y, 0.0)),
            material=dark_steel,
            name=f"sleeve_side_{side}",
        )

    # Prismatic tip stage.  Its hidden length stays inside the guide sleeve
    # after extension, while the rubber pad remains outside the sleeve mouth.
    tip_slider = model.part("tip_slider")
    tip_slider.visual(
        Box((0.200, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="slide_bar",
    )
    tip_slider.visual(
        Box((0.030, 0.060, 0.044)),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        material=rubber_black,
        name="tip_pad",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-0.70, upper=1.25),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "tip_slide",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=tip_slider,
        origin=Origin(xyz=(0.515, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    tip_slider = object_model.get_part("tip_slider")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    tip_slide = object_model.get_articulation("tip_slide")

    ctx.check("shoulder is revolute", shoulder.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("elbow is revolute", elbow.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("tip stage is prismatic", tip_slide.articulation_type == ArticulationType.PRISMATIC)

    ctx.expect_within(
        upper_arm,
        base,
        axes="yz",
        inner_elem="shoulder_hub",
        margin=0.002,
        name="shoulder hub sits inside base yoke",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="yz",
        inner_elem="elbow_hub",
        outer_elem="elbow_bridge",
        margin=0.060,
        name="elbow hub is carried by upper yoke",
    )
    ctx.expect_within(
        tip_slider,
        forearm,
        axes="yz",
        inner_elem="slide_bar",
        margin=0.0,
        name="slider bar stays inside guide bore",
    )
    ctx.expect_overlap(
        tip_slider,
        forearm,
        axes="x",
        elem_a="slide_bar",
        elem_b="sleeve_top",
        min_overlap=0.150,
        name="collapsed slider remains engaged in sleeve",
    )

    upper_rest = ctx.part_element_world_aabb(upper_arm, elem="elbow_bridge")
    with ctx.pose({shoulder: 0.9, elbow: 0.6}):
        upper_raised = ctx.part_element_world_aabb(upper_arm, elem="elbow_bridge")
        ctx.check(
            "shoulder lift raises elbow end",
            upper_rest is not None
            and upper_raised is not None
            and upper_raised[0][2] > upper_rest[0][2] + 0.08,
            details=f"rest={upper_rest}, raised={upper_raised}",
        )

    slider_rest = ctx.part_world_position(tip_slider)
    with ctx.pose({tip_slide: 0.100}):
        slider_extended = ctx.part_world_position(tip_slider)
        ctx.expect_overlap(
            tip_slider,
            forearm,
            axes="x",
            elem_a="slide_bar",
            elem_b="sleeve_top",
            min_overlap=0.050,
            name="extended slider retains insertion",
        )
    ctx.check(
        "tip slide extends outward",
        slider_rest is not None
        and slider_extended is not None
        and slider_extended[0] > slider_rest[0] + 0.095,
        details=f"rest={slider_rest}, extended={slider_extended}",
    )

    return ctx.report()


object_model = build_object_model()
