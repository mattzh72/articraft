from __future__ import annotations

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
    model = ArticulatedObject(name="under_slung_two_link_arm")

    painted = model.material("painted_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    link_blue = model.material("blue_powdercoat", rgba=(0.08, 0.28, 0.56, 1.0))
    machined = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("black_rubber", rgba=(0.03, 0.03, 0.025, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((0.64, 0.22, 0.075)),
        origin=Origin(xyz=(0.12, 0.0, 0.105)),
        material=painted,
        name="mount_plate",
    )
    top_support.visual(
        Box((0.12, 0.13, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=painted,
        name="clevis_bridge",
    )
    for y, name in ((0.055, "clevis_cheek_0"), (-0.055, "clevis_cheek_1")):
        top_support.visual(
            Box((0.12, 0.018, 0.13)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=painted,
            name=name,
        )
    for y, name in ((0.067, "pin_cap_0"), (-0.067, "pin_cap_1")):
        top_support.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=name,
        )
    for x in (-0.13, 0.31):
        for y in (-0.07, 0.07):
            top_support.visual(
                Cylinder(radius=0.014, length=0.016),
                origin=Origin(xyz=(x, y, 0.150)),
                material=machined,
                name=f"mount_bolt_{x:+.2f}_{y:+.2f}",
            )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.040, length=0.092),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="shoulder_boss",
    )
    upper_link.visual(
        Box((0.045, 0.040, 0.410)),
        origin=Origin(xyz=(0.0, 0.0, -0.245)),
        material=link_blue,
        name="hanging_web",
    )
    upper_link.visual(
        Box((0.055, 0.105, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.452)),
        material=link_blue,
        name="elbow_bridge",
    )
    for y, name in ((0.050, "elbow_cheek_0"), (-0.050, "elbow_cheek_1")):
        upper_link.visual(
            Box((0.070, 0.018, 0.120)),
            origin=Origin(xyz=(0.0, y, -0.520)),
            material=link_blue,
            name=name,
        )
        upper_link.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.0, y * 1.24, -0.520), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"elbow_pin_cap_{0 if y > 0 else 1}",
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.035, length=0.082),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="elbow_boss",
    )
    forearm.visual(
        Box((0.320, 0.045, 0.045)),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        material=link_blue,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.135, 0.080, 0.012)),
        origin=Origin(xyz=(0.3975, 0.0, 0.0285)),
        material=painted,
        name="transition_top",
    )
    forearm.visual(
        Box((0.135, 0.080, 0.012)),
        origin=Origin(xyz=(0.3975, 0.0, -0.0285)),
        material=painted,
        name="transition_bottom",
    )
    forearm.visual(
        Box((0.220, 0.080, 0.012)),
        origin=Origin(xyz=(0.570, 0.0, 0.031)),
        material=painted,
        name="sleeve_top",
    )
    forearm.visual(
        Box((0.220, 0.080, 0.012)),
        origin=Origin(xyz=(0.570, 0.0, -0.031)),
        material=painted,
        name="sleeve_bottom",
    )
    for y, name in ((0.046, "sleeve_side_0"), (-0.046, "sleeve_side_1")):
        forearm.visual(
            Box((0.220, 0.012, 0.050)),
            origin=Origin(xyz=(0.570, y, 0.0)),
            material=painted,
            name=name,
        )
    slider_tip = model.part("slider_tip")
    slider_tip.visual(
        Box((0.420, 0.045, 0.030)),
        origin=Origin(xyz=(-0.130, 0.0, 0.0)),
        material=machined,
        name="slider_member",
    )
    for y, name in ((0.03125, "guide_shoe_0"), (-0.03125, "guide_shoe_1")):
        slider_tip.visual(
            Box((0.420, 0.0175, 0.020)),
            origin=Origin(xyz=(-0.130, y, 0.0)),
            material=dark,
            name=name,
        )
    slider_tip.visual(
        Box((0.050, 0.060, 0.040)),
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
        material=machined,
        name="tip_carriage",
    )
    slider_tip.visual(
        Box((0.026, 0.070, 0.050)),
        origin=Origin(xyz=(0.133, 0.0, 0.0)),
        material=dark,
        name="contact_pad",
    )

    model.articulation(
        "support_to_upper",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=upper_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.4, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.7, lower=-1.10, upper=1.35),
    )
    model.articulation(
        "forearm_to_slider",
        ArticulationType.PRISMATIC,
        parent=forearm,
        child=slider_tip,
        origin=Origin(xyz=(0.680, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    slider = object_model.get_part("slider_tip")
    support_joint = object_model.get_articulation("support_to_upper")
    elbow_joint = object_model.get_articulation("upper_to_forearm")
    slider_joint = object_model.get_articulation("forearm_to_slider")

    ctx.expect_within(
        slider,
        forearm,
        axes="yz",
        inner_elem="slider_member",
        margin=0.0,
        name="slider member fits within sleeve side clearance",
    )
    ctx.expect_overlap(
        slider,
        forearm,
        axes="x",
        elem_a="slider_member",
        elem_b="sleeve_top",
        min_overlap=0.200,
        name="collapsed slider remains deeply inserted",
    )

    rest_tip = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: 0.220}):
        ctx.expect_within(
            slider,
            forearm,
            axes="yz",
            inner_elem="slider_member",
            margin=0.0,
            name="extended slider stays guided in sleeve",
        )
        ctx.expect_overlap(
            slider,
            forearm,
            axes="x",
            elem_a="slider_member",
            elem_b="sleeve_top",
            min_overlap=0.090,
            name="extended slider retains insertion",
        )
        extended_tip = ctx.part_world_position(slider)
    ctx.check(
        "terminal stage extends along forearm",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.20,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    rest_upper = ctx.part_world_aabb(upper)
    with ctx.pose({support_joint: 0.45}):
        swung_upper = ctx.part_world_aabb(upper)
    ctx.check(
        "shoulder revolute joint swings hanging link",
        rest_upper is not None
        and swung_upper is not None
        and swung_upper[0][0] < rest_upper[0][0] - 0.15,
        details=f"rest_aabb={rest_upper}, swung_aabb={swung_upper}",
    )

    rest_forearm = ctx.part_world_aabb(forearm)
    with ctx.pose({elbow_joint: 0.80}):
        bent_forearm = ctx.part_world_aabb(forearm)
    ctx.check(
        "elbow revolute joint bends forearm downward",
        rest_forearm is not None
        and bent_forearm is not None
        and bent_forearm[0][2] < rest_forearm[0][2] - 0.20,
        details=f"rest_aabb={rest_forearm}, bent_aabb={bent_forearm}",
    )

    return ctx.report()


object_model = build_object_model()
