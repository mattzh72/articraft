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
    model = ArticulatedObject(name="open_frame_revolute_chain")

    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    cast_gray = Material("cast_gray", rgba=(0.38, 0.39, 0.40, 1.0))
    brushed_metal = Material("brushed_metal", rgba=(0.72, 0.74, 0.72, 1.0))
    pin_metal = Material("pin_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    tab_orange = Material("tab_orange", rgba=(0.92, 0.40, 0.12, 1.0))
    black = Material("black", rgba=(0.02, 0.02, 0.02, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.32, 0.26, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="base_plate",
    )
    pedestal.visual(
        Box((0.08, 0.09, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=cast_gray,
        name="upright_post",
    )
    pedestal.visual(
        Box((0.14, 0.26, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=cast_gray,
        name="yoke_bridge",
    )
    pedestal.visual(
        Box((0.14, 0.035, 0.24)),
        origin=Origin(xyz=(0.0, -0.12, 0.52)),
        material=cast_gray,
        name="near_yoke_cheek",
    )
    pedestal.visual(
        Cylinder(radius=0.042, length=0.020),
        origin=Origin(xyz=(0.0, -0.1296, 0.55), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="near_axis_cap",
    )
    pedestal.visual(
        Box((0.14, 0.035, 0.24)),
        origin=Origin(xyz=(0.0, 0.12, 0.52)),
        material=cast_gray,
        name="far_yoke_cheek",
    )
    pedestal.visual(
        Cylinder(radius=0.042, length=0.020),
        origin=Origin(xyz=(0.0, 0.1296, 0.55), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="far_axis_cap",
    )

    link_0 = model.part("link_0")
    link_0.visual(
        Box((0.52, 0.025, 0.038)),
        origin=Origin(xyz=(0.26, -0.075, 0.0)),
        material=brushed_metal,
        name="near_rail",
    )
    link_0.visual(
        Cylinder(radius=0.048, length=0.025),
        origin=Origin(xyz=(0.52, -0.075, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="near_distal_bearing",
    )
    link_0.visual(
        Box((0.52, 0.025, 0.038)),
        origin=Origin(xyz=(0.26, 0.075, 0.0)),
        material=brushed_metal,
        name="far_rail",
    )
    link_0.visual(
        Cylinder(radius=0.048, length=0.025),
        origin=Origin(xyz=(0.52, 0.075, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="far_distal_bearing",
    )
    link_0.visual(
        Cylinder(radius=0.055, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="proximal_hub",
    )
    link_0.visual(
        Cylinder(radius=0.014, length=0.17),
        origin=Origin(xyz=(0.28, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="middle_spacer",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        Box((0.38, 0.020, 0.030)),
        origin=Origin(xyz=(0.19, -0.045, 0.0)),
        material=brushed_metal,
        name="near_rail",
    )
    link_1.visual(
        Box((0.38, 0.020, 0.030)),
        origin=Origin(xyz=(0.19, 0.045, 0.0)),
        material=brushed_metal,
        name="far_rail",
    )
    link_1.visual(
        Cylinder(radius=0.038, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="proximal_hub",
    )
    link_1.visual(
        Cylinder(radius=0.030, length=0.11),
        origin=Origin(xyz=(0.38, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="tip_cross_pin",
    )
    link_1.visual(
        Box((0.09, 0.085, 0.018)),
        origin=Origin(xyz=(0.420, 0.0, 0.0)),
        material=tab_orange,
        name="end_tab",
    )
    link_1.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.430, 0.0, 0.011), rpy=(0.0, 0.0, 0.0)),
        material=black,
        name="tab_bore_mark",
    )

    model.articulation(
        "pedestal_to_link_0",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.52, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.2, lower=-1.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    first_joint = object_model.get_articulation("pedestal_to_link_0")
    second_joint = object_model.get_articulation("link_0_to_link_1")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "exactly two revolute joints",
        len(revolute_joints) == 2,
        details=f"revolute joints={[joint.name for joint in revolute_joints]}",
    )
    ctx.check(
        "joint axes are parallel",
        first_joint.axis == second_joint.axis == (0.0, -1.0, 0.0),
        details=f"axes={first_joint.axis}, {second_joint.axis}",
    )

    ctx.expect_within(
        link_0,
        pedestal,
        axes="y",
        inner_elem="proximal_hub",
        outer_elem="yoke_bridge",
        margin=0.001,
        name="first hub sits between pedestal cheeks",
    )
    ctx.expect_contact(
        pedestal,
        link_0,
        elem_a="near_yoke_cheek",
        elem_b="proximal_hub",
        contact_tol=0.00001,
        name="first axis supported by near cheek",
    )
    ctx.expect_contact(
        pedestal,
        link_0,
        elem_a="far_yoke_cheek",
        elem_b="proximal_hub",
        contact_tol=0.00001,
        name="first axis supported by far cheek",
    )
    ctx.expect_within(
        link_1,
        link_0,
        axes="y",
        inner_elem="proximal_hub",
        outer_elem=None,
        margin=0.001,
        name="second hub sits inside link fork gap",
    )
    ctx.expect_contact(
        link_0,
        link_1,
        elem_a="near_distal_bearing",
        elem_b="proximal_hub",
        contact_tol=0.00001,
        name="second axis supported by near bearing",
    )
    ctx.expect_contact(
        link_0,
        link_1,
        elem_a="far_distal_bearing",
        elem_b="proximal_hub",
        contact_tol=0.00001,
        name="second axis supported by far bearing",
    )
    ctx.expect_origin_gap(
        link_1,
        link_0,
        axis="x",
        min_gap=0.45,
        max_gap=0.60,
        name="second joint is at link zero tip",
    )

    rest_tip = ctx.part_element_world_aabb(link_1, elem="end_tab")
    with ctx.pose({first_joint: 0.65, second_joint: 0.45}):
        raised_tip = ctx.part_element_world_aabb(link_1, elem="end_tab")
    ctx.check(
        "positive joint motion raises the chain",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > rest_tip[0][2] + 0.20,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
