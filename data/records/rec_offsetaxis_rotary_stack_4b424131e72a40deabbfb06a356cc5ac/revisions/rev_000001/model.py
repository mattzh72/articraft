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
    model = ArticulatedObject(name="tower_offset_dual_rotary_stack")

    dark_cast = Material("dark_cast_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_blue = Material("satin_blue_anodized", rgba=(0.08, 0.18, 0.33, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    bearing_black = Material("black_bearing_surfaces", rgba=(0.015, 0.016, 0.018, 1.0))
    safety_orange = Material("orange_index_handle", rgba=(0.95, 0.38, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.36, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_cast,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=bearing_black,
        name="lower_bearing_race",
    )
    base.visual(
        Cylinder(radius=0.13, length=0.047),
        origin=Origin(xyz=(0.0, 0.0, 0.0665)),
        material=brushed_steel,
        name="center_pedestal",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.305, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=satin_blue,
        name="turntable_disk",
    )
    lower_stage.visual(
        Cylinder(radius=0.325, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=satin_blue,
        name="outer_rim",
    )
    lower_stage.visual(
        Cylinder(radius=0.095, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=brushed_steel,
        name="lower_rotor_hub",
    )
    lower_stage.visual(
        Box((0.145, 0.190, 0.038)),
        origin=Origin(xyz=(0.215, 0.0, 0.063)),
        material=brushed_steel,
        name="cheek_foot",
    )
    lower_stage.visual(
        Box((0.085, 0.130, 0.300)),
        origin=Origin(xyz=(0.215, 0.0, 0.200)),
        material=brushed_steel,
        name="side_cheek",
    )
    lower_stage.visual(
        Box((0.055, 0.026, 0.255)),
        origin=Origin(xyz=(0.170, 0.075, 0.180), rpy=(0.0, math.radians(-13.0), 0.0)),
        material=brushed_steel,
        name="gusset_0",
    )
    lower_stage.visual(
        Box((0.055, 0.026, 0.255)),
        origin=Origin(xyz=(0.170, -0.075, 0.180), rpy=(0.0, math.radians(-13.0), 0.0)),
        material=brushed_steel,
        name="gusset_1",
    )
    lower_stage.visual(
        Cylinder(radius=0.072, length=0.060),
        origin=Origin(xyz=(0.220, 0.0, 0.340)),
        material=bearing_black,
        name="upper_bearing_cap",
    )

    upper_head = model.part("upper_head")
    upper_head.visual(
        Cylinder(radius=0.135, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=satin_blue,
        name="head_disk",
    )
    upper_head.visual(
        Cylinder(radius=0.055, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=brushed_steel,
        name="head_hub",
    )
    upper_head.visual(
        Box((0.150, 0.035, 0.026)),
        origin=Origin(xyz=(0.135, 0.0, 0.058)),
        material=safety_orange,
        name="pointer_tab",
    )
    upper_head.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(0.200, 0.0, 0.087)),
        material=safety_orange,
        name="grip_post",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "lower_stage_to_upper_head",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=upper_head,
        origin=Origin(xyz=(0.220, 0.0, 0.370)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.4, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_head = object_model.get_part("upper_head")
    lower_joint = object_model.get_articulation("base_to_lower_stage")
    upper_joint = object_model.get_articulation("lower_stage_to_upper_head")

    ctx.check(
        "two independent revolute joints",
        lower_joint.articulation_type == ArticulationType.REVOLUTE
        and upper_joint.articulation_type == ArticulationType.REVOLUTE
        and lower_joint.mimic is None
        and upper_joint.mimic is None,
        details=f"lower={lower_joint.articulation_type}, upper={upper_joint.articulation_type}",
    )
    ctx.check(
        "both rotary axes are vertical",
        tuple(lower_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"lower_axis={lower_joint.axis}, upper_axis={upper_joint.axis}",
    )

    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="lower_bearing_race",
        max_gap=0.001,
        max_penetration=0.00001,
        name="lower turntable rests on base bearing",
    )
    ctx.expect_overlap(
        lower_stage,
        base,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="lower_bearing_race",
        min_overlap=0.30,
        name="lower turntable is centered on supported bearing",
    )
    ctx.expect_gap(
        upper_head,
        lower_stage,
        axis="z",
        positive_elem="head_disk",
        negative_elem="upper_bearing_cap",
        max_gap=0.001,
        max_penetration=0.00001,
        name="upper head rests on cheek bearing",
    )
    ctx.expect_overlap(
        upper_head,
        lower_stage,
        axes="xy",
        elem_a="head_disk",
        elem_b="upper_bearing_cap",
        min_overlap=0.13,
        name="upper head is centered on offset cheek bearing",
    )
    ctx.expect_origin_distance(
        upper_head,
        lower_stage,
        axes="xy",
        min_dist=0.20,
        max_dist=0.24,
        name="upper axis is displaced from lower axis",
    )

    rest_upper_pos = ctx.part_world_position(upper_head)
    with ctx.pose({lower_joint: math.pi / 2.0}):
        swept_upper_pos = ctx.part_world_position(upper_head)
    ctx.check(
        "lower stage carries the offset tower around the base axis",
        rest_upper_pos is not None
        and swept_upper_pos is not None
        and rest_upper_pos[0] > 0.20
        and abs(rest_upper_pos[1]) < 0.01
        and abs(swept_upper_pos[0]) < 0.02
        and swept_upper_pos[1] > 0.20,
        details=f"rest={rest_upper_pos}, swept={swept_upper_pos}",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        low, high = aabb
        return (
            (low[0] + high[0]) / 2.0,
            (low[1] + high[1]) / 2.0,
            (low[2] + high[2]) / 2.0,
        )

    with ctx.pose({upper_joint: 0.0}):
        pointer_rest = _aabb_center(ctx.part_element_world_aabb(upper_head, elem="pointer_tab"))
    with ctx.pose({upper_joint: math.pi / 2.0}):
        pointer_rotated = _aabb_center(ctx.part_element_world_aabb(upper_head, elem="pointer_tab"))
    ctx.check(
        "upper head rotates independently at the offset axis",
        rest_upper_pos is not None
        and pointer_rest is not None
        and pointer_rotated is not None
        and pointer_rest[0] > rest_upper_pos[0] + 0.10
        and abs(pointer_rest[1] - rest_upper_pos[1]) < 0.03
        and abs(pointer_rotated[0] - rest_upper_pos[0]) < 0.03
        and pointer_rotated[1] > rest_upper_pos[1] + 0.10,
        details=f"axis={rest_upper_pos}, rest_pointer={pointer_rest}, rotated_pointer={pointer_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
