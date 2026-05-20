from __future__ import annotations

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

BASE_STILE_NAMES = ("base_stile_0", "base_stile_1")
GUIDE_WEB_NAMES = ("guide_web_0", "guide_web_1")
GUIDE_LIP_OUTER_NAMES = ("guide_lip_outer_0", "guide_lip_outer_1")
GUIDE_LIP_INNER_NAMES = ("guide_lip_inner_0", "guide_lip_inner_1")
FLY_STILE_NAMES = ("fly_stile_0", "fly_stile_1")
STANDOFF_ARM_NAMES = ("standoff_arm_0", "standoff_arm_1")
STANDOFF_JOINT_NAMES = ("fly_to_standoff_0", "fly_to_standoff_1")
STANDOFF_PIN_NAMES = ("standoff_pin_0", "standoff_pin_1")
RUBBER_FOOT_NAMES = ("rubber_foot_0", "rubber_foot_1")
FOOT_PIN_NAMES = ("foot_pin_0", "foot_pin_1")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_access_extension_ladder")

    aluminium = model.material("brushed_aluminium", rgba=(0.74, 0.77, 0.76, 1.0))
    darker_aluminium = model.material("darker_aluminium", rgba=(0.52, 0.56, 0.56, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    yellow_label = model.material("safety_label_yellow", rgba=(1.0, 0.77, 0.08, 1.0))

    base = model.part("base_section")

    # Ground/base section: two aluminium stile rails bridged by round rungs.
    rail_x = 0.285
    rail_size = (0.055, 0.050, 3.60)
    for index, sx in enumerate((-1.0, 1.0)):
        base.visual(
            Box(rail_size),
            origin=Origin(xyz=(sx * rail_x, 0.0, 1.90)),
            material=aluminium,
            name=BASE_STILE_NAMES[index],
        )

    for index, z in enumerate((0.48, 0.78, 1.08, 1.38, 1.68, 1.98, 2.28, 2.58, 2.88, 3.18)):
        base.visual(
            Cylinder(radius=0.018, length=0.57),
            origin=Origin(xyz=(0.0, -0.018, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminium,
            name=f"base_rung_{index}",
        )

    # Prismatic fly-section channel guides.  Each guide has a back web bolted to
    # the fixed stile and two lips that straddle the sliding fly stile with a
    # visible clearance rather than intersecting it.
    for index, sx in enumerate((-1.0, 1.0)):
        x_center = sx * 0.225
        base.visual(
            Box((0.095, 0.012, 2.42)),
            origin=Origin(xyz=(x_center, 0.081, 2.16)),
            material=darker_aluminium,
            name=GUIDE_WEB_NAMES[index],
        )
        base.visual(
            Box((0.010, 0.050, 2.42)),
            origin=Origin(xyz=(x_center - sx * 0.0275, 0.095, 2.16)),
            material=darker_aluminium,
            name=GUIDE_LIP_OUTER_NAMES[index],
        )
        base.visual(
            Box((0.010, 0.050, 2.42)),
            origin=Origin(xyz=(x_center + sx * 0.0275, 0.095, 2.16)),
            material=darker_aluminium,
            name=GUIDE_LIP_INNER_NAMES[index],
        )
        base.visual(
            Box((0.035, 0.050, 0.080)),
            origin=Origin(xyz=(sx * 0.265, 0.050, 1.16)),
            material=darker_aluminium,
            name=f"guide_spacer_low_{index}",
        )
        base.visual(
            Box((0.035, 0.050, 0.080)),
            origin=Origin(xyz=(sx * 0.265, 0.050, 2.14)),
            material=darker_aluminium,
            name=f"guide_spacer_mid_{index}",
        )
        base.visual(
            Box((0.035, 0.050, 0.080)),
            origin=Origin(xyz=(sx * 0.265, 0.050, 3.12)),
            material=darker_aluminium,
            name=f"guide_spacer_high_{index}",
        )

    # Forks and pins at the foot pivots.
    for index, sx in enumerate((-1.0, 1.0)):
        x = sx * rail_x
        base.visual(
            Box((0.012, 0.070, 0.038)),
            origin=Origin(xyz=(x - 0.050, 0.060, 0.130)),
            material=darker_aluminium,
            name=f"foot_cheek_a_{index}",
        )
        base.visual(
            Box((0.012, 0.070, 0.038)),
            origin=Origin(xyz=(x + 0.050, 0.060, 0.130)),
            material=darker_aluminium,
            name=f"foot_cheek_b_{index}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.125),
            origin=Origin(xyz=(x, 0.085, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_aluminium,
            name=FOOT_PIN_NAMES[index],
        )
        base.visual(
            Box((0.120, 0.050, 0.030)),
            origin=Origin(xyz=(x, 0.035, 0.130)),
            material=darker_aluminium,
            name=f"foot_mount_{index}",
        )

    base.visual(
        Box((0.048, 0.004, 0.10)),
        origin=Origin(xyz=(-rail_x, -0.027, 1.53)),
        material=yellow_label,
        name="load_rating_label",
    )

    fly = model.part("fly_section")
    fly_rail_x = 0.225
    for index, sx in enumerate((-1.0, 1.0)):
        fly.visual(
            Box((0.045, 0.035, 3.32)),
            origin=Origin(xyz=(sx * fly_rail_x, 0.120, 1.66)),
            material=aluminium,
            name=FLY_STILE_NAMES[index],
        )

    for index, z in enumerate((0.20, 0.50, 0.80, 1.10, 1.40, 1.70, 2.00, 2.30, 2.60, 2.90, 3.20)):
        fly.visual(
            Cylinder(radius=0.016, length=0.420),
            origin=Origin(xyz=(0.0, 0.150, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminium,
            name=f"fly_rung_{index}",
        )

    fly.visual(
        Box((0.50, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.140, 3.275)),
        material=darker_aluminium,
        name="top_spreader",
    )

    # Roof stand-off hinges are carried by the fly section so the rubber
    # bumpers remain at the highest stile ends as the ladder extends.
    for index, sx in enumerate((-1.0, 1.0)):
        x = sx * fly_rail_x
        fly.visual(
            Box((0.020, 0.070, 0.060)),
            origin=Origin(xyz=(x - 0.050, 0.155, 3.330)),
            material=darker_aluminium,
            name=f"standoff_cheek_a_{index}",
        )
        fly.visual(
            Box((0.020, 0.070, 0.060)),
            origin=Origin(xyz=(x + 0.050, 0.155, 3.330)),
            material=darker_aluminium,
            name=f"standoff_cheek_b_{index}",
        )
        fly.visual(
            Cylinder(radius=0.006, length=0.125),
            origin=Origin(xyz=(x, 0.155, 3.330), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_aluminium,
            name=STANDOFF_PIN_NAMES[index],
        )
        fly.visual(
            Box((0.120, 0.050, 0.030)),
            origin=Origin(xyz=(x, 0.155, 3.285)),
            material=darker_aluminium,
            name=f"standoff_mount_{index}",
        )

    # The lower end of the fly section starts inside the guide track at z=0.78.
    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=1.40),
    )

    # Stand-off arms pivot from the two top stile ends and carry rubber pads that
    # hold the ladder away from a roof edge or wall.
    for index, sx in enumerate((-1.0, 1.0)):
        arm = model.part(STANDOFF_ARM_NAMES[index])
        arm.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_aluminium,
            name="hinge_sleeve",
        )
        arm.visual(
            Cylinder(radius=0.017, length=0.55),
            origin=Origin(xyz=(0.0, 0.385, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=aluminium,
            name="arm_tube",
        )
        arm.visual(
            Box((0.025, 0.110, 0.012)),
            origin=Origin(xyz=(0.0, 0.055, 0.013)),
            material=darker_aluminium,
            name="arm_socket",
        )
        arm.visual(
            Cylinder(radius=0.011, length=0.40),
            origin=Origin(xyz=(0.0, 0.31, -0.105), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_aluminium,
            name="lower_strut",
        )
        arm.visual(
            Box((0.018, 0.018, 0.110)),
            origin=Origin(xyz=(0.0, 0.18, -0.052)),
            material=darker_aluminium,
            name="strut_post_low",
        )
        arm.visual(
            Box((0.018, 0.018, 0.110)),
            origin=Origin(xyz=(0.0, 0.44, -0.052)),
            material=darker_aluminium,
            name="strut_post_high",
        )
        arm.visual(
            Box((0.160, 0.050, 0.070)),
            origin=Origin(xyz=(0.0, 0.645, 0.0)),
            material=rubber,
            name="rubber_bumper",
        )
        model.articulation(
            STANDOFF_JOINT_NAMES[index],
            ArticulationType.REVOLUTE,
            parent=fly,
            child=arm,
            origin=Origin(xyz=(sx * fly_rail_x, 0.155, 3.330)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-1.10, upper=0.45),
        )

    # Swiveling rubber feet at the base, one on each stile.
    for index, sx in enumerate((-1.0, 1.0)):
        foot = model.part(RUBBER_FOOT_NAMES[index])
        foot.visual(
            Cylinder(radius=0.018, length=0.082),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=darker_aluminium,
            name="pivot_sleeve",
        )
        foot.visual(
            Box((0.170, 0.105, 0.035)),
            origin=Origin(xyz=(0.0, 0.020, -0.095)),
            material=rubber,
            name="rubber_pad",
        )
        foot.visual(
            Box((0.012, 0.040, 0.092)),
            origin=Origin(xyz=(-0.032, 0.010, -0.048)),
            material=darker_aluminium,
            name="side_plate_a",
        )
        foot.visual(
            Box((0.012, 0.040, 0.092)),
            origin=Origin(xyz=(0.032, 0.010, -0.048)),
            material=darker_aluminium,
            name="side_plate_b",
        )
        model.articulation(
            f"base_to_foot_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=foot,
            origin=Origin(xyz=(sx * rail_x, 0.085, 0.130)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.55, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    fly_slide = object_model.get_articulation("base_to_fly")

    ctx.expect_within(
        fly,
        base,
        axes="x",
        inner_elem="fly_stile_0",
        outer_elem="guide_web_0",
        margin=0.010,
        name="fly stile 0 stays in channel width",
    )
    ctx.expect_within(
        fly,
        base,
        axes="x",
        inner_elem="fly_stile_1",
        outer_elem="guide_web_1",
        margin=0.010,
        name="fly stile 1 stays in channel width",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        elem_a="fly_stile_0",
        elem_b="guide_web_0",
        min_overlap=1.20,
        name="collapsed fly retained in guide",
    )

    rest_top = ctx.part_element_world_aabb(fly, elem="top_spreader")
    with ctx.pose({fly_slide: 1.40}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            elem_a="fly_stile_0",
            elem_b="guide_web_0",
            min_overlap=0.70,
            name="extended fly remains captured",
        )
        raised_top = ctx.part_element_world_aabb(fly, elem="top_spreader")
    ctx.check(
        "fly section extends upward",
        rest_top is not None
        and raised_top is not None
        and raised_top[0][2] > rest_top[0][2] + 1.0,
        details=f"rest_top={rest_top}, raised_top={raised_top}",
    )

    for index in (0, 1):
        arm = object_model.get_part(STANDOFF_ARM_NAMES[index])
        foot = object_model.get_part(RUBBER_FOOT_NAMES[index])
        arm_joint = object_model.get_articulation(STANDOFF_JOINT_NAMES[index])
        foot_joint = object_model.get_articulation(f"base_to_foot_{index}")

        ctx.allow_overlap(
            fly,
            arm,
            elem_a=STANDOFF_PIN_NAMES[index],
            elem_b="hinge_sleeve",
            reason="The visible stand-off hinge pin is intentionally captured inside the arm sleeve.",
        )
        ctx.expect_within(
            fly,
            arm,
            axes="yz",
            inner_elem=STANDOFF_PIN_NAMES[index],
            outer_elem="hinge_sleeve",
            margin=0.001,
            name=f"stand-off pin {index} centered in sleeve",
        )

        ctx.allow_overlap(
            base,
            foot,
            elem_a=FOOT_PIN_NAMES[index],
            elem_b="pivot_sleeve",
            reason="The foot pivot pin is intentionally captured inside the rubber-foot sleeve.",
        )
        ctx.expect_within(
            base,
            foot,
            axes="yz",
            inner_elem=FOOT_PIN_NAMES[index],
            outer_elem="pivot_sleeve",
            margin=0.001,
            name=f"foot pin {index} centered in sleeve",
        )

        arm_rest = ctx.part_element_world_aabb(arm, elem="rubber_bumper")
        with ctx.pose({arm_joint: -0.80}):
            arm_folded = ctx.part_element_world_aabb(arm, elem="rubber_bumper")
        ctx.check(
            f"stand-off arm {index} pivots",
            arm_rest is not None
            and arm_folded is not None
            and arm_folded[0][2] < arm_rest[0][2] - 0.20,
            details=f"rest={arm_rest}, folded={arm_folded}",
        )

        foot_rest = ctx.part_element_world_aabb(foot, elem="rubber_pad")
        with ctx.pose({foot_joint: 0.45}):
            foot_swiveled = ctx.part_element_world_aabb(foot, elem="rubber_pad")
        ctx.check(
            f"rubber foot {index} swivels",
            foot_rest is not None
            and foot_swiveled is not None
            and abs(foot_swiveled[0][1] - foot_rest[0][1]) > 0.025,
            details=f"rest={foot_rest}, swiveled={foot_swiveled}",
        )

    return ctx.report()


object_model = build_object_model()
