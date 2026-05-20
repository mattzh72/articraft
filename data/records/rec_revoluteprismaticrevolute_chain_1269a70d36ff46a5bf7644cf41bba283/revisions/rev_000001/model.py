from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_PLATE_LENGTH = 0.22
BASE_PLATE_WIDTH = 0.12
BASE_PLATE_THICKNESS = 0.014
HINGE_AXIS_Z = 0.072

SUPPORT_TUBE_LENGTH = 0.16
SUPPORT_TUBE_WIDTH = 0.050

SLIDER_TRAVEL = 0.090
SLIDER_BAR_LENGTH = 0.184
SLIDER_BAR_WIDTH = 0.028
SLIDER_BAR_HEIGHT = 0.018
TIP_HINGE_X = 0.202
def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinge_slide_hinge_chain")

    model.material("painted_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("anodized_dark", rgba=(0.19, 0.21, 0.24, 1.0))

    base = model.part("base_support")
    base.visual(
        Box((BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_PLATE_THICKNESS)),
        material="painted_steel",
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS / 2.0)),
        name="base_plate",
    )
    base.visual(
        Box((0.072, 0.044, 0.045)),
        material="painted_steel",
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS + 0.0225)),
        name="base_pedestal",
    )
    base.visual(
        Box((0.018, 0.010, 0.040)),
        material="painted_steel",
        origin=Origin(xyz=(0.0, 0.023, HINGE_AXIS_Z)),
        name="left_base_ear",
    )
    base.visual(
        Box((0.018, 0.010, 0.040)),
        material="painted_steel",
        origin=Origin(xyz=(0.0, -0.023, HINGE_AXIS_Z)),
        name="right_base_ear",
    )

    support = model.part("rotating_support")
    support.visual(
        Cylinder(radius=0.012, length=0.036),
        material="painted_steel",
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="hinge_barrel",
    )
    support.visual(
        Box((0.050, 0.034, 0.020)),
        material="painted_steel",
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        name="support_neck",
    )
    support.visual(
        Box((0.160, SUPPORT_TUBE_WIDTH, 0.006)),
        material="painted_steel",
        origin=Origin(xyz=(0.120, 0.0, -0.012)),
        name="support_floor",
    )
    support.visual(
        Box((0.160, 0.008, 0.024)),
        material="painted_steel",
        origin=Origin(xyz=(0.120, 0.021, 0.003)),
        name="left_support_wall",
    )
    support.visual(
        Box((0.160, 0.008, 0.024)),
        material="painted_steel",
        origin=Origin(xyz=(0.120, -0.021, 0.003)),
        name="right_support_wall",
    )

    slider = model.part("sliding_member")
    slider.visual(
        Box((SLIDER_BAR_LENGTH, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT)),
        material="machined_aluminum",
        origin=Origin(xyz=(SLIDER_BAR_LENGTH / 2.0, 0.0, 0.0)),
        name="slider_body",
    )
    slider.visual(
        Box((0.018, 0.006, 0.024)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.193, 0.015, 0.0)),
        name="left_slider_ear",
    )
    slider.visual(
        Box((0.018, 0.006, 0.024)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.193, -0.015, 0.0)),
        name="right_slider_ear",
    )

    tip = model.part("tip_fork")
    tip.visual(
        Cylinder(radius=0.010, length=0.024),
        material="anodized_dark",
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="fork_hub",
    )
    tip.visual(
        Box((0.050, 0.006, 0.012)),
        material="anodized_dark",
        origin=Origin(xyz=(0.027, 0.008, 0.0)),
        name="fork_body",
    )
    tip.visual(
        Box((0.050, 0.006, 0.012)),
        material="anodized_dark",
        origin=Origin(xyz=(0.027, -0.008, 0.0)),
        name="right_fork_tine",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "support_slide",
        ArticulationType.PRISMATIC,
        parent=support,
        child=slider,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=SLIDER_TRAVEL),
    )
    model.articulation(
        "tip_hinge",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=tip,
        origin=Origin(xyz=(TIP_HINGE_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-0.4, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("rotating_support")
    slider = object_model.get_part("sliding_member")
    tip = object_model.get_part("tip_fork")

    base_hinge = object_model.get_articulation("base_hinge")
    support_slide = object_model.get_articulation("support_slide")
    tip_hinge = object_model.get_articulation("tip_hinge")

    ctx.check(
        "chain uses revolute-prismatic-revolute topology",
        (
            base_hinge.articulation_type == ArticulationType.REVOLUTE
            and support_slide.articulation_type == ArticulationType.PRISMATIC
            and tip_hinge.articulation_type == ArticulationType.REVOLUTE
        ),
        details=(
            f"base={base_hinge.articulation_type}, "
            f"middle={support_slide.articulation_type}, "
            f"tip={tip_hinge.articulation_type}"
        ),
    )

    ctx.expect_within(
        slider,
        support,
        axes="yz",
        margin=0.003,
        name="slider stays centered inside the support tube at rest",
    )
    ctx.expect_overlap(
        slider,
        support,
        axes="x",
        min_overlap=0.150,
        name="slider remains substantially inserted at rest",
    )

    rest_support_aabb = ctx.part_world_aabb(support)
    with ctx.pose({base_hinge: 0.75}):
        lifted_support_aabb = ctx.part_world_aabb(support)

    ctx.check(
        "root hinge lifts the support upward",
        (
            rest_support_aabb is not None
            and lifted_support_aabb is not None
            and lifted_support_aabb[1][2] > rest_support_aabb[1][2] + 0.050
        ),
        details=f"rest={rest_support_aabb}, lifted={lifted_support_aabb}",
    )

    with ctx.pose({base_hinge: 0.35, support_slide: 0.0}):
        rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({base_hinge: 0.35, support_slide: SLIDER_TRAVEL}):
        extended_slider_pos = ctx.part_world_position(slider)

    ctx.check(
        "prismatic stage extends outward from the support",
        (
            rest_slider_pos is not None
            and extended_slider_pos is not None
            and extended_slider_pos[0] > rest_slider_pos[0] + 0.070
            and extended_slider_pos[2] > rest_slider_pos[2] + 0.020
        ),
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    with ctx.pose({support_slide: SLIDER_TRAVEL}):
        ctx.expect_within(
            slider,
            support,
            axes="yz",
            margin=0.003,
            name="slider stays centered inside the support tube when extended",
        )
        ctx.expect_overlap(
            slider,
            support,
            axes="x",
            min_overlap=0.055,
            name="slider still retains insertion at full extension",
        )

    with ctx.pose({base_hinge: 0.35, support_slide: 0.050, tip_hinge: 0.0}):
        straight_tip_center = _aabb_center(ctx.part_element_world_aabb(tip, elem="fork_body"))
    with ctx.pose({base_hinge: 0.35, support_slide: 0.050, tip_hinge: 0.75}):
        folded_tip_center = _aabb_center(ctx.part_element_world_aabb(tip, elem="fork_body"))

    ctx.check(
        "tip hinge rotates the fork upward",
        (
            straight_tip_center is not None
            and folded_tip_center is not None
            and folded_tip_center[2] > straight_tip_center[2] + 0.010
        ),
        details=f"straight={straight_tip_center}, folded={folded_tip_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
