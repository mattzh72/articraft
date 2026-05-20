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
    model = ArticulatedObject(name="wafer_handling_gripper")

    dark_anodized = Material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    black_hardcoat = Material("black_hardcoat_carriage", rgba=(0.015, 0.017, 0.020, 1.0))
    satin_steel = Material("satin_ground_steel", rgba=(0.66, 0.68, 0.67, 1.0))
    ceramic = Material("white_ceramic_contact", rgba=(0.86, 0.88, 0.82, 1.0))
    rubber = Material("matte_black_bumper", rgba=(0.005, 0.005, 0.006, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.30, 0.118, 0.080)),
        origin=Origin(xyz=(0.0, -0.125, 0.040)),
        material=dark_anodized,
        name="dense_body",
    )
    body.visual(
        Box((0.33, 0.026, 0.070)),
        origin=Origin(xyz=(0.0, -0.086, 0.043)),
        material=dark_anodized,
        name="guideway_backbone",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.46),
        origin=Origin(xyz=(0.0, -0.062, 0.060), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="upper_guideway",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.46),
        origin=Origin(xyz=(0.0, -0.062, 0.027), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="lower_guideway",
    )
    for x, name in ((-0.225, "guide_end_0"), (0.225, "guide_end_1")):
        body.visual(
            Box((0.018, 0.032, 0.084)),
            origin=Origin(xyz=(x, -0.062, 0.043)),
            material=dark_anodized,
            name=name,
        )
    for x, name in ((-0.095, "top_screw_0"), (0.095, "top_screw_1")):
        body.visual(
            Cylinder(radius=0.008, length=0.005),
            origin=Origin(xyz=(x, -0.125, 0.0815)),
            material=rubber,
            name=name,
        )

    def add_jaw(part_name: str, inward: float) -> None:
        jaw = model.part(part_name)
        jaw.visual(
            Box((0.050, 0.018, 0.074)),
            origin=Origin(xyz=(0.0, -0.040, 0.045)),
            material=black_hardcoat,
            name="slide_plate",
        )
        for z, tag in (
            (0.069, "upper_top_bearing"),
            (0.051, "upper_bottom_bearing"),
            (0.036, "lower_top_bearing"),
            (0.018, "lower_bottom_bearing"),
        ):
            jaw.visual(
                Box((0.056, 0.025, 0.006)),
                origin=Origin(xyz=(0.0, -0.0495, z)),
                material=black_hardcoat,
                name=tag,
            )
        jaw.visual(
            Box((0.020, 0.052, 0.060)),
            origin=Origin(xyz=(inward * 0.020, -0.017, 0.050)),
            material=black_hardcoat,
            name="finger_root",
        )
        jaw.visual(
            Box((0.012, 0.292, 0.086)),
            origin=Origin(xyz=(inward * 0.030, 0.105, 0.052)),
            material=black_hardcoat,
            name="fork_blade",
        )
        jaw.visual(
            Box((0.010, 0.058, 0.072)),
            origin=Origin(xyz=(inward * 0.030, 0.233, 0.052)),
            material=ceramic,
            name="fork_tip_pad",
        )
        jaw.visual(
            Box((0.006, 0.036, 0.050)),
            origin=Origin(xyz=(inward * 0.039, 0.230, 0.052)),
            material=ceramic,
            name="inner_contact_face",
        )

    add_jaw("left_jaw", inward=1.0)
    add_jaw("right_jaw", inward=-1.0)

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child="left_jaw",
        origin=Origin(xyz=(-0.130, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.055),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child="right_jaw",
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.055),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left = object_model.get_part("left_jaw")
    right = object_model.get_part("right_jaw")
    body = object_model.get_part("body")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

    ctx.check(
        "both jaws use independent prismatic slides",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and right_slide.articulation_type == ArticulationType.PRISMATIC
        and left_slide.child == "left_jaw"
        and right_slide.child == "right_jaw",
        details=f"left={left_slide.articulation_type}, right={right_slide.articulation_type}",
    )
    ctx.expect_origin_gap(
        right,
        left,
        axis="x",
        min_gap=0.24,
        max_gap=0.28,
        name="open jaw origins straddle pickup center",
    )
    ctx.expect_overlap(
        left,
        body,
        axes="x",
        elem_a="slide_plate",
        elem_b="upper_guideway",
        min_overlap=0.040,
        name="left carriage remains on guideway",
    )
    ctx.expect_overlap(
        right,
        body,
        axes="x",
        elem_a="slide_plate",
        elem_b="upper_guideway",
        min_overlap=0.040,
        name="right carriage remains on guideway",
    )

    left_open = ctx.part_world_position(left)
    right_open = ctx.part_world_position(right)
    with ctx.pose({left_slide: 0.055, right_slide: 0.055}):
        left_closed = ctx.part_world_position(left)
        right_closed = ctx.part_world_position(right)
        ctx.expect_origin_gap(
            right,
            left,
            axis="x",
            min_gap=0.12,
            max_gap=0.16,
            name="closed jaws still leave wafer pickup gap",
        )
        ctx.expect_overlap(
            left,
            body,
            axes="x",
            elem_a="slide_plate",
            elem_b="upper_guideway",
            min_overlap=0.040,
            name="left carriage retained when closed",
        )
        ctx.expect_overlap(
            right,
            body,
            axes="x",
            elem_a="slide_plate",
            elem_b="upper_guideway",
            min_overlap=0.040,
            name="right carriage retained when closed",
        )

    ctx.check(
        "jaws close symmetrically toward center",
        left_open is not None
        and right_open is not None
        and left_closed is not None
        and right_closed is not None
        and left_closed[0] > left_open[0] + 0.050
        and right_closed[0] < right_open[0] - 0.050,
        details=f"left_open={left_open}, left_closed={left_closed}, right_open={right_open}, right_closed={right_closed}",
    )

    return ctx.report()


object_model = build_object_model()
